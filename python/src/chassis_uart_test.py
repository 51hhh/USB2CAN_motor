#!/usr/bin/env python3
"""
STM32F407VE 底盘定位系统 - 上位机串口测试工具

通信接口默认: /dev/ttyUSB0

当前 mcu_main.c 默认启用 ODOM 二进制协议:
- MCU -> Host: AA 55 + version + type + seq + len + payload + crc16
- Host -> MCU: SET_LOCAL_ORIGIN(type=0x30) 用于真正回零/重设局部原点

同时保留旧协议兼容:
- 普通命令: 0x0F + 6字节 + 0xAA
- 旧复位命令: 0xBB + 6字节 + 0xCC
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import serial

NORMAL_HEAD = 0x0F
NORMAL_TAIL = 0xAA
RESET_HEAD = 0xBB
RESET_TAIL = 0xCC
DATA_LEN = 6
FRAME_LEN = 8

ODOM_HEADER = b"\xAA\x55"
ODOM_VERSION = 0x01
ODOM_MSG_STATE = 0x02
ODOM_MSG_SET_LOCAL_ORIGIN = 0x30
ODOM_MSG_SET_LOCAL_ORIGIN_ACK = 0x31
ODOM_FRAME_OVERHEAD = 9
ODOM_MAX_PAYLOAD_LEN = 64
RAD_TO_DEG = 57.2957795


@dataclass
class BcPose:
    x: float
    y: float
    yaw: float
    roll: float


@dataclass
class OdomState:
    seq: int
    t_sample_us: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    status_bits: int
    quality: int


@dataclass
class OdomSetOriginAck:
    seq: int
    acked_seq: int
    result_code: int
    event_counter: int


def parse_6bytes(items: Iterable[str]) -> bytes:
    vals = []
    for s in items:
        token = s.strip()
        if not token:
            raise ValueError("存在空字节参数")

        try:
            # 优先支持十进制 / 0x十六进制
            v = int(token, 0)
        except ValueError:
            # 兼容无前缀十六进制，如 B / BB
            if all(c in "0123456789abcdefABCDEF" for c in token) and len(token) <= 2:
                v = int(token, 16)
            else:
                raise ValueError(
                    f"无法解析字节: '{s}'，请使用十进制(如 11) 或十六进制(如 0x0B/0B/B/BB)"
                )

        if not (0 <= v <= 0xFF):
            raise ValueError(f"字节超范围: {s} (应在 0~255)")
        vals.append(v)

    if len(vals) != DATA_LEN:
        raise ValueError(f"必须提供 {DATA_LEN} 个字节，当前 {len(vals)}")
    return bytes(vals)


def build_frame(head: int, payload6: bytes) -> bytes:
    if len(payload6) != DATA_LEN:
        raise ValueError(f"payload 必须是 {DATA_LEN} 字节")
    if head == NORMAL_HEAD:
        return bytes([NORMAL_HEAD]) + payload6 + bytes([NORMAL_TAIL])
    if head == RESET_HEAD:
        return bytes([RESET_HEAD]) + payload6 + bytes([RESET_TAIL])
    raise ValueError(f"未知帧头: 0x{head:02X}")


def _decode_i16_le(lo: int, hi: int) -> int:
    v = (lo & 0xFF) | ((hi & 0xFF) << 8)
    if v & 0x8000:
        v -= 0x10000
    return v


def decode_xy_from_payload(payload6: bytes) -> tuple[int, int]:
    """按固件 DATARELOAD 约定解析 payload 前4字节为 X/Y (int16 little-endian)。"""
    if len(payload6) != DATA_LEN:
        raise ValueError(f"payload 必须是 {DATA_LEN} 字节")
    x = _decode_i16_le(payload6[0], payload6[1])
    y = _decode_i16_le(payload6[2], payload6[3])
    return x, y


def odom_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_odom_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = bytes([
        ODOM_VERSION,
        msg_type & 0xFF,
        seq & 0xFF,
        len(payload) & 0xFF,
        (len(payload) >> 8) & 0xFF,
    ]) + payload
    crc = odom_crc16(body)
    return ODOM_HEADER + body + struct.pack("<H", crc)


def build_set_origin_frame(seq: int, x: float, y: float, yaw: float, flags: int = 0) -> bytes:
    payload = struct.pack("<fffB3x", x, y, yaw, flags & 0xFF)
    return build_odom_frame(ODOM_MSG_SET_LOCAL_ORIGIN, seq, payload)


def parse_bc_line(line: str) -> Optional[BcPose]:
    # 期望: bc X Y yaw roll
    stripped = line.strip()
    parts = stripped.split()
    if len(parts) >= 5 and parts[0].lower() == "bc":
        try:
            return BcPose(
                x=float(parts[1]),
                y=float(parts[2]),
                yaw=float(parts[3]),
                roll=float(parts[4]),
            )
        except ValueError:
            return None

    csv_parts = stripped.split(",")
    if len(csv_parts) < 4:
        return None
    try:
        return BcPose(
            x=float(csv_parts[0]),
            y=float(csv_parts[1]),
            yaw=float(csv_parts[2]),
            roll=float(csv_parts[3]),
        )
    except ValueError:
        return None


def parse_odom_payload(msg_type: int, seq: int, payload: bytes) -> Optional[object]:
    if msg_type == ODOM_MSG_STATE and len(payload) == 36:
        values = struct.unpack("<QffffffHBB", payload)
        return OdomState(
            seq=seq,
            t_sample_us=values[0],
            x=values[1],
            y=values[2],
            yaw=values[3],
            vx=values[4],
            vy=values[5],
            wz=values[6],
            status_bits=values[7],
            quality=values[8],
        )

    if msg_type == ODOM_MSG_SET_LOCAL_ORIGIN_ACK and len(payload) == 8:
        acked_seq, result_code, event_counter = struct.unpack("<HHI", payload)
        return OdomSetOriginAck(
            seq=seq,
            acked_seq=acked_seq,
            result_code=result_code,
            event_counter=event_counter,
        )

    return None


class ChassisTester:
    def __init__(self, port: str, baud: int, timeout: float = 0.02):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._rx_buf = bytearray()
        self._seq = 0

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_normal(self, payload6: bytes) -> bytes:
        frame = build_frame(NORMAL_HEAD, payload6)
        self.ser.write(frame)
        return frame

    def send_reset(self, payload6: bytes) -> bytes:
        frame = build_frame(RESET_HEAD, payload6)
        self.ser.write(frame)
        return frame

    def send_set_origin(self, x: float, y: float, yaw: float, flags: int = 0) -> bytes:
        frame = build_set_origin_frame(self._seq, x, y, yaw, flags)
        self._seq = (self._seq + 1) & 0xFF
        self.ser.write(frame)
        return frame

    def read_events(self, raw: bool = False) -> list[object]:
        events: list[object] = []
        waiting = self.ser.in_waiting
        if waiting <= 0:
            time.sleep(0.001)
            return events

        chunk = self.ser.read(waiting)
        if raw:
            print("RAW:", " ".join(f"{b:02X}" for b in chunk))

        self._rx_buf.extend(chunk)

        while self._rx_buf:
            if self._rx_buf.startswith(ODOM_HEADER):
                if len(self._rx_buf) < ODOM_FRAME_OVERHEAD:
                    break

                version = self._rx_buf[2]
                payload_len = self._rx_buf[5] | (self._rx_buf[6] << 8)
                total_len = ODOM_FRAME_OVERHEAD + payload_len
                if version != ODOM_VERSION or payload_len > ODOM_MAX_PAYLOAD_LEN:
                    del self._rx_buf[0]
                    continue
                if len(self._rx_buf) < total_len:
                    break

                frame = bytes(self._rx_buf[:total_len])
                crc_calc = odom_crc16(frame[2:-2])
                crc_recv = frame[-2] | (frame[-1] << 8)
                if crc_calc != crc_recv:
                    del self._rx_buf[0]
                    continue

                msg_type = frame[3]
                seq = frame[4]
                payload = frame[7:-2]
                event = parse_odom_payload(msg_type, seq, payload)
                if event is not None:
                    events.append(event)
                del self._rx_buf[:total_len]
                continue

            header_pos = self._rx_buf.find(ODOM_HEADER)
            newline_pos = self._rx_buf.find(b"\n")

            if newline_pos >= 0 and (header_pos < 0 or newline_pos < header_pos):
                line = bytes(self._rx_buf[:newline_pos])
                del self._rx_buf[:newline_pos + 1]

                try:
                    text = line.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue

                if not text:
                    continue

                pose = parse_bc_line(text)
                events.append(pose if pose is not None else text)
                continue

            if header_pos > 0:
                del self._rx_buf[:header_pos]
                continue

            if header_pos < 0:
                if len(self._rx_buf) > 512:
                    del self._rx_buf[:-128]
                break

            break

        return events


def main() -> int:
    parser = argparse.ArgumentParser(description="底盘定位串口测试工具")
    parser.add_argument("pos_port", nargs="?", help="兼容旧脚本的位置参数串口，例如 /dev/ttyUSB0")
    parser.add_argument("pos_baud", nargs="?", help="兼容旧脚本的位置参数波特率，例如 115200")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口设备，默认 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--raw", action="store_true", help="打印原始字节")

    parser.add_argument("--send", nargs=6, metavar="BYTE", help="发送普通命令 6 字节")
    parser.add_argument(
        "--reset",
        nargs="*",
        metavar="BYTE",
        help=(
            "发送复位命令；可不带参数(默认 00 00 00 00 00 00)，"
            f"或提供 {DATA_LEN} 字节"
        ),
    )
    parser.add_argument(
        "--reset-zero",
        action="store_true",
        help="按当前 ODOM 二进制协议发送 SET_LOCAL_ORIGIN(0,0,0)，推荐使用",
    )
    parser.add_argument(
        "--set-origin",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW"),
        help="按当前 ODOM 二进制协议设置局部原点",
    )
    parser.add_argument(
        "--reset-repeat",
        type=int,
        default=1,
        help="复位帧重复发送次数（默认 1）",
    )
    parser.add_argument(
        "--reset-interval",
        type=float,
        default=0.05,
        help="复位重复发送间隔秒数（默认 0.05）",
    )

    parser.add_argument("--listen", action="store_true", help="持续监听上行 ODOM/文本数据")
    parser.add_argument("--duration", type=float, default=0.0, help="监听时长(秒)，0 表示无限")
    args = parser.parse_args()

    if args.pos_port:
        args.port = args.pos_port
    if args.pos_baud:
        args.baud = int(args.pos_baud)

    try:
        tester = ChassisTester(port=args.port, baud=args.baud)
        print(f"✅ 串口已打开: {args.port} @ {args.baud}")
    except Exception as e:
        print(f"❌ 打开串口失败: {e}")
        return 1

    try:
        has_send = args.send is not None
        has_legacy_reset = args.reset is not None
        has_odom_origin = args.reset_zero or (args.set_origin is not None)

        if args.reset_repeat < 1:
            raise ValueError("--reset-repeat 不能小于 1")
        if args.reset_interval < 0:
            raise ValueError("--reset-interval 不能小于 0")

        if has_send:
            payload = parse_6bytes(args.send)
            frame = tester.send_normal(payload)
            print("TX normal:", " ".join(f"{b:02X}" for b in frame))

        if has_odom_origin:
            if has_legacy_reset:
                raise ValueError("旧 --reset 与 ODOM --reset-zero/--set-origin 不能同时使用")
            if args.reset_zero and args.set_origin is not None:
                raise ValueError("--reset-zero 与 --set-origin 不能同时使用")

            x, y, yaw = (0.0, 0.0, 0.0) if args.reset_zero else tuple(args.set_origin)
            for i in range(args.reset_repeat):
                frame = tester.send_set_origin(x, y, yaw)
                print(f"TX odom set-origin [{i + 1}/{args.reset_repeat}]:", " ".join(f"{b:02X}" for b in frame))
                if i < args.reset_repeat - 1 and args.reset_interval > 0:
                    time.sleep(args.reset_interval)

        if has_legacy_reset:
            if len(args.reset) == 0:
                payload = bytes([0] * DATA_LEN)
            elif len(args.reset) == DATA_LEN:
                payload = parse_6bytes(args.reset)
            else:
                raise ValueError(f"--reset 需要 0 或 {DATA_LEN} 个字节，当前 {len(args.reset)}")

            if payload != bytes([0] * DATA_LEN):
                x_cmd, y_cmd = decode_xy_from_payload(payload)
                print(
                    f"⚠️ 当前 reset 载荷不会回零，将设置 X={x_cmd}, Y={y_cmd}"
                )

            frame_hex = " ".join(f"{b:02X}" for b in build_frame(RESET_HEAD, payload))
            for i in range(args.reset_repeat):
                tester.send_reset(payload)
                print(f"TX reset [{i + 1}/{args.reset_repeat}]:", frame_hex)
                if i < args.reset_repeat - 1 and args.reset_interval > 0:
                    time.sleep(args.reset_interval)

        need_listen = args.listen or (not has_send and not has_legacy_reset and not has_odom_origin)
        if need_listen:
            print("👂 监听中... 按 Ctrl+C 退出")
            start = time.time()
            frame_count = 0
            while True:
                for event in tester.read_events(raw=args.raw):
                    if isinstance(event, OdomState):
                        frame_count += 1
                        elapsed = time.time() - start
                        hz = frame_count / elapsed if elapsed > 0 else 0.0
                        print(
                            f"[#{event.seq:3d}] "
                            f"t={event.t_sample_us / 1e6:8.3f}s  "
                            f"x={event.x:+8.4f}  y={event.y:+8.4f}  "
                            f"yaw={event.yaw * RAD_TO_DEG:+8.2f}°  "
                            f"vx={event.vx:+6.3f}  vy={event.vy:+6.3f}  "
                            f"wz={event.wz:+6.3f}  "
                            f"status=0x{event.status_bits:04X}  q={event.quality}  "
                            f"({hz:.1f} Hz)"
                        )
                    elif isinstance(event, OdomSetOriginAck):
                        print(
                            f"ACK set-origin -> seq={event.seq}, acked={event.acked_seq}, "
                            f"result={event.result_code}, count={event.event_counter}"
                        )
                    elif isinstance(event, BcPose):
                        print(
                            f"POSE -> X={event.x:.4f}, Y={event.y:.4f}, "
                            f"Yaw={event.yaw:.3f}, Roll={event.roll:.3f}"
                        )
                    else:
                        print(f"RX: {event}")

                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break

        return 0
    except ValueError as e:
        print(f"❌ 参数错误: {e}")
        return 2
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
        return 0
    finally:
        tester.close()


if __name__ == "__main__":
    sys.exit(main())
