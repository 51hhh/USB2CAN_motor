#!/usr/bin/env python3
"""码盘串口快速测试脚本 — 裸读 MCU 二进制 ODOM_STATE 帧。

用法：
  python3 test_odom_serial.py [串口设备] [秒数]

例：
  python3 test_odom_serial.py /dev/ttyUSB0 5
"""

import serial
import struct
import sys
import time
import math

# ── 帧常量 ──────────────────────────────────────────────
SOF = bytes([0xAA, 0x55])
FRAME_LEN = 45   # 固定帧长 (ODOM_STATE)
MSG_ODOM_STATE = 0x02

# ── CRC-16/CCITT ────────────────────────────────────────
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

# ── 解析 ────────────────────────────────────────────────
def parse_frame(frame: bytes):
    if len(frame) < FRAME_LEN:
        return None
    if frame[0:2] != SOF:
        return None
    ver = frame[2]
    msg_type = frame[3]
    seq = frame[4]
    payload_len = struct.unpack_from("<H", frame, 5)[0]
    if payload_len != 36 or msg_type != MSG_ODOM_STATE:
        return None

    # CRC 校验：覆盖 byte[2] 到 byte[42]（含）= 41 字节
    crc_calc = crc16_ccitt(frame[2:43])
    crc_recv = struct.unpack_from("<H", frame, 43)[0]
    if crc_calc != crc_recv:
        return {"error": f"CRC mismatch: calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}"}

    payload = frame[7:43]
    t_us, x, y, yaw, vx, vy, wz, status, quality, _ = struct.unpack("<QffffffHBB", payload)
    return {
        "seq": seq, "t_us": t_us,
        "x": x, "y": y, "yaw": yaw,
        "vx": vx, "vy": vy, "wz": wz,
        "status": f"0x{status:04X}", "quality": quality,
        "yaw_deg": math.degrees(yaw),
    }

# ── 主逻辑 ──────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    print(f"打开 {port} @ 115200 8N1，抓取 {duration}s ...")
    ser = serial.Serial(port, 115200, bytesize=8, parity="N", stopbits=1, timeout=0.05)

    buf = bytearray()
    ok = 0
    crc_err = 0
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        # 搜索帧头
        while True:
            idx = buf.find(SOF)
            if idx < 0:
                buf.clear()
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < FRAME_LEN:
                break

            result = parse_frame(bytes(buf[:FRAME_LEN]))
            if result is None:
                buf = buf[1:]
                continue
            if "error" in result:
                crc_err += 1
                print(f"  [CRC ERR] {result['error']}")
                buf = buf[1:]
                continue

            ok += 1
            if ok <= 5 or ok % 100 == 0:
                print(f"  [{ok:>5d}] seq={result['seq']:>3d}  "
                      f"x={result['x']:+.4f} y={result['y']:+.4f} "
                      f"yaw={result['yaw_deg']:+.1f}°  "
                      f"vx={result['vx']:+.4f} vy={result['vy']:+.4f} wz={result['wz']:+.4f}  "
                      f"status={result['status']} q={result['quality']}")
            buf = buf[FRAME_LEN:]

    elapsed = time.monotonic() - t0
    ser.close()
    print(f"\n完成: {ok} 帧OK, {crc_err} CRC错误, {elapsed:.1f}s, 约 {ok/elapsed:.1f} Hz")

if __name__ == "__main__":
    main()
