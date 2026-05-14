#!/usr/bin/env python3
"""
示教记录脚本 — 重力补偿模式下高频采样记录电机角度

使用方法:
  1. 启动 motor_control_node
  2. 运行本脚本: python3 teach_record.py
  3. 手动拖动机械臂完成运动
  4. Ctrl+C 停止录制，自动保存 JSON 文件

参数:
  --tau-inner   大臂重力矩 Nm (默认 0, 无补偿; 需要补偿时手动指定如 3.1)
  --tau-outer   小臂重力矩 Nm (默认 0, 无补偿; 需要补偿时手动指定如 1.5)
  --kd          转子侧阻尼 (默认 0.005)
  --ramp        斜坡时间 s (默认 1.5)
  --rate        采样频率 Hz (默认 200)
  --output      输出文件名 (默认 teach_data.json, 追加模式, 最多保留10条)
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import json

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}
OUTER_IDS = {1, 3}
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85


class TeachRecorder:
    def __init__(self, args):
        self.tau_inner = args.tau_inner
        self.tau_outer = args.tau_outer
        self.gravity_offset = args.gravity_offset
        self.kd = args.kd
        self.ramp_time = args.ramp
        self.rate_hz = args.rate
        self.output = args.output

        rclpy.init()
        self.node = rclpy.create_node('teach_record')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.velocities = {}
        self.torques = {}
        self.temperatures = {}
        self.errors = {}
        self.last_update = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        self._stop = False
        self.records = []  # [{t, m0, m1, m2, m3, v0, v1, v2, v3}, ...]

        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.torques[msg.motor_id] = msg.torque
        self.temperatures[msg.motor_id] = msg.temperature
        self.errors[msg.motor_id] = msg.error
        self.last_update[msg.motor_id] = time.time()

    def _sig_handler(self, signum, frame):
        print("\n[STOP] 录制结束")
        self._stop = True

    def _send_gravity(self, ramp):
        """发送重力补偿命令（kp=0, 纯力矩模式）"""
        for mid in ALL_IDS:
            pos = self.positions.get(mid, 0.0)
            if mid in INNER_IDS:
                tau = self.tau_inner * math.cos(pos + self.gravity_offset) / GEAR_RATIO * ramp
            else:
                # 小臂: 需要绝对角度 = θ₁ + θ₂
                # 这里用同侧大臂的角度
                if mid == 1:
                    theta1 = self.positions.get(0, 0.0)
                else:
                    theta1 = self.positions.get(2, 0.0)
                abs_angle = theta1 + pos
                tau = -self.tau_outer * math.cos(abs_angle + self.gravity_offset) / GEAR_RATIO * ramp

            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 1
            cmd.position_target = 0.0
            cmd.velocity_target = 0.0
            cmd.kp = 0.0
            cmd.kd = self.kd
            cmd.torque_ff = tau
            self.pub.publish(cmd)

    def _brake_all(self):
        for mid in ALL_IDS:
            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 0
            self.pub.publish(cmd)

    def smoothstep(self, t):
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def check_node_alive(self):
        now = time.time()
        for mid in ALL_IDS:
            last = self.last_update.get(mid)
            if last is not None and (now - last) <= NODE_TIMEOUT:
                return True
        return False

    def wait_for_motors(self, timeout=15.0):
        print("[INFO] 等待电机上线...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            online = [mid for mid in ALL_IDS if mid in self.positions]
            if len(online) >= 1 and (time.time() - start) >= 2.0:
                missing = [mid for mid in ALL_IDS if mid not in self.positions]
                if missing:
                    print(f"[WARN] 电机 {missing} 未响应，{len(online)}/4 启动")
                else:
                    print("[OK] 所有电机已上线")
                return True
        return False

    def run(self):
        if not self.wait_for_motors():
            print("[FAIL] 无电机响应")
            self.shutdown()
            return

        print(f"\n{'='*60}")
        print(f"  示教录制模式")
        print(f"{'='*60}")
        print(f"  重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")
        print(f"  阻尼: kd={self.kd}")
        print(f"  采样率: {self.rate_hz}Hz")
        print(f"  输出文件: {self.output}")
        print(f"  斜坡: {self.ramp_time}s")
        print()
        print(f"  请手动拖动机械臂完成动作，按 Ctrl+C 停止录制")
        print(f"{'='*60}\n")

        dt = 1.0 / self.rate_hz
        t_start = time.time()
        loop_count = 0

        while not self._stop:
            t_now = time.time()
            elapsed = t_now - t_start

            if not self.check_node_alive():
                print("\n[WARN] 节点无响应")
                self._brake_all()
                break

            for _ in range(16):  # 排空全部待处理消息(4电机×~4帧)
                rclpy.spin_once(self.node, timeout_sec=0)

            # 重力补偿
            ramp = self.smoothstep(min(1.0, elapsed / self.ramp_time)) if self.ramp_time > 0 else 1.0
            self._send_gravity(ramp)

            # 记录数据
            if ramp >= 1.0:  # 斜坡完成后才开始记录
                record = {'t': elapsed - self.ramp_time}
                for mid in ALL_IDS:
                    record[f'm{mid}'] = self.positions.get(mid, 0.0)
                    record[f'v{mid}'] = self.velocities.get(mid, 0.0)
                    record[f'tau{mid}'] = self.torques.get(mid, 0.0)
                self.records.append(record)

            loop_count += 1
            if loop_count % self.rate_hz == 0:
                n = len(self.records)
                dur = self.records[-1]['t'] if n > 0 else 0.0
                parts = []
                for mid in sorted(self.positions):
                    parts.append(f"M{mid}:{math.degrees(self.positions[mid]):+6.1f}°")
                print(f"  [{elapsed:5.1f}s] 已录 {n} 帧 ({dur:.1f}s) | {' '.join(parts)}")

            sleep_time = dt - (time.time() - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._brake_all()
        self._save()
        self.shutdown()

    def _save(self):
        if not self.records:
            print("[WARN] 无数据")
            return

        MAX_RECORDINGS = 10

        new_rec = {
            'rate_hz': self.rate_hz,
            'tau_inner': self.tau_inner,
            'tau_outer': self.tau_outer,
            'n_frames': len(self.records),
            'duration': self.records[-1]['t'],
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'frames': self.records,
        }

        # 读取已有文件，追加到录制列表
        recordings = []
        try:
            with open(self.output, 'r') as f:
                existing = json.load(f)
            if 'recordings' in existing:
                recordings = existing['recordings']
            elif 'frames' in existing:
                # 兼容旧单录制格式，迁移为列表
                recordings = [existing]
        except (FileNotFoundError, json.JSONDecodeError):
            pass

        recordings.append(new_rec)
        # 只保留最近 MAX_RECORDINGS 条
        if len(recordings) > MAX_RECORDINGS:
            recordings = recordings[-MAX_RECORDINGS:]

        with open(self.output, 'w') as f:
            json.dump({'recordings': recordings}, f, indent=2)

        idx = len(recordings) - 1
        print(f"\n{'='*60}")
        print(f"  录制完成 (#{idx}, 共 {len(recordings)} 条)")
        print(f"  帧数: {len(self.records)}")
        print(f"  时长: {self.records[-1]['t']:.2f}s")
        print(f"  文件: {self.output}")
        print(f"  起始: M0={math.degrees(self.records[0]['m0']):+.1f}° M1={math.degrees(self.records[0]['m1']):+.1f}° M2={math.degrees(self.records[0]['m2']):+.1f}° M3={math.degrees(self.records[0]['m3']):+.1f}°")
        print(f"  终点: M0={math.degrees(self.records[-1]['m0']):+.1f}° M1={math.degrees(self.records[-1]['m1']):+.1f}° M2={math.degrees(self.records[-1]['m2']):+.1f}° M3={math.degrees(self.records[-1]['m3']):+.1f}°")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='示教录制')
    parser.add_argument('--tau-inner', type=float, default=0,
                        help='大臂重力矩 Nm (默认 0)')
    parser.add_argument('--tau-outer', type=float, default=0,
                        help='小臂重力矩 Nm (默认 0)')
    parser.add_argument('--gravity-offset', type=float, default=-1.5708,
                        help='零位到水平的角度偏移 rad (默认 -π/2, 零位=下垂)')
    parser.add_argument('--kd', type=float, default=0.0,
                        help='转子侧阻尼 (默认 0, 纯零力拖动)')
    parser.add_argument('--ramp', type=float, default=1.5,
                        help='斜坡时间 s (默认 1.5)')
    parser.add_argument('--rate', type=int, default=200,
                        help='采样频率 Hz (默认 200)')
    parser.add_argument('--output', type=str, default='teach_data.json',
                        help='输出文件 (默认 teach_data.json, 追加模式最多保留10条)')
    args = parser.parse_args()

    recorder = TeachRecorder(args)
    recorder.run()


if __name__ == '__main__':
    main()
