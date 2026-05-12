#!/usr/bin/env python3
"""
四电机五连杆机械臂 — 重力补偿 + 小kp位置运动

在已验证的纯力矩重力补偿基础上，增加很小的kp实现位置控制:
  τ = τ_ff (重力补偿) + kp*(p_des - p) + kd*(ω_des - ω)

三阶段运行:
  1. 斜坡: 力矩和kp从0平滑升起，位置目标=当前位置
  2. 运动: smoothstep轨迹从当前位置插值到目标位置
  3. 保持: 重力补偿 + kp 锁定在目标位置

参数:
  --tau-inner      M0/M2 大臂重力矩 Nm (默认 8.3)
  --tau-outer      M1/M3 小臂重力矩 Nm (默认 2.0)
  --kp             转子侧位置增益 (默认 0.1, 很小以避免噪音)
  --kd             转子侧阻尼 (默认 0.02)
  --target-inner   M0/M2 目标角度 deg (默认 100)
  --target-outer   M1/M3 目标角度 deg (默认 70)
  --move-time      运动时间 s (默认 3.0)
  --ramp           力矩斜坡时间 s (默认 1.5)

用法:
  python3 move_to_position.py
  python3 move_to_position.py --target-inner 90 --target-outer 60 --kp 0.15
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
RATE_HZ = 100
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85

INNER_IDS = {0, 2}  # 大臂
OUTER_IDS = {1, 3}  # 小臂


class MoveToPosition:
    def __init__(self, args):
        self.tau_inner = args.tau_inner
        self.tau_outer = args.tau_outer
        self.kp = args.kp
        self.kd = args.kd
        self.target_inner = math.radians(args.target_inner)
        self.target_outer = math.radians(args.target_outer)
        self.move_time = args.move_time
        self.ramp_time = args.ramp

        rclpy.init()
        self.node = rclpy.create_node('move_to_position')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.velocities = {}
        self.temperatures = {}
        self.errors = {}
        self.last_update = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        self._stop = False
        self._start_time = None
        self._initial_positions = {}

        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.temperatures[msg.motor_id] = msg.temperature
        self.errors[msg.motor_id] = msg.error
        self.last_update[msg.motor_id] = time.time()

    def _sig_handler(self, signum, frame):
        print("\n[STOP] 收到停止信号，安全退出...")
        self._stop = True

    def _send_cmd(self, motor_id, torque_ff, kp, kd, position_target):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1
        cmd.position_target = position_target
        cmd.velocity_target = 0.0
        cmd.kp = kp
        cmd.kd = kd
        cmd.torque_ff = torque_ff
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

    def gravity_torque_rotor(self, motor_id, angle_rad):
        tau = self.tau_inner if motor_id in INNER_IDS else self.tau_outer
        return tau * math.cos(angle_rad) / GEAR_RATIO

    def get_target(self, motor_id):
        return self.target_inner if motor_id in INNER_IDS else self.target_outer

    def get_trajectory_position(self, motor_id, elapsed):
        """当前轨迹位置: 斜坡期保持初始位置，运动期 smoothstep 到目标"""
        initial = self._initial_positions.get(motor_id, self.get_target(motor_id))
        target = self.get_target(motor_id)

        if elapsed < self.ramp_time:
            return initial

        move_elapsed = elapsed - self.ramp_time
        if move_elapsed >= self.move_time:
            return target

        t = self.smoothstep(move_elapsed / self.move_time)
        return initial + (target - initial) * t

    def get_kp_ramp(self, elapsed):
        if self.ramp_time <= 0 or elapsed >= self.ramp_time:
            return self.kp
        return self.kp * self.smoothstep(elapsed / self.ramp_time)

    def get_torque_ramp(self, elapsed):
        if self.ramp_time <= 0 or elapsed >= self.ramp_time:
            return 1.0
        return self.smoothstep(elapsed / self.ramp_time)

    def check_node_alive(self):
        now = time.time()
        for mid in ALL_IDS:
            last = self.last_update.get(mid)
            if last is not None and (now - last) <= NODE_TIMEOUT:
                return True
        return False

    def check_safety(self):
        for mid in ALL_IDS:
            temp = self.temperatures.get(mid)
            err = self.errors.get(mid, 0)
            if temp is not None and temp >= TEMP_CRITICAL:
                print(f"\n[CRITICAL] M{mid} 温度={temp}C >= {TEMP_CRITICAL}C!")
                return False
            if err in (1, 2, 3, 4):
                err_names = {1: '过热', 2: '过流', 3: '过压', 4: '编码器故障'}
                print(f"\n[ERROR] M{mid} 错误: {err_names[err]}")
                return False
            if err != 0:
                print(f"  [WARN] M{mid} error={err}（保留码，继续运行）")
        return True

    def wait_for_motors(self, timeout=15.0):
        print("[INFO] 等待电机上线...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            online = [mid for mid in ALL_IDS if mid in self.positions]
            if len(online) >= 1 and (time.time() - start) >= 2.0:
                missing = [mid for mid in ALL_IDS if mid not in self.positions]
                if missing:
                    print(f"\n[WARN] 电机 {missing} 未响应，以 {len(online)}/4 电机启动")
                else:
                    print("\n[OK] 所有电机已上线")
                return True
            if online and int((time.time() - start) * 10) % 10 == 0:
                print(f"\r   已检测到: {online}", end='', flush=True)
        online = [mid for mid in ALL_IDS if mid in self.positions]
        if online:
            print(f"\n[WARN] 超时，以 {len(online)}/4 电机启动: {online}")
            return True
        print(f"\n[FAIL] 超时，无电机检测到")
        return False

    def run(self):
        if not self.wait_for_motors():
            self.shutdown()
            return

        for mid in ALL_IDS:
            if mid in self.positions:
                self._initial_positions[mid] = self.positions[mid]

        target_inner_deg = math.degrees(self.target_inner)
        target_outer_deg = math.degrees(self.target_outer)
        print(f"\n  == 重力补偿 + 位置控制 ==")
        print(f"  M0/M2 大臂: tau={self.tau_inner:.1f}Nm → 目标 {target_inner_deg:.0f}°")
        print(f"  M1/M3 小臂: tau={self.tau_outer:.1f}Nm → 目标 {target_outer_deg:.0f}°")
        print(f"  kp={self.kp} | kd={self.kd}")
        print(f"  斜坡={self.ramp_time}s | 运动时间={self.move_time}s")
        for mid in sorted(self._initial_positions):
            print(f"  M{mid} 初始: {math.degrees(self._initial_positions[mid]):+.1f}°")
        print(f"\n  Ctrl+C 退出\n")

        dt = 1.0 / RATE_HZ
        self._start_time = time.time()
        loop_count = 0

        while not self._stop:
            t_loop_start = time.time()
            elapsed = t_loop_start - self._start_time

            if not self.check_node_alive():
                print("\n[WARN] 电机控制节点无响应（可能已断电），停止")
                self._brake_all()
                break

            if loop_count % (RATE_HZ * 10) == 0 and loop_count > 0:
                if not self.check_safety():
                    print("[STOP] 安全检查未通过，刹车")
                    self._brake_all()
                    break

            rclpy.spin_once(self.node, timeout_sec=0)

            ramp = self.get_torque_ramp(elapsed)
            kp_now = self.get_kp_ramp(elapsed)

            for mid in ALL_IDS:
                pos = self.positions.get(mid, 0.0)
                tau_ff = self.gravity_torque_rotor(mid, pos) * ramp
                pos_target = self.get_trajectory_position(mid, elapsed)
                self._send_cmd(mid, tau_ff, kp_now, self.kd, pos_target)

            loop_count += 1
            if loop_count % (RATE_HZ * 5) == 0:
                self._print_status(elapsed)

            sleep_time = dt - (time.time() - t_loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._print_report()
        self.shutdown()

    def _print_status(self, elapsed):
        parts = []
        for mid in sorted(self.positions):
            pos = self.positions[mid]
            target = self.get_target(mid)
            err_deg = math.degrees(target - pos)
            temp = self.temperatures.get(mid, -1)
            parts.append(f"M{mid}:{math.degrees(pos):+6.1f}° err={err_deg:+5.1f}° {temp}C")
        if elapsed < self.ramp_time:
            phase = "斜坡"
        elif elapsed < self.ramp_time + self.move_time:
            phase = "运动"
        else:
            phase = "保持"
        print(f"  [{elapsed:5.0f}s] {phase} {' | '.join(parts)}")

    def _print_report(self):
        print(f"\n{'='*60}")
        print(f"  运行报告")
        if self._start_time:
            print(f"  运行时长: {time.time() - self._start_time:.1f}s")
        print(f"  参数: kp={self.kp}, kd={self.kd}")
        print(f"  目标: 大臂={math.degrees(self.target_inner):.0f}° 小臂={math.degrees(self.target_outer):.0f}°")
        if self.positions:
            print(f"  最终位置:")
            for mid in sorted(self.positions):
                pos = self.positions[mid]
                target = self.get_target(mid)
                err = math.degrees(target - pos)
                temp = self.temperatures.get(mid, -1)
                label = '大臂' if mid in INNER_IDS else '小臂'
                print(f"    M{mid}({label}): {math.degrees(pos):+.1f}° → 目标 {math.degrees(target):.0f}° 误差 {err:+.1f}° | {temp}C")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='重力补偿 + 小kp位置运动')
    parser.add_argument('--tau-inner', type=float, default=3.1,
                        help='M0/M2 大臂重力矩 Nm (默认 3.1)')
    parser.add_argument('--tau-outer', type=float, default=1.5,
                        help='M1/M3 小臂重力矩 Nm (默认 1.5)')
    parser.add_argument('--kp', type=float, default=0.1,
                        help='转子侧位置增益 (默认 0.1)')
    parser.add_argument('--kd', type=float, default=0.02,
                        help='转子侧阻尼 (默认 0.02)')
    parser.add_argument('--target-inner', type=float, default=100.0,
                        help='M0/M2 目标角度 deg (默认 100)')
    parser.add_argument('--target-outer', type=float, default=70.0,
                        help='M1/M3 目标角度 deg (默认 70)')
    parser.add_argument('--move-time', type=float, default=3.0,
                        help='运动时间 s (默认 3.0)')
    parser.add_argument('--ramp', type=float, default=1.5,
                        help='力矩斜坡时间 s (默认 1.5)')
    args = parser.parse_args()

    ctrl = MoveToPosition(args)
    ctrl.run()


if __name__ == '__main__':
    main()
