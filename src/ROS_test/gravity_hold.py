#!/usr/bin/env python3
"""
四电机五连杆机械臂 — 纯力矩重力补偿保持（消噪版）

基于 GO-M8010-6 手册 v1.2:
  电机控制公式: τ = τ_ff + kp*(p_des - p) + kd*(ω_des - ω)
  所有参数均为转子侧，减速比 6.33

消噪策略:
  kp = 0  → 无位置校正（消除高频振荡噪音的根源）
  kd = 小 → 粘滞阻尼（手册推荐 0.02，抑制晃动且静音）
  τ_ff    → 纯力矩抵消重力

五连杆力矩分配:
  M0/M2 (大臂，内侧): 承受大臂+小臂+击球板 → --tau-inner
  M1/M3 (小臂，外侧): 仅承受小臂+击球板 → --tau-outer

参数:
  --tau-inner  M0/M2 大臂输出端重力矩 Nm（默认 6.0）
  --tau-outer  M1/M3 小臂输出端重力矩 Nm（默认 2.0）
  --kd         转子侧阻尼系数（默认 0.02，手册推荐值）
  --ramp       力矩斜坡时间 s（平滑启动，默认 1.5）

用法:
  python3 gravity_hold.py --tau-inner 6.0 --tau-outer 2.0
  python3 gravity_hold.py --tau-inner 8.0 --tau-outer 3.0 --kd 0.03

  Ctrl+C 安全退出 | 检测到断电自动停止
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
TEMP_WARN = 70       # 温度警告阈值
TEMP_CRITICAL = 85   # 温度临界阈值（手册: 90°C 触发保护，需断电重启）


# M0/M2 = 大臂(内侧), M1/M3 = 小臂(外侧)
INNER_IDS = {0, 2}  # 大臂
OUTER_IDS = {1, 3}  # 小臂


class GravityHold:
    def __init__(self, tau_inner, tau_outer, kd, ramp_time, gravity_offset=-math.pi/2):
        self.tau_inner = tau_inner  # M0/M2 大臂输出端重力矩
        self.tau_outer = tau_outer  # M1/M3 小臂输出端重力矩
        self.kd = kd
        self.ramp_time = ramp_time
        self.gravity_offset = gravity_offset

        rclpy.init()
        self.node = rclpy.create_node('gravity_hold')
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
        self._log = []

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

    def _send_cmd(self, motor_id, torque_ff, kd):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1
        cmd.position_target = 0.0
        cmd.velocity_target = 0.0
        cmd.kp = 0.0
        cmd.kd = kd
        cmd.torque_ff = torque_ff
        self.pub.publish(cmd)

    def _brake_all(self):
        for mid in ALL_IDS:
            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 0
            self.pub.publish(cmd)

    def ramp_factor(self, elapsed):
        """力矩斜坡因子: 0→1 平滑过渡"""
        if self.ramp_time <= 0 or elapsed >= self.ramp_time:
            return 1.0
        t = elapsed / self.ramp_time
        return t * t * (3.0 - 2.0 * t)  # smoothstep

    def gravity_torque_rotor(self, motor_id, theta1, theta2, ramp):
        if motor_id in INNER_IDS:
            return self.tau_inner * math.cos(theta1 + self.gravity_offset) / GEAR_RATIO * ramp
        else:
            # 小臂: 用绝对角度 θ₁+θ₂
            return -self.tau_outer * math.cos(theta1 + theta2 + self.gravity_offset) / GEAR_RATIO * ramp

    def check_node_alive(self):
        now = time.time()
        for mid in ALL_IDS:
            last = self.last_update.get(mid)
            if last is not None and (now - last) <= NODE_TIMEOUT:
                return True
        return False

    def check_safety(self):
        """检查温度和错误（仅对已知危险错误停机）"""
        for mid in ALL_IDS:
            temp = self.temperatures.get(mid)
            err = self.errors.get(mid, 0)
            if temp is not None and temp >= TEMP_CRITICAL:
                print(f"\n[CRITICAL] M{mid} 温度={temp}C >= {TEMP_CRITICAL}C! 90C将触发保护(需断电重启)")
                return False
            if err in (1, 2, 3, 4):  # 过热/过流/过压/编码器故障 → 停机
                err_names = {1: '过热', 2: '过流', 3: '过压', 4: '编码器故障'}
                print(f"\n[ERROR] M{mid} 错误: {err_names[err]}")
                return False
            if err != 0:  # 其他保留值 → 仅警告
                print(f"  [WARN] M{mid} error={err}（保留码，继续运行）")
        return True

    def wait_for_motors(self, timeout=15.0):
        print("[INFO] 等待电机上线...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            online = [mid for mid in ALL_IDS if mid in self.positions]
            if len(online) >= 1 and (time.time() - start) >= 2.0:
                # 至少1个电机且等待超过2秒
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

        print(f"\n  == 纯力矩重力补偿 (消噪模式) ==")
        print(f"  M0/M2 大臂: tau={self.tau_inner:.2f} Nm (输出端) = {self.tau_inner/GEAR_RATIO:.4f} Nm (转子侧)")
        print(f"  M1/M3 小臂: tau={self.tau_outer:.2f} Nm (输出端) = {self.tau_outer/GEAR_RATIO:.4f} Nm (转子侧)")
        print(f"  kp = 0 (无位置校正) | kd = {self.kd} (转子阻尼)")
        print(f"  斜坡 = {self.ramp_time}s | 频率 = {RATE_HZ}Hz")

        # 温度显示
        for mid in sorted(self.temperatures):
            print(f"  M{mid}: {self.temperatures[mid]}C", end='')
        print()

        # === 等待摆放阶段: 5秒内不发命令，让用户摆放机械臂 ===
        wait_sec = 5
        print(f"\n  请在 {wait_sec} 秒内摆放机械臂到期望角度...")
        t_wait_start = time.time()
        last_print = -1
        while time.time() - t_wait_start < wait_sec and not self._stop:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            remaining = wait_sec - int(time.time() - t_wait_start)
            if remaining != last_print:
                last_print = remaining
                parts = []
                for mid in sorted(self.positions):
                    parts.append(f"M{mid}:{math.degrees(self.positions[mid]):+6.1f}°")
                print(f"    倒计时 {remaining}s | {' '.join(parts)}")

        if self._stop:
            self.shutdown()
            return

        print(f"\n  开始重力补偿! Ctrl+C 退出\n")

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

            for _ in range(16):  # 排空全部待处理消息
                rclpy.spin_once(self.node, timeout_sec=0)

            ramp = self.ramp_factor(elapsed)
            theta1 = (self.positions.get(0, 0.0) + self.positions.get(2, 0.0)) / 2.0
            theta2 = (self.positions.get(1, 0.0) + self.positions.get(3, 0.0)) / 2.0
            snapshot = {}
            for mid in ALL_IDS:
                pos = self.positions.get(mid, 0.0)
                vel = self.velocities.get(mid, 0.0)
                tau_ff = self.gravity_torque_rotor(mid, theta1, theta2, ramp)
                self._send_cmd(mid, tau_ff, self.kd)
                snapshot[mid] = (pos, vel, tau_ff)

            loop_count += 1
            if loop_count % RATE_HZ == 0:
                self._log.append((elapsed, dict(snapshot)))
                if loop_count % (RATE_HZ * 5) == 0:
                    self._print_status(elapsed, snapshot)

            sleep_time = dt - (time.time() - t_loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._print_report()
        self.shutdown()

    def _print_status(self, elapsed, snapshot):
        parts = []
        for mid in sorted(snapshot):
            pos, vel, tau = snapshot[mid]
            temp = self.temperatures.get(mid, -1)
            parts.append(f"M{mid}:{math.degrees(pos):+6.1f}* t_r={tau:+.4f} {temp}C")
        ramp = self.ramp_factor(elapsed)
        print(f"  [{elapsed:5.0f}s] ramp={ramp:.2f} {' | '.join(parts)}")

    def _print_report(self):
        print(f"\n{'='*60}")
        print(f"  运行报告")
        if self._start_time:
            print(f"  运行时长: {time.time() - self._start_time:.1f}s")
        print(f"  参数: tau_inner={self.tau_inner}Nm, tau_outer={self.tau_outer}Nm, kd={self.kd}")
        if self.positions:
            print(f"  最终位置:")
            for mid in sorted(self.positions):
                pos = self.positions[mid]
                tau_base = self.tau_inner if mid in INNER_IDS else self.tau_outer
                tau_o = tau_base * math.cos(pos)
                temp = self.temperatures.get(mid, -1)
                label = '大臂' if mid in INNER_IDS else '小臂'
                print(f"    M{mid}({label}): {math.degrees(pos):+.2f} deg | tau_out={tau_o:+.2f} Nm | {temp}C")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='纯力矩重力补偿保持（消噪版）')
    parser.add_argument('--tau-inner', type=float, default=5.5,
                        help='M0/M2 大臂输出端重力矩 Nm（默认 6.0）')
    parser.add_argument('--tau-outer', type=float, default=0.5,
                        help='M1/M3 小臂输出端重力矩 Nm（默认 2.0）')
    parser.add_argument('--gravity-offset', type=float, default=-1.5708,
                        help='零位到水平的角度偏移 rad (默认 -π/2, 零位=下垂)')
    parser.add_argument('--kd', type=float, default=0.02,
                        help='转子侧阻尼系数（默认 0.02，手册推荐）')
    parser.add_argument('--ramp', type=float, default=1,
                        help='力矩斜坡时间 s（默认 1.5）')
    args = parser.parse_args()

    holder = GravityHold(args.tau_inner, args.tau_outer, args.kd, args.ramp,
                          args.gravity_offset)
    holder.run()


if __name__ == '__main__':
    main()
