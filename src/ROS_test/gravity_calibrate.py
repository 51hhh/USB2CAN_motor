#!/usr/bin/env python3
"""
重力补偿参数标定工具

原理: τ_gravity = tau_param × cos(θ + gravity_offset) / GEAR_RATIO
     在水平位置 cos(θ + gravity_offset) ≈ 1, τ ≈ tau_param / GEAR_RATIO
     → 通过观测不同 tau 值下的角度漂移，找到最优 tau

标定流程:
  1. 先校准零位（手臂自然下垂 = 0）
  2. 手动将手臂抬到指定角度（默认水平 ≈ 90°）
  3. 脚本在指定 tau 范围内扫描，每个值保持若干秒
  4. 记录角度漂移，选取漂移最小的 tau 值

用法:
  # 标定大臂 (先手动抬到水平)
  python3 gravity_calibrate.py --mode inner --tau-min 4.0 --tau-max 10.0 --steps 7

  # 标定小臂 (需先用已知 tau-inner 稳住大臂)
  python3 gravity_calibrate.py --mode outer --tau-inner 8.3 --tau-min 0.5 --tau-max 4.0 --steps 8

  # 精细标定 (缩小范围)
  python3 gravity_calibrate.py --mode inner --tau-min 7.5 --tau-max 9.0 --steps 10
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}
OUTER_IDS = {1, 3}
RATE_HZ = 100
NODE_TIMEOUT = 8.0

# 机械限位 (度)
INNER_LIMIT_DEG = 999.0  # 大臂限位
OUTER_LIMIT_DEG = 999.0  # 小臂限位 (暂时禁用)
LIMIT_MARGIN_DEG = 5.0    # 接近限位提前中止的余量


class GravityCalibrator:
    def __init__(self, args):
        self.mode = args.mode  # 'inner' or 'outer'
        self.tau_min = args.tau_min
        self.tau_max = args.tau_max
        self.steps = args.steps
        self.hold_sec = args.hold_sec
        self.kd = args.kd
        self.gravity_offset = args.gravity_offset

        # 当标定一侧时，另一侧使用固定值
        self.tau_inner_fixed = args.tau_inner
        self.tau_outer_fixed = args.tau_outer

        rclpy.init()
        self.node = rclpy.create_node('gravity_calibrate')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.velocities = {}
        self.last_update = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        self._stop = False
        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.last_update[msg.motor_id] = time.time()

    def _sig_handler(self, signum, frame):
        print("\n[STOP] Ctrl+C 安全退出...")
        self._stop = True

    def _send_cmd(self, motor_id, torque_ff, kd, kp=0.0, p_des=0.0):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1
        cmd.position_target = p_des
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

    def get_theta(self):
        """获取 θ₁, θ₂ 均值"""
        theta1 = (self.positions.get(0, 0.0) + self.positions.get(2, 0.0)) / 2.0
        theta2 = (self.positions.get(1, 0.0) + self.positions.get(3, 0.0)) / 2.0
        return theta1, theta2

    def compute_torque(self, motor_id, theta1, theta2, tau_inner, tau_outer):
        """计算单个电机的转子侧重力补偿力矩"""
        if motor_id in INNER_IDS:
            return tau_inner * math.cos(theta1 + self.gravity_offset) / GEAR_RATIO
        else:
            return -tau_outer * math.cos(theta1 + theta2 + self.gravity_offset) / GEAR_RATIO

    def return_to_position(self, target_theta1, target_theta2, duration=2.0):
        """用 kp 位控归位到目标角度"""
        KP_RESET = 1.0   # 转子侧 kp
        KD_RESET = 0.15   # 转子侧 kd
        dt = 1.0 / RATE_HZ
        t_start = time.time()
        while time.time() - t_start < duration and not self._stop:
            t_loop = time.time()
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0)
            for mid in ALL_IDS:
                p_des = target_theta1 if mid in INNER_IDS else target_theta2
                self._send_cmd(mid, 0.0, KD_RESET, KP_RESET, p_des)
            sleep_time = dt - (time.time() - t_loop)
            if sleep_time > 0:
                time.sleep(sleep_time)

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
                    print(f"[WARN] 电机 {missing} 未响应，以 {len(online)}/4 启动")
                else:
                    print("[OK] 所有电机已上线")
                return True
        return False

    def run_single_tau(self, tau_test, initial_theta1, initial_theta2):
        """
        用给定 tau 值保持若干秒，返回相对初始角度的漂移量。
        Returns: (drift_deg, start_angle_deg, end_angle_deg, hit_limit)
        """
        if self.mode == 'inner':
            tau_inner = tau_test
            tau_outer = self.tau_outer_fixed
            limit_deg = INNER_LIMIT_DEG
            initial_angle = initial_theta1
        else:
            tau_inner = self.tau_inner_fixed
            tau_outer = tau_test
            limit_deg = OUTER_LIMIT_DEG
            initial_angle = initial_theta2

        theta1_start, theta2_start = self.get_theta()
        hit_limit = False

        dt = 1.0 / RATE_HZ
        t_start = time.time()

        while not self._stop:
            t_loop = time.time()
            elapsed = t_loop - t_start

            if elapsed >= self.hold_sec:
                break

            if not self.check_node_alive():
                print("[WARN] 电机无响应")
                return None

            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0)

            theta1, theta2 = self.get_theta()

            # 检测限位
            check_angle = theta1 if self.mode == 'inner' else theta2
            if abs(math.degrees(check_angle)) >= limit_deg - LIMIT_MARGIN_DEG:
                hit_limit = True
                break

            for mid in ALL_IDS:
                tau_ff = self.compute_torque(mid, theta1, theta2, tau_inner, tau_outer)
                self._send_cmd(mid, tau_ff, self.kd)

            sleep_time = dt - (time.time() - t_loop)
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 记录结束角度——漂移相对于初始放置角度
        theta1_end, theta2_end = self.get_theta()

        if self.mode == 'inner':
            drift_rad = theta1_end - initial_angle
            return (math.degrees(drift_rad), math.degrees(initial_angle), math.degrees(theta1_end), hit_limit)
        else:
            drift_rad = theta2_end - initial_angle
            return (math.degrees(drift_rad), math.degrees(initial_angle), math.degrees(theta2_end), hit_limit)

    def run(self):
        if not self.wait_for_motors():
            print("[FAIL] 无电机响应")
            self.shutdown()
            return

        theta1, theta2 = self.get_theta()
        label = '大臂' if self.mode == 'inner' else '小臂'
        target_angle = theta1 if self.mode == 'inner' else theta2

        print(f"\n{'='*60}")
        print(f"  重力参数标定 — {label}")
        print(f"  当前角度: θ₁={math.degrees(theta1):+.1f}° θ₂={math.degrees(theta2):+.1f}°")
        print(f"  gravity_offset = {self.gravity_offset:.4f} rad ({math.degrees(self.gravity_offset):+.1f}°)")
        print(f"  cos(θ + offset) = {math.cos(target_angle + self.gravity_offset):.3f}")
        print(f"  tau 扫描范围: [{self.tau_min:.1f}, {self.tau_max:.1f}] Nm, {self.steps} 步")
        print(f"  每步保持: {self.hold_sec}s | kd = {self.kd}")
        if self.mode == 'outer':
            print(f"  固定 tau_inner = {self.tau_inner_fixed} Nm")
        elif self.mode == 'inner':
            print(f"  固定 tau_outer = {self.tau_outer_fixed} Nm")
        print(f"{'='*60}")

        # 5秒准备时间
        print(f"\n  请将{label}移到测试角度，5秒后开始扫描...")
        t_wait = time.time()
        while time.time() - t_wait < 5 and not self._stop:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            remaining = 5 - int(time.time() - t_wait)
            theta1, theta2 = self.get_theta()
            angle = theta1 if self.mode == 'inner' else theta2
            print(f"\r    倒计时 {remaining}s | {label}角度: {math.degrees(angle):+.1f}°   ", end='', flush=True)
        print()

        if self._stop:
            self._brake_all()
            self.shutdown()
            return

        # 记录初始放置角度作为全局基准
        init_theta1, init_theta2 = self.get_theta()
        init_angle = init_theta1 if self.mode == 'inner' else init_theta2
        print(f"  初始基准角度: {math.degrees(init_angle):+.1f}°")

        # 扫描
        tau_values = [self.tau_min + (self.tau_max - self.tau_min) * i / max(self.steps - 1, 1)
                      for i in range(self.steps)]

        results = []
        print(f"\n  {'tau(Nm)':>8}  {'漂移(°)':>8}  {'基准(°)':>8}  {'结束(°)':>8}  {'评级':>6}")
        print(f"  {'─'*50}")

        for i, tau in enumerate(tau_values):
            if self._stop:
                break

            # 归位到初始角度（首次不需要）
            if i > 0:
                print(f"  ... 归位中 ...", end='\r')
                self.return_to_position(init_theta1, init_theta2, duration=2.0)

            result = self.run_single_tau(tau, init_theta1, init_theta2)
            if result is None:
                break

            drift, start, end, hit_limit = result
            abs_drift = abs(drift)

            if hit_limit:
                grade = '▓限位'
            elif abs_drift < 1.0:
                grade = '★★★'
            elif abs_drift < 3.0:
                grade = '★★'
            elif abs_drift < 6.0:
                grade = '★'
            else:
                grade = '—'

            direction = '↓' if drift < 0 else '↑'
            results.append((tau, drift, start, end, hit_limit))
            limit_mark = ' ←触限位!' if hit_limit else ''
            print(f"  {tau:8.2f}  {drift:+7.2f}{direction}  {start:+7.1f}  {end:+7.1f}  {grade}{limit_mark}")

            # 两步之间短暂静止
            if i < len(tau_values) - 1 and not self._stop:
                time.sleep(0.3)

        self._brake_all()

        # 分析结果
        if results:
            print(f"\n{'='*60}")
            print(f"  标定结果分析")

            best = min(results, key=lambda x: abs(x[1]))
            if best[4]:  # hit_limit
                print(f"  ☹ 所有值均触及限位，请减小 tau 范围")
            else:
                print(f"  最优 tau = {best[0]:.2f} Nm (漂移 {best[1]:+.2f}°)")

            # 如果最优在边界，提示扩大范围
            non_limit = [r for r in results if not r[4]]
            if non_limit:
                best_nl = min(non_limit, key=lambda x: abs(x[1]))
                if best_nl[0] == tau_values[0]:
                    print(f"  ⚠ 最优值在下限! 建议减小 --tau-min")
                elif best_nl[0] == tau_values[-1]:
                    print(f"  ⚠ 最优值在上限! 建议增大 --tau-max")

            # 漂移方向分析 (只用未触限位的数据点)
            valid = [r for r in results if not r[4]]
            neg_drifts = [r for r in valid if r[1] < -1.0]
            pos_drifts = [r for r in valid if r[1] > 1.0]
            if neg_drifts and pos_drifts:
                # 有交叉点，可以插值
                for j in range(len(valid)-1):
                    d1 = valid[j][1]
                    d2 = valid[j+1][1]
                    if d1 * d2 < 0:  # 符号变化
                        t1, t2 = valid[j][0], valid[j+1][0]
                        tau_zero = t1 + (t2 - t1) * (-d1) / (d2 - d1)
                        print(f"  插值最优 tau ≈ {tau_zero:.2f} Nm (零漂移交叉点)")
                        print(f"  建议精细标定: --tau-min {tau_zero-0.5:.1f} --tau-max {tau_zero+0.5:.1f} --steps 10")
                        break

            # 建议
            cos_val = math.cos(target_angle + self.gravity_offset)
            if abs(cos_val) < 0.3:
                print(f"  ⚠ cos(θ+offset)={cos_val:.2f}，当前角度接近竖直，灵敏度低。")
                print(f"    建议将手臂移到更接近水平的位置再标定。")

            print(f"{'='*60}")

        self.shutdown()

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='重力补偿参数标定工具')
    parser.add_argument('--mode', choices=['inner', 'outer'], required=True,
                        help='标定目标: inner=大臂, outer=小臂')
    parser.add_argument('--tau-min', type=float, default=3.0,
                        help='扫描下限 Nm (默认 3.0)')
    parser.add_argument('--tau-max', type=float, default=12.0,
                        help='扫描上限 Nm (默认 12.0)')
    parser.add_argument('--steps', type=int, default=7,
                        help='扫描步数 (默认 7)')
    parser.add_argument('--hold-sec', type=float, default=4.0,
                        help='每步保持时间 s (默认 4.0)')
    parser.add_argument('--tau-inner', type=float, default=0.0,
                        help='标定小臂时固定的大臂tau Nm (默认 0)')
    parser.add_argument('--tau-outer', type=float, default=0.0,
                        help='标定大臂时固定的小臂tau Nm (默认 0)')
    parser.add_argument('--gravity-offset', type=float, default=-math.pi/2,
                        help='零位角度偏移 rad (默认 -π/2)')
    parser.add_argument('--kd', type=float, default=0.02,
                        help='转子阻尼 (默认 0.02)')
    args = parser.parse_args()

    calibrator = GravityCalibrator(args)
    calibrator.run()


if __name__ == '__main__':
    main()
