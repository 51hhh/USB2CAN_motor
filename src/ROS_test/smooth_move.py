#!/usr/bin/env python3
"""
丝滑电机运动控制器 — 外部力矩控制版
- 五次多项式 S 曲线规划（起止速度/加速度 = 0）
- 外部 PD + 重力/摩擦前馈 → 全部通过 torque_ff 发送
- 电机内部 kp/kd 设为极小值（安全兜底），力矩主要由外部计算
- 100Hz 外部 PD 输出平滑力矩，不会产生高频振荡噪音

机械臂结构：
  - 左臂: L1(id=0) + L2(id=1) 两电机驱动
  - 右臂: R1(id=2) + R2(id=3) 两电机驱动
  - 0° = 水平向前
  - 每臂约 2.5kg 负载, 力臂 ~0.2m

用法:
  python3 smooth_move.py --L1 100 --L2 70 --R1 100 --R2 70 --duration 10
  python3 smooth_move.py --all 0 --duration 8
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import sys

# ─── 机械参数 ───
ARM_MASS = 2.5             # 每条臂等效负载质量（kg），总 5kg 分两臂
ARM_LENGTH = 0.20          # 等效力臂长度（m）
GRAVITY = 9.8              # 重力加速度
MOTORS_PER_ARM = 2         # 每臂电机数量
COULOMB_FRICTION = 1.0     # 库仑摩擦估计（Nm）

# ─── 安全参数 ───
MAX_ANGLE_DEG = 180.0
MAX_VEL_RAD = 3.0
MIN_DURATION = 3.0
MAX_TORQUE = 10.0          # 外部力矩输出限幅（Nm）

# ─── 外部 PD 增益（控制 torque_ff 计算）───
EXT_KP = 5.0     # 外部位置增益（Nm/rad）
EXT_KD = 1.5     # 外部速度阻尼（Nm/(rad/s)）

# ─── 电机内部增益（极小安全兜底，不参与主控制）───
MOTOR_KP = 0.3   # 内部位置刚度（极低，仅安全保护）
MOTOR_KD = 0.3   # 内部速度阻尼（极低，仅安全保护）

RATE_HZ = 100


class SmoothMover:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('smooth_mover')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)
        self._stop = False
        signal.signal(signal.SIGINT, self._emergency_stop)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position

    def _emergency_stop(self, signum, frame):
        """Ctrl+C: 立即停止所有电机"""
        print("\n🛑 紧急停止！发送刹车命令...")
        self._stop = True
        for mid in [0, 1, 2, 3]:
            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 0  # BRAKE
            self.pub.publish(cmd)
        time.sleep(0.1)

    def _send_cmd(self, motor_id, pos, vel, kp=KP, kd=KD, torque_ff=0.0):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1  # FOC
        cmd.position_target = pos
        cmd.velocity_target = vel
        cmd.kp = kp
        cmd.kd = kd
        cmd.torque_ff = torque_ff
        self.pub.publish(cmd)

    @staticmethod
    def gravity_torque(angle_rad):
        """计算单电机需要的重力补偿力矩

        假设 0° = 水平, 正方向向上
        τ_gravity = (m * g * L * cos(θ)) / 每臂电机数
        """
        return ARM_MASS * GRAVITY * ARM_LENGTH * math.cos(angle_rad) / MOTORS_PER_ARM

    @staticmethod
    def feedforward_torque(angle_rad, vel_des):
        """计算完整前馈力矩 = 重力补偿 + 摩擦补偿

        摩擦补偿在运动方向上叠加库仑摩擦项，帮助电机克服静摩擦启动
        """
        tau_gravity = ARM_MASS * GRAVITY * ARM_LENGTH * math.cos(angle_rad) / MOTORS_PER_ARM
        # 摩擦补偿：与期望速度同方向的恒定力矩
        if abs(vel_des) > 0.01:
            tau_friction = COULOMB_FRICTION * (1.0 if vel_des > 0 else -1.0)
        else:
            tau_friction = 0.0
        return tau_gravity + tau_friction

    def wait_for_positions(self, timeout=10.0):
        """等待读取到全部 4 个电机位置"""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(self.positions) >= 4:
                return True
        return False

    @staticmethod
    def quintic_trajectory(t, T, p0, pf):
        """五次多项式轨迹规划

        满足边界条件: pos(0)=p0, pos(T)=pf, vel(0)=vel(T)=0, acc(0)=acc(T)=0

        返回: (position, velocity, acceleration)
        """
        if T <= 0:
            return pf, 0.0, 0.0
        s = t / T
        s = max(0.0, min(1.0, s))

        # 五次多项式: s(t) = 10s³ - 15s⁴ + 6s⁵
        poly = 10 * s**3 - 15 * s**4 + 6 * s**5
        dpoly = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        ddpoly = (60 * s - 180 * s**2 + 120 * s**3) / (T * T)

        disp = pf - p0
        pos = p0 + disp * poly
        vel = disp * dpoly
        acc = disp * ddpoly
        return pos, vel, acc

    def move(self, targets_deg, duration):
        """
        执行丝滑运动（带重力补偿）
        """
        # 安全检查：角度限幅
        for mid, deg in targets_deg.items():
            if abs(deg) > MAX_ANGLE_DEG:
                print(f"❌ 电机 {mid} 目标角度 {deg}° 超过安全限制 ±{MAX_ANGLE_DEG}°")
                return False

        duration = max(duration, MIN_DURATION)

        # 读取当前位置
        print("📡 读取电机当前位置...")
        if not self.wait_for_positions():
            print("❌ 未能读取到所有电机位置，请确认节点和电机正常")
            return False

        starts_rad = {}
        targets_rad = {}
        for mid, deg in targets_deg.items():
            starts_rad[mid] = self.positions.get(mid, 0.0)
            targets_rad[mid] = math.radians(deg)

        # 安全检查：峰值速度
        for mid in targets_deg:
            disp = abs(targets_rad[mid] - starts_rad[mid])
            peak_vel = 1.875 * disp / duration
            if peak_vel > MAX_VEL_RAD:
                duration = max(duration, 1.875 * disp / MAX_VEL_RAD + 0.5)
                print(f"⚠️  自动延长运动时间至 {duration:.1f}s（电机 {mid} 峰值速度限制）")

        print(f"\n🎯 运动参数:")
        print(f"   时间: {duration:.1f}s | kp={KP} | kd={KD} | 频率={RATE_HZ}Hz")
        print(f"   重力补偿: {ARM_MASS}kg × {ARM_LENGTH}m / {MOTORS_PER_ARM}电机")
        print(f"   轨迹: 五次多项式 S 曲线")
        print(f"   {'电机':>8s}  {'起点':>8s}  {'终点':>8s}  {'行程':>8s}")
        for mid in sorted(targets_deg):
            s = math.degrees(starts_rad[mid])
            e = targets_deg[mid]
            d = e - s
            print(f"   id={mid:>4d}  {s:>+7.1f}°  {e:>+7.1f}°  {d:>+7.1f}°")

        print(f"\n▶ 开始运动...")
        dt = 1.0 / RATE_HZ
        steps = int(duration * RATE_HZ)

        t_start = time.time()
        for i in range(steps + 1):
            if self._stop:
                print("⏹ 已紧急停止")
                return False

            t = i * dt
            for mid in targets_deg:
                pos, vel, _ = self.quintic_trajectory(
                    t, duration, starts_rad[mid], targets_rad[mid])

                # 前馈力矩 = 重力补偿 + 摩擦补偿
                tau_ff = self.feedforward_torque(pos, vel)

                # vel_des=0: 让电机 kd 项变为纯粘滞阻尼 -kd*vel_actual
                # 避免 kd*(vel_des-vel_actual) 的高频力矩翻转噪音
                self._send_cmd(mid, pos, 0.0, torque_ff=tau_ff)

            # 周期间处理回调（更新位置反馈）
            rclpy.spin_once(self.node, timeout_sec=0)
            elapsed = time.time() - t_start
            sleep_time = (i + 1) * dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 到达目标后，保持 2 秒稳定（带重力补偿）
        print("   到达目标，稳定中...")
        hold_steps = int(2.0 * RATE_HZ)
        for _ in range(hold_steps):
            if self._stop:
                break
            for mid in targets_deg:
                tau_ff = self.feedforward_torque(targets_rad[mid], 0.0)
                self._send_cmd(mid, targets_rad[mid], 0.0, torque_ff=tau_ff)
            rclpy.spin_once(self.node, timeout_sec=0)
            time.sleep(dt)

        # 报告最终位置
        rclpy.spin_once(self.node, timeout_sec=0.5)
        print(f"\n✅ 运动完成:")
        for mid in sorted(targets_deg):
            actual = math.degrees(self.positions.get(mid, float('nan')))
            target = targets_deg[mid]
            err = actual - target
            print(f"   id={mid}: {actual:+.2f}° (目标 {target:+.1f}°, 误差 {err:+.2f}°)")

        return True

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='丝滑电机运动控制器')
    parser.add_argument('--L1', type=float, default=None, help='L1 目标角度（度）')
    parser.add_argument('--L2', type=float, default=None, help='L2 目标角度（度）')
    parser.add_argument('--R1', type=float, default=None, help='R1 目标角度（度）')
    parser.add_argument('--R2', type=float, default=None, help='R2 目标角度（度）')
    parser.add_argument('--all', type=float, default=None, help='所有电机目标角度（度）')
    parser.add_argument('--duration', type=float, default=8.0, help='运动时间（秒，默认8）')
    args = parser.parse_args()

    targets = {}
    if args.all is not None:
        targets = {0: args.all, 1: args.all, 2: args.all, 3: args.all}
    else:
        if args.L1 is not None: targets[0] = args.L1
        if args.L2 is not None: targets[1] = args.L2
        if args.R1 is not None: targets[2] = args.R1
        if args.R2 is not None: targets[3] = args.R2

    if not targets:
        print("请指定目标角度，例如:")
        print("  python3 smooth_move.py --L1 100 --L2 70 --R1 100 --R2 70")
        print("  python3 smooth_move.py --all 0 --duration 8")
        return

    mover = SmoothMover()
    try:
        mover.move(targets, args.duration)
    finally:
        mover.shutdown()


if __name__ == '__main__':
    main()
