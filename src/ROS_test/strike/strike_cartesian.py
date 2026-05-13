#!/usr/bin/env python3
"""
五连杆击球系统 — 笛卡尔空间直线轨迹 + 速度规划

系统简化（姿态固定）:
  左右两臂对称同步 → 退化为 2-DOF 平面串联臂
  M0/M2 = 大臂 θ₁, M1/M3 = 小臂 θ₂
  末端坐标 (x, z) 在肩关节坐标系下

坐标系约定（肩关节为原点）:
  x: 水平向前（正）
  z: 竖直向上（正）
  θ₁: 大臂与水平(+x)的夹角，逆时针为正 ← M0/M2 电机读数
  θ₂: 小臂相对大臂的夹角（相对角），逆时针为正 ← M1/M3 电机读数
  注: 副电机(L2/R2)外壳与大臂固定一体，转子通过连杆驱动小臂
      因此电机输出角直接对应 θ₂（相对角），无需额外换算

运动学:
  FK: x = L₁cos(θ₁) + L₂cos(θ₁+θ₂)
      z = L₁sin(θ₁) + L₂sin(θ₁+θ₂)
  IK: 余弦定理解析解
  雅可比: 末端速度 = J · 关节速度

轨迹规划:
  笛卡尔空间直线插值 + 梯形速度剖面
  到达击球点时末端速度 = v_target（沿运动方向）

阶段:
  1. 重力补偿启动（斜坡）
  2. 匀加速 → 匀速 → 到达击球点（末端速度 = v_target）
  3. 击球后减速停止或保持

用法:
  # 从当前位置直线运动到示教击球点，到达速度 0.5 m/s
  python3 strike_cartesian.py --v-target 0.5

  # 指定击球点关节角度（自动通过FK计算笛卡尔位置）
  python3 strike_cartesian.py --strike-inner 100 --strike-outer 70 --v-target 0.3

  # 指定加速度
  python3 strike_cartesian.py --v-target 0.8 --accel 2.0
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import numpy as np

# ====================== 机构参数 ======================
L1 = 0.20  # 大臂长度 (m) — arm_params.yaml
L2 = 0.22  # 小臂长度 (m) — arm_params.yaml
GEAR_RATIO = 6.33

ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}  # 大臂
OUTER_IDS = {1, 3}  # 小臂
RATE_HZ = 200        # 笛卡尔轨迹需要更高频率
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85


# ====================== 运动学 ======================

def fk(theta1, theta2):
    """正运动学: (θ₁, θ₂) → (x, z)"""
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    z = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, z


def ik(x, z, elbow_sign=1):
    """
    逆运动学: (x, z) → (θ₁, θ₂)
    elbow_sign: +1=肘上(默认, 匹配示教配置), -1=肘下
    返回 (theta1, theta2) 或 None if unreachable
    """
    d_sq = x * x + z * z
    d = math.sqrt(d_sq)

    # 可达性检查
    if d > (L1 + L2) or d < abs(L1 - L2):
        return None

    # θ₂: 余弦定理
    cos_theta2 = (d_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = elbow_sign * math.acos(cos_theta2)

    # θ₁
    phi = math.atan2(z, x)
    beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta1 = phi - beta

    return theta1, theta2


def jacobian(theta1, theta2):
    """
    雅可比矩阵 J: ṗ = J · q̇
    J = [[-L1*s1 - L2*s12,  -L2*s12],
         [ L1*c1 + L2*c12,   L2*c12]]
    """
    s1 = math.sin(theta1)
    c1 = math.cos(theta1)
    s12 = math.sin(theta1 + theta2)
    c12 = math.cos(theta1 + theta2)
    return np.array([
        [-L1 * s1 - L2 * s12, -L2 * s12],
        [ L1 * c1 + L2 * c12,  L2 * c12]
    ])


def jacobian_inv(theta1, theta2):
    """雅可比逆矩阵（用于 ẋ → q̇ 映射）"""
    J = jacobian(theta1, theta2)
    det = np.linalg.det(J)
    if abs(det) < 1e-6:
        return None  # 奇异
    return np.linalg.inv(J)


# ====================== 速度剖面 ======================

class StrikeProfile:
    """
    击球速度剖面（笛卡尔空间沿路径方向）

    策略: 5阶 smoothstep 位置插值，确保平滑启停
      s(t) = s_total * smootherstep(t / t_total)
      v(t) = ds/dt = s_total / t_total * 30*u²*(1-u)²  （u = t/T）
      
    特点:
      - 起始和终止速度均为 0（安全停靠）
      - 加速度连续，无突变（电机平滑）
      - 峰值速度 = 15/8 * s_total/t_total ≈ 1.875 * 平均速度
    """

    def __init__(self, s_total, t_total):
        self.s_total = s_total
        self.t_total = t_total
        self.v_avg = s_total / t_total if t_total > 0 else 0.0
        self.v_peak = 1.875 * self.v_avg  # 5阶 smoothstep 峰值速度
        self.v_end = 0.0  # 到达终点速度为0（安全）
        
    def evaluate(self, t):
        """
        返回 (s, v): 路径位置和速度
        使用 5阶 smootherstep: 6u⁵ - 15u⁴ + 10u³
        导数（速度）: 30u²(1-u)² * s_total/t_total
        """
        if t <= 0:
            return 0.0, 0.0
        
        if t >= self.t_total:
            return self.s_total, 0.0
        
        u = t / self.t_total
        # 5阶 smootherstep
        s = self.s_total * (6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3)
        v = self.s_total / self.t_total * 30.0 * u * u * (1.0 - u) * (1.0 - u)
        return s, v


# ====================== 击球控制器 ======================

class StrikeController:
    def __init__(self, args):
        self.tau_inner = args.tau_inner
        self.tau_outer = args.tau_outer
        self.gravity_offset = args.gravity_offset
        self.kd = args.kd
        self.ramp_time = args.ramp
        self.max_time = args.traj_time
        self.hold_time = args.hold_time
        self.kp_hold = args.kp_hold

        # 击球点（关节空间 → 笛卡尔空间）
        self.strike_theta1 = math.radians(args.strike_inner)
        self.strike_theta2 = math.radians(args.strike_outer)
        self.strike_x, self.strike_z = fk(self.strike_theta1, self.strike_theta2)

        rclpy.init()
        self.node = rclpy.create_node('strike_cartesian')
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
        self.profile = None
        self.start_x = 0.0
        self.start_z = 0.0
        self.direction = np.array([1.0, 0.0])  # 运动方向单位向量

        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.temperatures[msg.motor_id] = msg.temperature
        self.errors[msg.motor_id] = msg.error
        self.last_update[msg.motor_id] = time.time()

    def _sig_handler(self, signum, frame):
        print("\n[STOP] Ctrl+C")
        self._stop = True

    def _send_cmd(self, motor_id, torque_ff, kp, kd, position_target, velocity_target=0.0):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1
        cmd.position_target = position_target
        cmd.velocity_target = velocity_target
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

    def gravity_torque_rotor(self, motor_id, theta1, theta2, ramp=1.0):
        """重力补偿力矩(转子侧). 外臂用绝对角度 θ₁+θ₂"""
        if motor_id in INNER_IDS:
            tau = self.tau_inner
            angle = theta1
        else:
            tau = self.tau_outer
            angle = theta1 + theta2
        return tau * math.cos(angle + self.gravity_offset) / GEAR_RATIO * ramp

    def get_current_joint(self):
        """获取当前关节角度（取对称电机的均值）"""
        p0 = self.positions.get(0, 0.0)
        p2 = self.positions.get(2, 0.0)
        p1 = self.positions.get(1, 0.0)
        p3 = self.positions.get(3, 0.0)
        theta1 = (p0 + p2) / 2.0
        theta2 = (p1 + p3) / 2.0
        return theta1, theta2

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
                print(f"\n[CRITICAL] M{mid} 温度={temp}C")
                return False
            if err in (1, 2, 3, 4):
                return False
            if err != 0:
                print(f"  [WARN] M{mid} error={err}")
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
                    print(f"[WARN] 电机 {missing} 未响应，{len(online)}/4 启动")
                else:
                    print("[OK] 所有电机已上线")
                return True
        return False

    def run(self):
        if not self.wait_for_motors():
            self.shutdown()
            return

        # 当前位置
        theta1_cur, theta2_cur = self.get_current_joint()
        self.start_x, self.start_z = fk(theta1_cur, theta2_cur)

        # 运动方向和距离
        dx = self.strike_x - self.start_x
        dz = self.strike_z - self.start_z
        s_total = math.sqrt(dx * dx + dz * dz)

        if s_total < 0.001:
            print("[INFO] 已在击球点，无需运动")
            self.shutdown()
            return

        self.direction = np.array([dx / s_total, dz / s_total])

        # 创建速度剖面 (5阶 smootherstep，平滑启停)
        traj_time = self.max_time
        self.profile = StrikeProfile(s_total, traj_time)

        print(f"\n{'='*60}")
        print(f"  笛卡尔直线轨迹击球")
        print(f"{'='*60}")
        print(f"  起点: ({self.start_x:.4f}, {self.start_z:.4f}) m")
        print(f"       θ₁={math.degrees(theta1_cur):.1f}° θ₂={math.degrees(theta2_cur):.1f}°")
        print(f"  击球点: ({self.strike_x:.4f}, {self.strike_z:.4f}) m")
        print(f"       θ₁={math.degrees(self.strike_theta1):.1f}° θ₂={math.degrees(self.strike_theta2):.1f}°")
        print(f"  直线距离: {s_total:.4f} m")
        print(f"  方向: ({self.direction[0]:.3f}, {self.direction[1]:.3f})")
        print(f"  目标末端速度: 峰值 {self.profile.v_peak:.3f} m/s (平均 {self.profile.v_avg:.3f} m/s)")
        print(f"  轨迹时间: {self.profile.t_total:.2f}s (smootherstep 平滑启停)")
        print(f"  重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")
        print(f"  斜坡: {self.ramp_time}s | 击球后保持: {self.hold_time}s")
        print(f"  Ctrl+C 安全退出\n")

        self._run_loop()

    def _run_loop(self):
        dt = 1.0 / RATE_HZ
        t_start = time.time()
        phase = 'ramp'  # ramp → trajectory → hold
        t_traj_start = None
        loop_count = 0

        while not self._stop:
            t_now = time.time()
            elapsed = t_now - t_start

            if not self.check_node_alive():
                print("\n[WARN] 节点无响应")
                self._brake_all()
                break

            if loop_count % (RATE_HZ * 5) == 0 and loop_count > 0:
                if not self.check_safety():
                    self._brake_all()
                    break

            rclpy.spin_once(self.node, timeout_sec=0)

            # === 状态机 ===
            if phase == 'ramp':
                ramp_frac = self.smoothstep(min(1.0, elapsed / self.ramp_time)) if self.ramp_time > 0 else 1.0
                # 斜坡期: 重力补偿缓升，位置保持当前
                theta1_r, theta2_r = self.get_current_joint()
                for mid in ALL_IDS:
                    pos = self.positions.get(mid, 0.0)
                    tau_ff = self.gravity_torque_rotor(mid, theta1_r, theta2_r, ramp_frac)
                    self._send_cmd(mid, tau_ff, 0.0, self.kd, pos)

                if elapsed >= self.ramp_time:
                    phase = 'trajectory'
                    t_traj_start = time.time()
                    print(f"  [{elapsed:.1f}s] 斜坡完成 → 开始直线轨迹")

            elif phase == 'trajectory':
                t_traj = time.time() - t_traj_start

                if t_traj >= self.profile.t_total:
                    # 到达击球点
                    phase = 'hold'
                    t_hold_start = time.time()
                    theta1_cur, theta2_cur = self.get_current_joint()
                    x_cur, z_cur = fk(theta1_cur, theta2_cur)
                    v_now = math.sqrt(
                        self.velocities.get(0, 0.0)**2 +
                        self.velocities.get(1, 0.0)**2) if self.positions else 0.0
                    print(f"  [{elapsed:.1f}s] 到达击球点! 位置=({x_cur:.4f},{z_cur:.4f}) 末端速度≈{self.profile.v_end:.3f} m/s")
                    continue

                # 沿直线插值
                s, v_path = self.profile.evaluate(t_traj)
                x_target = self.start_x + self.direction[0] * s
                z_target = self.start_z + self.direction[1] * s

                # 笛卡尔速度
                vx_target = self.direction[0] * v_path
                vz_target = self.direction[1] * v_path

                # IK
                result = ik(x_target, z_target)
                if result is None:
                    print(f"  [ERROR] IK 无解 at ({x_target:.4f}, {z_target:.4f})")
                    self._brake_all()
                    break

                theta1_d, theta2_d = result

                # 雅可比逆: 笛卡尔速度 → 关节速度
                Ji = jacobian_inv(theta1_d, theta2_d)
                if Ji is not None:
                    v_cart = np.array([vx_target, vz_target])
                    qdot = Ji @ v_cart
                    qdot1, qdot2 = qdot[0], qdot[1]
                else:
                    qdot1, qdot2 = 0.0, 0.0

                # 发送命令（4电机）
                for mid in ALL_IDS:
                    pos = self.positions.get(mid, 0.0)
                    tau_ff = self.gravity_torque_rotor(mid, theta1_d, theta2_d)
                    if mid in INNER_IDS:
                        p_des, v_des = theta1_d, qdot1
                    else:
                        p_des, v_des = theta2_d, qdot2
                    self._send_cmd(mid, tau_ff, self.kp_hold, self.kd, p_des, v_des)

            elif phase == 'hold':
                t_hold = time.time() - t_hold_start
                if t_hold >= self.hold_time:
                    print(f"  [{elapsed:.1f}s] 保持时间结束")
                    self._brake_all()
                    break

                # 保持在击球点（重力补偿 + kp 锁定）
                for mid in ALL_IDS:
                    pos = self.positions.get(mid, 0.0)
                    tau_ff = self.gravity_torque_rotor(mid, self.strike_theta1, self.strike_theta2)
                    if mid in INNER_IDS:
                        p_des = self.strike_theta1
                    else:
                        p_des = self.strike_theta2
                    self._send_cmd(mid, tau_ff, self.kp_hold, self.kd, p_des)

            loop_count += 1

            # 状态打印（每秒）
            if loop_count % RATE_HZ == 0:
                self._print_status(elapsed, phase)

            sleep_time = dt - (time.time() - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._print_report()
        self.shutdown()

    def _print_status(self, elapsed, phase):
        theta1, theta2 = self.get_current_joint()
        x, z = fk(theta1, theta2)
        parts = []
        for mid in sorted(self.positions):
            pos = self.positions[mid]
            temp = self.temperatures.get(mid, -1)
            parts.append(f"M{mid}:{math.degrees(pos):+6.1f}° {temp}C")
        print(f"  [{elapsed:5.1f}s] {phase:10s} pos=({x:.4f},{z:.4f}) " +
              f"θ₁={math.degrees(theta1):+.1f}° θ₂={math.degrees(theta2):+.1f}° | {' '.join(parts)}")

    def _print_report(self):
        print(f"\n{'='*60}")
        print(f"  击球运动报告")
        print(f"  击球点: ({self.strike_x:.4f}, {self.strike_z:.4f}) m")
        print(f"  轨迹时间: {self.profile.t_total:.2f}s (smootherstep)")
        print(f"  峰值速度: {self.profile.v_peak:.3f} m/s  平均速度: {self.profile.v_avg:.3f} m/s")
        theta1, theta2 = self.get_current_joint()
        x, z = fk(theta1, theta2)
        print(f"  最终位置: ({x:.4f}, {z:.4f}) m")
        print(f"       θ₁={math.degrees(theta1):.1f}° θ₂={math.degrees(theta2):.1f}°")
        err_x = x - self.strike_x
        err_z = z - self.strike_z
        err = math.sqrt(err_x * err_x + err_z * err_z)
        print(f"  位置误差: {err*1000:.1f} mm")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='笛卡尔空间直线轨迹击球')

    # 击球点参数（通过示教关节角定义）
    parser.add_argument('--strike-inner', type=float, default=100.0,
                        help='击球点大臂角度 deg (默认 100)')
    parser.add_argument('--strike-outer', type=float, default=70.0,
                        help='击球点小臂角度 deg (默认 70)')

    # 速度规划
    parser.add_argument('--traj-time', type=float, default=5.0,
                        help='轨迹执行时间 s (默认 5.0，smoothstep平滑启停)')

    # 重力补偿
    parser.add_argument('--tau-inner', type=float, default=3.1,
                        help='大臂重力矩 Nm (默认 3.1)')
    parser.add_argument('--tau-outer', type=float, default=1.5,
                        help='小臂重力矩 Nm (默认 1.5)')
    parser.add_argument('--gravity-offset', type=float, default=-math.pi/2,
                        help='零位与水平方向的角度差 rad (默认 -π/2, 即零位=竖直向下)')
    parser.add_argument('--kd', type=float, default=0.02,
                        help='转子阻尼 (默认 0.02)')
    parser.add_argument('--kp-hold', type=float, default=0.5,
                        help='轨迹跟踪kp (转子侧, 默认 0.5)')

    # 时间参数
    parser.add_argument('--ramp', type=float, default=1.5,
                        help='重力补偿斜坡时间 s (默认 1.5)')
    parser.add_argument('--hold-time', type=float, default=5.0,
                        help='击球后保持时间 s (默认 5.0)')

    args = parser.parse_args()

    ctrl = StrikeController(args)
    ctrl.run()


if __name__ == '__main__':
    main()
python3 src/ROS_test/strike_joint.py