#!/usr/bin/env python3
"""
五连杆击球系统 — 关节空间五次多项式轨迹规划

借鉴宇树四足跳跃控制策略:
  - 关节空间规划，不调用IK/Jacobian逆（避免奇异位形）
  - 五次多项式轨迹: 6边界条件完全确定，加加速度连续
  - 前馈力矩(重力+惯性) + 关节PD跟踪
  - 终点关节速度由击球末端速度需求反推(仅在击球点算一次Jacobian逆)

坐标系约定:
  θ₁: 大臂与水平(+x)的夹角，逆时针为正 ← M0/M2
  θ₂: 小臂相对大臂的夹角（相对角） ← M1/M3

五次多项式边界条件:
  q(0) = q_start,  q̇(0) = 0,  q̈(0) = 0     ← 静止出发
  q(T) = q_strike,  q̇(T) = q̇_target,  q̈(T) = 0  ← 到达击球点时有速度

阶段:
  1. ramp: 重力补偿斜坡启动
  2. trajectory: 五次多项式轨迹跟踪
  3. hold: 击球后保持/减速

用法:
  python3 strike_joint.py
  python3 strike_joint.py --strike-inner 100 --strike-outer 70 --v-target 0.5
  python3 strike_joint.py --traj-time 3.0 --kp 3.0 --kd 0.15
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import numpy as np


# ====================== 机构参数 ======================
L1 = 0.20   # 大臂长度 (m)
L2 = 0.22   # 小臂长度 (m)
GEAR_RATIO = 6.33

ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}  # 大臂 θ₁
OUTER_IDS = {1, 3}  # 小臂 θ₂
RATE_HZ = 200
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85


# ====================== 运动学(仅用于击球点速度映射) ======================

def fk(theta1, theta2):
    """正运动学: (θ₁, θ₂) → (x, z)"""
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    z = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, z


def jacobian(theta1, theta2):
    """雅可比矩阵 J: ṗ = J · q̇"""
    s1 = math.sin(theta1)
    c1 = math.cos(theta1)
    s12 = math.sin(theta1 + theta2)
    c12 = math.cos(theta1 + theta2)
    return np.array([
        [-L1 * s1 - L2 * s12, -L2 * s12],
        [ L1 * c1 + L2 * c12,  L2 * c12]
    ])


# ====================== 五次多项式轨迹 ======================

class QuinticTrajectory:
    """
    五次多项式关节空间轨迹

    q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵

    边界条件 (每个关节独立):
      q(0) = q0,   q̇(0) = v0,   q̈(0) = 0
      q(T) = qf,   q̇(T) = vf,   q̈(T) = 0
    """

    def __init__(self, q_start, q_end, v_start, v_end, T):
        """
        q_start: [theta1_0, theta2_0]
        q_end:   [theta1_f, theta2_f]
        v_start: [dtheta1_0, dtheta2_0]  (通常 [0, 0])
        v_end:   [dtheta1_f, dtheta2_f]  (击球速度)
        T: 轨迹时间 (s)
        """
        self.T = T
        self.n_joints = len(q_start)
        self.coeffs = []  # 每个关节的 [a0, a1, a2, a3, a4, a5]

        for i in range(self.n_joints):
            A = np.array([
                [1,   0,     0,       0,         0,          0],
                [0,   1,     0,       0,         0,          0],
                [0,   0,     2,       0,         0,          0],
                [1,   T,     T**2,    T**3,      T**4,       T**5],
                [0,   1,     2*T,     3*T**2,    4*T**3,     5*T**4],
                [0,   0,     2,       6*T,       12*T**2,    20*T**3],
            ])
            b = np.array([q_start[i], v_start[i], 0.0,
                          q_end[i],   v_end[i],   0.0])
            c = np.linalg.solve(A, b)
            self.coeffs.append(c)

    def evaluate(self, t):
        """返回 (q, qdot, qddot) 各为 ndarray[n_joints]"""
        t = max(0.0, min(t, self.T))
        q = np.zeros(self.n_joints)
        qdot = np.zeros(self.n_joints)
        qddot = np.zeros(self.n_joints)

        for i, c in enumerate(self.coeffs):
            a0, a1, a2, a3, a4, a5 = c
            q[i] = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
            qdot[i] = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            qddot[i] = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3

        return q, qdot, qddot


# ====================== 简化动力学前馈 ======================

def gravity_torque(theta1, theta2, tau_inner, tau_outer, gravity_offset=-math.pi/2):
    """
    重力补偿力矩 (输出轴侧)
    大臂: tau_inner * cos(θ₁ + gravity_offset)
    小臂: tau_outer * cos(θ₁ + θ₂ + gravity_offset)  ← 注意用绝对角度θ₁+θ₂
    gravity_offset: 零位与水平方向的角度差 (默认 -π/2, 即零位=竖直向下)
    返回 [tau1, tau2] (输出轴侧 Nm)
    """
    tau1 = tau_inner * math.cos(theta1 + gravity_offset)
    tau2 = tau_outer * math.cos(theta1 + theta2 + gravity_offset)
    return np.array([tau1, tau2])


def inertia_torque(theta1, theta2, qddot, m1=0.5, m2=0.3):
    """
    简化惯性力矩前馈 (输出轴侧), 忽略科氏力/离心力
    τ_inertia ≈ M(q) * q̈
    M 为简化惯量矩阵 （均匀杆假设）:
      M11 = (m1*lc1² + I1) + m2*(L1² + lc2² + 2*L1*lc2*cos(θ2)) + I2
      M12 = m2*(lc2² + L1*lc2*cos(θ2)) + I2
      M22 = m2*lc2² + I2
    lc = L/2 (质心在杆中点), I = mL²/12
    """
    lc1 = L1 / 2.0
    lc2 = L2 / 2.0
    I1 = m1 * L1**2 / 12.0
    I2 = m2 * L2**2 / 12.0
    c2 = math.cos(theta2)

    M11 = m1 * lc1**2 + I1 + m2 * (L1**2 + lc2**2 + 2 * L1 * lc2 * c2) + I2
    M12 = m2 * (lc2**2 + L1 * lc2 * c2) + I2
    M22 = m2 * lc2**2 + I2

    tau1 = M11 * qddot[0] + M12 * qddot[1]
    tau2 = M12 * qddot[0] + M22 * qddot[1]
    return np.array([tau1, tau2])


# ====================== 击球控制器 ======================

class StrikeController:
    def __init__(self, args):
        self.tau_inner = args.tau_inner
        self.tau_outer = args.tau_outer
        self.gravity_offset = args.gravity_offset
        self.kp = args.kp
        self.kd = args.kd
        self.kp_hold = args.kp_hold
        self.kd_hold = args.kd_hold
        self.ramp_time = args.ramp
        self.traj_time = args.traj_time
        self.hold_time = args.hold_time
        self.v_target = args.v_target
        self.strike_dir = np.array([
            math.cos(math.radians(args.strike_direction)),
            math.sin(math.radians(args.strike_direction))
        ])
        self.m1 = args.m1
        self.m2 = args.m2

        # 击球点 (关节空间)
        self.strike_theta1 = math.radians(args.strike_inner)
        self.strike_theta2 = math.radians(args.strike_outer)
        self.strike_x, self.strike_z = fk(self.strike_theta1, self.strike_theta2)

        # 计算击球点的关节速度需求
        v_cart = self.v_target * self.strike_dir  # [vx, vz]
        J = jacobian(self.strike_theta1, self.strike_theta2)
        det = np.linalg.det(J)
        if abs(det) < 1e-6:
            print("[ERROR] 击球点处 Jacobian 奇异!")
            self.qdot_strike = np.array([0.0, 0.0])
        else:
            self.qdot_strike = np.linalg.solve(J, v_cart)

        rclpy.init()
        self.node = rclpy.create_node('strike_joint')
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
        self.traj = None

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
        """发送电机命令 — 所有参数均为转子侧"""
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

    def get_current_joint(self):
        """获取当前关节角度 (取对称电机均值)"""
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

        # 创建五次多项式轨迹
        q_start = np.array([theta1_cur, theta2_cur])
        q_end = np.array([self.strike_theta1, self.strike_theta2])
        v_start = np.array([0.0, 0.0])
        v_end = self.qdot_strike

        self.traj = QuinticTrajectory(q_start, q_end, v_start, v_end, self.traj_time)

        # 计算轨迹峰值速度和加速度（用于安全检查和显示）
        n_samples = 100
        v_max = np.zeros(2)
        a_max = np.zeros(2)
        for k in range(n_samples + 1):
            t = k * self.traj_time / n_samples
            _, qd, qdd = self.traj.evaluate(t)
            v_max = np.maximum(v_max, np.abs(qd))
            a_max = np.maximum(a_max, np.abs(qdd))

        x_start, z_start = fk(theta1_cur, theta2_cur)

        print(f"\n{'='*60}")
        print(f"  关节空间五次多项式击球")
        print(f"{'='*60}")
        print(f"  起点: θ₁={math.degrees(theta1_cur):+.1f}° θ₂={math.degrees(theta2_cur):+.1f}°")
        print(f"        FK=({x_start:.4f}, {z_start:.4f}) m")
        print(f"  击球点: θ₁={math.degrees(self.strike_theta1):.1f}° θ₂={math.degrees(self.strike_theta2):.1f}°")
        print(f"          FK=({self.strike_x:.4f}, {self.strike_z:.4f}) m")
        print(f"  末端速度目标: {self.v_target:.2f} m/s 方向={math.degrees(math.atan2(self.strike_dir[1], self.strike_dir[0])):.0f}°")
        print(f"  → 关节速度需求: θ̇₁={math.degrees(self.qdot_strike[0]):.1f}°/s θ̇₂={math.degrees(self.qdot_strike[1]):.1f}°/s")
        print(f"  轨迹时间: {self.traj_time:.1f}s")
        print(f"  峰值关节速度: θ̇₁={math.degrees(v_max[0]):.1f}°/s θ̇₂={math.degrees(v_max[1]):.1f}°/s")
        print(f"  峰值关节加速度: θ̈₁={math.degrees(a_max[0]):.0f}°/s² θ̈₂={math.degrees(a_max[1]):.0f}°/s²")
        print(f"  控制参数: kp={self.kp} kd={self.kd} (轨迹) | kp={self.kp_hold} kd={self.kd_hold} (保持)")
        print(f"  重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")
        print(f"  惯性前馈: m1={self.m1}kg m2={self.m2}kg")
        print(f"  斜坡: {self.ramp_time}s | 保持: {self.hold_time}s")
        print(f"  Ctrl+C 安全退出\n")

        self._run_loop()

    def _run_loop(self):
        dt = 1.0 / RATE_HZ
        t_start = time.time()
        phase = 'ramp'
        t_traj_start = None
        t_hold_start = None
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

            theta1_cur, theta2_cur = self.get_current_joint()

            # === 状态机 ===
            if phase == 'ramp':
                ramp_frac = self.smoothstep(min(1.0, elapsed / self.ramp_time)) if self.ramp_time > 0 else 1.0

                # 斜坡期: 重力补偿缓升, 位置保持当前
                g_tau = gravity_torque(theta1_cur, theta2_cur, self.tau_inner, self.tau_outer, self.gravity_offset)
                for mid in ALL_IDS:
                    pos = self.positions.get(mid, 0.0)
                    idx = 0 if mid in INNER_IDS else 1
                    tau_ff = g_tau[idx] / GEAR_RATIO * ramp_frac
                    self._send_cmd(mid, tau_ff, 0.0, self.kd, pos)

                if elapsed >= self.ramp_time:
                    phase = 'trajectory'
                    t_traj_start = time.time()
                    print(f"  [{elapsed:.1f}s] 斜坡完成 → 开始轨迹跟踪")

            elif phase == 'trajectory':
                t_traj = time.time() - t_traj_start

                if t_traj >= self.traj.T:
                    # 到达击球点 → 直接进入 hold，本帧继续发命令
                    phase = 'hold'
                    t_hold_start = time.time()
                    x_cur, z_cur = fk(theta1_cur, theta2_cur)
                    print(f"  [{elapsed:.1f}s] 到达击球点! θ₁={math.degrees(theta1_cur):+.1f}° θ₂={math.degrees(theta2_cur):+.1f}° FK=({x_cur:.4f},{z_cur:.4f})")
                    # 不 continue, 让 hold 立即发命令

                if phase == 'trajectory':
                    # 轨迹插值
                    q_d, qdot_d, qddot_d = self.traj.evaluate(t_traj)

                    # 前馈力矩: 重力 + 惯性
                    g_tau = gravity_torque(q_d[0], q_d[1], self.tau_inner, self.tau_outer, self.gravity_offset)
                    i_tau = inertia_torque(q_d[0], q_d[1], qddot_d, self.m1, self.m2)
                    tau_ff_joint = g_tau + i_tau  # 输出轴侧 Nm

                    for mid in ALL_IDS:
                        idx = 0 if mid in INNER_IDS else 1
                        tau_ff_rotor = tau_ff_joint[idx] / GEAR_RATIO
                        self._send_cmd(mid, tau_ff_rotor, self.kp, self.kd,
                                       q_d[idx], qdot_d[idx])

                # fall through to hold if phase just changed
                if phase == 'hold':
                    g_tau = gravity_torque(self.strike_theta1, self.strike_theta2,
                                          self.tau_inner, self.tau_outer, self.gravity_offset)
                    for mid in ALL_IDS:
                        idx = 0 if mid in INNER_IDS else 1
                        tau_ff_rotor = g_tau[idx] / GEAR_RATIO
                        p_des = self.strike_theta1 if mid in INNER_IDS else self.strike_theta2
                        self._send_cmd(mid, tau_ff_rotor, self.kp_hold, self.kd_hold, p_des)

            elif phase == 'hold':
                t_hold = time.time() - t_hold_start
                if t_hold >= self.hold_time:
                    print(f"  [{elapsed:.1f}s] 保持时间结束")
                    self._brake_all()
                    break

                g_tau = gravity_torque(self.strike_theta1, self.strike_theta2,
                                       self.tau_inner, self.tau_outer, self.gravity_offset)
                for mid in ALL_IDS:
                    idx = 0 if mid in INNER_IDS else 1
                    tau_ff_rotor = g_tau[idx] / GEAR_RATIO
                    p_des = self.strike_theta1 if mid in INNER_IDS else self.strike_theta2
                    self._send_cmd(mid, tau_ff_rotor, self.kp_hold, self.kd_hold, p_des)

            loop_count += 1

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
            vel = self.velocities.get(mid, 0.0)
            temp = self.temperatures.get(mid, -1)
            parts.append(f"M{mid}:{math.degrees(pos):+6.1f}° {temp}C")
        print(f"  [{elapsed:5.1f}s] {phase:10s} θ₁={math.degrees(theta1):+.1f}° θ₂={math.degrees(theta2):+.1f}° " +
              f"FK=({x:.4f},{z:.4f}) | {' '.join(parts)}")

    def _print_report(self):
        theta1, theta2 = self.get_current_joint()
        x, z = fk(theta1, theta2)
        err_t1 = math.degrees(theta1 - self.strike_theta1)
        err_t2 = math.degrees(theta2 - self.strike_theta2)
        err_x = x - self.strike_x
        err_z = z - self.strike_z
        err_pos = math.sqrt(err_x**2 + err_z**2)

        print(f"\n{'='*60}")
        print(f"  击球运动报告")
        print(f"  击球点: θ₁={math.degrees(self.strike_theta1):.1f}° θ₂={math.degrees(self.strike_theta2):.1f}°")
        print(f"          FK=({self.strike_x:.4f}, {self.strike_z:.4f}) m")
        print(f"  最终位置: θ₁={math.degrees(theta1):+.1f}° θ₂={math.degrees(theta2):+.1f}°")
        print(f"            FK=({x:.4f}, {z:.4f}) m")
        print(f"  关节偏差: Δθ₁={err_t1:+.2f}° Δθ₂={err_t2:+.2f}°")
        print(f"  末端距离偏差: {err_pos*1000:.1f} mm")
        print(f"  目标末端速度: {self.v_target:.2f} m/s 方向={math.degrees(math.atan2(self.strike_dir[1], self.strike_dir[0])):.0f}°")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='关节空间五次多项式击球')

    # 击球点
    parser.add_argument('--strike-inner', type=float, default=100.0,
                        help='击球点大臂角度 deg (默认 100)')
    parser.add_argument('--strike-outer', type=float, default=70.0,
                        help='击球点小臂角度 deg (默认 70)')

    # 末端速度目标
    parser.add_argument('--v-target', type=float, default=0.3,
                        help='击球点目标末端速度 m/s (默认 0.3)')
    parser.add_argument('--strike-direction', type=float, default=90.0,
                        help='击球方向 deg, 0=水平正方向, 90=垂直向上 (默认 90)')

    # 轨迹参数
    parser.add_argument('--traj-time', type=float, default=5.0,
                        help='轨迹执行时间 s (默认 5.0)')

    # PD 参数 (转子侧)
    parser.add_argument('--kp', type=float, default=3.0,
                        help='轨迹跟踪 kp (转子侧, 默认 3.0, ≈关节侧120 Nm/rad)')
    parser.add_argument('--kd', type=float, default=0.15,
                        help='轨迹跟踪 kd (转子侧, 默认 0.15)')
    parser.add_argument('--kp-hold', type=float, default=2.0,
                        help='保持阶段 kp (转子侧, 默认 2.0)')
    parser.add_argument('--kd-hold', type=float, default=0.3,
                        help='保持阶段 kd (转子侧, 默认 0.3, 高阻尼减速)')

    # 重力补偿
    parser.add_argument('--tau-inner', type=float, default=3.1,
                        help='大臂重力矩 Nm (默认 3.1)')
    parser.add_argument('--tau-outer', type=float, default=1.5,
                        help='小臂重力矩 Nm (默认 1.5)')
    parser.add_argument('--gravity-offset', type=float, default=-math.pi/2,
                        help='零位与水平方向的角度差 rad (默认 -π/2, 即零位=竖直向下)')

    # 惯性前馈质量参数
    parser.add_argument('--m1', type=float, default=0.5,
                        help='大臂等效质量 kg (默认 0.5)')
    parser.add_argument('--m2', type=float, default=0.3,
                        help='小臂+击球板等效质量 kg (默认 0.3)')

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
