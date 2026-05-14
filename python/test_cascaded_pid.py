#!/usr/bin/env python3
"""
双臂并联机构电机测试脚本 —— 串级 PID + 梯形速度规划

测试流程：
  Phase 1: 大臂电机走到 30°（电机自身角度）→ 等待到位
  Phase 2: 大臂电机和小臂电机同时启动
           大臂: 30° → 105°，小臂: 当前位置 → 75°
           使用梯形速度规划 + 串级PID（位置环→速度环→力矩环）

角度说明：
  所有角度均为电机自身角度（经过 direction/offset 校准后的输出轴角度），
  与运动学 q1/θ2 无关。

串级控制结构：
  外环（位置）: pos_error → PID → 速度指令补偿
  中环（速度）: vel_error → PID → 力矩指令
  内环（力矩）: 由电机 FOC 硬件完成
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State


# ═══════════════════════════════════════════════════════
#  电机配置
# ═══════════════════════════════════════════════════════
MOTOR_L1 = {"id": 0, "device": "/dev/ttyUSB0"}  # 左主臂 q1
MOTOR_L2 = {"id": 1, "device": "/dev/ttyUSB0"}  # 左副臂 θ2
MOTOR_R1 = {"id": 2, "device": "/dev/ttyUSB1"}  # 右主臂 q1
MOTOR_R2 = {"id": 3, "device": "/dev/ttyUSB1"}  # 右副臂 θ2

# 电机内部 PD 增益（与示教节点一致：kp=1.0, kd=0.15）
MOTOR_KP = 1.0
MOTOR_KD = 0.15

# 重力补偿参数（来自 teach_playback_control_node.py 示教节点）
GEAR_RATIO = 6.33
TAU_INNER = 3.1     # 大臂输出端重力矩 (Nm)，M0/M2
TAU_OUTER = 1.5     # 小臂输出端重力矩 (Nm)，M1/M3
GRAVITY_OFFSET = -math.pi / 2  # 零位=下垂
INNER_IDS = {0, 2}  # 大臂电机ID
OUTER_IDS = {1, 3}  # 小臂电机ID

# 控制频率
CONTROL_HZ = 200
CONTROL_DT = 1.0 / CONTROL_HZ


# ═══════════════════════════════════════════════════════
#  梯形速度规划器
# ═══════════════════════════════════════════════════════
class TrapezoidalProfile:
    """梯形速度规划：加速—匀速—减速"""

    def __init__(self, p_start: float, p_end: float,
                 v_max: float, a_max: float):
        """
        Args:
            p_start: 起始位置 (rad)
            p_end:   终止位置 (rad)
            v_max:   最大速度 (rad/s)，正值
            a_max:   最大加速度 (rad/s²)，正值
        """
        self.p_start = p_start
        self.p_end = p_end
        self.direction = 1.0 if p_end >= p_start else -1.0
        self.distance = abs(p_end - p_start)
        self.v_max = v_max
        self.a_max = a_max

        # 检查是否能达到最大速度（三角形 vs 梯形）
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc ** 2

        if 2.0 * d_acc >= self.distance:
            # 三角形：达不到 v_max
            self.t_acc = math.sqrt(self.distance / a_max)
            self.t_cruise = 0.0
            self.t_dec = self.t_acc
            self.v_peak = a_max * self.t_acc
        else:
            # 梯形
            self.t_acc = t_acc
            d_cruise = self.distance - 2.0 * d_acc
            self.t_cruise = d_cruise / v_max
            self.t_dec = t_acc
            self.v_peak = v_max

        self.T = self.t_acc + self.t_cruise + self.t_dec

    def evaluate(self, t: float) -> tuple:
        """
        Returns: (position, velocity, acceleration) at time t
        """
        t = max(0.0, min(t, self.T))
        d = self.direction

        if t <= self.t_acc:
            # 加速段
            a = self.a_max
            v = a * t
            p = self.p_start + d * 0.5 * a * t ** 2
            return p, d * v, d * a

        t2 = t - self.t_acc
        p_end_acc = self.p_start + d * 0.5 * self.a_max * self.t_acc ** 2

        if t2 <= self.t_cruise:
            # 匀速段
            v = self.v_peak
            p = p_end_acc + d * v * t2
            return p, d * v, 0.0

        t3 = t2 - self.t_cruise
        p_end_cruise = p_end_acc + d * self.v_peak * self.t_cruise

        # 减速段
        a = self.a_max
        v = self.v_peak - a * t3
        p = p_end_cruise + d * (self.v_peak * t3 - 0.5 * a * t3 ** 2)
        return p, d * max(v, 0.0), -d * a

    @property
    def duration(self) -> float:
        return self.T


# ═══════════════════════════════════════════════════════
#  串级 PID 控制器
# ═══════════════════════════════════════════════════════
class PIDController:
    """增量式 PID，带积分限幅和输出限幅"""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_max: float, integral_max: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integral_max = integral_max if integral_max > 0 else out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, error: float, dt: float) -> float:
        # 积分
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        # 微分
        if self.first:
            derivative = 0.0
            self.first = False
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_max, min(self.out_max, output))


# ═══════════════════════════════════════════════════════
#  单轴串级控制器（位置环 → 速度环 → 力矩）
# ═══════════════════════════════════════════════════════
class CascadedAxisController:
    """
    外环（位置）→ 速度指令补偿
    中环（速度）→ 力矩指令
    内环 = 电机 FOC 硬件
    """

    def __init__(self, name: str,
                 pos_kp=2.0, pos_ki=0.0, pos_kd=0.0, vel_max=2.0,
                 vel_kp=0.5, vel_ki=0.1, vel_kd=0.01, torque_max=1.0):
        self.name = name
        self.pos_pid = PIDController(pos_kp, pos_ki, pos_kd,
                                     out_max=vel_max, integral_max=vel_max * 0.5)
        self.vel_pid = PIDController(vel_kp, vel_ki, vel_kd,
                                     out_max=torque_max, integral_max=torque_max * 0.5)

    def reset(self):
        self.pos_pid.reset()
        self.vel_pid.reset()

    def compute(self, pos_ref: float, vel_ref: float,
                pos_actual: float, vel_actual: float,
                dt: float) -> tuple:
        """
        Returns: (position_target, velocity_target, torque_ff)
            position_target: 规划位置（给电机内部PD参考）
            velocity_target: 规划速度 + 位置环补偿
            torque_ff:       速度环输出力矩
        """
        # 位置外环：误差 → 速度补偿
        pos_error = pos_ref - pos_actual
        vel_compensation = self.pos_pid.update(pos_error, dt)

        # 速度中环：(规划速度 + 补偿速度) vs 实际速度 → 力矩
        vel_target = vel_ref + vel_compensation
        vel_error = vel_target - vel_actual
        torque_ff = self.vel_pid.update(vel_error, dt)

        return pos_ref, vel_target, torque_ff


# ═══════════════════════════════════════════════════════
#  测试节点
# ═══════════════════════════════════════════════════════
class MotorTestNode(Node):
    def __init__(self):
        super().__init__("motor_test_cascaded")

        self.cmd_pub = self.create_publisher(
            UnitreeGO8010Command, "unitree_go8010_command", 10)

        # 电机状态：{motor_id: {position, velocity, torque, online}}
        self.states = {}
        self._state_lock = threading.Lock()
        self.state_sub = self.create_subscription(
            UnitreeGO8010State, "unitree_go8010_states", self._state_cb, 10)

        self.get_logger().info("MotorTestNode 就绪")

    def _state_cb(self, msg: UnitreeGO8010State):
        with self._state_lock:
            self.states[msg.motor_id] = {
                "position": msg.position,
                "velocity": msg.velocity,
                "torque": msg.torque,
                "online": msg.online,
                "name": msg.joint_name,
            }

    def gravity_torque(self, motor_id: int) -> float:
        """计算指定电机的重力补偿力矩（转子侧 Nm）"""
        with self._state_lock:
            # θ1 = 左右大臂平均位置, θ2 = 左右小臂平均位置
            theta1 = (self.states.get(0, {}).get("position", 0.0)
                      + self.states.get(2, {}).get("position", 0.0)) / 2.0
            theta2 = (self.states.get(1, {}).get("position", 0.0)
                      + self.states.get(3, {}).get("position", 0.0)) / 2.0
        if motor_id in INNER_IDS:
            return TAU_INNER * math.cos(theta1 + GRAVITY_OFFSET) / GEAR_RATIO
        else:
            return -TAU_OUTER * math.cos(theta1 + theta2 + GRAVITY_OFFSET) / GEAR_RATIO

    def send_foc(self, motor: dict, pos: float, vel: float = 0.0,
                 torque_ff: float = 0.0,
                 kp: float = MOTOR_KP, kd: float = MOTOR_KD):
        # 叠加重力补偿
        tau_gravity = self.gravity_torque(motor["id"])
        msg = UnitreeGO8010Command()
        msg.id = motor["id"]
        msg.device = motor["device"]
        msg.mode = 1  # FOC
        msg.position_target = pos
        msg.velocity_target = vel
        msg.torque_ff = torque_ff + tau_gravity
        msg.kp = kp
        msg.kd = kd
        self.cmd_pub.publish(msg)

    def send_brake(self, motor: dict):
        msg = UnitreeGO8010Command()
        msg.id = motor["id"]
        msg.device = motor["device"]
        msg.mode = 0
        self.cmd_pub.publish(msg)

    def brake_all(self):
        for m in [MOTOR_L1, MOTOR_L2, MOTOR_R1, MOTOR_R2]:
            self.send_brake(m)
        self.get_logger().info("全部刹车")

    def get_motor_pos(self, motor_id: int) -> float:
        with self._state_lock:
            s = self.states.get(motor_id)
            return s["position"] if s else 0.0

    def get_motor_vel(self, motor_id: int) -> float:
        with self._state_lock:
            s = self.states.get(motor_id)
            return s["velocity"] if s else 0.0

    def get_motor_torque(self, motor_id: int) -> float:
        with self._state_lock:
            s = self.states.get(motor_id)
            return s["torque"] if s else 0.0

    def is_online(self, motor_id: int) -> bool:
        with self._state_lock:
            s = self.states.get(motor_id)
            return s["online"] if s else False

    def wait_for_motors(self, motor_ids: list, timeout: float = 5.0) -> bool:
        """等待电机上线（状态由后台线程更新）"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if all(self.is_online(mid) for mid in motor_ids):
                return True
            time.sleep(0.05)
        return False

    def print_status(self, label=""):
        """打印所有电机状态"""
        if label:
            print(f"\n── {label} ──")
        with self._state_lock:
            for mid in sorted(self.states.keys()):
                s = self.states[mid]
                online = "✓" if s["online"] else "✗"
                print(f"  ID={mid} [{s['name']}] {online}  "
                      f"pos={math.degrees(s['position']):7.2f}°  "
                      f"vel={s['velocity']:6.2f} rad/s  "
                      f"τ={s['torque']:6.3f} Nm")


def smoothstep(t):
    """smoothstep 插值：0→1 平滑过渡"""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def run_test(node: MotorTestNode):
    """主测试流程 —— smoothstep + 电机内部PD + 重力补偿"""

    # ── 轨迹参数 ────────────────────────────────
    WP1_MAIN = math.radians(30.0)    # 预备位: 大臂 30°
    WP1_SUB  = math.radians(0.0)     # 预备位: 小臂 0°
    WP2_MAIN = math.radians(108.0)   # 终点: 大臂 108°
    WP2_SUB  = math.radians(75.0)    # 终点: 小臂 75°

    SEG1_TIME = 0.25   # 起点 → 预备位（蓄力）
    SEG2_TIME = 0.15   # 预备位 → 击打终点（爆发）
    HOLD_TIME = 2.0    # 终点保持

    # 四电机
    motor_main_L = MOTOR_L1
    motor_sub_L  = MOTOR_L2
    motor_main_R = MOTOR_R1
    motor_sub_R  = MOTOR_R2

    # ── 等待电机上线 ──────────────────────────
    node.get_logger().info("等待电机上线...")
    all_ids = [motor_main_L["id"], motor_sub_L["id"],
               motor_main_R["id"], motor_sub_R["id"]]
    if not node.wait_for_motors(all_ids, timeout=10.0):
        node.get_logger().error("电机未全部上线，退出")
        return
    node.print_status("电机已上线")

    def move_to(targets: list, move_time: float, label: str):
        """Smoothstep 插值运动到目标。targets: [(motor, target_rad), ...]"""
        starts = {m["id"]: node.get_motor_pos(m["id"]) for m, _ in targets}
        node.get_logger().info(
            f"=== {label} ({move_time:.2f}s) ===  " +
            "  ".join(f"ID{m['id']}: {math.degrees(starts[m['id']]):.0f}°→{math.degrees(t):.0f}°"
                      for m, t in targets))
        t0 = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - t0
            frac = smoothstep(elapsed / move_time) if move_time > 0 else 1.0
            for motor, target in targets:
                p0 = starts[motor["id"]]
                node.send_foc(motor, p0 + (target - p0) * frac, 0.0, 0.0)
            if elapsed >= move_time:
                break
            time.sleep(CONTROL_DT)

    def hold_at(targets: list, hold_time: float, label: str):
        """在目标位置保持，持续发送重力补偿"""
        node.get_logger().info(f"保持 {hold_time}s...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < hold_time and rclpy.ok():
            for motor, target in targets:
                node.send_foc(motor, target, 0.0, 0.0)
            time.sleep(CONTROL_DT)

    # ── 连续击打：蓄力 → 爆发（两段 smoothstep 直连，不停顿）──
    wp1_targets = [
        (motor_main_L, WP1_MAIN), (motor_sub_L, WP1_SUB),
        (motor_main_R, WP1_MAIN), (motor_sub_R, WP1_SUB),
    ]
    wp2_targets = [
        (motor_main_L, WP2_MAIN), (motor_sub_L, WP2_SUB),
        (motor_main_R, WP2_MAIN), (motor_sub_R, WP2_SUB),
    ]

    move_to(wp1_targets, SEG1_TIME, "蓄力")
    move_to(wp2_targets, SEG2_TIME, "击打")    # 无间隔直接接上
    hold_at(wp2_targets, HOLD_TIME, "终点保持")

    # ── Phase 3: 速度控制 ────────────────────────
    # kp=0 消除位置环，只用 kd 做速度阻尼跟踪
    # 电机公式: τ = kd*(vel_target - vel_actual) + torque_ff
    VEL_TARGET = 2.5     # 目标速度 (rad/s) ≈ 143.2°/s
    VEL_DURATION = 1.0    # 速度控制持续时间 (s)
    VEL_KD = 0.15         # 速度阻尼系数

    node.get_logger().info(
        f"=== Phase 3: 速度控制 vel={VEL_TARGET:.2f} rad/s ({math.degrees(VEL_TARGET):.1f}°/s) ===")

    all_motors = [motor_main_L, motor_sub_L, motor_main_R, motor_sub_R]
    t0 = time.monotonic()
    last_log = -1.0

    while rclpy.ok():
        elapsed = time.monotonic() - t0
        if elapsed >= VEL_DURATION:
            break

        # 速度斜坡 (0.5s 内从0加速到目标)
        ramp = smoothstep(elapsed / 0.5)
        vel_cmd = VEL_TARGET * ramp

        for motor in all_motors:
            tau_g = node.gravity_torque(motor["id"])
            node.send_foc(motor, 0.0, vel_cmd, tau_g,
                          kp=0.0, kd=VEL_KD)

        if elapsed - last_log >= 1.0:
            last_log = elapsed
            parts = []
            for motor in all_motors:
                vel = node.get_motor_vel(motor["id"])
                pos = node.get_motor_pos(motor["id"])
                parts.append(f"ID{motor['id']}: {math.degrees(pos):.1f}° v={vel:.2f}")
            node.get_logger().info(f"P3(速度) t={elapsed:.1f}s  {' '.join(parts)}")

        time.sleep(CONTROL_DT)

    node.print_status("Phase 3 速度控制完成")

    # 速度控制后刹停，回到位置保持
    node.get_logger().info("减速停止...")
    stop_targets = []
    for motor in all_motors:
        stop_targets.append((motor, node.get_motor_pos(motor["id"])))
    hold_at(stop_targets, 1.0, "减速保持")

    # ── Phase 4: 力矩控制 ────────────────────────
    # kp=0, kd=0, 只发 torque_ff（纯力矩模式）
    # 持续施加一个向下的力矩，验证力矩控制
    TAU_CMD = 1.0           # 目标力矩 (Nm, 转子侧)
    TAU_DURATION =1.0      # 力矩控制持续时间 (s)
    TAU_KD_SAFETY = 0.05    # 微小阻尼防止飞车

    node.get_logger().info(
        f"=== Phase 4: 力矩控制 τ={TAU_CMD:.2f} Nm (转子侧) ===")

    t0 = time.monotonic()
    last_log = -1.0

    while rclpy.ok():
        elapsed = time.monotonic() - t0
        if elapsed >= TAU_DURATION:
            break

        # 力矩斜坡
        ramp = smoothstep(elapsed / 0.5)
        tau_cmd = TAU_CMD * ramp

        for motor in all_motors:
            tau_g = node.gravity_torque(motor["id"])
            # 重力补偿 + 额外指令力矩
            node.send_foc(motor, 0.0, 0.0, tau_g + tau_cmd,
                          kp=0.0, kd=TAU_KD_SAFETY)

        if elapsed - last_log >= 1.0:
            last_log = elapsed
            parts = []
            for motor in all_motors:
                pos = node.get_motor_pos(motor["id"])
                tor = node.get_motor_torque(motor["id"])
                parts.append(f"ID{motor['id']}: {math.degrees(pos):.1f}° τ={tor:.3f}")
            node.get_logger().info(f"P4(力矩) t={elapsed:.1f}s  {' '.join(parts)}")

        time.sleep(CONTROL_DT)

    node.print_status("Phase 4 力矩控制完成")

    # ── 刹车 ────────────────────────────────────
    node.brake_all()
    node.print_status("测试结束")


def main():
    rclpy.init()
    node = MotorTestNode()

    # 后台线程持续处理回调，确保电机状态实时更新
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("""
╔═══════════════════════════════════════════════════╗
║   电机全模式控制测试 + 重力补偿                  ║
║                                                   ║
║   连续击打: 蓄力0.25s → 爆发0.15s (smoothstep)  ║
║   Phase 3: 速度控制 — 2.5 rad/s (纯速度模式)     ║
║   Phase 4: 力矩控制 — 0.3 Nm (纯力矩模式)        ║
║   全程重力补偿: 3.1/1.5 Nm                       ║
║                                                   ║
║   Ctrl+C 紧急刹车                                 ║
╚═══════════════════════════════════════════════════╝
""")

    try:
        run_test(node)
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
        node.brake_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
