#!/usr/bin/env python3
"""
击球控制脚本 —— 手柄 LB 触发两阶段击打

GO-M8010-6 FOC 控制公式:
  τ = torque_ff + kp * (pos_target - pos) + kd * (vel_target - vel)
  三项同时生效，每个电机每个阶段可独立设置全部5个参数

流程：
  1. 按 LB / Enter → 蓄力位置 (大臂 20°, 小臂 0°)
  2. 再按 LB / Enter → 击打 (大臂 100°, 小臂 80°)
  3. 循环，Ctrl+C 刹车退出

参数调节：修改 READY_CMD / STRIKE_CMD 字典中的 pos/vel/torque_ff/kp/kd

注意：可与底盘 joystick_control_node 同时运行，LB（button[4]）不冲突。
  底盘使用: 摇杆(axes 0,1,3) + A/B/X/Y/Start 按钮
"""

import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State

# 手柄按钮索引（Xbox 标准映射）
JOY_BUTTON_LB = 4

# ═══════════════════════════════════════════════════════
# 电机配置
# ═══════════════════════════════════════════════════════
MOTOR_L1 = {"id": 0, "device": "/dev/ttyUSB0"}  # 左大臂
MOTOR_L2 = {"id": 1, "device": "/dev/ttyUSB0"}  # 左小臂
MOTOR_R1 = {"id": 2, "device": "/dev/ttyUSB1"}  # 右大臂
MOTOR_R2 = {"id": 3, "device": "/dev/ttyUSB1"}  # 右小臂

INNER_IDS = {0, 2}  # 大臂电机
OUTER_IDS = {1, 3}  # 小臂电机

CONTROL_HZ = 200
CONTROL_DT = 1.0 / CONTROL_HZ

# 重力补偿
TAU_INNER = 8.0    # 大臂输出端重力矩 Nm
TAU_OUTER = 3.0    # 小臂输出端重力矩 Nm
GRAVITY_OFFSET = -math.pi / 2
GEAR_RATIO = 6.33

# ═══════════════════════════════════════════════════════
# 击打参数 — 每个阶段的完整 FOC 指令
# τ = torque_ff + kp*(pos_target - pos) + kd*(vel_target - vel)
# 重力补偿自动叠加到 torque_ff 上，无需手动计算
# ═══════════════════════════════════════════════════════

# ── 蓄力阶段 ──
READY_CMD = {
    "main": {  # 大臂 (ID 0, 2)
        "pos": math.radians(30.0),   # 目标位置 rad
        "vel": 0.0,                  # 目标速度 rad/s
        "torque_ff": 0.0,            # 前馈力矩 Nm（转子侧，重力补偿另算）
        "kp": 0.5,                # 位置增益
        "kd": 0.1,                  # 速度增益/阻尼
    },
    "sub": {   # 小臂 (ID 1, 3)
        "pos": math.radians(0.0),
        "vel": 0.0,
        "torque_ff": 0.0,
        "kp": 0.5,
        "kd": 0.1,
    },
}

# ── 击打阶段（速度模式 + 位置监督）──
# kp=0 关闭位置环，纯速度/力矩驱动，全力加速
# 位置监督：大臂到达 100° 时记录速度并切换刹停
STRIKE_CMD = {
    "main": {  # 大臂
        "pos": 0.0,                  # kp=0 时无效
        "vel": 9.0,                 # 目标速度 rad/s (≈860°/s)
        "torque_ff": 0.5,            # 额外加速力矩（重力补偿另算）
        "kp": 0.0,                   # 无位置环 → 纯速度驱动
        "kd": 0.1,                  # 速度跟踪阻尼
    },
    "sub": {   # 小臂
        "pos": 0.0,
        "vel": 7.5,
        "torque_ff": 0.5,
        "kp": 0.0,
        "kd": 0.1,
    },
}

# ── 击打后刹停 ──
# 到达击球位后: 纯刹车(kp=0), 减速到0即可, 停在哪无所谓
# 击球位: 大臂100° 小臂80°  硬限位: 大臂120° 小臂100°
STRIKE_HOLD_CMD = {
    "main": {
        "pos": 0.0,                  # kp=0 所以 pos 无意义
        "vel": 0.0,                  # 目标速度=0, kd产生制动力
        "torque_ff": 0.0,
        "kp": 0.0,                   # 不做位置控制
        "kd": 0.1,                   # 纯阻尼刹车
    },
    "sub": {
        "pos": 0.0,
        "vel": 0.0,
        "torque_ff": 0.0,
        "kp": 0.0,
        "kd": 0.1,
    },
}

STRIKE_TRIGGER = math.radians(110.0)   # 大臂到达击球位后切换刹停（唯一触发条件）
STRIKE_TIMEOUT = 3.0                  # 击打最大时间保护 (s)
HOLD_DURATION = 1.0                   # 击打后保持 (s)

# ── 缓慢回蓄力 ──
RETURN_TIME = 1.5      # 回蓄力位的插值时间 (s)


class StrikeNode(Node):
    def __init__(self):
        super().__init__("strike_test")

        self.cmd_pub = self.create_publisher(
            UnitreeGO8010Command, "unitree_go8010_command", 10)

        self.states = {}
        self._state_lock = threading.Lock()
        self.create_subscription(
            UnitreeGO8010State, "unitree_go8010_states", self._state_cb, 10)

        # 手柄 LB 按钮事件
        self._lb_event = threading.Event()
        self._lb_prev = 0
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        self.get_logger().info("StrikeNode 就绪 (LB 或 Enter 触发)")

        # 键盘监听线程 (按 Enter 触发与 LB 相同事件)
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()

    def _keyboard_listener(self):
        """监听键盘 Enter 键，触发与 LB 相同的事件"""
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if line is not None:
                    self._lb_event.set()
            except Exception:
                break

    def _joy_cb(self, msg: Joy):
        """检测 LB 按钮上升沿（按下瞬间触发一次）"""
        if len(msg.buttons) > JOY_BUTTON_LB:
            current = msg.buttons[JOY_BUTTON_LB]
            if current and not self._lb_prev:
                self._lb_event.set()
            self._lb_prev = current

    def wait_lb(self, prompt: str = ""):
        """阻塞等待 LB 按下或键盘 Enter，替代 input()"""
        if prompt:
            print(prompt)
        self._lb_event.clear()
        while rclpy.ok():
            if self._lb_event.wait(timeout=0.1):
                return True
        return False

    def _state_cb(self, msg: UnitreeGO8010State):
        with self._state_lock:
            self.states[msg.motor_id] = {
                "position": msg.position,
                "velocity": msg.velocity,
                "torque": msg.torque,
                "online": msg.online,
                "name": msg.joint_name,
            }

    # ── 重力补偿 ──────────────────────────────────
    def gravity_torque(self, motor_id: int) -> float:
        with self._state_lock:
            theta1 = (self.states.get(0, {}).get("position", 0.0)
                      + self.states.get(2, {}).get("position", 0.0)) / 2.0
            theta2 = (self.states.get(1, {}).get("position", 0.0)
                      + self.states.get(3, {}).get("position", 0.0)) / 2.0
        if motor_id in INNER_IDS:
            return TAU_INNER * math.cos(theta1 + GRAVITY_OFFSET) / GEAR_RATIO
        else:
            return -TAU_OUTER * math.cos(theta1 + theta2 + GRAVITY_OFFSET) / GEAR_RATIO

    # ── FOC 发送（自动叠加重力补偿）──────────────
    def send_foc(self, motor: dict, pos: float, vel: float,
                 torque_ff: float, kp: float, kd: float):
        """发送完整 FOC 指令，torque_ff 会自动叠加重力补偿"""
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

    # ── 状态查询 ──────────────────────────────────
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

    def wait_for_motors(self, motor_ids: list, timeout: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if all(self.is_online(mid) for mid in motor_ids):
                return True
            time.sleep(0.05)
        return False

    def print_status(self, label=""):
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

    # ── 发送阶段指令 ──────────────────────────────
    def send_phase(self, cmd: dict, label: str):
        """根据 cmd 字典发送 FOC 指令到四个电机
        cmd = {"main": {pos,vel,torque_ff,kp,kd}, "sub": {pos,vel,torque_ff,kp,kd}}
        """
        main = cmd["main"]
        sub = cmd["sub"]
        self.get_logger().info(
            f"=== {label} ===  "
            f"大臂: pos={math.degrees(main['pos']):.1f}° vel={main['vel']:.2f} τ_ff={main['torque_ff']:.2f} "
            f"kp={main['kp']:.2f} kd={main['kd']:.2f}  |  "
            f"小臂: pos={math.degrees(sub['pos']):.1f}° vel={sub['vel']:.2f} τ_ff={sub['torque_ff']:.2f} "
            f"kp={sub['kp']:.2f} kd={sub['kd']:.2f}")
        # 左大臂 + 右大臂
        for motor in [MOTOR_L1, MOTOR_R1]:
            self.send_foc(motor, main["pos"], main["vel"],
                          main["torque_ff"], main["kp"], main["kd"])
        # 左小臂 + 右小臂
        for motor in [MOTOR_L2, MOTOR_R2]:
            self.send_foc(motor, sub["pos"], sub["vel"],
                          sub["torque_ff"], sub["kp"], sub["kd"])

    def hold_phase(self, cmd: dict, hold_time: float):
        """持续发送指令保持"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < hold_time and rclpy.ok():
            main = cmd["main"]
            sub = cmd["sub"]
            for motor in [MOTOR_L1, MOTOR_R1]:
                self.send_foc(motor, main["pos"], main["vel"],
                              main["torque_ff"], main["kp"], main["kd"])
            for motor in [MOTOR_L2, MOTOR_R2]:
                self.send_foc(motor, sub["pos"], sub["vel"],
                              sub["torque_ff"], sub["kp"], sub["kd"])
            time.sleep(CONTROL_DT)

    def slow_move_to(self, target_main: float, target_sub: float,
                     move_time: float, label: str,
                     kp: float = 0.5, kd: float = 0.15):
        """smoothstep 缓慢插值到目标位置（回蓄力用）"""
        start_main = (self.get_motor_pos(0) + self.get_motor_pos(2)) / 2.0
        start_sub = (self.get_motor_pos(1) + self.get_motor_pos(3)) / 2.0
        self.get_logger().info(
            f"=== {label} ({move_time:.1f}s) ===  "
            f"大臂: {math.degrees(start_main):.1f}°→{math.degrees(target_main):.1f}°  "
            f"小臂: {math.degrees(start_sub):.1f}°→{math.degrees(target_sub):.1f}°")
        t0 = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - t0
            t = min(1.0, elapsed / move_time) if move_time > 0 else 1.0
            frac = t * t * (3.0 - 2.0 * t)  # smoothstep
            pos_m = start_main + (target_main - start_main) * frac
            pos_s = start_sub + (target_sub - start_sub) * frac
            for motor in [MOTOR_L1, MOTOR_R1]:
                self.send_foc(motor, pos_m, 0.0, 0.0, kp, kd)
            for motor in [MOTOR_L2, MOTOR_R2]:
                self.send_foc(motor, pos_s, 0.0, 0.0, kp, kd)
            if elapsed >= move_time:
                break
            time.sleep(CONTROL_DT)


def run_strike(node: StrikeNode):
    """主击球流程"""

    all_ids = [MOTOR_L1["id"], MOTOR_L2["id"], MOTOR_R1["id"], MOTOR_R2["id"]]

    node.get_logger().info("等待电机上线...")
    if not node.wait_for_motors(all_ids):
        node.get_logger().error("电机未全部上线，退出")
        return
    node.print_status("电机已上线")

    while rclpy.ok():
        # ── 蓄力（缓慢移动到位）─────────────────
        node.wait_lb("\n按 LB / Enter → 蓄力 (大臂 20°, 小臂 0°)...")
        node.slow_move_to(READY_CMD["main"]["pos"], READY_CMD["sub"]["pos"],
                          RETURN_TIME, "缓慢蓄力",
                          READY_CMD["main"]["kp"], READY_CMD["main"]["kd"])
        node.send_phase(READY_CMD, "蓄力保持")
        node.hold_phase(READY_CMD, 0.5)
        node.print_status("蓄力完成")

        # ── 击打（速度模式 + 位置监督）────────────
        node.wait_lb("\n按 LB / Enter → 击打!...")
        node.send_phase(STRIKE_CMD, "击打-加速")
        t0 = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - t0
            main_pos = (node.get_motor_pos(0) + node.get_motor_pos(2)) / 2.0
            sub_pos = (node.get_motor_pos(1) + node.get_motor_pos(3)) / 2.0
            if main_pos >= STRIKE_TRIGGER:
                main_vel = (node.get_motor_vel(0) + node.get_motor_vel(2)) / 2.0
                node.get_logger().info(
                    f"到达击球位: 大臂={math.degrees(main_pos):.1f}° 小臂={math.degrees(sub_pos):.1f}° "
                    f"vel={main_vel:.1f} rad/s ({math.degrees(main_vel):.0f}°/s)  "
                    f"t={elapsed:.3f}s")
                break
            if elapsed >= STRIKE_TIMEOUT:
                node.get_logger().warn(f"击打超时 {STRIKE_TIMEOUT}s")
                break
            node.hold_phase(STRIKE_CMD, CONTROL_DT)
        # 切换刹停保持
        node.send_phase(STRIKE_HOLD_CMD, "刹停")
        node.hold_phase(STRIKE_HOLD_CMD, HOLD_DURATION)
        node.print_status("击打完成")

        print("\n── 循环: 再次按 LB 或 Enter 缓慢回蓄力, Ctrl+C 退出 ──")


def main():
    rclpy.init()
    node = StrikeNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    r = READY_CMD
    s = STRIKE_CMD
    print(f"""
╔═══════════════════════════════════════════════════════════╗
║  击球控制脚本 — 速度模式 + 位置监督                       ║
║                                                           ║
║  FOC: τ = torque_ff + kp*(pos-cur) + kd*(vel-cur_vel)    ║
║  重力补偿自动叠加 ({TAU_INNER}/{TAU_OUTER} Nm)                       ║
║                                                           ║
║  蓄力: 大臂 {math.degrees(r['main']['pos']):5.1f}°  小臂 {math.degrees(r['sub']['pos']):5.1f}°               ║
║  击打: kp=0 vel={s['main']['vel']:.0f} rad/s τ_ff={s['main']['torque_ff']:.1f}Nm  ║
║  刹车位: 大臂≥{math.degrees(STRIKE_TRIGGER):.0f}°（唯一触发条件）            ║
║  硬限位: 大臂120° 小臂100°(安全余量)                     ║
║  回蓄力: smoothstep {RETURN_TIME:.1f}s                                ║
║                                                           ║
║  LB / Enter → 蓄力 → LB / Enter → 击打 → 循环          ║
║  Ctrl+C 紧急刹车                                         ║
║  可与底盘手柄同时使用(LB不冲突)                          ║
╚═══════════════════════════════════════════════════════════╝
""")

    try:
        run_strike(node)
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
    finally:
        node.brake_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
