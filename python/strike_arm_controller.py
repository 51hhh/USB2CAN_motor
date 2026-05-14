#!/usr/bin/env python3
"""
双臂平行连杆机械臂控制脚本
通过 ROS2 话题发送 UnitreeGO8010Command 控制四个 GO-M8010-6 电机

电机映射：
  ID=0  strike_motor_L1  /dev/ttyUSB0  左主臂 (q1_L)
  ID=1  strike_motor_L2  /dev/ttyUSB0  左副臂 (θ2_L)
  ID=2  strike_motor_R1  /dev/ttyUSB1  右主臂 (q1_R)
  ID=3  strike_motor_R2  /dev/ttyUSB1  右副臂 (θ2_R)

角度约定 (ParallelArmKinematics)：
  q1:  从 -Y 轴（竖直向下）顺时针为正, 范围 0~110°
  θ2:  从 -Y 轴在大臂局部坐标系中逆时针为正, 范围 55~135°（四连杆可达）

position_target 单位：弧度（msg注释写"度"但代码以弧度解释）

用法：
  1. 先启动 motor_control_node：
     ros2 run motor_control_ros2 motor_control_node --ros-args --params-file src/motor_control_ros2/config/motors.yaml

  2. 运行本脚本：
     python3 python/strike_arm_controller.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State


# ── 电机配置 ──────────────────────────────────────────────
MOTOR_L1 = {"id": 0, "device": "/dev/ttyUSB0", "name": "strike_motor_L1"}  # 左 q1
MOTOR_L2 = {"id": 1, "device": "/dev/ttyUSB0", "name": "strike_motor_L2"}  # 左 θ2
MOTOR_R1 = {"id": 2, "device": "/dev/ttyUSB1", "name": "strike_motor_R1"}  # 右 q1
MOTOR_R2 = {"id": 3, "device": "/dev/ttyUSB1", "name": "strike_motor_R2"}  # 右 θ2

# 默认 PD 增益
DEFAULT_KP = 0.01
DEFAULT_KD = 0.01

# 安全限位（弧度）
Q1_MIN = math.radians(0)
Q1_MAX = math.radians(110)
THETA2_MIN = math.radians(55)
THETA2_MAX = math.radians(135)


class StrikeArmController(Node):
    def __init__(self):
        super().__init__("strike_arm_controller")

        # 发布器
        self.cmd_pub = self.create_publisher(
            UnitreeGO8010Command, "unitree_go8010_command", 10
        )

        # 订阅状态
        self.motor_states = {}
        self.state_sub = self.create_subscription(
            UnitreeGO8010State, "unitree_go8010_states", self._state_cb, 10
        )

        self.get_logger().info("StrikeArmController 就绪，等待电机状态...")

    def _state_cb(self, msg: UnitreeGO8010State):
        self.motor_states[msg.joint_name] = {
            "online": msg.online,
            "position": msg.position,
            "velocity": msg.velocity,
            "torque": msg.torque,
            "temp": msg.temperature,
        }

    # ── 基础指令 ─────────────────────────────────────────
    def send_foc(self, motor_cfg: dict, position_rad: float,
                 kp: float = DEFAULT_KP, kd: float = DEFAULT_KD,
                 vel: float = 0.0, torque_ff: float = 0.0):
        """发送单个电机 FOC 位置命令"""
        msg = UnitreeGO8010Command()
        msg.id = motor_cfg["id"]
        msg.device = motor_cfg["device"]
        msg.mode = 1  # FOC
        msg.position_target = position_rad
        msg.velocity_target = vel
        msg.torque_ff = torque_ff
        msg.kp = kp
        msg.kd = kd
        self.cmd_pub.publish(msg)

    def send_brake(self, motor_cfg: dict):
        """刹车"""
        msg = UnitreeGO8010Command()
        msg.id = motor_cfg["id"]
        msg.device = motor_cfg["device"]
        msg.mode = 0  # BRAKE
        self.cmd_pub.publish(msg)

    def brake_all(self):
        """全部刹车"""
        for m in [MOTOR_L1, MOTOR_L2, MOTOR_R1, MOTOR_R2]:
            self.send_brake(m)
        self.get_logger().info("全部电机已刹车")

    # ── 四连杆正解 (θ2 → q2) ────────────────────────────
    @staticmethod
    def four_bar_forward(theta2: float,
                         r: float = 0.030,
                         l: float = 0.200,
                         d: float = 0.020) -> float:
        """四连杆正解：θ2 → q2"""
        delta_x = r * math.sin(theta2)
        delta_y = -(r * math.cos(theta2) - d)
        A = 2 * d * delta_x
        B = 2 * d * delta_y
        K = delta_x**2 + delta_y**2 + d**2 - l**2
        denom = math.sqrt(A**2 + B**2)
        if denom < 1e-12:
            raise ValueError("四连杆奇异")
        ratio = K / denom
        if abs(ratio) > 1.0:
            raise ValueError(f"四连杆无解 theta2={math.degrees(theta2):.1f}°")
        phi = math.atan2(A, B)
        q2 = phi + math.acos(ratio)
        return q2

    # ── 关节角 → 电机命令（考虑左右对称）───────────────
    def set_left_arm(self, q1_rad: float, theta2_rad: float,
                     kp: float = DEFAULT_KP, kd: float = DEFAULT_KD):
        """设置左臂关节角度（弧度）"""
        q1_rad = max(Q1_MIN, min(Q1_MAX, q1_rad))
        theta2_rad = max(THETA2_MIN, min(THETA2_MAX, theta2_rad))
        self.send_foc(MOTOR_L1, q1_rad, kp, kd)
        self.send_foc(MOTOR_L2, theta2_rad, kp, kd)
        self.get_logger().info(
            f"左臂: q1={math.degrees(q1_rad):.1f}° θ2={math.degrees(theta2_rad):.1f}°"
        )

    def set_right_arm(self, q1_rad: float, theta2_rad: float,
                      kp: float = DEFAULT_KP, kd: float = DEFAULT_KD):
        """设置右臂关节角度（弧度）"""
        q1_rad = max(Q1_MIN, min(Q1_MAX, q1_rad))
        theta2_rad = max(THETA2_MIN, min(THETA2_MAX, theta2_rad))
        self.send_foc(MOTOR_R1, q1_rad, kp, kd)
        self.send_foc(MOTOR_R2, theta2_rad, kp, kd)
        self.get_logger().info(
            f"右臂: q1={math.degrees(q1_rad):.1f}° θ2={math.degrees(theta2_rad):.1f}°"
        )

    def set_both_arms(self, q1_rad: float, theta2_rad: float,
                      kp: float = DEFAULT_KP, kd: float = DEFAULT_KD):
        """对称设置两臂（同一关节角）"""
        self.set_left_arm(q1_rad, theta2_rad, kp, kd)
        self.set_right_arm(q1_rad, theta2_rad, kp, kd)

    # ── 状态打印 ─────────────────────────────────────────
    def print_states(self):
        if not self.motor_states:
            self.get_logger().warn("尚未收到电机状态")
            return
        print("\n=== 电机状态 ===")
        for name, s in sorted(self.motor_states.items()):
            status = "在线" if s["online"] else "离线"
            print(f"  {name}: {status}  "
                  f"pos={math.degrees(s['position']):.1f}°  "
                  f"vel={s['velocity']:.2f} rad/s  "
                  f"torque={s['torque']:.3f} Nm  "
                  f"temp={s['temp']}°C")
        print()


def main():
    rclpy.init()
    ctrl = StrikeArmController()

    print("""
╔══════════════════════════════════════════════════╗
║         双臂平行连杆机械臂控制器                 ║
╠══════════════════════════════════════════════════╣
║  命令：                                          ║
║    s         - 查看电机状态                      ║
║    b         - 全部刹车                          ║
║    h <角度>  - 竖直悬挂姿态(q1=角度°, θ2=90°)   ║
║    p <q1> <θ2>  - 设置双臂(对称, 角度°)          ║
║    l <q1> <θ2>  - 设置左臂(角度°)                ║
║    r <q1> <θ2>  - 设置右臂(角度°)                ║
║    kp <值>   - 修改刚度系数(默认60)              ║
║    kd <值>   - 修改阻尼系数(默认6)               ║
║    q         - 刹车并退出                        ║
╚══════════════════════════════════════════════════╝
""")

    kp = DEFAULT_KP
    kd = DEFAULT_KD

    try:
        while rclpy.ok():
            rclpy.spin_once(ctrl, timeout_sec=0.05)

            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd == "q":
                ctrl.brake_all()
                break
            elif cmd == "b":
                ctrl.brake_all()
            elif cmd == "s":
                rclpy.spin_once(ctrl, timeout_sec=0.5)
                ctrl.print_states()
            elif cmd == "h":
                angle = float(parts[1]) if len(parts) > 1 else 45.0
                q1 = math.radians(angle)
                theta2 = math.radians(90.0)
                ctrl.set_both_arms(q1, theta2, kp, kd)
            elif cmd == "p":
                if len(parts) < 3:
                    print("用法: p <q1°> <θ2°>")
                    continue
                q1 = math.radians(float(parts[1]))
                theta2 = math.radians(float(parts[2]))
                ctrl.set_both_arms(q1, theta2, kp, kd)
            elif cmd == "l":
                if len(parts) < 3:
                    print("用法: l <q1°> <θ2°>")
                    continue
                q1 = math.radians(float(parts[1]))
                theta2 = math.radians(float(parts[2]))
                ctrl.set_left_arm(q1, theta2, kp, kd)
            elif cmd == "r":
                if len(parts) < 3:
                    print("用法: r <q1°> <θ2°>")
                    continue
                q1 = math.radians(float(parts[1]))
                theta2 = math.radians(float(parts[2]))
                ctrl.set_right_arm(q1, theta2, kp, kd)
            elif cmd == "kp":
                kp = float(parts[1]) if len(parts) > 1 else DEFAULT_KP
                print(f"kp = {kp}")
            elif cmd == "kd":
                kd = float(parts[1]) if len(parts) > 1 else DEFAULT_KD
                print(f"kd = {kd}")
            else:
                print(f"未知命令: {cmd}")

    except KeyboardInterrupt:
        ctrl.brake_all()
    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
