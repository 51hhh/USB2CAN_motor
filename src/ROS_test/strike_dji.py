#!/usr/bin/env python3
"""
DJI 3508 击球控制器

击球流程:
  1. 蓄力: 速度模式顺时针旋转(角度增加方向)到蓄力角度(默认210°), 到位后位置锁定
  2. 击球: 从蓄力位置开始, 直接电流输出驱动逆时针旋转, 转满1.25圈(450°)后停止
  3. 回零: 逆时针慢速回到0°, 位置锁定

路径示例:
  蓄力: 当前 → ... → 210° (顺时针, 角度增加)
  击球: 210° → 直接电流驱动逆时针 → 转满450° → 回零
方向约束: 击球时逆时针(角度减小), 蓄力时顺时针(角度增加)
GM3508 电流范围: -16384 ~ 16384

用法:
  python3 strike_dji.py
  python3 strike_dji.py --charge-angle 210 --current -16384
  python3 strike_dji.py --current -10000 --motor DJI3508_1 --motor2 DJI3508_2
  python3 strike_dji.py --rotation 540
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import DJIMotorCommandAdvanced, DJIMotorState
import time
import math
import argparse
import signal

# 控制模式常量(与 msg 定义一致)
MODE_DIRECT = 0
MODE_VELOCITY = 1
MODE_POSITION = 2

RATE_HZ = 200


class StrikeDJI:
    def __init__(self, args):
        self.charge_angle = args.charge_angle      # 蓄力角度(度)
        self.strike_current = args.current          # 击球电流(负值=逆时针)
        self.charge_speed = args.charge_speed      # 蓄力速度(RPM, 正值)
        self.motor_name = args.motor                # 主电机名
        self.motor2_name = args.motor2              # 副电机名(对称安装, 可选)
        self.charge_threshold = args.charge_threshold  # 蓄力到位阈值(度)
        self.return_speed = args.return_speed    # 回零速度(RPM, 正值)
        self.rotation = args.rotation            # 加速转角(度)

        rclpy.init()
        self.node = rclpy.create_node('strike_dji_controller')
        self.pub = self.node.create_publisher(
            DJIMotorCommandAdvanced, '/dji_motor_command_advanced', 10)

        self.current_angle = None
        self.current_rpm = None
        self.node.create_subscription(
            DJIMotorState, '/dji_motor_states', self._state_cb, 10)

        self._stop = False
        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        if msg.joint_name == self.motor_name:
            self.current_angle = msg.angle  # 度, [0, 360)
            self.current_rpm = msg.rpm

    def _sig_handler(self, signum, frame):
        print("\n[STOP] 收到停止信号, 制动...")
        self._stop = True

    def _publish_cmd(self, msg):
        """发布命令到主电机，如有副电机则同时发布反向命令"""
        self.pub.publish(msg)
        if self.motor2_name:
            msg2 = DJIMotorCommandAdvanced()
            msg2.joint_name = self.motor2_name
            msg2.mode = msg.mode
            if msg.mode == MODE_DIRECT:
                msg2.direct_output = -msg.direct_output  # 对称安装, 反向电流
            elif msg.mode == MODE_VELOCITY:
                msg2.velocity_target = -msg.velocity_target  # 反向速度
            elif msg.mode == MODE_POSITION:
                msg2.position_target = msg.position_target  # 位置由 mirror 处理
            self.pub.publish(msg2)

    def send_position(self, angle_deg):
        """发送位置控制命令"""
        msg = DJIMotorCommandAdvanced()
        msg.joint_name = self.motor_name
        msg.mode = MODE_POSITION
        msg.position_target = math.radians(angle_deg)
        self._publish_cmd(msg)

    def send_velocity(self, rpm):
        """发送速度控制命令, rpm 为转子侧 RPM"""
        msg = DJIMotorCommandAdvanced()
        msg.joint_name = self.motor_name
        msg.mode = MODE_VELOCITY
        # 底层回调会做 rad/s → RPM 转换, 这里需要传 rad/s
        msg.velocity_target = rpm * 2.0 * math.pi / 60.0
        self._publish_cmd(msg)

    def send_direct(self, current):
        """发送直接电流控制命令, current 范围 -16384~16384"""
        msg = DJIMotorCommandAdvanced()
        msg.joint_name = self.motor_name
        msg.mode = MODE_DIRECT
        msg.direct_output = int(max(-16384, min(16384, current)))
        self._publish_cmd(msg)

    def send_stop(self):
        """发送零电流停止"""
        msg = DJIMotorCommandAdvanced()
        msg.joint_name = self.motor_name
        msg.mode = MODE_DIRECT
        msg.direct_output = 0
        self._publish_cmd(msg)

    def wait_for_state(self, timeout=5.0):
        """等待收到第一次电机状态"""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.current_angle is not None:
                return True
        return False

    def angle_diff_ccw(self, current, target):
        """
        计算从 current 到 target 的逆时针(角度减小方向)距离
        返回值 > 0 表示还需要走多少度(逆时针)
        例: current=180, target=320 → 逆时针距离 = 360 - (320-180) = 220°
        例: current=10, target=320 → 逆时针距离 = (10 - 320) % 360 = 50°
        """
        diff = (current - target) % 360.0
        return diff

    def angle_diff_cw(self, current, target):
        """
        计算从 current 到 target 的顺时针(角度增加方向)距离
        返回值 > 0 表示还需要走多少度(顺时针)
        例: current=320, target=180 → 顺时针距离 = (180-320) % 360 = 220°
        例: current=170, target=180 → 顺时针距离 = (180-170) % 360 = 10°
        """
        diff = (target - current) % 360.0
        return diff

    def run(self):
        print(f"=== DJI 击球控制器 ===")
        print(f"  电机: {self.motor_name}")
        print(f"  蓄力角度: {self.charge_angle}°")
        print(f"  击球电流: {self.strike_current}")
        print(f"  加速转角: {self.rotation}°")
        if self.motor2_name:
            print(f"  副电机: {self.motor2_name} (对称, 反向电流)")
        print()

        # 等待电机状态
        print("[等待] 读取电机状态...")
        if not self.wait_for_state():
            print("[错误] 超时未收到电机状态, 请检查 motor_control_node 是否运行")
            return

        print(f"[就绪] 当前角度: {self.current_angle:.1f}°, RPM: {self.current_rpm}")

        # ========== 阶段1: 蓄力(顺时针, 角度增加方向) ==========
        print(f"\n[蓄力] 顺时针旋转到 {self.charge_angle}° ...")

        charge_rpm = self.charge_speed

        while not self._stop:
            rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            if self.current_angle is None:
                continue

            remaining = self.angle_diff_cw(self.current_angle, self.charge_angle)

            if remaining < self.charge_threshold:
                self.send_position(self.charge_angle)
                print(f"[蓄力完成] 当前角度: {self.current_angle:.1f}°, 剩余: {remaining:.1f}°")
                break

            self.send_velocity(charge_rpm)

        if self._stop:
            self.send_stop()
            return

        # 位置模式锁定稳定 0.5 秒
        lock_start = time.time()
        while not self._stop and time.time() - lock_start < 0.5:
            rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            self.send_position(self.charge_angle)

        # 等待用户确认击球
        input("\n按 Enter 键开始击球...")

        if self._stop:
            self.send_stop()
            return

        # ========== 阶段2: 击球(直接电流) ==========
        print(f"\n[击球] 直接电流驱动, 电流: {self.strike_current}, 转{self.rotation}°...")

        last_angle = self.current_angle
        total_rotated = 0.0

        while not self._stop:
            rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            if self.current_angle is None:
                continue

            delta = (last_angle - self.current_angle) % 360.0
            if delta < 180.0:
                total_rotated += delta
            last_angle = self.current_angle

            # 全程发送电流命令
            self.send_direct(self.strike_current)

            if total_rotated >= self.rotation:
                print(f"[击球完成] 累积转角: {total_rotated:.1f}°, 当前角度: {self.current_angle:.1f}°")
                break

        if self._stop:
            self.send_stop()
            return

        # ========== 阶段3: 逆时针慢速回零 ==========
        return_rpm = -self.return_speed
        print(f"\n[回零] 逆时针慢速回到 0°, 速度: {return_rpm} RPM ...")

        return_threshold = 5.0
        while not self._stop:
            rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            if self.current_angle is None:
                continue

            remaining = self.angle_diff_ccw(self.current_angle, 0.0)

            if remaining < return_threshold:
                self.send_position(0.0)
                print(f"[到达] 当前角度: {self.current_angle:.1f}°")
                break

            self.send_velocity(return_rpm)

        if self._stop:
            self.send_stop()
            return

        # 位置锁定稳定 1 秒
        hold_start = time.time()
        while not self._stop and time.time() - hold_start < 1.0:
            rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            self.send_position(0.0)

        print(f"[完成] 最终角度: {self.current_angle:.1f}°")
        self.send_stop()

    def shutdown(self):
        self.send_stop()
        time.sleep(0.05)
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='DJI 3508 击球控制器')
    parser.add_argument('--charge-angle', type=float, default=240.0,
                        help='蓄力角度(度, 默认210)')
    parser.add_argument('--current', type=int, default=-16384,
                        help='击球电流(负值=逆时针, 范围-16384~16384, 默认-16384)')
    parser.add_argument('--charge-speed', type=float, default=1000.0,
                        help='蓄力旋转速度(RPM, 正值, 默认1000)')
    parser.add_argument('--motor', type=str, default='DJI3508_1',
                        help='控制的电机名(默认DJI3508_1)')
    parser.add_argument('--motor2', type=str, default='DJI3508_2',
                        help='副电机名(对称安装, 自动反向, 默认DJI3508_2, 空=单电机)')
    parser.add_argument('--charge-threshold', type=float, default=5.0,
                        help='蓄力到位阈值(度, 默认5)')
    parser.add_argument('--return-speed', type=float, default=3000.0,
                        help='回零速度(RPM, 正值, 默认1000)')
    parser.add_argument('--rotation', type=float, default=450.0,
                        help='击球加速转角(度, 默认450=1.25圈)')
    args = parser.parse_args()

    controller = StrikeDJI(args)
    try:
        controller.run()
    finally:
        controller.shutdown()


if __name__ == '__main__':
    main()
