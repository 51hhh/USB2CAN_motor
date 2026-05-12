#!/usr/bin/env python3
"""
DJI 3508 电机零点校准工具
将当前位置设置为0度基准点

用法: python3 calibrate_dji_zero.py
  - 需要 motor_control_node 正在运行
  - 将球拍置于重力自然垂放位置后运行
  - 执行后自动重启节点并验证归零结果
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import DJIMotorState
import yaml
import re
import os
import signal
import subprocess
import time
import math

CONFIG_PATH = '/home/rosemaryrabbit/USB2CAN_motor/src/motor_control_ros2/config/motors.yaml'
WORKSPACE = '/home/rosemaryrabbit/USB2CAN_motor'

# 需要校准的 DJI 电机名称（与 motors.yaml 中一致）
MOTOR_NAMES = ['DJI3508_1', 'DJI3508_2']


def read_positions(timeout=8.0):
    """订阅话题读取所有 DJI 电机当前角度（输出轴弧度）"""
    rclpy.init()
    node = rclpy.create_node('dji_zero_calibrator')
    positions = {}

    def cb(msg):
        if msg.joint_name in MOTOR_NAMES:
            # angle 字段已经是 getAngleDegrees() 的结果（含 offset），转弧度
            positions[msg.joint_name] = math.radians(msg.angle)

    node.create_subscription(DJIMotorState, '/dji_motor_states', cb, 10)

    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(positions) >= len(MOTOR_NAMES):
            break

    node.destroy_node()
    rclpy.shutdown()
    return positions


def patch_offset_in_yaml(config_path, motor_name, new_offset):
    """用正则精确替换指定电机的 offset 值（保持文件格式完全不变）"""
    with open(config_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 匹配: "name: <motor_name>" 后面最近的 "offset: <数字>"
    pattern = (
        r'(- name: ' + re.escape(motor_name) + r'\s*\n'
        r'(?:    \w+:.*\n)*?'
        r'    offset: )'
        r'[-+]?\d*\.?\d+'
    )
    match = re.search(pattern, content)
    if not match:
        # 如果没有 offset 字段，在 id 行后面添加
        insert_pattern = (
            r'(- name: ' + re.escape(motor_name) + r'\s*\n'
            r'    type: \w+\s*\n'
            r'    id: \d+)'
        )
        insert_match = re.search(insert_pattern, content)
        if not insert_match:
            print(f"  ❌ 未在配置中找到 {motor_name}")
            return False
        new_content = (content[:insert_match.end()] +
                       f'\n    offset: {new_offset:.6f}' +
                       content[insert_match.end():])
        with open(config_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True

    new_content = content[:match.start()] + match.group(1) + f'{new_offset:.6f}' + content[match.end():]
    with open(config_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    return True


def restart_motor_control_node():
    """杀死旧节点，重新启动"""
    print("\n🔄 重启 motor_control_node ...")
    subprocess.run(['pkill', '-9', '-f', 'motor_control_ros2/motor_control_node'],
                   stderr=subprocess.DEVNULL)
    time.sleep(2)

    env = os.environ.copy()
    setup_cmd = f'source /opt/ros/humble/setup.bash && source {WORKSPACE}/install/setup.bash && ros2 run motor_control_ros2 motor_control_node'
    proc = subprocess.Popen(['bash', '-c', setup_cmd], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=os.setpgrp)
    print(f"  ✓ 节点已启动 (PID={proc.pid})，等待初始化...")
    time.sleep(8)
    return proc


def verify_zero(timeout=6.0):
    """验证所有电机位置是否接近 0"""
    positions = read_positions(timeout)
    print("\n=== 零位验证 ===")
    all_ok = True
    for name in MOTOR_NAMES:
        if name not in positions:
            print(f"  {name}: ❌ 未收到数据")
            all_ok = False
            continue
        deg = math.degrees(positions[name])
        ok = abs(deg) < 2.0 or abs(deg - 360.0) < 2.0
        mark = "✅" if ok else "⚠️"
        print(f"  {name}: {deg:.4f}° {mark}")
        if not ok:
            all_ok = False
    return all_ok


def main():
    print("\n" + "=" * 60)
    print("  DJI 3508 电机零点校准工具")
    print("  请确保球拍处于重力自然垂放位置")
    print("=" * 60)

    # 1. 读取当前位置
    print("\n📡 读取 DJI 电机当前位置...")
    positions = read_positions(timeout=8.0)

    if len(positions) < len(MOTOR_NAMES):
        print(f"\n❌ 仅读取到 {len(positions)}/{len(MOTOR_NAMES)} 个电机")
        print(f"   已读取: {list(positions.keys())}")
        print("   请确认 motor_control_node 正在运行且电机已上电")
        return

    # 2. 读取当前配置中的 offset
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    old_offsets = {}
    for iface in config.get('can_interfaces', []):
        for motor in iface.get('motors', []):
            name = motor.get('name', '')
            if name in MOTOR_NAMES:
                old_offsets[name] = motor.get('offset', 0.0)

    # 3. 计算新 offset 并写入
    # getAngleDegrees() = (getOutputPosition() - offset) 归一化到 [0,360)
    # 当前读到的角度已经减了旧 offset
    # 要让当前位置变为 0：new_offset = old_offset + current_angle_rad
    print("\n📝 更新 offset:")
    print("-" * 60)
    for name in MOTOR_NAMES:
        cur_pos_rad = positions[name]
        old_off = old_offsets.get(name, 0.0)
        new_off = old_off + cur_pos_rad

        ok = patch_offset_in_yaml(CONFIG_PATH, name, new_off)
        if ok:
            print(f"  {name}:")
            print(f"    当前角度: {math.degrees(cur_pos_rad):.2f}°  旧offset: {old_off:.6f} rad  新offset: {new_off:.6f} rad")

    print("-" * 60)
    print("✓ 配置已更新")

    # 4. 需要重新编译（offset 通过 yaml 加载，install 目录需要更新）
    print("\n🔨 重新编译（更新 install 目录的配置文件）...")
    build_cmd = f'cd {WORKSPACE} && source /opt/ros/humble/setup.bash && colcon build --packages-select motor_control_ros2'
    ret = subprocess.run(['bash', '-c', build_cmd],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if ret.returncode != 0:
        print("  ⚠️ 编译可能有警告，继续...")

    # 5. 重启节点
    proc = restart_motor_control_node()

    # 6. 验证
    ok = verify_zero()
    if ok:
        print("\n✅ 零点校准完成！所有电机位置接近 0°")
    else:
        print("\n⚠️  部分电机偏差较大，可能需要再次执行校准")

    # 7. 清理：终止校准中启动的 motor_control_node
    print("\n🧹 清理：终止 motor_control_node ...")
    subprocess.run(['pkill', '-9', '-f', 'motor_control_ros2/motor_control_node'],
                   stderr=subprocess.DEVNULL)
    print("  ✓ 已终止")
    print()


if __name__ == '__main__':
    main()
