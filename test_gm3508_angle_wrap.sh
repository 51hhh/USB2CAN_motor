#!/bin/bash
# GM3508 角度过零测试脚本
# 测试最短路径算法

echo "=========================================="
echo "GM3508 角度过零测试（最短路径）"
echo "=========================================="
echo ""
echo "测试内容："
echo "1. 从 10° 到 350°（应该逆时针 -20°）"
echo "2. 从 350° 到 10°（应该顺时针 +20°）"
echo "3. 跨越 0° 点的连续测试"
echo ""
echo "注意观察电机旋转方向！"
echo ""

# 加载环境
source /home/rick/desktop/ros/usb2can/install/setup.bash

# 等待用户确认
read -p "请确认 GM3508 电机已上线，按 Enter 开始测试..."

echo ""
echo "=========================================="
echo "测试 1: 从 10° 到 350°"
echo "=========================================="
echo "预期：电机应该逆时针旋转约 20°（最短路径）"
echo ""

echo "步骤 1: 移动到 10°..."
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 10.0}'

sleep 4

echo "步骤 2: 移动到 350°..."
echo ">>> 观察：电机应该逆时针旋转（向左），而不是顺时针旋转一大圈！"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 350.0}'

sleep 4

echo ""
read -p "电机是否逆时针旋转了约 20°？(y/n) " answer1
if [ "$answer1" = "y" ]; then
    echo "✅ 测试 1 通过"
else
    echo "❌ 测试 1 失败 - 检查角度过零逻辑"
fi

echo ""
echo "=========================================="
echo "测试 2: 从 350° 到 10°"
echo "=========================================="
echo "预期：电机应该顺时针旋转约 20°（最短路径）"
echo ""

echo "步骤 1: 确认在 350°..."
sleep 1

echo "步骤 2: 移动到 10°..."
echo ">>> 观察：电机应该顺时针旋转（向右），而不是逆时针旋转一大圈！"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 10.0}'

sleep 4

echo ""
read -p "电机是否顺时针旋转了约 20°？(y/n) " answer2
if [ "$answer2" = "y" ]; then
    echo "✅ 测试 2 通过"
else
    echo "❌ 测试 2 失败 - 检查角度过零逻辑"
fi

echo ""
echo "=========================================="
echo "测试 3: 跨越 0° 点连续测试"
echo "=========================================="
echo "测试序列：0° → 180° → 0° → 350° → 10° → 0°"
echo ""

echo "步骤 1: 移动到 0°..."
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 0.0}'
sleep 3

echo "步骤 2: 移动到 180°（顺时针 180°）..."
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 180.0}'
sleep 3

echo "步骤 3: 回到 0°（逆时针 180°）..."
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 0.0}'
sleep 3

echo "步骤 4: 移动到 350°（逆时针 10°，最短路径）..."
echo ">>> 观察：应该逆时针小幅旋转"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 350.0}'
sleep 3

echo "步骤 5: 移动到 10°（顺时针 20°，最短路径）..."
echo ">>> 观察：应该顺时针小幅旋转"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 10.0}'
sleep 3

echo "步骤 6: 回到 0°（逆时针 10°）..."
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI3508_2", mode: 2, position_target: 0.0}'
sleep 3

echo ""
read -p "所有跨越 0° 点的移动都选择了最短路径吗？(y/n) " answer3
if [ "$answer3" = "y" ]; then
    echo "✅ 测试 3 通过"
else
    echo "❌ 测试 3 失败 - 检查角度过零逻辑"
fi

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""

# 停止电机
echo "停止电机..."
ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  '{joint_name: "DJI3508_2", output: 0}'

echo ""
echo "测试总结："
if [ "$answer1" = "y" ] && [ "$answer2" = "y" ] && [ "$answer3" = "y" ]; then
    echo "✅ 所有测试通过！角度过零逻辑工作正常。"
else
    echo "⚠️  部分测试失败，请检查："
    echo "   1. 确认使用了 DJIMotorCommandAdvanced 消息"
    echo "   2. 确认 mode = 2（位置控制模式）"
    echo "   3. 检查 cascade_controller.hpp 中的角度归一化逻辑"
fi

echo ""
echo "减速比验证："
echo "- 手动旋转输出轴 1 圈，监控应显示 0° → 360°"
echo "- 电机轴实际转了 19 圈"
echo ""
