#!/bin/bash
# DJI 电机 PID 串级控制测试脚本

echo "=========================================="
echo "  DJI 电机 PID 串级控制测试"
echo "=========================================="
echo ""

# 设置环境
source /home/rick/desktop/ros/usb2can/install/setup.bash

echo "测试 1: 直接输出模式（向后兼容）"
echo "----------------------------------------"
echo "发送命令: 电压 1000"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 0, direct_output: 1000}'
sleep 2

echo ""
echo "测试 2: 速度控制模式"
echo "----------------------------------------"
echo "发送命令: 速度 3.14 rad/s"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 1, velocity_target: 3.14}'
sleep 3

echo ""
echo "测试 3: 位置控制模式"
echo "----------------------------------------"
echo "发送命令: 位置 1.57 rad (90度)"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 2, position_target: 1.57}'
sleep 3

echo ""
echo "测试 4: 位置控制 - 回到零位"
echo "----------------------------------------"
echo "发送命令: 位置 0.0 rad"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 2, position_target: 0.0}'
sleep 3

echo ""
echo "测试 5: 停止电机"
echo "----------------------------------------"
echo "发送命令: 直接输出 0"
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 0, direct_output: 0}'

echo ""
echo "=========================================="
echo "  测试完成！"
echo "=========================================="
echo ""
echo "提示："
echo "1. 使用 'ros2 topic echo /dji_motor_states' 查看电机状态"
echo "2. 调整 PID 参数请编辑: config/pid_params.yaml"
echo "3. 重新编译后参数生效: colcon build --packages-select motor_control_ros2"
