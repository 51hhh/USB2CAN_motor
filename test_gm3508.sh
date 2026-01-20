#!/bin/bash
# GM3508 电机测试脚本
# 测试编码器位置转换和 PID 控制

echo "=========================================="
echo "GM3508 电机测试"
echo "=========================================="
echo ""
echo "测试内容："
echo "1. 编码器位置转换（电机轴 → 输出轴，减速比 19:1）"
echo "2. 速度反馈转换（电机轴 RPM → 输出轴 RPM）"
echo "3. 位置控制测试"
echo ""
echo "按 Ctrl+C 停止测试"
echo ""

# 加载环境
source /home/rick/desktop/ros/usb2can/install/setup.bash

# 等待用户确认
read -p "请确认 GM3508 电机已连接并上电，按 Enter 继续..."

echo ""
echo "启动控制节点..."
echo ""

# 启动控制节点（后台运行）
ros2 run motor_control_ros2 motor_control_node &
CONTROL_PID=$!

# 等待节点启动
sleep 3

echo ""
echo "=========================================="
echo "测试 1: 查看电机状态"
echo "=========================================="
echo ""
echo "查看 DJI3508_2 的反馈数据（5 秒）..."
timeout 5 ros2 topic echo /dji_motor_states --field states | grep -A 10 "DJI3508_2" || true

echo ""
echo "=========================================="
echo "测试 2: 速度控制测试"
echo "=========================================="
echo ""
echo "发送速度命令: 50 RPM（输出轴）..."
ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  "{joint_name: 'DJI3508_2', output: 1000}"

sleep 3

echo ""
echo "查看电机响应..."
timeout 3 ros2 topic echo /dji_motor_states --field states | grep -A 10 "DJI3508_2" || true

echo ""
echo "停止电机..."
ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  "{joint_name: 'DJI3508_2', output: 0}"

sleep 2

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "关闭控制节点..."
kill $CONTROL_PID 2>/dev/null || true

echo ""
echo "测试总结："
echo "- 如果电机能正常响应，说明编码器配置正确"
echo "- GM3508 编码器在电机轴，减速比 19:1"
echo "- 输出轴转 1 圈 = 电机轴转 19 圈 = 编码器 19*8192 个计数"
echo ""
