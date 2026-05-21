#!/bin/bash
# 简单的硬件测试脚本

# 确保已 source ROS2 环境
source /home/rick/ros2_humble/install/setup.bash
source /home/rick/desktop/ros/usb2can/install/setup.bash

echo "=================================="
echo "ROS2 电机控制包 - 硬件测试"
echo "=================================="

echo "请选择要测试的电机类型:"
echo "1. DJI GM6020 (CAN)"
echo "2. 达妙 DM4340 (MIT)"
echo "3. 宇树 A1 (USB-Serial)"
echo "4. 仅查看状态 (所有电机)"

read -p "请输入选项 (1-4): " choice

case $choice in
    1)
        echo "正在发送 DJI GM6020 控制命令 (ID 1)..."
        ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand "{joint_name: 'DJI6020_1', output: 2000}"
        ;;
    2)
        echo "正在发送 达妙 DM4340 控制命令 (MIT 模式)..."
        ros2 topic pub --once /damiao_motor_command motor_control_ros2/msg/DamiaoMotorCommand "{joint_name: 'joint1', pos_des: 0.0, vel_des: 0.0, kp: 5.0, kd: 0.1, torque_ff: 0.0}"
        ;;
    3)
        echo "正在发送 宇树 A1 控制命令 (力位混合模式)..."
        ros2 topic pub --once /unitree_motor_command motor_control_ros2/msg/UnitreeMotorCommand "{joint_name: 'fl_hip_motor', mode: 10, kp: 0.5, kd: 0.05, pos_des: 0.0, vel_des: 0.0, torque_ff: 0.0}"
        ;;
    4)
        echo "正在监听所有电机状态话题..."
        echo "提示：如果没有节点运行，请先执行: ros2 launch motor_control_ros2 motor_control.launch.py"
        echo "按 Ctrl+C 停止"
        
        # 检查节点是否运行
        if ! ros2 node list | grep -q motor_control_node; then
            echo "错误：motor_control_node 未运行！"
            echo "请先在另一个终端执行: ros2 launch motor_control_ros2 motor_control.launch.py"
            exit 1
        fi
        
        # 使用 timeout 避免无限等待
        timeout 30 ros2 topic echo /dji_motor_states &
        PID1=$!
        timeout 30 ros2 topic echo /damiao_motor_states &
        PID2=$!
        timeout 30 ros2 topic echo /unitree_motor_states &
        PID3=$!
        
        wait $PID1 $PID2 $PID3
        ;;
    *)
        echo "无效选项"
        ;;
esac

echo "测试结束"
