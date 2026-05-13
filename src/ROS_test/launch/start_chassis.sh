#!/bin/bash

echo "=========================================="
echo "  全向轮底盘手柄控制系统"
echo "=========================================="

cd /home/rosemaryrabbit/USB2CAN_motor

# 清理旧进程
echo "清理旧进程..."
pkill -9 -f "motor_control_node|omni_chassis|joystick_control|joy_node" 2>/dev/null
sleep 1

# 设置设备权限
echo "设置设备权限..."
sudo chmod 666 /dev/ttyACM* 2>/dev/null
sudo chmod 666 /dev/input/js0 2>/dev/null

# 检查设备
echo ""
echo "设备状态:"
ls -l /dev/ttyACM* 2>/dev/null || echo "  ⚠️  未找到USB2CAN设备"
ls -l /dev/input/js0 2>/dev/null || echo "  ⚠️  未找到手柄设备"

echo ""
echo "启动节点..."
echo ""

# 1. 启动电机控制节点
echo "1/4 启动电机控制节点..."
gnome-terminal --title="电机控制" -- bash -c "
    cd /home/rosemaryrabbit/USB2CAN_motor
    source install/setup.bash
    echo '=========================================='
    echo '  电机控制节点'
    echo '=========================================='
    ros2 run motor_control_ros2 motor_control_node
    read -p '按Enter关闭...'
" &
sleep 1

# 2. 启动底盘控制节点
echo "2/4 启动底盘控制节点..."
gnome-terminal --title="底盘控制" -- bash -c "
    cd /home/rosemaryrabbit/USB2CAN_motor
    source install/setup.bash
    echo '=========================================='
    echo '  底盘控制节点'
    echo '=========================================='
    ros2 run motor_control_ros2 omni_chassis_control_node \
      --ros-args --params-file src/motor_control_ros2/config/omni_chassis_params.yaml
    read -p '按Enter关闭...'
" &
sleep 1

# 3. 启动Joy节点
echo "3/4 启动Joy节点..."
gnome-terminal --title="手柄输入" -- bash -c "
    source /opt/ros/humble/setup.bash
    echo '=========================================='
    echo '  手柄输入节点'
    echo '=========================================='
    ros2 run joy joy_node --ros-args -p device_id:=0
    read -p '按Enter关闭...'
" &
sleep 1

# 4. 启动手柄控制节点
echo "4/4 启动手柄控制节点..."
gnome-terminal --title="手柄控制" -- bash -c "
    cd /home/rosemaryrabbit/USB2CAN_motor
    source install/setup.bash
    echo '=========================================='
    echo '  手柄控制节点'
    echo '=========================================='
    echo ''
    echo '🎮 控制说明：'
    echo '  Start键: 启用/禁用控制'
    echo '  B键: 急停'
    echo '  X键: 慢速模式 (50%)'
    echo '  A键: 正常模式 (80%)'
    echo '  Y键: 快速模式 (100%)'
    echo ''
    echo '  左摇杆Y: 前进/后退'
    echo '  左摇杆X: 左右平移'
    echo '  右摇杆X: 旋转'
    echo ''
    echo '=========================================='
    echo ''
    ros2 run motor_control_ros2 joystick_control_node \
      --ros-args --params-file src/motor_control_ros2/config/joystick_params.yaml
    read -p '按Enter关闭...'
" &

sleep 2

echo ""
echo "=========================================="
echo "  ✅ 所有节点已启动"
echo "=========================================="
echo ""
echo "📋 操作步骤："
echo "  1. 按 Start 键启用控制"
echo "  2. 按 Y/A/X 选择速度模式"
echo "  3. 推动左摇杆控制移动"
echo "  4. 推动右摇杆控制旋转"
echo ""
echo "⚠️  当前配置："
echo "  - 启动柔和，加速度0.5 m/s²"
echo "  - 最大速度: 0.10 m/s (~1600 RPM)"
echo "  - B 键急停"
echo ""
echo "=========================================="
