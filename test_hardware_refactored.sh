#!/bin/bash

# 硬件测试脚本
# 用于验证重构后的电机控制系统

set -e

echo "=========================================="
echo "  电机控制系统硬件测试"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_device() {
    local device=$1
    local name=$2
    
    if [ -e "$device" ]; then
        echo -e "${GREEN}✓${NC} $name 存在: $device"
        ls -l "$device"
        return 0
    else
        echo -e "${RED}✗${NC} $name 不存在: $device"
        return 1
    fi
}

# 1. 检查硬件设备
echo "1. 检查硬件设备"
echo "----------------------------------------"

CAN_DEVICE="/dev/ttyACM0"
SERIAL_DEVICE="/dev/ttyUSB0"

check_device "$CAN_DEVICE" "CAN 设备 (USB-CAN)" || echo -e "${YELLOW}  提示: 请连接 USB-CAN 适配器${NC}"
check_device "$SERIAL_DEVICE" "串口设备 (USB-485)" || echo -e "${YELLOW}  提示: 请连接 USB-485 适配器${NC}"

echo ""

# 2. 检查设备权限
echo "2. 检查设备权限"
echo "----------------------------------------"

check_permission() {
    local device=$1
    if [ -e "$device" ]; then
        if [ -r "$device" ] && [ -w "$device" ]; then
            echo -e "${GREEN}✓${NC} $device 权限正常"
        else
            echo -e "${RED}✗${NC} $device 权限不足"
            echo -e "${YELLOW}  运行: sudo chmod 666 $device${NC}"
        fi
    fi
}

check_permission "$CAN_DEVICE"
check_permission "$SERIAL_DEVICE"

echo ""

# 3. 检查编译状态
echo "3. 检查编译状态"
echo "----------------------------------------"

WORKSPACE="/home/rick/desktop/ros/usb2can"
NODE_EXECUTABLE="$WORKSPACE/install/motor_control_ros2/lib/motor_control_ros2/motor_control_node"

if [ -f "$NODE_EXECUTABLE" ]; then
    echo -e "${GREEN}✓${NC} 节点可执行文件存在"
    ls -lh "$NODE_EXECUTABLE"
else
    echo -e "${RED}✗${NC} 节点可执行文件不存在"
    echo -e "${YELLOW}  运行: cd $WORKSPACE && colcon build --packages-select motor_control_ros2${NC}"
    exit 1
fi

echo ""

# 4. 检查 ROS2 环境
echo "4. 检查 ROS2 环境"
echo "----------------------------------------"

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS2 环境未加载"
    echo -e "${YELLOW}  运行: source /opt/ros/humble/setup.bash${NC}"
    exit 1
else
    echo -e "${GREEN}✓${NC} ROS2 版本: $ROS_DISTRO"
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} 工作空间已编译"
else
    echo -e "${RED}✗${NC} 工作空间未编译"
    exit 1
fi

echo ""

# 5. 运行测试选项
echo "5. 测试选项"
echo "----------------------------------------"
echo "请选择测试模式:"
echo "  1) 仅启动节点（查看初始化日志）"
echo "  2) 启动节点并监控 CAN 数据"
echo "  3) 启动节点并发送测试命令"
echo "  4) 退出"
echo ""
read -p "请输入选项 [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "启动电机控制节点..."
        echo "按 Ctrl+C 停止"
        echo ""
        cd "$WORKSPACE"
        source install/setup.bash
        ros2 run motor_control_ros2 motor_control_node
        ;;
    2)
        echo ""
        echo "启动节点并监控 CAN 数据..."
        echo "在另一个终端运行: ros2 topic echo /dji_motor_states"
        echo "按 Ctrl+C 停止"
        echo ""
        cd "$WORKSPACE"
        source install/setup.bash
        ros2 run motor_control_ros2 motor_control_node
        ;;
    3)
        echo ""
        echo "启动节点并发送测试命令..."
        echo ""
        echo "在另一个终端运行以下命令发送测试:"
        echo ""
        echo "# DJI 电机测试 (GM6020 电压控制)"
        echo "ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \\"
        echo "  '{joint_name: \"yaw_motor\", output: 1000}'"
        echo ""
        echo "# 达妙电机测试 (MIT 模式)"
        echo "ros2 topic pub --once /damiao_motor_command motor_control_ros2/msg/DamiaoMotorCommand \\"
        echo "  '{joint_name: \"joint1_motor\", pos_des: 0.0, vel_des: 0.0, kp: 10.0, kd: 1.0, torque_ff: 0.0}'"
        echo ""
        cd "$WORKSPACE"
        source install/setup.bash
        ros2 run motor_control_ros2 motor_control_node
        ;;
    4)
        echo "退出测试"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac
