#!/bin/bash
# 底盘电机零位标定辅助脚本

# 确保已 source ROS2 环境
source /home/rick/ros2_humble/install/setup.bash
source /home/rick/desktop/ros/usb2can/install/setup.bash

echo "=========================================="
echo "  底盘电机零位标定辅助工具"
echo "=========================================="
echo ""

# 检查节点是否运行
if ! ros2 node list | grep -q motor_control_node; then
    echo "❌ 错误：motor_control_node 未运行！"
    echo ""
    echo "请先在另一个终端启动电机控制节点："
    echo "  ros2 run motor_control_ros2 motor_control_node"
    echo ""
    exit 1
fi

echo "✅ 检测到 motor_control_node 正在运行"
echo ""

echo "请选择操作："
echo "1. 查看当前电机角度（用于零位标定）"
echo "2. 测试直线前进"
echo "3. 测试横向平移"
echo "4. 测试原地旋转"
echo "5. 停止所有运动"
echo ""

read -p "请输入选项 (1-5): " choice

case $choice in
    1)
        echo ""
        echo "=========================================="
        echo "  当前电机角度读取"
        echo "=========================================="
        echo ""
        echo "📋 操作步骤："
        echo "1. 手动将所有舵轮调整到正前方位置"
        echo "2. 记录下方显示的每个转向电机角度"
        echo "3. 将角度值填入 chassis_params.yaml 的 steer_offset 参数"
        echo ""
        echo "正在监听电机状态（10秒）..."
        echo "按 Ctrl+C 提前停止"
        echo ""
        
        timeout 10 ros2 topic echo /dji_motor_states --field states | grep -A 3 "joint_name: 'DJI6020"
        
        echo ""
        echo "✅ 读取完成"
        echo ""
        echo "📝 配置文件位置："
        echo "  src/motor_control_ros2/config/chassis_params.yaml"
        ;;
        
    2)
        echo ""
        echo "=========================================="
        echo "  测试：直线前进"
        echo "=========================================="
        echo ""
        echo "预期结果："
        echo "  - 所有舵轮朝向 0° (正前方)"
        echo "  - 所有驱动轮向前滚动"
        echo "  - 底盘直线前进"
        echo ""
        echo "发送命令：线速度 X = 0.5 m/s"
        
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
          "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        
        echo ""
        echo "✅ 命令已发送"
        echo "⚠️  请观察底盘运动是否符合预期"
        ;;
        
    3)
        echo ""
        echo "=========================================="
        echo "  测试：横向平移（向左）"
        echo "=========================================="
        echo ""
        echo "预期结果："
        echo "  - 所有舵轮朝向 90° (正左方)"
        echo "  - 底盘向左平移"
        echo ""
        echo "发送命令：线速度 Y = 0.5 m/s"
        
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
          "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        
        echo ""
        echo "✅ 命令已发送"
        echo "⚠️  请观察底盘运动是否符合预期"
        ;;
        
    4)
        echo ""
        echo "=========================================="
        echo "  测试：原地旋转（逆时针）"
        echo "=========================================="
        echo ""
        echo "预期结果："
        echo "  - 四个舵轮形成切向排列"
        echo "  - 底盘逆时针旋转"
        echo ""
        echo "发送命令：角速度 Z = 0.5 rad/s"
        
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
          "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
        
        echo ""
        echo "✅ 命令已发送"
        echo "⚠️  请观察底盘运动是否符合预期"
        ;;
        
    5)
        echo ""
        echo "=========================================="
        echo "  停止所有运动"
        echo "=========================================="
        echo ""
        
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
          "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        
        echo "✅ 停止命令已发送"
        ;;
        
    *)
        echo "❌ 无效选项"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo ""
echo "📖 相关文档："
echo "  .agent/chassis_motor_calibration.md"
echo ""
