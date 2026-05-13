#!/bin/bash

echo "🚀 启动 ROS 电机控制系统"
echo "================================"

# 检查 roscore 是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "⚠️  ROS master 未运行，正在启动 roscore..."
    
    # 在后台启动 roscore
    gnome-terminal --title="ROS Core" -- bash -c "roscore; exec bash" &
    
    # 等待 roscore 完全启动
    echo "等待 roscore 启动..."
    sleep 3
    
    echo "✅ roscore 已启动"
else
    echo "✅ roscore 已在运行"
fi

echo ""
echo "📋 请选择运行模式:"
echo "  1) 启动电机控制器 (main.py --ros)"
echo "  2) 启动测试发布器 (ros_target_publisher.py)"
echo "  3) 同时启动两者（推荐测试）"
echo "  4) 退出"
echo ""
read -p "请选择 [1-4]: " choice

case $choice in
    1)
        echo "启动电机控制器..."
        python main.py --ros
        ;;
    2)
        echo "启动测试发布器..."
        python ros_target_publisher.py
        ;;
    3)
        echo "同时启动控制器和发布器..."
        gnome-terminal --title="电机控制器" -- bash -c "python main.py --ros; exec bash" &
        sleep 2
        gnome-terminal --title="测试发布器" -- bash -c "python ros_target_publisher.py; exec bash" &
        echo "✅ 已在新终端中启动"
        ;;
    4)
        echo "退出"
        exit 0
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
