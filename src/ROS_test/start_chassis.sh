#!/bin/bash

# 全向轮底盘手柄控制系统 - tmux 版
# 数据流: joy_node → joystick_control_node(/cmd_vel_joy) → cmd_vel_mux_node(/cmd_vel) → omni_chassis_control_node → motor_control_node

SESSION="chassis"
WS_DIR="$HOME/USB2CAN_motor"

echo "=========================================="
echo "  全向轮底盘手柄控制系统"
echo "=========================================="

cd "$WS_DIR" || { echo "工作区 $WS_DIR 不存在"; exit 1; }

# 清理旧进程和旧 session
echo "清理旧进程..."
pkill -9 -f "motor_control_node|omni_chassis|joystick_control|joy_node|cmd_vel_mux" 2>/dev/null
tmux kill-session -t "$SESSION" 2>/dev/null
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

# 公共 source 命令
SOURCE_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash"

# 单窗口分屏布局：5个节点在同一页面
echo "启动全部 5 个节点（分屏模式）..."

# 创建 session + 第一个 pane（motor_control_node）
tmux new-session -d -s "$SESSION" \
  "bash -c '$SOURCE_CMD && echo [1]motor_control_node && ros2 run motor_control_ros2 motor_control_node; exec bash'"

# 分裂出第2个 pane（omni_chassis_control_node）
tmux split-window -t "$SESSION" -h \
  "bash -c '$SOURCE_CMD && echo [2]omni_chassis_control_node && ros2 run motor_control_ros2 omni_chassis_control_node --ros-args --params-file src/motor_control_ros2/config/omni_chassis_params.yaml; exec bash'"

# 分裂出第3个 pane（cmd_vel_mux_node）
tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [3]cmd_vel_mux_node && ros2 run motor_control_ros2 cmd_vel_mux_node --ros-args --params-file src/motor_control_ros2/config/cmd_vel_mux_params.yaml -p active_source:=joy; exec bash'"

# 分裂出第4个 pane（joy_node）
tmux split-window -t "$SESSION" -v \
  "bash -c 'source /opt/ros/humble/setup.bash && echo [4]joy_node && ros2 run joy joy_node --ros-args -p device_id:=0; exec bash'"

# 分裂出第5个 pane（joystick_control_node）
tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [5]joystick_control_node && ros2 run motor_control_ros2 joystick_control_node --ros-args --params-file src/motor_control_ros2/config/joystick_params.yaml; exec bash'"

# 自动平铺布局
tmux select-layout -t "$SESSION" tiled

sleep 2

echo ""
echo "=========================================="
echo "  ✅ 所有节点已启动 (tmux session: $SESSION)"
echo "=========================================="
echo ""
echo "📋 操作步骤："
echo "  1. 按 Start 键启用控制"
echo "  2. 按 Y/A/X 选择速度模式"
echo "  3. 推动左摇杆控制移动"
echo "  4. 推动右摇杆控制旋转"
echo ""
echo "⚠️  当前配置："
echo "  - 加速度 2.0 m/s²"
echo "  - 慢速(X): 0.5 m/s | 正常(A): 1.0 m/s | 快速(Y): 2.0 m/s"
echo "  - B 键急停"
echo ""
echo "🔧 调试命令："
echo "  tmux attach -t $SESSION     # 查看所有节点"
echo "  tmux select-window -t N     # 切换窗口 (0-4)"
echo "  Ctrl+B D                    # 断开 tmux"
echo ""
echo "🛑 停止所有节点："
echo "  tmux kill-session -t $SESSION"
echo "=========================================="
