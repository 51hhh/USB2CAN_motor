#!/bin/bash
# ==============================================================================
# 击球臂一键启动脚本 — tmux 多窗格版
#
# 数据流:
#   motor_control_node (RS485 → 4×GO8010)
#        ↕ unitree_go8010_command / unitree_go8010_states
#   strike_node (状态机 + 手柄/键盘/视觉 → 击球控制)
#
# 启动:
#   bash src/ROS_test/launch/start_strike.sh
#
# 停止:
#   tmux kill-session -t strike
# ==============================================================================

SESSION="strike"
WS_DIR="$HOME/USB2CAN_motor"

echo "=========================================="
echo "  击球臂一键启动系统"
echo "=========================================="

cd "$WS_DIR" || { echo "工作区 $WS_DIR 不存在"; exit 1; }

# ── 清理旧进程 ─────────────────────────────────────────────────
echo "清理旧进程..."
pkill -9 -f "motor_control_node|strike_node" 2>/dev/null
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

# ── 设备权限 ───────────────────────────────────────────────────
echo "设置设备权限..."
sudo chmod 666 /dev/ttyUSB* 2>/dev/null
sudo chmod 666 /dev/input/js* 2>/dev/null

# ── 设备检查 ───────────────────────────────────────────────────
echo ""
echo "设备状态:"
ls -l /dev/ttyUSB* 2>/dev/null || echo "  ⚠️  未找到 USB-RS485 设备"
ls -l /dev/input/js* 2>/dev/null || echo "  ⚠️  未找到手柄（键盘模式仍可用）"

# ── 环境变量 ───────────────────────────────────────────────────
SOURCE_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash"

echo ""
echo "启动节点..."
echo ""

# ── tmux 布局 ─────────────────────────────────────────────────
# Pane 1: motor_control_node (底层通信)
tmux new-session -d -s "$SESSION" \
  "bash -c '$SOURCE_CMD && echo [1] motor_control_node && ros2 run motor_control_ros2 motor_control_node; exec bash'"

# Pane 2: strike_node (击球控制)
tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [2] strike_node && ros2 run motor_control_ros2 strike_node --ros-args --params-file src/motor_control_ros2/config/strike_node_params.yaml; exec bash'"

# 自动平铺
tmux select-layout -t "$SESSION" tiled

sleep 2

echo ""
echo "=========================================="
echo "  ✅ 击球臂系统已启动 (tmux: $SESSION)"
echo "=========================================="
echo ""
echo "📋 键盘操作:"
echo "  1 = center  2 = left  3 = right  4 = strong"
echo "  空格/Enter = 触发击打"
echo "  q = 退出"
echo ""
echo "🎮 手柄操作（需连接 Xbox 手柄）:"
echo "  A/B/X/Y = 选方案（center/left/right/strong）"
echo "  LB = 触发击打"
echo ""
echo "🔧 查看所有窗口:"
echo "  tmux attach -t $SESSION"
echo "  Ctrl+B ↑↓ 切换窗格"
echo "  Ctrl+B D   断开"
echo ""
echo "🛑 停止系统:"
echo "  tmux kill-session -t $SESSION"
echo "=========================================="
