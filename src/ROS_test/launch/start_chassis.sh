#!/bin/bash

SESSION="chassis"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
START_RVIZ="${START_RVIZ:-0}"
START_FRONTVEHICLE="${START_FRONTVEHICLE:-1}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-88}"

CONTROL_CONFIG="$WS_DIR/src/motor_control_ros2/config/control_params.yaml"
MOTOR_CONFIG="$WS_DIR/src/motor_control_ros2/config/motors.yaml"
PID_CONFIG="$WS_DIR/src/motor_control_ros2/config/pid_params.yaml"
CHASSIS_CONFIG="$WS_DIR/src/motor_control_ros2/config/omni_chassis_params.yaml"
RC_USB_CONFIG="$WS_DIR/src/motor_control_ros2/config/rc_usb_control_params.yaml"
VISION_GOAL_CONFIG="$WS_DIR/src/motor_control_ros2/config/vision_goal_tracker_params.yaml"
MCU_CONFIG="$WS_DIR/src/wheel_imu_ekf/config/mcu_bridge.yaml"
RVIZ_CONFIG="$WS_DIR/src/wheel_imu_ekf/rviz/mcu_odom.rviz"
FRONTVEHICLE_RUN="${FRONTVEHICLE_RUN:-/home/nvidia/frontvehicle/run.sh}"
LOG_ROOT="$WS_DIR/log/chassis_start"
RUN_LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"

echo "=========================================="
echo "  Robot_1A 底盘最小闭环"
echo "=========================================="

cd "$WS_DIR" || { echo "工作区 $WS_DIR 不存在"; exit 1; }

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 环境不存在: $ROS_SETUP"
  echo "可通过 ROS_SETUP=/path/to/setup.bash 指定"
  exit 1
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "未找到 $WS_DIR/install/setup.bash"
  echo "请先执行: colcon build --packages-select motor_control_ros2 wheel_imu_ekf"
  exit 1
fi

for file in "$CONTROL_CONFIG" "$MOTOR_CONFIG" "$PID_CONFIG" "$CHASSIS_CONFIG" "$RC_USB_CONFIG" "$VISION_GOAL_CONFIG" "$MCU_CONFIG"; do
  if [ ! -f "$file" ]; then
    echo "配置文件不存在: $file"
    exit 1
  fi
done

if [ "$START_RVIZ" = "1" ] && [ ! -f "$RVIZ_CONFIG" ]; then
  echo "RViz 配置文件不存在: $RVIZ_CONFIG"
  exit 1
fi

if [ "$START_FRONTVEHICLE" = "1" ] && [ ! -x "$FRONTVEHICLE_RUN" ]; then
  echo "frontvehicle 启动脚本不存在或不可执行: $FRONTVEHICLE_RUN"
  echo "可通过 START_FRONTVEHICLE=0 只启动底盘，或 FRONTVEHICLE_RUN=/path/to/run.sh 指定"
  exit 1
fi

echo ""
echo "设备状态:"
ls -l /dev/robocon_usb2can 2>/dev/null || echo "  未找到 /dev/robocon_usb2can (USB2CAN, 2e88:4603)"
ls -l /dev/robocon_rc 2>/dev/null || echo "  未找到 /dev/robocon_rc (STM32 RC CDC, 0483:5740)"
ls -l /dev/robocon_odom 2>/dev/null || echo "  未找到 /dev/robocon_odom (QinHeng odom, 1a86:7522)"
mkdir -p "$RUN_LOG_DIR"

ENV_VARS="unset CYCLONEDDS_URI && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE"
SOURCE_CMD="cd \"$WS_DIR\" && $ENV_VARS && source \"$ROS_SETUP\" && source \"$WS_DIR/install/setup.bash\""

send_node() {
  local pane="$1"
  local name="$2"
  local command="$3"
  local log_file="$RUN_LOG_DIR/$name.log"

  tmux send-keys -t "$pane" \
    "$SOURCE_CMD && set -o pipefail && echo [$name] log=$log_file && ( $command ) 2>&1 | tee -a \"$log_file\"; echo [$name] exited=\${PIPESTATUS[0]}" C-m
}

echo "清理旧进程..."
pkill -9 -f "motor_control_node|omni_chassis_control_node|rc_usb_control_node|vision_goal_tracker_node|mcu_odom_bridge_node|frontvehicle_tracker|joystick_control_node|cmd_vel_mux_node|joy_node|rviz2" 2>/dev/null
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

NODE_COUNT=5
if [ "$START_FRONTVEHICLE" = "1" ]; then
  NODE_COUNT=$((NODE_COUNT + 1))
fi
if [ "$START_RVIZ" = "1" ]; then
  NODE_COUNT=$((NODE_COUNT + 1))
fi

echo ""
echo "启动 $NODE_COUNT 个节点..."

PANE_MOTOR="$(tmux new-session -d -s "$SESSION" -n core -P -F "#{pane_id}" bash)"
PANE_OMNI="$(tmux split-window -t "$PANE_MOTOR" -h -P -F "#{pane_id}" bash)"
PANE_MCU="$(tmux split-window -t "$PANE_MOTOR" -v -P -F "#{pane_id}" bash)"
PANE_RC="$(tmux split-window -t "$PANE_OMNI" -v -P -F "#{pane_id}" bash)"
PANE_VISION="$(tmux split-window -t "$PANE_MCU" -v -P -F "#{pane_id}" bash)"

send_node "$PANE_MOTOR" "motor_control_node" \
  "ros2 run motor_control_ros2 motor_control_node --ros-args -p control_config_file:=$CONTROL_CONFIG -p config_file:=$MOTOR_CONFIG -p pid_config_file:=$PID_CONFIG"
send_node "$PANE_OMNI" "omni_chassis_control_node" \
  "ros2 run motor_control_ros2 omni_chassis_control_node --ros-args -p config_file:=$CHASSIS_CONFIG"
send_node "$PANE_MCU" "mcu_odom_bridge_node" \
  "ros2 run wheel_imu_ekf mcu_odom_bridge_node --ros-args --params-file $MCU_CONFIG"
send_node "$PANE_RC" "rc_usb_control_node" \
  "ros2 run motor_control_ros2 rc_usb_control_node --ros-args --params-file $RC_USB_CONFIG"
send_node "$PANE_VISION" "vision_goal_tracker_node" \
  "ros2 run motor_control_ros2 vision_goal_tracker_node --ros-args --params-file $VISION_GOAL_CONFIG"

if [ "$START_FRONTVEHICLE" = "1" ]; then
  PANE_FRONTVEHICLE="$(tmux new-window -t "$SESSION" -n frontvehicle -P -F "#{pane_id}" bash)"
  send_node "$PANE_FRONTVEHICLE" "frontvehicle_tracker" \
    "\"$FRONTVEHICLE_RUN\" --no-gui --ros"
fi

if [ "$START_RVIZ" = "1" ]; then
  PANE_RVIZ="$(tmux new-window -t "$SESSION" -n rviz -P -F "#{pane_id}" bash)"
  send_node "$PANE_RVIZ" "rviz2" "rviz2 -d $RVIZ_CONFIG"
fi

tmux select-layout -t "$SESSION":core tiled
tmux select-window -t "$SESSION":core

echo ""
echo "tmux attach -t $SESSION"
echo "日志目录: $RUN_LOG_DIR"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "左拨杆上档: 手动遥控 | 左拨杆中档: 硬急停零速 | 左拨杆下档: 视觉速度(/vision/cmd_vel) | START_FRONTVEHICLE=0 可不启动相机 | START_RVIZ=1 可额外启动 RViz"
