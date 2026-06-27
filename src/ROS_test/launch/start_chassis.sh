#!/bin/bash

SESSION="chassis"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
START_RVIZ="${START_RVIZ:-1}"

CONTROL_CONFIG="$WS_DIR/src/motor_control_ros2/config/control_params.yaml"
MOTOR_CONFIG="$WS_DIR/src/motor_control_ros2/config/motors.yaml"
PID_CONFIG="$WS_DIR/src/motor_control_ros2/config/pid_params.yaml"
CHASSIS_CONFIG="$WS_DIR/src/motor_control_ros2/config/omni_chassis_params.yaml"
JOYSTICK_CONFIG="$WS_DIR/src/motor_control_ros2/config/joystick_params.yaml"
MCU_CONFIG="$WS_DIR/src/wheel_imu_ekf/config/mcu_bridge.yaml"
RVIZ_CONFIG="$WS_DIR/src/wheel_imu_ekf/rviz/mcu_odom.rviz"

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

for file in "$CONTROL_CONFIG" "$MOTOR_CONFIG" "$PID_CONFIG" "$CHASSIS_CONFIG" "$JOYSTICK_CONFIG" "$MCU_CONFIG"; do
  if [ ! -f "$file" ]; then
    echo "配置文件不存在: $file"
    exit 1
  fi
done

if [ "$START_RVIZ" = "1" ] && [ ! -f "$RVIZ_CONFIG" ]; then
  echo "RViz 配置文件不存在: $RVIZ_CONFIG"
  exit 1
fi

echo "清理旧进程..."
pkill -9 -f "motor_control_node|omni_chassis_control_node|joystick_control_node|mcu_odom_bridge_node|joy_node|rviz2" 2>/dev/null
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

echo "设置设备权限..."
sudo chmod 666 /dev/ttyACM* 2>/dev/null
sudo chmod 666 /dev/ttyUSB* 2>/dev/null
sudo chmod 666 /dev/input/js0 2>/dev/null

echo ""
echo "设备状态:"
ls -l /dev/ttyACM* 2>/dev/null || echo "  未找到 USB2CAN"
ls -l /dev/ttyUSB* 2>/dev/null || echo "  未找到定位串口"
ls -l /dev/input/js0 2>/dev/null || echo "  未找到手柄"

ENV_VARS="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=0"
SOURCE_CMD="cd \"$WS_DIR\" && $ENV_VARS && source \"$ROS_SETUP\" && source \"$WS_DIR/install/setup.bash\""

NODE_COUNT=5
if [ "$START_RVIZ" = "1" ]; then
  NODE_COUNT=6
fi

echo ""
echo "启动 $NODE_COUNT 个节点..."

tmux new-session -d -s "$SESSION" \
  "bash -c '$SOURCE_CMD && echo [1]motor_control_node && ros2 run motor_control_ros2 motor_control_node --ros-args -p control_config_file:=\"$CONTROL_CONFIG\" -p config_file:=\"$MOTOR_CONFIG\" -p pid_config_file:=\"$PID_CONFIG\"; exec bash'"

tmux split-window -t "$SESSION" -h \
  "bash -c '$SOURCE_CMD && echo [2]omni_chassis_control_node && ros2 run motor_control_ros2 omni_chassis_control_node --ros-args -p config_file:=\"$CHASSIS_CONFIG\"; exec bash'"

tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [3]mcu_odom_bridge_node && ros2 run wheel_imu_ekf mcu_odom_bridge_node --ros-args --params-file \"$MCU_CONFIG\"; exec bash'"

tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [4]joy_node && ros2 run joy joy_node --ros-args -p device_id:=0; exec bash'"

tmux split-window -t "$SESSION" -v \
  "bash -c '$SOURCE_CMD && echo [5]joystick_control_node && ros2 run motor_control_ros2 joystick_control_node --ros-args --params-file \"$JOYSTICK_CONFIG\"; exec bash'"

if [ "$START_RVIZ" = "1" ]; then
  tmux split-window -t "$SESSION" -v \
    "bash -c '$SOURCE_CMD && echo [6]rviz2 && rviz2 -d \"$RVIZ_CONFIG\"; exec bash'"
fi

tmux select-layout -t "$SESSION" tiled

echo ""
echo "tmux attach -t $SESSION"
echo "Start: 使能/禁用控制 | B: 急停/解除急停 | START_RVIZ=0 可关闭 RViz"
