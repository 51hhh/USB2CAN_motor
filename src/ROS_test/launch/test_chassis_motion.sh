#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-88}"
SPEED="${SPEED:-0.4}"
TURN="${TURN:-0.5}"
DURATION="${DURATION:-1.2}"
RATE="${RATE:-20}"

cd "$WS_DIR"
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
source "$ROS_SETUP"
source "$WS_DIR/install/setup.bash"

times=$(python3 -c "print(max(1, int(round(float('$DURATION') * float('$RATE')))))")

publish_cmd() {
  local name="$1"
  local x="$2"
  local y="$3"
  local wz="$4"
  echo "[$name] linear.x=$x linear.y=$y angular.z=$wz"
  ros2 topic pub --rate "$RATE" --times "$times" /vision/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $x, y: $y, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $wz}}" >/dev/null
  sleep 0.4
}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE"
echo "请确认底盘已架空或周围安全，左拨杆下档 vision，随时可拨中档急停。"
sleep 2

publish_cmd "forward" 0.0 "$SPEED" 0.0
publish_cmd "backward" 0.0 "-$SPEED" 0.0
publish_cmd "right" "$SPEED" 0.0 0.0
publish_cmd "left" "-$SPEED" 0.0 0.0
publish_cmd "ccw" 0.0 0.0 "$TURN"
publish_cmd "cw" 0.0 0.0 "-$TURN"
publish_cmd "stop" 0.0 0.0 0.0
