#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
CONFIG=""
BAG_PATH=""
START_RVIZ="${START_RVIZ:-1}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
PLAYER_ARGS=()

usage() {
  echo "Usage: $0 <bag_dir> [--config <yaml>] [--no-rviz] [player args...]"
  echo ""
  echo "Examples:"
  echo "  $0 bags/bag_20260627_120000"
  echo "  $0 bags/test --config src/rviz_bag_tools/config/chassis_bag.yaml --speed 2.0"
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

BAG_PATH="$1"
shift

while [ $# -gt 0 ]; do
  case "$1" in
    --config|-c)
      CONFIG="$2"
      shift 2
      ;;
    --no-rviz)
      START_RVIZ=0
      shift
      ;;
    *)
      PLAYER_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ ! -d "$BAG_PATH" ]; then
  echo "Bag directory not found: $BAG_PATH"
  exit 1
fi

if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

PKG_PREFIX="$(ros2 pkg prefix rviz_bag_tools)"
PKG_SHARE="$PKG_PREFIX/share/rviz_bag_tools"

if [ -z "$CONFIG" ]; then
  CONFIG="$PKG_SHARE/config/chassis_bag.yaml"
fi

if [ -z "$RVIZ_CONFIG" ]; then
  RVIZ_CONFIG="$PKG_SHARE/rviz/replay_chassis.rviz"
fi

echo "=========================================="
echo "  RViz rosbag replay"
echo "=========================================="
echo "Bag:    $BAG_PATH"
echo "Config: $CONFIG"
echo "RViz:   $RVIZ_CONFIG"
echo ""

if [ "$START_RVIZ" = "1" ]; then
  rviz2 -d "$RVIZ_CONFIG" >/tmp/rviz_bag_tools_replay.log 2>&1 &
  RVIZ_PID=$!
  echo "RViz started, pid=$RVIZ_PID"
  sleep 1
fi

ros2 run rviz_bag_tools interactive_bag_player.py \
  --bag "$BAG_PATH" \
  --config "$CONFIG" \
  "${PLAYER_ARGS[@]}"
