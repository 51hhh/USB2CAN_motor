#!/bin/bash
set -e

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
CONFIG=""
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
ADAPTER_PID=""

usage() {
  echo "Usage: $0 [--config <yaml>] [--rviz <rviz_file>]"
  echo ""
  echo "This mode only starts RViz. It does not publish or bridge any topic."
}

while [ $# -gt 0 ]; do
  case "$1" in
    --config|-c)
      CONFIG="$2"
      shift 2
      ;;
    --rviz)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

PKG_PREFIX="$(ros2 pkg prefix rviz_bag_tools)"
PKG_SHARE="$PKG_PREFIX/share/rviz_bag_tools"

if [ -z "$CONFIG" ]; then
  CONFIG="$PKG_SHARE/config/live_remote.yaml"
fi

if [ -f "$CONFIG" ]; then
  ENV_EXPORTS="$(python3 - "$CONFIG" <<'PY'
import shlex
import sys
from pathlib import Path
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
live = data.get("live", {}) or {}
env = live.get("env", {}) or {}
for key, value in env.items():
    if value is None or str(value) == "":
        continue
    print(f"export {key}={shlex.quote(str(value))}")
rviz = live.get("rviz_config")
if rviz:
    print(f"export RVIZ_BAG_TOOLS_CONFIG_RVIZ={shlex.quote(str(rviz))}")
adapter = live.get("adapter", {}) or {}
for key, value in adapter.items():
    if value is None:
        continue
    env_key = "RVIZ_BAG_TOOLS_ADAPTER_" + key.upper()
    if isinstance(value, bool):
        value = "true" if value else "false"
    print(f"export {env_key}={shlex.quote(str(value))}")
PY
)"
  eval "$ENV_EXPORTS"
fi

if [ -z "$RVIZ_CONFIG" ]; then
  RVIZ_CONFIG="${RVIZ_BAG_TOOLS_CONFIG_RVIZ:-rviz/live_chassis.rviz}"
fi

if [[ "$RVIZ_CONFIG" != /* ]]; then
  RVIZ_CONFIG="$PKG_SHARE/$RVIZ_CONFIG"
fi

echo "=========================================="
echo "  Live RViz viewer"
echo "=========================================="
echo "Config: $CONFIG"
echo "RViz:   $RVIZ_CONFIG"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo ""

if [ "${RVIZ_BAG_TOOLS_ADAPTER_ENABLED:-false}" = "true" ]; then
  ADAPTER_CMD=(
    ros2 run rviz_bag_tools live_odom_adapter.py
    --input-odom "${RVIZ_BAG_TOOLS_ADAPTER_INPUT_ODOM:-/odom}"
    --output-odom "${RVIZ_BAG_TOOLS_ADAPTER_OUTPUT_ODOM:-/remote_view/odom}"
    --path-topic "${RVIZ_BAG_TOOLS_ADAPTER_PATH_TOPIC:-/remote_view/path}"
    --frame-id "${RVIZ_BAG_TOOLS_ADAPTER_FRAME_ID:-remote_odom}"
    --child-frame-id "${RVIZ_BAG_TOOLS_ADAPTER_CHILD_FRAME_ID:-remote_base_link}"
    --max-path-poses "${RVIZ_BAG_TOOLS_ADAPTER_MAX_PATH_POSES:-5000}"
    --stamp-policy "${RVIZ_BAG_TOOLS_ADAPTER_STAMP_POLICY:-preserve}"
    --qos-reliability "${RVIZ_BAG_TOOLS_ADAPTER_QOS_RELIABILITY:-reliable}"
    --qos-depth "${RVIZ_BAG_TOOLS_ADAPTER_QOS_DEPTH:-50}"
  )
  if [ "${RVIZ_BAG_TOOLS_ADAPTER_PUBLISH_TF:-true}" != "true" ]; then
    ADAPTER_CMD+=(--no-tf)
  fi
  "${ADAPTER_CMD[@]}" >/tmp/rviz_bag_tools_live_adapter.log 2>&1 &
  ADAPTER_PID=$!
  echo "Live odom adapter started, pid=$ADAPTER_PID"
fi

cleanup() {
  if [ -n "$ADAPTER_PID" ]; then
    kill "$ADAPTER_PID" 2>/dev/null || true
    wait "$ADAPTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rviz2 -d "$RVIZ_CONFIG"
