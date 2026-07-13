#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$PACKAGE_ROOT/../../.." && pwd)}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
CAMERA_MODEL="${CAMERA_MODEL:-zed2}"
CAMERA_NAME="${CAMERA_NAME:-zed}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash. Install ROS 2 Humble first." >&2
  exit 1
fi

if [[ ! -f "$SCOUT_WS/install/setup.bash" ]]; then
  echo "Missing $SCOUT_WS/install/setup.bash. Build the ScoutMini ROS workspace first." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION

echo "Starting ZED wrapper and image2rtsp..."
echo "View on LAN: rtsp://<robot_ip>:8554/zed"
ros2 launch scoutmini_streaming zed_rtsp.launch.py \
  camera_model:="$CAMERA_MODEL" \
  camera_name:="$CAMERA_NAME"
