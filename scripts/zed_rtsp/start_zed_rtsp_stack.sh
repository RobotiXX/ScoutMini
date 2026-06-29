#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
RTSP_WS="${RTSP_WS:-/home/nvidia/image2rtsp_ws}"
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

if [[ ! -f "$RTSP_WS/install/setup.bash" ]]; then
  echo "Missing $RTSP_WS/install/setup.bash. Build image2rtsp first." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${RTSP_PID:-}" ]]; then
    kill "$RTSP_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "${ZED_PID:-}" ]]; then
    kill "$ZED_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION

echo "Starting ZED wrapper..."
ros2 launch zed_wrapper zed_camera.launch.py camera_model:="$CAMERA_MODEL" camera_name:="$CAMERA_NAME" &
ZED_PID="$!"

sleep 8

echo "Starting image2rtsp..."
(
  cd "$RTSP_WS"
  set +u
  source install/setup.bash
  set -u
  ros2 launch image2rtsp image2rtsp.launch.py
) &
RTSP_PID="$!"

echo "ZED RTSP stack started."
echo "View on LAN: rtsp://<robot-ip>:8554/zed"

wait -n "$ZED_PID" "$RTSP_PID"
