#!/usr/bin/env bash
set -euo pipefail

SCOUT_REPO_ROOT="${SCOUT_REPO_ROOT:-${REPO_ROOT:-$HOME/repos/ScoutMini}}"
SCOUT_WS="${SCOUT_WS:-$SCOUT_REPO_ROOT/ros2_ws}"
CAMERA_TIMEOUT="${CAMERA_TIMEOUT:-90}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export SCOUT_REPO_ROOT SCOUT_WS RMW_IMPLEMENTATION

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble is not installed." >&2; exit 1; }
[[ -f "$SCOUT_WS/install/setup.bash" ]] || { echo "Workspace is not built: $SCOUT_WS" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/install/setup.bash"
set -u

cleanup() {
  [[ -n "${BRIDGE_PID:-}" ]] && kill "$BRIDGE_PID" >/dev/null 2>&1 || true
  [[ -n "${CAMERA_PID:-}" ]] && kill "$CAMERA_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 camera_name:=zed2 &
CAMERA_PID=$!

for ((second = 0; second < CAMERA_TIMEOUT; second++)); do
  if ros2 run scoutmini_streaming stream_health --camera >/dev/null 2>&1; then
    break
  fi
  kill -0 "$CAMERA_PID" >/dev/null 2>&1 || { echo "ZED launch exited." >&2; exit 1; }
  sleep 1
done

ros2 run scoutmini_streaming stream_health --camera || {
  echo "ZED did not publish a frame within ${CAMERA_TIMEOUT}s." >&2
  exit 1
}

ros2 launch scoutmini_streaming image2rtsp.launch.py &
BRIDGE_PID=$!
wait "$BRIDGE_PID"
