#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
BAG_ROOT="${BAG_ROOT:-$REPO_ROOT/scripts/zed_rtsp/rosbags}"
DURATION_SECONDS="${DURATION_SECONDS:-60}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$BAG_ROOT/zed_debug_$STAMP}"

TOPICS=(
  "${ZED_RGB_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"
  "${ZED_DEPTH_TOPIC:-/zed/zed_node/depth/depth_registered}"
  "/tf"
  "/tf_static"
)

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$SCOUT_WS/install/setup.bash" ]]; then
  echo "Missing ROS setup files." >&2
  exit 1
fi

mkdir -p "$BAG_ROOT"

set +u
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "Recording ZED debug bag for ${DURATION_SECONDS}s..."
echo "Output: $OUT_DIR"
printf "Topics:\n"
printf "  %s\n" "${TOPICS[@]}"

timeout --preserve-status "$DURATION_SECONDS" ros2 bag record -o "$OUT_DIR" "${TOPICS[@]}"
status="$?"

if [[ "$status" -eq 124 || "$status" -eq 143 ]]; then
  echo "Bag recording stopped after timeout."
  exit 0
fi

exit "$status"
