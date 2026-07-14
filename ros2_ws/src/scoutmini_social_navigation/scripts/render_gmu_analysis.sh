#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 SOURCE_BAG OUTPUT_DIR [ROS_DOMAIN_ID]" >&2
  exit 2
fi

source_bag=$(realpath "$1")
output_dir=$(realpath -m "$2")
domain_id=${3:-87}
scene=$(basename "${source_bag}")
analysis_bag="${output_dir}/${scene}_analysis_bag"
log_dir="${output_dir}/logs"

if [[ ! -f "${source_bag}/metadata.yaml" ]]; then
  echo "Not a rosbag2 directory: ${source_bag}" >&2
  exit 2
fi
if [[ -e "${analysis_bag}" ]]; then
  echo "Refusing to overwrite existing analysis bag: ${analysis_bag}" >&2
  exit 2
fi
if ! command -v ffmpeg >/dev/null; then
  echo "ffmpeg is required" >&2
  exit 2
fi

mkdir -p "${output_dir}" "${log_dir}"
export ROS_DOMAIN_ID="${domain_id}"

launch_pid=''
record_pid=''
stop_group() {
  local pid=$1
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in {1..100}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in {1..50}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  stop_group "${record_pid}"
  stop_group "${launch_pid}"
}
trap cleanup EXIT INT TERM

setsid ros2 launch scoutmini_social_navigation gmu_analysis_pipeline.launch.py \
  >"${log_dir}/pipeline.log" 2>&1 &
launch_pid=$!

sleep 10
setsid ros2 bag record -o "${analysis_bag}" \
  /people/tracks_2d \
  /people/detector_diagnostics \
  /people/fusion_diagnostics \
  /adascore/shadow/diagnostics \
  /adascore/shadow/people \
  /adascore/shadow/cmd_vel \
  /adascore_shadow/robot_local_trajectories \
  /adascore_shadow/robot_local_plan \
  /sfm/markers/obstacle_points \
  >"${log_dir}/record.log" 2>&1 &
record_pid=$!

sleep 2
ros2 bag play "${source_bag}" --clock --rate 0.5 --topics \
  /insta360_x4/image_raw/compressed \
  /insta360_x4/camera_info \
  /velodyne_points \
  /odom \
  >"${log_dir}/playback.log" 2>&1

sleep 2
cleanup
trap - EXIT INT TERM

ros2 run scoutmini_social_navigation render_bag_analysis \
  --source-bag "${source_bag}" \
  --analysis-bag "${analysis_bag}" \
  --output-dir "${output_dir}" \
  --scene "${scene}" \
  --fps 8.0 \
  | tee "${log_dir}/render.log"
