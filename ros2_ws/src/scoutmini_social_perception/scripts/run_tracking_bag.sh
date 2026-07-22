#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 5 ]]; then
  echo "Usage: $0 SOURCE_BAG OUTPUT_DIR [DOMAIN_ID] [START_SEC] [DURATION_SEC]" >&2
  exit 2
fi

source_bag=$(realpath "$1")
output_dir=$(realpath -m "$2")
domain_id=${3:-88}
start_sec=${4:-0}
duration_sec=${5:-0}
scene=$(basename "${source_bag}")
analysis_bag="${output_dir}/tracking_bag"
log_dir="${output_dir}/logs"
model_path=${SCOUTMINI_YOLO_MODEL:-${HOME}/models/yolo/yolo11n_960.engine}
reid_model_path=${SCOUTMINI_REID_MODEL:-${HOME}/models/yolo/yolo26n-cls.pt}
playback_rate=${SCOUTMINI_PLAYBACK_RATE:-1.0}
target_fps=${SCOUTMINI_TARGET_FPS:-8.0}
image_size=${SCOUTMINI_IMGSZ:-960}
confidence_threshold=${SCOUTMINI_CONFIDENCE_THRESHOLD:-0.35}
iou_threshold=${SCOUTMINI_IOU_THRESHOLD:-0.45}
tracker_config=${SCOUTMINI_TRACKER_CONFIG:-}

if [[ ! -f "${source_bag}/metadata.yaml" ]]; then
  echo "Not a rosbag2 directory: ${source_bag}" >&2
  exit 2
fi
if [[ ! -f "${model_path}" ]] || [[ ! -f "${reid_model_path}" ]]; then
  echo "Detector or ReID model is missing" >&2
  exit 2
fi
if [[ -e "${analysis_bag}" ]]; then
  echo "Refusing to overwrite analysis bag: ${analysis_bag}" >&2
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
  sleep 2
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  stop_group "${record_pid}"
  stop_group "${launch_pid}"
}
trap cleanup EXIT INT TERM

launch_arguments=(
  model_path:="${model_path}"
  reid_model_path:="${reid_model_path}"
  image_topic:=/insta360_x4/image_raw/compressed
  input_type:=compressed
  device:=0
  target_fps:="${target_fps}"
  imgsz:="${image_size}"
  confidence_threshold:="${confidence_threshold}"
  iou_threshold:="${iou_threshold}"
  publish_debug_image:=false
  use_sim_time:=true
)
if [[ -n "${tracker_config}" ]]; then
  launch_arguments+=(tracker_config:="${tracker_config}")
fi
setsid ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  "${launch_arguments[@]}" \
  >"${log_dir}/pipeline.log" 2>&1 &
launch_pid=$!

sleep 10
setsid ros2 bag record -o "${analysis_bag}" \
  /people/tracks_2d \
  /people/detector_diagnostics \
  >"${log_dir}/record.log" 2>&1 &
record_pid=$!

sleep 2
play_arguments=(
  "${source_bag}"
  --clock
  --rate "${playback_rate}"
  --start-offset "${start_sec}"
  --topics /insta360_x4/image_raw/compressed
)
if [[ "${duration_sec}" == "0" ]]; then
  ros2 bag play "${play_arguments[@]}" >"${log_dir}/playback.log" 2>&1
else
  wall_duration=$(awk \
    -v duration="${duration_sec}" \
    -v rate="${playback_rate}" \
    'BEGIN {printf "%.3f", duration / rate}')
  set +e
  timeout --signal=INT --kill-after=10 "${wall_duration}s" \
    ros2 bag play "${play_arguments[@]}" \
    >"${log_dir}/playback.log" 2>&1
  playback_status=$?
  set -e
  if [[ ${playback_status} -ne 0 && ${playback_status} -ne 124 ]]; then
    echo "Bag playback failed with exit ${playback_status}" >&2
    exit "${playback_status}"
  fi
fi

sleep 2
cleanup
trap - EXIT INT TERM

ros2 run scoutmini_social_perception evaluate_tracking_bag \
  --source-bag "${source_bag}" \
  --analysis-bag "${analysis_bag}" \
  --output-dir "${output_dir}" \
  --scene "${scene}" \
  --fps "${target_fps}" \
  >"${log_dir}/evaluation.log"
