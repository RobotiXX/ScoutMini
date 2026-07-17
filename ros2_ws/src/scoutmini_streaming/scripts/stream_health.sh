#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCOUT_REPO_ROOT="${SCOUT_REPO_ROOT:-${REPO_ROOT:-}}"
if [[ -z "$SCOUT_REPO_ROOT" ]]; then
  SCOUT_REPO_ROOT="$(git -C "$PACKAGE_ROOT" rev-parse --show-toplevel 2>/dev/null || true)"
fi
SCOUT_REPO_ROOT="${SCOUT_REPO_ROOT:-$HOME/repos/ScoutMini}"
SCOUT_WS="${SCOUT_WS:-$SCOUT_REPO_ROOT/ros2_ws}"
ZED_IMAGE_TOPIC="${ZED_IMAGE_TOPIC:-/zed2/zed_node/rgb/color/rect/image/compressed}"
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/zed}"
WEBRTC_HEALTH_URL="${WEBRTC_HEALTH_URL:-http://127.0.0.1:8889/zed/}"
CAMERA_TIMEOUT="${CAMERA_TIMEOUT:-8}"
RTSP_TIMEOUT="${RTSP_TIMEOUT:-8}"
WEBRTC_TIMEOUT="${WEBRTC_TIMEOUT:-3}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || return 1
  [[ -f "$SCOUT_WS/install/setup.bash" ]] || return 1
  set +u
  source /opt/ros/humble/setup.bash
  source "$SCOUT_WS/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION
}

camera_ready() {
  source_ros || return 1
  timeout "$CAMERA_TIMEOUT" ros2 topic echo --no-daemon --once \
    --qos-profile sensor_data --no-arr "$ZED_IMAGE_TOPIC" >/dev/null 2>&1
}

rtsp_ready() {
  command -v ffprobe >/dev/null 2>&1 || return 1
  local probe_output
  probe_output="$(timeout "$RTSP_TIMEOUT" ffprobe -v error -rtsp_transport tcp \
    -select_streams v:0 -show_entries stream=codec_name \
    -of default=noprint_wrappers=1:nokey=1 "$RTSP_URL" 2>/dev/null || true)"
  grep -Fxq "h264" <<< "$probe_output"
}

webrtc_ready() {
  command -v curl >/dev/null 2>&1 || return 1
  curl --fail --silent --show-error --max-time "$WEBRTC_TIMEOUT" \
    "$WEBRTC_HEALTH_URL" >/dev/null 2>&1
}

report_probe() {
  local stage="$1"
  local success_message="$2"
  local failure_message="$3"
  shift 3

  if "$@"; then
    printf 'PASS %-18s %s\n' "$stage" "$success_message"
    return 0
  fi

  printf 'FAIL %-18s %s\n' "$stage" "$failure_message"
  return 1
}

print_status() {
  local status=0

  report_probe \
    "CAMERA_TOPIC" \
    "fresh frame received from $ZED_IMAGE_TOPIC" \
    "no fresh frame from $ZED_IMAGE_TOPIC within ${CAMERA_TIMEOUT}s" \
    camera_ready || status=1
  report_probe \
    "RTSP_MEDIA" \
    "H.264 video detected at $RTSP_URL" \
    "no H.264 video detected at $RTSP_URL within ${RTSP_TIMEOUT}s" \
    rtsp_ready || status=1
  report_probe \
    "WEBRTC_HTTP" \
    "gateway responded at $WEBRTC_HEALTH_URL" \
    "gateway did not respond at $WEBRTC_HEALTH_URL within ${WEBRTC_TIMEOUT}s" \
    webrtc_ready || status=1

  return "$status"
}

case "${1:---status}" in
  --camera) camera_ready ;;
  --rtsp) rtsp_ready ;;
  --webrtc) webrtc_ready ;;
  --ready) camera_ready && rtsp_ready && webrtc_ready ;;
  --status) print_status ;;
  *) echo "Usage: $0 {--camera|--rtsp|--webrtc|--ready|--status}" >&2; exit 2 ;;
esac
