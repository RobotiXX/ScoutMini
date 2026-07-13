#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$PACKAGE_ROOT/../../.." && pwd)}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
RTSP_SCRIPT="${RTSP_SCRIPT:-$PACKAGE_ROOT/scripts/start_zed_rtsp_stack.sh}"
MEDIAMTX_CONFIG="${MEDIAMTX_CONFIG:-$PACKAGE_ROOT/config/mediamtx_zed_webrtc.yml}"
IMAGE2RTSP_CONFIG="${IMAGE2RTSP_CONFIG:-$PACKAGE_ROOT/config/image2rtsp_zed.yaml}"
LOCAL_MEDIAMTX_BIN="$PACKAGE_ROOT/test/tools/mediamtx-1.19.2-linux-arm64/mediamtx"
LEGACY_LOCAL_MEDIAMTX_BIN="$PACKAGE_ROOT/test/tools/mediamtx-v1.19.2-linux-arm64/mediamtx"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-$LOCAL_MEDIAMTX_BIN}"
WEBRTC_URL_PATH="${WEBRTC_URL_PATH:-zed/}"
START_RTSP="${START_RTSP:-1}"
RTSP_HOST="${RTSP_HOST:-127.0.0.1}"
RTSP_PORT="${RTSP_PORT:-8554}"
WEBRTC_PORT="${WEBRTC_PORT:-8889}"
REUSE_EXISTING_ZED="${REUSE_EXISTING_ZED:-1}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ZED_IMAGE_TOPICS="${ZED_IMAGE_TOPICS:-/zed2/zed_node/rgb/color/rect/image/compressed /zed/zed_node/rgb/color/rect/image/compressed}"

if [[ ! -x "$MEDIAMTX_BIN" ]]; then
  if [[ -x "$LEGACY_LOCAL_MEDIAMTX_BIN" ]]; then
    MEDIAMTX_BIN="$LEGACY_LOCAL_MEDIAMTX_BIN"
  elif command -v mediamtx >/dev/null 2>&1; then
    MEDIAMTX_BIN="$(command -v mediamtx)"
  else
    echo "Missing mediamtx. Expected local binary at: $LOCAL_MEDIAMTX_BIN" >&2
    echo "Also checked legacy local binary at: $LEGACY_LOCAL_MEDIAMTX_BIN" >&2
    echo "Or set MEDIAMTX_BIN=/path/to/mediamtx." >&2
    exit 1
  fi
fi

if [[ ! -f "$MEDIAMTX_CONFIG" ]]; then
  echo "Missing MediaMTX config: $MEDIAMTX_CONFIG" >&2
  exit 1
fi

if [[ ! -f "$IMAGE2RTSP_CONFIG" ]]; then
  echo "Missing image2rtsp config: $IMAGE2RTSP_CONFIG" >&2
  exit 1
fi

if [[ "$START_RTSP" != "0" && ! -x "$RTSP_SCRIPT" ]]; then
  echo "Missing executable RTSP stack script: $RTSP_SCRIPT" >&2
  exit 1
fi

cleanup() {
  if [[ -n "${MEDIAMTX_PID:-}" ]]; then
    kill "$MEDIAMTX_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "${RTSP_PID:-}" ]]; then
    kill "$RTSP_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_tcp() {
  local host="$1"
  local port="$2"
  local timeout_s="$3"
  local start_s
  start_s="$(date +%s)"

  while true; do
    if timeout 1 bash -c ":</dev/tcp/${host}/${port}" >/dev/null 2>&1; then
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      return 1
    fi

    sleep 1
  done
}

source_ros() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "Missing /opt/ros/humble/setup.bash. Install ROS 2 Humble first." >&2
    return 1
  fi
  if [[ ! -f "$SCOUT_WS/install/setup.bash" ]]; then
    echo "Missing $SCOUT_WS/install/setup.bash. Build the ScoutMini ROS workspace first." >&2
    return 1
  fi

  set +u
  source /opt/ros/humble/setup.bash
  source "$SCOUT_WS/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION
}

find_existing_zed_topic() {
  local topics
  topics="$(timeout 5 ros2 topic list --no-daemon 2>/dev/null || true)"
  [[ -n "$topics" ]] || return 1

  local candidate
  for candidate in $ZED_IMAGE_TOPICS; do
    if grep -Fxq "$candidate" <<<"$topics"; then
      printf "%s\n" "$candidate"
      return 0
    fi
  done

  return 1
}

start_image2rtsp_for_topic() {
  local topic="$1"

  (
    cd "$REPO_ROOT"
    source_ros
    echo "Starting image2rtsp from existing ZED topic: $topic"
    ros2 run image2rtsp image2rtsp --ros-args \
      --params-file "$IMAGE2RTSP_CONFIG" \
      -p topic:="$topic"
  ) &
  RTSP_PID="$!"
}

if [[ "$START_RTSP" != "0" ]]; then
  existing_zed_topic=""
  if [[ "$REUSE_EXISTING_ZED" != "0" ]]; then
    source_ros
    existing_zed_topic="$(find_existing_zed_topic || true)"
  fi

  if [[ -n "$existing_zed_topic" ]]; then
    echo "Reusing existing ZED topic; not starting a second ZED wrapper."
    start_image2rtsp_for_topic "$existing_zed_topic"
  else
    echo "Starting ZED RTSP stack..."
    "$RTSP_SCRIPT" &
    RTSP_PID="$!"
  fi

  echo "Waiting for RTSP on ${RTSP_HOST}:${RTSP_PORT}..."
  if ! wait_for_tcp "$RTSP_HOST" "$RTSP_PORT" 45; then
    echo "RTSP did not become reachable on ${RTSP_HOST}:${RTSP_PORT}." >&2
    exit 1
  fi
else
  echo "START_RTSP=0; expecting existing RTSP stream on ${RTSP_HOST}:${RTSP_PORT}."
fi

echo "Starting MediaMTX WebRTC gateway..."
"$MEDIAMTX_BIN" "$MEDIAMTX_CONFIG" &
MEDIAMTX_PID="$!"

echo "ZED WebRTC stack started."
echo "Open: http://<robot_ip_or_tailscale_ip>:${WEBRTC_PORT}/${WEBRTC_URL_PATH}"
echo "RTSP fallback: rtsp://<robot_ip>:${RTSP_PORT}/zed"

PIDS=("$MEDIAMTX_PID")
if [[ -n "${RTSP_PID:-}" ]]; then
  PIDS+=("$RTSP_PID")
fi

wait -n "${PIDS[@]}"
