#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
RTSP_SCRIPT="${RTSP_SCRIPT:-$REPO_ROOT/scripts/zed_rtsp/start_zed_rtsp_stack.sh}"
MEDIAMTX_CONFIG="${MEDIAMTX_CONFIG:-$REPO_ROOT/scripts/zed_rtsp/mediamtx_zed_webrtc.yml}"
LOCAL_MEDIAMTX_BIN="$REPO_ROOT/scripts/zed_rtsp/tools/mediamtx-v1.19.2-linux-arm64/mediamtx"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-$LOCAL_MEDIAMTX_BIN}"
WEBRTC_URL_PATH="${WEBRTC_URL_PATH:-zed/}"
START_RTSP="${START_RTSP:-1}"
RTSP_HOST="${RTSP_HOST:-127.0.0.1}"
RTSP_PORT="${RTSP_PORT:-8554}"
WEBRTC_PORT="${WEBRTC_PORT:-8889}"

if [[ ! -x "$MEDIAMTX_BIN" ]]; then
  if command -v mediamtx >/dev/null 2>&1; then
    MEDIAMTX_BIN="$(command -v mediamtx)"
  else
    echo "Missing mediamtx. Expected local binary at: $LOCAL_MEDIAMTX_BIN" >&2
    echo "Or set MEDIAMTX_BIN=/path/to/mediamtx." >&2
    exit 1
  fi
fi

if [[ ! -f "$MEDIAMTX_CONFIG" ]]; then
  echo "Missing MediaMTX config: $MEDIAMTX_CONFIG" >&2
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

if [[ "$START_RTSP" != "0" ]]; then
  echo "Starting ZED RTSP stack..."
  "$RTSP_SCRIPT" &
  RTSP_PID="$!"

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
echo "Open: http://<robot-ip-or-tailscale-ip>:${WEBRTC_PORT}/${WEBRTC_URL_PATH}"
echo "RTSP fallback: rtsp://<robot-ip>:${RTSP_PORT}/zed"

PIDS=("$MEDIAMTX_PID")
if [[ -n "${RTSP_PID:-}" ]]; then
  PIDS+=("$RTSP_PID")
fi

wait -n "${PIDS[@]}"
