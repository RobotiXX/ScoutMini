#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MEDIAMTX_CONFIG="${MEDIAMTX_CONFIG:-$PACKAGE_ROOT/config/mediamtx_zed_webrtc.yml}"
HEALTH_SCRIPT="${HEALTH_SCRIPT:-$SCRIPT_DIR/stream_health.sh}"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-$HOME/.local/lib/scoutmini/mediamtx/mediamtx}"
STATE_DIR="${SCOUTMINI_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp}/scoutmini}"
PID_FILE="$STATE_DIR/webrtc.pid"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-15}"
ZED_IMAGE_TOPIC="${ZED_IMAGE_TOPIC:-/zed2/zed_node/rgb/color/rect/image/compressed}"
CAMERA_TIMEOUT="${CAMERA_TIMEOUT:-10}"
RTSP_TIMEOUT="${RTSP_TIMEOUT:-10}"
PREFLIGHT_ATTEMPTS="${PREFLIGHT_ATTEMPTS:-3}"
export CAMERA_TIMEOUT RTSP_TIMEOUT ZED_IMAGE_TOPIC

[[ -x "$MEDIAMTX_BIN" ]] || {
  echo "Missing MediaMTX binary: $MEDIAMTX_BIN" >&2
  echo "Run scoutmini_streaming/test/install_mediamtx_local.sh." >&2
  exit 1
}
[[ -f "$MEDIAMTX_CONFIG" ]] || { echo "Missing config: $MEDIAMTX_CONFIG" >&2; exit 1; }
[[ -x "$HEALTH_SCRIPT" ]] || { echo "Missing health script: $HEALTH_SCRIPT" >&2; exit 1; }

wait_for_preflight() {
  local probe="$1"
  local label="$2"
  local attempt
  for ((attempt = 1; attempt <= PREFLIGHT_ATTEMPTS; attempt++)); do
    if "$HEALTH_SCRIPT" "$probe"; then
      return 0
    fi
    if (( attempt < PREFLIGHT_ATTEMPTS )); then
      echo "$label preflight attempt $attempt failed; retrying..." >&2
      sleep 1
    fi
  done
  return 1
}

echo "Checking externally managed ZED camera..."
wait_for_preflight --camera "ZED camera" || {
  echo "No frame received from $ZED_IMAGE_TOPIC." >&2
  echo "Start robot camera bringup before WebRTC." >&2
  exit 1
}

echo "Checking externally managed image2rtsp stream..."
wait_for_preflight --rtsp "RTSP" || {
  echo "RTSP is not producing an H.264 video track at rtsp://127.0.0.1:8554/zed." >&2
  echo "Start the scoutmini_streaming image2rtsp launch before WebRTC." >&2
  exit 1
}

mkdir -p "$STATE_DIR"
cleanup() {
  rm -f "$PID_FILE"
  if [[ -n "${MEDIAMTX_PID:-}" ]]; then
    kill "$MEDIAMTX_PID" >/dev/null 2>&1 || true
    wait "$MEDIAMTX_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$MEDIAMTX_BIN" "$MEDIAMTX_CONFIG" &
MEDIAMTX_PID=$!
printf '%s\n' "$MEDIAMTX_PID" > "$PID_FILE"

for ((second = 0; second < STARTUP_TIMEOUT; second++)); do
  if "$HEALTH_SCRIPT" --webrtc; then
    echo "WebRTC ready: http://<robot_ip_or_tailscale_ip>:8889/zed/"
    wait "$MEDIAMTX_PID"
    exit $?
  fi
  kill -0 "$MEDIAMTX_PID" >/dev/null 2>&1 || {
    echo "MediaMTX exited during startup." >&2
    wait "$MEDIAMTX_PID" || true
    exit 1
  }
  sleep 1
done

echo "MediaMTX did not expose the WebRTC endpoint within ${STARTUP_TIMEOUT}s." >&2
exit 1
