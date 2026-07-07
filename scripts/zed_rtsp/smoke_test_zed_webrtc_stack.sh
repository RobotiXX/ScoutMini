#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
STACK_SCRIPT="${STACK_SCRIPT:-$REPO_ROOT/scripts/zed_rtsp/start_zed_webrtc_stack.sh}"
TEST_SECONDS="${TEST_SECONDS:-60}"
WEBRTC_HOST="${WEBRTC_HOST:-127.0.0.1}"
WEBRTC_PORT="${WEBRTC_PORT:-8889}"
RTSP_PORT="${RTSP_PORT:-8554}"
ICE_PORT="${ICE_PORT:-8189}"
VIEWER_PATH="${VIEWER_PATH:-/zed/}"

if [[ ! -x "$STACK_SCRIPT" ]]; then
  echo "Missing executable stack script: $STACK_SCRIPT" >&2
  exit 1
fi

wait_for_http() {
  local url="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"

  while true; do
    if curl -fsS -o /dev/null "$url"; then
      return 0
    fi

    if (( "$(date +%s)" - start_s >= timeout_s )); then
      return 1
    fi

    sleep 1
  done
}

echo "Starting bounded ZED WebRTC smoke test for ${TEST_SECONDS}s..."
timeout --preserve-status "$TEST_SECONDS" "$STACK_SCRIPT" &
STACK_PID="$!"

VIEWER_URL="http://${WEBRTC_HOST}:${WEBRTC_PORT}${VIEWER_PATH}"
echo "Waiting for viewer endpoint: $VIEWER_URL"
if ! wait_for_http "$VIEWER_URL" 45; then
  echo "WebRTC viewer endpoint did not become reachable." >&2
  kill "$STACK_PID" >/dev/null 2>&1 || true
  wait "$STACK_PID" >/dev/null 2>&1 || true
  exit 1
fi

echo "Viewer endpoint is reachable."
echo "Expected listeners:"
ss -ltnup | grep -E ":${RTSP_PORT}|:${WEBRTC_PORT}|:${ICE_PORT}" || true

echo
echo "Open from a laptop on LAN or Tailscale:"
echo "  http://<robot-ip-or-tailscale-ip>:${WEBRTC_PORT}${VIEWER_PATH}"
echo
echo "Waiting for bounded test to finish..."

set +e
wait "$STACK_PID"
STATUS="$?"
set -e

if [[ "$STATUS" -eq 143 || "$STATUS" -eq 124 ]]; then
  echo "Smoke test timeout reached; stack stopped as expected."
  exit 0
fi

if [[ "$STATUS" -ne 0 ]]; then
  echo "Smoke test exited with status $STATUS." >&2
  exit "$STATUS"
fi

echo "Smoke test completed."
