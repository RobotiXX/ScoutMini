#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$PACKAGE_ROOT/../../.." && pwd)}"
STREAMING_ROOT="${STREAMING_ROOT:-$REPO_ROOT/ros2_ws/src/scoutmini_streaming}"
LEGACY_STREAMING_ROOT="$REPO_ROOT/scripts/zed_rtsp"

if [[ -x "$STREAMING_ROOT/scripts/start_zed_webrtc_stack.sh" ]]; then
  DEFAULT_START_SCRIPT="$STREAMING_ROOT/scripts/start_zed_webrtc_stack.sh"
  DEFAULT_STOP_SCRIPT="$STREAMING_ROOT/scripts/stop_zed_stream_stack.sh"
  DEFAULT_CHECK_SCRIPT="$STREAMING_ROOT/scripts/check_zed_stream_stack.sh"
elif [[ -x "$LEGACY_STREAMING_ROOT/start_zed_webrtc_stack.sh" ]]; then
  DEFAULT_START_SCRIPT="$LEGACY_STREAMING_ROOT/start_zed_webrtc_stack.sh"
  DEFAULT_STOP_SCRIPT="$LEGACY_STREAMING_ROOT/stop_zed_stream_stack.sh"
  DEFAULT_CHECK_SCRIPT="$LEGACY_STREAMING_ROOT/check_zed_stream_stack.sh"
else
  DEFAULT_START_SCRIPT="$STREAMING_ROOT/scripts/start_zed_webrtc_stack.sh"
  DEFAULT_STOP_SCRIPT="$STREAMING_ROOT/scripts/stop_zed_stream_stack.sh"
  DEFAULT_CHECK_SCRIPT="$STREAMING_ROOT/scripts/check_zed_stream_stack.sh"
fi

START_SCRIPT="${START_SCRIPT:-$DEFAULT_START_SCRIPT}"
STOP_SCRIPT="${STOP_SCRIPT:-$DEFAULT_STOP_SCRIPT}"
CHECK_SCRIPT="${CHECK_SCRIPT:-$DEFAULT_CHECK_SCRIPT}"
LOG_DIR="${LOG_DIR:-$PACKAGE_ROOT/logs/slack_stream_control}"
PID_FILE="${PID_FILE:-$LOG_DIR/zed_webrtc_stack.pid}"
LOG_FILE="${LOG_FILE:-$LOG_DIR/zed_webrtc_stack.log}"
RECENT_LOG_LINES="${RECENT_LOG_LINES:-0}"
WEBRTC_PORT="${WEBRTC_PORT:-8889}"
RTSP_PORT="${RTSP_PORT:-8554}"
ICE_PORT="${ICE_PORT:-8189}"
SCOUT_STREAM_URL="${SCOUT_STREAM_URL:-http://<robot_ip_or_tailscale_ip>:8889/zed/}"

usage() {
  cat <<EOF
Usage: $0 {status|start|stop|diagnostics}

Controls only the ZED RTSP/WebRTC stream stack. It does not start navigation,
teleop, mapping, task execution, or autonomous motion.
EOF
}

is_port_listening() {
  local port="$1"
  ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq ":${port}$"
}

print_status() {
  local state="offline"
  if is_port_listening "$WEBRTC_PORT"; then
    state="online"
  fi

  echo "ZED WebRTC stream: $state"
  echo "URL: $SCOUT_STREAM_URL"
  echo
  echo "Stream ports:"
  ss -ltnup 2>/dev/null | grep -E ":${RTSP_PORT}|:${WEBRTC_PORT}|:${ICE_PORT}" || echo "none"

  if [[ -f "$PID_FILE" ]]; then
    echo
    echo "PID file: $PID_FILE ($(cat "$PID_FILE" 2>/dev/null || true))"
  fi
}

start_stream() {
  mkdir -p "$LOG_DIR"

  if is_port_listening "$WEBRTC_PORT"; then
    echo "ZED WebRTC stream is already online."
    print_status
    return 0
  fi

  if [[ ! -x "$START_SCRIPT" ]]; then
    echo "Missing executable start script: $START_SCRIPT" >&2
    return 1
  fi

  echo "Starting ZED RTSP/WebRTC stack in the background..."
  echo "Log: $LOG_FILE"

  if command -v setsid >/dev/null 2>&1; then
    (
      cd "$REPO_ROOT"
      exec nohup setsid "$START_SCRIPT" >>"$LOG_FILE" 2>&1 </dev/null
    ) &
  else
    (
      cd "$REPO_ROOT"
      exec nohup "$START_SCRIPT" >>"$LOG_FILE" 2>&1 </dev/null
    ) &
  fi
  local pid="$!"
  echo "$pid" >"$PID_FILE"

  local waited=0
  while (( waited < 75 )); do
    if is_port_listening "$WEBRTC_PORT"; then
      echo "ZED WebRTC stream started."
      print_status
      return 0
    fi

    if ! kill -0 "$pid" >/dev/null 2>&1; then
      echo "ZED stream start process exited before WebRTC became available." >&2
      tail -40 "$LOG_FILE" 2>/dev/null || true
      return 1
    fi

    sleep 1
    waited=$((waited + 1))
  done

  echo "Timed out waiting for WebRTC port ${WEBRTC_PORT}." >&2
  tail -40 "$LOG_FILE" 2>/dev/null || true
  return 1
}

stop_stream() {
  if [[ ! -x "$STOP_SCRIPT" ]]; then
    echo "Missing executable stop script: $STOP_SCRIPT" >&2
    return 1
  fi

  "$STOP_SCRIPT"
  rm -f "$PID_FILE"
}

diagnostics() {
  print_status

  echo
  echo "Diagnostics:"
  if [[ -x "$CHECK_SCRIPT" ]]; then
    "$CHECK_SCRIPT"
  else
    echo "Missing diagnostics script: $CHECK_SCRIPT"
  fi

  if [[ "$RECENT_LOG_LINES" != "0" && -f "$LOG_FILE" ]]; then
    echo
    echo "Recent stream log:"
    tail "-$RECENT_LOG_LINES" "$LOG_FILE"
  fi
}

command="${1:-status}"
case "$command" in
  status)
    print_status
    ;;
  start)
    start_stream
    ;;
  stop)
    stop_stream
    ;;
  diagnostics|diag)
    diagnostics
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
