#!/usr/bin/env bash
set -euo pipefail

SCOUT_REPO_ROOT="${SCOUT_REPO_ROOT:-${REPO_ROOT:-$HOME/repos/ScoutMini}}"
SCOUT_WS="${SCOUT_WS:-$SCOUT_REPO_ROOT/ros2_ws}"
SERVICE_NAME="${SCOUT_WEBRTC_SERVICE:-scoutmini-webrtc.service}"
SCOUT_STREAM_URL="${SCOUT_STREAM_URL:-http://<robot_ip_or_tailscale_ip>:8889/zed/}"

usage() {
  echo "Usage: $0 {status|start|stop|diagnostics}"
}

source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble is not installed." >&2; return 1; }
  [[ -f "$SCOUT_WS/install/setup.bash" ]] || { echo "Workspace is not built: $SCOUT_WS" >&2; return 1; }
  set +u
  source /opt/ros/humble/setup.bash
  source "$SCOUT_WS/install/setup.bash"
  set -u
}

resolve_scripts() {
  source_ros
  local share
  share="$(ros2 pkg prefix --share scoutmini_streaming)"
  HEALTH_SCRIPT="${HEALTH_SCRIPT:-$share/scripts/stream_health.sh}"
  CHECK_SCRIPT="${CHECK_SCRIPT:-$share/scripts/check_zed_stream_stack.sh}"
}

print_status() {
  resolve_scripts
  local service_state
  service_state="$(systemctl --user is-active "$SERVICE_NAME" 2>/dev/null || true)"
  echo "WebRTC service: ${service_state:-unknown}"
  echo "URL: $SCOUT_STREAM_URL"
  "$HEALTH_SCRIPT" --status || true
}

start_stream() {
  resolve_scripts
  systemctl --user start "$SERVICE_NAME"
  for ((second = 0; second < 75; second++)); do
    if "$HEALTH_SCRIPT" --ready; then
      echo "ZED WebRTC stream is healthy."
      print_status
      return 0
    fi
    if [[ "$(systemctl --user is-failed "$SERVICE_NAME" 2>/dev/null || true)" == "failed" ]]; then
      break
    fi
    sleep 1
  done
  echo "WebRTC did not become healthy." >&2
  journalctl --user -u "$SERVICE_NAME" -n 40 --no-pager >&2 || true
  return 1
}

stop_stream() {
  systemctl --user stop "$SERVICE_NAME"
  echo "Stopped WebRTC service. Camera and image2rtsp were not touched."
}

diagnostics() {
  resolve_scripts
  "$CHECK_SCRIPT" || true
  echo
  journalctl --user -u "$SERVICE_NAME" -n 40 --no-pager || true
}

case "${1:-status}" in
  status) print_status ;;
  start) start_stream ;;
  stop) stop_stream ;;
  diagnostics|diag) diagnostics ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
