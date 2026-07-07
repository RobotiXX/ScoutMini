#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
SIGNAL_TIMEOUT="${SIGNAL_TIMEOUT:-8}"

patterns=(
  "$REPO_ROOT/scripts/zed_rtsp/start_zed_webrtc_stack.sh"
  "$REPO_ROOT/scripts/zed_rtsp/start_zed_rtsp_stack.sh"
  "mediamtx.*mediamtx_zed_webrtc.yml"
  "ros2 launch image2rtsp image2rtsp.launch.py"
  "image2rtsp"
  "component_container_isolated.*__node:=zed_container"
)

echo "Stopping ZED RTSP/WebRTC stack processes..."

matched=()
for pattern in "${patterns[@]}"; do
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    if [[ "$pid" != "$$" ]]; then
      matched+=("$pid")
    fi
  done < <(pgrep -f "$pattern" || true)
done

if [[ "${#matched[@]}" -eq 0 ]]; then
  echo "No ZED stream stack wrapper, image2rtsp launch, or MediaMTX process found."
else
  unique_pids="$(printf "%s\n" "${matched[@]}" | sort -n | uniq)"
  echo "$unique_pids" | while read -r pid; do
    [[ -n "$pid" ]] || continue
    echo "TERM $pid $(ps -p "$pid" -o comm= 2>/dev/null || true)"
    kill -TERM "$pid" >/dev/null 2>&1 || true
  done

  sleep "$SIGNAL_TIMEOUT"

  echo "$unique_pids" | while read -r pid; do
    [[ -n "$pid" ]] || continue
    if kill -0 "$pid" >/dev/null 2>&1; then
      echo "KILL $pid $(ps -p "$pid" -o comm= 2>/dev/null || true)"
      kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
fi

echo
echo "Remaining stream ports:"
ss -ltnup | grep -E ':8554|:8889|:8189' || echo "none"
