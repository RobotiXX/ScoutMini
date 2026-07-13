#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$PACKAGE_ROOT/../../.." && pwd)}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
ZED_RTSP_TOPIC="${ZED_RTSP_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"

echo "== Host =="
hostname || true
whoami || true
hostname -I 2>/dev/null || echo "hostname -I unavailable in this environment"

echo
echo "== Tailscale =="
if command -v tailscale >/dev/null 2>&1; then
  tailscale version || true
  tailscale status || true
  tailscale ip -4 || true
else
  echo "tailscale: not installed"
fi

echo
echo "== MediaMTX =="
LOCAL_MEDIAMTX="$PACKAGE_ROOT/test/tools/mediamtx-v1.19.2-linux-arm64/mediamtx"
if [[ -x "$LOCAL_MEDIAMTX" ]]; then
  "$LOCAL_MEDIAMTX" --version || true
else
  echo "local mediamtx: missing at $LOCAL_MEDIAMTX"
fi

echo
echo "== Files =="
for file in \
  "$PACKAGE_ROOT/scripts/start_zed_rtsp_stack.sh" \
  "$PACKAGE_ROOT/scripts/start_zed_webrtc_stack.sh" \
  "$PACKAGE_ROOT/config/mediamtx_zed_webrtc.yml" \
  "$SCOUT_WS/install/setup.bash"; do
  if [[ -e "$file" ]]; then
    echo "ok: $file"
  else
    echo "missing: $file"
  fi
done

echo
echo "== Ports =="
if ! ss -ltnup 2>/tmp/zed_rtsp_ss.err | grep -E ':8554|:8889|:8189'; then
  if [[ -s /tmp/zed_rtsp_ss.err ]]; then
    echo "port check unavailable: $(head -n 1 /tmp/zed_rtsp_ss.err)"
  else
    echo "no RTSP/WebRTC listeners"
  fi
fi

echo
echo "== ROS topic =="
if [[ -f /opt/ros/humble/setup.bash && -f "$SCOUT_WS/install/setup.bash" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  source "$SCOUT_WS/install/setup.bash"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  if ! timeout 5 ros2 topic info --no-daemon "$ZED_RTSP_TOPIC" 2>/tmp/zed_rtsp_ros2.err; then
    if [[ -s /tmp/zed_rtsp_ros2.err ]]; then
      echo "ROS topic check unavailable: $(tail -n 1 /tmp/zed_rtsp_ros2.err)"
    else
      echo "ROS topic not visible within timeout"
    fi
  fi
else
  echo "ROS setup files missing; skipping topic check"
fi
