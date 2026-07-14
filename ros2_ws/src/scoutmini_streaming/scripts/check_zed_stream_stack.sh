#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HEALTH_SCRIPT="${HEALTH_SCRIPT:-$SCRIPT_DIR/stream_health.sh}"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-$HOME/.local/lib/scoutmini/mediamtx/mediamtx}"

status=0

echo "== CONFIG =="
for file in "$HEALTH_SCRIPT" "/opt/ros/humble/setup.bash"; do
  if [[ -e "$file" ]]; then
    echo "PASS CONFIG             found $file"
  else
    echo "FAIL CONFIG             missing $file"
    status=1
  fi
done
if [[ -f "${SCOUT_WS:-$HOME/repos/ScoutMini/ros2_ws}/install/setup.bash" ]]; then
  echo "PASS CONFIG             ROS workspace is built"
else
  echo "FAIL CONFIG             ROS workspace setup is missing"
  status=1
fi

echo
echo "== Stream health =="
"$HEALTH_SCRIPT" --status || status=1

echo
echo "== User services =="
systemctl --user status scoutmini-webrtc.service scoutmini-slack.service \
  --no-pager 2>&1 || true
for service in scoutmini-webrtc.service scoutmini-slack.service; do
  service_state="$(systemctl --user is-active "$service" 2>/dev/null || true)"
  case "$service_state" in
    active)
      echo "PASS SERVICE_LIFECYCLE  $service is active"
      ;;
    failed)
      echo "FAIL SERVICE_LIFECYCLE  $service is failed; inspect its user journal"
      status=1
      ;;
    *)
      echo "SKIP SERVICE_LIFECYCLE  $service is ${service_state:-unknown} (manual-start policy)"
      ;;
  esac
done

echo
echo "== Dependencies =="
for command in ros2 ffprobe curl systemctl; do
  if command -v "$command" >/dev/null 2>&1; then
    echo "PASS CONFIG             found $command"
  else
    echo "FAIL CONFIG             missing $command"
    status=1
  fi
done
if [[ -x "$MEDIAMTX_BIN" ]]; then
  echo "PASS CONFIG             $("$MEDIAMTX_BIN" --version 2>&1 | head -n 1)"
else
  echo "FAIL CONFIG             missing $MEDIAMTX_BIN"
  status=1
fi

echo
echo "== Processes and ports =="
pgrep -af 'mediamtx|image2rtsp|zed_camera.launch.py|zed_component|component_container_isolated' || true
ss -ltnup 2>/dev/null | grep -E ':8554|:8889|:8189' || echo "no stream listeners"

echo
echo "== ZED USB =="
lsusb 2>/dev/null | grep -i 'stereolabs\|zed' || echo "ZED USB device not listed"
if command -v journalctl >/dev/null 2>&1; then
  journalctl -k --since '-30 min' --no-pager 2>/dev/null \
    | grep -Ei 'usb|uvc|zed|error -71|urb' | tail -n 40 || true
fi

echo
echo "== Network =="
hostname -I 2>/dev/null || true
if command -v tailscale >/dev/null 2>&1; then
  tailscale_ip="$(tailscale ip -4 2>/dev/null || true)"
  if [[ -n "$tailscale_ip" ]]; then
    echo "PASS ICE_NETWORK         Tailscale IPv4 is $tailscale_ip"
    echo "SKIP WEBRTC_MEDIA        verify ICE and advancing frames from a remote browser"
  else
    echo "FAIL ICE_NETWORK         Tailscale IPv4 is unavailable"
    status=1
  fi
  tailscale status 2>&1 | sed -n '/# Health check:/,$p' || true
else
  echo "FAIL ICE_NETWORK         tailscale command is missing"
  status=1
fi

echo
echo "Package root: $PACKAGE_ROOT"
exit "$status"
