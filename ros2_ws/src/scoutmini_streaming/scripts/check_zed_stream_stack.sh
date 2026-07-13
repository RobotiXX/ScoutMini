#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HEALTH_SCRIPT="${HEALTH_SCRIPT:-$SCRIPT_DIR/stream_health.sh}"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-$HOME/.local/lib/scoutmini/mediamtx/mediamtx}"

echo "== Stream health =="
"$HEALTH_SCRIPT" --status || true

echo
echo "== User services =="
systemctl --user status scoutmini-webrtc.service scoutmini-slack.service \
  --no-pager 2>&1 || true

echo
echo "== Dependencies =="
for command in ros2 ffprobe curl systemctl; do
  command -v "$command" >/dev/null 2>&1 && echo "ok: $command" || echo "missing: $command"
done
if [[ -x "$MEDIAMTX_BIN" ]]; then
  "$MEDIAMTX_BIN" --version 2>&1 | head -n 1
else
  echo "missing: $MEDIAMTX_BIN"
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
tailscale ip -4 2>/dev/null || true

echo
echo "Package root: $PACKAGE_ROOT"
