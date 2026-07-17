#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCOUT_REPO_ROOT="${SCOUT_REPO_ROOT:-${REPO_ROOT:-$HOME/repos/ScoutMini}}"
OUT_DIR="${OUT_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/scoutmini/diagnostics}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BUNDLE_DIR="$OUT_DIR/zed_stream_diag_$STAMP"

mkdir -p "$BUNDLE_DIR"

capture() {
  local name="$1"
  shift
  { printf '$'; printf ' %q' "$@"; printf '\n'; "$@" 2>&1 || true; } > "$BUNDLE_DIR/$name.txt"
}

capture_shell() {
  local name="$1"
  local command="$2"
  { echo "\$ $command"; bash -lc "$command" 2>&1 || true; } > "$BUNDLE_DIR/$name.txt"
}

capture date date --iso-8601=seconds
capture host hostnamectl
capture host_ips hostname -I
capture tailscale tailscale status
capture usb lsusb -t
capture usb_devices lsusb
capture stream_check "$PACKAGE_ROOT/scripts/check_zed_stream_stack.sh"
capture webrtc_log journalctl --user -u scoutmini-webrtc.service -n 200 --no-pager
capture slack_log journalctl --user -u scoutmini-slack.service -n 200 --no-pager
capture kernel_usb journalctl -k --since '-2 hours' --no-pager
capture_shell git_status "git -C '$SCOUT_REPO_ROOT' status --short"
capture_shell ros_logs "find '$HOME/.ros/log' -maxdepth 2 -type f -mmin -120 -print | sort | tail -80"

ARCHIVE="$OUT_DIR/zed_stream_diag_$STAMP.tar.gz"
tar -C "$OUT_DIR" -czf "$ARCHIVE" "$(basename "$BUNDLE_DIR")"
echo "$ARCHIVE"
