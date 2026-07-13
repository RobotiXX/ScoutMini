#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SOURCE_UNIT="$PACKAGE_ROOT/systemd/scoutmini-webrtc.service"
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"

[[ -f "$SOURCE_UNIT" ]] || { echo "Missing unit: $SOURCE_UNIT" >&2; exit 1; }
mkdir -p "$UNIT_DIR"
install -m 0644 "$SOURCE_UNIT" "$UNIT_DIR/scoutmini-webrtc.service"
systemctl --user daemon-reload

echo "Installed scoutmini-webrtc.service (disabled)."
echo "Start manually: systemctl --user start scoutmini-webrtc.service"
