#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SOURCE_UNIT="$PACKAGE_ROOT/systemd/scoutmini-slack.service"
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
REPO_ROOT="${SCOUT_REPO_ROOT:-${REPO_ROOT:-}}"
REPO_ROOT="${REPO_ROOT:-$(git -C "$PACKAGE_ROOT" rev-parse --show-toplevel 2>/dev/null || true)}"

[[ -f "$SOURCE_UNIT" ]] || { echo "Missing unit: $SOURCE_UNIT" >&2; exit 1; }
[[ -n "$REPO_ROOT" ]] || { echo "Unable to determine ScoutMini repository root." >&2; exit 1; }
mkdir -p "$UNIT_DIR"
TEMP_UNIT="$(mktemp)"
trap 'rm -f "$TEMP_UNIT"' EXIT
sed "s|@REPO_ROOT@|$REPO_ROOT|g" "$SOURCE_UNIT" > "$TEMP_UNIT"
install -m 0644 "$TEMP_UNIT" "$UNIT_DIR/scoutmini-slack.service"
systemctl --user daemon-reload

echo "Installed scoutmini-slack.service for $REPO_ROOT (disabled)."
echo "Start manually: systemctl --user start scoutmini-slack.service"
