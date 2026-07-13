#!/usr/bin/env bash
set -euo pipefail

USB_ROOT="${USB_ROOT:-/sys/bus/usb/devices}"
ZED_VENDOR_ID="${ZED_VENDOR_ID:-2b03}"

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "This test helper must run as root to reauthorize USB devices." >&2
  echo "Run: sudo $0" >&2
  exit 1
fi

zed_devices=()
for device in "$USB_ROOT"/*; do
  [[ -f "$device/idVendor" && -f "$device/authorized" ]] || continue
  [[ "$(cat "$device/idVendor")" == "$ZED_VENDOR_ID" ]] || continue
  zed_devices+=("$device")
done

[[ "${#zed_devices[@]}" -gt 0 ]] || {
  echo "No Stereolabs ZED USB devices found under $USB_ROOT." >&2
  exit 1
}

for device in "${zed_devices[@]}"; do
  product="$(cat "$device/product" 2>/dev/null || echo unknown)"
  echo "Reauthorizing $device ($product)"
  echo 0 > "$device/authorized"
done
sleep 2
for device in "${zed_devices[@]}"; do
  [[ -e "$device/authorized" ]] && echo 1 > "$device/authorized"
done
sleep 3
