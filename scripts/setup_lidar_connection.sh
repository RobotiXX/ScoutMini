#!/usr/bin/env bash
set -euo pipefail

IFACE="eno1"
CON_NAME="lidar"
IPV4_ADDR="192.168.1.100/24"

sudo nmcli connection delete "$CON_NAME" 2>/dev/null || true

sudo nmcli connection add \
  type ethernet \
  ifname "$IFACE" \
  con-name "$CON_NAME" \
  ipv4.method manual \
  ipv4.addresses "$IPV4_ADDR" \
  ipv6.method ignore \
  autoconnect yes

sudo nmcli connection up "$CON_NAME"

echo "Connection $CON_NAME is up."
ip addr show dev "$IFACE"