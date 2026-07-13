#!/usr/bin/env bash
set -euo pipefail

GROUP_NAME="scoutmini-network"
POLICY_NAME="50-scoutmini-network.rules"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
POLICY_SOURCE="${SCRIPT_DIR}/../ros2_ws/src/scoutmini_display/config/${POLICY_NAME}"
POLICY_DESTINATION="/etc/polkit-1/rules.d/${POLICY_NAME}"
TARGET_USER="${1:-${SUDO_USER:-}}"

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this installer with sudo: sudo $0 [dashboard-user]" >&2
    exit 1
fi

if [[ -z ${TARGET_USER} || ${TARGET_USER} == root ]]; then
    echo "Pass the non-root account that runs the dashboard." >&2
    echo "Example: sudo $0 nvidia" >&2
    exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
    echo "User '${TARGET_USER}' does not exist." >&2
    exit 1
fi

if [[ ! -f ${POLICY_SOURCE} ]]; then
    echo "Policy file not found: ${POLICY_SOURCE}" >&2
    exit 1
fi

getent group "${GROUP_NAME}" >/dev/null || groupadd --system "${GROUP_NAME}"
usermod --append --groups "${GROUP_NAME}" "${TARGET_USER}"
install --owner=root --group=root --mode=0644 "${POLICY_SOURCE}" "${POLICY_DESTINATION}"

echo "Installed ${POLICY_DESTINATION}."
echo "Added ${TARGET_USER} to ${GROUP_NAME}."
echo "Reboot the Jetson (or fully sign out and back in) before running the dashboard again."
