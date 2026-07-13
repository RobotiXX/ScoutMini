#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SCOUT_WEBRTC_SERVICE:-scoutmini-webrtc.service}"
systemctl --user stop "$SERVICE_NAME"
echo "Stopped $SERVICE_NAME. Camera and image2rtsp were not touched."
