#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${1:-192.168.0.159}"
MOUNTPOINT="${MOUNTPOINT:-zed}"
URL="rtsp://${ROBOT_IP}:8554/${MOUNTPOINT}"

if ! command -v ffplay >/dev/null 2>&1; then
  echo "Missing ffplay. It is provided by the ffmpeg package." >&2
  if command -v apt-get >/dev/null 2>&1; then
    read -r -p "Install ffmpeg now with apt? [y/N] " answer
    case "$answer" in
      [Yy]*)
        sudo apt update
        sudo apt install -y ffmpeg
        ;;
      *)
        echo "Install later with: sudo apt install ffmpeg" >&2
        echo "Or open this URL in VLC: $URL" >&2
        exit 1
        ;;
    esac
  else
    echo "Install ffmpeg for ffplay, or open this URL in VLC:" >&2
    echo "$URL" >&2
    exit 1
  fi
fi

echo "Opening $URL"
ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 -sync ext "$URL"
