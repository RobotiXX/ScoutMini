#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 nvidia@<robot_ip>" >&2
  exit 2
fi

ROBOT_SSH="$1"
LOCAL_PORT="${LOCAL_PORT:-18554}"
REMOTE_PORT="${REMOTE_PORT:-8554}"
MOUNTPOINT="${MOUNTPOINT:-zed}"
URL="rtsp://127.0.0.1:${LOCAL_PORT}/${MOUNTPOINT}"

if ! command -v ssh >/dev/null 2>&1; then
  echo "Missing ssh client." >&2
  exit 1
fi

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
        echo "Or open this URL in VLC after starting a tunnel: $URL" >&2
        exit 1
        ;;
    esac
  else
    echo "Install ffmpeg for ffplay, or open this URL in VLC after starting a tunnel:" >&2
    echo "$URL" >&2
    exit 1
  fi
fi

cleanup() {
  if [[ -n "${TUNNEL_PID:-}" ]]; then
    kill "$TUNNEL_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "Opening SSH tunnel: 127.0.0.1:${LOCAL_PORT} -> ${ROBOT_SSH}:127.0.0.1:${REMOTE_PORT}"
ssh -N -L "${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}" "$ROBOT_SSH" &
TUNNEL_PID="$!"

sleep 1
if ! kill -0 "$TUNNEL_PID" >/dev/null 2>&1; then
  echo "SSH tunnel exited before the viewer could start." >&2
  wait "$TUNNEL_PID" || true
  exit 1
fi

echo "Opening $URL"
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 -sync ext "$URL"
