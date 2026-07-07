#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
VERSION="${VERSION:-v1.19.2}"
ASSET="mediamtx_${VERSION}_linux_arm64.tar.gz"
EXPECTED_SHA256="${EXPECTED_SHA256:-562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688}"
TOOLS_DIR="$REPO_ROOT/scripts/zed_rtsp/tools"
DOWNLOAD_DIR="$TOOLS_DIR/downloads"
INSTALL_DIR="$TOOLS_DIR/mediamtx-${VERSION#v}-linux-arm64"
BASE_URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}"

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
  echo "This installer is pinned for Linux aarch64 Jetson systems." >&2
  echo "Detected: $(uname -s) $(uname -m)" >&2
  exit 1
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "Missing curl." >&2
  exit 1
fi

if ! command -v sha256sum >/dev/null 2>&1; then
  echo "Missing sha256sum." >&2
  exit 1
fi

if ! command -v tar >/dev/null 2>&1; then
  echo "Missing tar." >&2
  exit 1
fi

mkdir -p "$DOWNLOAD_DIR" "$INSTALL_DIR"

echo "Downloading MediaMTX ${VERSION} for Linux arm64..."
curl -L --fail --show-error --output "$DOWNLOAD_DIR/$ASSET" "$BASE_URL/$ASSET"
curl -L --fail --show-error --output "$DOWNLOAD_DIR/checksums.sha256" "$BASE_URL/checksums.sha256"

ACTUAL_SHA256="$(sha256sum "$DOWNLOAD_DIR/$ASSET" | awk '{print $1}')"
if [[ "$ACTUAL_SHA256" != "$EXPECTED_SHA256" ]]; then
  echo "Checksum mismatch for $ASSET" >&2
  echo "Expected: $EXPECTED_SHA256" >&2
  echo "Actual:   $ACTUAL_SHA256" >&2
  exit 1
fi

echo "Checksum verified: $ACTUAL_SHA256"
tar -xzf "$DOWNLOAD_DIR/$ASSET" -C "$INSTALL_DIR"

echo "Installed local MediaMTX binary:"
"$INSTALL_DIR/mediamtx" --version
echo "$INSTALL_DIR/mediamtx"
