#!/usr/bin/env bash
set -euo pipefail

VERSION="${VERSION:-v1.19.2}"
ASSET="mediamtx_${VERSION}_linux_arm64.tar.gz"
EXPECTED_SHA256="${EXPECTED_SHA256:-562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688}"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/scoutmini/mediamtx/$VERSION"
INSTALL_DIR="${MEDIAMTX_INSTALL_DIR:-$HOME/.local/lib/scoutmini/mediamtx}"
BASE_URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}"

[[ "$(uname -s)" == "Linux" && "$(uname -m)" == "aarch64" ]] || {
  echo "Expected Linux aarch64; found $(uname -s) $(uname -m)." >&2
  exit 1
}
for command in curl sha256sum tar; do
  command -v "$command" >/dev/null 2>&1 || { echo "Missing $command." >&2; exit 1; }
done

mkdir -p "$CACHE_DIR" "$INSTALL_DIR"
curl -L --fail --show-error --output "$CACHE_DIR/$ASSET" "$BASE_URL/$ASSET"

ACTUAL_SHA256="$(sha256sum "$CACHE_DIR/$ASSET" | awk '{print $1}')"
[[ "$ACTUAL_SHA256" == "$EXPECTED_SHA256" ]] || {
  echo "Checksum mismatch: expected $EXPECTED_SHA256, got $ACTUAL_SHA256" >&2
  exit 1
}

TEMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TEMP_DIR"' EXIT
tar -xzf "$CACHE_DIR/$ASSET" -C "$TEMP_DIR"
install -m 0755 "$TEMP_DIR/mediamtx" "$INSTALL_DIR/mediamtx"
"$INSTALL_DIR/mediamtx" --version
echo "$INSTALL_DIR/mediamtx"
