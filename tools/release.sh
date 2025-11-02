#!/usr/bin/env bash
set -euo pipefail

# Usage: tools/release.sh 3.2
# Generates SHA256SUMS.txt for the given version and prints git commands
# to commit, tag (vX.Y) and push, which triggers the GitHub Actions release (draft).

if [[ ${1:-} == "" ]]; then
  echo "Usage: $0 <version>" >&2
  echo "Example: $0 3.2" >&2
  exit 1
fi

VER="$1"
BIN="FlukeBridge-ver${VER}-XIAO-ESP32C3.bin"

if [[ ! -f "$BIN" ]]; then
  echo "Error: binary '$BIN' not found in $(pwd)" >&2
  echo "Make sure you exported the compiled binary and renamed it accordingly." >&2
  exit 2
fi

echo "Computing SHA256 for $BIN ..."
sha256sum "$BIN" > SHA256SUMS.txt
cat SHA256SUMS.txt

echo
echo "Next steps (copy-paste):"
cat <<CMD
git add "$BIN" SHA256SUMS.txt CHANGELOG.md README.md
git commit -m "v$VER – release"
git tag -a v$VER -m "v$VER"
git push origin main
git push origin v$VER
CMD

echo
echo "A draft Release will be created by GitHub Actions with attached assets."

