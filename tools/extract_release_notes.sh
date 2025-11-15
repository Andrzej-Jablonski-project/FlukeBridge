#!/usr/bin/env bash
set -euo pipefail

# Usage: tools/extract_release_notes.sh 3.3.3
# Extracts only the section for the given version from CHANGELOG.md
# and writes it to RELEASE_NOTES.md (used by GitHub Releases).

if [[ ${1:-} == "" ]]; then
  echo "Usage: $0 <version>" >&2
  exit 1
fi

VER="$1"
IN="CHANGELOG.md"
OUT="RELEASE_NOTES.md"

if [[ ! -f "$IN" ]]; then
  echo "ERROR: $IN not found" >&2
  exit 2
fi

# Match line that starts with version like: 3.3.3 – ... (dash can be unicode en dash)
# Stop at next version header (X.Y or X.Y.Z followed by dash)
awk -v ver="$VER" '
  BEGIN { start=0 }
  {
    if ($0 ~ "^" ver "[[:space:]]+[\xE2\x80\x93\-]") { start=1 } # en dash or hyphen
    if (start) { print $0; next }
  }
  ' "$IN" | awk '
    NR==1{ inver=1; print; next }
    {
      # If we encounter a new version header, stop
      if ($0 ~ /^[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]+[\xE2\x80\x93\-]/) { exit }
      print
    }
  ' > "$OUT"

if [[ ! -s "$OUT" ]]; then
  echo "WARN: Could not extract notes for $VER; falling back to full changelog" >&2
  cp "$IN" "$OUT"
fi

echo "Wrote $OUT (notes for $VER)"

