#!/bin/bash
# Launches the Fluke host script and OBS with HTTP→USB fallback
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
URL=${URL:-http://fluke-bridge.local/status.json}
SER=${SER:-/dev/ttyACM0}
# Recommended performance flags (override by setting FLAGS/QS_GAP_MS before calling)
FLAGS=${FLAGS:---no-fsync --write-interval 0 --csv-on-change}
QS_GAP_MS=${QS_GAP_MS:-60}

echo "[RUN] Probing ${URL}..."
if curl -fsS --connect-timeout 1 --max-time 2 "$URL" >/dev/null 2>&1; then
  echo "[RUN] HTTP available – using ${URL}"
  python3 "$SCRIPT_DIR/fluke_read.py" --http "$URL" $FLAGS &
else
  echo "[RUN] HTTP unavailable – using USB ${SER}"
  python3 "$SCRIPT_DIR/fluke_read.py" --serial "$SER" $FLAGS --qs-gap-ms "$QS_GAP_MS" &
fi
FLUKE_PID=$!

sleep 1

obs || true

kill "$FLUKE_PID" 2>/dev/null || true
