#!/bin/bash
# Launches the Fluke host script and OBS with HTTP→USB fallback
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
URL=${URL:-http://fluke-bridge.local/status.json}
SER=${SER:-/dev/ttyACM0}

echo "[RUN] Probing ${URL}..."
if curl -fsS --connect-timeout 1 --max-time 2 "$URL" >/dev/null 2>&1; then
  echo "[RUN] HTTP available – using ${URL}"
  python3 "$SCRIPT_DIR/fluke_read.py" --http "$URL" &
else
  echo "[RUN] HTTP unavailable – using USB ${SER}"
  python3 "$SCRIPT_DIR/fluke_read.py" --serial "$SER" &
fi
FLUKE_PID=$!

sleep 1

obs || true

kill "$FLUKE_PID" 2>/dev/null || true
