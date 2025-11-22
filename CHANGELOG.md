Changelog
=========

All notable changes to this project will be documented in this file.

3.3.5 – Less eager FULL; require USB + charge time
- FULL requires USB present, at least 15 minutes spent in CHARGING, flat slope, and normalized voltage near full (±50 mV); hold time increased to 60 s.
- Negative-slope exit from CHARGING removed (stays latched while USB is present) to avoid flicker on small sags during CV charging.
- USB presence threshold lowered to 3.90 V for this hardware (charging detection works at ~4.0 V on cell).

3.3.4 – USB-gated charging trend
- Charging trend (CHARGING state and percent rate limit) is now allowed only when USB is present (VBAT above USB_PRESENT_V).
- When USB is absent, charging timers and trend are reset, preventing spurious CHARGING/DISCHARGE flips on small voltage drift.
- Percent still cannot increase when not in CHARGING, removing unrealistic growth during pure discharge.

3.3.3 – Per-version release notes (CI)
- GitHub Actions: release body shows only the changes for the current version (extracted from CHANGELOG into RELEASE_NOTES.md).
- Script: tools/release.sh checks if tag `v<VER>` already exists before creating it.
- Note: before tagging, ensure the binary `FlukeBridge-ver<VER>-XIAO-ESP32C3.bin` is present and SHA256 is updated (the script does this), then tag.

3.3.2 – Battery charge/full detection tuning
- Trend-based charging detection thresholds relaxed: on >= +0.30 mV/s for 4 s (>=10 mV total), off on <= −0.15 mV/s for 6 s; near‑zero slope off after 60 s (<= 0.03 mV/s).
- FULL detection now uses SoC calibration: considers full when normalized VBAT is within ~20 mV of reference full and slope is flat for 15 s.
- Effect: charging state is recognized earlier and percentage no longer jumps during charging (rate limiting applies while CHARGING).

3.3.1 – Config UI + performance
- Config page: responsive 3/2/1‑column grid for Advanced tuning (no overflow on focus).
- Config page: added "Restore defaults" button for tuning (persists in NVS).
- Config page: Result box wraps/scrolls long lines (no overflow outside card).
- ADC sampling: VBAT read averages 4 samples without per‑sample delays (snappier UI).
- Firmware version bump to `ver3.3.1` (shown in UI/OTA).

3.3 – Filtered VBAT + deep sleep
- Add exponential filter and hysteresis for battery voltage/LED.
- Enter deep sleep after 15 s in CRIT to protect the cell.
- `/status.json`: add `battery.v_raw` and `sleep_in` fields.
- Add battery state string in JSON (`battery.state`).
- Smoothed percent: voltage deadband + quantization + rate limiting.
- ADC improvements: `analogReadMilliVolts()` + runtime VBAT config (divider + scale) via USB (`VBAT?`, `VBAT DIV`, `VBAT SCALE`, `VBAT CAL`, `VBAT CLEAR`).

3.2 – OTA + dual Wi‑Fi/USB mode
- Add OTA updates via `/update` (Basic Auth: admin/fluke1234).
- Add PING/VER/MODE/WIFI? USB commands; keep QM/QS/ID/QDDA/RI/RMP.
- Dashboard uptime in days + HH:MM:SS.
- Status page and index link to `/update`.

3.1 – mDNS + OL handling
- Improve OL detection and formatting.
- Add mDNS `fluke-bridge.local`.
- Long‑press BOOT clears both SDK Wi‑Fi and Preferences.

3.0 – Dashboard + battery logic
- Add dark dashboard `/status.html` and JSON API `/status.json`.
- Battery ADC + LED behavior.

2.0 – HTTP JSON API
- Provide `/status.json` with Fluke + battery + Wi‑Fi info.

1.0 – Wi‑Fi TCP bridge
- Initial Wi‑Fi client mode with basic endpoints.

0.1 – Basic serial bridge
- Read Fluke via IR/UART and print frames.
