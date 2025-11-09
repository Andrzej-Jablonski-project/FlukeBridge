Changelog
=========

All notable changes to this project will be documented in this file.

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
