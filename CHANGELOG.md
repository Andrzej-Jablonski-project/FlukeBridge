Changelog
=========

All notable changes to this project will be documented in this file.

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

