# 🧭 Fluke Bridge Wi‑Fi Adapter

![Release](https://img.shields.io/github/v/release/Andrzej-Jablonski-project/FlukeBridge?display_name=release)
![Downloads](https://img.shields.io/github/downloads/Andrzej-Jablonski-project/FlukeBridge/total)
![CI](https://github.com/Andrzej-Jablonski-project/FlukeBridge/actions/workflows/release.yml/badge.svg?branch=main)
![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)

Wireless bridge for Fluke 287/289 multimeters with live dashboard, HTTP API, and OBS integration

## 📘 Overview

Important: This project is designed and tested specifically for the Seeed Studio XIAO ESP32‑C3. Pinout (RX=20, TX=21, LED=GPIO3), BOOT behavior, and battery heuristics assume this board. Other ESP32 variants may require pin and wiring changes.

Fluke Bridge is a compact Wi‑Fi adapter based on Seeed XIAO ESP32‑C3, designed to connect a Fluke 287/289 multimeter through its infrared serial interface (LM393) and stream live measurements via a modern web dashboard or directly into OBS.

The project enables:

- 📡 Wireless measurement streaming (Voltage, Current, Resistance, Temperature, Capacitance, Diode Test)
- 💻 Real‑time dashboard — dark themed web interface at `http://fluke-bridge.local`
- 🎥 OBS integration via local text files
- ⚙️ OTA firmware update directly from browser
- 🔋 Battery monitoring with LED indication
- 📶 Automatic Wi‑Fi setup via captive portal
- 🔄 USB fallback mode when Wi‑Fi is not configured

<p>
  <img src="docs/images/ico.png" alt="Fluke Bridge icon" width="96">
</p>

## 🧩 Hardware

| Component | Function | Notes |
|-----------|----------|-------|
| Seeed XIAO ESP32‑C3 | MCU + Wi‑Fi | Tiny module with USB‑C |
| LM393 IR transceiver | Interface to Fluke optical port | RX=GPIO20, TX=GPIO21, non‑inverted |
| Li‑ion 18650 cell | Power supply | via protection board |
| DW01A + 8205A | Battery protection | B+/B− to cell, P+/P− to ESP |
| GPIO3 LED | Battery status indicator | ON = low batt, blink = critical |
| BOOT button | Long press = Wi‑Fi reset | same button as XIAO BOOT |
| RESET button | Restart device | standard |

<p>
  <img src="docs/images/fluke-cable.png" alt="IR link with XIAO ESP32-C3 – LM393 schematic" width="720">
</p>

## 🔋 Battery Indicator Logic (ver3.3)

Filtered voltage with hysteresis for stable UI and LED behavior. Battery percent uses a Li‑ion OCV (open‑circuit voltage) piecewise‑linear mapping with additional smoothing to avoid 1% flicker.

| Battery State | Thresholds (filtered) | LED Behavior (GPIO3) |
|---------------|-----------------------|----------------------|
| No battery / PCM cutoff | < 1.0 V | OFF |
| Full | > 4.15 V | OFF |
| USB connected | VBAT > 4.60 V (heuristic) | OFF |
| Low (WARN) | ON ≤ 3.40 V, clears > 3.50 V | ON steady |
| Critical (CRIT) | ON ≤ 3.00 V, clears > 3.10 V | Blinking (~1 Hz) |

Power saving: if CRIT holds continuously for 15 s, Wi‑Fi turns off and the device enters deep sleep. Wakeup via RESET.

## 🌐 Network Operation

### 1️⃣ Initial Setup (AP mode)
If no Wi‑Fi credentials exist:
- ESP starts its own AP: SSID: `FlukeBridge‑XXXX`
- Open any browser → automatic captive portal
- Enter SSID + password → ESP reboots into client mode

<p>
  <img src="docs/images/WI-FI-setup.png" alt="Captive portal Wi‑Fi setup" width="720">
</p>

### 2️⃣ Normal Operation (Client mode)
When connected:
- Accessible via mDNS → `http://fluke-bridge.local`
- Endpoints:
  - `/` → index page with links
- `/status.html` → dashboard (dark mode)
- `/status.json` → live JSON API
- `/config` → password-protected Config (SOC/VBAT)
- `/update` → OTA firmware upload panel

<p>
  <img src="docs/images/dashboard.png" alt="Dashboard – client mode" width="960">
</p>

## ⚙️ OTA Firmware Update

1. In Arduino IDE → Sketch → Export compiled binary
2. Open browser: `http://fluke-bridge.local/update`
3. Upload the `.ino.bin` file (application binary)
4. Wait for message and auto‑reboot

Notes:
- Use the app binary `<sketch>.ino.bin` (do not upload merged/with_bootloader images via OTA).
- Settings in Preferences (Wi‑Fi SSID/pass) remain intact across OTA.

<p>
  <img src="docs/images/OTA-update.png" alt="OTA upload panel" width="720">
</p>

## 🔌 USB Serial Mode (fallback)

If Wi‑Fi is not configured:
- ESP32‑C3 communicates directly with Fluke over USB CDC.
- Use the Python tool `pc/fluke_read.py` with `--serial`.
- When Wi‑Fi is configured later, continue using HTTP (`/status.json`).

### USB SoC Calibration (optional)
You can calibrate the percent mapping to your battery chemistry/cell by teaching the firmware your actual "full" and "empty" voltages (after resting a few minutes, not under heavy load):

- `SOC SET_FULL`  → saves current filtered VBAT as calibrated FULL
- `SOC SET_EMPTY` → saves current filtered VBAT as calibrated EMPTY
- `SOC?`          → prints calibration status and values
- `SOC CLEAR`     → clears calibration (reverts to defaults)

Once calibrated, the firmware normalizes measured VBAT between your `EMPTY↔FULL` to the reference OCV curve before converting to percent. Displayed voltage remains the real measured value.

### USB VBAT Divider / Calibration (universal hardware)
If you use a different resistor divider or want to correct tolerance, you can configure VBAT measurement at runtime (persisted in NVS):

- `VBAT?`            → prints measured VBAT, raw (pre‑scale), divider and scale
- `VBAT DIV <R1> <R2>` → set divider resistors in ohms (e.g., `VBAT DIV 620000 470000`), saves to NVS
- `VBAT SCALE <k>`   → set additional scale factor (e.g., `VBAT SCALE 0.965`), saves to NVS
- `VBAT CAL <volts>` → one‑point calibration: enter true VBAT from a reference DMM; firmware computes scale and saves
- `VBAT CLEAR`       → reset to defaults (620k/470k, scale=1.0)

Firmware uses ESP32‑C3 ADC calibration via `analogReadMilliVolts()` with appropriate attenuation and averages multiple samples for stability.

## 📡 JSON API — `/status.json`

Example (ver3.3 adds filtered voltage, battery state and sleep timer):

```json
{
  "wifi": { "ssid": "rzuf3", "rssi": -29, "ip": "172.28.7.203" },
  "battery": {
    "voltage": 3.72,
    "v_raw": 3.68,
    "percent": 56,
    "state": "discharge",
    "usb_present": false,
    "warn": false,
    "crit": false,
    "charging": false,
    "full": false,
    "no_batt": false
  },
  "fluke": {
    "pretty": "22.6 °C",
    "value": "22.6",
    "unit": "CEL",
    "status": "NORMAL",
    "flags": "NONE",
    "ol": false,
    "raw": "22.6,CEL,NORMAL,NONE"
  },
  "uptime": 234,
  "sleep_in": -1
}
```

### Fields Reference

| Field         | Description                      |
|---------------|----------------------------------|
| `wifi.*`      | Wi‑Fi connection info            |
| `battery.*`   | Battery telemetry                |
| `battery.v_raw`| Instantaneous (unfiltered) voltage |
| `battery.state`| One of: `discharge`, `charging`, `full`, `no_batt` |
| `fluke.pretty`| Human‑friendly formatted value   |
| `fluke.value` | Raw numeric                      |
| `fluke.unit`  | Measurement unit                 |
| `fluke.status`| Meter status (e.g., NORMAL, CONT, DIODE) |
| `fluke.ol`    | True if overload (OL)            |
| `uptime`      | Seconds since boot               |
| `sleep_in`    | Seconds to deep sleep in CRIT (−1 otherwise) |

## 🧠 Firmware Features

| Feature            | Description                                   |
|--------------------|-----------------------------------------------|
| Auto Wi‑Fi setup   | Captive portal + preferences storage          |
| HTTP server        | `/`, `/status.json`, `/update`                |
| Async dashboard    | Auto‑refresh via AJAX                         |
| mDNS support       | Access as `fluke-bridge.local`                |
| Battery ADC monitor| Percentage + LED logic                        |
| IR UART            | 115200 baud, RX=20, TX=21 (non‑inverted)      |
| OTA Update         | Native OTA via `Update.h`                      |
| BOOT long‑press    | Wi‑Fi reset (clear NVS + Preferences)         |
| Dark theme UI      | Consistent design                             |
| Failsafe USB       | Automatic serial fallback                     |

## 🐍 Python Tool — `fluke_read.py`

Features
- Works with both HTTP (Wi‑Fi) and USB serial
- Saves data for OBS overlays
- Detects status: LIVE, HOLD, OL, OFF
  - Note: this computed status is produced by the Python tool logic; the firmware JSON exposes the meter status, not LIVE/HOLD/OL/OFF directly.

Usage
```
# HTTP (Wi‑Fi)
python3 pc/fluke_read.py --http http://fluke-bridge.local/status.json

# USB (fallback)
python3 pc/fluke_read.py --serial /dev/ttyACM0
```

Options

| Flag          | Description                    |
|---------------|--------------------------------|
| `--http <url>`| Use HTTP mode                  |
| `--serial <p>`| Use serial mode (USB CDC)      |
| `--tcp h:p`   | Optional TCP bridge mode       |
| `--dir <path>`| Output folder for text/CSV     |

Performance tips

You can tune I/O latency and update behavior without changing defaults:

- `--no-fsync` or `FLUKE_NO_FSYNC=1` – disable fsync() on writes (lower latency, less disk wear).
- `--write-interval <s>` or `FLUKE_WRITE_INTERVAL_S` – minimum seconds between periodic rewrites (default 0.2). Use `0` to rewrite only on change.
- `--qs-gap-ms <ms>` or `FLUKE_QS_GAP_MS` – delay between QM and QS reads in two‑channel mode (default 80 ms).
- `--csv-on-change` or `FLUKE_CSV_ON_CHANGE=1` – append to CSV only when output changed (instead of every loop).

Examples
```
# Lower latency file updates and only on change
python3 pc/fluke_read.py --http http://fluke-bridge.local/status.json \
  --no-fsync --write-interval 0 --csv-on-change

# Faster AC/DC second read gap (if stable in your setup)
python3 pc/fluke_read.py --serial /dev/ttyACM0 --qs-gap-ms 40
```

Output files

By default files are written next to the script (pc/ folder). You can override the folder with `--dir <path>` or environment variable `FLUKE_DIR`.

| File              | Purpose                           |
|-------------------|-----------------------------------|
| `fluke_value.txt` | Measured value (pretty, 1 line)   |
| `fluke_status.txt`| Status (LIVE / HOLD / OL / OFF)   |

## 🎥 OBS Integration

In OBS add two Text sources (Read from file) pointing to files in the same folder as `pc/fluke_read.py` (unless you used `--dir`):
- `pc/fluke_value.txt` → measurement
- `pc/fluke_status.txt` → status

Then run (prefers HTTP, falls back to USB):
```
cd pc
./run_obs_with_fluke.sh
```

Values refresh about every second.

**OBS Lua Script**
- Location: `pc/fluke_status_symbol_freetype.lua`.
- Purpose: updates a Text (FreeType2) source to show plain status text (LIVE/HOLD/OFF) and color; reads `fluke_status.txt`.
- Setup:
  - In OBS: Tools -> Scripts -> + -> select `pc/fluke_status_symbol_freetype.lua`.
  - Set "Source name (FreeType2)" to the name of your Text (FreeType2) source.
  - Set "Status file" to the path of `fluke_status.txt` (default lives next to `pc/fluke_read.py`, or in the folder you passed via `--dir`/`FLUKE_DIR`).
  - Optionally adjust "Interval (ms)" and colors for LIVE/HOLD/OFF.
- Notes:
  - The script disables "Read from file" and sets the text directly on the source.
  - No symbols are used; status text is exactly "LIVE", "HOLD", or "OFF".

## 🛠 Dependencies

- Arduino IDE and esp32 board (v3.0+)
- Libraries: `WiFi.h`, `WebServer.h`, `Preferences.h`, `DNSServer.h`, `ESPmDNS.h`, `Update.h`
- Python 3 with `pyserial` for USB mode: `pip install pyserial`

## 🧾 Version History

| Version | Features                           |
|---------|------------------------------------|
| 0.1     | Basic serial bridge                |
| 1.0     | Wi‑Fi TCP bridge                   |
| 2.0     | HTTP JSON API                      |
| 3.0     | Dashboard + battery logic          |
| 3.1     | OL handling fix + mDNS             |
| 3.2     | OTA + dual Wi‑Fi/USB mode          |
| 3.3     | VBAT filtering + hysteresis; CRIT→deep sleep; JSON: v_raw, sleep_in |

## 🚀 Release (tag + draft on GitHub)

Use the helper script to generate the checksum and print the git commands:

```
cd github
bash tools/release.sh 3.3
```

This will create `SHA256SUMS.txt` and show the exact `git add/commit/tag/push` steps. Pushing the tag `v3.3` triggers GitHub Actions to create a draft Release with for example:
- `FlukeBridge-ver3.3-XIAO-ESP32C3.bin`
- `SHA256SUMS.txt`

You can then publish the Release from the GitHub UI.

## 🧰 Troubleshooting

| Symptom            | Fix                                                |
|--------------------|----------------------------------------------------|
| No setup AP        | Hold BOOT > 3 s to clear Wi‑Fi and reboot          |
| Dashboard 404      | Open `/` (root) or `/status.html` directly         |
| OBS shows OFF      | Check `fluke.status` and USB/HTTP connectivity     |
| OL shown as 9.99e37| Update firmware ≥ 3.1                              |
| OTA failed         | Ensure stable Wi‑Fi and battery; retry or USB flash|
| USB flash fails    | Hold BOOT while plugging in to enter bootloader    |

## ⚡ License

MIT License (permissive). See LICENSE.

© 2025 Andrzej Jabłoński / AudioSzum.pl

## 📁 Repository Structure (this folder)

- `pc/fluke_read.py` – host script: reads from HTTP `/status.json` or USB; writes text for OBS.
- `pc/run_obs_with_fluke.sh` – starts the Python script (HTTP with fallback to USB) and OBS.
- `FlukeBridge-ver3.2-XIAO-ESP32C3.bin` – prebuilt firmware image (ver3.2). For ver3.3, build from source or create a new release.
- `docs/images/*.png` – screenshots used in this README, plus IR schematic.
- `LICENSE`, `CHANGELOG.md`, `.gitignore`, `README.md` – project metadata.

Note: Firmware sources are included under `firmware/`. If you prefer flashing a prebuilt image, upload `FlukeBridge-ver3.2-XIAO-ESP32C3.bin` via the device’s OTA page.

## 🛠 Build and Upload (USB)

1) Open `firmware/ver3.3/ver3.3.ino` (or earlier versions) in Arduino IDE.
2) Select board: Seeed Studio XIAO ESP32‑C3 (ESP32‑C3 core v3.0+). Upload via USB‑C.

## 🎥 Run with OBS on the host

- `cd pc && ./run_obs_with_fluke.sh`
- The script probes HTTP (`/status.json`); if unavailable, it falls back to USB (`/dev/ttyACM0`).

### 🪟 Windows Setup

1) Install Python 3 (from python.org). In PowerShell:
```
py -3 -m pip install --upgrade pip
py -3 -m pip install pyserial
```

2) Access the device
- Via HTTP (recommended): `http://fluke-bridge.local/status.json` (mDNS). If `.local` does not resolve on Windows, use the IP from your router or the index page.
- Via USB: find the COM port in Device Manager (e.g., `COM3`).

3) Run the reader on Windows
```
REM HTTP (Wi‑Fi)
py -3 pc\fluke_read.py --http http://fluke-bridge.local/status.json

REM USB (fallback)
py -3 pc\fluke_read.py --serial COM3
```

4) Optional helper (Windows BAT)
```
REM Probes HTTP, falls back to USB COM3, and tries to launch OBS if installed
pc\run_obs_with_fluke.bat

REM Override defaults:
set URL=http://<device-ip>/status.json
set SER=COM5
pc\run_obs_with_fluke.bat
```

5) OBS on Windows
- Add two Text (GDI+) sources → Read from file: `pc\fluke_value.txt`, `pc\fluke_status.txt` (same folder as the script), or the folder passed with `--dir`.
- Run the reader (HTTP preferred). Files update about once per second.

## 🔁 Wi‑Fi Reset (BOOT)

- Hold BOOT (> 3 s). Device clears Wi‑Fi credentials (NVS + Preferences), restarts into AP mode with captive portal.
## 🔧 Web Config (SOC/VBAT)

Open `/config` (same Basic Auth as OTA). You can:
- Set/clear SoC calibration using current VBAT (FULL/EMPTY)
- Query VBAT, set divider (R1/R2), set scale, or calibrate to DMM voltage

Under the hood, this calls `/api/soc` and `/api/vbat` endpoints that return JSON (usable from scripts, too).

### 🔋 Charging/Full Tuning (Advanced)

Open `/config` and use the “Charging/Full detection – Advanced” section to tune runtime behavior (saved in NVS, no reflash needed). Fields:

- on_dvdt (mV/s) – minimum positive slope of filtered VBAT to enter CHARGING
- on_hold (ms) – how long the slope must hold to confirm CHARGING
- on_mindv (V) – minimum net VBAT rise during on_hold (filters false positives)
- off_dvdt (mV/s) – negative slope magnitude to exit CHARGING
- off_hold (ms) – how long the negative slope must hold to exit CHARGING
- zero_dvdt (mV/s) – absolute |dV/dt| considered ~flat
- zero_hold (ms) – flat duration to exit CHARGING (handles post‑unplug rebound)
- full_hold (ms) – time near VBAT_FULL with flat slope to mark FULL
- freeze_ms – percent “freeze” duration after entering CHARGING (prevents immediate jump)
- chg_rate (ms) – while CHARGING: max +1% every N ms
- d_rate (ms) – while DISCHARGE: ±1% every N ms

API endpoints (JSON):
- GET `/api/tune?action=get` → returns current values
- GET `/api/tune?action=set&on_dvdt=0.8&on_hold=6000&on_mindv=0.015&off_dvdt=0.2&off_hold=8000&zero_dvdt=0.05&zero_hold=120000&full_hold=20000&freeze_ms=15000&chg_rate=30000&d_rate=5000`

Notes:
- All values persist in NVS under the `tune` namespace.
- Defaults are chosen for typical 18650 behavior; adjust if your pack/charger is different.
