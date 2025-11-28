/*
  FlukeBridge ver3.3  (ESP32-C3 XIAO + Fluke 287 IR Bridge)
  ----------------------------------------------------------
  Overview / behavior:
   - Fluke serial: 115200 8N1, RX=20, TX=21 (no invert)
   - BOOT long-press (>3 s on GPIO9): clear Wi‑Fi credentials (Preferences + SDK) and reboot
   - Wi‑Fi: STA with fallback to open AP "FlukeBridge-XXXX" with captive portal
   - Web (sync WebServer):
       * /status.json  (value/unit/status/flags/ol + Wi‑Fi RSSI + battery + uptime + sleep_in)
       * /status.html  (dark dashboard, auto-refresh 1 s)
       * /             (Wi‑Fi config in AP mode; saves to NVS and reboots)
       * /update       (OTA with Basic Auth: admin / fluke1234)

  Battery monitor:
   - Divider 620 kΩ / 470 kΩ -> GPIO2 (A2); LED on GPIO3
   - Filtering:
       * Exponential moving average: vFilt = 0.8 * vPrev + 0.2 * vNow, sampled every 50 ms
       * status.json fields:
           - battery.voltage = filtered voltage
           - battery.v_raw   = instantaneous (unfiltered) voltage
   - LED policy (using filtered voltage):
       * USB present (VBAT > 4.60 V)            -> LED OFF
       * No battery (VBAT < 1.00 V)             -> LED OFF
       * Full (VBAT > 4.15 V)                   -> LED OFF
       * WARN hysteresis: ON ≤ 3.40 V, OFF > 3.50 V -> LED solid ON
       * CRIT hysteresis: ON ≤ 3.00 V, OFF > 3.10 V -> LED blinks ~1 Hz
       * Charging hint: 3.70–4.20 V and not FULL   -> LED solid ON

  Soft-off (battery protection):
   - If CRIT remains continuously for 15 s:
       turn Wi‑Fi OFF then enter esp_deep_sleep_start()
   - Wakeup only via RESET. Deep sleep current: microamps (board dependent)
   - status.json exposes sleep_in: seconds until deep sleep (−1 when not in CRIT)

  Note: USB presence threshold is heuristic (VBAT > 4.60 V measured on the divider).
  If your board routes VBAT differently when USB is present, adjust the threshold.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <HardwareSerial.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_sleep.h>

// ---------------- Pins / HW ----------------
static const int PIN_RX = 20;      // Fluke -> ESP RX (from LM393)
static const int PIN_TX = 21;      // ESP -> Fluke TX (to LM393)
static const int PIN_ADC_VBAT = 2; // battery divider 620k/470k -> GPIO2 (A2)
static const int PIN_LED = 3;      // low-battery / charging LED
static const int BOOT_KEY_PIN = 9; // XIAO ESP32-C3 BOOT (active LOW)

// ---------------- UART (Fluke) -------------
HardwareSerial U(1);
#define FLUKE_BAUD 115200

// ---------------- Wi-Fi / Web --------------
Preferences prefs;
WebServer server(80);
DNSServer dnsServer; // for captive portal in AP mode
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
static const char *FW_VER = "ver3.3.5";
static const char *OTA_USER = "admin";
static const char *OTA_PASS = "fluke1234";

// ---------------- Fluke sample -------------
struct FlukeSample
{
    String raw, value, unit, status, flags;
    bool ol;
};
static FlukeSample gFluke;

// ---------------- Battery monitor ----------
static const float R1_VBAT = 620000.0f; // legacy default (ohms)
static const float R2_VBAT = 470000.0f; // legacy default (ohms)
static const float R1_DEFAULT = R1_VBAT;
static const float R2_DEFAULT = R2_VBAT;
static const float ADC_VREF = 3.3f;
static const float USB_PRESENT_V = 3.90f;    // heuristic USB present (tuned for typical Li-ion charge around 4.0+ V)
static const float USB_PRESENT_HYST = 0.12f; // hysteresis for USB presence (set at +0.12 V, clear at -0.12 V)
static const float VBAT_FULL = 4.15f;
static const float VBAT_NOBAT = 1.00f;
// Hysteresis thresholds (filtered voltage)
static const float VBAT_WARN_ON = 3.40f;
static const float VBAT_WARN_OFF = 3.50f;
static const float VBAT_CRIT_ON = 3.00f;  // enter CRIT at/below 3.00 V
static const float VBAT_CRIT_OFF = 3.10f; // leave CRIT above 3.10 V

// Simple exponential filter for VBAT
static const float VBAT_FILT_ALPHA = 0.80f;        // 0.8*old + 0.2*new
static const uint32_t VBAT_SAMPLE_MS = 50;         // sample interval for ADC
static const uint32_t CRIT_SLEEP_DELAY_MS = 15000; // 15 s in CRIT -> deep sleep
// Display percent smoothing
static const int PCT_MIN_DELTA = 3;              // immediate update if change >= 3%
static const uint32_t PCT_HOLD_MS = 3000;        // otherwise require 3 s stable change
static const int PCT_QUANT = 2;                  // quantize percent to nearest 2%
static const int PCT_RATE_STEP = 1;              // limit change to 1% per step
static const uint32_t PCT_RATE_MS = 1000;        // at most once per 1 s
static const float PCT_VDELTA_MIN = 0.020f;      // ignore VBAT changes <= 20 mV for percent target
static const uint32_t PCT_VDELTA_HOLD_MS = 2000; // require voltage delta to persist 2 s
// dV/dt-based charging/full detection and UI behavior
static const uint32_t DVDT_WIN_MS = 4000; // window for dV/dt estimate
// Defaults for trend-based thresholds (overridable via /config)
static const float DEF_CHG_ON_DVDT_MV_S = 0.30f;        // charging if slope >= +0.30 mV/s (more permissive)
static const float DEF_CHG_OFF_DVDT_MV_S = 0.15f;       // stop charging if negative slope magnitude >= 0.15 mV/s
static const uint32_t DEF_CHG_ON_HOLD_MS = 4000;        // require slope ON for 4 s
static const uint32_t DEF_CHG_OFF_HOLD_MS = 6000;       // require slope OFF for 6 s
static const float DEF_FULL_DVDT_MAX = 0.2f;            // mV/s, near-flat slope to call full
static const uint32_t DEF_FULL_HOLD_MS = 60000;         // 60 s near FULL with flat slope
static const uint32_t DEF_FULL_MIN_CHG_MS = 900000;     // require 15 min in CHARGING before FULL is possible
static const uint32_t DEF_CHG_FREEZE_MS = 15000;        // freeze percent for 15 s on charge start
static const uint32_t DEF_CHG_PCT_RATE_MS = 30000;      // while charging: +1% at most every 30 s
static const uint32_t DEF_DISCH_PCT_RATE_MS = 5000;     // while discharging: 1% per 5 s
static const float DEF_CHG_ON_MIN_DV = 0.010f;          // require >=10 mV rise during ON hold
static const float DEF_CHG_OFF_ZERO_DVDT = 0.03f;       // ~flat slope threshold (|dV/dt| <= 0.03 mV/s)
static const uint32_t DEF_CHG_OFF_ZERO_HOLD_MS = 60000; // 60 s of ~flat to consider OFF

// Runtime tuneable copies (loaded from NVS)
float gCfgChgOnDvdt = DEF_CHG_ON_DVDT_MV_S;
float gCfgChgOffDvdt = DEF_CHG_OFF_DVDT_MV_S;
uint32_t gCfgChgOnHoldMs = DEF_CHG_ON_HOLD_MS;
uint32_t gCfgChgOffHoldMs = DEF_CHG_OFF_HOLD_MS;
float gCfgFullDvdtMax = DEF_FULL_DVDT_MAX;
uint32_t gCfgFullHoldMs = DEF_FULL_HOLD_MS;
uint32_t gCfgFullMinChgMs = DEF_FULL_MIN_CHG_MS;
uint32_t gCfgChgFreezeMs = DEF_CHG_FREEZE_MS;
uint32_t gCfgChgPctRateMs = DEF_CHG_PCT_RATE_MS;
uint32_t gCfgDischPctRateMs = DEF_DISCH_PCT_RATE_MS;
float gCfgChgOnMinDv = DEF_CHG_ON_MIN_DV;
float gCfgChgOffZeroDvdt = DEF_CHG_OFF_ZERO_DVDT;
uint32_t gCfgChgOffZeroHoldMs = DEF_CHG_OFF_ZERO_HOLD_MS;

bool battNoBatt = false, battCrit = false, battWarn = false, battCharging = false, battFull = false, usbPresent = false;
// Runtime-configurable VBAT measurement config
float gR1 = R1_DEFAULT, gR2 = R2_DEFAULT; // ohms
float gVbatScale = 1.0f;                  // additional scale/correction factor
float gVbatRaw = 0.0f, gVbatFilt = 0.0f;
uint32_t gVbatLastMs = 0;
uint32_t critStartMs = 0;
int gPercentDisp = -1;         // stabilized percent reported in JSON/UI
uint32_t gPercentChangeT0 = 0; // timer when small change detected
uint32_t gPercentStepT0 = 0;   // rate-limit timer
float gPctVref = NAN;          // last VBAT used for percent target
int gPctTarget = -1;           // target percent (after deadband + quantization)
uint32_t gPctVdeltaT0 = 0;     // timer for sustained voltage delta
// dV/dt and charging/full detection state
float gDvdtMVs = 0.0f;
uint32_t gDvdtRefT = 0;
float gDvdtRefV = 0.0f;
bool gTrendCharging = false;
bool gTrendFull = false;
uint32_t gChgOnT0 = 0, gChgOffT0 = 0, gFullT0 = 0;
uint32_t gChgFreezeUntil = 0;
float gChgOnV0 = 0.0f;      // voltage at charge-ON candidate start
float gChgMaxV = 0.0f;      // max VBAT seen since charge-ON candidate start
uint32_t gChgZeroT0 = 0;    // timer for near-zero slope OFF
uint32_t gChgSessionT0 = 0; // when CHARGING latched during this session
uint32_t gUsbDropT0 = 0;    // timer for USB-present drop based on falling VBAT
uint32_t gUsbRiseT0 = 0;    // timer for USB-present latch on rising VBAT
uint32_t gUsbPresentT0 = 0; // timestamp when USB became present
float gUsbLatchV = 0.0f;    // VBAT at USB latch time
float gUsbRiseV0 = 0.0f;    // VBAT when soft-latch detection started
uint32_t ledTmr = 0;
bool ledOn = false;
bool usbPresentLatched = false;

// ---------------- BOOT long-press ----------
uint32_t bootT0 = 0;
bool bootPressed = false;

// ---------------- Misc ---------------------
uint32_t tPoll = 0;  // QM poll timer
String usbCmdBuf;    // USB CDC command buffer (for OBS/py client)
bool gDebug = false; // USB debug flag (minimal for now)

// ================= Helpers =================

// Forward declarations for functions used before their definitions
String readLineWithTimeout(uint32_t timeout_ms = 400);
bool sendCmd(const char *cmd, String *payload_out, uint32_t ack_to, uint32_t pay_to);
String wifiModeStr();

// Load/save VBAT measurement config (divider + scale) from NVS
void loadVbatConfig()
{
    Preferences p;
    p.begin("vbat");
    float r1 = p.getFloat("r1", R1_DEFAULT);
    float r2 = p.getFloat("r2", R2_DEFAULT);
    float sc = p.getFloat("scale", 1.0f);
    p.end();
    if (r1 > 1000 && r2 > 100 && r1 < 5e6 && r2 < 5e6)
    {
        gR1 = r1;
        gR2 = r2;
    }
    if (sc > 0.5f && sc < 1.5f)
        gVbatScale = sc;
    else
        gVbatScale = 1.0f;
}

void saveVbatConfig()
{
    Preferences p;
    p.begin("vbat");
    p.putFloat("r1", gR1);
    p.putFloat("r2", gR2);
    p.putFloat("scale", gVbatScale);
    p.end();
}

void clearVbatConfig()
{
    Preferences p;
    p.begin("vbat");
    p.clear();
    p.end();
    gR1 = R1_DEFAULT;
    gR2 = R2_DEFAULT;
    gVbatScale = 1.0f;
}

// Low-level VBAT (battery) voltage read in volts, without extra scaling
float readVBATRawNoScale()
{
    // dummy read to settle S/H cap
    (void)analogReadMilliVolts(PIN_ADC_VBAT);
    const int N = 4;
    uint32_t sum = 0;
    for (int i = 0; i < N; ++i)
    {
        sum += (uint32_t)analogReadMilliVolts(PIN_ADC_VBAT);
    }
    float mv = (float)sum / (float)N; // millivolts at pin
    float v_pin = mv / 1000.0f;
    float v_bat = v_pin * (gR1 + gR2) / gR2;
    return v_bat;
}

// Instantaneous VBAT read from ADC (no filtering), with scale correction
float readVBAT()
{
    float v = readVBATRawNoScale();
    return v * gVbatScale;
}

// Piecewise-linear OCV mapping (Li-ion) voltage -> SoC percent
// Voltages in volts, descending order preferred for clarity.
static const int OCV_N = 16;
static const float OCV_V[OCV_N] = {4.20f, 4.15f, 4.10f, 4.05f, 4.00f, 3.95f, 3.90f, 3.85f, 3.80f, 3.75f, 3.70f, 3.65f, 3.60f, 3.55f, 3.45f, 3.30f};
static const int OCV_PCT[OCV_N] = {100, 96, 92, 88, 84, 79, 72, 62, 50, 40, 30, 22, 16, 10, 5, 0};

int vbatPercentOCV(float v)
{
    if (v >= OCV_V[0])
        return OCV_PCT[0];
    if (v <= OCV_V[OCV_N - 1])
        return OCV_PCT[OCV_N - 1];
    for (int i = 0; i < OCV_N - 1; ++i)
    {
        float vHi = OCV_V[i];
        float vLo = OCV_V[i + 1];
        if (v <= vHi && v >= vLo)
        {
            int pHi = OCV_PCT[i];
            int pLo = OCV_PCT[i + 1];
            float t = (v - vLo) / (vHi - vLo);
            int p = (int)roundf(pLo + t * (pHi - pLo));
            if (p < 0)
                p = 0;
            if (p > 100)
                p = 100;
            return p;
        }
    }
    // Fallback (should not reach)
    int p = (int)roundf((v - 3.30f) * (95.0f / (4.18f - 3.30f)));
    if (p < 0)
        p = 0;
    if (p > 100)
        p = 100;
    return p;
}

// ===== SoC calibration (USB commands set per-cell calibration) =====
static float socVFullCal = NAN;                       // calibrated "full" voltage (measured), persisted in NVS
static float socVEmptyCal = NAN;                      // calibrated "empty" voltage (measured), persisted in NVS
static const float SOC_VREF_FULL = OCV_V[0];          // 4.20 V
static const float SOC_VREF_EMPTY = OCV_V[OCV_N - 1]; // 3.30 V

void loadSocCal()
{
    Preferences p;
    p.begin("soc");
    socVFullCal = p.getFloat("vfull", NAN);
    socVEmptyCal = p.getFloat("vempty", NAN);
    p.end();
}

void saveSocFull(float v)
{
    Preferences p;
    p.begin("soc");
    p.putFloat("vfull", v);
    p.end();
    socVFullCal = v;
}

void saveSocEmpty(float v)
{
    Preferences p;
    p.begin("soc");
    p.putFloat("vempty", v);
    p.end();
    socVEmptyCal = v;
}

bool socCalValid()
{
    return (!isnan(socVFullCal) && !isnan(socVEmptyCal) && (socVFullCal - socVEmptyCal) > 0.05f);
}

// Map measured voltage using calibration so that
// v = socVEmptyCal -> SOC_VREF_EMPTY and v = socVFullCal -> SOC_VREF_FULL
float socNormalizeVoltage(float v)
{
    if (!socCalValid())
        return v;
    float scale = (SOC_VREF_FULL - SOC_VREF_EMPTY) / (socVFullCal - socVEmptyCal);
    float vnorm = SOC_VREF_EMPTY + (v - socVEmptyCal) * scale;
    if (vnorm < SOC_VREF_EMPTY)
        vnorm = SOC_VREF_EMPTY;
    if (vnorm > SOC_VREF_FULL)
        vnorm = SOC_VREF_FULL;
    return vnorm;
}

// Update battery filter at most every VBAT_SAMPLE_MS
void updateBatteryFilter()
{
    uint32_t now = millis();
    if (now - gVbatLastMs < VBAT_SAMPLE_MS)
        return;
    gVbatLastMs = now;
    gVbatRaw = readVBAT();
    if (gVbatFilt <= 0.01f)
    {
        gVbatFilt = gVbatRaw; // initialize filter on first run
    }
    else
    {
        gVbatFilt = VBAT_FILT_ALPHA * gVbatFilt + (1.0f - VBAT_FILT_ALPHA) * gVbatRaw;
    }

    // dV/dt measurement and trend-based charging/full detection
    if (gDvdtRefT == 0)
    {
        gDvdtRefT = now;
        gDvdtRefV = gVbatFilt;
    }
    if (now - gDvdtRefT >= DVDT_WIN_MS)
    {
        float dt = (now - gDvdtRefT) / 1000.0f;
        gDvdtMVs = ((gVbatFilt - gDvdtRefV) / dt) * 1000.0f; // mV/s
        gDvdtRefT = now;
        gDvdtRefV = gVbatFilt;
    }
    // USB presence: latch only on sustained rise; unlatch on sustained fall/drop.
    bool usbPrev = usbPresentLatched;
    const float usbRiseSlope = 0.35f; // mV/s to consider "rising" (aggressive)
    const float usbDropSlope = -0.05f;
    const uint32_t usbRiseHoldMs = 4000;
    const uint32_t usbDropHoldMs = 8000;
    const float usbRiseDelta = 0.05f; // +50 mV during rise window
    const float usbDropDelta = 0.08f; // -80 mV from latch to allow unlatch when not in CV

    if (!usbPresentLatched)
    {
        if (gVbatFilt >= 3.85f && gDvdtMVs >= usbRiseSlope)
        {
            if (gUsbRiseT0 == 0)
            {
                gUsbRiseT0 = now;
                gUsbRiseV0 = gVbatFilt;
            }
            if ((now - gUsbRiseT0) >= usbRiseHoldMs && (gVbatFilt - gUsbRiseV0) >= usbRiseDelta)
            {
                usbPresentLatched = true;
                gUsbPresentT0 = now;
                gUsbLatchV = gVbatFilt;
                gUsbRiseT0 = 0;
                gUsbRiseV0 = 0.0f;
            }
        }
        else
        {
            gUsbRiseT0 = 0;
            gUsbRiseV0 = 0.0f;
        }
    }
    else
    {
        // If we are clearly in CV (VBAT >= 4.05 V), keep USB latched unless a hard drop shows up.
        bool inCvPlateau = (gVbatFilt >= 4.05f);
        if (inCvPlateau)
        {
            gUsbDropT0 = 0;
        }
        else
        {
            bool allowDrop = ((gUsbLatchV - gVbatFilt) >= usbDropDelta && gVbatFilt <= 3.95f) ||
                             (gDvdtMVs <= usbDropSlope && gVbatFilt <= 3.90f) ||
                             (gDvdtMVs <= -0.25f);
            if (allowDrop)
            {
                if (gUsbDropT0 == 0)
                    gUsbDropT0 = now;
                if ((now - gUsbDropT0) >= usbDropHoldMs)
                    usbPresentLatched = false;
            }
            else
            {
                gUsbDropT0 = 0;
            }
        }

        // Long flat near latch voltage → unlatch (no cable if brak netto zysku)
        if (usbPresentLatched && gUsbPresentT0 != 0 && (now - gUsbPresentT0) >= 60000 &&
            (gUsbLatchV - gVbatFilt) >= 0.02f && fabsf(gDvdtMVs) < 0.02f)
        {
            usbPresentLatched = false;
        }

        if (!usbPresentLatched)
        {
            gUsbDropT0 = 0;
            gUsbRiseT0 = 0;
            gUsbRiseV0 = 0.0f;
            gUsbPresentT0 = 0;
            gUsbLatchV = 0.0f;
            gTrendCharging = false;
            gChgSessionT0 = 0;
            gChgOnT0 = gChgOffT0 = 0;
            gChgMaxV = 0.0f;
            gChgZeroT0 = 0;
        }
    }

    if (!usbPrev && usbPresentLatched)
    {
        gUsbPresentT0 = now;
        gUsbLatchV = gVbatFilt;
    }
    else if (usbPrev && !usbPresentLatched)
    {
        gUsbPresentT0 = 0;
        gUsbLatchV = 0.0f;
        gUsbRiseV0 = 0.0f;
    }

    bool usbNowForTrend = usbPresentLatched;
    bool wasCharging = gTrendCharging;
    // FULL detection: use calibrated/normalized voltage near top and flat slope sustained
    // Require USB present and minimum time in CHARGING before FULL can trigger.
    // Treat as FULL when normalized VBAT is within ~50 mV of SOC reference full and slope is near zero
    float vnormForFull = socNormalizeVoltage(gVbatFilt);
    if (usbNowForTrend && gChgSessionT0 != 0 && (now - gChgSessionT0) >= gCfgFullMinChgMs &&
        vnormForFull >= (SOC_VREF_FULL - 0.05f) && fabsf(gDvdtMVs) <= gCfgFullDvdtMax)
    {
        if (gFullT0 == 0)
            gFullT0 = now;
        if (now - gFullT0 >= gCfgFullHoldMs)
            gTrendFull = true;
    }
    else
    {
        gFullT0 = 0;
        gTrendFull = false;
    }
    // CHARGING detection: positive slope sustained, and not full.
    // Only allow CHARGING when USB is present (VBAT above USB_PRESENT_V).
    // If USB is present for a few seconds and VBAT gained >=10 mV (or slope clearly positive),
    // latch CHARGING even without meeting the aggressive slope threshold to avoid false DISCHARGE.
    if (!gTrendFull && usbNowForTrend)
    {
        // Auto-latch charging when USB has been present for a while and VBAT rose a bit
        if (!gTrendCharging && gUsbPresentT0 != 0 && (now - gUsbPresentT0) >= 4000 &&
            (gVbatFilt >= (gUsbLatchV + 0.01f) || gDvdtMVs >= 0.05f))
        {
            gTrendCharging = true;
            gChgSessionT0 = now;
            gChgFreezeUntil = now + gCfgChgFreezeMs;
        }

        if (gDvdtMVs >= gCfgChgOnDvdt)
        {
            if (gChgOnT0 == 0)
            {
                gChgOnT0 = now;
                gChgOnV0 = gVbatFilt;
                gChgMaxV = gVbatFilt;
            }
            if (now - gChgOnT0 >= gCfgChgOnHoldMs)
            {
                if ((gVbatFilt - gChgOnV0) >= gCfgChgOnMinDv)
                    gTrendCharging = true;
            }
            // track the maximum VBAT during ON candidate / charging phase
            if (gVbatFilt > gChgMaxV)
                gChgMaxV = gVbatFilt;
        }
        else
        {
            gChgOnT0 = 0;
        }
        // NOTE: negative-slope based CHARGING exit was removed for this
        // hardware revision; CHARGING now turns off when USB is no longer
        // present or when FULL is reached. This avoids spurious exits on
        // small voltage sags during constant-voltage charging.
        // also consider OFF if slope remains ~zero for a long time AND
        // there was no meaningful net voltage gain since charge-ON candidate.
        // This prevents falsely dropping out of CHARGING during CV plateau
        // where slope is near zero but cell is still charging.
        if (fabsf(gDvdtMVs) <= gCfgChgOffZeroDvdt)
        {
            // require tiny net gain (<5 mV) since gChgOnV0 to allow zero-slope OFF
            if ((gVbatFilt - gChgOnV0) < 0.005f)
            {
                if (gChgZeroT0 == 0)
                    gChgZeroT0 = now;
                if (now - gChgZeroT0 >= gCfgChgOffZeroHoldMs)
                    gTrendCharging = false;
            }
            else
            {
                gChgZeroT0 = 0; // net gain present -> keep charging latched
            }
        }
        else
        {
            gChgZeroT0 = 0;
        }
    }
    else
    {
        gTrendCharging = false;
        gChgOnT0 = gChgOffT0 = 0;
        gChgMaxV = 0.0f;
        gChgZeroT0 = 0;
    }
    if (!wasCharging && gTrendCharging)
    {
        gChgFreezeUntil = now + gCfgChgFreezeMs;
        gChgSessionT0 = now;
    }
    if (!gTrendCharging)
        gChgSessionT0 = 0;

    // Update stabilized percentage with hysteresis and hold time
    float vForPct = socNormalizeVoltage(gVbatFilt);
    int pCand = vbatPercentOCV(vForPct);
    if (PCT_QUANT > 1)
    {
        int step = PCT_QUANT;
        pCand = ((pCand + step / 2) / step) * step; // round to nearest step
        if (pCand > 100)
            pCand = 100;
        if (pCand < 0)
            pCand = 0;
    }
    // Deadband on voltage: update target percent only when VBAT moves by > PCT_VDELTA_MIN
    if (isnan(gPctVref) || gPctTarget < 0)
    {
        gPctVref = vForPct;
        gPctTarget = pCand;
        gPctVdeltaT0 = 0;
    }
    else if (fabsf(vForPct - gPctVref) > PCT_VDELTA_MIN)
    {
        if (gPctVdeltaT0 == 0)
            gPctVdeltaT0 = now;
        if (now - gPctVdeltaT0 >= PCT_VDELTA_HOLD_MS)
        {
            gPctVref = vForPct;
            gPctTarget = pCand;
            gPctVdeltaT0 = 0;
        }
    }
    else
    {
        gPctVdeltaT0 = 0; // delta not big/sustained enough
    }
    int pNow = gPctTarget;
    if (gTrendFull)
    {
        gPercentDisp = 100;
        gPercentChangeT0 = 0;
        gPercentStepT0 = now;
    }
    else if (gPercentDisp < 0)
    {
        gPercentDisp = pNow;
        gPercentChangeT0 = 0;
        gPercentStepT0 = now;
    }
    else
    {
        int diff = pNow - gPercentDisp;
        bool allowed = false;
        // Freeze after charging start
        if (gTrendCharging && (int32_t)(gChgFreezeUntil - now) > 0)
        {
            allowed = false;
        }
        else if (abs(diff) >= PCT_MIN_DELTA)
        {
            allowed = true;
            gPercentChangeT0 = 0;
        }
        else if (diff != 0)
        {
            if (gPercentChangeT0 == 0)
                gPercentChangeT0 = now;
            if (now - gPercentChangeT0 >= PCT_HOLD_MS)
            {
                allowed = true;
                gPercentChangeT0 = 0;
            }
        }
        else
        {
            gPercentChangeT0 = 0;
        }

        uint32_t rateMs = gTrendCharging ? gCfgChgPctRateMs : gCfgDischPctRateMs;
        if (gTrendCharging && diff < 0)
            allowed = false; // don't go down while charging
        // Strong guard: do not increase percent when not in CHARGING
        // (prevents unrealistic growth in DISCHARGE during CV-like behavior)
        if (diff > 0 && !gTrendCharging)
            allowed = false;
        if (allowed && (now - gPercentStepT0 >= rateMs))
        {
            int step = (diff > 0) ? PCT_RATE_STEP : -PCT_RATE_STEP;
            if (abs(diff) <= PCT_RATE_STEP)
                step = diff;
            gPercentDisp += step;
            if (gPercentDisp < 0)
                gPercentDisp = 0;
            if (gPercentDisp > 100)
                gPercentDisp = 100;
            gPercentStepT0 = now;
        }
    }
}

int vbatPercent(float v)
{
    if (v <= 3.30f)
        return 5;
    if (v >= 4.18f)
        return 100;
    return (int)roundf((v - 3.30f) * (95.0f / (4.18f - 3.30f)));
}

// LED policy with filtered VBAT, hysteresis and CRIT->sleep
void batteryLedTask()
{
    updateBatteryFilter();
    float vb = gVbatFilt;

    usbPresent = usbPresentLatched; // heuristics with hysteresis computed in updateBatteryFilter
    battNoBatt = (vb < VBAT_NOBAT);
    // Use trend-based full/charging states computed in updateBatteryFilter
    battFull = (!battNoBatt && gTrendFull);

    // Hysteresis on WARN/CRIT (disabled when USB present or no-batt)
    if (usbPresent || battNoBatt)
    {
        battCrit = false;
        battWarn = false;
        critStartMs = 0;
    }
    else
    {
        // Critical state with hysteresis
        if (battCrit)
        {
            if (vb > VBAT_CRIT_OFF)
            {
                battCrit = false;
                critStartMs = 0;
            }
        }
        else
        {
            if (vb <= VBAT_CRIT_ON)
            {
                battCrit = true;
                critStartMs = millis();
            }
        }

        // Warn state with hysteresis (only when not critical)
        if (!battCrit)
        {
            if (battWarn)
            {
                if (vb > VBAT_WARN_OFF)
                    battWarn = false;
            }
            else
            {
                if (vb <= VBAT_WARN_ON)
                    battWarn = true;
            }
        }
        else
        {
            battWarn = false;
        }
    }

    // Charging: trend-based
    battCharging = (!battFull && gTrendCharging);

    // Apply LED rules
    uint32_t now = millis();
    // OFF cases:
    if (battNoBatt || usbPresent || (!battWarn && !battCrit) || battFull)
    {
        digitalWrite(PIN_LED, LOW);
        ledOn = false;
        return;
    }
    // Critical BLINK:
    if (battCrit)
    {
        if (!ledOn && now - ledTmr >= 500)
        {
            digitalWrite(PIN_LED, HIGH);
            ledOn = true;
            ledTmr = now;
        }
        else if (ledOn && now - ledTmr >= 500)
        {
            digitalWrite(PIN_LED, LOW);
            ledOn = false;
            ledTmr = now;
        }
        // Auto deep sleep after continuous CRIT for CRIT_SLEEP_DELAY_MS
        if (critStartMs != 0 && (now - critStartMs) >= CRIT_SLEEP_DELAY_MS)
        {
            Serial.println("[BAT] Critical for 15 s -> deep sleep");
            WiFi.mode(WIFI_OFF);
            delay(20);
            digitalWrite(PIN_LED, LOW);
            delay(10);
            esp_deep_sleep_start();
        }
        return;
    }
    // Low / Charging: solid ON
    if (battWarn || battCharging)
    {
        digitalWrite(PIN_LED, HIGH);
        ledOn = true;
        return;
    }
    // Fallback OFF
    digitalWrite(PIN_LED, LOW);
    ledOn = false;
}

// BOOT long-press handler
void wifiFactoryResetAndReboot()
{
    Serial.println("\n[WiFi] Clearing credentials (NVS) and rebooting...");
    // Erase SDK Wi-Fi credentials and our stored prefs (SSID/PASS)
    WiFi.disconnect(true, true);
    prefs.begin("wifi");
    prefs.clear();
    prefs.end();
    // small blink
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(PIN_LED, HIGH);
        delay(100);
        digitalWrite(PIN_LED, LOW);
        delay(100);
    }
    delay(200);
    ESP.restart();
}

void bootLongPressTask()
{
    int lev = digitalRead(BOOT_KEY_PIN); // LOW = pressed
    uint32_t now = millis();
    if (lev == LOW)
    {
        if (!bootPressed)
        {
            bootPressed = true;
            bootT0 = now;
        }
        if (bootPressed && (now - bootT0 > 3000))
        {
            bootPressed = false;
            wifiFactoryResetAndReboot();
        }
    }
    else
    {
        bootPressed = false;
    }
}

// Read a line from USB Serial ending with \r or \n (non-blocking).
// Returns true if a full line was captured in out (without CR/LF), false otherwise.
bool readUsbLine(String &out)
{
    bool got = false;
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n')
        {
            if (usbCmdBuf.length() > 0)
            {
                out = usbCmdBuf;
                usbCmdBuf = "";
                got = true;
            }
        }
        else if ((uint8_t)c >= 0x20)
        {
            usbCmdBuf += c;
        }
        // ignore other control chars
        if (got)
            break;
    }
    return got;
}

// Handle simple line-based USB commands from host (e.g. "QM", "QS").
void handleUsbCommands()
{
    String line;
    if (!readUsbLine(line))
        return;

    line.trim();
    line.toUpperCase();

    if (line == "PING")
    {
        String mode = wifiModeStr();
        String ip = (mode == "AP") ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
        Serial.print("PONG ");
        Serial.print(FW_VER);
        Serial.print(" ");
        Serial.print(mode);
        if (ip.length())
        {
            Serial.print(" ");
            Serial.print(ip);
        }
        Serial.print("\r");
        return;
    }
    if (line == "VER")
    {
        Serial.print(FW_VER);
        Serial.print("\r");
        return;
    }
    if (line == "MODE")
    {
        Serial.print(wifiModeStr());
        Serial.print("\r");
        return;
    }
    if (line == "WIFI?")
    {
        String mode = wifiModeStr();
        String ssid = WiFi.SSID();
        String ip = (mode == "AP") ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
        Serial.print(mode);
        Serial.print(" ");
        Serial.print(ssid);
        if (ip.length())
        {
            Serial.print(" ");
            Serial.print(ip);
        }
        Serial.print("\r");
        return;
    }
    if (line == "DEBUG ON")
    {
        gDebug = true;
        Serial.print("OK\r");
        return;
    }
    if (line == "DEBUG OFF")
    {
        gDebug = false;
        Serial.print("OK\r");
        return;
    }
    if (line == "DEBUG STATUS")
    {
        Serial.print(gDebug ? "DEBUG ON\r" : "DEBUG OFF\r");
        return;
    }
    if (line == "VBAT?")
    {
        float v_raw = readVBATRawNoScale();
        float v = v_raw * gVbatScale;
        Serial.print("VBAT ");
        Serial.print(v, 3);
        Serial.print("V (raw=");
        Serial.print(v_raw, 3);
        Serial.print("V) ");
        Serial.print("DIV=");
        Serial.print((int)gR1);
        Serial.print("/");
        Serial.print((int)gR2);
        Serial.print(" ");
        Serial.print("SCALE=");
        Serial.print(gVbatScale, 4);
        Serial.print("\r");
        return;
    }
    if (line.startsWith("VBAT DIV "))
    {
        // Format: VBAT DIV <R1_ohm> <R2_ohm>
        float r1 = 0, r2 = 0;
        int n = sscanf(line.c_str() + 9, "%f %f", &r1, &r2);
        if (n == 2 && r1 > 1000 && r2 > 100 && r1 < 5e6 && r2 < 5e6)
        {
            gR1 = r1;
            gR2 = r2;
            saveVbatConfig();
            Serial.print("OK DIV ");
            Serial.print((int)gR1);
            Serial.print("/");
            Serial.print((int)gR2);
            Serial.print("\r");
        }
        else
        {
            Serial.print("ERR DIV\r");
        }
        return;
    }
    if (line.startsWith("VBAT SCALE "))
    {
        float k = 0;
        int n = sscanf(line.c_str() + 11, "%f", &k);
        if (n == 1 && k > 0.5f && k < 1.5f)
        {
            gVbatScale = k;
            saveVbatConfig();
            Serial.print("OK SCALE ");
            Serial.print(gVbatScale, 4);
            Serial.print("\r");
        }
        else
        {
            Serial.print("ERR SCALE\r");
        }
        return;
    }
    if (line.startsWith("VBAT CAL "))
    {
        // Format: VBAT CAL <volts>
        float vref = 0;
        int n = sscanf(line.c_str() + 9, "%f", &vref);
        if (n == 1 && vref > 2.5f && vref < 4.5f)
        {
            float v_raw = readVBATRawNoScale();
            if (v_raw > 2.5f)
            {
                gVbatScale = vref / v_raw;
                saveVbatConfig();
                Serial.print("OK SCALE ");
                Serial.print(gVbatScale, 4);
                Serial.print(" (v_raw=");
                Serial.print(v_raw, 3);
                Serial.print("V)\r");
            }
            else
            {
                Serial.print("ERR CAL RAW\r");
            }
        }
        else
        {
            Serial.print("ERR CAL\r");
        }
        return;
    }
    if (line == "VBAT CLEAR")
    {
        clearVbatConfig();
        saveVbatConfig();
        Serial.print("OK VBAT CLEARED\r");
        return;
    }
    if (line == "SOC?")
    {
        Serial.print("SOC ");
        Serial.print(socCalValid() ? "ACTIVE " : "INACTIVE ");
        Serial.print("FULL=");
        Serial.print(isnan(socVFullCal) ? -1 : socVFullCal, 2);
        Serial.print("V EMPTY=");
        Serial.print(isnan(socVEmptyCal) ? -1 : socVEmptyCal, 2);
        Serial.print("V\r");
        return;
    }
    if (line == "SOC SET_FULL")
    {
        float v = (gVbatFilt > 0.01f) ? gVbatFilt : readVBAT();
        saveSocFull(v);
        Serial.print("OK FULL=");
        Serial.print(v, 3);
        Serial.print("V\r");
        return;
    }
    if (line == "SOC SET_EMPTY")
    {
        float v = (gVbatFilt > 0.01f) ? gVbatFilt : readVBAT();
        saveSocEmpty(v);
        Serial.print("OK EMPTY=");
        Serial.print(v, 3);
        Serial.print("V\r");
        return;
    }
    if (line == "SOC CLEAR")
    {
        Preferences p;
        p.begin("soc");
        p.remove("vfull");
        p.remove("vempty");
        p.end();
        socVFullCal = NAN;
        socVEmptyCal = NAN;
        Serial.print("OK CLEARED\r");
        return;
    }
    if (line == "QM")
    {
        String pl;
        if (sendCmd("QM\r", &pl, 250, 900))
        {
            Serial.print(pl);
            Serial.print("\r");
        }
        else
        {
            Serial.print("\r");
        }
        return;
    }
    if (line == "QS")
    {
        String pl;
        if (sendCmd("QS\r", &pl, 250, 900))
        {
            Serial.print(pl);
            Serial.print("\r");
        }
        else
        {
            Serial.print("\r");
        }
        return;
    }
    if (line == "ID")
    {
        String pl;
        if (sendCmd("ID\r", &pl, 400, 900))
        {
            Serial.print(pl);
            Serial.print("\r");
        }
        else
        {
            Serial.print("\r");
        }
        return;
    }
    if (line == "QDDA")
    {
        String pl;
        if (sendCmd("QDDA\r", &pl, 400, 1200))
        {
            Serial.print(pl);
            Serial.print("\r");
        }
        else
        {
            Serial.print("\r");
        }
        return;
    }
    if (line == "RI")
    {
        sendCmd("RI\r", nullptr, 800, 0);
        Serial.print("\r");
        return;
    }
    if (line == "RMP")
    {
        sendCmd("RMP\r", nullptr, 800, 0);
        Serial.print("\r");
        return;
    }
    // Unknown command -> just CR to keep reader in sync
    Serial.print("\r");
}

String wifiModeStr()
{
    wifi_mode_t m = WiFi.getMode();
    if (m == WIFI_AP)
        return "AP";
    if (m == WIFI_STA)
        return "STA";
    if (m == WIFI_AP_STA)
        return "AP+STA";
    return "OFF";
}

// Fluke line parser
void parseFlukeLine(const String &lineIn)
{
    gFluke.raw = lineIn;
    String t[4];
    int idx = 0, start = 0;
    for (int i = 0; i < lineIn.length() && idx < 4; i++)
    {
        if (lineIn[i] == ',')
        {
            t[idx++] = lineIn.substring(start, i);
            start = i + 1;
        }
    }
    if (idx < 4 && start <= lineIn.length())
        t[idx++] = lineIn.substring(start);

    gFluke.value = (idx > 0) ? t[0] : "";
    gFluke.unit = (idx > 1) ? t[1] : "";
    gFluke.status = (idx > 2) ? t[2] : "";
    gFluke.flags = (idx > 3) ? t[3] : "";

    gFluke.value.trim();
    gFluke.unit.trim();
    gFluke.status.trim();
    gFluke.flags.trim();

    gFluke.ol = false;
    if (gFluke.value == "OL")
        gFluke.ol = true;
    if (gFluke.value.indexOf("E+37") >= 0)
        gFluke.ol = true; // +9.999...E+37
}
// Try to parse String to double (accepts E notation). Returns true on success.
// Parse "1.23E-3" → double
bool toDouble(const String &s, double &out)
{
    if (s.length() == 0)
        return false;
    char buf[64];
    size_t n = s.length();
    if (n >= sizeof(buf))
        n = sizeof(buf) - 1;
    for (size_t i = 0; i < n; i++)
        buf[i] = s[i];
    buf[n] = '\0';
    char *endp = nullptr;
    double v = strtod(buf, &endp);
    if (endp == buf)
        return false;
    out = v;
    return true;
}

// --------- SI formatters ----------
String formatVolts(double v)
{
    double a = fabs(v);
    char o[32];
    if (a < 1e-3)
        snprintf(o, sizeof(o), "%.0f µV", v * 1e6);
    else if (a < 1.0)
        snprintf(o, sizeof(o), "%.3f mV", v * 1e3);
    else
        snprintf(o, sizeof(o), "%.3f V", v);
    return String(o);
}

String formatAmps(double v)
{
    double a = fabs(v);
    char o[32];
    if (a < 1e-6)
        snprintf(o, sizeof(o), "%.0f nA", v * 1e9);
    else if (a < 1e-3)
        snprintf(o, sizeof(o), "%.0f µA", v * 1e6);
    else if (a < 1.0)
        snprintf(o, sizeof(o), "%.3f mA", v * 1e3);
    else
        snprintf(o, sizeof(o), "%.3f A", v);
    return String(o);
}

String formatOhms(double v)
{
    double a = fabs(v);
    char o[32];
    if (a < 1e3)
        snprintf(o, sizeof(o), "%.3f Ω", v);
    else if (a < 1e6)
        snprintf(o, sizeof(o), "%.3f kΩ", v / 1e3);
    else if (a < 1e9)
        snprintf(o, sizeof(o), "%.3f MΩ", v / 1e6);
    else
        snprintf(o, sizeof(o), "%.3f GΩ", v / 1e9);
    return String(o);
}

String formatFarads(double v)
{
    double a = fabs(v);
    char o[32];
    if (a < 1e-9)
        snprintf(o, sizeof(o), "%.0f pF", v * 1e12);
    else if (a < 1e-6)
        snprintf(o, sizeof(o), "%.0f nF", v * 1e9);
    else if (a < 1e-3)
        snprintf(o, sizeof(o), "%.2f µF", v * 1e6);
    else if (a < 1.0)
        snprintf(o, sizeof(o), "%.3f mF", v * 1e3);
    else
        snprintf(o, sizeof(o), "%.3f F", v);
    return String(o);
}

String formatHertz(double v)
{
    double a = fabs(v);
    char o[32];
    if (a < 1e3)
        snprintf(o, sizeof(o), "%.2f Hz", v);
    else if (a < 1e6)
        snprintf(o, sizeof(o), "%.3f kHz", v / 1e3);
    else
        snprintf(o, sizeof(o), "%.3f MHz", v / 1e6);
    return String(o);
}

String formatPercent(double v)
{
    char o[32];
    snprintf(o, sizeof(o), "%.2f %%", v);
    return String(o);
}

String formatCelsius(double v)
{
    char o[32];
    snprintf(o, sizeof(o), "%.1f °C", v);
    return String(o);
}
String formatFahrenheit(double v)
{
    char o[32];
    snprintf(o, sizeof(o), "%.1f °F", v);
    return String(o);
}

// Returns a pretty value with unit (based on gFluke.unit)
String prettyFlukeValue(const String &value, const String &unitRaw, const String &status, const String &flags)
{
    // OL / open circuit
    if (value == "OL" || value.indexOf("E+37") >= 0)
        return "OL";

    // Parse numerically (for most modes)
    double v;
    bool ok = toDouble(value, v);

    // Normalize unit/status/flags to UPPER
    String u = unitRaw;
    u.trim();
    u.toUpperCase();
    String st = status;
    st.trim();
    st.toUpperCase();
    String fl = flags;
    fl.trim();
    fl.toUpperCase();

    // Special modes per Fluke:
    // Diode: some models report forward drop in V (unit=VDC) + flag/status "DIODE"
    bool isDiode = (u.indexOf("DIODE") >= 0) || (st.indexOf("DIODE") >= 0) || (fl.indexOf("DIODE") >= 0);
    if (isDiode && ok)
    {
        // e.g. 0.612 Vf
        char o[32];
        snprintf(o, sizeof(o), "%.3f Vf", v);
        return String(o);
    }

    // Continuity: usually OHM + flag/status "CONT" (buzzer threshold depends on meter)
    bool isCont = (st.indexOf("CONT") >= 0) || (fl.indexOf("CONT") >= 0) || (st.indexOf("CONTINUITY") >= 0);
    if (isCont && ok)
    {
        // Show BEEP for very low resistance, otherwise format ohms
        if (v <= 50.0)
            return "BEEP"; // default threshold ~50 Ω (adjust if needed)
        return formatOhms(v);
    }

    // Temperature
    if ((u == "CEL") || (u == "°C"))
    {
        if (ok)
            return formatCelsius(v);
        return value + " °C";
    }
    if ((u == "FAH") || (u == "°F"))
    {
        if (ok)
            return formatFahrenheit(v);
        return value + " °F";
    }

    // Resistance
    if ((u == "OHM") || (u == "Ω"))
    {
        if (ok)
            return formatOhms(v);
        return value + " Ω";
    }

    // Voltage
    if ((u == "VDC") || (u == "VAC") || (u == "V"))
    {
        if (ok)
            return formatVolts(v);
        return value + " V";
    }

    // Current
    if ((u == "ADC") || (u == "AAC") || (u == "A"))
    {
        if (ok)
            return formatAmps(v);
        return value + " A";
    }

    // Frequency
    if ((u == "HZ") || (u == "FREQ"))
    {
        if (ok)
            return formatHertz(v);
        return value + " Hz";
    }

    // Duty / percentage
    if ((u == "PCT") || (u == "PERCENT") || (u == "%"))
    {
        if (ok)
            return formatPercent(v);
        return value + " %";
    }

    // Capacitance (some Fluke report "FARAD", "F", "CAP")
    if ((u == "FARAD") || (u == "CAP") || (u == "F"))
    {
        if (ok)
            return formatFarads(v);
        return value + " F";
    }

    // Other / fallback: compact 6g + unit
    if (ok)
    {
        char o[48];
        snprintf(o, sizeof(o), "%.6g", v);
        return String(o) + (u.length() ? (" " + u) : "");
    }
    return value + (unitRaw.length() ? (" " + unitRaw) : "");
}

// Read CR-terminated line from Fluke
String readLineWithTimeout(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    String s;
    while (millis() - t0 < timeout_ms)
    {
        while (U.available())
        {
            char c = (char)U.read();
            if (c == '\r')
                return s;
            if (c != '\n')
                s += c;
        }
        delay(1);
    }
    return s;
}

// Send Fluke command, capture ACK + optional payload
bool sendCmd(const char *cmd, String *payload_out,
             uint32_t ack_to, uint32_t pay_to)
{
    U.write(cmd);
    String ack = readLineWithTimeout(ack_to);
    if (ack.length() == 0)
        ack = readLineWithTimeout(ack_to);
    if (ack.length() != 1)
    {
        Serial.print("[ACK?] ");
        Serial.println(ack);
        return false;
    }
    if (payload_out)
    {
        String pl = readLineWithTimeout(pay_to);
        *payload_out = pl;
        if (pl.length())
            parseFlukeLine(pl);
    }
    return true;
}

// JSON escaper (basic)
String safe(const String &s)
{
    String t = s;
    t.replace("\r", "");
    t.replace("\n", "");
    t.replace("\"", "\\\"");
    return t;
}

// =============== Web Config (SOC/VBAT over HTTP) ===============

void handleConfigPage()
{
    if (!otaAuth())
        return; // reuse OTA Basic Auth
    static const char PROGMEM page[] = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlukeBridge - Config</title>
<style>*{box-sizing:border-box}body{font-family:sans-serif;padding:16px;margin:0;background:#0b1220;color:#e8eef7}
.wrap{max-width:920px;margin:0 auto}
section{background:#121a2a;border:1px solid #1b2540;border-radius:12px;padding:16px;margin:14px 0}
label{display:block;margin:8px 0 4px}
input{width:100%;max-width:100%;padding:10px;border-radius:8px;border:1px solid #1b2540;background:#0e1526;color:#e8eef7}
button{padding:10px 14px;border:1px solid #1b2540;background:#0e1526;color:#e8eef7;border-radius:10px;cursor:pointer;margin-top:8px}
small{display:block;opacity:.75;margin-top:4px}
pre{background:#0e1526;border:1px solid #1b2540;border-radius:10px;padding:12px;white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere;overflow:auto;max-width:100%}
.form-narrow{max-width:100%;margin:0}
.btn-row{display:flex;flex-wrap:wrap;gap:10px;justify-content:flex-start;margin:8px 0 12px}
.grid-adv{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin-top:10px}
.grid-adv>div{min-width:0;overflow:hidden}
@media (max-width:980px){.grid-adv{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media (max-width:600px){.grid-adv{grid-template-columns:1fr}}
@media (max-width:520px){button{width:100%}}
</style></head><body>
<div class="wrap">
<h2>Config (SOC/VBAT)</h2>
<section><h3>Battery SoC</h3>
<div class="btn-row">
  <button onclick="doSoc('status')">SOC Status</button>
  <button onclick="doSoc('set_full')">Set FULL (use current VBAT)</button>
  <button onclick="doSoc('set_empty')">Set EMPTY (use current VBAT)</button>
  <button onclick="doSoc('clear')">Clear SOC Calibration</button>
</div>
</section>
<section><h3>Charging/Full detection - Advanced</h3>
<div class="btn-row"><button onclick="loadTune()">Load current</button></div>
<div class="grid-adv">
  <div><label>on_dvdt (mV/s)</label><input id="on_dvdt" placeholder="0.8"><small>Minimum positive slope to enter CHARGING</small></div>
  <div><label>on_hold (ms)</label><input id="on_hold" placeholder="6000"><small>Time the slope must hold (enter)</small></div>
  <div><label>on_mindv (V)</label><input id="on_mindv" placeholder="0.015"><small>Minimum net VBAT rise during on_hold</small></div>
  <div><label>off_dvdt (mV/s)</label><input id="off_dvdt" placeholder="0.2"><small>Negative slope magnitude to exit CHARGING</small></div>
  <div><label>off_hold (ms)</label><input id="off_hold" placeholder="8000"><small>Time the negative slope must hold (exit)</small></div>
  <div><label>zero_dvdt (mV/s)</label><input id="zero_dvdt" placeholder="0.05"><small>|dV/dt| considered ~flat</small></div>
  <div><label>zero_hold (ms)</label><input id="zero_hold" placeholder="120000"><small>Flat slope duration to exit CHARGING</small></div>
  <div><label>full_hold (ms)</label><input id="full_hold" placeholder="20000"><small>Time near VBAT_FULL with flat slope to mark FULL</small></div>
  <div><label>full_min_chg (ms)</label><input id="full_min_chg" placeholder="900000"><small>Minimum time in CHARGING before FULL can trigger</small></div>
  <div><label>freeze_ms</label><input id="freeze_ms" placeholder="15000"><small>Freeze percent after CHARGING start</small></div>
  <div><label>chg_rate (ms)</label><input id="chg_rate" placeholder="30000"><small>Charging: max +1% every N ms</small></div>
  <div><label>d_rate (ms)</label><input id="d_rate" placeholder="5000"><small>Discharge: ±1% every N ms</small></div>
</div>
<div class="btn-row">
  <button onclick="saveTune()">Save tuning</button>
  <button onclick="resetTune()">Restore defaults</button>
</div>
</section>
<section><h3>VBAT Measurement</h3>
<div class="btn-row"><button onclick="doVbat('query')">VBAT?</button></div>
<div class="form-narrow">
  <label>R1 (ohms)</label><input id="r1" placeholder="620000">
  <label>R2 (ohms)</label><input id="r2" placeholder="470000">
  <div class="btn-row"><button onclick="setDiv()">Set Divider</button></div>
  <label>Scale (k)</label><input id="scale" placeholder="1.0000">
  <div class="btn-row"><button onclick="setScale()">Set Scale</button></div>
  <label>Calibrate to DMM (volts)</label><input id="calv" placeholder="3.640">
  <div class="btn-row"><button onclick="calVbat()">VBAT CAL</button></div>
</div>
</section>
<section><h3>Result</h3><pre id="out">—</pre></section>
<p><a href="/">Home</a> • <a href="/status.html">Dashboard</a> • <a href="/update">OTA</a></p>
</div>
<script>
const auth='Basic '+btoa('admin:fluke1234');
const outEl=document.getElementById('out');
const fetchOpts={headers:{Authorization:auth,'Cache-Control':'no-cache','Pragma':'no-cache'}};
const setOut=(txt)=>{ if(outEl) outEl.textContent=txt; };
async function doSoc(action){
  try{
    setOut('...');
    const r=await fetch('/api/soc?action='+encodeURIComponent(action),fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}
async function doVbat(action){
  try{
    setOut('...');
    const r=await fetch('/api/vbat?action='+encodeURIComponent(action),fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}
async function setDiv(){
  const r1=document.getElementById('r1').value||''; const r2=document.getElementById('r2').value||'';
  try{
    setOut('...');
    const r=await fetch('/api/vbat?action=set_div&r1='+encodeURIComponent(r1)+'&r2='+encodeURIComponent(r2),fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}
async function setScale(){
  const k=document.getElementById('scale').value||'';
  try{
    setOut('...');
    const r=await fetch('/api/vbat?action=set_scale&k='+encodeURIComponent(k),fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}
async function calVbat(){
  const v=document.getElementById('calv').value||'';
  try{
    setOut('...');
    const r=await fetch('/api/vbat?action=calibrate&v='+encodeURIComponent(v),fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}

async function loadTune(){
  try{
    setOut('...');
    const r=await fetch('/api/tune?action=get',fetchOpts); const j=await r.json();
    for(const k in j){ const el=document.getElementById(k); if(el){ el.value=j[k]; } }
    setOut(JSON.stringify(j));
  }catch(e){ setOut('ERR '+e); }
}
async function saveTune(){
  const ids=['on_dvdt','off_dvdt','on_hold','off_hold','full_hold','full_min_chg','freeze_ms','chg_rate','d_rate','on_mindv','zero_dvdt','zero_hold'];
  const qs=ids.map(id=> id+'='+encodeURIComponent(document.getElementById(id).value||'')).join('&');
  try{
    setOut('...');
    const r=await fetch('/api/tune?action=set&'+qs,fetchOpts);
    setOut(await r.text());
  }catch(e){ setOut('ERR '+e); }
}
async function resetTune(){
  try{
    setOut('...');
    const r=await fetch('/api/tune?action=reset',fetchOpts);
    const t=await r.text();
    setOut(t);
    try{ await loadTune(); }catch(e){}
  }catch(e){ setOut('ERR '+e); }
}
window.addEventListener('load', ()=>{ try{ loadTune(); doSoc('status'); doVbat('query'); }catch(e){ setOut('ERR init '+e); } });
</script></body></html>
)HTML";
    server.send_P(200, "text/html; charset=utf-8", page);
}

void handleApiSoc()
{
    if (!otaAuth())
        return;
    String action = server.arg("action");
    if (action == "status")
    {
        String s = "{\"ok\":true,\"soc\":\"";
        s += socCalValid() ? "ACTIVE" : "INACTIVE";
        s += "\",\"vfull\":" + String(isnan(socVFullCal) ? -1 : socVFullCal, 3);
        s += ",\"vempty\":" + String(isnan(socVEmptyCal) ? -1 : socVEmptyCal, 3) + "}";
        server.send(200, "application/json", s);
        return;
    }
    if (action == "set_full")
    {
        float v = (gVbatFilt > 0.01f) ? gVbatFilt : readVBAT();
        saveSocFull(v);
        server.send(200, "application/json", String("{\"ok\":true,\"vfull\":" + String(v, 3) + "}"));
        return;
    }
    if (action == "set_empty")
    {
        float v = (gVbatFilt > 0.01f) ? gVbatFilt : readVBAT();
        saveSocEmpty(v);
        server.send(200, "application/json", String("{\"ok\":true,\"vempty\":" + String(v, 3) + "}"));
        return;
    }
    if (action == "clear")
    {
        Preferences p;
        p.begin("soc");
        p.remove("vfull");
        p.remove("vempty");
        p.end();
        socVFullCal = NAN;
        socVEmptyCal = NAN;
        server.send(200, "application/json", "{\"ok\":true}");
        return;
    }
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_action\"}");
}

void handleApiVbat()
{
    if (!otaAuth())
        return;
    String action = server.arg("action");
    if (action == "query")
    {
        float v_raw = readVBATRawNoScale();
        float v = v_raw * gVbatScale;
        String s = "{\"ok\":true,\"v\":" + String(v, 3) + ",\"raw\":" + String(v_raw, 3) + ",\"r1\":" + String((int)gR1) + ",\"r2\":" + String((int)gR2) + ",\"scale\":" + String(gVbatScale, 4) + "}";
        server.send(200, "application/json", s);
        return;
    }
    if (action == "set_div")
    {
        float r1 = server.hasArg("r1") ? server.arg("r1").toFloat() : 0;
        float r2 = server.hasArg("r2") ? server.arg("r2").toFloat() : 0;
        if (r1 > 1000 && r2 > 100 && r1 < 5e6 && r2 < 5e6)
        {
            gR1 = r1;
            gR2 = r2;
            saveVbatConfig();
            server.send(200, "application/json", "{\"ok\":true}");
        }
        else
            server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_divider\"}");
        return;
    }
    if (action == "set_scale")
    {
        float k = server.hasArg("k") ? server.arg("k").toFloat() : 0;
        if (k > 0.5f && k < 1.5f)
        {
            gVbatScale = k;
            saveVbatConfig();
            server.send(200, "application/json", "{\"ok\":true}");
        }
        else
            server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_scale\"}");
        return;
    }
    if (action == "calibrate")
    {
        float vref = server.hasArg("v") ? server.arg("v").toFloat() : 0;
        float v_raw = readVBATRawNoScale();
        if (vref > 2.5f && vref < 4.5f && v_raw > 2.5f)
        {
            gVbatScale = vref / v_raw;
            saveVbatConfig();
            server.send(200, "application/json", String("{\\\"ok\\\":true,\\\"scale\\\":" + String(gVbatScale, 4) + "}"));
        }
        else
            server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_cal\"}");
        return;
    }
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_action\"}");
}

void handleApiTune()
{
    if (!otaAuth())
        return;
    String action = server.arg("action");
    if (action == "get")
    {
        String s = "{";
        s += "\"on_dvdt\":" + String(gCfgChgOnDvdt, 3) + ",";
        s += "\"off_dvdt\":" + String(gCfgChgOffDvdt, 3) + ",";
        s += "\"on_hold\":" + String(gCfgChgOnHoldMs) + ",";
        s += "\"off_hold\":" + String(gCfgChgOffHoldMs) + ",";
        s += "\"full_hold\":" + String(gCfgFullHoldMs) + ",";
        s += "\"full_min_chg\":" + String(gCfgFullMinChgMs) + ",";
        s += "\"freeze_ms\":" + String(gCfgChgFreezeMs) + ",";
        s += "\"chg_rate\":" + String(gCfgChgPctRateMs) + ",";
        s += "\"d_rate\":" + String(gCfgDischPctRateMs) + ",";
        s += "\"on_mindv\":" + String(gCfgChgOnMinDv, 3) + ",";
        s += "\"zero_dvdt\":" + String(gCfgChgOffZeroDvdt, 3) + ",";
        s += "\"zero_hold\":" + String(gCfgChgOffZeroHoldMs);
        s += "}";
        server.send(200, "application/json", s);
        return;
    }
    if (action == "set")
    {
        Preferences tp;
        tp.begin("tune");
        if (server.hasArg("on_dvdt"))
        {
            gCfgChgOnDvdt = server.arg("on_dvdt").toFloat();
            tp.putFloat("on_dvdt", gCfgChgOnDvdt);
        }
        if (server.hasArg("off_dvdt"))
        {
            gCfgChgOffDvdt = server.arg("off_dvdt").toFloat();
            tp.putFloat("off_dvdt", gCfgChgOffDvdt);
        }
        if (server.hasArg("on_hold"))
        {
            String s = server.arg("on_hold");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgChgOnHoldMs = v;
                    tp.putUInt("on_hold", gCfgChgOnHoldMs);
                }
            }
        }
        if (server.hasArg("off_hold"))
        {
            String s = server.arg("off_hold");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgChgOffHoldMs = v;
                    tp.putUInt("off_hold", gCfgChgOffHoldMs);
                }
            }
        }
        if (server.hasArg("full_hold"))
        {
            String s = server.arg("full_hold");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgFullHoldMs = v;
                    tp.putUInt("full_hold", gCfgFullHoldMs);
                }
            }
        }
        if (server.hasArg("full_min_chg"))
        {
            String s = server.arg("full_min_chg");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgFullMinChgMs = v;
                    tp.putUInt("full_min_chg", gCfgFullMinChgMs);
                }
            }
        }
        if (server.hasArg("freeze_ms"))
        {
            String s = server.arg("freeze_ms");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgChgFreezeMs = v;
                    tp.putUInt("freeze_ms", gCfgChgFreezeMs);
                }
            }
        }
        if (server.hasArg("chg_rate"))
        {
            String s = server.arg("chg_rate");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgChgPctRateMs = v;
                    tp.putUInt("chg_rate", gCfgChgPctRateMs);
                }
            }
        }
        if (server.hasArg("d_rate"))
        {
            String s = server.arg("d_rate");
            s.trim();
            if (s.length())
            {
                uint32_t v = (uint32_t)strtoul(s.c_str(), nullptr, 10);
                if (v > 0)
                {
                    gCfgDischPctRateMs = v;
                    tp.putUInt("d_rate", gCfgDischPctRateMs);
                }
            }
        }
        if (server.hasArg("on_mindv"))
        {
            gCfgChgOnMinDv = server.arg("on_mindv").toFloat();
            tp.putFloat("on_mindv", gCfgChgOnMinDv);
        }
        if (server.hasArg("zero_dvdt"))
        {
            gCfgChgOffZeroDvdt = server.arg("zero_dvdt").toFloat();
            tp.putFloat("zero_dvdt", gCfgChgOffZeroDvdt);
        }
        if (server.hasArg("zero_hold"))
        {
            gCfgChgOffZeroHoldMs = server.arg("zero_hold").toInt();
            tp.putUInt("zero_hold", gCfgChgOffZeroHoldMs);
        }
        tp.end();
        server.send(200, "application/json", "{\"ok\":true}");
        return;
    }
    if (action == "reset")
    {
        // Restore defaults and persist to NVS
        gCfgChgOnDvdt = DEF_CHG_ON_DVDT_MV_S;
        gCfgChgOffDvdt = DEF_CHG_OFF_DVDT_MV_S;
        gCfgChgOnHoldMs = DEF_CHG_ON_HOLD_MS;
        gCfgChgOffHoldMs = DEF_CHG_OFF_HOLD_MS;
        gCfgFullDvdtMax = DEF_FULL_DVDT_MAX;
        gCfgFullHoldMs = DEF_FULL_HOLD_MS;
        gCfgFullMinChgMs = DEF_FULL_MIN_CHG_MS;
        gCfgChgFreezeMs = DEF_CHG_FREEZE_MS;
        gCfgChgPctRateMs = DEF_CHG_PCT_RATE_MS;
        gCfgDischPctRateMs = DEF_DISCH_PCT_RATE_MS;
        gCfgChgOnMinDv = DEF_CHG_ON_MIN_DV;
        gCfgChgOffZeroDvdt = DEF_CHG_OFF_ZERO_DVDT;
        gCfgChgOffZeroHoldMs = DEF_CHG_OFF_ZERO_HOLD_MS;

        Preferences tp;
        tp.begin("tune");
        tp.putFloat("on_dvdt", gCfgChgOnDvdt);
        tp.putFloat("off_dvdt", gCfgChgOffDvdt);
        tp.putUInt("on_hold", gCfgChgOnHoldMs);
        tp.putUInt("off_hold", gCfgChgOffHoldMs);
        tp.putFloat("full_dvdt", gCfgFullDvdtMax);
        tp.putUInt("full_hold", gCfgFullHoldMs);
        tp.putUInt("full_min_chg", gCfgFullMinChgMs);
        tp.putUInt("freeze_ms", gCfgChgFreezeMs);
        tp.putUInt("chg_rate", gCfgChgPctRateMs);
        tp.putUInt("d_rate", gCfgDischPctRateMs);
        tp.putFloat("on_mindv", gCfgChgOnMinDv);
        tp.putFloat("zero_dvdt", gCfgChgOffZeroDvdt);
        tp.putUInt("zero_hold", gCfgChgOffZeroHoldMs);
        tp.end();
        server.send(200, "application/json", "{\"ok\":true,\"reset\":true}");
        return;
    }
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_action\"}");
}

// ================= Wi-Fi / Portal ================

void handleRoot()
{
    if (WiFi.getMode() == WIFI_AP)
    {
        String html = F(
            "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>FlukeBridge WiFi Setup</title></head><body style='font-family:sans-serif;padding:16px'>"
            "<h2>Wi-Fi Setup</h2><form method='POST' action='/save'>"
            "SSID:<br><input name='s' style='width:100%'><br><br>"
            "Password:<br><input name='p' type='password' style='width:100%'><br><br>"
            "<button type='submit'>Save & Reboot</button></form>"
            "<p>After reboot the device will try to connect to the given network.</p>"
            "</body></html>");
        server.send(200, "text/html", html);
    }
    else
    {
        String html = String(F(
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>FlukeBridge</title></head>"
            "<body style='font-family:sans-serif;padding:16px;background:#0b1220;color:#e8eef7'>"
            "<h2>FlukeBridge</h2>"));
        html += "Connected to <b>" + WiFi.SSID() + "</b> (RSSI " + String(WiFi.RSSI()) + " dBm)<br>";
        html += "<p><a href='/status.html'>Dashboard</a> • <a href='/status.json'>status.json</a> • <a href='/update'>Firmware Update</a></p>";
        html += "<p>Hold BOOT &gt; 3 s to clear Wi-Fi and reboot into AP mode.</p>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    }
}

void handleSave()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }
    String ssid = server.arg("s");
    String pass = server.arg("p");
    prefs.begin("wifi");
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    server.send(200, "text/html", "<h3>Saved. Rebooting...</h3>");
    delay(600);
    ESP.restart();
}

void startConfigAP()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    String apName = "FlukeBridge-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.softAP(apName.c_str()); // OPEN (no password), like ver01 auto-opens portal
    Serial.print("[AP] SSID: ");
    Serial.println(apName);
    Serial.print("[AP] IP: ");
    Serial.println(WiFi.softAPIP());

    // Captive DNS: resolve all to 192.168.4.1
    dnsServer.start(DNS_PORT, "*", apIP);

    // Web routes for config
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);

    // Catch-all to root (captive)
    server.onNotFound([]()
                      { handleRoot(); });

    server.begin();
}

void connectOrStartAP()
{
    prefs.begin("wifi");
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("pass", "");
    prefs.end();

    if (ssid == "")
    {
        Serial.println("[WiFi] No stored creds -> AP mode");
        startConfigAP();
        return;
    }

    WiFi.mode(WIFI_STA);
    Serial.printf("[WiFi] Connecting to %s\n", ssid.c_str());
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000)
    {
        delay(200);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        WiFi.setHostname("fluke-bridge"); // hostname on LAN
        if (MDNS.begin("fluke-bridge"))
        {                                       // answers to fluke-bridge.local
            MDNS.addService("http", "tcp", 80); // useful for browsers/scanners
            Serial.println("[mDNS] fluke-bridge.local ready");
        }
        else
        {
            Serial.println("[mDNS] start failed");
        }

        Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        server.on("/", HTTP_GET, handleRoot);
        // status endpoints also in STA mode
        // (defined below with server.on)
    }
    else
    {
        Serial.println("\n[WiFi] Failed -> AP mode");
        startConfigAP();
    }
}

// ================= Status endpoints ===============

void handleStatusJson()
{
    updateBatteryFilter();
    float vb = gVbatFilt;
    String json = "{";

    json += "\"wifi\":{";
    json += "\"ssid\":\"" + WiFi.SSID() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += "},";

    // Battery state string for easier client use (trend-based)
    String bstate = "discharge";
    if (battNoBatt)
        bstate = "no_batt";
    else if (gTrendFull)
        bstate = "full";
    else if (gTrendCharging)
        bstate = "charging";

    json += "\"battery\":{";
    json += "\"voltage\":" + String(vb, 2) + ",";
    json += "\"percent\":" + String((gPercentDisp >= 0) ? gPercentDisp : vbatPercent(vb)) + ",";
    json += "\"v_raw\":" + String(gVbatRaw, 2) + ",";
    json += "\"state\":\"" + bstate + "\",";
    json += "\"usb_present\":";
    json += (usbPresent ? "true" : "false");
    json += ",";
    json += "\"warn\":";
    json += (battWarn ? "true" : "false");
    json += ",";
    json += "\"crit\":";
    json += (battCrit ? "true" : "false");
    json += ",";
    json += "\"charging\":";
    json += (battCharging ? "true" : "false");
    json += ",";
    json += "\"full\":";
    json += (battFull ? "true" : "false");
    json += ",";
    json += "\"no_batt\":";
    json += (battNoBatt ? "true" : "false");
    json += "},";

    json += "\"fluke\":{";
    json += "\"pretty\":\"" + safe(prettyFlukeValue(gFluke.value, gFluke.unit, gFluke.status, gFluke.flags)) + "\",";
    json += "\"value\":\"" + safe(gFluke.value) + "\",";
    json += "\"unit\":\"" + safe(gFluke.unit) + "\",";
    json += "\"status\":\"" + safe(gFluke.status) + "\",";
    json += "\"flags\":\"" + safe(gFluke.flags) + "\",";
    json += "\"ol\":";
    json += (gFluke.ol ? "true" : "false");
    json += ",";
    json += "\"raw\":\"" + safe(gFluke.raw) + "\"";
    json += "},";

    // Optional: time to deep sleep when in CRIT (seconds), -1 otherwise
    int sleep_in = -1;
    if (battCrit && critStartMs != 0)
    {
        uint32_t now = millis();
        if (now - critStartMs < CRIT_SLEEP_DELAY_MS)
        {
            sleep_in = (int)((CRIT_SLEEP_DELAY_MS - (now - critStartMs)) / 1000);
        }
        else
        {
            sleep_in = 0;
        }
    }

    json += "\"uptime\":" + String(millis() / 1000) + ",";
    json += "\"sleep_in\":" + String(sleep_in);
    json += "}";
    server.send(200, "application/json", json);
}

void handleStatusHtml()
{
    // Dark UI similar to your ver1
    static const char PROGMEM html[] = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlukeBridge - Status</title>
<style>
:root{--bg:#0b1220;--fg:#e8eef7;--card:#121a2a;--border:#1b2540;--ok:#96f59b;--warn:#ffd479;--crit:#ff7b7b;--accent:#3aa7ff}
*{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,sans-serif}
.wrap{max-width:920px;margin:22px auto;padding:0 16px}
h1{font-size:20px;margin:0 0 14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:14px;box-shadow:0 8px 20px rgba(0,0,0,.25)}
.k{opacity:.7;font-size:12px;margin-bottom:6px}
.v{font-size:18px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ok{color:var(--ok)} .warn{color:var(--warn)} .crit{color:var(--crit)}
.bar{height:8px;background:#1c2745;border-radius:999px;overflow:hidden;margin-top:8px}
.fill{height:100%;background:var(--accent);transition:width .3s}
.baticon{width:38px;height:18px;border:2px solid var(--accent);border-radius:3px;position:relative;display:inline-block;margin-right:8px}
.baticon::after{content:'';position:absolute;top:4px;right:-6px;width:4px;height:8px;background:var(--accent);border-radius:1px}
.batfill{height:100%;background:var(--accent);width:50%}
a{color:#9bd0ff;text-decoration:none} footer{text-align:center;opacity:.6;font-size:12px;margin:16px 0}
.small{opacity:.8;font-size:12px}
.badge{display:inline-block;border:1px solid var(--border);border-radius:999px;padding:2px 8px;margin-left:6px}
</style></head><body><div class="wrap">
<h1>Fluke Wi-Fi Bridge <span id="ssid" class="badge"></span></h1>
<div class="grid">
  <div class="card"><div class="k">Fluke</div>
    <div class="v" id="fluke_val">—</div>
    <div class="v small" id="fluke_meta">—</div>
  </div>
  <div class="card"><div class="k">Wi-Fi</div>
     <div class="v" id="ip_info">—</div>
    <div class="v" id="wifi_info">—</div>
    <div class="bar"><div class="fill" id="wifi_bar"></div></div>
  </div>
  <div class="card"><div class="k">Battery</div>
    <div class="v"><span class="baticon"><div class="batfill" id="batfill"></div></span><span id="bat_v">—</span></div>
    <div class="bar"><div class="fill" id="bat_bar"></div></div>
    <div class="v small" id="bat_flags">—</div>
  </div>
  <div class="card"><div class="k">System</div>
    <div class="v" id="uptime">—</div>
  </div>
</div>
<footer>Hold BOOT &gt; 3 s to clear Wi-Fi and reboot (AP: FlukeBridge-XXXX) • <a href="/status.json">status.json</a> • <a href="/config">Config</a> • <a href="/update">OTA</a></footer>
</div>
<script>
const auth='Basic '+btoa('admin:fluke1234');
async function tick(){
 try{
  const r=await fetch('/status.json',{cache:'no-store'});const j=await r.json();
  // Wi-Fi
  document.getElementById('ssid').textContent = j.wifi.ssid||'AP mode';
  const rssi=j.wifi.rssi||-100; const pct=Math.max(0,Math.min(100,2*(rssi+100)));
  document.getElementById('ip_info').textContent =`${j.wifi.ip || j.ip || '—'}`;
  document.getElementById('wifi_info').textContent =`${j.wifi.ssid || 'AP'} • RSSI ${rssi} dBm`;
  document.getElementById('wifi_bar').style.width=pct+'%';

  // Fluke
  const pretty = j.fluke.pretty || ((j.fluke.value||'—') + ' ' + (j.fluke.unit||''));
  document.getElementById('fluke_val').textContent = pretty;
  const meta = (j.fluke.ol?'⚠️ OL • ':'') + (j.fluke.status||'') + (j.fluke.flags&&j.fluke.flags!=='NONE'?' • '+j.fluke.flags:'');
  const fm = document.getElementById('fluke_meta'); fm.textContent = meta||'NORMAL';
  fm.className='v small ' + (j.fluke.ol?'crit':(j.fluke.status==='NORMAL'?'ok':'warn'));

  // Battery
  const vb=j.battery.voltage||0; const p=j.battery.percent||0;
  document.getElementById('bat_v').textContent = vb.toFixed(2)+' V ('+p+'%)';
  document.getElementById('bat_bar').style.width = Math.max(0,Math.min(100,p))+'%';
  const fill=document.getElementById('batfill');
  fill.style.width=p+'%'; fill.style.background = (p>60?'#96f59b':(p>30?'#ffd479':'#ff7b7b'));
  let flags=[]; if(j.battery.no_batt)flags.push('NO_BATT'); if(j.battery.usb_present)flags.push('USB');
  // primary state: DISCHARGE / CHARGING / FULL / NO_BATT
  const bstate=(j.battery.state||'').toUpperCase();
  if(bstate) flags.unshift(bstate);
  if(j.battery.crit)flags.push('CRIT'); else if(j.battery.warn)flags.push('LOW');
  document.getElementById('bat_flags').textContent = flags.join(' • ');

  // Uptime (with days)
  function fmtDHMS(t){
    t = Math.max(0, Math.floor(t||0));
    const d = Math.floor(t/86400);
    const h = Math.floor((t%86400)/3600);
    const m = Math.floor((t%3600)/60);
    const s = t%60;
    const pad = n => String(n).padStart(2,'0');
    return (d>0 ? (d+'d ') : '') + pad(h)+':'+pad(m)+':'+pad(s);
  }
  document.getElementById('uptime').textContent = 'Uptime: '+fmtDHMS(j.uptime||0);
 }catch(e){}
}
setInterval(tick,1000); tick();
</script></body></html>
)HTML";
    server.send_P(200, "text/html; charset=utf-8", html);
}

// ================= OTA (HTTP upload) ================

bool otaAuth()
{
    if (!server.authenticate(OTA_USER, OTA_PASS))
    {
        server.requestAuthentication();
        return false;
    }
    return true;
}

void registerOtaRoutes()
{
    // Upload form
    server.on("/update", HTTP_GET, []()
              {
        if (!otaAuth()) return;
        static const char PROGMEM page[] = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlukeBridge - OTA Update</title></head><body style="font-family:sans-serif;padding:16px">
<h2>OTA Update (ver3.3.5)</h2>
<form method="POST" action="/update" enctype="multipart/form-data">
  <input type="file" name="update" accept=".bin,application/octet-stream"><br><br>
  <button type="submit">Upload & Flash</button>
  <p>Login: admin / fluke1234</p>
</form>
</body></html>
)HTML";
        server.send_P(200, "text/html; charset=utf-8", page); });

    // Upload handler
    server.on("/update", HTTP_POST, []()
              {
                  if (!otaAuth()) return;
                  bool ok = !Update.hasError();
                  server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
                  delay(300);
                  if (ok) ESP.restart(); }, []()
              {
                  if (!otaAuth()) return;
                  HTTPUpload &up = server.upload();
                  if (up.status == UPLOAD_FILE_START)
                  {
                      // Begin update with unknown size (ESP32 handles partitions)
                      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                      {
                          Update.printError(Serial);
                      }
                  }
                  else if (up.status == UPLOAD_FILE_WRITE)
                  {
                      if (Update.write(up.buf, up.currentSize) != up.currentSize)
                      {
                          Update.printError(Serial);
                      }
                  }
                  else if (up.status == UPLOAD_FILE_END)
                  {
                      if (!Update.end(true))
                      {
                          Update.printError(Serial);
                      }
                  }
                  else if (up.status == UPLOAD_FILE_ABORTED)
                  {
                      Update.end();
                  } });
}

// ================== Setup / Loop =================

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[FlukeBridge ver3] boot");

    // IO
    pinMode(BOOT_KEY_PIN, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_ADC_VBAT, ADC_11db);

    // Load VBAT config (divider + scale)
    loadVbatConfig();

    // UART to Fluke
    U.begin(FLUKE_BAUD, SERIAL_8N1, PIN_RX, PIN_TX, false);
    Serial.println("[UART] 115200 8N1, RX=20, TX=21");

    // Load SoC calibration (USB-set)
    loadSocCal();

    // Load tuning thresholds (charging/full detection & UI behavior)
    {
        Preferences tp;
        tp.begin("tune");
        gCfgChgOnDvdt = tp.getFloat("on_dvdt", DEF_CHG_ON_DVDT_MV_S);
        gCfgChgOffDvdt = tp.getFloat("off_dvdt", DEF_CHG_OFF_DVDT_MV_S);
        gCfgChgOnHoldMs = tp.getUInt("on_hold", DEF_CHG_ON_HOLD_MS);
        gCfgChgOffHoldMs = tp.getUInt("off_hold", DEF_CHG_OFF_HOLD_MS);
        gCfgFullDvdtMax = tp.getFloat("full_dvdt", DEF_FULL_DVDT_MAX);
        gCfgFullHoldMs = tp.getUInt("full_hold", DEF_FULL_HOLD_MS);
        gCfgFullMinChgMs = tp.getUInt("full_min_chg", DEF_FULL_MIN_CHG_MS);
        gCfgChgFreezeMs = tp.getUInt("freeze_ms", DEF_CHG_FREEZE_MS);
        gCfgChgPctRateMs = tp.getUInt("chg_rate", DEF_CHG_PCT_RATE_MS);
        gCfgDischPctRateMs = tp.getUInt("d_rate", DEF_DISCH_PCT_RATE_MS);
        gCfgChgOnMinDv = tp.getFloat("on_mindv", DEF_CHG_ON_MIN_DV);
        gCfgChgOffZeroDvdt = tp.getFloat("zero_dvdt", DEF_CHG_OFF_ZERO_DVDT);
        gCfgChgOffZeroHoldMs = tp.getUInt("zero_hold", DEF_CHG_OFF_ZERO_HOLD_MS);
        tp.end();
    }

    // Wi-Fi up
    connectOrStartAP();

    // Common endpoints (available both in AP and STA)
    server.on("/status.json", HTTP_GET, handleStatusJson);
    server.on("/status.html", HTTP_GET, handleStatusHtml);
    registerOtaRoutes(); // /update (Basic Auth)
    // Config routes (Basic Auth)
    server.on("/config", HTTP_GET, handleConfigPage);
    server.on("/api/soc", HTTP_GET, handleApiSoc);
    server.on("/api/vbat", HTTP_GET, handleApiVbat);
    server.on("/api/tune", HTTP_GET, handleApiTune);
    // Root handler was bound in connectOrStartAP/startConfigAP
    server.begin();
    Serial.println("[HTTP] Web server started");

    // Small hello to Fluke
    String id;
    if (sendCmd("ID\r", &id, 400, 900))
    {
        Serial.println("[Fluke] ID OK: " + id);
    }
    else
    {
        Serial.println("[Fluke] ID failed (but UART is up)");
    }
}

void loop()
{
    // Web + captive DNS (AP mode)
    server.handleClient();
    if (WiFi.getMode() == WIFI_AP)
        dnsServer.processNextRequest();

    // BOOT long press
    bootLongPressTask();

    // USB serial commands for OBS/py client (QM/QS etc.)
    handleUsbCommands();

    // Fluke periodic QM poll
    if (millis() - tPoll > 400)
    {
        tPoll = millis();
        sendCmd("QM\r", nullptr, 250, 900);
    }
    // Read any async lines from Fluke
    while (U.available())
    {
        String line = readLineWithTimeout(10);
        if (line.length())
        {
            parseFlukeLine(line);
            // Do NOT print raw lines to Serial to avoid interfering with USB protocol
            // Serial.println("[RX] " + line);
        }
    }

    // Battery LED behavior
    batteryLedTask();

    // Console single-char passthrough removed in favor of line-based USB protocol
}
