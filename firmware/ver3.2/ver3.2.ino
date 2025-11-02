/*
  Full-ver3.ino  (ESP32-C3 XIAO + Fluke 287 IR Bridge)
  ----------------------------------------------------
  Goals:
   - Keep the proven Fluke serial logic from ver01 (115200 8N1, RX=20 TX=21, no invert)
   - Add BOOT long-press (GPIO9) to clear Wi-Fi creds (Preferences) and reboot
   - Wi-Fi station with fallback to AP (open, no password): "FlukeBridge-XXXX"
     + Captive portal (DNS catch-all) -> opens Wi-Fi config page automatically
   - Sync WebServer (no Async) with:
       * /status.json  (value/unit/status/flags/ol + wifi rssi + battery + uptime)
       * /status.html  (dark dashboard like your ver1, auto-refresh 1s)
       * /      (Wi-Fi config form in AP mode: SSID/password; saves to NVS and reboots)
   - Battery monitor: divider 620k / 470k -> GPIO2 (A2)
     * LED on GPIO3 with states:
       - < 1.0 V             -> OFF (no-battery / PCM cut)
       - USB present (>4.6V) -> OFF
       - > 3.45 V            -> OFF (normal)
       - 3.30–3.45 V         -> ON (solid)
       - < 3.30 V            -> BLINK (critically low)
       - Charging (3.7–4.2V rising) -> ON (treated as low-level "charging" lamp)
       - Full (>4.15V)       -> OFF
    (Note: USB presence is heuristically detected by VBAT > 4.60 V on the divider path.
           If your Xiao powers VBAT rail differently when USB is connected, adjust threshold.)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <HardwareSerial.h>
#include <ESPmDNS.h>
#include <Update.h>

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
static const char *FW_VER = "ver3.2";
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
static const float R1_VBAT = 620000.0f;
static const float R2_VBAT = 470000.0f;
static const float ADC_VREF = 3.3f;
static const float USB_PRESENT_V = 4.60f; // heuristic USB present
static const float VBAT_FULL = 4.15f;
static const float VBAT_WARN = 3.45f;
static const float VBAT_CRIT = 3.30f;
static const float VBAT_NOBAT = 1.00f;

bool battNoBatt = false, battCrit = false, battWarn = false, battCharging = false, battFull = false, usbPresent = false;
uint32_t ledTmr = 0;
bool ledOn = false;

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

float readVBAT()
{
    int raw = analogRead(PIN_ADC_VBAT);
    float v_adc = (raw / 4095.0f) * ADC_VREF;
    return v_adc * (R1_VBAT + R2_VBAT) / R2_VBAT;
}

int vbatPercent(float v)
{
    if (v <= 3.30f)
        return 5;
    if (v >= 4.18f)
        return 100;
    return (int)roundf((v - 3.30f) * (95.0f / (4.18f - 3.30f)));
}

// LED policy per user table
void batteryLedTask()
{
    float vb = readVBAT();
    usbPresent = (vb > USB_PRESENT_V); // USB heuristic
    battNoBatt = (vb < VBAT_NOBAT);
    battFull = (!battNoBatt && vb > VBAT_FULL);
    battCrit = (!battNoBatt && !usbPresent && vb < VBAT_CRIT);
    battWarn = (!battNoBatt && !usbPresent && !battCrit && vb >= VBAT_CRIT && vb < VBAT_WARN);
    // "charging" hint: if between ~3.7 and 4.2, but not full yet and not USB-off policy
    battCharging = (!battNoBatt && !battFull && vb >= 3.70f && vb <= 4.20f);

    // Apply LED rules
    uint32_t now = millis();
    // OFF cases:
    if (battNoBatt || usbPresent || (vb > VBAT_WARN) || battFull)
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
    float vb = readVBAT();
    String json = "{";

    json += "\"wifi\":{";
    json += "\"ssid\":\"" + WiFi.SSID() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += "},";

    json += "\"battery\":{";
    json += "\"voltage\":" + String(vb, 2) + ",";
    json += "\"percent\":" + String(vbatPercent(vb)) + ",";
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

    json += "\"uptime\":" + String(millis() / 1000);
    json += "}";
    server.send(200, "application/json", json);
}

void handleStatusHtml()
{
    // Dark UI similar to your ver1
    static const char PROGMEM html[] = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlukeBridge – Status</title>
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
<footer>Hold BOOT &gt; 3 s to clear Wi-Fi and reboot (AP: FlukeBridge-XXXX) • <a href="/update">Firmware Update</a></footer>
</div>
<script>
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
  let flags=[]; if(j.battery.no_batt)flags.push('no-batt'); if(j.battery.usb_present)flags.push('USB');
  if(j.battery.full)flags.push('full'); else if(j.battery.charging)flags.push('charging');
  if(j.battery.crit)flags.push('CRIT'); else if(j.battery.warn)flags.push('low');
  document.getElementById('bat_flags').textContent = flags.join(' • ')||'OK';

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
<title>FlukeBridge – OTA Update</title></head><body style="font-family:sans-serif;padding:16px">
<h2>OTA Update (ver3.2)</h2>
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

    // UART to Fluke
    U.begin(FLUKE_BAUD, SERIAL_8N1, PIN_RX, PIN_TX, false);
    Serial.println("[UART] 115200 8N1, RX=20, TX=21");

    // Wi-Fi up
    connectOrStartAP();

    // Common endpoints (available both in AP and STA)
    server.on("/status.json", HTTP_GET, handleStatusJson);
    server.on("/status.html", HTTP_GET, handleStatusHtml);
    registerOtaRoutes(); // /update (Basic Auth)
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
