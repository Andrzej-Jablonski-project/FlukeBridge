#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time, math, json, socket, argparse, re
from datetime import datetime

# ── Optional: pyserial (USB mode only) ────────────────────────────────────────
try:
    import serial  # noqa: F401  # import checked in SerialTransport class
except Exception:
    pass

# ── Paths ─────────────────────────────────────────────────────────────────────
# Default: write into the folder where this script resides (portable).
# You can override the output directory using the FLUKE_DIR environment
# variable or the command-line flag: --dir <path> (see entrypoint below).
def _default_base_dir():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except Exception:
        # fallback: current working directory
        return os.getcwd()

def set_base_dir(path: str):
    global BASE_DIR, VALUE_TXT, STATUS_TXT, LOG_CSV, DEBUG_LOG
    BASE_DIR = os.path.abspath(path)
    VALUE_TXT  = os.path.join(BASE_DIR, "fluke_value.txt")
    STATUS_TXT = os.path.join(BASE_DIR, "fluke_status.txt")
    LOG_CSV    = os.path.join(BASE_DIR, "fluke_log.csv")
    DEBUG_LOG  = os.path.join(BASE_DIR, "fluke_debug.log")

# Initialize base directory (ENV → defaults to script directory)
set_base_dir(os.environ.get("FLUKE_DIR", _default_base_dir()))

# ── Port ──────────────────────────────────────────────────────────────────────
SER_PORT = "/dev/ttyACM0"
BAUD     = 115200
TIMEOUT  = 0.1
WRITE_TIMEOUT = 0.1

# ── Performance tuning (CLI/env) ─────────────────────────────────────────────
# Keep defaults compatible with previous behavior. You can override via
# flags or environment variables.
REWRITE_INTERVAL_S = float(os.environ.get("FLUKE_WRITE_INTERVAL_S", "0.2"))
QS_GAP_MS          = int(float(os.environ.get("FLUKE_QS_GAP_MS", "80")))
CSV_ON_CHANGE      = os.environ.get("FLUKE_CSV_ON_CHANGE", "0") == "1"
USE_FSYNC          = not (os.environ.get("FLUKE_NO_FSYNC", "0") == "1")

# ── Thresholds / timings ──────────────────────────────────────────────────────
OL_OHM_THRESHOLD    = 1e8
OL_DIODE_THRESHOLD  = 2.5
F_OL_EPS_FARADS     = 1e-12  # treat <= 1 pF as OL when in HOLD/open
TEMP_C_MIN          = -273.15
TEMP_C_MAX          = 1000.0
TEMP_F_MIN          = -459.67
TEMP_F_MAX          = 1832.0
HOLD_TIME_S         = 0.7
OFF_TIMEOUT_S       = 2.0

# ── Normalization / recognition ───────────────────────────────────────────────
VALID_UNITS = {
    "VDC","VAC","ADC","AAC","OHM","Ω","HZ","F","H","DIODE",
    "TEMP_C","TEMP_F",
    "VAC+DC","VDC+AC","AAC+DC","ADC+AC"
}

OHM_LIKE    = {"OHM","Ω","F","DIODE","TEMP_C","TEMP_F"}
NUM_RE = re.compile(r"^[\+\-]?\d+(?:\.\d+)?(?:[eE][\+\-]?\d+)?$")
ACDC_RE = re.compile(r"\b(AC\s*[,/ ]\s*DC|DC\s*[,/ ]\s*AC)\b", re.IGNORECASE)

# ── Helper functions ──────────────────────────────────────────────────────────
def normalize_unit(unit_raw: str) -> str:
    if not unit_raw:
        return ""
    u = unit_raw.upper()
    if "DIODE" in u:
        return "DIODE"
    # temperature
    if u in ("C","°C","CEL","CELSIUS","DEG C","DEGC","DEG_C"):
        return "TEMP_C"
    if u in ("FAR","FAH","FAHR","FAHRENHEIT","DEG F","DEGF","DEG_F","°F"):
        return "TEMP_F"

    u_std = u.replace(" ", "").replace("_", "")
    u_std = u_std.replace("PLUS", "+")
    if u_std in ("VAC+DC","VACDC"):   return "VAC+DC"
    if u_std in ("VDC+AC","VDCAC"):   return "VDC+AC"
    if u_std in ("AAC+DC","AACDC"):   return "AAC+DC"
    if u_std in ("ADC+AC","ADCAC"):   return "ADC+AC"

    u = re.sub(r"[^A-ZΩ]", "", u).lstrip("G")
    mapping = {
        "VDC":"VDC", "VAC":"VAC",
        "ADC":"ADC", "AAC":"AAC",
        "OHM":"OHM", "OHMS":"OHM", "OH":"OHM",
        "HZ":"HZ", "F":"F", "H":"H",
        "Ω":"Ω"
    }
    if u in mapping:
        return mapping[u]
    if "Ω" in u:
        return "Ω"
    return u

def si_format(val, base_unit):
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return f"— {base_unit}"
    a = abs(val)
    scales = [(1e-9,"n"),(1e-6,"µ"),(1e-3,"m"),(1,""),(1e3,"k"),(1e6,"M"),(1e9,"G")]
    factor, prefix = 1, ""
    for s, p in scales:
        factor, prefix = s, p
        if a < s*1000 or (s, p) == scales[-1]:
            break
    scaled = val / factor
    if abs(scaled) >= 100:  txt = f"{scaled:,.0f}".replace(",", " ")
    elif abs(scaled) >= 10: txt = f"{scaled:,.2f}"
    else:                   txt = f"{scaled:,.3f}"
    return f"{txt} {prefix}{base_unit}"

def map_unit(unit_code):
    u = (unit_code or "").upper()
    if u == "VDC": return ("V","DC")
    if u == "VAC": return ("V","AC")
    if u == "ADC": return ("A","DC")
    if u == "AAC": return ("A","AC")
    if u in ("OHM","Ω"): return ("Ω","")
    if u == "HZ":  return ("Hz","")
    if u == "F":   return ("F","")
    if u == "H":   return ("H","")
    if u == "DIODE": return ("V","DIODE")
    if u == "TEMP_C": return ("°C","")
    if u == "TEMP_F": return ("°F","")
    if u in ("VAC+DC","VDC+AC"): return ("V","AC+DC")
    if u in ("AAC+DC","ADC+AC"): return ("A","AC+DC")
    return (u, "")

def is_trash_frame(raw: str) -> bool:
    if not raw:
        return True
    s = raw.upper()
    # Filter out transient garbage during mode switches, e.g., GLIMBO/GNONE
    if "GLIMBO" in s or "GNONE" in s or "LIMBO" in s:
        return True
    # Only accept frames that look like QM/QS: exactly 3 commas (4 fields)
    if s.count(',') != 3:
        return True
    return False

_NUMERIC = re.compile(r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:E[+-]?\d+)?$')

def normalize_line(s: str) -> str:
    if not s:
        return ""
    s = s.strip()
    if not s or s == "0":
        return ""
    if "\n" in s or "\r" in s:
        parts = [p.strip() for p in s.replace("\r", "\n").split("\n")]
        parts = [p for p in parts if p and p != "0"]
        if not parts:
            return ""
        s = parts[-1]

    if s.count(",") != 3:
        return ""
    if s[0] == ",":
        return ""

    val, unit, mode, extra = s.split(",", 3)
    if not _NUMERIC.match(val):
        return ""
    return ",".join([val, unit.strip(), mode.strip(), extra.strip()])

def parse_response(raw_line):
    if not raw_line:
        return None, None, "", "", "", False
    last = raw_line.split("\r")[-1].strip()
    if is_trash_frame(last):
        return None, None, "", "", last, False
    fields = [f.strip() for f in last.split(",")]
    if len(fields) < 2:
        return None, None, "", "", last, False
    val_str, unit_raw = fields[0], fields[1]
    unit_code = normalize_unit(unit_raw)
    mode  = (fields[2] if len(fields) > 2 else "").upper()
    extra = (fields[3] if len(fields) > 3 else "").upper()

    txt = val_str.upper()
    if txt in {"OL","OPEN","OVER","INF","INFINITY"}:
        extra = (extra + " OL").strip()
        return None, unit_code, mode, extra, unit_raw, (unit_code in VALID_UNITS)
    if not NUM_RE.match(val_str):
        return None, unit_code if unit_code in VALID_UNITS else None, mode, extra, unit_raw, False
    try:
        val = float(val_str)
    except:
        return None, unit_code, mode, extra, unit_raw, False
    if not math.isfinite(val) or abs(val) > 1e12:
        extra = (extra + " OL").strip()
        return None, unit_code, mode, extra, unit_raw, (unit_code in VALID_UNITS)
    return val, unit_code, mode, extra, unit_raw, (unit_code in VALID_UNITS)

def detect_ol(val, unit_code, mode, extra):
    txt = " ".join(filter(None, [unit_code, mode, extra])).upper()
    if any(tag in txt for tag in (" OL", "OPEN", "OVER")) or txt == "OL":
        return True
    if unit_code in ("OHM","Ω"):   return (val is None) or (abs(val) >= OL_OHM_THRESHOLD)
    if unit_code == "DIODE":       return (val is None) or (val >= OL_DIODE_THRESHOLD)
    if unit_code == "F":           return (val is None) or (val <= 0) or (isinstance(val, (int,float)) and val <= F_OL_EPS_FARADS)
    if unit_code == "TEMP_C":
        if (val is None) or ("OPEN_TC" in (mode or "")) or ("OPEN_TC" in (extra or "")):
            return True
        if isinstance(val, (int, float)) and (val < TEMP_C_MIN or val > TEMP_C_MAX):
            return True
    if unit_code == "TEMP_F":
        if (val is None) or ("OPEN_TC" in (mode or "")) or ("OPEN_TC" in (extra or "")):
            return True
        if isinstance(val, (int, float)) and (val < TEMP_F_MIN or val > TEMP_F_MAX):
            return True
    return False

def format_for_obs(val, unit_code, mode, extra):
    if unit_code in ("OHM","Ω","DIODE","F") and (val is None or detect_ol(val, unit_code, mode, extra)):
        return "OL"
    if detect_ol(val, unit_code, mode, extra):
        return "OL"
    base_unit, suffix = map_unit(unit_code)
    if val is None:
        return "—"
    pretty = si_format(val, base_unit)
    if suffix in ("AC","DC","AC+DC"): return f"{pretty} {suffix}"
    if suffix == "DIODE":             return f"{pretty} (diode)"
    return pretty

def safe_write(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
        if USE_FSYNC:
            f.flush()
            os.fsync(f.fileno())

def ensure_paths():
    os.makedirs(BASE_DIR, exist_ok=True)
    if not os.path.exists(LOG_CSV):
        with open(LOG_CSV, "w", encoding="utf-8") as f:
            f.write("timestamp,raw_value,unit_raw,unit_code,mode,extra,pretty,delta_s,status\n")
    safe_write(VALUE_TXT, "—\n")
    safe_write(STATUS_TXT, "⭘ OFF\n")

def log_debug(raw, parsed):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    val, unit_code, mode, extra, unit_raw, ok = parsed
    with open(DEBUG_LOG, "a", encoding="utf-8") as f:
        f.write(f"{ts} | RAW='{raw}' | unit_raw='{unit_raw}' | unit_code='{unit_code}' | "
                f"mode='{mode}' | extra='{extra}' | val='{val}' | ok={ok}\n")

# ── Transports (USB / TCP) ───────────────────────────────────────────────────
class Transport:
    def write(self, data: bytes): ...
    def read_until(self, delim: bytes, timeout_s: float) -> bytes: ...
    def reset_input_buffer(self): ...
    def close(self): ...

class SerialTransport(Transport):
    def __init__(self, port, baud, timeout, write_timeout):
        import serial
        self.ser = serial.Serial(port=port, baudrate=baud,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=timeout, write_timeout=write_timeout)
    def write(self, data: bytes):
        self.ser.write(data)
    def read_until(self, delim: bytes, timeout_s: float) -> bytes:
        old = self.ser.timeout
        try:
            self.ser.timeout = timeout_s
            # Read line-by-line and skip debug logs coming from ESP
            deadline = time.time() + timeout_s
            buf = b""
            while time.time() < deadline:
                chunk = self.ser.read_until(delim)
                if not chunk:
                    continue
                line = chunk.decode("ascii", "ignore").strip()
                if line.startswith("[RX]") or line.startswith("[WiFi]") or line.startswith("[TCP]") or line.startswith("[HTTP]"):
                    continue
                return chunk
            return b""
        finally:
            self.ser.timeout = old
    def reset_input_buffer(self):
        self.ser.reset_input_buffer()
    def close(self):
        try: self.ser.close()
        except: pass

class TcpTransport(Transport):
    def __init__(self, host, port, timeout=2.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        self.sock.settimeout(0.05)
        self.buf = bytearray()
        self.line_buf = b""

    def write(self, data: bytes):
        self.sock.sendall(data)

    def read_until(self, delim: bytes, timeout_s: float) -> bytes:
        t0 = time.time()
        end = t0 + timeout_s
        while time.time() < end:
            idx = self.line_buf.find(delim)
            if idx != -1:
                line = bytes(self.line_buf[:idx])
                self.line_buf = self.line_buf[idx + len(delim):]
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("[RX]") or s.startswith("[WiFi]") or s.startswith("[TCP]"):
                    continue
                return line + delim
            try:
                chunk = self.sock.recv(512)
                if chunk:
                    self.line_buf += chunk
                else:
                    time.sleep(0.005)
            except socket.timeout:
                time.sleep(0.005)
        if self.line_buf:
            idx = self.line_buf.find(delim)
            if idx != -1:
                line = bytes(self.line_buf[:idx])
                self.line_buf = self.line_buf[idx + len(delim):]
                return line + delim
        return b""

    def reset_input_buffer(self):
        self.buf.clear()
        self.line_buf = b""
        self.sock.settimeout(0.0)
        try:
            while True:
                data = self.sock.recv(1024)
                if not data:
                    break
        except:
            pass
        self.sock.settimeout(0.05)

    def close(self):
        try:
            self.sock.close()
        except:
            pass

def open_transport(args):
    if args.tcp:
        host, port = args.tcp.split(":")
        return TcpTransport(host, int(port))
    else:
        return SerialTransport(args.serial, BAUD, TIMEOUT, WRITE_TIMEOUT)

# ── QM/QS queries (USB/TCP) ──────────────────────────────────────────────────
def query_qm(tr):
    if not isinstance(tr, TcpTransport):
        tr.reset_input_buffer()
    tr.write(b"QM\r")
    raw = tr.read_until(b"\r", 0.8).decode(errors="ignore").strip()
    raw = normalize_line(raw)
    if not raw:
        extra = tr.read_until(b"\r", 1.2).decode(errors="ignore").strip()
        raw = normalize_line(extra)
    return raw

def query_qs(tr):
    tr.write(b"QS\r")
    first = tr.read_until(b"\r", 0.8).decode(errors="ignore").strip()
    first = normalize_line(first)
    if not first:
        second = tr.read_until(b"\r", 1.0).decode(errors="ignore").strip()
        second = normalize_line(second)
        return second or ""
    return first

# ── HTTP polling /status.json ─────────────────────────────────────────────────
def http_fetch_status(url, timeout=1.5):
    from urllib.request import Request, urlopen
    req = Request(url, headers={"Cache-Control":"no-cache","Pragma":"no-cache"})
    with urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8","ignore"))

def http_loop(args):
    ensure_paths()
    last_pretty = None
    last_status = "OFF"
    last_write_ts = 0.0

    while True:
        try:
            j = http_fetch_status(args.http, timeout=1.5)
            # Prefer structure: { fluke:{ value,unit,mode,flags,status,ol,... }, battery:{...}, wifi:{...} }
            fl = j.get("fluke", {})
            # Build pretty text ourselves to ensure consistent formatting (e.g., AC+DC)
            v_raw = fl.get("value")
            try:
                val = float(v_raw) if v_raw not in (None, "", "OL") else None
            except Exception:
                val = None
            unit_code = normalize_unit(fl.get("unit") or "")
            mode = (fl.get("mode") or "")
            extra = (fl.get("flags") or "")
            pretty = format_for_obs(val, unit_code, mode, extra)

            # status: use JSON if present; otherwise heuristic
            status = (fl.get("status") or "").upper()
            if status not in {"LIVE","HOLD","OFF"}:
                if pretty in ("—", "", None):
                    status = "HOLD"
                elif fl.get("ol", False):
                    status = "HOLD"
                else:
                    status = "LIVE"

            now = time.time()
            should_write = (now - last_write_ts) >= REWRITE_INTERVAL_S or pretty != last_pretty or status != last_status
            if should_write:
                # Freeze value during HOLD to avoid transient garbage, but still show OL
                if status != "HOLD" or pretty == "OL":
                    safe_write(VALUE_TXT, pretty + "\n")
                safe_write(STATUS_TXT, status + "\n")
                last_pretty = pretty
                last_status = status
                last_write_ts = now
                print(pretty, "[", status, "]")

            # optional CSV from /status.json (fluke only)
            if CSV_ON_CHANGE and not should_write:
                pass
            else:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                with open(LOG_CSV, "a", encoding="utf-8") as f:
                    f.write(f"{ts},,,,{fl.get('status','')},{fl.get('flags','')},{pretty},,\n")

            time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nStopped.")
            sys.exit(0)
        except Exception as e:
            # transient network errors should not immediately force OFF
            print(f"[HTTP WARN] {e}")
            time.sleep(0.8)

# ── Main loop (USB/TCP) ──────────────────────────────────────────────────────
def serial_tcp_loop(args):
    ensure_paths()
    tr = None
    last_pretty = None
    last_change_ts = time.time()
    last_write_ts  = 0.0
    current_unit   = None
    current_unit_cmp = None
    hold_until     = 0.0
    last_status    = "OFF"
    last_ok_ts     = 0.0
    last_temp_hint_ts = 0.0

    while True:
        try:
            if tr is None:
                tr = open_transport(args)
                time.sleep(0.1)

            raw = query_qm(tr)
            val, unit_code, mode, extra, unit_raw, ok = parse_response(raw)
            log_debug(raw, (val, unit_code, mode, extra, unit_raw, ok))

            mx_all_upper = f"{unit_raw or ''} {mode or ''} {extra or ''}".upper()
            if (unit_code in {"TEMP_C","TEMP_F"}) or ("TEMP" in mx_all_upper) or ("DEG" in mx_all_upper) or ("°" in (unit_raw or "")):
                last_temp_hint_ts = time.time()

            mx = (mode or "") + (extra or "")
            tag = mx.replace(" ", "").upper()
            if tag in ("AC+DC","ACDC","DC+AC","DCAC"):
                uc = (unit_code or "").upper()
                if uc.startswith(("VD","VA")):
                    unit_code = "VAC+DC"
                elif uc.startswith(("AD","AA")):
                    unit_code = "AAC+DC"

            compare_unit = unit_code
            if unit_code in ("VAC+DC", "VDC+AC"): compare_unit = "V_ACDC"
            if unit_code in ("AAC+DC", "ADC+AC"): compare_unit = "A_ACDC"
            if unit_code == "F":
                if (time.time() - last_temp_hint_ts) < 2.0:
                    unit_code = "TEMP_F"

            mx_blob = f"{unit_raw or ''} {mode or ''} {extra or ''}"
            two_channel = bool(ACDC_RE.search(mx_blob))
            # If unit explicitly signals AC+DC, show combined value (e.g., "10 V AC+DC")
            # instead of AC|DC split view
            base_primary, suf_hint = map_unit(unit_code)
            if (suf_hint or "").upper() == "AC+DC":
                two_channel = False
            order_ac_first = True
            m_acdc = ACDC_RE.search(mx_blob)
            if m_acdc:
                txt = m_acdc.group(1).upper().replace(" ", "")
                order_ac_first = txt.startswith("AC")

            ac_pretty = None
            dc_pretty = None

            if two_channel:
                main_pretty = format_for_obs(val, unit_code, mode, extra)
                _, suf = map_unit(unit_code)
                suf = (suf or "").upper()
                if   suf == "AC": ac_pretty = main_pretty
                elif suf == "DC": dc_pretty = main_pretty
                else:
                    if order_ac_first: ac_pretty = main_pretty
                    else:              dc_pretty = main_pretty

                time.sleep(max(0.0, QS_GAP_MS / 1000.0))
                raw2 = query_qs(tr)
                v2, u2, m2, e2, ur2, ok2 = parse_response(raw2)
                log_debug(raw2, (v2, u2, m2, e2, ur2, ok2))

                if ok2 and (u2 in VALID_UNITS):
                    sec_pretty = format_for_obs(v2, u2, m2, e2)
                    _, suf2 = map_unit(u2)
                    suf2 = (suf2 or "").upper()
                    if   suf2 == "AC": ac_pretty = sec_pretty
                    elif suf2 == "DC": dc_pretty = sec_pretty
                else:
                    if v2 is not None:
                        base_primary, _ = map_unit(unit_code)
                        if order_ac_first:
                            if ac_pretty is None and base_primary in ("V","A"):
                                ac_pretty = f"{si_format(v2, base_primary)} AC"
                            elif dc_pretty is None and base_primary in ("V","A"):
                                dc_pretty = f"{si_format(v2, base_primary)} DC"
                        else:
                            if dc_pretty is None and base_primary in ("V","A"):
                                dc_pretty = f"{si_format(v2, base_primary)} DC"
                            elif ac_pretty is None and base_primary in ("V","A"):
                                ac_pretty = f"{si_format(v2, base_primary)} AC"

            now = time.time()

            if (unit_code not in VALID_UNITS) or (compare_unit != current_unit_cmp):
                current_unit = unit_code if unit_code in VALID_UNITS else current_unit
                current_unit_cmp = compare_unit if unit_code in VALID_UNITS else current_unit_cmp
                hold_until = now + HOLD_TIME_S
                status = "HOLD"

            if ok or (unit_code in VALID_UNITS):
                last_ok_ts = now

            silence_time = now - last_ok_ts
            if silence_time > OFF_TIMEOUT_S:
                status = "OFF"
            elif current_unit is None:
                status = "HOLD"
            elif now < hold_until:
                status = "HOLD"
            else:
                status = "LIVE" if (last_ok_ts and (now - last_ok_ts) < 2.0) else "HOLD"

            if status == "OFF":
                pretty = "—"
            elif status == "HOLD":
                if last_pretty and not detect_ol(val, current_unit, mode, extra):
                    pretty = last_pretty
                else:
                    pretty = "OL" if (current_unit in OHM_LIKE) else "—"
            else:
                if unit_code in VALID_UNITS:
                    main_pretty = format_for_obs(val, unit_code, mode, extra)
                    if two_channel:
                        left  = ac_pretty if order_ac_first else dc_pretty
                        right = dc_pretty if order_ac_first else ac_pretty
                        pretty = f"{left or '—'} | {right or '—'}"
                    else:
                        pretty = main_pretty
                else:
                    pretty = last_pretty if last_pretty else "—"

            delta_s = ""
            should_write = (now - last_write_ts) >= REWRITE_INTERVAL_S or pretty != last_pretty or status != last_status
            if should_write:
                if pretty != last_pretty:
                    delta_s = f"{(now - last_change_ts):.1f}"
                    last_change_ts = now
                    last_pretty = pretty
                last_write_ts = now
                # Freeze value during HOLD to avoid transient garbage, but still show OL
                if status != "HOLD" or pretty == "OL":
                    safe_write(VALUE_TXT, pretty + "\n")
                safe_write(STATUS_TXT, status + "\n")
                last_status = status
                print(f"{pretty}   [Δt={delta_s or '—'} s]   [{status}]")

            if CSV_ON_CHANGE and not should_write:
                pass
            else:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                with open(LOG_CSV, "a", encoding="utf-8") as f:
                    f.write(f"{ts},{val if val is not None else ''},{unit_raw},{unit_code or ''},{mode},{extra},{pretty},{delta_s},{status}\n")

            time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopped.")
            try:
                if tr: tr.close()
            except: pass
            sys.exit(0)
        except Exception as e:
            print(f"[WARN] {e}; retry in 0.5s")
            try:
                if tr: tr.close()
            except: pass
            tr = None
            time.sleep(0.5)

# ── Entrypoint ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--http", help="URL to /status.json on ESP (e.g., http://fluke-bridge.local/status.json)")
    ap.add_argument("--serial", default=SER_PORT, help="e.g., /dev/ttyACM0 (USB mode)")
    ap.add_argument("--tcp", help="host:port (e.g., 192.168.1.50:28700) — TCP bridge mode")
    ap.add_argument("--dir", help="output directory for fluke_*.txt/csv (default: this script's folder or FLUKE_DIR)")
    ap.add_argument("--no-fsync", action="store_true", help="disable fsync() on writes (lower latency, less disk wear)")
    ap.add_argument("--write-interval", type=float, default=None, help="min seconds between periodic rewrites (default 0.2; 0 disables periodic rewrites)")
    ap.add_argument("--qs-gap-ms", type=int, default=None, help="delay between QM and QS reads in two-channel mode (ms; default 80)")
    ap.add_argument("--csv-on-change", action="store_true", help="append CSV only when output changed (instead of every loop)")
    args = ap.parse_args()

    if args.dir:
        set_base_dir(args.dir)

    # Apply performance flags (CLI overrides env)
    if args.no_fsync:
        USE_FSYNC = False
    if args.write_interval is not None:
        REWRITE_INTERVAL_S = max(0.0, float(args.write_interval))
    if args.qs_gap_ms is not None:
        QS_GAP_MS = max(0, int(args.qs_gap_ms))
    if args.csv_on_change:
        CSV_ON_CHANGE = True

    if args.http:
        http_loop(args)
    else:
        serial_tcp_loop(args)
