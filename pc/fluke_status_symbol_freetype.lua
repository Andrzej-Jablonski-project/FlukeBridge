-- fluke_status_symbol_freetype.lua
-- Source: Text (FreeType2) – auto text (status only) and color.
-- Reads status file (LIVE/HOLD/OFF) and sets plain text (no symbols) and color.

local obs = obslua

-- defaults (can be changed in GUI)
local source_name = "Fluke_status"
local status_file = "fluke_status.txt"
local interval_ms = 100

-- colors (0xRRGGBB) – can be changed in GUI
local color_live = 0x00FF66  -- green
local color_hold = 0x00CCFF  -- blue
local color_off  = 0xBBBBBB  -- gray

-- state
local last_hash = ""

local function read_file(path)
  local f = io.open(path, "r")
  if not f then return nil end
  local txt = f:read("*a") or ""
  f:close()
  return txt
end

local function parse_status(s)
  if not s then return "OFF" end
  local t = string.upper(s)
  if string.find(t, "LIVE", 1, true) then return "LIVE" end
  if string.find(t, "HOLD", 1, true) then return "HOLD" end
  if string.find(t, "OFF",  1, true) then return "OFF"  end
  return "OFF"
end

local function set_text_and_color(src_name, text, rgb)
  local src = obs.obs_get_source_by_name(src_name)
  if src == nil then return end
  local settings = obs.obs_source_get_settings(src)

  -- Disable reading from file – script sets the text
  obs.obs_data_set_bool(settings, "read_from_file", false)

  -- set text and color
  obs.obs_data_set_string(settings, "text", text)
  obs.obs_data_set_bool(settings, "gradient", false)
  obs.obs_data_set_int(settings,  "color1", rgb)

  obs.obs_source_update(src, settings)
  obs.obs_data_release(settings)
  obs.obs_source_release(src)
end

local function tick()
  if source_name == "" or status_file == "" then return end
  local raw = read_file(status_file)
  if not raw then return end

  local st = parse_status(raw)
  local col = (st == "LIVE") and color_live or (st == "HOLD") and color_hold or color_off

  local now = st .. "|" .. string.format("%06X", col)
  if now == last_hash then return end
  last_hash = now

  set_text_and_color(source_name, st, col)
end

-- OBS API
function script_description()
  return "Text (FreeType2): auto status + color from fluke_status.txt (no symbols)."
end

function script_properties()
  local p = obs.obs_properties_create()
  obs.obs_properties_add_text (p, "source_name", "Source name (FreeType2)", obs.OBS_TEXT_DEFAULT)
  obs.obs_properties_add_path (p, "status_file", "Status file", obs.OBS_PATH_FILE, "TXT (*.txt);;All (*.*)", nil)
  obs.obs_properties_add_int  (p, "interval_ms", "Interval (ms)", 50, 1000, 10)

  obs.obs_properties_add_color(p, "color_live", "Color LIVE")
  obs.obs_properties_add_color(p, "color_hold", "Color HOLD")
  obs.obs_properties_add_color(p, "color_off",  "Color OFF")
  return p
end

function script_defaults(s)
  obs.obs_data_set_default_string(s, "source_name", source_name)
  obs.obs_data_set_default_string(s, "status_file", status_file)
  obs.obs_data_set_default_int   (s, "interval_ms", interval_ms)

  obs.obs_data_set_default_int   (s, "color_live", color_live)
  obs.obs_data_set_default_int   (s, "color_hold", color_hold)
  obs.obs_data_set_default_int   (s, "color_off",  color_off)
end

function script_update(s)
  source_name = obs.obs_data_get_string(s, "source_name")
  status_file = obs.obs_data_get_string(s, "status_file")
  interval_ms = obs.obs_data_get_int   (s, "interval_ms")

  color_live = obs.obs_data_get_int(s, "color_live")
  color_hold = obs.obs_data_get_int(s, "color_hold")
  color_off  = obs.obs_data_get_int(s, "color_off")

  obs.timer_remove(tick)
  if source_name ~= "" and status_file ~= "" then
    obs.timer_add(tick, math.max(50, interval_ms))
    tick()
  end
end

function script_unload()
  obs.timer_remove(tick)
end
