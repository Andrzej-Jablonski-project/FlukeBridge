@echo off
REM Fluke Bridge helper for Windows: HTTP→USB fallback + optional OBS launch
setlocal ENABLEDELAYEDEXPANSION

REM Defaults (can be overridden before calling)
if not defined URL set "URL=http://fluke-bridge.local/status.json"

REM Auto-detect COM port if SER is not preset by user
if not defined SER (
  for /f "usebackq delims=" %%C in (`powershell -NoProfile -Command "$p=Get-CimInstance Win32_SerialPort ^| Where-Object { $_.Name -match 'USB|Seeed|CP210|CH340|CH910|Silicon|FTDI' } ^| Select-Object -First 1 -ExpandProperty DeviceID; if(-not $p){ (Get-CimInstance Win32_SerialPort ^| Select-Object -First 1 -ExpandProperty DeviceID) }"`) do (
    set "SER=%%C"
  )
)
if not defined SER set "SER=COM3"
echo [RUN] Using serial port %SER% (override by: set SER=COMx)

echo [RUN] Probing %URL% ...
powershell -NoProfile -Command "try{ (Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 -Uri '%URL%') | Out-Null; exit 0 } catch { exit 1 }"
if %ERRORLEVEL% EQU 0 (
  echo [RUN] HTTP available – using %URL%
  start "fluke_read_http" /B cmd /C "py -3 %~dp0fluke_read.py --http %URL%"
) else (
  echo [RUN] HTTP unavailable – using USB %SER%
  start "fluke_read_usb" /B cmd /C "py -3 %~dp0fluke_read.py --serial %SER%"
)

REM Try to launch OBS if installed in default location (optional)
set "OBS_EXE=%ProgramFiles%\obs-studio\bin\64bit\obs64.exe"
if exist "%OBS_EXE%" (
  echo [RUN] Launching OBS: "%OBS_EXE%"
  start "OBS" "%OBS_EXE%"
) else (
  echo [INFO] OBS not found at default path. Launch OBS manually if needed.
)

echo [INFO] Press Ctrl+C in this window to stop background reader processes.
endlocal
