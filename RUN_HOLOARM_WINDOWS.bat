@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  where py >nul 2>nul
  if errorlevel 1 (
    echo 64-bit Python and the Python Launcher py are required. Python 3.12 is recommended.
    goto :failed
  )
  py -3 -m venv .venv
  if errorlevel 1 goto :failed
)
if not exist ".venv\holoarm_cad_installed_v1.ok" (
  .venv\Scripts\python.exe -m pip install -r requirements-windows.txt
  if errorlevel 1 goto :failed
  type nul > .venv\holoarm_cad_installed_v1.ok
)
.venv\Scripts\python.exe windows_launcher.py
if errorlevel 1 goto :failed
exit /b 0
:failed
echo Startup failed. Keep this window open to read the error message.
pause
exit /b 1
