@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  where py >nul 2>nul
  if errorlevel 1 (
    echo Can Python 64-bit va Python Launcher py. Khuyen dung Python 3.12.
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
echo Khong khoi dong duoc. Giu cua so nay de doc thong bao loi.
pause
exit /b 1
