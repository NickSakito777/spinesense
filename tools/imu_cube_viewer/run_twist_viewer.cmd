@echo off
setlocal
set "ROOT=%~dp0"
set "PY=python"
"%PY%" "%ROOT%serial_bridge.py" --serial auto --baud 921600 --port 8765
