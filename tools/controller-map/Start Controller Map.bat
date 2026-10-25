@echo off
rem Techmen 5669 Controller Map: double-click to start. Close the map window and the
rem minimized "Techmen 5669 Controller Map" window on the taskbar to stop it.
start "Techmen 5669 Controller Map" /min powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0controller-map.ps1"
