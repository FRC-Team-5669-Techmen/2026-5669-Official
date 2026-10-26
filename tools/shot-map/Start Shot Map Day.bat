@echo off
rem Techmen 5669 - Shot Map Day: opens everything you need. Double-click it.
rem 1) Driver Station  2) Elastic dashboard  3) Controller map  4) The step-by-step guide
set DS="C:\Program Files (x86)\FRC Driver Station\DriverStation.exe"
set ELASTIC="C:\Users\Public\wpilib\2026\elastic\elastic_dashboard.exe"
if exist %DS% (start "" %DS%) else (echo Driver Station not found: open it from the Start menu.)
if exist %ELASTIC% (start "" %ELASTIC%) else (echo Elastic not found: install WPILib 2026, or open Elastic from the WPILib tools.)
if exist "%~dp0controller-map\Start Controller Map.bat" (
  call "%~dp0controller-map\Start Controller Map.bat"
) else if exist "%~dp0..\controller-map\Start Controller Map.bat" (
  call "%~dp0..\controller-map\Start Controller Map.bat"
)
start "" "%~dp0Shot Map Day Guide.html"
