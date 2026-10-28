@echo off
rem TECHMEN 5669: copy the robot's match files and logs to match-data\, then push the small ones.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Sync-MatchData.ps1"
pause
