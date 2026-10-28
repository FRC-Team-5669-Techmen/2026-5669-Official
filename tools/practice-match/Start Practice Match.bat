@echo off
rem TECHMEN 5669 - Practice match screen for a SECOND device (not the driver station).
rem The robot serves it. On the robot Wi-Fi: 10.56.69.2. USB tether: 172.22.11.2.
rem No robot reachable? The page still runs a standalone match timer (button at the top).
ping -n 1 -w 700 10.56.69.2 | find "TTL=" >nul && (start "" "http://10.56.69.2:5806/" & exit /b)
ping -n 1 -w 700 172.22.11.2 | find "TTL=" >nul && (start "" "http://172.22.11.2:5806/" & exit /b)
echo Robot not reachable: opening the standalone timer from this folder.
start "" "%~dp0..\..\src\main\deploy\practice-match\index.html"
