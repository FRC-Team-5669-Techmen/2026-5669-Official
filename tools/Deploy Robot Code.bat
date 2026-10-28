@echo off
rem Techmen 5669 - deploys the robot code in this folder to the robot. Double-click it.
rem The robot must be ON and the laptop connected (USB, Ethernet tether or robot Wi-Fi).
cd /d "%~dp0.."
if exist "C:\Users\Public\wpilib\2026\jdk\bin\java.exe" set "JAVA_HOME=C:\Users\Public\wpilib\2026\jdk"
echo Deploying robot code from %CD% ...
echo.
call gradlew.bat deploy --offline
if errorlevel 1 (
  echo.
  echo Offline deploy failed, trying again online...
  call gradlew.bat deploy
)
echo.
if errorlevel 1 (echo DEPLOY FAILED: read the red text above.) else (echo DEPLOY DONE. The robot code restarts by itself.)
echo.
pause
