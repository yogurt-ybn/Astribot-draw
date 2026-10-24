@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_lightpaint.ps1" --diagnostic %*
set "taskExitCode=%errorlevel%"
if not "%taskExitCode%"=="0" pause
exit /b %taskExitCode%
