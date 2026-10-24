@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0camera_lightpainting\start_lightpaint.ps1" %*
set "taskExitCode=%errorlevel%"
if not "%taskExitCode%"=="0" pause
exit /b %taskExitCode%
