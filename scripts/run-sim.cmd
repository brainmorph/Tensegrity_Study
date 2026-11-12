@echo off
rem Runs scripts\run-sim.ps1 from cmd.exe or a double-click, whatever the
rem PowerShell execution policy. Same arguments, e.g. run-sim toggle_arm_snap --view
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run-sim.ps1" %*
exit /b %ERRORLEVEL%
