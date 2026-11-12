@echo off
rem Runs scripts\view-latest.ps1 from cmd.exe or a double-click, whatever
rem the PowerShell execution policy. Same arguments, e.g. view-latest --play
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0view-latest.ps1" %*
exit /b %ERRORLEVEL%
