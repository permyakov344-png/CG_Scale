@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0weekly-backup.ps1"
echo.
echo Sync finished. Log: ..\sketch_CG_backups\weekly-backup.log
pause
