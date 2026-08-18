@echo off
setlocal EnableExtensions
pushd "%~dp0" || goto :failed

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%CD%\scripts\package_portable.ps1"
if errorlevel 1 goto :failed

echo.
echo Portable package created successfully in the dist directory.
popd
pause
exit /b 0

:failed
echo.
echo Portable packaging failed. Review the messages above.
popd 2>nul
pause
exit /b 1

