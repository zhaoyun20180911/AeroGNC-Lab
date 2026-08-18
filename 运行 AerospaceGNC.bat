@echo off
setlocal EnableExtensions
pushd "%~dp0" || goto :launch_failed

set "APP_EXE=build\Release\AerospaceGNC.exe"

if not exist "%APP_EXE%" (
    echo [AerospaceGNC] First run: configuring and building Release...
    where cmake >nul 2>nul
    if errorlevel 1 goto :cmake_missing
    cmake -S . -B build -A x64
    if errorlevel 1 goto :build_failed
    cmake --build build --config Release --parallel
    if errorlevel 1 goto :build_failed
)

start "" "%CD%\%APP_EXE%"
popd
exit /b 0

:cmake_missing
echo CMake was not found. Install Visual Studio C++ and CMake, then try again.
goto :failed

:build_failed
echo AerospaceGNC build failed. Review the messages above.
goto :failed

:launch_failed
echo Unable to enter the AerospaceGNC project directory.

:failed
popd 2>nul
pause
exit /b 1
