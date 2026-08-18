@echo off
setlocal EnableExtensions
pushd "%~dp0" || goto :failed

where cmake >nul 2>nul
if errorlevel 1 goto :cmake_missing

cmake -S . -B build -A x64
if errorlevel 1 goto :failed
cmake --build build --config Release --parallel
if errorlevel 1 goto :failed
ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo AerospaceGNC build and tests passed.
popd
pause
exit /b 0

:cmake_missing
echo CMake was not found. Install Visual Studio C++ and CMake, then try again.

:failed
echo.
echo AerospaceGNC build or tests failed.
popd 2>nul
pause
exit /b 1
