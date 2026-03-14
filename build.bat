@echo off
setlocal EnableDelayedExpansion
title HAV Framework - Windows Build

echo.
echo  HAV Framework  -  Windows Build Script
echo  ========================================
echo.

set GENERATOR=
set BUILD_DIR=build

where cl >nul 2>&1
if %errorlevel%==0 (
    echo  [+] Detected MSVC compiler
    for /f "tokens=*" %%V in ('cl 2^>^&1 ^| findstr /i "Version"') do (
        echo  [+] Compiler: %%V
        echo %%V | findstr "19\.4" >nul && set GENERATOR=Visual Studio 18 2026
        if "!GENERATOR!"=="" echo %%V | findstr "19\.3" >nul && set GENERATOR=Visual Studio 17 2022
        if "!GENERATOR!"=="" echo %%V | findstr "19\.2" >nul && set GENERATOR=Visual Studio 16 2019
    )
    if "!GENERATOR!"=="" (
        echo  [~] VS version unclear, defaulting to VS 2026
        set GENERATOR=Visual Studio 18 2026
    )
) else (
    where g++ >nul 2>&1
    if !errorlevel!==0 (
        echo  [+] Detected MinGW/GCC
        set GENERATOR=MinGW Makefiles
    ) else (
        echo  [!] No C++ compiler found.
        echo      - Visual Studio 2026 with "Desktop development with C++"
        echo      - MinGW-w64: https://winlibs.com
        pause & exit /b 1
    )
)

echo  [+] Generator : !GENERATOR!

where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo  [!] CMake not found: https://cmake.org/download/
    pause & exit /b 1
)

where python >nul 2>&1
if %errorlevel% neq 0 (
    echo  [!] Python not found: https://python.org
    pause & exit /b 1
)
for /f "tokens=*" %%V in ('python --version 2^>^&1') do echo  [+] %%V

python -c "import pybind11" >nul 2>&1
if %errorlevel% neq 0 (
    echo  [+] Installing pybind11...
    python -m pip install pybind11 --quiet
)

echo.
echo  [+] Configuring...
if exist %BUILD_DIR% rmdir /s /q %BUILD_DIR%
mkdir %BUILD_DIR%

if "!GENERATOR!"=="MinGW Makefiles" (
    cmake -S . -B %BUILD_DIR% -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
) else (
    cmake -S . -B %BUILD_DIR% -G "!GENERATOR!" -A x64
)
if %errorlevel% neq 0 ( echo  [!] Configure failed. & pause & exit /b 1 )

echo  [+] Building...
cmake --build %BUILD_DIR% --config Release --parallel
if %errorlevel% neq 0 ( echo  [!] Build failed. & pause & exit /b 1 )

echo.
echo  [+] Outputs:
for %%f in (core\hav_native*.pyd core\Release\hav_native*.pyd) do (
    if exist "%%f" ( echo      %%f & copy /y "%%f" "core\" >nul 2>&1 )
)
for %%f in (hav_cli.exe Release\hav_cli.exe) do (
    if exist "%%f" echo      %%f
)

echo.
echo  Done! Run: hav_cli.exe   ^|   python core\hav_api.py   ^|   start frontend\index.html
echo.
pause