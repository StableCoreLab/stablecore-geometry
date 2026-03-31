@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

set "BUILD_DIR=%SCRIPT_DIR%\build\vs2022-x64"
set "GENERATOR=Visual Studio 17 2022"
set "ARCH=x64"
set "BUILD_TESTS=ON"

if not "%~1"=="" set "BUILD_DIR=%~1"
if not "%~2"=="" set "GENERATOR=%~2"
if not "%~3"=="" set "ARCH=%~3"
if not "%~4"=="" set "BUILD_TESTS=%~4"

where cmake >nul 2>nul
if errorlevel 1 (
    echo [ERROR] cmake was not found in PATH.
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo [INFO] Source Dir  : %SCRIPT_DIR%
echo [INFO] Build Dir   : %BUILD_DIR%
echo [INFO] Generator   : %GENERATOR%
echo [INFO] Architecture: %ARCH%
echo [INFO] Build Tests : %BUILD_TESTS%

cmake -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -G "%GENERATOR%" -A "%ARCH%" -DSTABLECORE_GEOMETRY_BUILD_TESTS=%BUILD_TESTS%
if errorlevel 1 (
    echo [ERROR] Failed to generate the Visual Studio solution.
    exit /b 1
)

echo [OK] Solution generated successfully.
echo [OK] Open: "%BUILD_DIR%\stablecore_geometry.sln"
exit /b 0