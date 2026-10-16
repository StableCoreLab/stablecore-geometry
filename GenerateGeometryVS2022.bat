@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

for %%I in ("%SCRIPT_DIR%\..\Build\GeometryVs2022") do set "BUILD_DIR=%%~fI"
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

echo [INFO] Source Dir  : %SCRIPT_DIR%
echo [INFO] Build Dir   : %BUILD_DIR%

rem The no-argument workflow takes its generator, architecture and build directory from CMakePresets.json.
if "%~1%~2%~3%~4"=="" (
    echo [INFO] Preset      : vs2022-x64
    cmake -S "%SCRIPT_DIR%" --preset vs2022-x64
    if errorlevel 1 (
        echo [ERROR] Failed to generate the Visual Studio solution.
        exit /b 1
    )
    echo [OK] Solution generated successfully.
    echo [OK] Solution: "%BUILD_DIR%\SCGeometry.sln"
    exit /b 0
)

echo [INFO] Generator   : %GENERATOR%
echo [INFO] Architecture: %ARCH%
echo [INFO] Build Tests : %BUILD_TESTS%
cmake -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -G "%GENERATOR%" -A "%ARCH%" -DSCGEOMETRY_BUILD_TESTS=%BUILD_TESTS%
if errorlevel 1 (
    echo [ERROR] Failed to generate the Visual Studio solution.
    exit /b 1
)

echo [OK] Solution generated successfully.
echo [OK] Solution: "%BUILD_DIR%\SCGeometry.sln"
exit /b 0
