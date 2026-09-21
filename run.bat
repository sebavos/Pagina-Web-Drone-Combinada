@echo off
title Estacion Terrena Holybro X650 (Pixhawk 6X) - UBO
color 0B
echo ====================================================================
echo   INICIANDO ESTACION TERRENA GCS - HOLYBRO X650 / PIXHAWK 6X
echo   Universidad Bernardo O'Higgins (UBO)
echo ====================================================================
echo.

:: Detectar el ejecutable de Python disponible
set PYTHON_EXE=
if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" (
    set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
) else if exist "%LOCALAPPDATA%\Programs\Python\Python314\python.exe" (
    set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Python314\python.exe"
) else if exist "%LOCALAPPDATA%\Programs\Python\Launcher\py.exe" (
    set "PYTHON_EXE=%LOCALAPPDATA%\Programs\Python\Launcher\py.exe"
) else (
    where py.exe >nul 2>&1 && set "PYTHON_EXE=py"
    if not defined PYTHON_EXE (
        where python.exe >nul 2>&1 && set "PYTHON_EXE=python"
    )
)

if not defined PYTHON_EXE (
    echo [ERROR] No se pudo encontrar Python en el sistema.
    echo Por favor instala Python o agregalo a tu PATH.
    pause
    exit /b 1
)

echo [1/2] Usando interprete: %PYTHON_EXE%
echo [2/2] Iniciando Servidor GCS (Flask + MAVLink 20Hz)...
start http://localhost:5000
"%PYTHON_EXE%" app.py

pause
