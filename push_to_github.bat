@echo off
title Subir Cambios a GitHub - Telemetria Holybro X650 (UBO)
color 0B
echo ====================================================================
echo   SUBIENDO PROYECTO A GITHUB: Telemetria_Holybro_x650
echo   Universidad Bernardo O'Higgins (UBO)
echo ====================================================================
echo.

:: Detectar ejecutable de Git en el sistema
set GIT_EXE=
where git.exe >nul 2>&1 && set "GIT_EXE=git"

if not defined GIT_EXE (
    if exist "C:\Program Files\Git\cmd\git.exe" (
        set "GIT_EXE=C:\Program Files\Git\cmd\git.exe"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\TeamFoundation\Team Explorer\Git\cmd\git.exe" (
        set "GIT_EXE=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\TeamFoundation\Team Explorer\Git\cmd\git.exe"
    ) else if exist "C:\Program Files (x86)\Git\cmd\git.exe" (
        set "GIT_EXE=C:\Program Files (x86)\Git\cmd\git.exe"
    ) else if exist "%LOCALAPPDATA%\Programs\Git\cmd\git.exe" (
        set "GIT_EXE=%LOCALAPPDATA%\Programs\Git\cmd\git.exe"
    )
)

if not defined GIT_EXE (
    echo [ERROR] No se encontro ejecutable de Git en el sistema.
    echo Por favor instala Git o agregalo a tu PATH.
    pause
    exit /b 1
)

echo [1/4] Ejecutable Git detectado.

:: Verificar si el directorio es repositorio Git
if not exist ".git" (
    echo [2/4] Inicializando repositorio Git local...
    "%GIT_EXE%" init
    "%GIT_EXE%" branch -M main
    "%GIT_EXE%" remote add origin https://github.com/sebavos/Pagina-Web-Drone-Combinada.git
) else (
    echo [2/4] Repositorio Git local verificado.
)

echo [3/4] Agregando archivos y creando commit...
"%GIT_EXE%" add .
"%GIT_EXE%" commit -m "Actualizacion Estacion Terrena GCS Holybro X650 UBO - Misiones, Puertos COM y UI"

echo [4/4] Subiendo cambios a GitHub (git push -u origin main)...
"%GIT_EXE%" push -u origin main

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ====================================================================
    echo   [EXITO] Proyecto subido correctamente a GitHub!
    echo   https://github.com/sebavos/Pagina-Web-Drone-Combinada
    echo ====================================================================
) else (
    echo.
    echo ====================================================================
    echo   [NOTA] Si fallo la autenticacion o el push, verifica tus credenciales
    echo   de GitHub en Windows Credential Manager.
    echo ====================================================================
)

echo.
pause
