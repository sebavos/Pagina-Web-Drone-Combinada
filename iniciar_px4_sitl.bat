@echo off
title PX4 Autopilot SITL (WSL2 Ubuntu) - Simulador Holybro X650
color 0A
echo ====================================================================
echo   INICIANDO PX4 AUTOPILOT SITL EN WSL2 (UBUNTU-24.04)
echo   Autopiloto: PX4 SITL (none_iris / Holybro X650)
echo   Conexiones:
echo     - MAVLink Broadcast: 0.0.0.0:14550 (Hacia la interfaz web GCS)
echo     - AirSim TCP Sensor: 127.0.0.1:4560 (Hacia Unreal Engine)
echo ====================================================================
echo.

wsl.exe -d Ubuntu-24.04 bash -c "cd ~/PX4-Autopilot && make px4_sitl none_iris"

pause
