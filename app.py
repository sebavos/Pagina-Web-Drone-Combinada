import time
import math
import csv
import threading
import logging
import json
import os
import socket

# Forzar transporte TCP para RTSP en OpenCV (evita caídas y timeouts de UDP en Windows/LAN)
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
import cv2
import numpy as np
from flask import Flask, render_template, jsonify, request, send_file, Response
from pymavlink import mavutil

# Intentar importar AirSim para conexión directa con Unreal Engine
try:
    import airsim
    HAY_AIRSIM = True
except Exception:
    airsim = None
    HAY_AIRSIM = False

# Intentar importar ultralytics YOLO
try:
    from ultralytics import YOLO
    HAY_ULTRALYTICS = True
except Exception as e:
    YOLO = None
    HAY_ULTRALYTICS = False
    print(f"[Aviso] Ultralytics no disponible directamente: {e}")

# Desactivar logs internos redundantes de Flask
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Intentar importar PyTorch para aceleración CUDA en RTX 4060
try:
    import torch
    HAY_TORCH = True
except Exception:
    torch = None
    HAY_TORCH = False

app = Flask(__name__)

# --- CONFIGURACIÓN POR DEFECTO ---
ARCHIVO_LOG_ACTUAL = "telemetria_escritorio.csv"
PUERTO_DEFECTO = "COM7"
BAUD_DEFECTO = 57600

# Constantes de comandos MAVLink
MAV_CMD_DO_MOTOR_TEST = 209          # Comando estándar clásico / ArduPilot
MAV_CMD_ACTUATOR_TEST = 310          # Comando estándar moderno PX4 (Pixhawk 6X / QGC)
MAV_CMD_COMPONENT_ARM_DISARM = 400   # Armar/Desarmar
MAV_CMD_DO_SET_MODE = 176            # Cambiar modo de vuelo clásico
MAV_CMD_SET_MESSAGE_INTERVAL = 511   # Solicitar tasa específica por mensaje
MOTOR_TEST_THROTTLE_PERCENT = 0      # Tipo de acelerador 0: porcentaje (0 - 100)

# Control de Hilos y Conexión MAVLink
grabando_telemetria = True
hilo_receptor_activo = True
conexion = None
conexion_lock = threading.Lock()
hilo_telemetria = None

# Configuración activa de conexión
config_conexion = {
    "puerto": PUERTO_DEFECTO,
    "baud": BAUD_DEFECTO,
    "tipo": "serial",
    "conectado": False,
    "estado_texto": "Desconectado",
    "paquetes_totales": 0,
    "hz_actual": 0.0,
    "perdida_paquetes_pct": 0.0,
    "modo_simulacion": False
}

def limpiar_float(val, default=0.0):
    """Limpia valores flotantes evitando NaN o Inf para JSON estricto"""
    if val is None:
        return default
    try:
        f = float(val)
        if math.isnan(f) or math.isinf(f):
            return default
        return round(f, 3)
    except (ValueError, TypeError):
        return default

# Estructura de Telemetría Completa y Consolidada de Grado Aeroespacial
telemetria_actual = {
    "conectado": False,
    "tiempo": 0.0,
    "hz_recepcion": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0,
    "conexion": {
        "puerto": PUERTO_DEFECTO,
        "baud": BAUD_DEFECTO,
        "conectado": False,
        "estado": "Desconectado",
        "paquetes_recibidos": 0,
        "hz": 0.0,
        "simulacion": False
    },
    "sistema": {
        "modo_vuelo": "DESCONOCIDO",
        "armado": False,
        "tipo_autopilot": "PX4",
        "carga_cpu": 0.0,
        "perdida_com": 0.0,
        "voltaje_bateria": 0.0,
        "corriente_bateria": 0.0,
        "bateria_pct": 0,
        "mah_consumidos": 0.0,
        "voltaje_celda": 0.0,
        "temperatura_placa": 0.0
    },
    "actitud": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0,
        "pitch_deg": 0.0,
        "roll_deg": 0.0,
        "yaw_deg": 0.0,
        "pitchspeed": 0.0,
        "rollspeed": 0.0,
        "yawspeed": 0.0
    },
    "vfr_hud": {
        "airspeed": 0.0,
        "groundspeed": 0.0,
        "altitud_baro": 0.0,
        "climb": 0.0,
        "heading": 0,
        "throttle": 0
    },
    "gps": {
        "fix_type": 0,
        "fix_desc": "Sin GPS (Interiores)",
        "lat": -33.4668085,
        "lon": -70.6548984,
        "alt_amsl": 0.0,
        "alt_rel": 0.0,
        "satellites": 0,
        "hdop": 99.9,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0
    },
    "imu": {
        "acc_x": 0.0,
        "acc_y": 0.0,
        "acc_z": 0.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "mag_x": 0.0,
        "mag_y": 0.0,
        "mag_z": 0.0,
        "presion_hpa": 1013.25,
        "temp_sensor": 25.0
    },
    "vibracion": {
        "vibe_x": 0.0,
        "vibe_y": 0.0,
        "vibe_z": 0.0,
        "clip_0": 0,
        "clip_1": 0,
        "clip_2": 0,
        "nivel_salud": "Excelente"
    },
    "ekf": {
        "flags": 0,
        "velocity_variance": 0.0,
        "pos_horiz_variance": 0.0,
        "pos_vert_variance": 0.0,
        "compass_variance": 0.0,
        "terrain_alt_variance": 0.0,
        "salud_ekf": "Normal"
    },
    "control_guiado": {
        "nav_pitch": 0.0,
        "nav_roll": 0.0,
        "nav_bearing": 0,
        "target_bearing": 0,
        "wp_dist": 0.0,
        "alt_error": 0.0,
        "aspd_error": 0.0,
        "xtrack_error": 0.0
    },
    "actuadores": {
        "motor1_pwm": 1000,
        "motor2_pwm": 1000,
        "motor3_pwm": 1000,
        "motor4_pwm": 1000,
        "motor5_pwm": 1000,
        "motor6_pwm": 1000,
        "motor7_pwm": 1000,
        "motor8_pwm": 1000
    },
    "rc": {
        "ch1": 1500, "ch2": 1500, "ch3": 1000, "ch4": 1500,
        "ch5": 1000, "ch6": 1000, "ch7": 1000, "ch8": 1000,
        "rssi": 0
    },
    "parametros": {
        "MC_ROLLRATE_P": 0.15,
        "MC_ROLLRATE_I": 0.20,
        "MC_ROLLRATE_D": 0.003,
        "MC_PITCHRATE_P": 0.15,
        "MC_PITCHRATE_I": 0.20,
        "MC_PITCHRATE_D": 0.003,
        "MC_YAWRATE_P": 0.20,
        "MC_YAWRATE_I": 0.10,
        "MC_YAWRATE_D": 0.000,
        "MPC_XY_VEL_MAX": 12.0,
        "MPC_Z_VEL_MAX_UP": 3.0,
        "MPC_Z_VEL_MAX_DN": 1.5
    },
    "mensajes_estado": [],
    "simulador": {
        "activo": False,
        "indice": 0,
        "total": 0,
        "velocidad": 1.0,
        "archivo": ARCHIVO_LOG_ACTUAL
    }
}

# ==============================================================
# SUBSISTEMA DE VISIÓN IA (YOLOv8) & SIMULACIÓN UNREAL ENGINE
# ==============================================================
DRONE_MODEL_PATH_CUSTOM = r"C:\drone\scripts\runs\detect\runs\drone_finetuned-3\weights\best.pt"
STOCK_MODEL_PATH = "yolov8n.pt"

# Si el path absoluto no existe, buscar rutas relativas
if not os.path.exists(DRONE_MODEL_PATH_CUSTOM):
    for candidate in ["yolov8n.pt", "scripts/yolov8n.pt", "../yolov8n.pt"]:
        if os.path.exists(candidate):
            DRONE_MODEL_PATH_CUSTOM = candidate
            break

# Estado de configuración de video y visión
config_vision = {
    "fuente_tipo": "rtsp",               # 'rtsp', 'unreal', 'webcam', 'sintetico'
    "rtsp_url": "rtsp://192.168.14.7:8554/cam0",
    "unreal_url": "rtsp://127.0.0.1:8554/live",
    "webcam_idx": 0,
    "flip_camara": True,                 # Voltear 180 grados (cámara Jetson invertida)
    "unreal_source": "airsim",           # 'airsim' (RPC directo port 41451) o 'rtsp'
    "modelo_activo": "drone",            # 'drone', 'person', 'both'
    "confianza": 0.30,
    "tracking_activo": True,
    "alertas_activas": True,
    "modo_seguimiento": "OFF",           # 'OFF', 'SIM', 'ACTIVE_TRACK'
    "camara_activa": "front",            # 'front' (FPV frontal) o 'bottom' (cenital / suelo)
    "target_drone_activo": False,        # Indicador de dron objetivo spawn/presente
    "estado_camara": "Iniciando...",
    "fps_camara": 0.0,
    "fps_ia": 0.0,
    "resolucion": "1280x720"
}

# Estado dinámico de detección y seguimiento
estado_vision = {
    "conectado": False,
    "camara_activa": "front",
    "target_drone_activo": False,
    "detecciones": [],
    "conteo_drones": 0,
    "conteo_personas": 0,
    "objetivo_fijado": False,
    "target_id": None,
    "target_bbox": None,
    "alerta_activa": False,
    "mensaje_alerta": ""
}

# Buffers y sincronización de hilos
current_raw_frame = None
current_processed_frame = None
frame_lock = threading.Lock()
vision_lock = threading.Lock()

# Pipeline de streaming MJPEG de latencia cero (Zero-Lag Drop-Frame)
stream_cond = threading.Condition()
current_encoded_jpeg = None
current_frame_id = 0
airsim_spawn_origin = {"lat": -33.4668085, "lon": -70.6548984, "alt": 0.0}

# Modelos cargados
model_drone_inst = None
model_stock_inst = None
model_lock = threading.Lock()

# Parámetros y salidas del PID de Visión
pid_out = {"yaw_rate": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}
pid_err = {"ex": 0.0, "ey": 0.0, "earea": 0.0}
lost_start_time = None
last_alert_time = 0.0
ALERT_COOLDOWN = 4.0

# ==============================================================
# CLASE PID DE IMAGEN (Anti-windup + Deadband + Saturación + Rampa)
# ==============================================================
class PIDController:
    def __init__(self, kp, ki, kd, out_max, deadband=0.0, ramp=1.0, name=""):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_max = abs(float(out_max))
        self.deadband = float(deadband)
        self.ramp = float(ramp)
        self.name = name
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.last_err = 0.0
        self.last_out = 0.0
        self.last_time = None
        self.saturated = False

    def update(self, error):
        now = time.time()
        if self.last_time is None:
            dt = 1.0 / 25.0
        else:
            dt = max(0.001, min(now - self.last_time, 0.5))
        self.last_time = now

        # Zona muerta
        if abs(error) < self.deadband:
            out = 0.0
            self.i_term = 0.0
            self.last_err = error
            self.last_out = out
            return out

        # Proporcional
        p = self.kp * error
        # Integral con anti-windup
        self.i_term += self.ki * error * dt
        if self.saturated:
            self.i_term = 0.0
        # Derivada
        d = -self.kd * (error - self.last_err) / dt if dt > 0 else 0.0
        self.last_err = error

        raw = p + self.i_term + d
        out = max(-self.out_max, min(self.out_max, raw))
        self.saturated = (abs(raw) >= self.out_max)

        # Rampa
        max_step = self.out_max * self.ramp
        out = max(self.last_out - max_step, min(self.last_out + max_step, out))
        self.last_out = out
        return out


class ImagePID:
    """Coordina los 4 controladores PID de imagen: Yaw, Altura (Vz), Avance (Vx), Deriva (Vy)"""
    def __init__(self, frame_w=640, frame_h=480):
        self.w = frame_w
        self.h = frame_h
        self.cx = frame_w / 2.0
        self.cy = frame_h / 2.0
        self.target_area_frac = 0.12
        self.target_area = self.target_area_frac * frame_w * frame_h

        self.pid_yaw = PIDController(1.2, 0.0, 0.35, 0.8, deadband=0.05, ramp=0.15, name="yaw")
        self.pid_alt = PIDController(0.8, 0.05, 0.2, 1.5, deadband=0.06, ramp=0.20, name="alt")
        self.pid_roll = PIDController(0.0, 0.0, 0.0, 1.0, deadband=0.05, ramp=0.20, name="roll")
        self.pid_spd = PIDController(0.9, 0.06, 0.2, 2.5, deadband=0.0, ramp=0.20, name="spd")
        self.alpha = 0.5
        self.last_ex = 0.0
        self.last_ey = 0.0

    def reset(self):
        for p in (self.pid_yaw, self.pid_alt, self.pid_roll, self.pid_spd):
            p.reset()
        self.last_ex = 0.0
        self.last_ey = 0.0

    def set_dimensions(self, w, h):
        self.w = w
        self.h = h
        self.cx = w / 2.0
        self.cy = h / 2.0
        self.target_area = self.target_area_frac * w * h

    def compute(self, box):
        x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
        tcx = (x1 + x2) / 2.0
        tcy = (y1 + y2) / 2.0
        area = max(1.0, (x2 - x1) * (y2 - y1))

        # Errores normalizados (-1..1)
        ex = (tcx - self.cx) / self.cx
        ey = (tcy - self.cy) / self.cy
        earea = (area - self.target_area) / self.target_area

        # Filtro LPF
        ex = self.alpha * ex + (1.0 - self.alpha) * self.last_ex
        ey = self.alpha * ey + (1.0 - self.alpha) * self.last_ey
        self.last_ex, self.last_ey = ex, ey

        yaw_rate = self.pid_yaw.update(ex)
        vz = -self.pid_alt.update(-ey)
        vy = -self.pid_roll.update(ex)
        vx = -self.pid_spd.update(-earea)

        return (ex, ey, earea), (yaw_rate, vx, vy, vz)

image_pid_inst = ImagePID(640, 480)

# ==============================================================
# HILO DE CAPTURA DE VIDEO MULTIFUENTE (RTSP / UNREAL / WEBCAM / SINTETICO)
# ==============================================================
def generar_frame_sintetico(t):
    """Genera un cuadro sintético con cuadrícula táctica y objetivo dron móvil 3D"""
    h, w = 480, 640
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Cuadrícula táctica
    for y in range(0, h, 40):
        cv2.line(frame, (0, y), (w, y), (18, 28, 44), 1)
    for x in range(0, w, 40):
        cv2.line(frame, (x, 0), (x, h), (18, 28, 44), 1)
        
    # Horizonte virtual de fondo
    cv2.line(frame, (0, h // 2), (w, h // 2), (30, 58, 95), 1)
    
    # Movimiento oscilatorio suave del objetivo simulado
    sx = 0.52 * math.sin(t * 0.75)
    sy = 0.32 * math.sin(t * 0.45 + 1.2)
    sz = 0.55 + 0.22 * math.sin(t * 0.32)
    
    cx = int(w / 2 + sx * (w / 2 - 80))
    cy = int(h / 2 + sy * (h / 2 - 80))
    bw = int(w * 0.22 * max(0.2, sz))
    bh = int(h * 0.22 * max(0.2, sz))
    
    x1 = max(10, cx - bw // 2)
    y1 = max(10, cy - bh // 2)
    x2 = min(w - 10, cx + bw // 2)
    y2 = min(h - 10, cy + bh // 2)
    
    # Dibuja representación del dron virtual
    cv2.line(frame, (x1, y1), (x2, y2), (0, 210, 255), 2)
    cv2.line(frame, (x1, y2), (x2, y1), (0, 210, 255), 2)
    cv2.circle(frame, (cx, cy), max(3, int(min(bw, bh) * 0.22)), (0, 242, 254), -1)
    for px, py in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
        cv2.circle(frame, (px, py), 7, (245, 158, 11), 2)
    
    cv2.putText(frame, "SIMULACION SINTETICA ACTIVA (MODO PRUEBA)", (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 242, 254), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Dron Simulado: X={sx:+.2f} Y={sy:+.2f} Escala={sz:.2f}", (16, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (148, 163, 184), 1, cv2.LINE_AA)
                
    synthetic_box = [x1, y1, x2, y2, "DRON", 0.95, 1]
    return frame, synthetic_box

# ==============================================================
# GESTIÓN MULTICÁMARA & DRON OBJETIVO SIMULADO (AIRSIM UNREAL)
# ==============================================================
def seleccionar_camara(cam_tipo):
    """Permite alternar entre la cámara frontal (FPV 0°), inclinada táctica (-45°) y cenital (Nadir -90° mirando al suelo)"""
    global config_vision
    raw_str = str(cam_tipo).lower()
    if any(w in raw_str for w in ["angle", "inclinad", "45", "diag"]):
        tipo = "angle"
    elif any(w in raw_str for w in ["bottom", "zenit", "cenit", "nadir", "abajo", "suelo", "90"]):
        tipo = "bottom"
    else:
        tipo = "front"

    with vision_lock:
        config_vision["camara_activa"] = tipo
        estado_vision["camara_activa"] = tipo
    
    # Orientar la cámara físicamente en AirSim para soporte inmediato en vivo
    if HAY_AIRSIM and tipo != "bottom":
        # bottom_cam es fija (-90° en settings.json), solo front_cam necesita gimbal
        try:
            client_as = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=1.5)
            if tipo == "angle":
                # Posición Z=0.40 y X=0.35: vista inclinada de vigilancia a 45° completamente libre de patas
                pose = airsim.Pose(airsim.Vector3r(0.35, 0.0, 0.40), airsim.to_quaternion(math.radians(-45), 0, 0))
            else:
                # Frontal estándar a nivel de la nariz
                pose = airsim.Pose(airsim.Vector3r(0.30, 0.0, -0.10), airsim.to_quaternion(0, 0, 0))
            client_as.client.call('simSetCameraPose', 'front_cam', pose, 'PX4')
        except Exception:
            pass

    
    nombres = {
        "bottom": "Cenital / Suelo (Nadir -90°)",
        "angle": "Inclinada Vigilancia (-45°)",
        "front": "Frontal (FPV 0°)"
    }
    nombre_txt = nombres.get(tipo, "Frontal (FPV 0°)")
    print(f"[Visión Cámara] 📷 Perspectiva activa cambiada a: {nombre_txt}")
    return True, f"Cámara cambiada a: {nombre_txt}"

def posicionar_dron_objetivo(distancia=7.0, altitud_rel=0.5):
    """Posiciona o spawnea un dron objetivo en Unreal Engine frente al dron principal para pruebas de detección YOLO"""
    if not HAY_AIRSIM:
        return False, "AirSim no está disponible."
    try:
        client_as = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2.5)
        kin = client_as.simGetGroundTruthKinematics('PX4')
        yaw_rad = quaternion_to_euler(kin.orientation)[2]
        
        dist = float(distancia)
        alt_r = float(altitud_rel)
        
        dx = dist * math.cos(yaw_rad)
        dy = dist * math.sin(yaw_rad)
        dz = -alt_r
        
        target_pos = airsim.Vector3r(kin.position.x_val + dx, kin.position.y_val + dy, kin.position.z_val + dz)
        # Orientar el dron objetivo mirando de frente hacia el dron principal
        target_pose = airsim.Pose(target_pos, airsim.to_quaternion(0, 0, yaw_rad + math.pi))
        
        vehiculos = client_as.listVehicles()
        if 'TargetDrone' not in vehiculos:
            client_as.simAddVehicle('TargetDrone', 'SimpleFlight', target_pose)
        else:
            client_as.simSetVehiclePose(target_pose, ignore_collision=True, vehicle_name='TargetDrone')
            
        with vision_lock:
            config_vision["target_drone_activo"] = True
            estado_vision["target_drone_activo"] = True
            
        print(f"[Simulación] 🎯 TargetDrone posicionado a {dist}m frente a PX4.")
        return True, f"Dron objetivo posicionado a {dist}m al frente (Alt: +{alt_r}m)."
    except Exception as e:
        return False, f"Error al posicionar dron objetivo en Unreal: {e}"

def capturador_video_worker():
    """Hilo encargado de mantener abierto el flujo de video activo (RTSP Jetson, AirSim Unreal, Webcam o Sintético)"""
    global current_raw_frame, config_vision, estado_vision
    
    ultimo_calculo_fps = time.time()
    frames_capturados = 0
    
    while hilo_receptor_activo:
        fuente = config_vision.get("fuente_tipo", "rtsp")
        
        # 1. MODO SINTÉTICO DE PRUEBA
        if fuente == "sintetico":
            config_vision["estado_camara"] = "Objetivo Sintético (Test 3D)"
            estado_vision["conectado"] = True
            frame, synth_box = generar_frame_sintetico(time.time())
            with frame_lock:
                current_raw_frame = frame
                config_vision["_synth_box"] = synth_box
            frames_capturados += 1
            if time.time() - ultimo_calculo_fps >= 1.0:
                config_vision["fps_camara"] = round(frames_capturados / (time.time() - ultimo_calculo_fps), 1)
                frames_capturados = 0
                ultimo_calculo_fps = time.time()
            time.sleep(0.033)
            continue

        # 2. MODO SIMULACIÓN UNREAL ENGINE (AirSim RPC o RTSP)
        if fuente == "unreal":
            unreal_conectado = False
            
            # Intento A: AirSim RPC Nativo (puerto 41451)
            if HAY_AIRSIM:
                try:
                    # Mostrar frame de diagnóstico MIENTRAS conecta (no bloquear stream)
                    config_vision["estado_camara"] = "Conectando a AirSim RPC (puerto 41451)..."
                    diag = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.rectangle(diag, (10, 10), (630, 470), (168, 85, 247), 2)
                    cv2.putText(diag, "CONECTANDO A UNREAL ENGINE...", (70, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (168, 85, 247), 2, cv2.LINE_AA)
                    cv2.putText(diag, "AirSim RPC en 127.0.0.1:41451", (100, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(diag, "Presiona PLAY (Alt+P) en Unreal si esta detenido.", (50, 170),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 242, 254), 1, cv2.LINE_AA)
                    with frame_lock:
                        current_raw_frame = diag
                        config_vision["_synth_box"] = None
                    
                    client_as = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=1.0)
                    if not client_as.ping():
                        raise RuntimeError("AirSim ping fallo")
                    config_vision["estado_camara"] = "Conectado a Unreal Engine (AirSim RPC)"
                    estado_vision["conectado"] = True
                    unreal_conectado = True
                    print("[AirSim Cámara] ✅ Conectado exitosamente a Unreal Engine (puerto 41451)")
                    # Comandos de rendimiento y calidad de render UE
                    for cmd in ['t.MaxFPS 60', 'r.VSync 0', 't.IdleWhenNotForeground 0',
                                'r.Streaming.PoolSize 2048', 'r.Streaming.MipBias 0',
                                'r.Streaming.HiddenPrimitiveScale 1',
                                'sg.PostProcessQuality 4',
                                'r.PostProcessAAQuality 6',
                                'r.DefaultFeature.AntiAliasing 2',
                                'r.TemporalAACurrentFrameWeight 0.2',
                                'r.SceneColorFringeQuality 0',
                                'r.Tonemapper.Quality 5']:
                        try:
                            client_as.simRunConsoleCommand(cmd)
                        except Exception:
                            pass
                    # Distancias de renderizado UE: eliminar desaparición de terreno al ascender
                    for draw_cmd in [
                        'r.ViewDistanceScale 4',
                        'r.LightMaxDrawDistanceScale 4',
                        'foliage.LODDistanceScale 4',
                        'r.StaticMeshLODDistanceScale 4',
                        'r.SkeletalMeshLODBias -2',
                        'r.ForceLOD -1',
                        'r.LandscapeLODBias -2',
                        'wp.Runtime.UpdateStreaming 1',
                    ]:
                        try:
                            client_as.simRunConsoleCommand(draw_cmd)
                        except Exception:
                            pass
                    
                    consecutivos_err = 0
                    while config_vision.get("fuente_tipo") == "unreal" and hilo_receptor_activo:
                        try:
                            cam_sel = config_vision.get("camara_activa", "front")
                            # Usar bottom_cam dedicada para vista cenital (FOV 70°, 1280x1280)
                            # Usar front_cam como gimbal para vista frontal e inclinada
                            if cam_sel == "bottom":
                                cam_req = "bottom_cam"
                            else:
                                cam_req = "front_cam"
                            raw = None
                            try:
                                raw = client_as.client.call('simGetImages', [
                                    airsim.ImageRequest(cam_req, airsim.ImageType.Scene, False, False)
                                ], "PX4")
                            except Exception:
                                # Fallback a cámara 0
                                raw = client_as.client.call('simGetImages', [
                                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                                ], "PX4")
                            if not raw:
                                raw = client_as.client.call('simGetImages', [
                                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                                ], "PX4")
                            if raw and len(raw) > 0:
                                resp = airsim.ImageResponse.from_msgpack(raw[0])
                                if len(resp.image_data_uint8) > 0:
                                    img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
                                    frame_as = img1d.reshape(resp.height, resp.width, 3)
                                    frame_as = cv2.cvtColor(frame_as, cv2.COLOR_RGB2BGR)
                                    
                                    with frame_lock:
                                        current_raw_frame = frame_as
                                        config_vision["_synth_box"] = None
                                        config_vision["resolucion"] = f"{resp.width}x{resp.height}"
                                    frames_capturados += 1
                                    consecutivos_err = 0
                                else:
                                    time.sleep(0.01)
                            else:
                                time.sleep(0.01)
                        except Exception as e_im:
                            consecutivos_err += 1
                            if consecutivos_err >= 3:
                                print(f"[AirSim Cámara] Desconexión detectada en RPC ({e_im}). Reiniciando cliente...")
                                try:
                                    client_as.client.close()
                                except Exception:
                                    pass
                                break # Salir del while interno para reconectar client_as de inmediato
                            time.sleep(0.05)

                        if time.time() - ultimo_calculo_fps >= 1.0:
                            config_vision["fps_camara"] = round(frames_capturados / (time.time() - ultimo_calculo_fps), 1)
                            frames_capturados = 0
                            ultimo_calculo_fps = time.time()
                        time.sleep(0.005)
                except Exception as e_as:
                    unreal_conectado = False
                    time.sleep(1.0)


            # Intento B: Si AirSim RPC no respondió, probar RTSP secundario solo si fue configurado explícitamente
            if not unreal_conectado and config_vision.get("fuente_tipo") == "unreal":
                uri_unreal = config_vision.get("unreal_url", "")
                if uri_unreal.startswith("rtsp://") and uri_unreal != "rtsp://127.0.0.1:8554/live":
                    cap_u = cv2.VideoCapture(uri_unreal, cv2.CAP_FFMPEG)
                    cap_u.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    if cap_u.isOpened():
                        config_vision["estado_camara"] = "Conectado a Unreal (RTSP Stream)"
                        estado_vision["conectado"] = True
                        while cap_u.isOpened() and config_vision.get("fuente_tipo") == "unreal" and hilo_receptor_activo:
                            ret, frame = cap_u.read()
                            if not ret: break
                            with frame_lock:
                                current_raw_frame = frame.copy()
                                config_vision["_synth_box"] = None
                            frames_capturados += 1
                            if time.time() - ultimo_calculo_fps >= 1.0:
                                config_vision["fps_camara"] = round(frames_capturados / (time.time() - ultimo_calculo_fps), 1)
                                frames_capturados = 0
                                ultimo_calculo_fps = time.time()
                            time.sleep(0.01)
                        cap_u.release()
                else:
                    # Pantalla de diagnóstico táctica para Unreal
                    config_vision["estado_camara"] = "Esperando Unreal Engine (Play Alt+P)..."
                    estado_vision["conectado"] = False
                    diag_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.rectangle(diag_frame, (10, 10), (630, 470), (168, 85, 247), 2)
                    cv2.putText(diag_frame, "SIMULADOR UNREAL ENGINE (AIRSIM)", (50, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (168, 85, 247), 2, cv2.LINE_AA)
                    cv2.putText(diag_frame, "Estado: Conectando a AirSim RPC en 127.0.0.1:41451...", (50, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(diag_frame, "Presiona PLAY (Alt+P) en Unreal Editor si esta detenido.", (50, 170),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 242, 254), 1, cv2.LINE_AA)
                    
                    with frame_lock:
                        current_raw_frame = diag_frame
                        config_vision["_synth_box"] = None
                    time.sleep(0.5)
            continue

        # 3. MODO RTSP DRON REAL (Jetson Nano DevKit) O WEBCAM
        if fuente == "rtsp":
            uri = config_vision.get("rtsp_url", "rtsp://192.168.14.7:8554/cam0")
            tipo_txt = "Cámara Jetson Nano (RTSP)"
            cap = cv2.VideoCapture(uri, cv2.CAP_FFMPEG)
        else:
            uri = int(config_vision.get("webcam_idx", 0))
            tipo_txt = f"Webcam USB {uri}"
            cap = cv2.VideoCapture(uri)

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not cap.isOpened():
            config_vision["estado_camara"] = f"Buscando {tipo_txt} ({uri})..."
            estado_vision["conectado"] = False
            time.sleep(1.5)
            continue
            
        config_vision["estado_camara"] = f"Conectado a {tipo_txt}"
        estado_vision["conectado"] = True
        print(f"[Cámara] ✅ Flujo activo establecido con {tipo_txt} ({uri})")
        
        while cap.isOpened() and config_vision.get("fuente_tipo") == fuente and hilo_receptor_activo:
            ret, frame = cap.read()
            if not ret or frame is None:
                config_vision["estado_camara"] = f"Cuadro perdido ({tipo_txt}). Reconectando..."
                estado_vision["conectado"] = False
                break
            
            # Si es la cámara del Jetson y está invertida, voltear 180°
            if config_vision.get("flip_camara", True) and fuente == "rtsp":
                frame = cv2.flip(frame, -1)

            with frame_lock:
                current_raw_frame = frame.copy()
                config_vision["_synth_box"] = None
                config_vision["resolucion"] = f"{frame.shape[1]}x{frame.shape[0]}"
                
            frames_capturados += 1
            if time.time() - ultimo_calculo_fps >= 1.0:
                config_vision["fps_camara"] = round(frames_capturados / (time.time() - ultimo_calculo_fps), 1)
                frames_capturados = 0
                ultimo_calculo_fps = time.time()
                
            time.sleep(0.005)
            
        cap.release()
        time.sleep(0.5)

# ==============================================================
# HILO DE INFERENCIA YOLOv8 & OVERLAY TÁCTICO DJI
# ==============================================================
def dibujar_overlay_tactico(frame, dets, target_box=None, pid_e=None, pid_o=None, modo="OFF"):
    """Dibuja retículas tácticas DJI, brackets de objetivo y HUD de vuelo"""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    
    # 1. Cruz central de puntería táctica
    color_reticula = (0, 242, 254)
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), color_reticula, 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), color_reticula, 1)
    cv2.circle(frame, (cx, cy), 6, color_reticula, 1)
    
    # 2. Brackets de esquina DJI para cada detección
    for d in dets:
        x1, y1, x2, y2, label, conf, tid = d
        es_target = (target_box is not None and d == target_box)
        
        if es_target:
            color = (0, 242, 254) # Cyan brillante para objetivo fijado
            grosor = 2
        elif label == "DRON":
            color = (0, 0, 255)   # Rojo para drones no fijados
            grosor = 2
        else:
            color = (0, 255, 120) # Verde para personas/otros
            grosor = 1
            
        # Brackets en 4 esquinas
        length = min(22, max(8, int((x2 - x1) * 0.2)))
        # Top-left
        cv2.line(frame, (x1, y1), (x1 + length, y1), color, grosor)
        cv2.line(frame, (x1, y1), (x1, y1 + length), color, grosor)
        # Top-right
        cv2.line(frame, (x2, y1), (x2 - length, y1), color, grosor)
        cv2.line(frame, (x2, y1), (x2, y1 + length), color, grosor)
        # Bottom-left
        cv2.line(frame, (x1, y2), (x1 + length, y2), color, grosor)
        cv2.line(frame, (x1, y2), (x1, y2 - length), color, grosor)
        # Bottom-right
        cv2.line(frame, (x2, y2), (x2 - length, y2), color, grosor)
        cv2.line(frame, (x2, y2), (x2, y2 - length), color, grosor)
        
        # Centro de masa del objeto
        bx_c, by_c = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(frame, (bx_c, by_c), 3, color, -1)
        
        # Etiqueta con Track ID
        id_str = f" #{tid}" if tid >= 0 else ""
        tag = f"{label}{id_str} {conf:.2f}"
        cv2.putText(frame, tag, (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        
        # Si es el objetivo fijado, trazar vector hacia el centro
        if es_target:
            cv2.line(frame, (cx, cy), (bx_c, by_c), (0, 242, 254), 1, cv2.LINE_AA)
            cv2.rectangle(frame, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), (0, 242, 254), 1)

    # 3. Badge superior de modo de seguimiento
    modo_color = (148, 163, 184)
    if modo == "ACTIVE_TRACK":
        modo_color = (0, 0, 255)
    elif modo == "SIM":
        modo_color = (0, 242, 254)
    cv2.rectangle(frame, (w - 170, 10), (w - 10, 36), (15, 23, 42), -1)
    cv2.rectangle(frame, (w - 170, 10), (w - 10, 36), modo_color, 1)
    cv2.putText(frame, f"TRACK: {modo}", (w - 158, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, modo_color, 1, cv2.LINE_AA)

    # 4. Telemetría de control PID superpuesta en la parte inferior
    if pid_e is not None and pid_o is not None and modo != "OFF":
        hud_txt = f"Ex:{pid_e['ex']:+.2f} Ey:{pid_e['ey']:+.2f} | YawR:{pid_o['yaw_rate']:+.2f}r/s Vx:{pid_o['vx']:+.2f}m/s Vz:{pid_o['vz']:+.2f}m/s"
        cv2.rectangle(frame, (10, h - 32), (w - 10, h - 8), (6, 11, 22), -1)
        cv2.rectangle(frame, (10, h - 32), (w - 10, h - 8), (30, 58, 95), 1)
        cv2.putText(frame, hud_txt, (18, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 242, 254), 1, cv2.LINE_AA)
        
    return frame

def yolo_inferencia_worker():
    """Hilo de ejecución continua para detección YOLOv8 y cálculo PID"""
    global current_processed_frame, estado_vision, config_vision, pid_out, pid_err
    global model_drone_inst, model_stock_inst, lost_start_time, last_alert_time
    global current_encoded_jpeg, current_frame_id, stream_cond
    
    print("[YOLO] Inicializando modelos de visión por computadora...")
    if HAY_ULTRALYTICS:
        try:
            if os.path.exists(DRONE_MODEL_PATH_CUSTOM):
                model_drone_inst = YOLO(DRONE_MODEL_PATH_CUSTOM)
                print(f"[YOLO] Modelo custom cargado: {DRONE_MODEL_PATH_CUSTOM}")
            else:
                model_drone_inst = YOLO(STOCK_MODEL_PATH)
                print(f"[YOLO] Modelo stock asignado a drones: {STOCK_MODEL_PATH}")
        except Exception as e:
            print(f"[YOLO] Error al cargar modelo custom: {e}")
            
        try:
            model_stock_inst = YOLO(STOCK_MODEL_PATH)
            print(f"[YOLO] Modelo stock cargado: {STOCK_MODEL_PATH}")
        except Exception as e:
            print(f"[YOLO] Error al cargar modelo stock: {e}")
            
    device_yolo = 0 if (HAY_TORCH and torch is not None and torch.cuda.is_available()) else "cpu"
    print(f"[YOLO] Dispositivo de inferencia seleccionado: {device_yolo}")

    ultimo_tiempo_fps = time.time()
    frames_ia = 0

    while hilo_receptor_activo:
        with frame_lock:
            if current_raw_frame is None:
                time.sleep(0.02)
                continue
            frame = current_raw_frame.copy()
            synth_box = config_vision.get("_synth_box")

        h, w = frame.shape[:2]
        image_pid_inst.set_dimensions(w, h)
        
        conf = float(config_vision.get("confianza", 0.30))
        modelo_sel = config_vision.get("modelo_activo", "drone")
        dets = []
        dron_visto = False
        
        # 1. Caso de objetivo sintético (prioridad de validación sin cámara)
        if synth_box is not None:
            x1, y1, x2, y2, label, box_conf, tid = synth_box
            dets.append((x1, y1, x2, y2, label, box_conf, tid))
            dron_visto = True
        elif HAY_ULTRALYTICS:
            # 2. Inferencia real con YOLO
            try:
                # Modelo drone
                if modelo_sel in ("drone", "both") and model_drone_inst is not None:
                    res_drone = model_drone_inst.track(frame, conf=conf, persist=True, tracker="bytetrack.yaml", imgsz=480, device=device_yolo, verbose=False)
                    for r in res_drone:
                        if r.boxes is not None:
                            for b in r.boxes:
                                x1, y1, x2, y2 = b.xyxy[0].int().tolist()
                                c = float(b.conf[0])
                                tid = int(b.id[0]) if b.id is not None else -1
                                cls_id = int(b.cls[0])
                                nombre = model_drone_inst.names.get(cls_id, "DRON").upper()
                                label = "DRON" if "DRONE" in nombre or nombre == "DRON" or cls_id == 0 else nombre
                                dets.append((x1, y1, x2, y2, label, c, tid))
                                if label == "DRON":
                                    dron_visto = True
                                    
                # Modelo personas/stock
                if modelo_sel in ("person", "both") and model_stock_inst is not None:
                    res_stock = model_stock_inst.track(frame, conf=conf, classes=[0], persist=True, tracker="bytetrack.yaml", imgsz=480, device=device_yolo, verbose=False)
                    for r in res_stock:
                        if r.boxes is not None:
                            for b in r.boxes:
                                x1, y1, x2, y2 = b.xyxy[0].int().tolist()
                                c = float(b.conf[0])
                                tid = int(b.id[0]) if b.id is not None else -1
                                dets.append((x1, y1, x2, y2, "PERSONA", c, tid))
            except Exception as e_inf:
                pass

        # 3. Selección de objetivo principal (mayor área)
        target_obj = None
        best_area = 0
        for d in dets:
            x1, y1, x2, y2 = d[0], d[1], d[2], d[3]
            a = (x2 - x1) * (y2 - y1)
            if a > best_area:
                best_area = a
                target_obj = d

        # 4. Actualización del PID
        modo_seg = config_vision.get("modo_seguimiento", "OFF")
        if target_obj is not None:
            errs, outs = image_pid_inst.compute(target_obj)
            pid_err["ex"], pid_err["ey"], pid_err["earea"] = errs
            pid_out["yaw_rate"], pid_out["vx"], pid_out["vy"], pid_out["vz"] = outs
            lost_start_time = None
        else:
            if lost_start_time is None:
                lost_start_time = time.time()
            if time.time() - lost_start_time > 3.0:
                pid_out["yaw_rate"] = 0.0
                pid_out["vx"] = 0.0
                pid_out["vy"] = 0.0
                pid_out["vz"] = 0.0

        # 5. Renderizado del frame táctico
        frame_drawn = dibujar_overlay_tactico(frame, dets, target_box=target_obj, pid_e=pid_err, pid_o=pid_out, modo=modo_seg)
        
        with vision_lock:
            current_processed_frame = frame_drawn
            estado_vision["detecciones"] = [
                {"box": [d[0], d[1], d[2], d[3]], "label": d[4], "conf": round(d[5], 2), "id": d[6]}
                for d in dets
            ]
            estado_vision["conteo_drones"] = sum(1 for d in dets if d[4] == "DRON")
            estado_vision["conteo_personas"] = sum(1 for d in dets if d[4] == "PERSONA")
            estado_vision["objetivo_fijado"] = (target_obj is not None)
            estado_vision["target_id"] = target_obj[6] if target_obj is not None else None
            estado_vision["target_bbox"] = [target_obj[0], target_obj[1], target_obj[2], target_obj[3]] if target_obj is not None else None

        # 5b. Pipeline de transmisión web ultraliviana (Zero-Lag Drop-Frame HD)
        try:
            h_d, w_d = frame_drawn.shape[:2]
            # Mantener resolución nativa HD (hasta 1280px de ancho) sin recortar texturas
            if w_d > 1280:
                target_w = 1280
                target_h = int(h_d * (1280.0 / w_d))
                frame_stream = cv2.resize(frame_drawn, (target_w, target_h), interpolation=cv2.INTER_AREA)
            else:
                frame_stream = frame_drawn

            # Compresión JPEG al 88% para máxima nitidez de texturas y contornos sin artefactos
            ret_enc, buf_enc = cv2.imencode('.jpg', frame_stream, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if ret_enc:
                jpeg_bytes = buf_enc.tobytes()
                with stream_cond:
                    current_encoded_jpeg = jpeg_bytes
                    current_frame_id += 1
                    stream_cond.notify_all()
        except Exception:
            pass

        # 6. Alertas audibles
        if dron_visto and config_vision.get("alertas_activas", True):
            if time.time() - last_alert_time > ALERT_COOLDOWN:
                last_alert_time = time.time()
                estado_vision["alerta_activa"] = True
                estado_vision["mensaje_alerta"] = "Dron detectado"
        else:
            if time.time() - last_alert_time > 1.5:
                estado_vision["alerta_activa"] = False

        frames_ia += 1
        if time.time() - ultimo_tiempo_fps >= 1.0:
            config_vision["fps_ia"] = round(frames_ia / (time.time() - ultimo_tiempo_fps), 1)
            frames_ia = 0
            ultimo_tiempo_fps = time.time()
            
        time.sleep(0.005)

# ==============================================================
# HILO DE CONTROL DE SEGUIMIENTO ACTIVO MAVLINK (OFFBOARD / GUIDED)
# ==============================================================
def enviar_velocidad_ned(vx, vy, vz, yaw_rate):
    """Envía consignas de velocidad NED y tasa de guiñada al autopiloto"""
    global conexion
    if conexion is None:
        return
    with conexion_lock:
        try:
            target_sys = getattr(conexion, 'target_system', 1) or 1
            target_comp = getattr(conexion, 'target_component', 1) or 1
            
            type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
            
            conexion.mav.set_position_target_local_ned_send(
                0,                                   # time_boot_ms
                target_sys,
                target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Marco NED
                type_mask,
                0, 0, 0,                             # Posición (ignorada)
                float(vx), float(vy), float(vz),     # Velocidad m/s
                0, 0, 0,                             # Aceleración (ignorada)
                float(yaw_rate), 0.0                 # yaw_rate (rad/s), yaw (ignorado)
            )
        except Exception:
            pass

def body_to_ned(vx_body, vy_body, yaw_rad):
    """Convierte velocidades de marco cuerpo a marco NED usando el rumbo actual"""
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    return vx_body * cos_y - vy_body * sin_y, vx_body * sin_y + vy_body * cos_y

def pid_seguimiento_worker():
    """Bucle periódico a 25 Hz para aplicar control de seguimiento en modo ACTIVE_TRACK"""
    global config_vision, estado_vision, pid_out, lost_start_time
    WATCHDOG_LOST_S = 3.0
    
    while hilo_receptor_activo:
        modo_seg = config_vision.get("modo_seguimiento", "OFF")
        
        if modo_seg == "ACTIVE_TRACK":
            esta_armado = telemetria_actual["sistema"]["armado"]
            modo_vuelo = str(telemetria_actual["sistema"]["modo_vuelo"]).upper()
            es_guiado_offboard = ("OFFBOARD" in modo_vuelo or "GUIDED" in modo_vuelo or "POSCTL" in modo_vuelo)
            
            with vision_lock:
                hay_objetivo = estado_vision["objetivo_fijado"]
                lost_t = lost_start_time
                y_rate = pid_out["yaw_rate"]
                vx_b = pid_out["vx"]
                vy_b = pid_out["vy"]
                vz_b = pid_out["vz"]

            yaw_actual = telemetria_actual["actitud"]["yaw"]
            
            # Watchdog de seguridad si se pierde el objetivo
            if not hay_objetivo and lost_t is not None:
                if (time.time() - lost_t) > WATCHDOG_LOST_S:
                    vx_b, vy_b, vz_b, y_rate = 0.0, 0.0, 0.0, 0.0
            
            if conexion is not None and esta_armado and es_guiado_offboard:
                vx_ned, vy_ned = body_to_ned(vx_b, vy_b, yaw_actual)
                enviar_velocidad_ned(vx_ned, vy_ned, vz_b, y_rate)
                
        time.sleep(0.04)

# ==============================================================
# DETECCIÓN DE PUERTOS SERIALES Y ENDPOINTS SITL / UNREAL
# ==============================================================
def listar_puertos():
    """Detecta puertos COM disponibles en Windows, puertos tty en Linux y SITL/Unreal"""
    puertos = [
        {"id": "AIRSIM_RPC", "nombre": "🎮 Simulación Unreal Engine (AirSim RPC 41451)", "tipo": "rpc"},
        {"id": "udpin:0.0.0.0:14550", "nombre": "🎮 Simulación PX4 SITL (udpin:0.0.0.0:14550)", "tipo": "udp"}
    ]
    
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            desc = f"{p.device} ({p.description})" if p.description else p.device
            puertos.append({"id": p.device, "nombre": desc, "tipo": "serial"})
    except Exception:
        pass

    if os.name == 'nt':
        ids_existentes = {p["id"] for p in puertos}
        try:
            import serial
            for i in range(1, 25):
                port_name = f"COM{i}"
                if port_name not in ids_existentes:
                    try:
                        s = serial.Serial(port_name)
                        s.close()
                        puertos.append({"id": port_name, "nombre": f"{port_name} (Detectado en escaneo)", "tipo": "serial"})
                    except Exception:
                        pass
        except Exception:
            pass

    puertos_fijos = [
        {"id": "AIRSIM_RPC", "nombre": "🎮 Simulación Unreal Engine (AirSim RPC 41451)", "tipo": "rpc"},
        {"id": "udpin:0.0.0.0:14550", "nombre": "🎮 Simulación PX4 SITL (udpin:0.0.0.0:14550)", "tipo": "udp"},
        {"id": "COM1", "nombre": "COM1 (Puerto Serie)", "tipo": "serial"},
        {"id": "COM2", "nombre": "COM2", "tipo": "serial"},
        {"id": "COM3", "nombre": "COM3 (Pixhawk / USB Serial)", "tipo": "serial"},
        {"id": "COM4", "nombre": "COM4 (Radio Telemetría SiK)", "tipo": "serial"},
        {"id": "COM5", "nombre": "COM5", "tipo": "serial"},
        {"id": "COM6", "nombre": "COM6", "tipo": "serial"},
        {"id": "COM7", "nombre": "COM7 (Pixhawk USB / Telemetría)", "tipo": "serial"},
        {"id": "COM8", "nombre": "COM8", "tipo": "serial"},
        {"id": "COM9", "nombre": "COM9", "tipo": "serial"},
        {"id": "COM10", "nombre": "COM10", "tipo": "serial"},
        {"id": "COM11", "nombre": "COM11 (Antena Telemetría USB)", "tipo": "serial"},
        {"id": "COM12", "nombre": "COM12", "tipo": "serial"},
        {"id": "/dev/ttyUSB0", "nombre": "/dev/ttyUSB0 (Radio SiK Linux)", "tipo": "serial"},
        {"id": "/dev/ttyACM0", "nombre": "/dev/ttyACM0 (Pixhawk Linux)", "tipo": "serial"},
        {"id": "udp:127.0.0.1:14550", "nombre": "🎮 UDP 127.0.0.1:14550 (PX4 SITL / Unreal Engine)", "tipo": "udp"},
        {"id": "udp:127.0.0.1:14540", "nombre": "🎮 UDP 127.0.0.1:14540 (PX4 SITL Offboard / Unreal)", "tipo": "udp"},
        {"id": "udp:0.0.0.0:14550", "nombre": "UDP 0.0.0.0:14550 (Escucha GCS / Wi-Fi)", "tipo": "udp"},
        {"id": "tcp:127.0.0.1:5760", "nombre": "TCP 127.0.0.1:5760 (SITL TCP)", "tipo": "tcp"}
    ]
    
    ids_existentes = {p["id"] for p in puertos}
    for pf in puertos_fijos:
        if pf["id"] not in ids_existentes:
            puertos.append(pf)
            
    return puertos

# ==============================================================
# DECODIFICACIÓN DE MODOS DE VUELO
# ==============================================================
def decodificar_modo_vuelo(msg):
    """Decodifica el modo de vuelo para PX4 y ArduPilot desde el mensaje HEARTBEAT"""
    custom_mode = msg.custom_mode
    autopilot = msg.autopilot
    
    if autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        
        px4_modes = {
            1: "MANUAL",
            2: "ALTCTL (Alt. Hold)",
            3: "POSCTL (Pos. Hold)",
            4: "AUTO",
            5: "ACRO",
            6: "OFFBOARD",
            7: "STABILIZED",
            8: "RATTITUDE"
        }
        
        px4_auto_submodes = {
            1: "AUTO - READY",
            2: "AUTO - TAKEOFF",
            3: "AUTO - LOITER",
            4: "AUTO - MISSION",
            5: "AUTO - RTL",
            6: "AUTO - LAND",
            7: "AUTO - RTGS",
            8: "AUTO - FOLLOW ME",
            9: "AUTO - PRECLAND"
        }
        
        if main_mode == 4 and sub_mode in px4_auto_submodes:
            return px4_auto_submodes[sub_mode]
        return px4_modes.get(main_mode, f"PX4 ({main_mode})")
        
    elif autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        ardu_copter_modes = {
            0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
            5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
            13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
            17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS"
        }
        return ardu_copter_modes.get(custom_mode, f"Ardu ({custom_mode})")
        
    return f"Modo {custom_mode}"

# ==============================================================
# SOLICITUD ACTIVA DE STREAMS MAVLINK (STREAM SCHEDULER)
# ==============================================================
def configurar_frecuencias_mavlink(conn):
    """Solicita activamente streams a alta frecuencia al Pixhawk o PX4 SITL"""
    if conn is None:
        return
    try:
        target_sys = conn.target_system if conn.target_system > 0 else 1
        target_comp = conn.target_component if conn.target_component > 0 else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 20)
        ]
        
        for stream_id, rate in streams:
            conn.mav.request_data_stream_send(
                target_sys, target_comp,
                stream_id, rate, 1
            )
            time.sleep(0.01)

        mensajes_clave = [
            (30, 50000),   # ATTITUDE (20 Hz)
            (74, 100000),  # VFR_HUD (10 Hz)
            (33, 100000),  # GLOBAL_POSITION_INT (10 Hz)
            (105, 50000),  # HIGHRES_IMU (20 Hz)
            (241, 100000), # VIBRATION (10 Hz)
            (193, 200000), # EKF_STATUS_REPORT (5 Hz)
            (36, 100000),  # SERVO_OUTPUT_RAW (10 Hz)
            (65, 200000)   # RC_CHANNELS (5 Hz)
        ]
        
        for msg_id, interval_us in mensajes_clave:
            conn.mav.command_long_send(
                target_sys, target_comp,
                MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
            time.sleep(0.01)
            
        print("[MAVLink] Streams de alta frecuencia solicitados exitosamente (20Hz/10Hz).")
    except Exception as e:
        print(f"[MAVLink] Advertencia configurando streams: {e}")

# ==============================================================
# COMANDOS DE VUELO Y SEGURIDAD MAVLINK
# ==============================================================
def cambiar_modo_vuelo_cmd(modo_solicitado):
    """Envía comando MAVLink para cambiar el modo de vuelo"""
    global conexion, config_conexion
    modo = modo_solicitado.upper().strip()
    
    if config_conexion.get("puerto") == "AIRSIM_RPC":
        telemetria_actual["sistema"]["modo_vuelo"] = modo
        return True, f"Modo de simulación cambiado a: {modo}"
        
    if conexion is None:
        return False, "Pixhawk no conectado."
    
    target_sys = conexion.target_system if conexion.target_system > 0 else 1
    target_comp = conexion.target_component if conexion.target_component > 0 else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

    with conexion_lock:
        try:
            es_px4 = telemetria_actual["sistema"]["tipo_autopilot"] == "PX4"

            if es_px4:
                px4_map = {
                    "MANUAL": (1, 0),
                    "ALTCTL": (2, 0),
                    "ALT_HOLD": (2, 0),
                    "POSCTL": (3, 0),
                    "POS_HOLD": (3, 0),
                    "ACRO": (5, 0),
                    "OFFBOARD": (6, 0),
                    "STABILIZED": (7, 0),
                    "AUTO_RTL": (4, 5),
                    "RTL": (4, 5),
                    "AUTO_LAND": (4, 6),
                    "LAND": (4, 6),
                    "AUTO": (4, 4),
                    "MISSION": (4, 4),
                    "AUTO_MISSION": (4, 4),
                    "AUTO_LOITER": (4, 3),
                    "LOITER": (4, 3)
                }
                
                if modo in px4_map:
                    main_mode, sub_mode = px4_map[modo]
                    custom_mode = (main_mode << 16) | (sub_mode << 24)
                    conexion.mav.command_long_send(
                        target_sys, target_comp,
                        MAV_CMD_DO_SET_MODE,
                        0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        custom_mode,
                        0, 0, 0, 0, 0
                    )
                    return True, f"Modo PX4 cambiado a: {modo}"
            else:
                ardu_map = {
                    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
                    "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9, "POSHOLD": 16
                }
                if modo in ardu_map:
                    custom_mode = ardu_map[modo]
                    conexion.mav.command_long_send(
                        target_sys, target_comp,
                        MAV_CMD_DO_SET_MODE,
                        0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        custom_mode,
                        0, 0, 0, 0, 0
                    )
                    return True, f"Modo ArduPilot cambiado a: {modo}"
                    
            return False, f"Modo {modo} no reconocido para el autopiloto actual."
        except Exception as e:
            return False, f"Error al cambiar modo: {e}"

def armar_desarmar_cmd(armar=True, force=False):
    """Envía comando MAVLink para armar o desarmar los motores con soporte para PX4 SITL y Pixhawk real"""
    global conexion, config_conexion
    
    conn_activa = conexion
    cerrar_al_terminar = False
    
    if conn_activa is None:
        try:
            conn_activa = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
            conn_activa.wait_heartbeat(timeout=1.5)
            cerrar_al_terminar = True
        except Exception:
            pass

    if conn_activa is not None:
        try:
            target_sys = getattr(conn_activa, 'target_system', 1) or 1
            target_comp = getattr(conn_activa, 'target_component', 1) or 1
            
            param1 = 1.0 if armar else 0.0
            param2 = 21196.0 if (force or armar) else 0.0
            
            conn_activa.mav.command_long_send(
                target_sys, target_comp,
                MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                param1,
                param2,
                0, 0, 0, 0, 0
            )
            telemetria_actual["sistema"]["armado"] = armar
            estado_txt = "ARMADO" if armar else "DESARMADO"
            if cerrar_al_terminar:
                time.sleep(0.1)
                try: conn_activa.close()
                except Exception: pass
            return True, f"Motores {estado_txt.lower()}s exitosamente."
        except Exception as e:
            return False, f"Error al transmitir comando de armado: {e}"
            
    telemetria_actual["sistema"]["armado"] = armar
    return True, f"Modo simulación: Motores {'armados' if armar else 'desarmados'}."

def despegar_cmd(altitud=15.0):
    """Arma el dron y comanda despegue automático a la altitud indicada (MAV_CMD_NAV_TAKEOFF)"""
    global conexion, config_conexion
    
    # 1. Armar primero los motores con override de seguridad
    armar_desarmar_cmd(armar=True, force=True)
    time.sleep(0.3)
    
    conn_activa = conexion
    cerrar_al_terminar = False
    
    if conn_activa is None:
        try:
            conn_activa = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
            conn_activa.wait_heartbeat(timeout=1.5)
            cerrar_al_terminar = True
        except Exception as e:
            return False, f"No se pudo conectar a MAVLink para despegue: {e}"

    try:
        target_sys = getattr(conn_activa, 'target_system', 1) or 1
        target_comp = getattr(conn_activa, 'target_component', 1) or 1
        
        # Enviar MAV_CMD_NAV_TAKEOFF (22)
        conn_activa.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, float('nan'), 0, 0, float(altitud)
        )
        
        telemetria_actual["sistema"]["armado"] = True
        telemetria_actual["sistema"]["modo_vuelo"] = "AUTO - TAKEOFF"
        
        if cerrar_al_terminar:
            time.sleep(0.2)
            try: conn_activa.close()
            except Exception: pass
            
        print(f"[Vuelo] 🛫 Comando de despegue enviado a altitud {altitud}m.")
        return True, f"Despegue comandado exitosamente a {altitud}m de altitud."
    except Exception as e:
        return False, f"Error al transmitir comando de despegue: {e}"

def aterrizar_cmd():
    """Comanda aterrizaje inmediato en la posición actual (MAV_CMD_NAV_LAND)"""
    global conexion
    conn_activa = conexion
    cerrar_al_terminar = False
    if conn_activa is None:
        try:
            conn_activa = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
            conn_activa.wait_heartbeat(timeout=1.5)
            cerrar_al_terminar = True
        except Exception as e:
            return False, f"No hay conexión MAVLink para aterrizaje: {e}"
            
    try:
        target_sys = getattr(conn_activa, 'target_system', 1) or 1
        target_comp = getattr(conn_activa, 'target_component', 1) or 1
        conn_activa.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, float('nan'), 0, 0, 0
        )
        telemetria_actual["sistema"]["modo_vuelo"] = "AUTO - LAND"
        if cerrar_al_terminar:
            time.sleep(0.2)
            try: conn_activa.close()
            except Exception: pass
        print("[Vuelo] 🛬 Comando de aterrizaje (LAND) enviado.")
        return True, "Comando de aterrizaje (LAND) transmitido al autopiloto."
    except Exception as e:
        return False, f"Error al transmitir aterrizaje: {e}"

def retorno_cmd():
    """Comanda retorno al punto de despegue (RTL)"""
    global conexion
    conn_activa = conexion
    cerrar_al_terminar = False
    if conn_activa is None:
        try:
            conn_activa = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
            conn_activa.wait_heartbeat(timeout=1.5)
            cerrar_al_terminar = True
        except Exception as e:
            return False, f"No hay conexión MAVLink para RTL: {e}"
            
    try:
        target_sys = getattr(conn_activa, 'target_system', 1) or 1
        target_comp = getattr(conn_activa, 'target_component', 1) or 1
        conn_activa.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        telemetria_actual["sistema"]["modo_vuelo"] = "AUTO - RTL"
        if cerrar_al_terminar:
            time.sleep(0.2)
            try: conn_activa.close()
            except Exception: pass
        print("[Vuelo] 🏠 Comando de retorno a casa (RTL) enviado.")
        return True, "Comando de retorno (RTL) transmitido al autopiloto."
    except Exception as e:
        return False, f"Error al transmitir RTL: {e}"

def enviar_comando_motor(motor_id, valor_porcentaje, timeout_s=3.0):
    """Prueba de motor MAVLink compatible con PX4 (310) y ArduPilot (209)"""
    global conexion
    if conexion is None:
        return False, "Pixhawk no conectado."

    try:
        motor_id = int(motor_id)
        valor_porcentaje = float(valor_porcentaje)
    except (ValueError, TypeError):
        return False, "Parámetros de motor inválidos."

    with conexion_lock:
        try:
            target_system = conexion.target_system if conexion.target_system > 0 else 1
            target_component = conexion.target_component if conexion.target_component > 0 else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

            if valor_porcentaje <= 0:
                conexion.mav.command_long_send(
                    target_system, target_component,
                    MAV_CMD_ACTUATOR_TEST,
                    0, -1.0, 0.0, 0, 0, motor_id, 0, 0
                )
                conexion.mav.command_long_send(
                    target_system, target_component,
                    MAV_CMD_DO_MOTOR_TEST,
                    0, motor_id, MOTOR_TEST_THROTTLE_PERCENT, 0, 0, 0, 0, 0
                )
                return True, f"Motor {motor_id} detenido (0%)."
            else:
                val_px4 = min(max(valor_porcentaje / 100.0, 0.0), 1.0)
                conexion.mav.command_long_send(
                    target_system, target_component,
                    MAV_CMD_ACTUATOR_TEST,
                    0, val_px4, float(timeout_s), 0, 0, motor_id, 0, 0
                )
                conexion.mav.command_long_send(
                    target_system, target_component,
                    MAV_CMD_DO_MOTOR_TEST,
                    0, motor_id, MOTOR_TEST_THROTTLE_PERCENT, float(valor_porcentaje), float(timeout_s), 0, 0, 0
                )
                return True, f"Motor {motor_id} activado al {valor_porcentaje}% ({timeout_s}s)."
        except Exception as e:
            return False, f"Error al transmitir comando MAVLink: {e}"

# ==============================================================
# GESTIÓN DE PARÁMETROS MAVLINK
# ==============================================================
def leer_parametro_mavlink(param_nombre):
    global conexion
    if conexion is None:
        return False, "Pixhawk no conectado."
    with conexion_lock:
        try:
            target_sys = conexion.target_system if conexion.target_system > 0 else 1
            target_comp = conexion.target_component if conexion.target_component > 0 else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            param_bytes = param_nombre.encode('ascii')[:16]
            conexion.mav.param_request_read_send(target_sys, target_comp, param_bytes, -1)
            return True, f"Solicitud de parámetro {param_nombre} enviada."
        except Exception as e:
            return False, str(e)

def escribir_parametro_mavlink(param_nombre, valor):
    global conexion
    if conexion is None:
        return False, "Pixhawk no conectado."
    try:
        val_flt = float(valor)
    except ValueError:
        return False, "Valor numérico requerido."
        
    with conexion_lock:
        try:
            target_sys = conexion.target_system if conexion.target_system > 0 else 1
            target_comp = conexion.target_component if conexion.target_component > 0 else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            param_bytes = param_nombre.encode('ascii')[:16]
            conexion.mav.param_set_send(target_sys, target_comp, param_bytes, val_flt, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            telemetria_actual["parametros"][param_nombre] = val_flt
            return True, f"Parámetro {param_nombre} establecido en {val_flt}."
        except Exception as e:
            return False, str(e)

# ==============================================================
# CONVERSIÓN Y LECTURA DE TELEMETRÍA AIRSIM (UNREAL ENGINE)
# ==============================================================
def quaternion_to_euler(q):
    if hasattr(q, 'w_val'):
        w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    elif hasattr(q, 'w'):
        w, x, y, z = q.w, q.x, q.y, q.z
    else:
        w, x, y, z = q[0], q[1], q[2], q[3]
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, roll, yaw

def airsim_telemetria_worker():
    """Hilo de telemetría directo desde Unreal Engine (AirSim RPC puerto 41451)"""
    global telemetria_actual, config_conexion, hilo_receptor_activo
    if not HAY_AIRSIM:
        return
        
    client_as_telem = None
    ultimo_exito = 0
    telem_err_count = 0
    
    while hilo_receptor_activo:
        # Prioridad absoluta a MAVLink como fuente única de verdad para evitar asincronía y saltos de muestreo
        hay_mavlink_reciente = (time.time() - telemetria_actual.get("_ultimo_mavlink_ts", 0)) < 2.0
        if not hay_mavlink_reciente:
            try:
                if client_as_telem is None:
                    client_as_telem = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2.0)
                    if not client_as_telem.ping():
                        raise RuntimeError("AirSim ping fallo")
                    print("[AirSim Telemetría] ✅ Conectado a Unreal Engine (AirSim RPC)")
                
                kin = client_as_telem.simGetGroundTruthKinematics(vehicle_name="PX4")
                pitch_rad, roll_rad, yaw_rad = quaternion_to_euler(kin.orientation)
                pos = kin.position
                try:
                    gps = client_as_telem.getGpsData(vehicle_name="PX4")
                    geo = gps.gnss.geo_point
                    if not math.isnan(geo.latitude) and abs(geo.latitude) > 1.0:
                        lat = round(geo.latitude, 7)
                        lon = round(geo.longitude, 7)
                        airsim_spawn_origin["lat"] = lat - (pos.x_val / 111139.0)
                        airsim_spawn_origin["lon"] = lon - (pos.y_val / (111139.0 * math.cos(math.radians(lat))))
                    else:
                        raise ValueError("Coordenadas GPS no válidas")
                except Exception:
                    lat = round(airsim_spawn_origin["lat"] + (pos.x_val / 111139.0), 7)
                    lon = round(airsim_spawn_origin["lon"] + (pos.y_val / (111139.0 * math.cos(math.radians(airsim_spawn_origin["lat"])))), 7)
                
                alt_rel = round(max(0.0, -pos.z_val), 2)
                alt_amsl = round(ORIGEN_PARQUE_OHIGGINS_ALT + alt_rel, 2)
                
                vel = kin.linear_velocity
                groundspeed = round(math.sqrt(vel.x_val**2 + vel.y_val**2), 2)
                
                pitch_deg = round(math.degrees(pitch_rad), 2)
                roll_deg = round(math.degrees(roll_rad), 2)
                yaw_deg = round((math.degrees(yaw_rad) + 360) % 360, 2)
                
                telemetria_actual["pitch"] = round(pitch_rad, 4)
                telemetria_actual["roll"] = round(roll_rad, 4)
                telemetria_actual["yaw"] = round(yaw_rad, 4)
                telemetria_actual["actitud"]["pitch"] = round(pitch_rad, 4)
                telemetria_actual["actitud"]["pitch_deg"] = pitch_deg
                telemetria_actual["actitud"]["roll"] = round(roll_rad, 4)
                telemetria_actual["actitud"]["roll_deg"] = roll_deg
                telemetria_actual["actitud"]["yaw"] = round(yaw_rad, 4)
                telemetria_actual["actitud"]["yaw_deg"] = yaw_deg
                
                telemetria_actual["gps"]["lat"] = lat
                telemetria_actual["gps"]["lon"] = lon
                telemetria_actual["gps"]["alt_rel"] = alt_rel
                telemetria_actual["gps"]["alt_amsl"] = round(ORIGEN_PARQUE_OHIGGINS_ALT + alt_rel, 2)
                telemetria_actual["gps"]["satellites"] = 18
                telemetria_actual["gps"]["fix_desc"] = "3D Fix (Unreal AirSim)"
                telemetria_actual["gps"]["fix_type"] = 3
                telemetria_actual["gps"]["hdop"] = 0.8
                
                telemetria_actual["vfr_hud"]["groundspeed"] = groundspeed
                telemetria_actual["vfr_hud"]["altitud_baro"] = alt_rel
                telemetria_actual["vfr_hud"]["heading"] = int(yaw_deg)
                telemetria_actual["vfr_hud"]["climb"] = round(-vel.z_val, 2)
                
                armado = (alt_rel > 0.5 or groundspeed > 0.1)
                telemetria_actual["sistema"]["armado"] = armado
                telemetria_actual["sistema"]["modo_vuelo"] = "UNREAL AIRSIM"
                telemetria_actual["sistema"]["voltaje_bateria"] = 16.2
                telemetria_actual["sistema"]["bateria_pct"] = 96
                telemetria_actual["sistema"]["voltaje_celda"] = 4.05
                
                telemetria_actual["conectado"] = True
                telemetria_actual["conexion"]["conectado"] = True
                telemetria_actual["conexion"]["estado"] = "Unreal Engine (AirSim RPC)"
                telemetria_actual["conexion"]["puerto"] = "AIRSIM_RPC"
                telemetria_actual["conexion"]["simulacion"] = True
                telemetria_actual["hz_recepcion"] = 20.0
                telemetria_actual["conexion"]["hz"] = 20.0
                ultimo_exito = time.time()
                telem_err_count = 0
                
            except Exception:
                telem_err_count += 1
                if telem_err_count >= 5:
                    client_as_telem = None
                    telem_err_count = 0
                if time.time() - ultimo_exito > 3.0:
                    if config_conexion.get("puerto") == "AIRSIM_RPC":
                        telemetria_actual["conexion"]["estado"] = "Esperando Unreal (Play Alt+P)..."
                        
        time.sleep(0.05)

# ==============================================================
# HILO RECEPTOR DE TELEMETRÍA MULTI-MENSAJE MAVLINK
# ==============================================================
def receptor_mavlink_worker():
    global grabando_telemetria, telemetria_actual, conexion, config_conexion, hilo_receptor_activo
    
    ultimo_mensaje_tiempo = time.time()
    ultimo_print_tiempo = time.time()
    ultimo_calculo_hz = time.time()
    contador_paquetes = 0
    start_time = time.time()
    
    while hilo_receptor_activo:
        try:
            if config_conexion["modo_simulacion"]:
                time.sleep(0.1)
                continue

            puerto_actual = config_conexion["puerto"]
            baud_actual = config_conexion["baud"]
            puerto_mavlink = "udpin:0.0.0.0:14550" if puerto_actual == "AIRSIM_RPC" else puerto_actual

            if conexion is None and config_conexion.get("deseado_conectar", True):
                print(f"[Telemetría] Intentando conectar a MAVLink en {puerto_mavlink}...")
                config_conexion["estado_texto"] = f"Conectando a {puerto_actual}..."
                
                try:
                    if puerto_mavlink.startswith("udp") or puerto_mavlink.startswith("tcp:"):
                        nueva_conn = mavutil.mavlink_connection(puerto_mavlink, autoreconnect=True)
                    else:
                        nueva_conn = mavutil.mavserial(puerto_mavlink, baud=baud_actual, autoreconnect=True)
                        
                    with conexion_lock:
                        conexion = nueva_conn
                        
                    start_time = time.time()
                    ultimo_mensaje_tiempo = time.time()
                    config_conexion["conectado"] = True
                    config_conexion["estado_texto"] = f"Conectado ({puerto_actual})"
                    print(f"[Telemetría] Conexión establecida en {puerto_actual}.")
                    
                    threading.Thread(target=configurar_frecuencias_mavlink, args=(nueva_conn,), daemon=True).start()
                    
                except Exception as err_conn:
                    config_conexion["conectado"] = False
                    config_conexion["estado_texto"] = f"Error: {err_conn}"
                    time.sleep(2.5)
                    continue

            with open(ARCHIVO_LOG_ACTUAL, mode='a', newline='') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow([
                        "Tiempo(s)", "Pitch(rad)", "Roll(rad)", "Yaw(rad)", 
                        "AltRel(m)", "Groundspeed(m/s)", "Voltaje(V)", "Corriente(A)", "Bateria(%)",
                        "Latitud", "Longitud", "Satellites", "ModoVuelo", "VibeX", "VibeY", "VibeZ"
                    ])
                
                while hilo_receptor_activo and conexion is not None and not config_conexion["modo_simulacion"]:
                    msg = conexion.recv_match(blocking=True, timeout=0.04)
                    
                    if msg is not None:
                        tipo = msg.get_type()
                        tiempo_actual = round(time.time() - start_time, 2)
                        contador_paquetes += 1
                        config_conexion["paquetes_totales"] += 1
                        
                        telemetria_actual["tiempo"] = tiempo_actual
                        telemetria_actual["conectado"] = True
                        telemetria_actual["_ultimo_mavlink_ts"] = time.time()
                        telemetria_actual["conexion"]["conectado"] = True
                        telemetria_actual["conexion"]["puerto"] = config_conexion["puerto"]
                        telemetria_actual["conexion"]["baud"] = config_conexion["baud"]
                        telemetria_actual["conexion"]["paquetes_recibidos"] = config_conexion["paquetes_totales"]
                        ultimo_mensaje_tiempo = time.time()

                        if time.time() - ultimo_calculo_hz >= 1.0:
                            delta_t = time.time() - ultimo_calculo_hz
                            hz = round(contador_paquetes / delta_t, 1)
                            telemetria_actual["hz_recepcion"] = hz
                            telemetria_actual["conexion"]["hz"] = hz
                            config_conexion["hz_actual"] = hz
                            contador_paquetes = 0
                            ultimo_calculo_hz = time.time()

                        if tipo == 'ATTITUDE':
                            pitch = limpiar_float(msg.pitch)
                            roll = limpiar_float(msg.roll)
                            yaw = limpiar_float(msg.yaw)
                            
                            telemetria_actual["actitud"]["pitch"] = pitch
                            telemetria_actual["actitud"]["roll"] = roll
                            telemetria_actual["actitud"]["yaw"] = yaw
                            telemetria_actual["actitud"]["pitch_deg"] = round(math.degrees(pitch), 1)
                            telemetria_actual["actitud"]["roll_deg"] = round(math.degrees(roll), 1)
                            telemetria_actual["actitud"]["yaw_deg"] = round((math.degrees(yaw) + 360) % 360, 1)
                            telemetria_actual["actitud"]["pitchspeed"] = limpiar_float(msg.pitchspeed)
                            telemetria_actual["actitud"]["rollspeed"] = limpiar_float(msg.rollspeed)
                            telemetria_actual["actitud"]["yawspeed"] = limpiar_float(msg.yawspeed)

                            telemetria_actual["pitch"] = pitch
                            telemetria_actual["roll"] = roll
                            telemetria_actual["yaw"] = yaw

                            writer.writerow([
                                tiempo_actual, pitch, roll, yaw,
                                telemetria_actual["gps"]["alt_rel"],
                                telemetria_actual["vfr_hud"]["groundspeed"],
                                telemetria_actual["sistema"]["voltaje_bateria"],
                                telemetria_actual["sistema"]["corriente_bateria"],
                                telemetria_actual["sistema"]["bateria_pct"],
                                telemetria_actual["gps"]["lat"],
                                telemetria_actual["gps"]["lon"],
                                telemetria_actual["gps"]["satellites"],
                                telemetria_actual["sistema"]["modo_vuelo"],
                                telemetria_actual["vibracion"]["vibe_x"],
                                telemetria_actual["vibracion"]["vibe_y"],
                                telemetria_actual["vibracion"]["vibe_z"]
                            ])
                            file.flush()

                        elif tipo == 'VFR_HUD':
                            telemetria_actual["vfr_hud"]["airspeed"] = limpiar_float(msg.airspeed, 0.0)
                            telemetria_actual["vfr_hud"]["groundspeed"] = limpiar_float(msg.groundspeed, 0.0)
                            telemetria_actual["vfr_hud"]["altitud_baro"] = limpiar_float(msg.alt, 0.0)
                            telemetria_actual["vfr_hud"]["climb"] = limpiar_float(msg.climb, 0.0)
                            telemetria_actual["vfr_hud"]["heading"] = int(msg.heading) if hasattr(msg, 'heading') else 0
                            telemetria_actual["vfr_hud"]["throttle"] = int(msg.throttle) if hasattr(msg, 'throttle') else 0

                        elif tipo == 'ALTITUDE':
                            telemetria_actual["gps"]["alt_amsl"] = limpiar_float(msg.altitude_amsl, 0.0)
                            telemetria_actual["gps"]["alt_rel"] = limpiar_float(msg.altitude_relative, 0.0)

                        elif tipo == 'GLOBAL_POSITION_INT':
                            if msg.lat != 0 and msg.lon != 0 and abs(msg.lat / 1e7) > 1.0:
                                telemetria_actual["gps"]["lat"] = round(msg.lat / 1e7, 7)
                                telemetria_actual["gps"]["lon"] = round(msg.lon / 1e7, 7)
                            telemetria_actual["gps"]["alt_amsl"] = round(msg.alt / 1000.0, 2)
                            telemetria_actual["gps"]["alt_rel"] = round(msg.relative_alt / 1000.0, 2)
                            telemetria_actual["gps"]["vx"] = round(msg.vx / 100.0, 2)
                            telemetria_actual["gps"]["vy"] = round(msg.vy / 100.0, 2)
                            telemetria_actual["gps"]["vz"] = round(msg.vz / 100.0, 2)

                        elif tipo == 'GPS_RAW_INT':
                            telemetria_actual["gps"]["fix_type"] = msg.fix_type
                            fix_map = {0: "Sin GPS (Interiores)", 1: "Sin Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
                            telemetria_actual["gps"]["fix_desc"] = fix_map.get(msg.fix_type, f"Fix {msg.fix_type}")
                            telemetria_actual["gps"]["satellites"] = msg.satellites_visible
                            telemetria_actual["gps"]["hdop"] = round(msg.eph / 100.0 if hasattr(msg, 'eph') else 99.9, 2)
                            if msg.fix_type >= 2 and msg.lat != 0 and msg.lon != 0 and abs(msg.lat / 1e7) > 1.0:
                                telemetria_actual["gps"]["lat"] = round(msg.lat / 1e7, 7)
                                telemetria_actual["gps"]["lon"] = round(msg.lon / 1e7, 7)

                        elif tipo == 'VIBRATION':
                            v_x = limpiar_float(msg.vibration_x)
                            v_y = limpiar_float(msg.vibration_y)
                            v_z = limpiar_float(msg.vibration_z)
                            telemetria_actual["vibracion"]["vibe_x"] = v_x
                            telemetria_actual["vibracion"]["vibe_y"] = v_y
                            telemetria_actual["vibracion"]["vibe_z"] = v_z
                            telemetria_actual["vibracion"]["clip_0"] = msg.clipping_0
                            telemetria_actual["vibracion"]["clip_1"] = msg.clipping_1
                            telemetria_actual["vibracion"]["clip_2"] = msg.clipping_2
                            
                            max_vibe = max(v_x, v_y, v_z)
                            if max_vibe < 30:
                                telemetria_actual["vibracion"]["nivel_salud"] = "Excelente (<30 m/s²)"
                            elif max_vibe < 60:
                                telemetria_actual["vibracion"]["nivel_salud"] = "Precaución (30-60 m/s²)"
                            else:
                                telemetria_actual["vibracion"]["nivel_salud"] = "Crítico (>60 m/s²)"

                        elif tipo == 'EKF_STATUS_REPORT':
                            telemetria_actual["ekf"]["flags"] = msg.flags
                            telemetria_actual["ekf"]["velocity_variance"] = limpiar_float(msg.velocity_variance)
                            telemetria_actual["ekf"]["pos_horiz_variance"] = limpiar_float(msg.pos_horiz_variance)
                            telemetria_actual["ekf"]["pos_vert_variance"] = limpiar_float(msg.pos_vert_variance)
                            telemetria_actual["ekf"]["compass_variance"] = limpiar_float(msg.compass_variance)
                            telemetria_actual["ekf"]["terrain_alt_variance"] = limpiar_float(msg.terrain_alt_variance)
                            max_var = max(msg.velocity_variance, msg.pos_horiz_variance, msg.pos_vert_variance, msg.compass_variance)
                            telemetria_actual["ekf"]["salud_ekf"] = "Óptimo" if max_var < 0.5 else ("Advertencia" if max_var < 1.0 else "Fallo EKF")

                        elif tipo == 'NAV_CONTROLLER_OUTPUT':
                            telemetria_actual["control_guiado"]["nav_pitch"] = limpiar_float(msg.nav_pitch)
                            telemetria_actual["control_guiado"]["nav_roll"] = limpiar_float(msg.nav_roll)
                            telemetria_actual["control_guiado"]["nav_bearing"] = int(msg.nav_bearing)
                            telemetria_actual["control_guiado"]["target_bearing"] = int(msg.target_bearing)
                            telemetria_actual["control_guiado"]["wp_dist"] = int(msg.wp_dist)
                            telemetria_actual["control_guiado"]["alt_error"] = limpiar_float(msg.alt_error)
                            telemetria_actual["control_guiado"]["aspd_error"] = limpiar_float(msg.aspd_error)
                            telemetria_actual["control_guiado"]["xtrack_error"] = limpiar_float(msg.xtrack_error)

                        elif tipo == 'SYS_STATUS':
                            v_bat = msg.voltage_battery
                            if v_bat == 65535 or v_bat <= 0:
                                telemetria_actual["sistema"]["voltaje_bateria"] = 5.0
                                telemetria_actual["sistema"]["voltaje_celda"] = 3.8
                            else:
                                v_tot = round(v_bat / 1000.0, 2)
                                telemetria_actual["sistema"]["voltaje_bateria"] = v_tot
                                celdas = 6 if v_tot > 18.0 else (4 if v_tot > 13.0 else (3 if v_tot > 8.0 else 1))
                                telemetria_actual["sistema"]["voltaje_celda"] = round(v_tot / celdas, 2)

                            telemetria_actual["sistema"]["corriente_bateria"] = round(msg.current_battery / 100.0, 2) if msg.current_battery >= 0 else 0.0
                            telemetria_actual["sistema"]["bateria_pct"] = msg.battery_remaining if msg.battery_remaining >= 0 else 100
                            telemetria_actual["sistema"]["carga_cpu"] = round(msg.load / 10.0, 1)
                            telemetria_actual["sistema"]["perdida_com"] = round(msg.drop_rate_comm / 100.0, 1) if hasattr(msg, 'drop_rate_comm') else 0.0

                        elif tipo == 'BATTERY_STATUS':
                            if msg.battery_remaining >= 0:
                                telemetria_actual["sistema"]["bateria_pct"] = msg.battery_remaining
                            if msg.current_battery >= 0:
                                telemetria_actual["sistema"]["corriente_bateria"] = round(msg.current_battery / 100.0, 2)
                            if hasattr(msg, 'current_consumed') and msg.current_consumed >= 0:
                                telemetria_actual["sistema"]["mah_consumidos"] = msg.current_consumed

                        elif tipo == 'HEARTBEAT':
                            telemetria_actual["sistema"]["armado"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                            telemetria_actual["sistema"]["modo_vuelo"] = decodificar_modo_vuelo(msg)
                            telemetria_actual["sistema"]["tipo_autopilot"] = "PX4" if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 else "ArduPilot"

                        elif tipo == 'SCALED_PRESSURE':
                            telemetria_actual["imu"]["presion_hpa"] = limpiar_float(msg.press_abs, 1013.2)
                            telemetria_actual["sistema"]["temperatura_placa"] = limpiar_float(msg.temperature / 100.0 if hasattr(msg, 'temperature') else 25.0, 25.0)

                        elif tipo in ('HIGHRES_IMU', 'RAW_IMU'):
                            if tipo == 'HIGHRES_IMU':
                                telemetria_actual["imu"]["acc_x"] = limpiar_float(msg.xacc)
                                telemetria_actual["imu"]["acc_y"] = limpiar_float(msg.yacc)
                                telemetria_actual["imu"]["acc_z"] = limpiar_float(msg.zacc)
                                telemetria_actual["imu"]["gyro_x"] = limpiar_float(msg.xgyro)
                                telemetria_actual["imu"]["gyro_y"] = limpiar_float(msg.ygyro)
                                telemetria_actual["imu"]["gyro_z"] = limpiar_float(msg.zgyro)
                                telemetria_actual["imu"]["mag_x"] = limpiar_float(msg.xmag)
                                telemetria_actual["imu"]["mag_y"] = limpiar_float(msg.ymag)
                                telemetria_actual["imu"]["mag_z"] = limpiar_float(msg.zmag)
                                telemetria_actual["imu"]["presion_hpa"] = limpiar_float(msg.abs_pressure, 1013.2)
                                telemetria_actual["sistema"]["temperatura_placa"] = limpiar_float(msg.temperature, 25.0)

                        elif tipo == 'SERVO_OUTPUT_RAW':
                            telemetria_actual["actuadores"]["motor1_pwm"] = msg.servo1_raw
                            telemetria_actual["actuadores"]["motor2_pwm"] = msg.servo2_raw
                            telemetria_actual["actuadores"]["motor3_pwm"] = msg.servo3_raw
                            telemetria_actual["actuadores"]["motor4_pwm"] = msg.servo4_raw
                            if hasattr(msg, 'servo5_raw'):
                                telemetria_actual["actuadores"]["motor5_pwm"] = msg.servo5_raw
                                telemetria_actual["actuadores"]["motor6_pwm"] = msg.servo6_raw
                                telemetria_actual["actuadores"]["motor7_pwm"] = msg.servo7_raw
                                telemetria_actual["actuadores"]["motor8_pwm"] = msg.servo8_raw

                        elif tipo == 'RC_CHANNELS':
                            telemetria_actual["rc"]["ch1"] = msg.chan1_raw
                            telemetria_actual["rc"]["ch2"] = msg.chan2_raw
                            telemetria_actual["rc"]["ch3"] = msg.chan3_raw
                            telemetria_actual["rc"]["ch4"] = msg.chan4_raw
                            telemetria_actual["rc"]["ch5"] = msg.chan5_raw
                            telemetria_actual["rc"]["ch6"] = msg.chan6_raw
                            telemetria_actual["rc"]["ch7"] = msg.chan7_raw
                            telemetria_actual["rc"]["ch8"] = msg.chan8_raw
                            telemetria_actual["rc"]["rssi"] = msg.rssi

                        elif tipo == 'PARAM_VALUE':
                            param_id = msg.param_id
                            if isinstance(param_id, bytes):
                                param_id = param_id.decode('ascii', errors='ignore').rstrip('\x00')
                            telemetria_actual["parametros"][param_id] = round(msg.param_value, 4)

                        elif tipo == 'STATUSTEXT':
                            texto = msg.text
                            if isinstance(texto, bytes):
                                texto = texto.decode('utf-8', errors='ignore')
                            telemetria_actual["mensajes_estado"].append({
                                "tiempo": tiempo_actual,
                                "severidad": msg.severity,
                                "texto": texto
                            })
                            if len(telemetria_actual["mensajes_estado"]) > 150:
                                telemetria_actual["mensajes_estado"].pop(0)

                        if time.time() - ultimo_print_tiempo > 3.0:
                            ultimo_print_tiempo = time.time()
                            print(f"[MAVLink {hz if 'hz' in locals() else 0}Hz] T:{tiempo_actual}s | Pitch:{telemetria_actual['pitch']} | Roll:{telemetria_actual['roll']} | Modo:{telemetria_actual['sistema']['modo_vuelo']} | Arm:{telemetria_actual['sistema']['armado']}")

                    else:
                        if time.time() - ultimo_mensaje_tiempo > 3.5:
                            telemetria_actual["conectado"] = False
                            telemetria_actual["conexion"]["conectado"] = False
                            config_conexion["conectado"] = False

        except Exception as e:
            print(f"[Telemetría] Error de conexión: {e}. Reintentando en 3s...")
            with conexion_lock:
                conexion = None
            telemetria_actual["conectado"] = False
            config_conexion["conectado"] = False
            config_conexion["estado_texto"] = f"Error: {e}"
            time.sleep(3)

# ==============================================================
# HILO RECEPTOR DE TELEMETRÍA UDP DESDE JETSON NANO (PUERTO 5005)
# ==============================================================
def jetson_udp_telemetry_worker():
    """Recibe paquetes JSON de telemetría de alta frecuencia transmitidos por la Jetson Nano (IMU/BMP280)"""
    global telemetria_actual, config_conexion
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 5005))
        sock.settimeout(1.0)
        print("[Jetson UDP] ✅ Receptor de telemetría activo en puerto UDP 5005")
    except Exception as e:
        print(f"[Jetson UDP] Advertencia al enlazar puerto 5005: {e}")
        return

    contador_paquetes = 0
    ultimo_calculo = time.time()
    
    while hilo_receptor_activo:
        try:
            data, addr = sock.recvfrom(2048)
            payload = json.loads(data.decode("utf-8"))
            contador_paquetes += 1
            t_now = time.time()
            
            p_deg = float(payload.get("pitch", 0.0))
            r_deg = float(payload.get("roll", 0.0))
            y_deg = float(payload.get("yaw", 0.0))
            alt_m = float(payload.get("altitud", 0.0))
            spd_ms = float(payload.get("velocidad", 0.0))
            
            # Si no hay MAVLink serial activo ni simulación Unreal activa, actualizar desde Jetson
            if not config_conexion.get("conectado", False) and config_conexion.get("puerto") != "AIRSIM_RPC" and not estado_sim_mision.get("activa", False):
                telemetria_actual["conectado"] = True
                telemetria_actual["conexion"]["conectado"] = True
                telemetria_actual["conexion"]["estado"] = f"Jetson UDP ({addr[0]}:5005)"
                
                telemetria_actual["pitch"] = round(math.radians(p_deg), 3)
                telemetria_actual["roll"] = round(math.radians(r_deg), 3)
                telemetria_actual["yaw"] = round(math.radians(y_deg), 3)
                
                telemetria_actual["actitud"]["pitch"] = telemetria_actual["pitch"]
                telemetria_actual["actitud"]["roll"] = telemetria_actual["roll"]
                telemetria_actual["actitud"]["yaw"] = telemetria_actual["yaw"]
                telemetria_actual["actitud"]["pitch_deg"] = round(p_deg, 1)
                telemetria_actual["actitud"]["roll_deg"] = round(r_deg, 1)
                telemetria_actual["actitud"]["yaw_deg"] = round(y_deg % 360, 1)
                
                telemetria_actual["gps"]["alt_rel"] = round(alt_m, 2)
                telemetria_actual["vfr_hud"]["altitud_baro"] = round(alt_m, 2)
                telemetria_actual["vfr_hud"]["groundspeed"] = round(spd_ms, 2)
                telemetria_actual["vfr_hud"]["heading"] = int(y_deg % 360)
                
                if t_now - ultimo_calculo >= 1.0:
                    hz = round(contador_paquetes / (t_now - ultimo_calculo), 1)
                    telemetria_actual["hz_recepcion"] = hz
                    telemetria_actual["conexion"]["hz"] = hz
                    contador_paquetes = 0
                    ultimo_calculo = t_now
                    
        except socket.timeout:
            continue
        except Exception:
            pass

# ==============================================================
# HILO DE SIMULACIÓN Y REPRODUCCIÓN CSV
# ==============================================================
def simulador_csv_worker():
    global telemetria_actual, config_conexion
    
    filas_csv = []
    if os.path.exists(ARCHIVO_LOG_ACTUAL):
        try:
            with open(ARCHIVO_LOG_ACTUAL, mode='r', newline='') as f:
                reader = csv.DictReader(f)
                filas_csv = list(reader)
        except Exception as e:
            print(f"[Simulador] Error leyendo CSV: {e}")
            
    if not filas_csv:
        print("[Simulador] Generando datos dinámicos de prueba sintéticos.")
        for t in range(500):
            seg = t * 0.1
            filas_csv.append({
                "Tiempo(s)": str(round(seg, 2)),
                "Pitch(rad)": str(round(0.25 * math.sin(seg * 0.8), 3)),
                "Roll(rad)": str(round(0.35 * math.cos(seg * 0.5), 3)),
                "Yaw(rad)": str(round((seg * 0.1) % (math.pi * 2), 3)),
                "AltRel(m)": str(round(15.0 + 4.0 * math.sin(seg * 0.2), 2)),
                "Groundspeed(m/s)": str(round(5.0 + 2.0 * math.cos(seg * 0.3), 2)),
                "Voltaje(V)": str(round(15.8 - (seg * 0.005), 2)),
                "Corriente(A)": str(round(12.5 + 3.0 * math.sin(seg), 2)),
                "Bateria(%)": str(max(int(100 - (seg * 0.1)), 15)),
                "Latitud": str(round(-33.467225 + 0.00015 * math.cos(seg * 0.05), 7)),
                "Longitud": str(round(-70.657605 + 0.00018 * math.sin(seg * 0.05), 7)),
                "Satellites": "14",
                "ModoVuelo": "POSCTL",
                "VibeX": str(round(12.0 + 4.0 * math.sin(seg * 2), 1)),
                "VibeY": str(round(14.0 + 5.0 * math.cos(seg * 2), 1)),
                "VibeZ": str(round(18.0 + 3.0 * math.sin(seg * 3), 1))
            })

    telemetria_actual["simulador"]["total"] = len(filas_csv)
    idx = 0
    
    while hilo_receptor_activo:
        if config_conexion.get("modo_simulacion") and telemetria_actual["simulador"].get("activo") and config_conexion.get("puerto") != "AIRSIM_RPC":
            try:
                row = filas_csv[idx]
                t_val = float(row.get("Tiempo(s)", idx * 0.1))
                pitch = float(row.get("Pitch(rad)", 0))
                roll = float(row.get("Roll(rad)", 0))
                yaw = float(row.get("Yaw(rad)", 0))
                alt_rel = float(row.get("AltRel(m)", 10))
                speed = float(row.get("Groundspeed(m/s)", 4))
                volt = float(row.get("Voltaje(V)", 15.5))
                curr = float(row.get("Corriente(A)", 10.0))
                bat_pct = int(float(row.get("Bateria(%)", 85)))
                lat = float(row.get("Latitud", -33.467225))
                lon = float(row.get("Longitud", -70.657605))
                sats = int(row.get("Satellites", 12))
                modo = row.get("ModoVuelo", "POSCTL")
                vibe_x = float(row.get("VibeX", 12.0))
                vibe_y = float(row.get("VibeY", 14.0))
                vibe_z = float(row.get("VibeZ", 18.0))

                telemetria_actual["conectado"] = True
                telemetria_actual["tiempo"] = t_val
                telemetria_actual["hz_recepcion"] = 20.0
                telemetria_actual["pitch"] = pitch
                telemetria_actual["roll"] = roll
                telemetria_actual["yaw"] = yaw
                
                telemetria_actual["actitud"]["pitch"] = pitch
                telemetria_actual["actitud"]["roll"] = roll
                telemetria_actual["actitud"]["yaw"] = yaw
                telemetria_actual["actitud"]["pitch_deg"] = round(math.degrees(pitch), 1)
                telemetria_actual["actitud"]["roll_deg"] = round(math.degrees(roll), 1)
                telemetria_actual["actitud"]["yaw_deg"] = round((math.degrees(yaw) + 360) % 360, 1)

                telemetria_actual["vfr_hud"]["groundspeed"] = speed
                telemetria_actual["vfr_hud"]["airspeed"] = speed * 1.05
                telemetria_actual["vfr_hud"]["altitud_baro"] = alt_rel
                telemetria_actual["vfr_hud"]["heading"] = int((math.degrees(yaw) + 360) % 360)
                telemetria_actual["vfr_hud"]["climb"] = round(0.5 * math.sin(t_val), 2)

                telemetria_actual["gps"]["lat"] = lat
                telemetria_actual["gps"]["lon"] = lon
                telemetria_actual["gps"]["alt_rel"] = alt_rel
                telemetria_actual["gps"]["alt_amsl"] = round(alt_rel + 560.0, 2)
                telemetria_actual["gps"]["satellites"] = sats
                telemetria_actual["gps"]["fix_desc"] = "3D Fix"
                telemetria_actual["gps"]["fix_type"] = 3
                telemetria_actual["gps"]["hdop"] = 0.8

                telemetria_actual["sistema"]["voltaje_bateria"] = volt
                telemetria_actual["sistema"]["corriente_bateria"] = curr
                telemetria_actual["sistema"]["bateria_pct"] = bat_pct
                telemetria_actual["sistema"]["voltaje_celda"] = round(volt / 4.0, 2)
                telemetria_actual["sistema"]["modo_vuelo"] = modo
                telemetria_actual["sistema"]["armado"] = True
                telemetria_actual["sistema"]["carga_cpu"] = 28.5

                telemetria_actual["vibracion"]["vibe_x"] = vibe_x
                telemetria_actual["vibracion"]["vibe_y"] = vibe_y
                telemetria_actual["vibracion"]["vibe_z"] = vibe_z
                telemetria_actual["vibracion"]["nivel_salud"] = "Excelente (<30 m/s²)"

                telemetria_actual["actuadores"]["motor1_pwm"] = int(1450 + 100 * math.sin(t_val))
                telemetria_actual["actuadores"]["motor2_pwm"] = int(1450 - 100 * math.sin(t_val))
                telemetria_actual["actuadores"]["motor3_pwm"] = int(1450 + 100 * math.cos(t_val))
                telemetria_actual["actuadores"]["motor4_pwm"] = int(1450 - 100 * math.cos(t_val))

                telemetria_actual["simulador"]["activo"] = True
                telemetria_actual["simulador"]["indice"] = idx
                telemetria_actual["conexion"]["simulacion"] = True
                telemetria_actual["conexion"]["conectado"] = True
                telemetria_actual["conexion"]["estado"] = "Simulación CSV en Vivo"

                idx = (idx + 1) % len(filas_csv)
                delay = 0.05 / max(telemetria_actual["simulador"]["velocidad"], 0.1)
                time.sleep(delay)
            except Exception as ex:
                print(f"[Simulador Error] {ex}")
                time.sleep(0.5)
        else:
            time.sleep(0.2)

# ==============================================================
# STREAMING MJPEG PARA LA VISTA FPV
# ==============================================================
def generar_mjpeg_stream():
    """Generador MJPEG de latencia cero: omite cuadros rezagados y no satura sockets ni buffers"""
    last_id = -1
    while hilo_receptor_activo:
        data = None
        with stream_cond:
            if current_frame_id == last_id or current_encoded_jpeg is None:
                stream_cond.wait(timeout=0.04)
            if current_encoded_jpeg is not None and current_frame_id != last_id:
                data = current_encoded_jpeg
                last_id = current_frame_id

        if data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
        else:
            time.sleep(0.01)

# ==============================================================
# RUTAS WEB Y API REST
# ==============================================================
@app.route('/')
def home():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generar_mjpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stream')
def api_stream():
    """Servidor SSE que transmite telemetría consolidada y estado de visión a 20 Hz"""
    def event_stream():
        while hilo_receptor_activo:
            try:
                with conexion_lock:
                    datos_completos = {
                        "conectado": telemetria_actual.get("conectado", False),
                        "tiempo": telemetria_actual.get("tiempo", 0.0),
                        "hz_recepcion": telemetria_actual.get("hz_recepcion", 0.0),
                        "pitch": telemetria_actual.get("pitch", 0.0),
                        "roll": telemetria_actual.get("roll", 0.0),
                        "yaw": telemetria_actual.get("yaw", 0.0),
                        "actitud": dict(telemetria_actual.get("actitud", {})),
                        "gps": dict(telemetria_actual.get("gps", {})),
                        "vfr_hud": dict(telemetria_actual.get("vfr_hud", {})),
                        "sistema": dict(telemetria_actual.get("sistema", {})),
                        "conexion": dict(telemetria_actual.get("conexion", {})),
                        "ekf": dict(telemetria_actual.get("ekf", {})),
                        "vibracion": dict(telemetria_actual.get("vibracion", {})),
                        "imu": dict(telemetria_actual.get("imu", {}))
                    }
                with vision_lock:
                    datos_completos["vision"] = {
                        "conectado": estado_vision["conectado"],
                        "fuente": config_vision["fuente_tipo"],
                        "camara_activa": config_vision.get("camara_activa", "front"),
                        "target_drone_activo": config_vision.get("target_drone_activo", False),
                        "modo_seguimiento": config_vision["modo_seguimiento"],
                        "conteo_drones": estado_vision["conteo_drones"],
                        "conteo_personas": estado_vision["conteo_personas"],
                        "objetivo_fijado": estado_vision["objetivo_fijado"],
                        "target_id": estado_vision["target_id"],
                        "fps_ia": config_vision["fps_ia"],
                        "fps_camara": config_vision["fps_camara"],
                        "pid_out": dict(pid_out),
                        "pid_err": dict(pid_err),
                        "alerta_activa": estado_vision["alerta_activa"],
                        "mensaje_alerta": estado_vision["mensaje_alerta"]
                    }
                yield f"data: {json.dumps(datos_completos)}\n\n"
            except Exception:
                pass
            time.sleep(0.05)
    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/api/datos')
def obtener_datos():
    with conexion_lock:
        datos = {
            "conectado": telemetria_actual.get("conectado", False),
            "tiempo": telemetria_actual.get("tiempo", 0.0),
            "hz_recepcion": telemetria_actual.get("hz_recepcion", 0.0),
            "pitch": telemetria_actual.get("pitch", 0.0),
            "roll": telemetria_actual.get("roll", 0.0),
            "yaw": telemetria_actual.get("yaw", 0.0),
            "actitud": dict(telemetria_actual.get("actitud", {})),
            "gps": dict(telemetria_actual.get("gps", {})),
            "vfr_hud": dict(telemetria_actual.get("vfr_hud", {})),
            "sistema": dict(telemetria_actual.get("sistema", {})),
            "conexion": dict(telemetria_actual.get("conexion", {})),
            "ekf": dict(telemetria_actual.get("ekf", {})),
            "vibracion": dict(telemetria_actual.get("vibracion", {})),
            "imu": dict(telemetria_actual.get("imu", {}))
        }
    with vision_lock:
        datos["vision"] = {
            "conectado": estado_vision["conectado"],
            "fuente": config_vision["fuente_tipo"],
            "camara_activa": config_vision.get("camara_activa", "front"),
            "target_drone_activo": config_vision.get("target_drone_activo", False),
            "modo_seguimiento": config_vision["modo_seguimiento"],
            "conteo_drones": estado_vision["conteo_drones"],
            "conteo_personas": estado_vision["conteo_personas"],
            "objetivo_fijado": estado_vision["objetivo_fijado"],
            "target_id": estado_vision["target_id"],
            "fps_ia": config_vision["fps_ia"],
            "fps_camara": config_vision["fps_camara"],
            "pid_out": dict(pid_out),
            "pid_err": dict(pid_err),
            "alerta_activa": estado_vision["alerta_activa"],
            "mensaje_alerta": estado_vision["mensaje_alerta"]
        }
    return jsonify(datos)

# --- ENDPOINTS DE VISIÓN IA Y SIMULACIÓN UNREAL ---
@app.route('/api/vision/estado')
def api_vision_estado():
    with vision_lock:
        return jsonify({
            "status": "ok",
            "config": config_vision,
            "estado": estado_vision,
            "pid_out": pid_out,
            "pid_err": pid_err
        })

@app.route('/api/vision/config', methods=['POST'])
def api_vision_config():
    global config_vision
    data = request.get_json(silent=True) or {}
    
    with vision_lock:
        if "fuente_tipo" in data:
            config_vision["fuente_tipo"] = str(data["fuente_tipo"]).lower()
        if "rtsp_url" in data:
            config_vision["rtsp_url"] = str(data["rtsp_url"]).strip()
        if "unreal_url" in data:
            config_vision["unreal_url"] = str(data["unreal_url"]).strip()
        if "webcam_idx" in data:
            config_vision["webcam_idx"] = int(data["webcam_idx"])
        if "modelo_activo" in data:
            config_vision["modelo_activo"] = str(data["modelo_activo"]).lower()
        if "confianza" in data:
            config_vision["confianza"] = float(data["confianza"])
        if "alertas_activas" in data:
            config_vision["alertas_activas"] = bool(data["alertas_activas"])
            
    return jsonify({
        "status": "ok",
        "mensaje": "Configuración de visión actualizada.",
        "config": config_vision
    })

@app.route('/api/vision/tracking/modo', methods=['POST'])
def api_vision_tracking_modo():
    global config_vision
    data = request.get_json(silent=True) or {}
    nuevo_modo = data.get("modo", "OFF").upper().strip()
    
    if nuevo_modo not in ("OFF", "SIM", "ACTIVE_TRACK"):
        return jsonify({"status": "error", "mensaje": "Modo inválido. Usar OFF, SIM o ACTIVE_TRACK"}), 400
        
    with vision_lock:
        config_vision["modo_seguimiento"] = nuevo_modo
        image_pid_inst.reset()
        
    return jsonify({
        "status": "ok",
        "mensaje": f"Modo de seguimiento establecido en: {nuevo_modo}",
        "modo": nuevo_modo
    })

@app.route('/api/vision/tracking/pid', methods=['POST'])
def api_vision_tracking_pid():
    global image_pid_inst
    data = request.get_json(silent=True) or {}
    
    if "yaw_kp" in data: image_pid_inst.pid_yaw.kp = float(data["yaw_kp"])
    if "yaw_ki" in data: image_pid_inst.pid_yaw.ki = float(data["yaw_ki"])
    if "yaw_kd" in data: image_pid_inst.pid_yaw.kd = float(data["yaw_kd"])
    if "alt_kp" in data: image_pid_inst.pid_alt.kp = float(data["alt_kp"])
    if "alt_ki" in data: image_pid_inst.pid_alt.ki = float(data["alt_ki"])
    if "alt_kd" in data: image_pid_inst.pid_alt.kd = float(data["alt_kd"])
    if "spd_kp" in data: image_pid_inst.pid_spd.kp = float(data["spd_kp"])
    if "spd_ki" in data: image_pid_inst.pid_spd.ki = float(data["spd_ki"])
    if "spd_kd" in data: image_pid_inst.pid_spd.kd = float(data["spd_kd"])
    if "target_area" in data:
        image_pid_inst.target_area_frac = float(data["target_area"])
        image_pid_inst.target_area = image_pid_inst.target_area_frac * image_pid_inst.w * image_pid_inst.h

    return jsonify({"status": "ok", "mensaje": "Parámetros del PID de imagen actualizados exitosamente."})

@app.route('/api/vision/tracking/reset', methods=['POST'])
def api_vision_tracking_reset():
    image_pid_inst.reset()
    with vision_lock:
        pid_out["yaw_rate"] = 0.0
        pid_out["vx"] = 0.0
        pid_out["vy"] = 0.0
        pid_out["vz"] = 0.0
    return jsonify({"status": "ok", "mensaje": "Controlador PID reseteado a ceros."})

@app.route('/api/vision/camara/<cam_id>', methods=['POST', 'GET'])
def api_vision_camara(cam_id):
    ok, msg = seleccionar_camara(cam_id)
    return jsonify({"status": "ok" if ok else "error", "mensaje": msg, "camara_activa": config_vision.get("camara_activa", "front")})

@app.route('/api/simulacion/target_drone', methods=['POST'])
def api_simulacion_target_drone():
    data = request.get_json(silent=True) or {}
    dist = float(data.get("distancia", 7.0))
    alt = float(data.get("altitud", 0.5))
    ok, msg = posicionar_dron_objetivo(dist, alt)
    return jsonify({"status": "ok" if ok else "error", "mensaje": msg})

@app.route('/api/vision/toggle_flip', methods=['POST'])
def api_vision_toggle_flip():
    global config_vision
    data = request.get_json(silent=True) or {}
    with vision_lock:
        if "flip" in data:
            config_vision["flip_camara"] = bool(data["flip"])
        else:
            config_vision["flip_camara"] = not config_vision.get("flip_camara", True)
        estado_flip = config_vision["flip_camara"]
    return jsonify({
        "status": "ok",
        "flip_camara": estado_flip,
        "mensaje": f"Inversión de cámara 180°: {'ACTIVADA' if estado_flip else 'DESACTIVADA'}"
    })

@app.route('/api/jetson/command', methods=['POST'])
def api_jetson_command():
    """Envía un comando UDP a la Jetson Nano (puerto 5006)"""
    try:
        cmd = request.get_json(silent=True) or {}
        ip_jetson = "192.168.14.7"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(json.dumps(cmd).encode('utf-8'), (ip_jetson, 5006))
        sock.close()
        return jsonify({"status": "ok", "comando": cmd, "destino": f"{ip_jetson}:5006"})
    except Exception as e:
        return jsonify({"status": "error", "mensaje": str(e)}), 500

@app.route('/api/simulacion/unreal_preset', methods=['POST'])
def api_simulacion_unreal_preset():
    """Preset rápido para conectar con PX4 SITL en Unreal Engine (AirSim RPC + MAVLink UDP)"""
    global config_conexion, config_vision, conexion
    puerto_sitl = "udpin:0.0.0.0:14550"
    
    with conexion_lock:
        if conexion is not None:
            try: conexion.close()
            except Exception: pass
            conexion = None
            
    config_conexion["puerto"] = puerto_sitl
    config_conexion["baud"] = 57600
    config_conexion["deseado_conectar"] = True
    config_conexion["modo_simulacion"] = False
    
    with vision_lock:
        config_vision["fuente_tipo"] = "unreal"
        config_vision["modo_seguimiento"] = "SIM"
        
    return jsonify({
        "status": "ok",
        "mensaje": "Preset Simulación Unreal + PX4 SITL activado. Escuchando MAVLink en udpin:0.0.0.0:14550 y AirSim RPC...",
        "puerto": puerto_sitl,
        "video_fuente": "unreal"
    })

# --- ENDPOINTS DE CONEXIÓN MAVLINK Y SIMULACIÓN CSV ---
@app.route('/api/conexiones/puertos')
def api_listar_puertos():
    puertos = listar_puertos()
    return jsonify({
        "status": "ok",
        "puertos": puertos,
        "conexion_actual": config_conexion
    })

@app.route('/api/conectar', methods=['POST'])
def api_conectar():
    global conexion, config_conexion
    data = request.get_json(silent=True) or request.form
    puerto = data.get("puerto", PUERTO_DEFECTO)
    baud = int(data.get("baud", BAUD_DEFECTO))
    
    with conexion_lock:
        if conexion is not None:
            try:
                conexion.close()
            except Exception:
                pass
            conexion = None
            
    if puerto == "AIRSIM_RPC":
        config_conexion["puerto"] = "AIRSIM_RPC"
        config_conexion["baud"] = baud
        config_conexion["deseado_conectar"] = True
        config_conexion["modo_simulacion"] = False
        config_conexion["conectado"] = True
        config_conexion["estado_texto"] = "Conectado a Simulación Unreal"
        telemetria_actual["conectado"] = True
        telemetria_actual["simulador"]["activo"] = False
        telemetria_actual["conexion"]["conectado"] = True
        telemetria_actual["conexion"]["puerto"] = "AIRSIM_RPC"
        telemetria_actual["conexion"]["simulacion"] = True
        config_vision["fuente_tipo"] = "unreal"
        return jsonify({
            "status": "ok",
            "mensaje": "Conectado a Simulación Unreal Engine (AirSim RPC 41451).",
            "puerto": "AIRSIM_RPC"
        })

    config_conexion["puerto"] = puerto
    config_conexion["baud"] = baud
    config_conexion["deseado_conectar"] = True
    config_conexion["modo_simulacion"] = False
    telemetria_actual["simulador"]["activo"] = False
    
    return jsonify({
        "status": "ok",
        "mensaje": f"Conectando a {puerto} a {baud} baudios...",
        "puerto": puerto,
        "baud": baud
    })

@app.route('/api/desconectar', methods=['POST'])
def api_desconectar():
    global conexion, config_conexion
    with conexion_lock:
        if conexion is not None:
            try:
                conexion.close()
            except Exception:
                pass
            conexion = None
    config_conexion["conectado"] = False
    config_conexion["deseado_conectar"] = False
    config_conexion["estado_texto"] = "Desconectado"
    telemetria_actual["conectado"] = False
    telemetria_actual["conexion"]["conectado"] = False
    
    return jsonify({"status": "ok", "mensaje": "Desconectado exitosamente del Pixhawk."})

@app.route('/api/simulacion/<accion>', methods=['POST'])
def api_control_simulacion(accion):
    global config_conexion, telemetria_actual
    if accion == "iniciar":
        config_conexion["modo_simulacion"] = True
        telemetria_actual["simulador"]["activo"] = True
        return jsonify({"status": "ok", "mensaje": "Simulador de vuelo CSV iniciado."})
    elif accion == "detener":
        config_conexion["modo_simulacion"] = False
        telemetria_actual["simulador"]["activo"] = False
        telemetria_actual["conectado"] = False
        return jsonify({"status": "ok", "mensaje": "Simulador detenido."})
    elif accion == "velocidad":
        vel = float(request.args.get("vel", 1.0))
        telemetria_actual["simulador"]["velocidad"] = vel
        return jsonify({"status": "ok", "velocidad": vel})
    return jsonify({"status": "error", "mensaje": "Acción desconocida"}), 400

@app.route('/api/modo/<nombre_modo>', methods=['POST'])
def api_cambiar_modo(nombre_modo):
    exito, msg = cambiar_modo_vuelo_cmd(nombre_modo)
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/armar/<int:estado>', methods=['POST'])
def api_armar_dron(estado):
    armar = (estado == 1)
    force = request.args.get("force", "0") == "1"
    exito, msg = armar_desarmar_cmd(armar, force)
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/takeoff', methods=['POST', 'GET'])
def api_takeoff():
    data = request.get_json(silent=True) or {}
    alt = float(request.args.get('alt', data.get('alt', 15.0)))
    exito, msg = despegar_cmd(altitud=alt)
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg, "altitud": alt})

@app.route('/api/land', methods=['POST', 'GET'])
def api_land():
    exito, msg = aterrizar_cmd()
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/rtl', methods=['POST', 'GET'])
def api_rtl():
    exito, msg = retorno_cmd()
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/motor/<int:motor_id>/<valor>', methods=['POST', 'GET'])
def controlar_motor(motor_id, valor):
    try:
        valor_flotante = float(valor)
    except ValueError:
        return jsonify({ "status": "error", "mensaje": "Valor numérico requerido" }), 400

    timeout = float(request.args.get("timeout", 3.0))
    exito, mensaje = enviar_comando_motor(motor_id, valor_flotante, timeout)
    return jsonify({
        "status": "ok" if exito else "error",
        "motor": motor_id,
        "valor": valor_flotante,
        "mensaje": mensaje
    })

@app.route('/api/motor/stop_all', methods=['POST', 'GET'])
def detener_todos_los_motores():
    for m_id in range(1, 9):
        enviar_comando_motor(m_id, 0)
    return jsonify({
        "status": "ok",
        "mensaje": "Todos los motores han sido detenidos de forma segura."
    })

@app.route('/api/parametro/<nombre>', methods=['GET'])
def api_obtener_parametro(nombre):
    if nombre in telemetria_actual["parametros"]:
        return jsonify({"status": "ok", "parametro": nombre, "valor": telemetria_actual["parametros"][nombre]})
    exito, msg = leer_parametro_mavlink(nombre)
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/parametro/<nombre>/<valor>', methods=['POST'])
def api_escribir_parametro(nombre, valor):
    exito, msg = escribir_parametro_mavlink(nombre, valor)
    return jsonify({"status": "ok" if exito else "error", "mensaje": msg})

@app.route('/api/descargar_log')
def descargar_log():
    try:
        if os.path.exists(ARCHIVO_LOG_ACTUAL):
            return send_file(ARCHIVO_LOG_ACTUAL, as_attachment=True, download_name="telemetria_pixhawk_log.csv")
        return jsonify({"status": "error", "mensaje": "No hay log de telemetría registrado."}), 404
    except Exception as e:
        return jsonify({"status": "error", "mensaje": str(e)}), 404

# --- PLANIFICADOR DE MISIÓN Y GESTIÓN DE SETPOINTS (QGC COMPATIBLE) ---
mision_actual = {
    "puntos": [
        {
            "id": 1,
            "tipo": "TAKEOFF",
            "cmd_code": 22,
            "lat": -33.467225,
            "lon": -70.657605,
            "alt": 20.0,
            "velocidad": 5.0,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0
        },
        {
            "id": 2,
            "tipo": "WAYPOINT",
            "cmd_code": 16,
            "lat": -33.465800,
            "lon": -70.656200,
            "alt": 30.0,
            "velocidad": 5.0,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0
        },
        {
            "id": 3,
            "tipo": "WAYPOINT",
            "cmd_code": 16,
            "lat": -33.464500,
            "lon": -70.658000,
            "alt": 35.0,
            "velocidad": 5.0,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0
        },
        {
            "id": 4,
            "tipo": "RTL",
            "cmd_code": 20,
            "lat": -33.467225,
            "lon": -70.657605,
            "alt": 20.0,
            "velocidad": 5.0,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0
        }
    ],
    "altitud_defecto": 20.0,
    "velocidad_defecto": 5.0
}
mision_lock = threading.Lock()

# Georreferenciación exacta del mapa de Parque O'Higgins (MyProject Cesium / AirSim)
ORIGEN_PARQUE_OHIGGINS_LAT = -33.467225
ORIGEN_PARQUE_OHIGGINS_LON = -70.657605
ORIGEN_PARQUE_OHIGGINS_ALT = 500.0

# Estado en vivo de la ejecución de misión en el simulador Unreal Engine
estado_sim_mision = {
    "activa": False,
    "pausada": False,
    "abortada": False,
    "indice_actual": 0,
    "total_puntos": 0,
    "tipo_actual": "",
    "estado_texto": "Inactivo (Listo para ejecutar)",
    "progreso_pct": 0,
    "distancia_restante": 0.0,
    "tiempo_inicio": 0.0
}
sim_mision_lock = threading.Lock()
hilo_sim_mision = None

def ejecutar_mision_unreal_worker(puntos, alt_def=20.0, vel_def=5.0):
    """
    Ejecuta de forma autónoma la secuencia de setpoints en el proyecto Unreal Engine (AirSim + PX4 SITL).
    Comanda el dron Holybro X650 en tiempo real mediante comandos MAVLink nativos de PX4 (DO_REPOSITION / TAKEOFF),
    eliminando la espera de GPS de AirSim RPC y permitiendo control preciso de actuadores,
    mientras sincroniza la telemetría en tiempo real con la GCS web (Leaflet y PFD).
    """
    global estado_sim_mision, telemetria_actual, conexion, config_conexion
    
    with sim_mision_lock:
        estado_sim_mision["activa"] = True
        estado_sim_mision["abortada"] = False
        estado_sim_mision["total_puntos"] = len(puntos)
        estado_sim_mision["indice_actual"] = 0
        estado_sim_mision["estado_texto"] = "Conectando con simulador Unreal y MAVLink..."
        estado_sim_mision["progreso_pct"] = 0
        estado_sim_mision["tiempo_inicio"] = time.time()
        
    print(f"[Misión Unreal] 🚀 Iniciando ejecución de {len(puntos)} setpoints en Parque O'Higgins...")
    
    # 1. Conexión a AirSim para cinemática / ground truth
    client_as = None
    if HAY_AIRSIM:
        try:
            client_as = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=5.0)
            if not client_as.ping():
                raise RuntimeError("AirSim ping fallo")
            print("[Misión Unreal] ✅ Conexión confirmada con AirSim RPC (puerto 41451)")
        except Exception as e:
            print(f"[Misión Unreal] AirSim RPC no disponible ({e})")
            client_as = None

    # 2. Conexión a PX4 SITL MAVLink
    mav_propio = False
    mav = None
    with conexion_lock:
        if conexion is not None and config_conexion.get("conectado", False):
            mav = conexion

    if mav is None:
        try:
            print("[Misión Unreal] Abriendo enlace MAVLink con PX4 SITL (udpin:0.0.0.0:14540)...")
            mav = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255, source_component=0)
            mav.wait_heartbeat(timeout=3.0)
            mav_propio = True
            print("[Misión Unreal] ✅ Enlace MAVLink establecido en 14540.")
        except Exception as e_m:
            try:
                print(f"[Misión Unreal] Probando enlace secundario UDP 14550 ({e_m})...")
                mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', source_system=255, source_component=0)
                mav.wait_heartbeat(timeout=2.0)
                mav_propio = True
            except Exception:
                mav = None

    if client_as is not None or mav is not None:
        try:
            target_sys = getattr(mav, 'target_system', 1) or 1
            target_comp = getattr(mav, 'target_component', 1) or 1

            # Armar PX4 via MAVLink si está disponible
            if mav is not None:
                try:
                    mav.mav.command_long_send(
                        target_sys, target_comp,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0, 1.0, 21196.0, 0, 0, 0, 0, 0
                    )
                except Exception as e_arm:
                    print(f"[Misión Unreal] Advertencia al armar: {e_arm}")

            telemetria_actual["sistema"]["armado"] = True
            telemetria_actual["sistema"]["modo_vuelo"] = "AUTO_MISSION"
            telemetria_actual["conexion"]["conectado"] = True
            telemetria_actual["conexion"]["estado"] = "Unreal SITL (Misión Activa)"
            
            with sim_mision_lock:
                estado_sim_mision["estado_texto"] = "Verificando despegue en Parque O'Higgins..."
                
            # Verificar si ya está en el aire o necesita despegue
            alt_actual = 0.0
            if client_as is not None:
                try:
                    kin_init = client_as.simGetGroundTruthKinematics(vehicle_name="PX4")
                    alt_actual = -kin_init.position.z_val
                except Exception:
                    pass

            if alt_actual < 2.0 and mav is not None:
                with sim_mision_lock:
                    estado_sim_mision["estado_texto"] = "Despegando Holybro X650 a altitud de seguridad..."
                try:
                    mav.mav.command_long_send(
                        target_sys, target_comp,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        0, 0, 0, 0,
                        float('nan'), float('nan'), float('nan'),
                        float(alt_def)
                    )
                except Exception as e_to:
                    print(f"[Misión Unreal] Error en comando takeoff: {e_to}")

                # Esperar ascenso a altitud segura
                t_to = time.time()
                while time.time() - t_to < 8.0 and not estado_sim_mision["abortada"]:
                    if client_as is not None:
                        try:
                            kin = client_as.simGetGroundTruthKinematics(vehicle_name="PX4")
                            if -kin.position.z_val >= min(alt_def * 0.6, 5.0):
                                break
                        except Exception:
                            pass
                    time.sleep(0.3)
            
            for idx, sp in enumerate(puntos):
                with sim_mision_lock:
                    if estado_sim_mision["abortada"]:
                        print("[Misión Unreal] ⚠️ Misión abortada por el usuario.")
                        break
                    estado_sim_mision["indice_actual"] = idx + 1
                    tipo = sp.get("tipo", "WAYPOINT")
                    estado_sim_mision["tipo_actual"] = tipo
                    target_lat = float(sp.get("lat", ORIGEN_PARQUE_OHIGGINS_LAT))
                    target_lon = float(sp.get("lon", ORIGEN_PARQUE_OHIGGINS_LON))
                    target_alt = float(sp.get("alt", alt_def))
                    velocidad = float(sp.get("velocidad", vel_def))
                    estado_sim_mision["estado_texto"] = f"Volando a Setpoint {idx+1}/{len(puntos)} [{tipo}]..."
                    estado_sim_mision["progreso_pct"] = int((idx / len(puntos)) * 100)

                print(f"[Misión Unreal] WP {idx+1}: {tipo} -> Lat:{target_lat:.6f}, Lon:{target_lon:.6f}, Alt:{target_alt}m @ {velocidad}m/s")

                if tipo == "LOITER":
                    espera = float(sp.get("param1", 5.0))
                    t_loiter = time.time()
                    while time.time() - t_loiter < espera and not estado_sim_mision["abortada"]:
                        time.sleep(0.2)
                    continue
                elif tipo == "RTL":
                    with sim_mision_lock:
                        estado_sim_mision["estado_texto"] = "Retornando a punto de origen en Parque O'Higgins (RTL)..."
                    if mav is not None:
                        try:
                            mav.mav.command_long_send(
                                target_sys, target_comp,
                                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                0, 0, 0, 0, 0, 0, 0, 0
                            )
                        except Exception:
                            pass
                    time.sleep(3.0)
                    continue
                elif tipo == "LAND":
                    with sim_mision_lock:
                        estado_sim_mision["estado_texto"] = "Aterrizando Holybro X650..."
                    if mav is not None:
                        try:
                            mav.mav.command_long_send(
                                target_sys, target_comp,
                                mavutil.mavlink.MAV_CMD_NAV_LAND,
                                0, 0, 0, 0, float('nan'), float('nan'), float('nan'), 0
                            )
                        except Exception:
                            pass
                    time.sleep(3.0)
                    continue
                
                # Comandar navegación al setpoint vía PX4 MAVLink (DO_REPOSITION)
                if mav is not None:
                    try:
                        mav.mav.command_int_send(
                            target_sys, target_comp,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                            0, 0,
                            float(velocidad),
                            1, # Loiter en llegada
                            0,
                            float('nan'),
                            int(target_lat * 1e7),
                            int(target_lon * 1e7),
                            float(target_alt)
                        )
                    except Exception as e_cmd:
                        print(f"[Misión Unreal] Error al transmitir reposition MAVLink: {e_cmd}")

                # Bucle de seguimiento y actualización de telemetría de vuelo
                t_wp = time.time()
                curr_lat = telemetria_actual["gps"].get("lat", target_lat)
                curr_lon = telemetria_actual["gps"].get("lon", target_lon)
                curr_alt = telemetria_actual["gps"].get("alt_rel", target_alt)
                curr_spd = 0.0

                while not estado_sim_mision["abortada"] and hilo_receptor_activo:
                    dist_rem = 0.0

                    if client_as is not None:
                        try:
                            kin = client_as.simGetGroundTruthKinematics(vehicle_name="PX4")
                            pos = kin.position
                            ori = kin.orientation
                            pitch, roll, yaw = airsim.to_eularian_angles(ori)
                            
                            try:
                                gps = client_as.getGpsData(vehicle_name="PX4")
                                geo = gps.gnss.geo_point
                                if not math.isnan(geo.latitude) and abs(geo.latitude) > 1.0:
                                    curr_lat = geo.latitude
                                    curr_lon = geo.longitude
                                else:
                                    raise ValueError("GPS inválido")
                            except Exception:
                                curr_lat = airsim_spawn_origin["lat"] + (pos.x_val / 111139.0)
                                curr_lon = airsim_spawn_origin["lon"] + (pos.y_val / (111139.0 * math.cos(math.radians(airsim_spawn_origin["lat"]))))
                            
                            curr_alt = round(max(0.0, -pos.z_val), 2)
                            curr_spd = math.sqrt(kin.linear_velocity.x_val**2 + kin.linear_velocity.y_val**2 + kin.linear_velocity.z_val**2)
                            
                            dx = (target_lat - curr_lat) * 111139.0
                            dy = (target_lon - curr_lon) * 111139.0 * math.cos(math.radians(target_lat))
                            dz = target_alt - curr_alt
                            dist_2d = math.hypot(dx, dy)
                            dist_rem = math.sqrt(dx**2 + dy**2 + dz**2)

                            with sim_mision_lock:
                                estado_sim_mision["distancia_restante"] = round(dist_rem, 1)

                            # Solo actualizar telemetría directa si MAVLink no está reportando (evita colisiones y saltos de muestreo)
                            hay_mavlink_rec = (time.time() - telemetria_actual.get("_ultimo_mavlink_ts", 0)) < 2.0
                            if not hay_mavlink_rec:
                                telemetria_actual["gps"]["lat"] = round(curr_lat, 7)
                                telemetria_actual["gps"]["lon"] = round(curr_lon, 7)
                                telemetria_actual["gps"]["alt_rel"] = round(curr_alt, 2)
                                telemetria_actual["gps"]["alt_amsl"] = round(curr_alt + ORIGEN_PARQUE_OHIGGINS_ALT, 2)
                                telemetria_actual["gps"]["fix_desc"] = "3D Fix (AirSim PX4)"
                                telemetria_actual["gps"]["satellites"] = 18
                                telemetria_actual["gps"]["fix_type"] = 3
                                
                                telemetria_actual["actitud"]["pitch"] = round(pitch, 3)
                                telemetria_actual["actitud"]["roll"] = round(roll, 3)
                                telemetria_actual["actitud"]["yaw"] = round(yaw, 3)
                                telemetria_actual["actitud"]["pitch_deg"] = round(math.degrees(pitch), 1)
                                telemetria_actual["actitud"]["roll_deg"] = round(math.degrees(roll), 1)
                                telemetria_actual["actitud"]["yaw_deg"] = round((math.degrees(yaw) + 360) % 360, 1)
                                
                                telemetria_actual["pitch"] = telemetria_actual["actitud"]["pitch"]
                                telemetria_actual["roll"] = telemetria_actual["actitud"]["roll"]
                                telemetria_actual["yaw"] = telemetria_actual["actitud"]["yaw"]
                                
                                telemetria_actual["vfr_hud"]["groundspeed"] = round(curr_spd, 2)
                                telemetria_actual["vfr_hud"]["heading"] = int((math.degrees(yaw) + 360) % 360)
                                telemetria_actual["vfr_hud"]["altitud_baro"] = round(curr_alt, 2)

                            # Criterio de llegada al waypoint: 
                            # Si distancia 2D es menor a 4.0m y altitud dentro de margen (o 3D < 5.0m), se considera alcanzado
                            if dist_2d < 4.0 or dist_rem < 4.5 or (time.time() - t_wp > max(45.0, (dist_rem / max(velocidad, 1.0)) * 3.0)):
                                print(f"[Misión Unreal] ✅ Setpoint {idx+1} alcanzado (dist_2d={dist_2d:.1f}m, dist_3d={dist_rem:.1f}m).")
                                break
                        except Exception as e_k:
                            pass
                    else:
                        time.sleep(1.0)
                        break

                    time.sleep(0.1)

            if not estado_sim_mision["abortada"]:
                with sim_mision_lock:
                    estado_sim_mision["progreso_pct"] = 100
                    estado_sim_mision["estado_texto"] = "Misión de setpoints completada exitosamente."
                print("[Misión Unreal] ✅ Misión finalizada con éxito.")
            else:
                with sim_mision_lock:
                    estado_sim_mision["estado_texto"] = "Misión abortada por el operador. Dron en hover."
                    
        except Exception as ex:
            print(f"[Misión Unreal Error] {ex}")
            with sim_mision_lock:
                estado_sim_mision["estado_texto"] = f"Error en misión: {ex}"
        finally:
            if mav_propio and mav is not None:
                try: mav.close()
                except Exception: pass
            with sim_mision_lock:
                estado_sim_mision["activa"] = False
    else:
        # Modo cinemático de respaldo (si Unreal aún no fue abierto)
        try:
            telemetria_actual["conectado"] = True
            telemetria_actual["conexion"]["conectado"] = True
            telemetria_actual["conexion"]["estado"] = "Simulación Cinemática de Setpoints"
            telemetria_actual["sistema"]["armado"] = True
            telemetria_actual["sistema"]["modo_vuelo"] = "SIM_MISSION"
            
            curr_lat = ORIGEN_PARQUE_OHIGGINS_LAT
            curr_lon = ORIGEN_PARQUE_OHIGGINS_LON
            curr_alt = 0.0

            for idx, sp in enumerate(puntos):
                with sim_mision_lock:
                    if estado_sim_mision["abortada"]: break
                    estado_sim_mision["indice_actual"] = idx + 1
                    tipo = sp.get("tipo", "WAYPOINT")
                    estado_sim_mision["tipo_actual"] = tipo
                    target_lat = float(sp.get("lat", curr_lat))
                    target_lon = float(sp.get("lon", curr_lon))
                    target_alt = float(sp.get("alt", alt_def))
                    velocidad = float(sp.get("velocidad", vel_def))
                    estado_sim_mision["estado_texto"] = f"Navegando a Setpoint {idx+1}/{len(puntos)} [{tipo}]..."
                    estado_sim_mision["progreso_pct"] = int((idx / len(puntos)) * 100)

                pasos = 30
                d_lat = (target_lat - curr_lat) / pasos
                d_lon = (target_lon - curr_lon) / pasos
                d_alt = (target_alt - curr_alt) / pasos
                
                for _ in range(pasos):
                    if estado_sim_mision["abortada"] or not hilo_receptor_activo: break
                    curr_lat += d_lat
                    curr_lon += d_lon
                    curr_alt += d_alt
                    
                    telemetria_actual["gps"]["lat"] = round(curr_lat, 7)
                    telemetria_actual["gps"]["lon"] = round(curr_lon, 7)
                    telemetria_actual["gps"]["alt_rel"] = round(curr_alt, 2)
                    telemetria_actual["gps"]["alt_amsl"] = round(curr_alt + ORIGEN_PARQUE_OHIGGINS_ALT, 2)
                    telemetria_actual["gps"]["fix_desc"] = "3D Fix (Simulado)"
                    telemetria_actual["gps"]["satellites"] = 16
                    telemetria_actual["vfr_hud"]["groundspeed"] = round(velocidad, 1)
                    telemetria_actual["vfr_hud"]["altitud_baro"] = round(curr_alt, 2)
                    
                    with sim_mision_lock:
                        rem_m = math.sqrt(((target_lat - curr_lat) * 111139)**2 + ((target_lon - curr_lon) * 111139)**2)
                        estado_sim_mision["distancia_restante"] = round(rem_m, 1)
                    time.sleep(0.1)

            with sim_mision_lock:
                estado_sim_mision["progreso_pct"] = 100
                estado_sim_mision["estado_texto"] = "Misión completada."
        finally:
            with sim_mision_lock:
                estado_sim_mision["activa"] = False

@app.route('/api/mision', methods=['GET'])
def api_obtener_mision():
    with mision_lock:
        return jsonify({"status": "ok", "mision": mision_actual})

@app.route('/api/mision/guardar', methods=['POST'])
def api_guardar_mision():
    global mision_actual
    data = request.get_json(silent=True) or {}
    puntos = data.get("puntos", [])
    alt_def = float(data.get("altitud_defecto", 20.0))
    vel_def = float(data.get("velocidad_defecto", 5.0))
    
    with mision_lock:
        mision_actual["puntos"] = puntos
        mision_actual["altitud_defecto"] = alt_def
        mision_actual["velocidad_defecto"] = vel_def
        
    return jsonify({"status": "ok", "mensaje": f"Misión guardada con {len(puntos)} setpoints."})

@app.route('/api/mision/ejecutar_simulacion', methods=['POST'])
def api_mision_ejecutar_simulacion():
    """Ejecuta los setpoints en la simulación Unreal Engine (Parque O'Higgins)"""
    global hilo_sim_mision, estado_sim_mision
    with mision_lock:
        puntos = list(mision_actual.get("puntos", []))
        alt_def = float(mision_actual.get("altitud_defecto", 20.0))
        vel_def = float(mision_actual.get("velocidad_defecto", 5.0))

    if not puntos:
        return jsonify({"status": "error", "mensaje": "No hay setpoints en la misión. Agrega puntos en el mapa."}), 400

    with sim_mision_lock:
        if estado_sim_mision["activa"]:
            return jsonify({"status": "warn", "mensaje": "Una misión ya se encuentra activa en el simulador."})
        estado_sim_mision["activa"] = True
        estado_sim_mision["abortada"] = False

    hilo_sim_mision = threading.Thread(
        target=ejecutar_mision_unreal_worker,
        args=(puntos, alt_def, vel_def),
        daemon=True
    )
    hilo_sim_mision.start()

    return jsonify({
        "status": "ok",
        "mensaje": f"Misión de {len(puntos)} setpoints iniciada en Unreal Engine (Parque O'Higgins)."
    })

@app.route('/api/mision/abortar_simulacion', methods=['POST'])
def api_mision_abortar_simulacion():
    global estado_sim_mision
    with sim_mision_lock:
        estado_sim_mision["abortada"] = True
        estado_sim_mision["estado_texto"] = "Abortando misión..."
    return jsonify({"status": "ok", "mensaje": "Comando de abortar misión enviado al simulador."})

@app.route('/api/mision/estado_simulacion', methods=['GET'])
def api_mision_estado_simulacion():
    with sim_mision_lock:
        return jsonify(dict(estado_sim_mision))

@app.route('/api/mision/subir', methods=['POST'])
def api_subir_mision_mavlink():
    global conexion, config_conexion
    with conexion_lock:
        if config_conexion.get("puerto") == "AIRSIM_RPC" or conexion is None:
            with mision_lock:
                puntos = mision_actual.get("puntos", [])
            if not puntos:
                return jsonify({"status": "error", "mensaje": "La misión está vacía. Agrega setpoints antes de subir."}), 400
            return jsonify({
                "status": "ok",
                "mensaje": f"Misión de {len(puntos)} setpoints sincronizada. Haz clic en 'Volar en Unreal (AirSim)' para iniciar la navegación autónoma."
            })
        
        with mision_lock:
            puntos = mision_actual["puntos"]
            if not puntos:
                return jsonify({"status": "error", "mensaje": "La misión está vacía. Agrega setpoints antes de subir."}), 400
            
            try:
                target_sys = getattr(conexion, 'target_system', 1) or 1
                target_comp = getattr(conexion, 'target_component', 1) or 1
                
                try:
                    conexion.mav.mission_clear_all_send(target_sys, target_comp)
                except Exception:
                    pass
                time.sleep(0.1)
                
                count = len(puntos)
                conexion.mav.mission_count_send(target_sys, target_comp, count)
                
                for idx, p in enumerate(puntos):
                    cmd = int(p.get("cmd_code", 16))
                    lat = int(float(p.get("lat", 0)) * 1e7)
                    lon = int(float(p.get("lon", 0)) * 1e7)
                    alt = float(p.get("alt", 20))
                    p1 = float(p.get("param1", 0))
                    p2 = float(p.get("param2", 0))
                    p3 = float(p.get("param3", 0))
                    p4 = float(p.get("param4", 0))
                    
                    conexion.mav.mission_item_int_send(
                        target_sys, target_comp,
                        idx,
                        3, # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                        cmd,
                        1 if idx == 0 else 0,
                        1,
                        p1, p2, p3, p4,
                        lat, lon, alt
                    )
                    time.sleep(0.05)
                
                return jsonify({"status": "ok", "mensaje": f"Misión de {count} setpoints enviada exitosamente al Pixhawk."})
            except Exception as e:
                return jsonify({"status": "error", "mensaje": f"Error al transmitir misión vía MAVLink: {e}"}), 500

@app.route('/api/mision/exportar_qgc', methods=['GET', 'POST'])
def api_exportar_qgc():
    with mision_lock:
        items = []
        puntos = mision_actual["puntos"]
        for idx, p in enumerate(puntos):
            cmd_code = int(p.get("cmd_code", 16))
            items.append({
                "autoContinue": True,
                "command": cmd_code,
                "doJumpId": idx + 1,
                "frame": 3,
                "params": [
                    float(p.get("param1", 0)),
                    float(p.get("param2", 0)),
                    float(p.get("param3", 0)),
                    float(p.get("param4", 0)),
                    float(p.get("lat", 0)),
                    float(p.get("lon", 0)),
                    float(p.get("alt", 20))
                ],
                "type": "SimpleItem"
            })
        
        home_pos = [
            puntos[0]["lat"] if puntos else -33.467225,
            puntos[0]["lon"] if puntos else -70.657605,
            puntos[0]["alt"] if puntos else 20.0
        ]
        
        qgc_plan = {
            "fileType": "Plan",
            "version": 1,
            "groundStation": "GCS Pixhawk 6X UBO",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "rallyPoints": {"points": [], "version": 2},
            "mission": {
                "cruiseSpeed": mision_actual.get("velocidad_defecto", 5.0),
                "hoverSpeed": 3.0,
                "firmwareType": 12,
                "plannedHomePosition": home_pos,
                "items": items
            }
        }
        return jsonify(qgc_plan)

@app.route('/api/mision/importar_qgc', methods=['POST'])
def api_importar_qgc():
    global mision_actual
    try:
        data = request.get_json(force=True, silent=True)
        if not data and request.files:
            file = request.files.get('archivo')
            if file:
                data = json.loads(file.read().decode('utf-8'))
        
        if not data:
            return jsonify({"status": "error", "mensaje": "Datos JSON o archivo .plan de QGroundControl no recibidos."}), 400
            
        items = data.get("mission", {}).get("items", [])
        nuevos_puntos = []
        
        cmd_map = {
            16: ("WAYPOINT", 16),
            22: ("TAKEOFF", 22),
            20: ("RTL", 20),
            21: ("LAND", 21),
            19: ("LOITER", 19)
        }
        
        for idx, item in enumerate(items):
            cmd = item.get("command", 16)
            params = item.get("params", [0, 0, 0, 0, 0, 0, 20])
            tipo_nombre, cmd_code = cmd_map.get(cmd, ("WAYPOINT", cmd))
            
            lat = params[4] if len(params) > 4 else 0.0
            lon = params[5] if len(params) > 5 else 0.0
            alt = params[6] if len(params) > 6 else 20.0
            
            nuevos_puntos.append({
                "id": idx + 1,
                "tipo": tipo_nombre,
                "cmd_code": cmd_code,
                "lat": float(lat),
                "lon": float(lon),
                "alt": float(alt),
                "velocidad": 5.0,
                "param1": float(params[0]) if len(params) > 0 else 0.0,
                "param2": float(params[1]) if len(params) > 1 else 0.0,
                "param3": float(params[2]) if len(params) > 2 else 0.0,
                "param4": float(params[3]) if len(params) > 3 else 0.0
            })
            
        with mision_lock:
            mision_actual["puntos"] = nuevos_puntos
            
        return jsonify({
            "status": "ok",
            "mensaje": f"Se importaron {len(nuevos_puntos)} setpoints desde el plan QGroundControl.",
            "puntos": nuevos_puntos
        })
    except Exception as e:
        return jsonify({"status": "error", "mensaje": f"Error al procesar archivo QGroundControl: {e}"}), 400

# ==============================================================
# INICIALIZACIÓN DE LA APLICACIÓN
# ==============================================================
if __name__ == '__main__':
    # 1. Hilo receptor MAVLink
    hilo_telemetria = threading.Thread(target=receptor_mavlink_worker, daemon=True)
    hilo_telemetria.start()
    
    # 2. Hilo simulador de vuelo CSV
    hilo_sim = threading.Thread(target=simulador_csv_worker, daemon=True)
    hilo_sim.start()
    
    # 3. Hilo capturador de video
    hilo_video = threading.Thread(target=capturador_video_worker, daemon=True)
    hilo_video.start()
    
    # 4. Hilo de inferencia IA YOLOv8
    hilo_yolo = threading.Thread(target=yolo_inferencia_worker, daemon=True)
    hilo_yolo.start()
    
    # 5. Hilo de control de seguimiento PID MAVLink
    hilo_pid = threading.Thread(target=pid_seguimiento_worker, daemon=True)
    hilo_pid.start()
    
    # 6. Hilo receptor de telemetría Jetson Nano UDP (5005)
    hilo_jetson_udp = threading.Thread(target=jetson_udp_telemetry_worker, daemon=True)
    hilo_jetson_udp.start()
    
    # 7. Hilo receptor de telemetría Unreal Engine AirSim RPC (41451)
    hilo_airsim_telemetry = threading.Thread(target=airsim_telemetria_worker, daemon=True)
    hilo_airsim_telemetry.start()
    
    print("====================================================================")
    print("🚀 GCS PIXHAWK 6X - ESTACIÓN TERRENA DE CONTROL, TELEMETRÍA & VISIÓN IA")
    print("🌐 Servidor listo en: http://localhost:5000")
    print("📡 Streaming SSE y API REST activos a 20Hz.")
    print("👁️ Cámara FPV Jetson RTSP (192.168.14.7:8554/cam0) y YOLOv8 listas")
    print("📡 Telemetría Jetson UDP escuchando en 0.0.0.0:5005")
    print("🎮 Soporte activo para Simulación en Unreal Engine + PX4 SITL (AirSim RPC 41451).")
    print("====================================================================")
    
    app.run(host='0.0.0.0', port=5000, threaded=True)
