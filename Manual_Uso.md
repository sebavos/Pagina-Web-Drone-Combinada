# 🚁 Manual de Uso - Estación Terrena GCS (Pixhawk 6X & Flask)

Estación Terrena de Control en Tiempo Real (Ground Control Station - GCS) desarrollada para la telemetría, planificación de misiones y diagnóstico de drones **Holybro X650** equipados con **Pixhawk 6X**, **PX4 / ArduPilot**, **Python (Flask)**, **pymavlink** y **Chart.js / Leaflet.js**.

---

## 🚀 Características Principales

* **Telemetría en Vivo 20 Hz:** Lectura en tiempo real de actitud (Pitch, Roll, Yaw), velocidad, altitud AGL, estado de batería por celda, varianza EKF y vibraciones 3D.
* **Planificador de Misiones & Setpoints (Estilo QGroundControl):** Creación interactiva de rutas haciendo clic sobre el mapa, asignación de altitudes y velocidades, subida directa al Pixhawk vía MAVLink e importación/exportación de archivos `.plan` de QGroundControl.
* **Selección Libre de Puertos COM & Web Serial API:** Menú desplegable con entrada manual libre (`COM1` a `COM256`, `/dev/ttyUSB0`) y detección nativa por navegador mediante Web Serial API.
* **Primary Flight Display (PFD) 60 FPS:** Horizonte artificial aeronáutico en HTML5 Canvas con cintas de rumbo, altitud y velocidad.
* **Gráficos Dinámicos Continuos:** Historial de telemetría a 60 FPS con ventanas ajustables (10s, 30s, 60s, 120s, 300s o Historial Completo sin cortes).
* **Control de Actuadores (Sliders de Motores):** Panel interactivo de pruebas de motores adaptado para el firmware PX4 / ArduPilot con parada de emergencia instantánea (Tecla `Espacio`).
* **Sintonizador PID:** Modificación y lectura de parámetros de control en vivo guardando directamente en la memoria flash del autopiloto.
* **Simulador CSV Integrado:** Reproducción de vuelos grabados paso a paso con control de velocidad (0.5x, 1x, 2x, 4x).

---

## 🛠️ Tecnologías Utilizadas

* **Backend:** Python 3.10+, Flask, PyMAVLink, PySerial
* **Frontend:** HTML5, CSS3 Glassmorphism, JavaScript (ES6)
* **Librerías de Visualización:** Chart.js v4.5.1, Leaflet.js v1.9.4
* **Hardware:** Holybro Pixhawk 6X (PX4 / ArduPilot) & Holybro X650 Quadcopter

---

## 🔌 Conexión del Hardware y Selección de Puertos

1. Conecta tu **Pixhawk 6X** o radio módem SiK al computador mediante USB o telemetría 915 MHz / 433 MHz.
2. Inicia la aplicación y abre `http://localhost:5000`.
3. En la barra superior de conexión:
   - Selecciona tu puerto en el menú desplegable (ej. `COM7`, `COM3`, `COM4`).
   - Si tu puerto no aparece listado, elige **`✏️ Otro / Escribir puerto...`** y escribe directamente el puerto asignado (ej: `COM8`, `COM12`, `/dev/ttyUSB0`).
   - También puedes presionar el botón **`🔌 Web Serial`** para que tu navegador detecte y vincule el puerto COM USB automáticamente.
4. Selecciona los baudios (habitualmente `57600` para radio o `115200` / `921600` para USB) y haz clic en **`🔌 Conectar`**.

---

## 🗺️ Guía del Planificador de Misiones (Setpoints)

1. Dirígete a la pestaña **`🗺️ Planificador de Misión (Setpoints)`**.
2. **Agregar Setpoints**: Haz clic en cualquier ubicación del mapa para agregar waypoints numerados (`WP1`, `WP2`, `WP3`...).
3. **Modificar Parámetros**: En la tabla derecha, puedes cambiar el tipo de comando (`TAKEOFF`, `WAYPOINT`, `LOITER`, `RTL`, `LAND`) y la altitud objetivo en metros.
4. **Reordenar / Arrastrar**: Arrastra directamente los marcadores en el mapa para ajustar coordenadas o usa los botones `⬆️` / `⬇️` en la tabla.
5. **Enviar al Dron**: Haz clic en **`🚀 Enviar al Dron`** para transmitir la misión al Pixhawk vía MAVLink.
6. **Ejecutar Misión**: Presiona **`▶ Iniciar (AUTO)`** para cambiar el modo del dron a `AUTO` y comenzar la ruta.
7. **Integración con QGroundControl**: Usa **`💾 Exportar QGC .plan`** o **`📂 Importar QGC`** para compartir misiones con QGroundControl.

---

## 👁️ Sistema de Visión IA YOLOv8 & Streaming FPV

1. Dirígete a la pestaña **`👁️ Visión IA & Simulación Unreal`**.
2. **Seleccionar Entrada de Video**:
   - **Objetivo Sintético (Test)**: Ideal para pruebas de escritorio sin hardware conectado; genera un objetivo móvil 3D con retícula táctica y métricas.
   - **Dron Real RTSP**: Transmite desde la Jetson Nano del dron (`rtsp://192.168.14.7:8554/cam0`).
   - **Simulación Unreal**: Ingiere el stream virtual generado por Unreal Engine (`rtsp://127.0.0.1:8554/live` o virtual camera).
   - **Webcam USB / OBS**: Usa cualquier dispositivo de captura conectado localmente.
3. **Ajuste de Inferencia YOLO**:
   - Alterna entre el modelo especializado en drones (`drone_finetuned-3`), el modelo stock de personas (`yolov8n`), o inferencia dual.
   - Ajusta el umbral de confianza mínimo (10% a 90%) en caliente con el control deslizante.
   - Activa o silencia las alertas de voz (TTS).
4. **Mini FPV Flotante (PiP)**:
   - Presiona el botón **`📷 Mini FPV`** en el PFD o en el panel de visión para mantener visible la cámara mientras navegas por otras pestañas.

---

## 🎮 Simulación en Unreal Engine con PX4 SITL & Seguimiento PID

1. **Vincular Simulación**:
   - Haz clic en el botón superior **`🎮 Preset Unreal SITL`**.
   - Esto autoconfigura la conexión MAVLink en el puerto **UDP 14550** y conmuta la fuente de video a Unreal Engine.
2. **Modos de Seguimiento Visual (Image PID)**:
   - **OFF**: Visualización pasiva de cámara y detecciones.
   - **SIM (Visual)**: El PID calcula los errores de centrado horizontal ($e_x$), vertical ($e_y$) y distancia ($e_{area}$), actualizando el HUD táctico y los indicadores de velocidad **sin mover los motores**.
   - **ACTIVE TRACK (MAVLink)**: El sistema envía activamente comandos de velocidad (`SET_POSITION_TARGET_LOCAL_NED`) al autopiloto en modo `OFFBOARD` / `GUIDED` para mantener el dron apuntando y centrado hacia el objetivo.
3. **Seguridad del Seguimiento**:
   - **Watchdog de Pérdida**: Si el objetivo sale del campo visual por más de 3.0 segundos, las velocidades se resetean automáticamente a cero (el dron mantiene posición).
   - **Parada de Emergencia**: El botón **`🛑 CORTE MOTOR`** o la tecla **`Espacio`** fuerzan el desarmado inmediato del vehículo.

---

## 🗺️ Ejecución Autónoma de Misiones y Setpoints en Unreal Engine (Parque O'Higgins)

1. **Abrir el Proyecto Unreal**:
   - Haz doble clic en el acceso directo **`iniciar_unreal_parque_ohiggins.bat`** (en tu Escritorio o en la raíz).
   - Abrirá Unreal Engine 5.8 con el proyecto **`MyProject`**, cargando el mapa 3D fotorrealista de **Parque O'Higgins** y el dron **Holybro X650**.
   - En Unreal Editor, presiona el botón verde **Play (Alt+P)** para arrancar el entorno de simulación AirSim RPC.
2. **Planificar la Ruta de Vuelo**:
   - En la GCS, abre la pestaña **`🗺️ Planificador de Misión (Setpoints)`**.
   - Haz clic sobre el mapa en los puntos deseados de Parque O'Higgins. El sistema creará automáticamente los waypoints numerados con altitud y velocidad configurables.
3. **Ejecutar la Misión Virtual**:
   - Presiona el botón cyan **`🎮 Volar en Unreal (AirSim)`**.
   - El dron Holybro X650 despegará de inmediato en Parque O'Higgins y navegará secuencialmente cada setpoint.
   - En la barra superior verás en vivo el **punto actual**, la **distancia restante**, el **porcentaje de avance** y la posición del dron moviéndose en tiempo real sobre el mapa Leaflet y los instrumentos PFD.
   - Si deseas abortar en cualquier momento, presiona **`🛑 Abortar Simulador`** para frenar y mantener el dron en hover.

---

## ⚠️ Advertencia de Seguridad Crítica

* Si vas a realizar pruebas en el **Banco de Actuadores (Motores)**, **retira las hélices físicamente** del dron para evitar accidentes.
* Presiona la tecla **`Espacio`** en cualquier momento para activar la parada de emergencia instantánea.

---

## 🚀 Subir Cambios a GitHub

Para respaldar tu código en GitHub, simplemente haz doble clic en el archivo **`push_to_github.bat`**. El script detectará Git, creará el commit y subirá los cambios a tu repositorio automáticamente.
