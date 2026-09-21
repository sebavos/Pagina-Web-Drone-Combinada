# Guía de Ejecución y Operación del Sistema de Simulación

## Dron Holybro X650 + PX4 SITL (WSL2) + Unreal Engine 5.5 (Cesium 3D) + GCS Web YOLOv8

Esta guía documenta la arquitectura completa y el procedimiento paso a paso para ejecutar el ecosistema de simulación y control terrestre.

---

## 1. Arquitectura del Sistema

```
+-----------------------------------------------------------------------------------+
|                                 SISTEMA COMPLETO                                  |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|  [WSL2: Ubuntu 24.04]                                                             |
|  PX4 Autopilot SITL (make px4_sitl none_iris)                                     |
|     │                                                                             |
|     ├── TCP Port 4560  ────────► [Unreal Engine 5.5: AirSim Plugin]              |
|     │                            - Mapa 3D: Parque O'Higgins (Cesium Fotogrametría)|
|     │                            - Cámaras HD: 1280x720, Gamma 2.2, 60 FPS        |
|     │                            - Servidor RPC: 127.0.0.1:41451                  |
|     │                                                                             |
|     └── UDP Port 14550 ────────► [GCS Web / Backend Python: app.py]               |
|                                  - Telemetría MAVLink 20-300 Hz                   |
|                                  - Captura AirSim RPC HD                          |
|                                  - Detección IA YOLOv8 (CUDA RTX 4060)            |
|                                  - Servidor Flask + SSE: http://localhost:5000     |
|                                                                                   |
+-----------------------------------------------------------------------------------+
```

---

## 2. Mejoras de Calidad Visual y Rendimiento Aplicadas

1. **Eliminación del límite de 3 FPS en segundo plano de Unreal Engine:**
   - Unreal Engine tiene por defecto activada la directiva `bThrottleCPUWhenNotForeground=True`, la cual reduce el motor a **3.0 FPS** cuando la ventana pierde el foco (por ejemplo, al interactuar con el navegador web).
   - Se configuró `bThrottleCPUWhenNotForeground=False` y `bMonitorEditorPerformance=False` en las configuraciones del proyecto (`DefaultEditorSettings.ini`, `DefaultEditorPerProjectUserSettings.ini`, `EditorSettings.ini`).
   - Con esto, Unreal Engine y el streaming de texturas de Cesium se mantienen corriendo a **60 FPS constantes** incluso con el navegador en primer plano.

2. **Cámaras Virtuales HD en `settings.json`:**
   - Resolución incrementada de 480p (854x480) a **720p HD (1280x720)**.
   - Cesium calcula el nivel de detalle (LOD / SSE) en base a la resolución vertical de la cámara. A 720p, Cesium solicita y renderiza teselas fotogramétricas de alta resolución y mipmaps nítidos.
   - Se incorporó `"TargetGamma": 2.2`, eliminando la imagen pálida o deslavada del espacio lineal y logrando el mismo contraste, brillo y saturación que el viewport de Unreal.

3. **Pipeline de Transmisión Web Optimizado:**
   - `app.py` transmite ahora a 720p nativo sin comprimir excesivamente los cuadros.
   - Calidad de codificación JPEG incrementada de 75 a **88%** (`cv2.IMWRITE_JPEG_QUALITY, 88`), eliminando bordes pixelados y desenfoques.
   - Inferencia de YOLOv8 optimizada con aceleración por GPU (`device=0`) a más de 70 FPS.

4. **Sistema de Cámara Táctica / Gimbal de 3 Posiciones:**
   - **Frontal (FPV 0°):** Vista al horizonte para vuelo manual y navegación.
   - **Inclinada 45°:** Vista de vigilancia táctica hacia adelante y abajo (libre de patas y hélices).
   - **Cenital / Suelo (-90° Nadir):** Vista perpendicular al suelo para ortofotografía, inspección y aterrizaje.

---

## 3. Pasos de Ejecución (Paso a Paso)

### Paso 1: Iniciar PX4 SITL en WSL2

Abre el script lanzador o una terminal de WSL2:

- **Opción A (Recomendada - Script rápido):**
  Haz doble clic en:
  ```
  iniciar_px4_sitl.bat
  ```

- **Opción B (Manual desde PowerShell o terminal WSL):**
  ```bash
  wsl -d Ubuntu-24.04
  cd ~/PX4-Autopilot
  make px4_sitl none_iris
  ```
  *Nota: Espera hasta que aparezca el prompt interactivo de PX4: `pxh> `.*

---

### Paso 2: Iniciar Unreal Engine 5.5

- **Opción A (Recomendada):**
  Haz doble clic en:
  ```
  iniciar_unreal_parque_ohiggins.bat
  ```

- **Opción B (Manual):**
  Abre Unreal Engine 5.5 y carga el proyecto:
  ```
  C:\Users\matias.hinrichsen\Desktop\MyProject\MyProject\MyProject.uproject
  ```

- **Iniciar la Simulación:**
  1. El proyecto abrirá automáticamente el mapa completo: `parque_ohiggins_full.umap`.
  2. Haz clic en el botón **Play** o presiona la combinación de teclas **`Alt + P`**.
  3. En la esquina superior izquierda aparecerán los mensajes verdes de AirSim indicando la conexión con PX4 en el puerto 4560.

---

### Paso 3: Iniciar la Estación Terrena (GCS Web)

- **Opción A (Recomendada):**
  Haz doble clic en:
  ```
  run.bat
  ```

- **Opción B (Manual):**
  Abre una terminal en la carpeta del proyecto y ejecuta:
  ```powershell
  python app.py
  ```

El servidor web quedará escuchando en `http://localhost:5000`.

---

### Paso 4: Operación desde la Interfaz Web

1. Abre tu navegador web en: **[http://localhost:5000](http://localhost:5000)**.
2. **Conexión a la Simulación:**
   - En el panel de conexión superior o mediante el botón de preset, selecciona la opción **"Simulación Unreal + PX4 SITL"**.
   - El sistema se conectará automáticamente a:
     - Telemetría MAVLink: `udpin:0.0.0.0:14550`
     - Video HD AirSim: `127.0.0.1:41451`
3. **Control de Vuelo:**
   - Haz clic en **"Armar"** para armar los motores del dron.
   - Haz clic en **"Despegar (Takeoff)"** para elevar el dron automáticamente a la altitud predeterminada (15 metros).
4. **Alternar Cámaras:**
   - En el visor de video superior, utiliza los tres botones tácticos:
     - **📷 Frontal (0°)**
     - **📐 45° (Inclinada)**
     - **⬇️ Cenital (-90°)**
5. **Prueba de Detección de Drones con YOLOv8:**
   - Haz clic en el botón **"🎯 Spawn Dron Objetivo"**.
   - Aparecerá un segundo dron estático a 7 metros de distancia para validar los cuadros de delimitación (bounding boxes), el porcentaje de confianza y el seguimiento PID.
6. **Navegación de Misiones (Waypoints):**
   - En la pestaña de Misiones, define setpoints sobre el mapa satelital del Parque O'Higgins o importa un plan de QGroundControl.
   - Haz clic en **"Volar en Unreal (AirSim)"** para ejecutar la misión de waypoints de forma autónoma.

---

## 4. Solución de Problemas Frecuentes

| Síntoma | Causa | Solución |
| :--- | :--- | :--- |
| **Video en negro o "Esperando Unreal Engine"** | La simulación en Unreal Editor no está corriendo. | Presiona **`Alt + P`** en la ventana de Unreal Editor para iniciar la simulación. |
| **Telemetría desconectada (0 Hz)** | PX4 SITL no está corriendo en WSL2. | Ejecuta `iniciar_px4_sitl.bat` y verifica que aparezca el prompt `pxh>`. |
| **"Address already in use" en puerto 5000** | Una instancia previa de `app.py` sigue corriendo. | Ejecuta en PowerShell: `Stop-Process -Name python -Force` y luego vuelve a ejecutar `run.bat`. |
| **El dron no responde a Takeoff** | El dron no está en modo POSCTL o no tiene Fix 3D. | Espera unos segundos a que el GPS de PX4 sincronice y luego presiona **Armar** seguido de **Takeoff**. |
