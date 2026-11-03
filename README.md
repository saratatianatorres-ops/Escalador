# Escalador: robot trepador por succión con hélices

Robot de tracción diferencial que se pega a paredes y techos con el empuje de dos hélices brushless.
Lo controla una ESP32 con **micro-ROS sobre ESP-NOW** (sin WiFi ni router) y se maneja desde una interfaz
de ROS 2 en el PC. Incluye modelo 3D para **RViz** y simulación en **Gazebo** con la misma interfaz.

```
[PC: ROS 2 + interfaz + micro_ros_agent] ⇄ USB 921600 ⇄ [ESP32 puente] ⇄ ESP-NOW ⇄ [ESP32 robot: nodo micro-ROS]
```

**Hardware:** 2 motores brushless A2217 con 2 ESC 30A SimonK (señal compartida en IO18) · 4 motorreductores N20
(dos por lado) con un DRV8833 · IMU MPU6050 · ESP32 DevKitC-32D · regulador LM2596.
Detalles en [hardware/](hardware/README.md).

---

## Estructura del repositorio

```text
Escalador/
├── README.md
├── firmware/
│   ├── robot_receptor/        # ESP32 del robot: nodo micro-ROS, ESC, DRV8833, MPU6050 (PlatformIO)
│   └── puente_pc/             # ESP32 del PC: copia bytes entre el USB y ESP-NOW
├── ros2_ws/src/
│   ├── robot_trepador_gui/          # Interfaz de control (Python: rclpy + Tkinter) y launch del robot real
│   ├── robot_trepador_description/  # Modelo URDF, RViz y odometría con la IMU
│   └── robot_trepador_gazebo/       # Mundo, puente ROS-Gazebo y firmware simulado
├── hardware/                  # Esquemático, PCB, conexiones y lista de materiales
├── docs/                      # Arquitectura, firmware, física, simulación e instalación
└── scripts/                   # Instalación de dependencias
```

| Documento | Contenido |
|---|---|
| [docs/arquitectura.md](docs/arquitectura.md) | Grafo de nodos y tópicos, seguridad y próximo paso (supervisor) |
| [docs/firmware.md](docs/firmware.md) | Firmware de las dos ESP32, tópicos, JSON de estado, protocolo ESP-NOW |
| [docs/fisica_transiciones.md](docs/fisica_transiciones.md) | Qué hace falta para pasar del suelo a la pared y al techo |
| [docs/simulacion.md](docs/simulacion.md) | RViz con el robot real y simulación en Gazebo (qué reproduce y qué no) |
| [docs/instalacion_vm.md](docs/instalacion_vm.md) | Ubuntu + ROS 2 + micro-ROS + PlatformIO paso a paso (VirtualBox) |
| [hardware/README.md](hardware/README.md) | Diagrama de bloques, pines y puntos a revisar en la placa |
| [hardware/lista_materiales.md](hardware/lista_materiales.md) | Componentes, pesos y precios |

---

## Paquetes de ROS 2

### `robot_trepador_gui`
- Nodo `interfaz`: ruedas (▲ ▼ ◀ ▶ o W A S D), hélices (+ / −), batería, PARADA (espacio) y vista 3D de la IMU.
- `robot.launch.py`: arranca el `micro_ros_agent` y la interfaz; con `rviz:=true`, también RViz.

### `robot_trepador_description`
- `urdf/robot_trepador.urdf.xacro`: chasis, 4 ruedas, 2 hélices e IMU con las medidas del robot.
  Con `sim:=true` añade los plugins de Gazebo.
- Nodo `odometria_imu`: pone el robot real en RViz (orientación de la IMU, avance según el PWM).

### `robot_trepador_gazebo`
- `worlds/habitacion.sdf`: suelo, pared a 0,8 m y techo a 1 m.
- Nodo `firmware_simulado`: hace lo mismo que el ESP32 (armado, rampa, timeouts, parada, JSON de estado),
  así que la interfaz funciona igual con el robot real o con Gazebo.
- `simulacion.launch.py`: Gazebo + robot + puente + RViz + interfaz.

---

## Puesta en marcha

### 1. Instalar (una vez)

Con Ubuntu 24.04 y ROS 2 Jazzy (si no los tienes: [docs/instalacion_vm.md](docs/instalacion_vm.md)):

```bash
./scripts/instalar_dependencias.sh
source ros2_ws/install/setup.bash
```

### 2. Subir los firmwares (PlatformIO)

```bash
cd firmware/robot_receptor && pio run -t upload    # la primera vez tarda (compila micro-ROS)
cd ../puente_pc        && pio run -t upload
```

Las dos ESP usan el canal 1 y se encuentran solas (no hace falta copiar MACs).

### 3. Usar el robot

1. Conecta la ESP puente por USB y enciende el robot. **Déjalo quieto unos 5 s** (armado de los ESC y
   calibración de la IMU).
2. Lanza todo:
   ```bash
   ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0            # añade rviz:=true para ver el modelo 3D
   ```
3. En la interfaz:
   - **▲ ▼ ◀ ▶** (mantener pulsado) o flechas / W A S D: adelante, atrás y giro sobre sí mismo. Los
     deslizadores fijan la velocidad.
   - **+ / −**: hélices de 100 en 100 µs entre 1000 y **1700 como máximo**. Es el límite para que los A2217
     no pasen de ~23–25 A; el robot lo aplica aunque le manden más. El robot sube en rampa;
     "en el robot" muestra el valor real.
   - **Batería**: barra con el porcentaje, el voltaje total y por celda; se pone amarilla con batería baja y
     roja (parpadeando) en estado crítico. Necesita el divisor de tensión en el IO34
     ([hardware/README.md](hardware/README.md#medición-de-la-batería-agregar-a-la-placa)).
   - **PARADA** (o espacio): apaga hélices y ruedas y bloquea el robot hasta pulsar LIBERAR.
   - Vista 3D: la cara naranja es el frente. "Yaw a cero" alinea el giro, que deriva porque el MPU6050 no
     tiene brújula.

Si la interfaz se cierra o se pierde la conexión, el robot apaga las hélices en 2 s y frena las ruedas en 0,5 s.

### 4. Simular sin el robot

```bash
ros2 launch robot_trepador_gazebo simulacion.launch.py
```

Opciones y límites de la simulación en [docs/simulacion.md](docs/simulacion.md).
