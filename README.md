# Robot trepador – proyecto completo

```
[PC: ROS 2 + interfaz + micro_ros_agent] ⇄ USB 921600 ⇄ [ESP32 puente] ⇄ ESP-NOW ⇄ [ESP32 robot: nodo micro-ROS]
```

| Carpeta | Qué es |
|---|---|
| `firmware/robot_receptor/` | ESP32 del robot: nodo micro-ROS, ESC (pin 18), DRV8833, MPU6050 |
| `firmware/puente_pc/` | ESP32 del PC: copia bytes entre el USB y ESP-NOW (sin micro-ROS) |
| `ros2_ws/src/robot_trepador_gui/` | Paquete ROS 2: interfaz gráfica y launch que arranca el agente |

## 1. Subir los firmwares (PlatformIO)

```bash
cd firmware/robot_receptor && pio run -t upload    # primera vez: tarda (compila micro-ROS)
cd ../puente_pc        && pio run -t upload
```

Las dos ESP usan el canal 1 y se encuentran solas (no hace falta copiar MACs).
`board_microros_distro` en el robot debe ser tu distro de ROS 2 (viene en `jazzy`).

## 2. Preparar el PC (una vez)

```bash
sudo apt install python3-tk
# micro_ros_agent: https://github.com/micro-ROS/micro_ros_setup (o el contenedor Docker microros/micro-ros-agent)
cd ros2_ws
colcon build --packages-select robot_trepador_gui
source install/setup.bash
```

## 3. Usar

1. Conecta la ESP puente por USB y enciende el robot. **Déjalo quieto ~5 s** (armado de ESC + calibración de la IMU).
2. Lanza todo:
   ```bash
   ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0
   ```
3. En la interfaz:
   - **▲ ▼ ◀ ▶** (mantener pulsado) o flechas / W A S D: adelante, atrás y giro sobre sí mismo (robot diferencial). Los deslizadores fijan la velocidad.
   - **+ / −** (o teclas + y −): hélices de 100 en 100 µs entre 1000 y 1800. El robot sube en rampa; "en el robot" muestra el valor real.
   - **PARADA** (o espacio): apaga hélices y ruedas y bloquea el robot hasta pulsar LIBERAR.
   - Vista 3D: la caja es el robot (cara naranja = delante, flecha = sentido de avance). "Yaw a cero" alinea el giro, que deriva porque el MPU6050 no tiene brújula.

Si la interfaz se cierra o se pierde la conexión, el robot apaga las hélices en 2 s y frena las ruedas en 0,5 s.

Para monitorear todo desde otra terminal: `ros2 topic echo /robot/estado`, `ros2 topic echo /robot/imu`, `ros2 topic echo /robot/cmd_vel`, `ros2 topic echo /robot/esc`.
