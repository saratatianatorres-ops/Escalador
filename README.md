# Robot trepador – proyecto completo

```
[PC: ROS 2 + interfaz + micro_ros_agent] ⇄ USB 921600 ⇄ [ESP32 puente] ⇄ ESP-NOW ⇄ [ESP32 robot: nodo micro-ROS]
```

| Carpeta | Qué es |
|---|---|
| `firmware/robot_receptor/` | ESP32 del robot: nodo micro-ROS, ESC (pin 18), DRV8833, MPU6050 |
| `firmware/puente_pc/` | ESP32 del PC: copia bytes entre el USB y ESP-NOW (sin micro-ROS) |
| `ros2_ws/src/robot_trepador_gui/` | Paquete ROS 2: interfaz gráfica (Python: rclpy + Tkinter) y launch que arranca el agente |
| `ros2_ws/src/robot_trepador_description/` | Modelo URDF del robot y RViz (con el robot real, orientado por la IMU) |
| `ros2_ws/src/robot_trepador_gazebo/` | Simulación en Gazebo Harmonic: habitación con suelo, pared y techo |

## 1. Subir los firmwares (PlatformIO)

```bash
cd firmware/robot_receptor && pio run -t upload    # primera vez: tarda (compila micro-ROS)
cd ../puente_pc        && pio run -t upload
```

Las dos ESP usan el canal 1 y se encuentran solas (no hace falta copiar MACs).
`board_microros_distro` en el robot debe ser tu distro de ROS 2 (viene en `jazzy`).

## 2. Preparar el PC (una vez)

```bash
sudo apt install python3-tk ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-ros-gz
# micro_ros_agent: https://github.com/micro-ROS/micro_ros_setup (o el contenedor Docker microros/micro-ros-agent)
cd ros2_ws
colcon build
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

## 4. Ver el robot real en RViz

```bash
ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0 rviz:=true
```

El modelo 3D se orienta con la IMU y avanza con el PWM que el robot reporta en `/robot/estado`
(nodo `odometria_imu`). La orientación es real; la posición es una estimación (sin encoders se
desvía con el tiempo y el yaw deriva). Para poner la posición a cero:
`ros2 topic pub --once /robot/odom_reiniciar std_msgs/msg/Bool "{data: true}"`.

## 5. Simulación en Gazebo (sin el robot)

```bash
ros2 launch robot_trepador_gazebo simulacion.launch.py
```

Abre Gazebo con la habitación (pared a 0,8 m delante, techo a 1 m), RViz y **la misma interfaz**.
El nodo `firmware_simulado` hace lo que haría el ESP32: armado de 3 s, rampa del ESC, timeouts,
parada de emergencia, PWM mínimo de los N20 y `/robot/estado` con el mismo JSON. Así la interfaz
no sabe si habla con el robot o con Gazebo.

Opciones: `rviz:=false`, `interfaz:=false`, `gazebo_gui:=false` (solo servidor),
`empuje_max_n:=12.0` (N por hélice a 2000 µs), `x:= y:= yaw:=` (dónde aparece),
`reparto_delantera:=0.0` (apaga la hélice delantera y deja la trasera; hoy los ESC van en
paralelo, así que el robot real siempre usa 1.0).

Medidas del modelo (en `robot_trepador.urdf.xacro`, fáciles de cambiar): 0,8 kg, 4 ruedas de 35 mm
con ejes a ±9,5 cm y vía de 15 cm, 0,2 N·m por rueda, fricción 1,0, hélices a +4 cm y −9 cm,
10 N por hélice al 100 %.

**Probar en la pared:** pon las hélices al máximo con el robot en el suelo y luego colócalo en la pared:

```bash
gz service -s /world/habitacion/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'name: "escalador", position: {x: 0.762, z: 0.4}, orientation: {w: 0.7071, y: -0.7071}'
```

Se sostiene con las hélices y sube con ▲ hasta la esquina del techo.

### Qué reproduce Gazebo y qué no

| Prueba | Resultado en Gazebo |
|---|---|
| Andar por el suelo, girar, hélices con rampa, parada | ✔ |
| Quedarse pegado a la pared con las hélices y subir por ella | ✔ |
| Llegar a la pared con los ESC en paralelo | Se queda contra la pared y se desvía de lado (igual que en MuJoCo) |
| Pasar del suelo a la pared o de la pared al techo (esquinas) | ✘ No pasa, ni con `reparto_delantera:=0.0` |

En las esquinas, la rueda delantera gira en falso contra la superficie nueva: el motor de contactos
de Gazebo (DART) casi no le da fricción cuando resbala tocando dos superficies a la vez. Se probó
con ruedas esféricas, más par (1 N·m), el detector de colisiones bullet, el motor Bullet y el
solver PGS, sin que cambie (o volviéndose inestable). Las transiciones hay que validarlas con las
simulaciones de MuJoCo o con el robot real; Gazebo sirve para todo lo demás.
