# Robot trepador – firmware del receptor (micro-ROS sobre ESP-NOW)

La ESP32 del robot es un **nodo micro-ROS**. No usa WiFi ni cable: sus mensajes
XRCE-DDS viajan por ESP-NOW hasta la ESP32 puente (la del PC), que solo los copia
a su USB. En el PC corre el agente normal de micro-ROS y todo aparece como tópicos de ROS 2.

```
[PC: ROS 2 + micro_ros_agent] ⇄ USB 921600 ⇄ [ESP32 puente] ⇄ ESP-NOW ⇄ [ESP32 robot: nodo micro-ROS]
```

## Estructura

| Archivo | Qué hace |
|---|---|
| `platformio.ini` | Proyecto PlatformIO con micro_ros_platformio, transporte `custom` |
| `src/config.h` | Pines, canal ESP-NOW, límites del ESC, timeouts, parámetros del IMU |
| `src/transporte_espnow.*` | Transporte personalizado de micro-ROS sobre ESP-NOW |
| `src/actuadores.*` | ESC (pin 18, rampa y timeout) y DRV8833 (PWM 20 kHz, decaimiento lento) |
| `src/imu.*` | MPU6050 por I2C, calibración del giro y filtro de Mahony |
| `src/main.cpp` | Nodo micro-ROS: tópicos, reconexión automática y seguridad |

## Tópicos

| Tópico | Tipo | Dirección | Uso |
|---|---|---|---|
| `/robot/esc` | `std_msgs/Int32` | PC → robot | µs para los ESC. `1000` apaga al instante; `1100`–`2000` sube/baja en rampa (20 µs cada 0,2 s) |
| `/robot/cmd_vel` | `geometry_msgs/Twist` | PC → robot | `linear.x` (m/s) y `angular.z` (rad/s). Lazo abierto: ajusta `V_MAX_MS` y `ANCHO_VIA_M` |
| `/robot/ruedas` | `std_msgs/Float32MultiArray` | PC → robot | `[izq, der]` de −1 a 1, PWM directo |
| `/robot/parada` | `std_msgs/Bool` | PC → robot | `true` = parada de emergencia enclavada; `false` la libera |
| `/robot/imu` | `sensor_msgs/Imu` | robot → PC | Orientación, giro y aceleración a 50 Hz (`frame_id = imu_link`) |
| `/robot/estado` | `std_msgs/String` (JSON) | robot → PC | Eco de todo lo recibido y aplicado, a 5 Hz |

Ejemplo de `/robot/estado`:

```json
{"t":53210,"parada":false,
 "esc":{"ord":1500,"obj":1500,"act":1340,"ms":120,"n":87},
 "ruedas":{"izq":0.50,"der":0.50,"pwm_izq":661,"pwm_der":661,"ms":40,"v":0.15,"w":0.00,"n_vel":310,"n_dir":0},
 "imu":{"ok":true,"roll":1.2,"pitch":-0.4,"yaw":12.8,"temp":31.5,"err":0},
 "enlace":{"rx":4120,"tx":9803,"fallos":3,"desc":0,"conex":1}}
```

- `esc.ord` última orden recibida, `obj` objetivo, `act` lo que se está mandando al ESC ahora (rampa), `ms` tiempo desde la última orden, `n` órdenes recibidas.
- `ruedas.izq/der` orden normalizada, `pwm_*` duty aplicado con signo (0–1023).
- `enlace` paquetes ESP-NOW recibidos/enviados, envíos fallidos, bytes perdidos por buffer lleno y número de conexiones al agente.

## Seguridad

- **ESC**: si no llega `/robot/esc` en 2 s → hélices a 1000 µs. Publica la orden de forma periódica (`-r 5`), igual que el transmisor original reenviaba cada 0,5 s.
- **Ruedas**: sin órdenes en 0,5 s → freno.
- **Agente o enlace perdido** (3 pings fallidos) → todo apagado, y se reconecta solo.
- **Al arrancar**: ESC a 1000 µs durante 3 s (armado) y ruedas frenadas.
- El IMU se calibra al encender: **deja el robot quieto ~2 s** tras el armado.

## Compilar y subir

```bash
pio run -t upload
pio device monitor          # el USB del robot queda libre para depurar
```

La primera compilación tarda varios minutos porque construye la librería de micro-ROS.
`board_microros_distro` tiene que coincidir con la distro de ROS 2 del PC; si la cambias:
`pio run -t clean_microros`.

## Usar desde el PC

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

ros2 topic list
ros2 topic echo /robot/estado
ros2 topic echo /robot/imu

ros2 topic pub -r 5 /robot/esc std_msgs/msg/Int32 "{data: 1300}"
ros2 topic pub -r 10 /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub -r 10 /robot/ruedas std_msgs/msg/Float32MultiArray "{data: [0.5, -0.5]}"
ros2 topic pub --once /robot/parada std_msgs/msg/Bool "{data: true}"
```

Para mover con el teclado: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot/cmd_vel`.
Para ver la orientación: `rqt_plot` sobre `/robot/imu`, o RViz2 con el display *Imu* (plugin `rviz_imu_plugin`).

## Protocolo con la ESP puente

Cada paquete ESP-NOW es `[cabecera][hasta 249 bytes]`:

| Cabecera | Sentido | Contenido |
|---|---|---|
| `0xA1` | robot → puente | bytes XRCE-DDS con framing serie: el puente los escribe tal cual en el USB |
| `0xA2` | puente → robot | bytes leídos del USB, partidos en trozos de ≤249 |

- Ambas ESP en el **canal 1** (`CANAL_ESPNOW`).
- MAC: si `MAC_PUENTE` en `config.h` se deja en `FF:FF:FF:FF:FF:FF`, el robot transmite en broadcast hasta que recibe el primer paquete `0xA2` y desde ahí solo habla con esa ESP. El robot imprime su propia MAC al arrancar, para configurarla en el puente.
- El framing XRCE tiene CRC: si se pierde un paquete, el cliente lo detecta y reintenta.

## Pendiente

- **Las dos hélices comparten el pin 18**, así que reciben siempre el mismo acelerador. Según `fisica_transiciones.md`, pasar de pared a techo necesita repartir el empuje (delantera casi a cero, trasera al máximo). Para eso cada ESC necesita su propio pin (p. ej. 18 y 19); el cambio en el firmware es pequeño.
- Sin encoders, `cmd_vel` es lazo abierto.
