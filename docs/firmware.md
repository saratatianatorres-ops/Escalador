# Firmware (ESP32): micro-ROS sobre ESP-NOW

La ESP32 del robot es un **nodo micro-ROS**. No usa WiFi ni cable: sus mensajes XRCE-DDS viajan por
ESP-NOW hasta la ESP32 puente (la del PC), que solo los copia a su USB. En el PC corre el agente normal
de micro-ROS y todo aparece como tópicos de ROS 2.

```
[PC: ROS 2 + micro_ros_agent] ⇄ USB 921600 ⇄ [ESP32 puente] ⇄ ESP-NOW ⇄ [ESP32 robot: nodo micro-ROS]
```

## Estructura (`firmware/robot_receptor/`)

| Archivo | Qué hace |
|---|---|
| `platformio.ini` | Proyecto PlatformIO con micro_ros_platformio y transporte `custom` |
| `src/config.h` | Pines, canal ESP-NOW, límites del ESC, timeouts y parámetros de la IMU |
| `src/transporte_espnow.*` | Transporte personalizado de micro-ROS sobre ESP-NOW |
| `src/actuadores.*` | ESC (pin 18, rampa y timeout) y DRV8833 (PWM 20 kHz, decaimiento lento) |
| `src/imu.*` | MPU6050 por I2C, calibración del giro y filtro de Mahony |
| `src/bateria.*` | Tensión de la batería (divisor en IO34), porcentaje por celda y avisos |
| `src/main.cpp` | Nodo micro-ROS: tópicos, reconexión automática y seguridad |

`firmware/puente_pc/` es la ESP32 del PC: copia bytes entre el USB y ESP-NOW, sin micro-ROS.

Los pines coinciden con el esquemático ([hardware/README.md](../hardware/README.md)):
ESC = IO18 · DRV8833 AIN1 = IO25, AIN2 = IO26, BIN1 = IO33, BIN2 = IO32 · MPU6050 SDA = IO21, SCL = IO22 ·
batería = IO34.
Si un lado de motores gira al revés, cambia `MOTOR_IZQ_INVERTIDO` o `MOTOR_DER_INVERTIDO` en `config.h`.

## Tópicos

| Tópico | Tipo | Dirección | Uso |
|---|---|---|---|
| `/robot/esc` | `std_msgs/Int32` | PC → robot | µs para los ESC. `1000` apaga al instante; `1100`–`2000` sube o baja en rampa (20 µs cada 0,2 s) |
| `/robot/cmd_vel` | `geometry_msgs/Twist` | PC → robot | `linear.x` (m/s) y `angular.z` (rad/s). Lazo abierto: ajusta `V_MAX_MS` y `ANCHO_VIA_M` |
| `/robot/ruedas` | `std_msgs/Float32MultiArray` | PC → robot | `[izq, der]` de −1 a 1, PWM directo |
| `/robot/parada` | `std_msgs/Bool` | PC → robot | `true` = parada de emergencia enclavada; `false` la libera |
| `/robot/imu` | `sensor_msgs/Imu` | robot → PC | Orientación, giro y aceleración a 50 Hz (`frame_id = imu_link`) |
| `/robot/estado` | `std_msgs/String` (JSON) | robot → PC | Eco de todo lo recibido y aplicado, y la batería, a 5 Hz |

Ejemplo de `/robot/estado`:

```json
{"t":53210,"parada":false,
 "esc":{"ord":1500,"obj":1500,"act":1340,"ms":120,"n":87},
 "ruedas":{"izq":0.50,"der":0.50,"pwm_izq":661,"pwm_der":661,"ms":40,"v":0.15,"w":0.00,"n_vel":310,"n_dir":0},
 "imu":{"ok":true,"roll":1.2,"pitch":-0.4,"yaw":12.8,"temp":31.5,"err":0},
 "enlace":{"rx":4120,"tx":9803,"fallos":3,"desc":0,"conex":1},
 "bat":{"ok":true,"v":7.82,"vc":3.91,"pct":72,"nivel":0,"celdas":2}}
```

- `esc.ord` es la última orden recibida, `obj` el objetivo, `act` lo que se está mandando al ESC ahora
  (rampa), `ms` el tiempo desde la última orden y `n` las órdenes recibidas.
- `ruedas.izq/der` es la orden normalizada y `pwm_*` el duty aplicado con signo (0–1023).
- `bat`: `ok` es false si no hay divisor en el IO34; `v` es la tensión total (filtrada, ~1 s); `vc` la
  tensión por celda; `pct` el porcentaje estimado; `nivel` vale 0 (bien), 1 (baja, < 3,5 V/celda) o
  2 (crítica, < 3,3 V/celda).
- `enlace` cuenta los paquetes ESP-NOW recibidos y enviados, los envíos fallidos, los bytes perdidos por
  buffer lleno y las conexiones al agente.

## Seguridad

- **ESC:** si no llega `/robot/esc` en 2 s, las hélices pasan a 1000 µs. Por eso la orden se publica de
  forma periódica (la interfaz lo hace a 5 Hz).
- **Ruedas:** sin órdenes durante 0,5 s, frenan.
- **Agente o enlace perdido** (3 pings fallidos): todo se apaga y se reconecta solo.
- **Al arrancar:** ESC a 1000 µs durante 3 s (armado) y ruedas frenadas.
- La IMU se calibra al encender: **deja el robot quieto unos 2 s** después del armado.

## Batería

Se configura al final de `config.h`: `BAT_CELDAS`, las resistencias del divisor, `BAT_CALIBRACION` y los
umbrales de aviso. El porcentaje sale de la tensión de cada celda, así que **baja cuando las hélices
piden mucha corriente** y se recupera al soltarlas. Para saber la carga real, míralo con las hélices
apagadas.

El robot **solo avisa: no apaga nada** por batería baja, porque cortar las hélices en la pared o en el
techo lo haría caer. Cuando aparezca el aviso crítico, baja el robot al suelo cuanto antes.

## Compilar y subir

```bash
cd firmware/robot_receptor && pio run -t upload    # la primera vez tarda: compila micro-ROS
cd ../puente_pc && pio run -t upload
pio device monitor                                 # mensajes de depuración por USB
```

`board_microros_distro` tiene que coincidir con la distro de ROS 2 del PC (viene en `jazzy`). Si la
cambias, ejecuta antes `pio run -t clean_microros`.

## Probar sin la interfaz

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

ros2 topic echo /robot/estado
ros2 topic pub -r 5 /robot/esc std_msgs/msg/Int32 "{data: 1300}"
ros2 topic pub -r 10 /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub -r 10 /robot/ruedas std_msgs/msg/Float32MultiArray "{data: [0.5, -0.5]}"
ros2 topic pub --once /robot/parada std_msgs/msg/Bool "{data: true}"
```

## Protocolo con la ESP puente

Cada paquete ESP-NOW es `[cabecera][hasta 249 bytes]`:

| Cabecera | Sentido | Contenido |
|---|---|---|
| `0xA1` | robot → puente | Bytes XRCE-DDS con framing serie: el puente los escribe tal cual en el USB |
| `0xA2` | puente → robot | Bytes leídos del USB, partidos en trozos de ≤ 249 |

- Las dos ESP usan el **canal 1** (`CANAL_ESPNOW`).
- Si `MAC_PUENTE` en `config.h` se deja en `FF:FF:FF:FF:FF:FF`, el robot transmite en broadcast hasta
  recibir el primer paquete `0xA2` y desde ahí solo habla con esa ESP.
- El framing XRCE tiene CRC: si se pierde un paquete, el cliente lo detecta y reintenta.

## Pendiente

- **Un pin por ESC**, para poder repartir el empuje ([fisica_transiciones.md](fisica_transiciones.md)).
- Sin encoders, `cmd_vel` es lazo abierto.
- Sin medición de corriente: no se sabe cuánta potencia consume cada brushless (solo el acelerador
  que se le manda al ESC).
