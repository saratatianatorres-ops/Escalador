# Simulación (Gazebo) y visualización (RViz)

Requiere ROS 2 Jazzy y Gazebo Harmonic (`scripts/instalar_dependencias.sh` instala todo).
Paquetes: `robot_trepador_description` (modelo URDF, RViz) y `robot_trepador_gazebo` (mundo, puente, firmware simulado).
El grafo de nodos está en [arquitectura.md](arquitectura.md).

## Robot real en RViz

```bash
ros2 launch robot_trepador_gui robot.launch.py puerto:=/dev/ttyUSB0 rviz:=true
```

El modelo 3D se orienta con la IMU y avanza con el PWM que el robot reporta en `/robot/estado`
(nodo `odometria_imu`). La orientación es real; la posición es una estimación (sin encoders se
desvía con el tiempo y el yaw deriva). Para poner la posición a cero:
`ros2 topic pub --once /robot/odom_reiniciar std_msgs/msg/Bool "{data: true}"`.

## Simulación en Gazebo (sin el robot)

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

Medidas del modelo (en `robot_trepador.urdf.xacro`, fáciles de cambiar): 0,8 kg, 4 ruedas de 35 mm (un N20 por rueda)
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
