#!/usr/bin/env python3
"""
Hace en la simulación lo mismo que el firmware del ESP32 (firmware/robot_receptor),
para que la interfaz funcione igual con Gazebo que con el robot real.

Escucha los mismos tópicos que el robot:
  /robot/esc      std_msgs/Int32              us del ESC (1000 = apagado, 1100..1700)
  /robot/cmd_vel  geometry_msgs/Twist         lazo abierto, como en el firmware
  /robot/ruedas   std_msgs/Float32MultiArray  [izq, der] de -1 a 1
  /robot/parada   std_msgs/Bool               parada de emergencia enclavada
  /robot/imu      sensor_msgs/Imu             (de Gazebo, para el JSON de estado)

Y manda a Gazebo (por ros_gz_bridge):
  /sim/cmd_vel            Twist con la velocidad que de verdad darían los motores
  /sim/empuje_delantero   Float64, N
  /sim/empuje_trasero     Float64, N
Publica /robot/estado (JSON con el mismo formato que el firmware).

Reproduce: armado de 3 s, rampa del ESC (20 us cada 200 ms, salto directo a 1100),
timeouts (ESC 2 s, ruedas 0,5 s), parada enclavada, PWM mínimo de arranque de los N20
y velocidad ≈ pwm / 1023 · v_max. Los dos ESC van en paralelo, así que las dos hélices
reciben el mismo empuje.

Batería simulada: LiPo de `bat_celdas` celdas y `bat_mah` mAh que se descarga según el acelerador
(hasta `bat_amp_motor` A por motor a fondo). La tensión cae con la corriente (resistencia interna),
y el porcentaje y los avisos se calculan igual que en firmware/robot_receptor/src/bateria.cpp.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, Float64, Int32, String

# --- Mismos valores que firmware/robot_receptor/src/config.h ---
ESC_APAGADO = 1000
ESC_MIN = 1100
ESC_MAX = 1700          # tope de seguridad, igual que en config.h
ESC_PASO_RAMPA = 20
ESC_INTERVALO_RAMPA = 0.2
ESC_TIEMPO_ARMADO = 3.0
ESC_TIMEOUT = 2.0
MOTOR_PWM_MAX = 1023
MOTOR_PWM_MIN_ARRANQUE = 300
MOTORES_TIMEOUT = 0.5
BAT_V_CELDA_BAJA = 3.50
BAT_V_CELDA_CRITICA = 3.30
CURVA_V = (3.30, 3.50, 3.68, 3.74, 3.80, 3.87, 3.95, 4.05, 4.20)
CURVA_PCT = (0, 5, 20, 35, 50, 65, 80, 92, 100)


def interpolar(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


def euler_grados(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def velocidad_a_pwm(v):
    v = max(-1.0, min(1.0, v))
    if abs(v) < 0.02:
        return 0
    mag = MOTOR_PWM_MIN_ARRANQUE + int(abs(v) * (MOTOR_PWM_MAX - MOTOR_PWM_MIN_ARRANQUE))
    return mag if v > 0 else -mag


class FirmwareSimulado(Node):
    def __init__(self):
        super().__init__('robot_trepador')
        self.declare_parameter('v_max', 0.30)           # V_MAX_MS
        self.declare_parameter('ancho_via', 0.195)      # ANCHO_VIA_M
        self.declare_parameter('empuje_max_n', 10.0)    # N por hélice a ESC_MAX (1700 us)
        self.declare_parameter('reparto_delantera', 1.0)  # fracción del empuje en la hélice delantera
        self.declare_parameter('bat_celdas', 2)
        self.declare_parameter('bat_mah', 2200.0)
        self.declare_parameter('bat_carga_inicial', 0.95)  # 0..1
        self.declare_parameter('bat_amp_motor', 24.0)      # A de cada A2217 a ESC_MAX (1700 us)
        self.declare_parameter('bat_mohm_celda', 10.0)     # resistencia interna por celda
        self.bat_celdas = int(self.get_parameter('bat_celdas').value)
        self.bat_ah = float(self.get_parameter('bat_mah').value) / 1000.0
        self.bat_soc = float(self.get_parameter('bat_carga_inicial').value)
        self.bat_amp_motor = float(self.get_parameter('bat_amp_motor').value)
        self.bat_r_celda = float(self.get_parameter('bat_mohm_celda').value) / 1000.0
        self.bat_v = None
        self.bat_nivel = 0
        self.t_bat = time.monotonic()
        self.v_max = float(self.get_parameter('v_max').value)
        self.via = float(self.get_parameter('ancho_via').value)
        self.empuje_max = float(self.get_parameter('empuje_max_n').value)
        self.reparto_del = float(self.get_parameter('reparto_delantera').value)

        self.t0 = time.monotonic()
        self.armado = False
        self.parada = False
        self.esc_obj = ESC_APAGADO
        self.esc_act = ESC_APAGADO
        self.esc_ultimo = ESC_APAGADO
        self.t_orden_esc = None
        self.t_orden_ruedas = None
        self.timeout_esc = True
        self.timeout_ruedas = True
        self.t_rampa = 0.0
        self.rueda_izq = self.rueda_der = 0.0
        self.pwm_izq = self.pwm_der = 0
        self.ultimo_v = self.ultimo_w = 0.0
        self.n = dict(esc=0, cmd_vel=0, ruedas=0, parada=0)
        self.imu = None
        self.t_imu = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.pub_del = self.create_publisher(Float64, '/sim/empuje_delantero', 10)
        self.pub_tra = self.create_publisher(Float64, '/sim/empuje_trasero', 10)
        self.pub_estado = self.create_publisher(String, '/robot/estado', 10)

        self.create_subscription(Int32, '/robot/esc', self._al_esc, 10)
        self.create_subscription(Twist, '/robot/cmd_vel', self._al_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/robot/ruedas', self._al_ruedas, 10)
        self.create_subscription(Bool, '/robot/parada', self._al_parada, 10)
        self.create_subscription(Imu, '/robot/imu', self._al_imu, qos_profile_sensor_data)

        self.create_timer(0.02, self._actualizar)      # 50 Hz hacia Gazebo
        self.create_timer(0.2, self._publicar_estado)  # 5 Hz, como ESTADO_PUB_HZ
        self.get_logger().info(f'Firmware simulado: armando ESC ({ESC_TIEMPO_ARMADO:.0f} s)...')

    # ---------------- Órdenes ----------------
    def _al_esc(self, m):
        self.n['esc'] += 1
        self.esc_ultimo = m.data
        self.t_orden_esc = time.monotonic()
        self.timeout_esc = False
        if self.parada or not self.armado:
            return
        if m.data < ESC_MIN:
            self._esc_apagar()
        else:
            self.esc_obj = min(m.data, ESC_MAX)

    def _al_cmd_vel(self, m):
        self.n['cmd_vel'] += 1
        self.ultimo_v, self.ultimo_w = m.linear.x, m.angular.z
        v_izq = m.linear.x - m.angular.z * self.via / 2
        v_der = m.linear.x + m.angular.z * self.via / 2
        self._ruedas_orden(v_izq / self.v_max, v_der / self.v_max)

    def _al_ruedas(self, m):
        self.n['ruedas'] += 1
        if len(m.data) >= 2:
            self._ruedas_orden(m.data[0], m.data[1])

    def _al_parada(self, m):
        self.n['parada'] += 1
        if m.data != self.parada:
            self.get_logger().warn('Parada de emergencia ' + ('ACTIVADA' if m.data else 'liberada'))
        self.parada = m.data
        if m.data:
            self._esc_apagar()
            self._aplicar_ruedas(0.0, 0.0)

    def _al_imu(self, m):
        o = m.orientation
        self.imu = euler_grados(o.w, o.x, o.y, o.z)
        self.t_imu = time.monotonic()

    def _ruedas_orden(self, izq, der):
        self.t_orden_ruedas = time.monotonic()
        self.timeout_ruedas = False
        if self.parada:
            return
        self._aplicar_ruedas(izq, der)

    def _aplicar_ruedas(self, izq, der):
        self.rueda_izq = max(-1.0, min(1.0, izq))
        self.rueda_der = max(-1.0, min(1.0, der))
        self.pwm_izq = velocidad_a_pwm(self.rueda_izq)
        self.pwm_der = velocidad_a_pwm(self.rueda_der)

    def _esc_apagar(self):
        self.esc_obj = self.esc_act = ESC_APAGADO

    # ---------------- Bucle ----------------
    def _actualizar(self):
        ahora = time.monotonic()
        if not self.armado and ahora - self.t0 >= ESC_TIEMPO_ARMADO:
            self.armado = True
            self.get_logger().info('ESC armados: listo')

        if not self.timeout_esc and ahora - self.t_orden_esc > ESC_TIMEOUT:
            self.timeout_esc = True
            if self.esc_obj != ESC_APAGADO or self.esc_act != ESC_APAGADO:
                self._esc_apagar()
                self.get_logger().warn('Sin órdenes de ESC: hélices apagadas')
        if not self.timeout_ruedas and ahora - self.t_orden_ruedas > MOTORES_TIMEOUT:
            self.timeout_ruedas = True
            if self.pwm_izq or self.pwm_der:
                self._aplicar_ruedas(0.0, 0.0)

        if ahora - self.t_rampa >= ESC_INTERVALO_RAMPA:
            self.t_rampa = ahora
            if self.esc_act < self.esc_obj:
                base = max(self.esc_act, ESC_MIN - ESC_PASO_RAMPA)
                self.esc_act = min(base + ESC_PASO_RAMPA, self.esc_obj)
            elif self.esc_act > self.esc_obj:
                self.esc_act = max(self.esc_act - ESC_PASO_RAMPA, self.esc_obj)

        # Ruedas: velocidad real aproximada a partir del PWM aplicado
        v_izq = self.pwm_izq / MOTOR_PWM_MAX * self.v_max
        v_der = self.pwm_der / MOTOR_PWM_MAX * self.v_max
        t = Twist()
        t.linear.x = (v_izq + v_der) / 2
        t.angular.z = (v_der - v_izq) / self.via
        self.pub_cmd.publish(t)

        # Hélices: empuje proporcional al acelerador
        acelerador = max(0.0, min(1.0, (self.esc_act - ESC_APAGADO) / (ESC_MAX - ESC_APAGADO)))
        empuje = acelerador * self.empuje_max
        self.pub_del.publish(Float64(data=empuje * self.reparto_del))
        self.pub_tra.publish(Float64(data=empuje))
        self._bateria(acelerador, ahora)

    def _bateria(self, acelerador, ahora):
        dt = ahora - self.t_bat
        self.t_bat = ahora
        corriente = (0.4 + 2 * self.bat_amp_motor * acelerador ** 1.5
                     + 0.3 * (abs(self.pwm_izq) + abs(self.pwm_der)) / MOTOR_PWM_MAX)
        self.bat_soc = max(0.0, self.bat_soc - corriente * dt / 3600.0 / self.bat_ah)
        v_celda_reposo = interpolar(self.bat_soc * 100, CURVA_PCT, CURVA_V)
        v = self.bat_celdas * v_celda_reposo - corriente * self.bat_r_celda * self.bat_celdas
        # mismo filtro que el firmware (10 Hz, 0,1) pasado a 50 Hz
        self.bat_v = v if self.bat_v is None else self.bat_v + 0.02 * (v - self.bat_v)
        vc = self.bat_v / self.bat_celdas
        if vc < BAT_V_CELDA_CRITICA:
            self.bat_nivel = 2
        elif vc < BAT_V_CELDA_BAJA:
            if self.bat_nivel == 0 or (self.bat_nivel == 2 and vc > BAT_V_CELDA_CRITICA + 0.05):
                self.bat_nivel = 1
        elif vc > BAT_V_CELDA_BAJA + 0.05:
            self.bat_nivel = 0

    def _publicar_estado(self):
        ahora = time.monotonic()

        def ms(t):
            return -1 if t is None else int((ahora - t) * 1000)

        imu_ok = self.imu is not None and ahora - self.t_imu < 0.5
        roll, pitch, yaw = self.imu if self.imu else (0.0, 0.0, 0.0)
        estado = {
            't': int((ahora - self.t0) * 1000),
            'parada': self.parada,
            'esc': {'ord': self.esc_ultimo, 'obj': self.esc_obj, 'act': self.esc_act,
                    'ms': ms(self.t_orden_esc), 'n': self.n['esc']},
            'ruedas': {'izq': round(self.rueda_izq, 2), 'der': round(self.rueda_der, 2),
                       'pwm_izq': self.pwm_izq, 'pwm_der': self.pwm_der,
                       'ms': ms(self.t_orden_ruedas),
                       'v': round(self.ultimo_v, 2), 'w': round(self.ultimo_w, 2),
                       'n_vel': self.n['cmd_vel'], 'n_dir': self.n['ruedas']},
            'imu': {'ok': imu_ok, 'roll': round(roll, 1), 'pitch': round(pitch, 1),
                    'yaw': round(yaw, 1), 'temp': 25.0, 'err': 0},
            'enlace': {'rx': sum(self.n.values()), 'tx': 0, 'fallos': 0, 'desc': 0, 'conex': 1},
            'bat': {'ok': self.bat_v is not None,
                    'v': round(self.bat_v or 0.0, 2),
                    'vc': round((self.bat_v or 0.0) / self.bat_celdas, 2),
                    'pct': int(round(interpolar((self.bat_v or 0.0) / self.bat_celdas,
                                                CURVA_V, CURVA_PCT))),
                    'nivel': self.bat_nivel, 'celdas': self.bat_celdas},
            'sim': True,
        }
        self.pub_estado.publish(String(data=json.dumps(estado, separators=(',', ':'))))


def main():
    rclpy.init()
    nodo = FirmwareSimulado()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
