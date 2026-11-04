#!/usr/bin/env python3
"""
Odometría del robot REAL para RViz (no hace falta con Gazebo, que da la pose exacta).

  orientación  <- /robot/imu    (MPU6050: roll y pitch fiables, el yaw deriva)
  avance       <- /robot/estado (PWM realmente aplicado a cada lado, como en el firmware:
                                 velocidad ≈ pwm / 1023 · v_max)

La posición se integra avanzando por el eje x del robot en 3D, así que cuando el
robot sube a la pared el modelo de RViz también sube. Sin encoders es una
estimación: se desvía con el tiempo (en la pared el robot va más lento de lo que cree).

Publica:
  TF odom -> base_link
  /robot/odom     nav_msgs/Odometry
  /joint_states   sensor_msgs/JointState  (ruedas y hélices, para que giren en RViz)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

RUEDAS_IZQ = ('rueda_del_izq_joint', 'rueda_tra_izq_joint')
RUEDAS_DER = ('rueda_del_der_joint', 'rueda_tra_der_joint')
HELICES = ('helice_delantero_joint', 'helice_trasero_joint')


def rotar(q, v):
    """Rota el vector v con el cuaternión q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 · (q_xyz × v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class OdometriaImu(Node):
    def __init__(self):
        super().__init__('odometria_imu')
        self.declare_parameter('v_max', 0.30)        # V_MAX_MS del firmware
        self.declare_parameter('pwm_max', 1023)      # MOTOR_PWM_MAX
        self.declare_parameter('ancho_via', 0.239)   # ANCHO_VIA_M
        self.declare_parameter('radio_rueda', 0.0165)  # JSumo SLT20, Ø33 mm
        self.declare_parameter('frecuencia', 50.0)

        self.v_max = float(self.get_parameter('v_max').value)
        self.pwm_max = float(self.get_parameter('pwm_max').value)
        self.via = float(self.get_parameter('ancho_via').value)
        self.r = float(self.get_parameter('radio_rueda').value)

        self.q = (1.0, 0.0, 0.0, 0.0)
        self.w_imu = (0.0, 0.0, 0.0)
        self.t_imu = 0.0
        self.v_izq = self.v_der = 0.0
        self.esc = 1000
        self.t_estado = 0.0
        self.pos = [0.0, 0.0, 0.0]
        self.ang_ruedas = [0.0, 0.0]
        self.ang_helices = 0.0
        self.t_prev = time.monotonic()

        self.tf = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '/robot/odom', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Imu, '/robot/imu', self._al_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/robot/estado', self._al_estado, 10)
        self.create_subscription(Bool, '/robot/odom_reiniciar', self._reiniciar, 10)
        self.create_timer(1.0 / float(self.get_parameter('frecuencia').value), self._paso)
        self.get_logger().info('Odometría IMU lista (publica odom -> base_link)')

    def _al_imu(self, m):
        o = m.orientation
        n = math.sqrt(o.w ** 2 + o.x ** 2 + o.y ** 2 + o.z ** 2)
        if n < 1e-6:
            return
        self.q = (o.w / n, o.x / n, o.y / n, o.z / n)
        self.w_imu = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        self.t_imu = time.monotonic()

    def _al_estado(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        r = d.get('ruedas', {})
        self.v_izq = float(r.get('pwm_izq', 0)) / self.pwm_max * self.v_max
        self.v_der = float(r.get('pwm_der', 0)) / self.pwm_max * self.v_max
        self.esc = int(d.get('esc', {}).get('act', 1000))
        self.t_estado = time.monotonic()

    def _reiniciar(self, m):
        if m.data:
            self.pos = [0.0, 0.0, 0.0]
            self.get_logger().info('Posición puesta a cero')

    def _paso(self):
        ahora = time.monotonic()
        dt = min(ahora - self.t_prev, 0.2)
        self.t_prev = ahora

        if ahora - self.t_estado > 0.5:          # sin estado: el robot frena (timeout 0,5 s)
            self.v_izq = self.v_der = 0.0
            self.esc = 1000
        v = (self.v_izq + self.v_der) / 2
        w = (self.v_der - self.v_izq) / self.via

        avance = rotar(self.q, (v * dt, 0.0, 0.0))
        for i in range(3):
            self.pos[i] += avance[i]
        self.ang_ruedas[0] += self.v_izq / self.r * dt
        self.ang_ruedas[1] += self.v_der / self.r * dt
        acelerador = max(0.0, min(1.0, (self.esc - 1000) / 1000))
        self.ang_helices += acelerador * 25.0 * dt   # giro lento solo para que se vea

        sello = self.get_clock().now().to_msg()
        qw, qx, qy, qz = self.q

        t = TransformStamped()
        t.header.stamp = sello
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.pos
        t.transform.rotation.w, t.transform.rotation.x = qw, qx
        t.transform.rotation.y, t.transform.rotation.z = qy, qz
        self.tf.sendTransform(t)

        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.pos
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = v
        o.twist.twist.angular.x, o.twist.twist.angular.y = self.w_imu[0], self.w_imu[1]
        o.twist.twist.angular.z = self.w_imu[2] if ahora - self.t_imu < 0.5 else w
        self.pub_odom.publish(o)

        js = JointState()
        js.header.stamp = sello
        js.name = list(RUEDAS_IZQ + RUEDAS_DER + HELICES)
        js.position = [self.ang_ruedas[0]] * 2 + [self.ang_ruedas[1]] * 2 + [self.ang_helices] * 2
        self.pub_js.publish(js)


def main():
    rclpy.init()
    nodo = OdometriaImu()
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
