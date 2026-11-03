#!/usr/bin/env python3
"""
Interfaz de control del robot trepador (ROS 2 + Tkinter).

Publica:
  /robot/cmd_vel  geometry_msgs/Twist   10 Hz (robot diferencial de 2 ruedas)
  /robot/esc      std_msgs/Int32         5 Hz (mantiene vivo el timeout del robot)
  /robot/parada   std_msgs/Bool          al pulsar PARADA / LIBERAR
Se suscribe a:
  /robot/imu      sensor_msgs/Imu        orientación 3D
  /robot/estado   std_msgs/String (JSON) valores realmente aplicados en el robot y batería

Teclado: flechas o W/A/S/D para moverse (mantener pulsado),
         + / - para las hélices, espacio = PARADA.
"""

import json
import math
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

ESC_APAGADO = 1000
ESC_PASO = 100
ESC_LIMITE = 1700     # tope de seguridad (~23-25 A por motor); el firmware también lo aplica


# ============================================================
#  Nodo ROS 2
# ============================================================
class NodoInterfaz(Node):
    def __init__(self):
        super().__init__('interfaz_robot')
        self.declare_parameter('v_lineal', 0.15)     # m/s al pulsar adelante/atrás
        self.declare_parameter('v_angular', 1.5)     # rad/s al pulsar izquierda/derecha
        self.declare_parameter('esc_max', ESC_LIMITE)  # tope del botón "+" (nunca más de ESC_LIMITE)

        self.lock = threading.Lock()
        self.lin = 0          # -1, 0, +1
        self.ang = 0          # -1, 0, +1
        self.v_lineal = self.get_parameter('v_lineal').value
        self.v_angular = self.get_parameter('v_angular').value
        self.esc_max = min(int(self.get_parameter('esc_max').value), ESC_LIMITE)
        if int(self.get_parameter('esc_max').value) > ESC_LIMITE:
            self.get_logger().warn(f'esc_max recortado a {ESC_LIMITE} us (límite de corriente de los A2217)')
        self.esc = ESC_APAGADO
        self.parada = False

        self.q = (1.0, 0.0, 0.0, 0.0)   # w, x, y, z
        self.t_imu = 0.0
        self.estado = {}
        self.t_estado = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pub_esc = self.create_publisher(Int32, '/robot/esc', 10)
        self.pub_parada = self.create_publisher(Bool, '/robot/parada', 10)
        self.create_subscription(Imu, '/robot/imu', self._al_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/robot/estado', self._al_estado, 10)

        self.create_timer(0.1, self._publicar_cmd)
        self.create_timer(0.2, self._publicar_esc)

    # ---- Suscripciones ----
    def _al_imu(self, m):
        o = m.orientation
        with self.lock:
            self.q = (o.w, o.x, o.y, o.z)
            self.t_imu = time.monotonic()

    def _al_estado(self, m):
        try:
            datos = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.estado = datos
            self.t_estado = time.monotonic()

    # ---- Publicación periódica ----
    def _publicar_cmd(self):
        t = Twist()
        with self.lock:
            if not self.parada:
                t.linear.x = self.lin * self.v_lineal
                t.angular.z = self.ang * self.v_angular
        self.pub_cmd.publish(t)

    def _publicar_esc(self):
        with self.lock:
            valor = ESC_APAGADO if self.parada else self.esc
        self.pub_esc.publish(Int32(data=valor))

    # ---- Órdenes desde la interfaz ----
    def mover(self, lin, ang):
        with self.lock:
            self.lin, self.ang = lin, ang
        self._publicar_cmd()          # respuesta inmediata, sin esperar al timer

    def cambiar_esc(self, delta):
        with self.lock:
            if self.parada:
                return self.esc
            self.esc = max(ESC_APAGADO, min(self.esc_max, self.esc + delta))
            valor = self.esc
        self._publicar_esc()
        return valor

    def fijar_parada(self, activa):
        with self.lock:
            self.parada = activa
            if activa:
                self.esc = ESC_APAGADO
                self.lin = self.ang = 0
        self.pub_parada.publish(Bool(data=activa))
        self._publicar_cmd()
        self._publicar_esc()

    def instantanea(self):
        with self.lock:
            return dict(q=self.q, t_imu=self.t_imu, estado=self.estado,
                        t_estado=self.t_estado, esc=self.esc, parada=self.parada,
                        lin=self.lin, ang=self.ang)


# ============================================================
#  Geometría para la vista 3D
# ============================================================
def matriz_de_cuaternion(w, x, y, z):
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
        (2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
    )


def aplicar(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] for i in range(3))


def euler_grados(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def multiplicar(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


class Camara:
    """Cámara fija, detrás y un poco a la derecha del robot, mirando hacia abajo."""

    def __init__(self, azimut_grados=205, elevacion_grados=25, escala=470, cx=210, cy=190):
        az, el = math.radians(azimut_grados), math.radians(elevacion_grados)
        self.c = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        self.r = (-math.sin(az), math.cos(az), 0.0)
        cx_, cy_, cz_ = self.c
        rx, ry, rz = self.r
        self.u = (cy_ * rz - cz_ * ry, cz_ * rx - cx_ * rz, cx_ * ry - cy_ * rx)
        self.escala, self.cx, self.cy = escala, cx, cy

    def proyectar(self, p):
        x = sum(p[i] * self.r[i] for i in range(3))
        y = sum(p[i] * self.u[i] for i in range(3))
        return (self.cx + x * self.escala, self.cy - y * self.escala)

    def profundidad(self, p):
        return sum(p[i] * self.c[i] for i in range(3))


# Caja del robot (semimedidas, m): x adelante, y izquierda, z arriba. Dos bases: 450 × 216 × 70 mm
LX, LY, LZ = 0.225, 0.108, 0.035
VERTICES = [(sx * LX, sy * LY, sz * LZ) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
#  índice = 4*(sx>0) + 2*(sy>0) + (sz>0)
CARAS = [
    ((4, 6, 7, 5), (1, 0, 0),  '#f08c2a', 'delante'),
    ((0, 1, 3, 2), (-1, 0, 0), '#6b7280', 'atrás'),
    ((2, 3, 7, 6), (0, 1, 0),  '#7aa2d6', 'izquierda'),
    ((0, 4, 5, 1), (0, -1, 0), '#7aa2d6', 'derecha'),
    ((1, 5, 7, 3), (0, 0, 1),  '#3b6fb6', 'arriba'),
    ((0, 2, 6, 4), (0, 0, -1), '#1f2a44', 'abajo'),
]


# ============================================================
#  Interfaz Tkinter
# ============================================================
class Interfaz:
    FONDO = '#14181f'
    PANEL = '#1d232d'
    TEXTO = '#e6e9ef'
    TENUE = '#8b93a1'

    def __init__(self, nodo: NodoInterfaz):
        self.nodo = nodo
        self.yaw_cero = 0.0
        self.teclas = set()
        self.liberar_pendiente = {}
        self.boton_lin = 0
        self.boton_ang = 0

        self.raiz = tk.Tk()
        self.raiz.title('Robot trepador')
        self.raiz.configure(bg=self.FONDO)
        self.raiz.protocol('WM_DELETE_WINDOW', self.cerrar)

        izq = tk.Frame(self.raiz, bg=self.FONDO)
        izq.grid(row=0, column=0, sticky='n', padx=12, pady=12)
        der = tk.Frame(self.raiz, bg=self.FONDO)
        der.grid(row=0, column=1, sticky='n', padx=(0, 12), pady=12)

        self._panel_movimiento(izq)
        self._panel_helices(izq)
        self._panel_bateria(izq)
        self._panel_parada(izq)
        self._panel_imu(der)

        self.barra = tk.Label(self.raiz, anchor='w', bg=self.PANEL, fg=self.TENUE,
                              font=('TkDefaultFont', 9), padx=8, pady=4)
        self.barra.grid(row=1, column=0, columnspan=2, sticky='ew')

        self._teclado()
        self.actualizar()

    # ---------- Paneles ----------
    def _titulo(self, padre, texto):
        tk.Label(padre, text=texto, bg=self.PANEL, fg=self.TEXTO,
                 font=('TkDefaultFont', 11, 'bold')).pack(anchor='w', padx=10, pady=(8, 4))

    def _panel(self, padre):
        f = tk.Frame(padre, bg=self.PANEL)
        f.pack(fill='x', pady=(0, 10))
        return f

    def _panel_movimiento(self, padre):
        p = self._panel(padre)
        self._titulo(p, 'Ruedas (mantén pulsado)')
        g = tk.Frame(p, bg=self.PANEL)
        g.pack(padx=10, pady=4)

        estilo = dict(width=6, height=2, font=('TkDefaultFont', 14, 'bold'),
                      bg='#2c3542', fg=self.TEXTO, activebackground='#3e7bd6',
                      activeforeground='white', relief='flat')
        botones = [
            ('▲', 0, 1, 1, 0),    # texto, fila, columna, lin, ang
            ('◀', 1, 0, 0, 1),
            ('▶', 1, 2, 0, -1),
            ('▼', 2, 1, -1, 0),
        ]
        for texto, f, c, lin, ang in botones:
            b = tk.Button(g, text=texto, **estilo)
            b.grid(row=f, column=c, padx=3, pady=3)
            b.bind('<ButtonPress-1>', lambda e, l=lin, a=ang: self._boton_mover(l, a))
            b.bind('<ButtonRelease-1>', lambda e: self._boton_mover(0, 0))
        tk.Label(g, text='■', bg=self.PANEL, fg=self.TENUE,
                 font=('TkDefaultFont', 14)).grid(row=1, column=1)

        self.v_lin = tk.DoubleVar(value=self.nodo.v_lineal)
        self.v_ang = tk.DoubleVar(value=self.nodo.v_angular)
        self._deslizador(p, 'Velocidad lineal (m/s)', self.v_lin, 0.05, 0.30, 0.01,
                         lambda v: setattr(self.nodo, 'v_lineal', float(v)))
        self._deslizador(p, 'Giro (rad/s)', self.v_ang, 0.5, 4.0, 0.1,
                         lambda v: setattr(self.nodo, 'v_angular', float(v)))

    def _deslizador(self, padre, texto, var, a, b, paso, al_cambiar):
        tk.Scale(padre, label=texto, variable=var, from_=a, to=b, resolution=paso,
                 orient='horizontal', length=240, command=al_cambiar,
                 bg=self.PANEL, fg=self.TEXTO, troughcolor='#2c3542',
                 highlightthickness=0, relief='flat').pack(padx=10, pady=(0, 6))

    def _panel_helices(self, padre):
        p = self._panel(padre)
        self._titulo(p, 'Hélices (ESC)')
        f = tk.Frame(p, bg=self.PANEL)
        f.pack(padx=10, pady=(2, 4))
        estilo = dict(width=4, height=1, font=('TkDefaultFont', 18, 'bold'),
                      bg='#2c3542', fg=self.TEXTO, activebackground='#3e7bd6', relief='flat')
        tk.Button(f, text='−', command=lambda: self.nodo.cambiar_esc(-ESC_PASO),
                  **estilo).grid(row=0, column=0)
        self.lbl_esc = tk.Label(f, text='1000 µs', width=9, bg=self.PANEL, fg=self.TEXTO,
                                font=('TkFixedFont', 16, 'bold'))
        self.lbl_esc.grid(row=0, column=1, padx=8)
        tk.Button(f, text='+', command=lambda: self.nodo.cambiar_esc(+ESC_PASO),
                  **estilo).grid(row=0, column=2)
        self.lbl_esc_real = tk.Label(p, text='en el robot: —', bg=self.PANEL, fg=self.TENUE)
        self.lbl_esc_real.pack(padx=10, pady=(0, 8))

    def _panel_bateria(self, padre):
        p = self._panel(padre)
        self._titulo(p, 'Batería')
        self.lienzo_bat = tk.Canvas(p, width=240, height=26, bg=self.PANEL, highlightthickness=0)
        self.lienzo_bat.pack(padx=10, pady=(2, 2))
        self.lbl_bat = tk.Label(p, text='—', bg=self.PANEL, fg=self.TENUE,
                                font=('TkFixedFont', 10))
        self.lbl_bat.pack(padx=10, pady=(0, 8))

    def _dibujar_bateria(self, bat, enlace):
        """Barra de carga. bat = dict 'bat' de /robot/estado (o None)."""
        c = self.lienzo_bat
        c.delete('all')
        c.create_rectangle(1, 3, 230, 23, outline=self.TENUE, width=2)   # cuerpo
        c.create_rectangle(231, 9, 238, 17, fill=self.TENUE, outline='')  # borne

        if not enlace or bat is None:
            self.lbl_bat.config(text='sin datos del robot' if enlace else '—', fg=self.TENUE)
            return
        if not bat.get('ok'):
            c.create_text(115, 13, text='sin medición', fill=self.TENUE)
            self.lbl_bat.config(text='falta el divisor de tensión en IO34', fg=self.TENUE)
            return

        pct = max(0, min(100, int(bat.get('pct', 0))))
        nivel = int(bat.get('nivel', 0))
        color = ('#57ab5a', '#e0a526', '#e5534b')[min(nivel, 2)]
        if nivel == 2 and int(time.monotonic() * 2) % 2:       # parpadeo en crítico
            color = '#7a2b27'
        ancho = int(226 * pct / 100)
        if ancho > 0:
            c.create_rectangle(4, 6, 4 + ancho, 20, fill=color, outline='')
        c.create_text(115, 13, text=f'{pct} %', fill='white',
                      font=('TkDefaultFont', 10, 'bold'))
        self.lbl_bat.config(
            text=f"{bat.get('v', 0):.2f} V  ·  {bat.get('vc', 0):.2f} V/celda "
                 f"({bat.get('celdas', '?')}S)",
            fg=self.TEXTO if nivel == 0 else color)

    def _panel_parada(self, padre):
        self.btn_parada = tk.Button(padre, text='PARADA (espacio)', height=2,
                                    font=('TkDefaultFont', 12, 'bold'),
                                    bg='#c0392b', fg='white', activebackground='#e74c3c',
                                    relief='flat', command=self._alternar_parada)
        self.btn_parada.pack(fill='x')

    def _panel_imu(self, padre):
        p = self._panel(padre)
        self._titulo(p, 'Orientación (MPU6050)')
        self.lienzo = tk.Canvas(p, width=420, height=360, bg='#0f1319', highlightthickness=0)
        self.lienzo.pack(padx=10)
        self.camara = Camara()
        f = tk.Frame(p, bg=self.PANEL)
        f.pack(fill='x', padx=10, pady=8)
        self.lbl_angulos = tk.Label(f, text='', bg=self.PANEL, fg=self.TEXTO,
                                    font=('TkFixedFont', 11), justify='left')
        self.lbl_angulos.pack(side='left')
        tk.Button(f, text='Yaw a cero', command=self._yaw_a_cero, bg='#2c3542',
                  fg=self.TEXTO, relief='flat').pack(side='right')

    # ---------- Acciones ----------
    def _boton_mover(self, lin, ang):
        self.boton_lin, self.boton_ang = lin, ang
        self._aplicar_movimiento()

    def _aplicar_movimiento(self):
        lin = self.boton_lin + (1 if self.teclas & {'Up', 'w'} else 0) \
                             - (1 if self.teclas & {'Down', 's'} else 0)
        ang = self.boton_ang + (1 if self.teclas & {'Left', 'a'} else 0) \
                             - (1 if self.teclas & {'Right', 'd'} else 0)
        self.nodo.mover(max(-1, min(1, lin)), max(-1, min(1, ang)))

    def _alternar_parada(self):
        self.nodo.fijar_parada(not self.nodo.instantanea()['parada'])

    def _yaw_a_cero(self):
        w, x, y, z = self.nodo.instantanea()['q']
        self.yaw_cero = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _teclado(self):
        mov = {'Up', 'Down', 'Left', 'Right', 'w', 'a', 's', 'd'}

        def presionar(e):
            k = e.keysym if e.keysym in ('Up', 'Down', 'Left', 'Right') else e.keysym.lower()
            if k in mov:
                pend = self.liberar_pendiente.pop(k, None)
                if pend:
                    self.raiz.after_cancel(pend)
                if k not in self.teclas:
                    self.teclas.add(k)
                    self._aplicar_movimiento()
            elif k in ('plus', 'kp_add', 'equal'):
                self.nodo.cambiar_esc(+ESC_PASO)
            elif k in ('minus', 'kp_subtract'):
                self.nodo.cambiar_esc(-ESC_PASO)
            elif k == 'space':
                self._alternar_parada()

        def soltar(e):
            k = e.keysym if e.keysym in ('Up', 'Down', 'Left', 'Right') else e.keysym.lower()
            if k not in mov:
                return

            # La repetición automática del teclado manda soltar+presionar seguidos:
            # se espera un poco antes de dar la tecla por soltada.
            def confirmar():
                self.liberar_pendiente.pop(k, None)
                self.teclas.discard(k)
                self._aplicar_movimiento()
            self.liberar_pendiente[k] = self.raiz.after(60, confirmar)

        self.raiz.bind('<KeyPress>', presionar)
        self.raiz.bind('<KeyRelease>', soltar)

    # ---------- Dibujo ----------
    def _dibujar_imu(self, q, fresca):
        c = self.lienzo
        cam = self.camara
        c.delete('all')

        # Suelo de referencia (fijo en el mundo)
        z_suelo = -0.10
        for i in range(-4, 5):
            a = i * 0.09
            for p1, p2 in (((a, -0.36, z_suelo), (a, 0.36, z_suelo)),
                           ((-0.36, a, z_suelo), (0.36, a, z_suelo))):
                c.create_line(*cam.proyectar(p1), *cam.proyectar(p2), fill='#232b36')
        for eje, color, nombre in (((0.42, 0, 0), '#e5534b', 'X'),
                                   ((0, 0.42, 0), '#57ab5a', 'Y'),
                                   ((0, 0, 0.28), '#539bf5', 'Z')):
            o = cam.proyectar((0, 0, z_suelo))
            pnt = cam.proyectar((eje[0], eje[1], eje[2] + z_suelo))
            c.create_line(*o, *pnt, fill=color, width=2, arrow='last')
            c.create_text(pnt[0] + 8, pnt[1], text=nombre, fill=color)

        # Robot rotado por la IMU (con el yaw corregido)
        r = multiplicar(rot_z(-self.yaw_cero), matriz_de_cuaternion(*q))
        verts = [aplicar(r, v) for v in VERTICES]
        visibles = []
        for idx, normal, color, _ in CARAS:
            n = aplicar(r, normal)
            if sum(n[i] * cam.c[i] for i in range(3)) <= 0:
                continue                                   # cara de espaldas a la cámara
            prof = sum(cam.profundidad(verts[i]) for i in idx) / 4
            visibles.append((prof, idx, color))
        for _, idx, color in sorted(visibles):
            pts = [coord for i in idx for coord in cam.proyectar(verts[i])]
            c.create_polygon(pts, fill=color if fresca else '#3a3f47',
                             outline='#0f1319', width=2)

        # Flecha "adelante" sobre el robot
        a = cam.proyectar(aplicar(r, (-0.12, 0, LZ + 0.002)))
        b = cam.proyectar(aplicar(r, (0.30, 0, LZ + 0.002)))
        c.create_line(*a, *b, fill='white', width=3, arrow='last', arrowshape=(12, 14, 5))

        if not fresca:
            c.create_text(210, 24, text='Sin datos de la IMU', fill='#e5534b',
                          font=('TkDefaultFont', 12, 'bold'))

    def actualizar(self):
        s = self.nodo.instantanea()
        ahora = time.monotonic()
        fresca = ahora - s['t_imu'] < 1.0
        enlace = ahora - s['t_estado'] < 1.5

        self._dibujar_imu(s['q'], fresca)
        roll, pitch, yaw = euler_grados(*s['q'])
        yaw = (yaw - math.degrees(self.yaw_cero) + 180) % 360 - 180
        self.lbl_angulos.config(
            text=f'roll  {roll:7.1f}°\npitch {pitch:7.1f}°\nyaw   {yaw:7.1f}°')

        self.lbl_esc.config(text=f"{s['esc']} µs")
        est = s['estado']
        if enlace and 'esc' in est:
            self.lbl_esc_real.config(text=f"en el robot: {est['esc'].get('act', '—')} µs")
        else:
            self.lbl_esc_real.config(text='en el robot: —')

        bat = est.get('bat') if enlace else None
        self._dibujar_bateria(bat, enlace)

        if s['parada']:
            self.btn_parada.config(text='LIBERAR PARADA (espacio)', bg='#7f8c8d')
        else:
            self.btn_parada.config(text='PARADA (espacio)', bg='#c0392b')

        partes = ['Robot conectado' if enlace else 'Sin conexión con el robot']
        if enlace:
            r = est.get('ruedas', {})
            e = est.get('enlace', {})
            partes.append(f"PWM izq {r.get('pwm_izq', 0)}  der {r.get('pwm_der', 0)}")
            partes.append(f"ESP-NOW fallos {e.get('fallos', 0)}")
            if est.get('parada'):
                partes.append('PARADA ACTIVA en el robot')
            if bat and bat.get('ok') and int(bat.get('nivel', 0)) == 2:
                partes.append('BATERÍA CRÍTICA: baja el robot')
            elif bat and bat.get('ok') and int(bat.get('nivel', 0)) == 1:
                partes.append('batería baja')
        if s['lin'] or s['ang']:
            partes.append(f"moviendo: lin {s['lin']:+d} giro {s['ang']:+d}")
        self.barra.config(text='   |   '.join(partes),
                          fg=self.TEXTO if enlace else '#e5534b')

        self.raiz.after(33, self.actualizar)

    def cerrar(self):
        self.nodo.fijar_parada(False)     # no dejar el robot enclavado
        self.nodo.mover(0, 0)
        with self.nodo.lock:
            self.nodo.esc = ESC_APAGADO
        self.nodo._publicar_esc()
        time.sleep(0.2)
        self.raiz.destroy()

    def ejecutar(self):
        self.raiz.mainloop()


def main():
    rclpy.init()
    nodo = NodoInterfaz()
    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()
    try:
        Interfaz(nodo).ejecutar()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
