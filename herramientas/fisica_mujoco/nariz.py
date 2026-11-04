#!/usr/bin/env python3
"""
Estudio de la nariz: ¿cuánto tienen que sobresalir las llantas para que el robot
pase solo del suelo a la pared (y de la pared al techo)?

Modelo (MuJoCo) del robot real: dos bases de 225 mm (450 × 216 × 70 mm), un ducto con
un A2217 en cada extremo, 4 llantas JSumo SLT20 (Ø33 mm) con un N20 cada una y 1,225 kg en total
(hardware/lista_materiales.md).

Uso:
    pip install mujoco numpy
    python3 nariz.py                       # barrido completo (tabla)
    python3 nariz.py --saliente 20 --modo trasera --ver   # un caso, con ventana

Modos de empuje:
    igual    las dos hélices al mismo acelerador (ESC en paralelo: el robot de hoy)
    trasera  delantera apagada hasta subir la nariz, luego las dos (necesita un pin por ESC)
"""

import argparse
import math

import mujoco
import numpy as np

# ---------------- Robot (m, kg, N) ----------------
LARGO_BASE = 0.225          # cada base
MEDIO_ANCHO = 0.100         # la placa mide 200 mm de ancho en la zona de los ductos
ESP_PLACA = 0.010
DUCTO_X, DUCTO_R, DUCTO_ALTO = 0.127, 0.0975, 0.070
R_RUEDA, ANCHO_BANDA = 0.0165, 0.018
RUEDA_Y = 0.0975            # centro de la llanta (vía 195 mm)
EJE_BAJO_PLACA = 0.005

MASA_TOTAL = 1.225                  # 2 bases × 220 g + 785 g del resto (peor caso)
MASA_CHASIS = MASA_TOTAL - 4 * 0.012  # todo menos las 4 llantas
MASA_RUEDA = 0.012
EMPUJE_MAX = 8.9            # N por ducto a 1700 us (909 gf medidos a 23 A)
PAR_N20 = 0.20              # N·m máximo por llanta
V_AVANCE = 0.10             # m/s

PARED_X = 0.80              # cara de la pared
TECHO_Z = 1.00              # cara del techo


def mjcf(saliente_mm, mu, par):
    """Genera el XML. saliente = cuánto adelanta el frente de la llanta al chasis (mm)."""
    rueda_x = LARGO_BASE + saliente_mm / 1000 - R_RUEDA
    z0 = R_RUEDA + EJE_BAJO_PLACA            # altura de la cara inferior de la placa
    ruedas = []
    for nombre, sx, sy in (('di', 1, 1), ('dd', 1, -1), ('ti', -1, 1), ('td', -1, -1)):
        ruedas.append(f"""
      <body name="r_{nombre}" pos="{sx*rueda_x} {sy*RUEDA_Y} {-EJE_BAJO_PLACA}">
        <joint name="j_{nombre}" type="hinge" axis="0 1 0" damping="0.0005" armature="0.0009"/>
        <!-- armature: inercia del rotor del N20 vista a través de la reducción 1:298 -->
        <geom type="cylinder" size="{R_RUEDA} {ANCHO_BANDA/2}" euler="90 0 0"
              mass="{MASA_RUEDA}" friction="{mu} 0.005 0.0001" condim="4"/>
      </body>""")
    # soportes del N20 (nariz): bloque detrás de cada llanta delantera y trasera
    soportes = []
    for sx in (1, -1):
        for sy in (1, -1):
            x = sx * rueda_x
            soportes.append(
                f'<geom type="box" size="0.0086 0.004 0.0076" pos="{x} {sy*0.082} {-EJE_BAJO_PLACA}" '
                f'mass="0" friction="0.3"/>')
            # brazo que une el soporte con la placa
            x_ini = sx * (LARGO_BASE - 0.015)
            soportes.append(
                f'<geom type="box" size="{abs(x - x_ini)/2 + 0.004} 0.004 {ESP_PLACA/2}" '
                f'pos="{(x + x_ini)/2} {sy*0.082} {ESP_PLACA/2}" mass="0" friction="0.3"/>')
    return f"""
<mujoco model="escalador">
  <option timestep="0.001" cone="elliptic" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><geom solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="suelo" type="plane" size="3 3 0.1" friction="{mu} 0.005 0.0001"/>
    <geom name="pared" type="box" pos="{PARED_X + 0.05} 0 0.6" size="0.05 3 0.6" friction="{mu} 0.005 0.0001"/>
    <geom name="techo" type="box" pos="0 0 {TECHO_Z + 0.05}" size="3 3 0.05" friction="{mu} 0.005 0.0001"/>
    <body name="robot" pos="0 0 {z0 + 0.002}">
      <freejoint/>
      <inertial pos="0 0 0.022" mass="{MASA_CHASIS}" diaginertia="{0.0045*MASA_CHASIS} {0.0161*MASA_CHASIS} {0.0198*MASA_CHASIS}"/>
      <geom type="box" size="{LARGO_BASE} {MEDIO_ANCHO} {ESP_PLACA/2}" pos="0 0 {ESP_PLACA/2}" mass="0" friction="0.3"/>
      <geom type="cylinder" size="{DUCTO_R} {(DUCTO_ALTO-ESP_PLACA)/2}" pos="{DUCTO_X} 0 {(DUCTO_ALTO+ESP_PLACA)/2}" mass="0" friction="0.3"/>
      <geom type="cylinder" size="{DUCTO_R} {(DUCTO_ALTO-ESP_PLACA)/2}" pos="{-DUCTO_X} 0 {(DUCTO_ALTO+ESP_PLACA)/2}" mass="0" friction="0.3"/>
      {''.join(soportes)}
      <site name="h_del" pos="{DUCTO_X} 0 0.045"/>
      <site name="h_tra" pos="{-DUCTO_X} 0 0.045"/>
      {''.join(ruedas)}
    </body>
  </worldbody>
  <actuator>
    <motor name="emp_del" site="h_del" gear="0 0 -1 0 0 0" ctrlrange="0 30"/>
    <motor name="emp_tra" site="h_tra" gear="0 0 -1 0 0 0" ctrlrange="0 30"/>
    <velocity joint="j_di" kv="0.5" forcerange="-{par} {par}"/>
    <velocity joint="j_dd" kv="0.5" forcerange="-{par} {par}"/>
    <velocity joint="j_ti" kv="0.5" forcerange="-{par} {par}"/>
    <velocity joint="j_td" kv="0.5" forcerange="-{par} {par}"/>
  </actuator>
</mujoco>"""


def inclinacion(d):
    """Ángulo de la nariz respecto al suelo (grados): 0 = suelo, 90 = pared, 180 = techo."""
    q = d.qpos[3:7]
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, q)
    x_robot = m.reshape(3, 3)[:, 0]          # hacia dónde apunta la nariz
    return math.degrees(math.atan2(x_robot[2], x_robot[0]))   # 0..180 en el plano x-z


def simular(saliente_mm, modo, mu=1.0, par=PAR_N20, empuje=EMPUJE_MAX, t_fin=40.0, ver=False):
    m = mujoco.MjModel.from_xml_string(mjcf(saliente_mm, mu, par))
    d = mujoco.MjData(m)
    w = V_AVANCE / R_RUEDA
    estado = 'suelo'
    hitos = {}
    visor = None
    if ver:
        from mujoco import viewer
        visor = viewer.launch_passive(m, d)
    while d.time < t_fin:
        t = d.time
        rampa = min(1.0, t / 1.0)
        z = d.qpos[2]
        x = d.qpos[0]
        ang = inclinacion(d)
        # estados para el reparto
        if estado == 'suelo' and ang > 60:
            estado = 'pared'
            hitos.setdefault('pared', t)
        if estado == 'pared' and z > TECHO_Z - 0.15 and ang > 120:
            estado = 'techo'
            hitos.setdefault('techo', t)
        if modo == 'igual':
            del_, tra = empuje, empuje
        else:   # 'trasera': delantera apagada mientras la nariz no esté arriba
            if estado == 'suelo':
                del_, tra = 0.0, empuje
            elif estado == 'pared' and z > TECHO_Z - 0.40:
                del_, tra = 0.0, empuje        # mismo truco en la esquina del techo
            else:
                del_, tra = empuje, empuje
        d.ctrl[0] = del_ * rampa
        d.ctrl[1] = tra * rampa
        d.ctrl[2:6] = w if t > 1.0 else 0.0
        mujoco.mj_step(m, d)
        if visor is not None and int(t * 1000) % 16 == 0:
            visor.sync()
        if z < -0.5 or (estado != 'suelo' and z < 0.05 and ang < 20 and 'pared' in hitos and t - hitos['pared'] > 2):
            hitos['cayo'] = t
            break
        if estado == 'techo' and z < TECHO_Z - 0.3:
            hitos['cayo'] = t
            break
        if estado == 'techo' and x < PARED_X - 0.5:
            hitos['avanza_techo'] = t
            break
    if visor is not None:
        visor.close()
    if 'cayo' in hitos:
        return 'se cae'
    if 'techo' in hitos:
        return 'suelo → pared → techo'
    if 'pared' in hitos:
        return 'suelo → pared' if d.qpos[2] > 0.3 else 'nariz arriba, no sube'
    return f'no sube (ángulo máx. en la esquina)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--saliente', type=float, help='mm que la llanta sobresale del chasis')
    ap.add_argument('--modo', choices=['igual', 'trasera'])
    ap.add_argument('--mu', type=float, default=1.0)
    ap.add_argument('--par', type=float, default=PAR_N20)
    ap.add_argument('--empuje', type=float, default=EMPUJE_MAX)
    ap.add_argument('--ver', action='store_true')
    a = ap.parse_args()
    if a.saliente is not None and a.modo:
        print(simular(a.saliente, a.modo, a.mu, a.par, a.empuje, ver=a.ver))
        return
    print(f'Masa {MASA_TOTAL:.3f} kg, empuje {a.empuje} N por ducto, μ = {a.mu}, par N20 = {a.par} N·m')
    print(f'{"saliente":>9} | {"igual (hoy)":<26} | {"trasera (un pin por ESC)":<26}')
    for s in (0, 10, 20, 30, 40):
        r1 = simular(s, 'igual', a.mu, a.par, a.empuje)
        r2 = simular(s, 'trasera', a.mu, a.par, a.empuje)
        print(f'{s:>6} mm | {r1:<26} | {r2:<26}', flush=True)


if __name__ == '__main__':
    main()
