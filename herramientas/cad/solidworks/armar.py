"""Arma el robot en el marco de ROS (x adelante, y izquierda, z arriba) con las piezas y
posiciones del ensamble Diseño 2.SLDASM. base_link: centro del robot, cara inferior de la placa."""
import numpy as np, trimesh, json, sys
M = 'mallas_sw/'
# Transformadas del ensamble (filas de rotación, traslación en m), ver COMPINSTANCETREE
def T(r, t):
    m = np.eye(4); m[:3, :3] = np.array(r).reshape(3, 3).T; m[:3, 3] = t; return m
# SolidWorks guarda la rotación por filas como p' = p · R (vector fila); por eso la traspuesta
I = [1, 0, 0, 0, 1, 0, 0, 0, 1]
RY180 = [-1, 0, 0, 0, 1, 0, 0, 0, -1]
RX180 = [1, 0, 0, 0, -1, 0, 0, 0, -1]
comp = {
    'base_1': ('base', T(I, [-0.08906140488407446, 0.01746250000000698, 1.0])),
    'base_2': ('base', T(RY180, [-0.08906140488407448, 0.01746250000000698, 1.0])),
    'agarre_1': ('agarre_motor', T([0.7062455224483752, 0, 0.7079669921836904, 0, 1, 0, -0.7079669921836904, 0, 0.7062455224483752], [-0.2160614048840745, -0.007537499999993032, 1.0])),
    'agarre_2': ('agarre_motor', T([0.7090622695161395, 0, 0.7051458699791285, 0, 1, 0, -0.7051458699791285, 0, 0.7090622695161395], [0.03793859511592554, -0.007537499999993025, 1.0])),
    'motor_1': ('motor_a2217', T(I, [-0.2160614048840745, -0.005537499999993036, 1.0])),
    'motor_2': ('motor_a2217', T(I, [0.03793859511592554, -0.005537499999993027, 1.0])),
    'llanta_di': ('llanta', T(I, [0.1284385951159256, -0.0025375, 0.87])),
    'llanta_ti': ('llanta', T(I, [-0.3065614048840746, -0.0025375, 0.87])),
    'llanta_dd': ('llanta', T(RX180, [0.1284385951159256, -0.0025375, 1.13])),
    'llanta_td': ('llanta', T(RX180, [-0.3065614048840746, -0.0025375, 1.13])),
}
n20 = {  # caja del N20 (12 × 13,2 × 35,9 mm), eje en +Z local
    'n20_dd': T(I, [0.1284385951159256, -0.0025375, 1.109]),
    'n20_td': T(I, [-0.3065614048840746, -0.0025375, 1.109]),
    'n20_di': T(RY180, [0.1284385951159256, -0.0025375, 0.891]),
    'n20_ti': T(RY180, [-0.3065614048840746, -0.0025375, 0.891]),
}
# Ensamble (X largo, Y arriba, Z ancho) -> ROS: x = X + 0.0891, y = -(Z - 1), z = Y - Y_placa
Y_PLACA = 0.01746250000000698 - 0.035
A2R = np.eye(4); A2R[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]; A2R[:3, 3] = [0.08906140488407446, 1.0, -Y_PLACA]
def a_ros(m4): return A2R @ m4

if __name__ == '__main__':
    escena = []
    for k, (pieza, m4) in comp.items():
        g = trimesh.load(M + pieza + '.stl'); g.apply_transform(a_ros(m4)); escena.append((k, g))
    for k, m4 in n20.items():
        b = trimesh.creation.box(extents=[0.012, 0.0132, 0.0359]); b.apply_translation([0, -0.0004, -0.00795])
        b.apply_transform(a_ros(m4)); escena.append((k, b))
    todo = trimesh.util.concatenate([g for _, g in escena])
    print('robot (mm):', (todo.bounds * 1000).round(1).tolist())
    for k, g in escena:
        print(f'{k:<10} centro {(g.bounds.mean(0) * 1000).round(1)}  min z {g.bounds[0][2]*1000:.1f}')
    todo.export('mallas_sw/robot_armado.stl')
