import sys, numpy as np, trimesh
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import armar
D = '../../../ros2_ws/src/robot_trepador_description/meshes/'
M = 'mallas_sw/'
def en_ros(pieza, m4):
    g = trimesh.load(M + pieza + '.stl'); g.apply_transform(armar.a_ros(m4)); return g
c = armar.comp
salidas = {
    'base_delantera.stl': en_ros(*c['base_1']),
    'base_trasera.stl': en_ros(*c['base_2']),
    'soporte_motor_delantero.stl': en_ros(*c['agarre_2']),
    'soporte_motor_trasero.stl': en_ros(*c['agarre_1']),
    'a2217_delantero.stl': en_ros(*c['motor_2']),
    'a2217_trasero.stl': en_ros(*c['motor_1']),
}
# llantas: en su propio marco (centro de la llanta, eje a lo largo de y)
for lado, k in (('izq', 'llanta_di'), ('der', 'llanta_dd')):
    g = en_ros(*c[k]); centro = g.bounds.mean(0); g.apply_translation(-centro)
    salidas[f'llanta_{lado}.stl'] = g
    print('llanta', lado, 'centro (mm)', (centro * 1000).round(1))
for nom, g in salidas.items():
    g.export(D + nom)
    print(f'{nom:<28} {len(g.faces):6d} tris  min {(g.bounds[0]*1000).round(1)} max {(g.bounds[1]*1000).round(1)}')
