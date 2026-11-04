import sys, os, tempfile, numpy as np, trimesh
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import teselas
S34 = 'salida_sw/017_FaceTessellations_000-000-034'
S35 = 'salida_sw/018_FaceTessellations_000-000-035'
OUT = 'mallas_sw'

def trozo(ruta, a, b=None):
    d = open(ruta, 'rb').read()
    seg = d[a * 4:(b * 4 if b else len(d))]
    t = tempfile.NamedTemporaryFile(delete=False); t.write(seg); t.close()
    caras, V, T = teselas.leer(t.name, 'tira'); os.unlink(t.name)
    m = trimesh.Trimesh(V, T, process=True)
    return m, len(caras)

def siguiente(ruta, desde):
    d = open(ruta, 'rb').read(); u = np.frombuffer(d[:len(d) // 4 * 4], dtype='<u4')
    for q in range(desde + 10, len(u) - 10):
        if u[q - 2] == 103 and u[q - 1] < 100 and u[q + 5] == 1 and u[q + 6] == 0 and 2100000 < u[q + 8] < 2110000:
            return q
    return None

piezas = {
    'base': (S34, 1, 143699),
    'agarre_motor': (S34, 143699, 164081),
    'motor_a2217': (S34, 164081, siguiente(S34, 164081)),
    'llanta': (S35, 319668, None),
}
for nom, (ruta, a, b) in piezas.items():
    m, nc = trozo(ruta, a, b)
    m.export(os.path.join(OUT, nom + '.stl'))
    print(f'{nom:<14} caras {nc:4d} tris {len(m.faces):6d} mm min {(m.bounds[0]*1000).round(1)} max {(m.bounds[1]*1000).round(1)}')
