import sys, numpy as np, trimesh
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import teselas, tempfile, os

def cuerpos(ruta):
    b = open(ruta, 'rb').read()
    u = np.frombuffer(b[:len(b) // 4 * 4], dtype='<u4')
    n = len(u) - 14
    p = np.where((u[5:n + 5] == 1) & (u[6:n + 6] == 0) & (u[7:n + 7] == 0)
                 & (u[8:n + 8] > 2100000) & (u[8:n + 8] < 2110000)
                 & (u[2:n + 2] > 0) & (u[2:n + 2] < 5000))[0]
    p = p[p >= 2]
    p = np.array([1] + [q for q in p if u[q - 2] == 103 and u[q - 1] < 100])
    lim = list(p) + [len(u)]
    out = []
    for k in range(len(p)):
        seg = b[lim[k] * 4:lim[k + 1] * 4]
        tmp = tempfile.NamedTemporaryFile(delete=False); tmp.write(seg); tmp.close()
        caras, V, T = teselas.leer(tmp.name, 'tira'); os.unlink(tmp.name)
        out.append(((int(u[p[k]]), int(u[p[k] + 1])), int(u[p[k] + 2]), len(caras), V, T))
    return out

if __name__ == '__main__':
    for i, (ids, nc, ncl, V, T) in enumerate(cuerpos(sys.argv[1])):
        if len(V):
            print(i, ids, 'caras', nc, 'leídas', ncl, 'tris', len(T), 'mm', (V.min(0) * 1000).round(1), (V.max(0) * 1000).round(1))
