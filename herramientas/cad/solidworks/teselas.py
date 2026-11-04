"""Lee las teselaciones de caras de un archivo SolidWorks (secciones FaceTessellations/...).

Cada cara trae: [4,8,2,n] n enteros = tamaño de cada tira de triángulos,
                [12,100,2,N] N vértices (x,y,z float32, metros),
                [12,100,2,N] N normales.
"""
import struct, sys
import numpy as np


def leer(ruta, modo='tira'):
    b = open(ruta, 'rb').read()
    u = np.frombuffer(b[:len(b) // 4 * 4], dtype='<u4')
    f = np.frombuffer(b[:len(b) // 4 * 4], dtype='<f4')
    caras = []
    i = 0
    n_u = len(u)
    while i < n_u - 4:
        if u[i] == 4 and u[i + 1] == 8 and u[i + 2] == 2:
            n = int(u[i + 3])
            if 0 < n < 100000 and i + 4 + n + 4 < n_u:
                tam = u[i + 4:i + 4 + n].astype(int)
                j = i + 4 + n
                N = int(tam.sum())
                if (j + 3 + 3 * N < n_u and u[j] == 12 and u[j + 1] == 100 and u[j + 2] == 2 and u[j + 3] == N
                        and tam.min() >= 3 and N > 0):
                    v = f[j + 4:j + 4 + 3 * N].reshape(-1, 3).astype(float)
                    k = j + 4 + 3 * N
                    nor = None
                    if k + 3 < n_u and u[k] == 12 and u[k + 1] == 100 and u[k + 3] == N:
                        nor = f[k + 4:k + 4 + 3 * N].reshape(-1, 3).astype(float)
                        k += 4 + 3 * N
                    caras.append((tam, v, nor))
                    i = k
                    continue
        i += 1
    tris = []
    todos = []
    o = 0
    for tam, v, nor in caras:
        todos.append(v)
        p = 0
        for t in tam:
            idx = list(range(p, p + t))
            if modo == 'tira':
                for q in range(t - 2):
                    a, bb, c = idx[q], idx[q + 1], idx[q + 2]
                    tris.append((a + o, bb + o, c + o) if q % 2 == 0 else (bb + o, a + o, c + o))
            else:
                for q in range(1, t - 1):
                    tris.append((idx[0] + o, idx[q] + o, idx[q + 1] + o))
            p += t
        o += len(v)
    V = np.concatenate(todos) if todos else np.zeros((0, 3))
    return caras, V, np.array(tris, dtype=int)


if __name__ == '__main__':
    caras, V, T = leer(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else 'tira')
    print('caras', len(caras), 'vértices', len(V), 'triángulos', len(T))
    print('límites (mm)', (V.min(0) * 1000).round(1), (V.max(0) * 1000).round(1))
