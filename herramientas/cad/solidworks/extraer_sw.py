"""Extrae las secciones comprimidas (deflate) de un archivo SolidWorks 2015+."""
import struct, sys, zlib, os
ruta, salida = sys.argv[1], sys.argv[2]
d = open(ruta, 'rb').read()
marca = b'\x14\x00\x06\x00\x08\x00'
i = 0; n = 0
def desofuscar(b):   # el nombre va con los bits rotados 4 posiciones
    return bytes(((c << 4) | (c >> 4)) & 0xFF for c in b)
while True:
    i = d.find(marca, i)
    if i < 0: break
    try:
        comp, desc, lnom = struct.unpack_from('<III', d, i + 14)
        nombre = desofuscar(d[i + 26:i + 26 + lnom]).decode('latin1', 'replace')
        ini = i + 26 + lnom
        datos = zlib.decompressobj(-15).decompress(d[ini:ini + comp])
        seguro = ''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in nombre)[:80]
        open(os.path.join(salida, f'{n:03d}_{seguro}'), 'wb').write(datos)
        print(f'{n:03d} {len(datos):>9} {nombre}')
        n += 1
    except Exception as e:
        pass
    i += 6
print('secciones:', n)
