#!/usr/bin/env python3
"""
Genera la Base v2 (nariz con las llantas adelantadas) a partir de hardware/cad/Base_2.STL.

Cambios respecto a la Base_2 original, en cada esquina delantera:
  - Soporte del N20 integrado: un bloque con un túnel de 12,3 × 10,3 mm donde entra el
    motor desde adentro. La caja reductora queda dentro de la llanta (la JSumo SLT20 tiene
    una cavidad de Ø18 × 14,5 mm) y el bloque sujeta la parte del motor que queda afuera.
  - Muesca en la esquina de la placa para que la llanta adelantada no la toque.

La llanta sobresale SALIENTE mm por delante del chasis. El estudio de MuJoCo
(herramientas/fisica_mujoco/nariz.py) muestra que desde 10 mm el robot pasa del suelo a
la pared; se usan 20 mm para tener margen.

Coordenadas del STL (mm): X = largo (0 = lado de la repisa, 225 = frente), Y = arriba
(0 = cara inferior de la placa), Z = ancho (0..216, centro en 108).

Uso:  pip install trimesh manifold3d numpy
      python3 base_v2.py            -> hardware/cad/Base_2_v2.STL y Base_1_v2.STL (espejo)
"""
import os

import numpy as np
import trimesh

AQUI = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.normpath(os.path.join(AQUI, '..', '..', 'hardware', 'cad'))

# ---------------- Parámetros (mm) ----------------
SALIENTE = 20.0          # frente de la llanta por delante del chasis (X = 225)
R_RUEDA = 16.5           # JSumo SLT20
ANCHO_RUEDA = 21.0
EJE_BAJO_PLACA = 5.0     # el eje queda 5 mm por debajo de la cara inferior de la placa
FRENTE = 225.0
ANCHO = 216.0

N20_ANCHO, N20_ALTO = 12.0, 10.0     # sección del motor (lados planos horizontales)
HOLGURA = 0.3                        # para impresión FDM
PARED = 2.4                          # espesor de pared del soporte
LARGO_SOPORTE = 9.0                  # cuánto del motor sujeta el bloque (a lo largo del eje)
SEPARACION_LLANTA = 1.0              # entre la cara interior de la llanta y el soporte
RESERVA_MUESCA = 2.0                 # holgura entre la llanta y la placa

EJE_X = FRENTE + SALIENTE - R_RUEDA  # 228,5 mm
EJE_Y = -EJE_BAJO_PLACA


def caja(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def cilindro_z(x, y, z0, z1, r):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=96)
    c.apply_translation([x, y, (z0 + z1) / 2])
    return c


def main():
    base = trimesh.load(os.path.join(CAD, 'Base_2.STL'))
    agregar, quitar = [], []

    for lado in (0, 1):                                  # Z bajo y Z alto (espejo)
        def z(a, b):
            return (a, b) if lado == 0 else (ANCHO - b, ANCHO - a)

        # 1. Muesca para la llanta: cilindro un poco más grande que la llanta
        z0, z1 = z(-1.0, ANCHO_RUEDA + SEPARACION_LLANTA)
        quitar.append(cilindro_z(EJE_X, EJE_Y, z0, z1, R_RUEDA + RESERVA_MUESCA))

        # 2. Soporte del N20 (bloque + brazo que se une a la placa)
        zs0, zs1 = z(ANCHO_RUEDA + SEPARACION_LLANTA,
                     ANCHO_RUEDA + SEPARACION_LLANTA + LARGO_SOPORTE)
        mx = N20_ANCHO / 2 + HOLGURA / 2 + PARED
        my = N20_ALTO / 2 + HOLGURA / 2 + PARED
        bloque = caja(EJE_X - mx, EJE_X + mx, EJE_Y - my, 10.0, zs0, zs1)
        brazo = caja(200.0, EJE_X, 0.0, 10.0, zs0, zs1)
        agregar += [bloque, brazo]
        # túnel del motor (atraviesa el bloque a lo largo de Z)
        quitar.append(caja(EJE_X - N20_ANCHO / 2 - HOLGURA / 2, EJE_X + N20_ANCHO / 2 + HOLGURA / 2,
                           EJE_Y - N20_ALTO / 2 - HOLGURA / 2, EJE_Y + N20_ALTO / 2 + HOLGURA / 2,
                           zs0 - 1, zs1 + 1))

    v2 = trimesh.boolean.union([base] + agregar, engine='manifold')
    v2 = trimesh.boolean.difference([v2] + quitar, engine='manifold')
    assert v2.is_watertight, 'la pieza no quedó cerrada'

    salida2 = os.path.join(CAD, 'Base_2_v2.STL')
    v2.export(salida2)
    # Base_1: espejo en X (el frente de la Base_1 es su extremo, la repisa va a la unión)
    v1 = v2.copy()
    v1.apply_transform(np.diag([-1, 1, 1, 1]))
    v1.apply_translation([FRENTE, 0, 0])
    v1.invert()                                          # el espejo invierte las caras
    v1.fix_normals()
    v1.export(os.path.join(CAD, 'Base_1_v2.STL'))

    print(f'Base_2_v2.STL: {v2.extents.round(1)} mm, {v2.volume/1000:.1f} cm³ '
          f'(original {base.volume/1000:.1f} cm³, {100*(v2.volume/base.volume-1):+.1f} %)')
    print(f'Eje de la llanta en X = {EJE_X:.1f} mm, Y = {EJE_Y:.1f} mm; '
          f'la llanta sobresale {SALIENTE:.0f} mm del chasis')


if __name__ == '__main__':
    main()
