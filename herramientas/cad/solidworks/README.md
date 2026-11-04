# Del ensamble de SolidWorks al URDF

Con esto se sacaron las mallas del URDF a partir de `hardware/cad/Diseno_2.SLDASM`, sin SolidWorks.
Un `.SLDASM` no trae las piezas `.SLDPRT`, pero sí guarda la **geometría de visualización** de cada
pieza (secciones `FaceTessellations/...`) y la **posición** de cada componente (`swXmlContents/COMPINSTANCETREE`).

| Script | Qué hace |
|---|---|
| `extraer_sw.py ARCHIVO.SLDASM salida_sw/` | Descomprime todas las secciones del archivo |
| `teselas.py` | Lee las caras teseladas: tiras de triángulos, vértices y normales en metros |
| `extraer_piezas.py` | Corta la base, el soporte del motor, el A2217 y la llanta (posiciones propias de este archivo) y las guarda en `mallas_sw/` |
| `armar.py` | Posiciones de cada componente (copiadas de `COMPINSTANCETREE`) y paso al marco de ROS |
| `exportar_urdf.py` | Escribe las mallas en `ros2_ws/src/robot_trepador_description/meshes/` |

```bash
pip install numpy trimesh
python3 extraer_sw.py ../../../hardware/cad/Diseno_2.SLDASM salida_sw
python3 extraer_piezas.py && python3 armar.py && python3 exportar_urdf.py
```

Los cortes de `extraer_piezas.py` y las transformadas de `armar.py` son de **esta versión** del ensamble.
Si cambias el diseño, lo más cómodo es exportar desde SolidWorks con el complemento
**SolidWorks to URDF Exporter** (sw_urdf_exporter), o guardar cada pieza como STL con la opción
"no trasladar a espacio positivo", y pasarme los archivos.
