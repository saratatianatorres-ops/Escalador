# Lista de materiales y presupuesto de peso

Los pesos marcados como **medido** son los que pesó el equipo (8 de octubre). Los **estimados**
hay que confirmarlos con una balanza. El URDF (`robot_trepador.urdf.xacro`) y las simulaciones
usan exactamente estos valores.

## Robot

Supuesto actual (peor caso que dio el equipo): **bases 2 × 220 g + todo lo demás 785 g = 1 225 g**.

| Grupo | Cant. | Componente | Peso (g) | Origen | Notas |
|---|---:|---|---:|---|---|
| Estructura | 2 | Base impresa v2 (`cad/Base_2_v2.STL` y `Base_1_v2.STL`) | 441 | medido 2 × 220 + nariz v2 | Ducto, soporte del A2217 y soportes de los N20 |
| Adhesión | 2 | Motor brushless A2217 2300KV | 153 | medido | 909 gf por motor a 23 A, batería entre 11,7 y 11,3 V |
| | 2 | Hélice + adaptador | 19 | medido | |
| Tracción | 4 | Motorreductor N20 + tornillos/pegante del soporte | 52 | medido 40 + estimado 12 | Uno por llanta, dos por lado en paralelo |
| | 4 | Goma JSumo SLT20 + rin (Ø33 × 21 mm) | 48 | estimado | Silicona, fricción ≈ 1. $ 33.000 + IVA (revisa si es por par) |
| Electrónica y energía | — | Batería 3S 2200 mAh (180), porta pilas (15), BMS (4), 2 ESC 30A SimonK, PCB (52), ESP32 (10), buck (10), regletas (5), IMU (2), puente H (2), tornillería y cables | 513 | 785 − lo de arriba | Peor caso |
| | | **Total** | **≈ 1 225** | | |

**Relación empuje/peso a 1700 µs:** 2 × 909 gf / 1 225 g = **1,48** con la batería entre 11,3 y 11,7 V.
En el techo, lo que aprieta las llantas contra la superficie es el empuje **menos** el peso: unos
590 gf en total (≈ 150 gf por llanta). Con la batería más baja, el empuje cae con el cuadrado del
voltaje y ese margen se achica. Cada gramo que se le quite al robot es agarre que se gana.

## Para medir la batería (agregar)

| Cant. | Componente | Función |
|---:|---|---|
| 1 + 1 | Resistencias 100 kΩ y 22 kΩ | Divisor de la batería 3S hacia el IO34 |
| 1 | Condensador cerámico 100 nF | Filtro del divisor |

## Placa

| Cant. | Componente |
|---:|---|
| 3 | Bornera de 3 pines (J1, J2, J3) |
| 1 | Placa de cobre de una cara |

## Estación en el PC

| Cant. | Componente | Función |
|---:|---|---|
| 1 | ESP32 (cualquier DevKit) | Puente USB ⇄ ESP-NOW (`firmware/puente_pc`) |
| 1 | Cable USB de datos | PC ⇄ ESP32 puente |
