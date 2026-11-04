# Lista de materiales y presupuesto de peso

Los pesos marcados como **medido** son los que pesó el equipo (8 de octubre). Los **estimados**
hay que confirmarlos con una balanza. El URDF (`robot_trepador.urdf.xacro`) y las simulaciones
usan exactamente estos valores.

## Robot

| Grupo | Cant. | Componente | Peso (g) | Origen | Notas |
|---|---:|---|---:|---|---|
| Estructura | 2 | Base impresa v2 (`cad/Base_2_v2.STL` y `Base_1_v2.STL`) | 692 | medido 690 + nariz v2 | Ducto, soporte del A2217 y soportes de los N20 |
| Adhesión | 2 | Motor brushless A2217 2300KV | 153 | medido | 909 gf por motor a 23 A, batería entre 11,7 y 11,3 V |
| | 2 | Hélice + adaptador | 19 | medido | |
| | 2 | ESC 30A SimonK | 50 | estimado | Señal compartida en IO18; tope de 1700 µs |
| Tracción | 4 | Motorreductor N20 | 40 | medido (2 × 10 g) | Uno por llanta, dos por lado en paralelo |
| | 4 | Tornillos o pegante de los soportes del N20 | 12 | estimado | El soporte ya va impreso en la base |
| | 4 | Goma JSumo SLT20 + rin (Ø33 × 21 mm) | 48 | estimado | Silicona, fricción ≈ 1. $ 33.000 + IVA (revisa si es por par) |
| | 1 | Puente H DRV8833 | 2 | medido | |
| Electrónica | 2 | PCB | 52 | medido | |
| | 1 | ESP32 DevKitC-32D | 10 | medido | |
| | 1 | Buck LM2596 (salida 5 V) | 10 | medido | |
| | 1 | Regletas | 5 | medido | |
| | 1 | BMS | 4 | medido | |
| | 1 | IMU MPU6050 | 2 | medido | |
| Energía | 1 | Batería 3S (11,1 V) 2200 mAh | 180 | medido | |
| | 1 | Porta pilas | 15 | medido | |
| Varios | — | Tornillería, cableado | 25 | estimado | |
| | | **Total** | **≈ 1 319** | | |

**Relación empuje/peso a 1700 µs:** 2 × 909 gf / 1 319 g = **1,38** con la batería entre 11,3 y 11,7 V.
En el techo, lo que aprieta las llantas contra la superficie es el empuje **menos** el peso: unos
500 gf en total (≈ 125 gf por llanta). Con la batería más baja, el empuje cae con el cuadrado del
voltaje y ese margen se acaba rápido. Cada gramo que se le quite al robot es agarre que se gana.

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
