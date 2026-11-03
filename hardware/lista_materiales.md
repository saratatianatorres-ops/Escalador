# Lista de materiales

Completa las columnas vacías con tus datos (proveedor, precio, peso medido). El peso importa:
las simulaciones de `docs/fisica_transiciones.md` suponen **0,8 kg en total** y **≥ 10 N de empuje por
hélice**. Pesa el robot armado y mide el empuje de cada A2217 sobre una balanza.

## Robot

| Cant. | Componente | Función | Peso típico (g) | Peso medido (g) | Precio | Proveedor |
|---:|---|---|---:|---:|---:|---|
| 2 | Motor brushless A2217 (KV: ___) | Hélices de succión | ~70 c/u | | | |
| 2 | Hélice (tamaño: ___ pulgadas) | Empuje hacia la superficie | | | | |
| 2 | ESC 30A SimonK | Control de cada A2217 (señal en IO18) | ~25 c/u | | | |
| 4 | Motorreductor N20 (___ V, ___ RPM) | Tracción, 2 por lado | ~10 c/u | | | |
| 4 | Rueda de goma blanda o silicona (Ø 70 mm en el modelo) | Fricción ≥ 0,9 | | | | |
| 1 | Módulo DRV8833 | Puente H de los N20 | ~3 | | | |
| 1 | ESP32 DevKitC-32D | Nodo micro-ROS del robot | ~10 | | | |
| 1 | Módulo MPU6050 | IMU (orientación) | ~2 | | | |
| 1 | LM2596 ajustable (salida 5 V) | Regulador para ESP32 y N20 | ~20 | | | |
| 1 | Batería LiPo (___ S, ___ mAh) | Alimentación | | | | |
| 3 | Bornera de 3 pines (J1, J2, J3) | ESC y motores | | | | |
| 1 | Placa de cobre (PCB de una cara) | Placa del robot | | | | |
| 1 | Chasis | | | | | |

## Estación en el PC

| Cant. | Componente | Función |
|---:|---|---|
| 1 | ESP32 (cualquier DevKit) | Puente USB ⇄ ESP-NOW (`firmware/puente_pc`) |
| 1 | Cable USB de datos | PC ⇄ ESP32 puente |

**Peso total medido:** ______ g · **Empuje medido por hélice al 100 %:** ______ g
