# Física de las transiciones suelo → pared → techo

El robot tiene que pasar de un plano a otro **por sí mismo**, sin cuerdas. Este documento resume qué hace
falta para lograrlo, cómo se comprobó en simulación y qué reglas de diseño salen de ahí.

> **Estado actual del robot:** los dos ESC comparten la señal (IO18), así que todavía **no** puede hacer el
> reparto de empuje que se describe abajo. Para eso cada ESC necesita su propio pin
> (ver [hardware/README.md](../hardware/README.md) y [arquitectura.md](arquitectura.md#5-próximo-paso-supervisor-idea-tomada-de-carro_trepador)).

## El problema

Un robot de ruedas que choca contra una pared a 90° no sube solo: para levantar la nariz, la rueda
delantera tiene que "trepar" la pared con fricción, y esa fricción depende de qué tan fuerte la empujan
las ruedas de atrás. Con el empuje de las hélices apretando el robot contra el suelo, la nariz queda clavada.

Haciendo el balance de momentos alrededor de la rueda trasera, el robot levanta la nariz si:

```
N_pared · (μ · entre_ejes + r)  ≥  momento que aprieta la nariz contra el suelo
```

El lado izquierdo crece con la fricción μ y el radio de rueda r. El derecho crece con el empuje que esté
**delante** de la rueda trasera. De ahí la idea clave:

> **Si la hélice delantera baja casi a cero y la trasera (encima del eje trasero) va al máximo, las ruedas
> traseras tienen mucha tracción para empujar contra la pared, pero casi no hay nada que apriete la nariz.**
> La nariz se levanta y, en cuanto el robot se inclina, el empuje ya apunta contra la pared y lo termina de pegar.

## Cómo se comprobó (MuJoCo)

Paso de 1 ms y contactos con fricción elíptica. Robot de 0,8 kg con 4 ruedas motrices de 35 mm de radio
y par limitado a 0,2 N·m por rueda. El empuje de cada hélice se aplicó en el eje z del robot. Habitación
de 1 m con suelo, pared y techo, y el robot avanzando a 0,1 m/s todo el tiempo.

### 1. Sin repartir el empuje: no sube

| Fricción | Empuje por hélice | Resultado |
|---|---|---|
| 0,8 y 1,0 | 12 a 18 N | 15 de 16 combinaciones: se queda contra la pared, sin subir |

### 2. Con el reparto, hélices simétricas (±7 cm del centro)

Suelo → pared funciona siempre. Pared → techo: nunca con fricción 0,8 (0 de 8) y solo a veces con 1,0
(3 de 8), porque en la pared la gravedad se come la tracción justo cuando la nariz tiene que despegarse.

### 3. Con el reparto y las hélices corridas hacia atrás (delantera +4 cm, trasera −9 cm)

| Fricción | Empuje por hélice | Reparto en pared | Radio de rueda | Resultado |
|---|---|---|---|---|
| 0,9 | 10 N | 30 % o 50 % | 35 o 42 mm | **suelo → pared → techo** (4 de 4) |
| 1,0 | 10 a 12 N | 30 % o 50 % | 35 o 42 mm | **suelo → pared → techo** (8 de 8) |

### 4. ¿Cuánta succión hace falta? (fricción 0,9, 10 N por hélice al 100 %)

| Acelerador | Resultado |
|---|---|
| 60 % | llega a la pared pero queda atascado en la esquina de abajo |
| 70 % | igual, y además se desvía de lado |
| 80 % | **sube y se cae**: el peor caso |
| 90 % | suelo → pared → techo |
| 100 % | suelo → pared → techo |

Es decir: el reparto solo funciona con **unos 9 N o más por hélice**. Con menos, es mejor que el robot
se quede contra la pared que intentar una subida que no va a lograr.

> **Tu robot tiene el acelerador limitado a 1700 µs** para que los A2217 no pasen de ~23–25 A y no se
> quemen. Lo que cuenta entonces no es el porcentaje sino **cuántos newtons empuja cada hélice a 1700 µs**.
> Mídelo sobre una balanza: si da 9–10 N o más (0,9–1 kgf), vale todo lo de esta tabla. Si da menos, el robot
> estará en la zona de "se queda atascado" o "sube y se cae". Para tener más empuje sin subir el
> acelerador habría que cambiar de hélice o de batería, y eso **también sube la corriente**: compruébalo
> con un vatímetro antes de usarlo.

### 5. Ida y vuelta

- **De reversa no baja**: en el techo, retrocediendo hacia la pared, se queda (seguro, sin caerse) contra la esquina.
- **Girando 180° y avanzando de frente, sí**: suelo → pared → techo, media vuelta en el techo,
  techo → pared → suelo. La media vuelta en el techo es lenta (unos 28 s) porque las 4 ruedas arrastran
  de lado con mucha fuerza normal.

## Reglas de diseño para el robot real

1. **Las ruedas delanteras tienen que ser lo primero que toca la pared.** Nada puede sobresalir por delante
   de ellas: ni el chasis, ni la protección de las hélices.
2. **Hélice trasera casi encima del eje trasero, delantera cerca del centro.** En el modelo: +4 cm y −9 cm
   desde el centro, con los ejes a ±9,5 cm.
3. **Tracción en las 4 ruedas.** Tu robot ya la tiene: un N20 por rueda, los dos de cada lado en paralelo en
   el DRV8833. La rueda delantera tiene que girar para trepar.
4. **Fricción de las ruedas ≥ 0,9** sobre la superficie: goma blanda o silicona. Con 0,8 no pasa al techo
   de forma confiable. Para medirla, pon el robot sobre la superficie e inclínala hasta que deslice: μ = tan(ángulo).
5. **Empuje ≥ 10 N por hélice a 1700 µs**, el máximo permitido (unos 1 kgf; los dos juntos ≈ 2,5 veces el
   peso) con un robot de 0,8 kg. Mide el empuje real de tus A2217 con su hélice sobre una balanza, y la
   corriente con un vatímetro, antes de confiar en el techo.
   Si tu robot pesa más, el empuje tiene que subir en la misma proporción.
6. **Centro de masa** cerca del centro o un poco hacia atrás (la batería atrás ayuda).

## Límites de lo que se validó

- **Gazebo no reproduce las esquinas.** Con las mismas medidas, en Gazebo (DART) la rueda delantera gira en
  falso contra la superficie nueva y el robot no levanta la nariz, ni siquiera con el reparto. Detalles en
  [simulacion.md](simulacion.md). Las transiciones hay que validarlas con MuJoCo o con el robot real.
- Solo **esquinas interiores** (suelo con pared, pared con techo). Las esquinas exteriores (el borde de una
  mesa, el canto de una pared) no se estudiaron y este diseño no las pasa.
- El empuje se modeló proporcional al acelerador; en un ESC real es aproximado. Por eso conviene medir.
- Sin encoders, la odometría estima la velocidad desde el PWM: en la pared el robot va más lento de lo que
  cree, y la posición se desvía con el tiempo (la orientación no, porque la da la IMU).

## Repetir las simulaciones de MuJoCo

Los scripts (`uturn.py`, `sweep2.py`) se hicieron en una conversación anterior y **no están en este
repositorio**. Si los tienes, guárdalos en `herramientas/fisica_mujoco/`:

```bash
pip install mujoco numpy
cd herramientas/fisica_mujoco
python3 uturn.py 0.9 10        # fricción 0,9, 10 N por hélice: ida, media vuelta y vuelta
python3 sweep2.py 0.9 1.0      # barrido de empuje con hélices simétricas
```
