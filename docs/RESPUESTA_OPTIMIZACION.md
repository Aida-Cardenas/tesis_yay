# Optimizaciones después de la revisión general

Cambios del 26 de octubre de 2026, hechos antes de cargar el firmware y de tomar datos con el robot. Cada uno mantiene lo que pidieron las revisiones anteriores ([1](RESPUESTA_REVISION.md), [2](RESPUESTA_REVISION_2.md), [3](RESPUESTA_REVISION_3.md), [4](RESPUESTA_REVISION_4.md)); la tabla dice cómo. La revisión de errores y seguridad previa está en [`REVISION_GENERAL.md`](REVISION_GENERAL.md).

## 1. Odometría a partir de los contadores de los encoders

**Antes.** El Arduino mandaba solo la velocidad de cada rueda cada 100 ms, y la Pi la multiplicaba por su propio periodo (10 ms) para obtener la posición. Los dos intervalos nunca coinciden exactamente, así que la odometría acumulaba un error que no venía de las ruedas, y cada línea perdida era distancia perdida. La resolución era de 385 cuentas por vuelta (un flanco de la fase A): a 0,15 m/s, un pulso de más o de menos en 100 ms era un 3,6 % de la velocidad.

**Ahora.**

- **Sketch (`robot_control.ino`).**
  - Cuenta los dos flancos de la fase A (770 cuentas por vuelta, error de cuantización 1,8 % a 0,15 m/s). Cada flanco suma o resta según la fase B, así que el contador sabe el sentido sin una variable aparte.
  - Agrega a cada línea el contador de cada rueda (`rp4.55,ln0.12,rc1234,lc65000,`). La velocidad y su signo salen de la diferencia de cuentas en el intervalo medido.
  - El PID recibe la velocidad en el sentido ordenado: una rueda que todavía gira al revés cuenta como negativa y el PID empuja más, en vez de creer que ya se mueve.
  - Las dos ruedas comparten el mismo código (`struct Wheel`): el cambio de sentido, el modo del PID y la lectura del encoder ya no están escritos dos veces.
- **Interfaz (`serial_protocol.cpp`, `bumperbot_interface.cpp`).**
  - La posición de cada rueda sale del contador, así que es exacta en cada lectura aunque se pierdan líneas. El contador de 16 bits da la vuelta y se usan diferencias, válidas aunque pase de 65535 a 0.
  - Entre lecturas la posición avanza con la última velocidad (para que `diff_drive_controller` vea un movimiento suave), y la lectura siguiente la devuelve al valor exacto.
  - Las cuentas por vuelta son el parámetro `encoder_counts_per_rev` del URDF (770).
  - Si llegan líneas sin contadores, el aviso dice que el sketch cargado es viejo.

| Lo que pidió la revisión | Cómo queda |
|---|---|
| R3-M1: sin lecturas por más de 0,3 s, velocidad cero y la posición deja de avanzar | Igual; además la posición vuelve a la última exacta en vez de quedarse con lo extrapolado |
| R4-L2: una lectura cuenta solo si trae las dos ruedas | Ahora tiene que traer velocidad y contador de las dos (`WheelReading::complete`) |
| R4-L3 y R4-L4: cero sin realimentación, reconexión cada 2 s | Sin cambios; al reconectar, el contador del Arduino reiniciado se toma como nueva referencia y la posición no salta |
| R4-L5: PID con `SetSampleTime(90)` y Ki, Kd reescalados | Igual; las ganancias se reescalan a partir de las del constructor, sin repetirlas |
| R4-L6: velocidad con el tiempo medido con `micros()`; "no se cambió el protocolo para enviar pulsos acumulados" | La velocidad se sigue dividiendo por el tiempo medido, y ahora el protocolo sí envía los pulsos acumulados |
| R4-M5: PID en MANUAL con consigna cero y al cambiar de sentido | Igual (`updateMode`, `setDirection`) |
| R4-M5: la calibración (CAL) se hace con el firmware final | Cambia la planta, así que CAL tiene que hacerse con este firmware; todavía no se tomaron datos |

Pruebas: gtests nuevos (posición desde los contadores, líneas perdidas, contador que da la vuelta, extrapolación y vuelta al valor exacto, reconexión con contador reiniciado, contadores mal formados). El sketch compila en CI para Uno y Nano. En el robot hay que repetir el paso 5 del README: además de `rp` y `lp`, los contadores `rc` y `lc` tienen que aumentar al avanzar.

## 2. Sintonización: desempate hacia las ganancias por defecto (punto abierto de R4-L16)

**Antes.** En la simulación, el costo casi no cambia con ky (11,4 cm con ky = 0,5 y 11,5 cm con ky = 4,1), así que la búsqueda terminaba en un ky distinto en cada corrida de CI y RT no se podía repetir.

**Ahora.** Entre todas las combinaciones simuladas cuyo costo queda a menos de un 1 % del mejor, se elige la más cercana a 1,5 / 6 / 3 en escala logarítmica (`closest_to_start`, `TIE_TOLERANCE`). Solo se aparta de los valores por defecto si eso mejora el costo más de un 1 %. Sigue sin devolver algo peor que las ganancias de partida, los límites de R4-L16 no cambian y el aviso de ganancia en el límite se mantiene. El informe de la sintonización explica el desempate. Prueba: `test_near_ties_are_broken_toward_the_starting_gains`.

## 3. Operación

- **`tmux` en la Raspberry Pi.** Si el robot se lanza por SSH y el Wi-Fi se corta un momento, el launch se cierra con la sesión y se pierde la corrida. El README explica cómo lanzarlo dentro de `tmux`.
- **Gazebo sin ventana para la toma de datos** (`gui:=false`). La ventana es lo que más frena a Gazebo, y R4-L14 marca como aviso de calidad cualquier corrida en la que el gemelo no alcance el tiempo real. Está en el README y en la cabecera de `protocolo_real.yaml`.

## Lo que se revisó y no se cambió

- **Predictor de Smith:** tarda menos de 1 ms por ciclo de 50 ms; no hacía falta optimizarlo.
- **Periodo del Arduino (100 ms):** bajarlo empeora la resolución de la velocidad, y R4-L5 y R4-L6 están ajustados a él.
- **Frecuencia de los comandos (100 Hz):** sobra, pero no causa ningún problema medible.
- **Carga de la Pi (AMCL, Nav2):** se cambia solo si, medida con todo corriendo, pasa de 70 %.
- **DDS:** se cambia solo si aparecen problemas de descubrimiento por Wi-Fi.
- **Ley de control, modelo, calibración y estadística:** sin cambios.
