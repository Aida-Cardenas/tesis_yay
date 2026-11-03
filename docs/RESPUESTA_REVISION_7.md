# Respuesta a la séptima revisión de código

Qué se hizo con cada punto de la séptima revisión (2 de noviembre de 2026, sobre `ca4b002`). La revisión no encontró nada alto ni medio y dice que nada impide la toma de datos; pidió un punto bajo antes de la sesión (HW-8) y dos de mantenimiento. Las respuestas anteriores están en [`RESPUESTA_REVISION_6.md`](RESPUESTA_REVISION_6.md) y las que esta enlaza.

**Resultado de CI:** pendiente.

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| HW-8 | En las paradas cortas de los experimentos (1-2 periodos del Arduino), la tendencia sale de una lectura mitad crucero y mitad frenada, es demasiado débil, y la odometría muestra velocidad inversa al final de cada parada | Corregido | La predicción tampoco pasa de la última velocidad enviada al Arduino en el mismo sentido (`advance()` recibe los dos comandos). `write()` guarda lo que mandó de verdad: 0 con `kSendZero`, después de una falla o cuando no escribe. La rueda va detrás de su comando, así que en una parada el tope queda por debajo de la velocidad real: la predicción se queda corta y la lectura siguiente mueve la posición hacia adelante, nunca hacia atrás. En crucero y al acelerar el comando está a la par o por encima de la velocidad medida y no cambia nada. Se mantienen la tendencia (HW-3) y el límite al acelerar (HW-6) | `ShortStopsDoNotShowAReverseSpeed`: paradas desde 4,5 rad/s (línea a 0,15 m/s) y 1,5 rad/s (giro en el lugar a 0,6 rad/s), a 21 y 34 rad/s², con la rueda 50 ms detrás del comando, velocidades redondeadas a 2 decimales, 770 cuentas por vuelta y 10 fases contra las lecturas: la velocidad de diff_drive nunca es negativa. Sin el comando, los 40 casos fallan. Las pruebas de HW-3 y HW-6 siguen pasando |
| N7-1 | Dentro de la repetición antes de CAL, `[s]` y `[x]` se comportaban distinto del lazo principal | Corregido | `[x]` muestra el mismo mensaje que el lazo principal ("No hay una corrida anterior…") y vuelve a preguntar; `[s]` dice que la corrida queda fuera de la calibración y de la sesión | `test_inside_the_redo_before_calibration_x_asks_again_and_s_says_what_happens` |
| N7-2 | Después de una falla, un reinicio del Arduino sin relanzar deja el robot detenido sin aviso | Corregido | Mientras haya una falla registrada, la Pi repite cada 5 s: "A wheel fault was seen: only zero speed is sent until real_robot.launch.py is restarted" | — |
| Docs | La redacción de HW-3 y HW-6 valía solo para frenadas largas | Corregido | Notas en `RESPUESTA_REVISION_5` (HW-3), `RESPUESTA_REVISION_6` (HW-6), `RESPUESTA_OPTIMIZACION` y el comentario de `WheelIntegrator` | — |

## Pruebas

- gtest de `bumperbot_firmware`: 30 pruebas (antes 29).
- pytest de `bumperbot_digital_twin` sin la integración: 163 aprobadas, 1 omitida (el panel, sin PyQt5), con mensajes de ROS de reemplazo; en CI corren contra Humble.

## Lo que sigue en el robot

Igual que antes: cargar el sketch nuevo, README paso 5 con las ruedas en el aire (ahora también prueba el corte por encoder suelto) y paso 3 (chrony), y `protocolo_real`.
