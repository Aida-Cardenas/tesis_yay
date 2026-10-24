# Respuesta a la tercera revisión de código

Qué se hizo con cada punto de la tercera revisión (21 de octubre de 2026). Las respuestas anteriores están en [`RESPUESTA_REVISION.md`](RESPUESTA_REVISION.md) y [`RESPUESTA_REVISION_2.md`](RESPUESTA_REVISION_2.md).

## Medias

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| R3-M1 | La interfaz integra para siempre la última velocidad si el Arduino deja de responder | Corregido | Nueva clase `WheelIntegrator` (`serial_protocol.cpp`). Guarda la hora, con reloj monótono, de la última lectura válida; si pasan más de 0,3 s (tres periodos del Arduino), pone las velocidades en cero y deja de avanzar la posición. Las líneas basura no cuentan como lectura. La interfaz avisa, como mucho cada 2 s, mientras no hay datos, y avisa cuando vuelven | gtest: mantiene la velocidad entre lecturas, se detiene tras 0,3 s sin datos (a 5 rad/s avanza como máximo 0,3 s y no 1,5 s), la basura no mantiene vivos los datos, no hay movimiento antes de la primera lectura |
| R3-M2 | Dos repeticiones por grupo no bastan para la prueba de Welch | Corregido | `protocolo_real` y `protocolo_simulacion` repiten 5 veces todo experimento que entra en una comparación. En el real son E1–E4, E7, G0T y los siete de latencia, de 42 a 73 corridas. El informe agrega la diferencia A − B con su intervalo de confianza del 95 % (Welch), el n de cada grupo, y marca como "no concluyente (n < 5)" cualquier comparación con menos de 5 corridas por grupo, aunque su p sea pequeño. Sin scipy no se informa p: la aproximación normal da valores demasiado pequeños con 1 a 4 grados de libertad | `test_compared_experiments_have_enough_runs` falla si un protocolo compara un experimento con menos de 5 repeticiones; `test_welch_interval_matches_reference` (contra scipy); `test_few_runs_are_not_conclusive_even_with_small_p`; `test_welch_gives_no_p_without_scipy` |

`protocolo_ci` se queda en 2 repeticiones para que la verificación no tarde mucho más. Sus comparaciones salen ahora marcadas como no concluyentes, y en `GEMELO_DIGITAL.md` sus valores de p figuran como indicativos. D vs DC dice "no concluyente".

## Bajas

| # | Observación | Estado | Cambio |
|---|---|---|---|
| R3-L1 | La respuesta exageraba la comprobación de D/DC | Corregido | `RESPUESTA_REVISION_2.md` dice ahora que la integración exige que DC no empeore el error real de D en más de un 50 % |
| R3-L2 | La explicación de los ~10 ms del retardo no está demostrada | Corregido | Se reemplazó por "se parece más a ruido de cuantización (pasos de Euler a 50 Hz del robot de prueba y registro a 20 Hz) que a un atraso sistemático; no se verificó la causa" |
| R3-L3 | La calibración abría un registro que grababa el reposicionamiento | Corregido | `calibrate()` cierra el registro (`close_log=True`); prueba en `test_runner_logic.py` |
| R3-L4 | Nombre fijo por usuario del archivo de parámetros fusionado | Corregido | `tempfile.mkstemp`, y un `OnShutdown` que lo borra al cerrar el launch |
| R3-L5 | El puente seguía informando la ruta de un registro cerrado | Corregido | `_close_log()` deja `csv_path` vacío |
| R3-L6 | "Robot de prueba exacto" era generoso | Corregido en la documentación | El filtro del gemelo sí aplica el retardo exacto; del robot de prueba ya no se dice que lo sea |

## Otros

- **Reintento de Gazebo.** CI publica un aviso visible ("Gazebo necesitó un reintento") cada vez que el primer intento falla, para poder ver si se vuelve frecuente.
- **N-F1.** Sigue pendiente en el robot: la comprobación del sentido de giro del paso 4 del README, antes de cualquier otra prueba física.

## Resultados de CI

Corrida del 24 de octubre de 2026 (commit `2318800`): todas las pruebas aprobadas. Gazebo con Nav2 pasó al primer intento (no apareció el aviso de reintento) y los siete nodos de Nav2 quedaron en `active [3]` con el robot simulado (mock).

| Comparación en `protocolo_ci` (error real, n = 2: no concluyente por diseño) | Sin | Con | p |
|---|---|---|---|
| M vs MC: gemelo calibrado | 23,9 cm | 2,4 cm | 0,003 |
| D vs DC: compensación, gemelo seguidor, 200 ms | 2,0 cm | 1,7 cm | 0,127 |
| R vs RC: predictor de Smith, real seguidor, 300 ms | 24,6 cm | 14,1 cm | 0,045 |
| R vs RT: ganancias sintonizadas, real seguidor, 300 ms | 24,6 cm | 12,8 cm | 0,031 |

En la corrida anterior del mismo día, D vs DC dio p = 0,173, en línea con lo de R3-M2: con 2 corridas, el p de una diferencia pequeña salta de una corrida a otra.

Retardo identificado en las tres últimas corridas de CI, con 0,1 s configurados: lineal 0,088, 0,106 y 0,110 s; angular 0,106, 0,094 y 0,091 s. Quedan a ambos lados de 0,1 s, lo que apoya leer la diferencia como ruido de cuantización y no como un atraso sistemático (R3-L2).
