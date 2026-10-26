# Respuesta a la cuarta revisión de código

Qué se hizo con cada punto de la cuarta revisión (24 de octubre de 2026). Las respuestas anteriores están en [`RESPUESTA_REVISION.md`](RESPUESTA_REVISION.md), [`RESPUESTA_REVISION_2.md`](RESPUESTA_REVISION_2.md) y [`RESPUESTA_REVISION_3.md`](RESPUESTA_REVISION_3.md).

**Resultado de CI:** corrida del 26 de octubre de 2026 (commit `54c6e7c`), todas las pruebas aprobadas, incluidas Gazebo con Nav2 (sin reintento), el robot con hardware simulado con Nav2 en `active [3]` y el sketch compilado para Uno y Nano. Números en `GEMELO_DIGITAL.md`, sección 14.

## Alta

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| R4-H1 | Cada par comparado se corre en dos bloques separados en el tiempo | Corregido | Los protocolos declaran grupos `interleave`. `session.build_schedule` los corre por rondas: una corrida de cada miembro por ronda, en orden aleatorio, con la semilla guardada en `sesion.json`. Cada grupo va en la posición de su último miembro, después de CAL y TUNE. `protocolo_real` se reordenó: E1/E2 (intercalados), E5 y E6 → CAL → TUNE (simulado, unos 2 min) → E3/E4/E7/G0T intercalados → L100/L100C/L250/L250C intercalados → L250R/L250RC/L250RT intercalados → A1/A2. G0T queda junto a su control E3. El voltaje de la batería se puede escribir en el mensaje de cada corrida y queda en la sesión | `test_session.py`: rondas completas, misma semilla da el mismo orden, el grupo va después de sus dependencias, errores de configuración; para los protocolos real y de simulación, todo par comparado está en el mismo grupo. En CI se ve el orden intercalado (DC_r1, D_r1, D_r2, DC_r2…) |

## Medias

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| R4-M1 | `rmse_real` acepta cualquier diferencia de relojes menor de 2 s | Corregido | `latency_echo` escribe su propia hora en el pong; el puente estima la diferencia como t_eco − (t_envío + RTT/2), con la muestra de menor RTT de las últimas 30, y la informa en `TwinConfigure`, en el CSV (`clock_offset_ms`) y en los metadatos. `twin_experiment` la mide antes de empezar: avisa si supera 10 ms y se detiene si supera 50 ms (`--max-clock-offset`, `--ignore-clock`); la guarda por corrida. `install_dependencies.sh` instala chrony: el PC sirve la hora a la red local y la Raspberry Pi la toma de él (`robot <IP del PC>`); README, paso 3. El informe dice "sin datos (relojes desincronizados)" en vez de "n < 5" cuando el error real no se puede calcular por los relojes | `test_clock_problem_is_reported_as_such`, `test_missing_experiment_and_clock_problems_are_visible` |
| R4-M2 | La sesión no se puede retomar y no hay margen si se pierde una corrida | Corregido | `sesion.json` guarda cada corrida (registro, completa, válida, motivo, voltaje, diferencia de relojes, RTT). `--resume <carpeta>` retoma la sesión con la misma semilla, salta las corridas ya hechas y reutiliza el modelo y las ganancias de la sesión (o `--model` / `--gains`). `[s]` reprograma la corrida al final de su grupo (una vez). `[x]` descarta la corrida anterior y la repite. `twin_report <carpeta de sesión>` usa solo sus corridas completas y válidas | `test_session_resume_keeps_only_complete_valid_runs`, `test_a_skipped_run_comes_back_once_at_the_end`, `test_report_from_a_session_folder_uses_only_its_valid_runs` |
| R4-M3 | Una corrida interrumpida cuenta como completa | Corregido | El registro se agrega a la lista solo después de `close_log`. Con Ctrl-C a mitad de una corrida, se detiene el recorrido, se cierra el registro, la corrida queda anotada como "interrumpida con Ctrl-C" y no entra en el informe; `finish()` lista las descartadas | `test_an_interrupted_run_does_not_count` |
| R4-M4 | 8 comparaciones a α = 0,05 sin corrección | Corregido | El informe agrega el p corregido por Holm-Bonferroni sobre las comparaciones concluyentes, y el veredicto se basa en él. Además dice cuánto sería el riesgo sin corregir. La tesis se apoya en los intervalos de confianza | `test_holm_adjustment` |
| R4-M5 | El PID del Arduino conserva el término integral en las paradas | Corregido | Con consigna cero (de ROS o del watchdog) el PID pasa a `MANUAL` con la salida en 0; al volver a una consigna distinta de cero, `SetMode(AUTOMATIC)` lo reinicia desde 0. Al cambiar de sentido, la PWM se apaga antes de invertir los pines y el PID se reinicia en el ciclo siguiente | Compila en CI (Uno y Nano). Se comprueba en el robot; como cambia la planta, la calibración del protocolo (CAL) se hace con el firmware nuevo |
| R4-M6 | El "predictor de Smith" ignora la dinámica identificada | Corregido | `smith_predict` acepta el modelo identificado: cada velocidad parte de la medida y responde a los comandos en vuelo con K, τ y L. El puente lo usa cuando hay un modelo cargado; sin modelo queda la reproducción cinemática, y `GEMELO_DIGITAL.md` describe las dos. La sintonización simula lo mismo | `test_model_based_smith_predictor_tracks_a_lagged_robot`: con K = 0,85, τ = 0,3 s, L = 0,1 s y 150 ms de red, el error RMS de la predicción baja de 16 mm (reproducción cinemática) a prácticamente cero, porque el modelo coincide con la planta. En CI, RC quedó en 14,3 cm (14,1 cm con la reproducción cinemática en la corrida anterior): con el robot de mentira, la mejora no se nota en el error real |

## Bajas

| # | Observación | Cambio |
|---|---|---|
| R4-L1 | Aviso falso con "inf" en cada arranque | Mientras no llega el primer dato: un INFO "Waiting for the first data from the Arduino"; aviso solo si tarda más de 3 s |
| R4-L2 | Una lectura de una sola rueda mantiene vivos los datos | Una lectura cuenta solo si trae las dos ruedas (gtest `OneWheelReadingsDoNotKeepTheDataAlive`) |
| R4-L3 | Sin realimentación se siguen mandando comandos | Con más de 1 s sin datos (después del primero) se envía velocidad cero |
| R4-L4 | Sin reconexión y error sin límite | Error con throttle; si falla la lectura o la escritura, el puerto se intenta reabrir cada 2 s |
| R4-L5 | PID_v1 se salta ciclos | `SetSampleTime(90)` con Ki y Kd reescalados para que el controlador se comporte como con 100 ms |
| R4-L6 | La velocidad supone 100 ms exactos | Se divide por el intervalo medido con `micros()`. No se cambió el protocolo serie para enviar pulsos acumulados |
| R4-L7 | Límites de ±1 rad/s en el URDF | ±21 rad/s (0,7 m/s / 0,033 m) |
| R4-L8 | La compensación no hace nada sin RTT | Aviso en el puente; el CSV guarda `rtt_ema_ms` y el informe avisa de cada corrida con más de un 10 % de compensación sin latencia medida |
| R4-L9 | Los dos robots se muestrean distinto en `rmse_real` | El CSV guarda la hora de llegada de la odometría del gemelo (`twin_rx`) y el error real interpola los dos robots (`test_true_error_interpolates_the_twin_by_arrival_time`) |
| R4-L10 | Bloqueo en la cola de la red emulada | Sin cambio en `netem`: la latencia medida y la compensación ven el mismo retardo. El eje de la figura dice "retardo nominal" y la documentación lo explica |
| R4-L11 | El RTT no se reinicia al cambiar la red por parámetro | Se reinicia en los dos caminos, y se descartan los pongs enviados antes del cambio |
| R4-L12 | Los metadatos guardan el líder equivocado | Los metadatos agregan `live` con el líder, las ganancias, el RTT y la diferencia de relojes en uso |
| R4-L13 | El detector usa el modelo nominal | Aviso al arrancar sin `model_file`, que dice cómo pasarle el modelo de la calibración |
| R4-L14 | Gazebo por debajo del tiempo real | El CSV guarda el reloj del gemelo (`twin_clock`); el resumen calcula el factor de tiempo real y el informe avisa si es menor de 0,95 |
| R4-L15 | La calibración se prueba con su propio recorrido | Real: CAL con E1, E2 y E5 (rectas y círculo), evaluación en cuadrados. Simulación: CAL con S5 y S6 (recta y círculo). Una prueba lo exige para los dos protocolos. CI sigue calibrando con M porque MC lo necesita, y es solo una prueba de humo |
| R4-L16 | Las ganancias sintonizadas quedan en los límites | Límites ampliados a 0,05; el resultado avisa si una ganancia queda en un límite; el informe de la sintonización explica qué minimiza. Ver "Punto abierto" abajo |
| R4-L17 | La vista previa de navegación deja su registro abierto | Se cierra al terminar la ejecución en el robot real |
| R4-L18 | El reintento de CI puede analizar el CSV del intento anterior | Se borran las carpetas al empezar |
| R4-L19 | Cinco pruebas fallan sin scipy | `pytest.importorskip("scipy")` en esas pruebas y `test_depend` de scipy |
| R4-L20 | `welch_t` con varianza cero | t con el signo de la diferencia; sin scipy, p = NaN (`test_zero_variance_keeps_the_sign_and_needs_scipy_for_p`) |
| R4-L21 | Un experimento faltante borra su comparación en silencio | Fila "sin datos (no hay corridas de …)" |
| R4-L22 | Documentos que no coinciden | 75 corridas (ahora 77, porque E5 pasó a 5 repeticiones para identificar mejor la velocidad angular); una sola corrida de CI en `GEMELO_DIGITAL.md`, con todas las filas n = 2 como no concluyentes; la frase sobre qué métrica usa cada comparación |

## Punto abierto: la sintonización

Con los límites ampliados, el optimizador eligió ky = 4,1 (antes estaba limitado a 0,5). En la simulación de la sintonización, ky casi no cambia el costo: 11,4 cm de error con ky = 0,5 y 11,5 cm con ky = 4,1, también con más retardo. Con el robot de mentira de CI, en cambio, RT dio 17,0 cm (antes 12,5 cm) y la mejora frente a R ya no es clara (p = 0,31, con n = 2). La simulación no capta algo que sí afecta al robot de mentira con ky alto. No se forzó ky bajo para recuperar el resultado anterior: G0T y L250RT con el robot físico dirán cuánto valen las ganancias sintonizadas, y la tesis lo reporta tal cual.

## Lo que sigue en el robot

1. Paso 5 del README (antes 4): comprobar el sentido de giro (N-F1).
2. Paso 3 del README: relojes con chrony.
3. Toma de datos con `protocolo_real` (77 corridas, unas 2 horas; se puede retomar con `--resume`).
