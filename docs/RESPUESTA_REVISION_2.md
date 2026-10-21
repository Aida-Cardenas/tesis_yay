# Respuesta a la segunda revisión de código

Qué se hizo con cada punto de la segunda revisión, dónde está el cambio y cómo se comprueba. Las pruebas corren en ROS 2 Humble en GitHub Actions (`.github/workflows/build.yml`). La respuesta a la primera revisión está en [`RESPUESTA_REVISION.md`](RESPUESTA_REVISION.md).

## Altas

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| N-T1 | Las comparaciones de latencia y compensación usan el error compensado | Corregido | Los tres protocolos comparan con `rmse_real`. La figura de error contra latencia usa siempre `rmse_real` y solo experimentos del protocolo; sin protocolo no se dibuja y el informe lo dice. `welch_t` descarta corridas NaN | `test_report.py`: los YAML del repositorio, registros que no son del protocolo, caso sin protocolo, NaN. La integración en CI exige `rmse_real(DC) < rmse_real(D)` |
| N-T2 | En modo interactivo el registro incluye el reposicionamiento del robot | Corregido | Nuevo campo `close_log` en `TwinConfigure`. El runner cierra el registro apenas termina el tiempo de asentamiento de cada corrida, antes de pedir que se coloque el robot | `test_runner_logic.py` comprueba que la última llamada de cada corrida es `close_log` |
| N-F1 | El robot puede andar hacia atrás con la odometría correcta | Comprobación agregada; falta hacerla en el robot | README, paso 4 de "Uso con el robot real": `rp03.00,lp03.00,` por el monitor serie debe hacerlo avanzar; un `angular.z` positivo debe girarlo en sentido antihorario con el rumbo de la odometría y `angular_velocity.z` de la IMU positivos | Se hace a mano en el robot, antes de las demás pruebas físicas |

## Medias

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| N-C1 | El filtro aplica el retardo con una muestra de menos | Corregido | `FirstOrderFilter.update` integra con la entrada vigente en (t_prev, t] y recién después guarda la nueva; además integra por tramos, así que aplica L exacto aunque no sea múltiplo del paso | `test_dynamics.py`: el ajuste recupera exactamente (1e-6) retardos de 0 a 300 ms, también fraccionarios (20, 70, 130 ms), y el filtro coincide con el modelo discreto |
| N-C1 (CI) | "Explica por qué CI identifica L = 0,05 s" | Causa distinta | CI no identifica el filtro sino al robot de prueba `fake_robot`. Después de corregir el filtro, CI siguió dando 0,05 s. Había dos causas: `fake_robot` tenía el mismo error de orden (avanzaba con el comando recién sacado de la cola), y el ajuste solo probaba retardos múltiplos del periodo del registro (50 ms). Se corrigió `fake_robot` y el ajuste ahora identifica retardos fraccionarios (`y[k+1] = a·y[k] + b1·u[k−d] + b2·u[k−d−1]`, discretización exacta con L = d·dt + ε) | `test_fit_recovers_a_delay_between_samples_from_a_faster_robot`. En CI: L = {{L_CI}} s con 0,1 s configurados. Lo que falta para 0,1 s se explica porque el registro ve cada cambio de comando hasta 50 ms después de que ocurrió |
| N-L1 | El CI del robot con mock no prueba que Nav2 arranque | Corregido | `grep -qx 'active \[3\]'` en map_server, amcl, planner, controller, smoother, behavior y bt_navigator; `grep -w active` en los controladores; un `map → odom` fijo reemplaza a AMCL, que sin LiDAR no lo publica | `ci/test_mock_stack.sh` |
| N-L2 | La única prueba real de Nav2 no bloquea el CI | Corregido | El paso de Gazebo y Nav2 ya no tiene `continue-on-error`; se repite una vez si falla (Gazebo sin pantalla a veces no arranca) y además exige que el LiDAR publique en `laser_link`. Se agregó `<ignition_frame_id>` junto a `<gz_frame_id>` para Fortress | `ci/test_gazebo_twin.sh` |
| N-F2 | Lecturas de 16 bits de la IMU partidas | Corregido | Una sola lectura de 14 bytes desde `0x3B` (`parse_burst` en `mpu6050_math.py`) | `test_imu_scaling.py` |
| N-F3 | Un giro constante al arrancar se toma como cero del giroscopio | Corregido | El driver escucha `/joint_states` y reinicia la medición mientras alguna rueda supera 0,05 rad/s (y 0,3 s después). Además avisa si el cero medido supera 0,35 rad/s, que no es plausible para un MPU6050 quieto | `test_imu_scaling.py` |
| N-T3 | El R² informado es el de un paso | Corregido | `model.r2` es ahora el de la respuesta simulada (`simulated_r2`); el de un paso queda como `r2_step`. Las etiquetas dicen "R² (respuesta simulada)" | `test_dynamics.py`: el de un paso supera al simulado en un modelo con error. En CI: R² simulado {{R2_CI}} |

## Bajas

| Observación | Estado | Cambio |
|---|---|---|
| La calibración del giroscopio se reinicia sin fin y llena el registro de avisos | Corregido | El aviso sale como mucho cada 5 s y dice cuántos intentos van |
| Una IMU perdida a mitad de la corrida no se vuelve a avisar | Corregido | Aviso al perderla; el driver sigue reintentando |
| `read()` integra con el reloj del sistema | Corregido | Integra con `period` de ros2_control |
| `ReadLine` sin tiempo de espera | Corregido | Se leen solo los bytes ya recibidos a un búfer y se procesa la última línea completa (`take_last_line`, con gtest); una línea a medias espera al ciclo siguiente |
| Filtro del giroscopio en 260 Hz con muestreo a 100 Hz | Corregido | `CONFIG = 3` (unos 44 Hz) |
| El parser acepta `rp-5.0` o `rp0x1A` | Corregido | Solo dígitos y punto (gtest) |
| El bloqueo por atasco no se libera si la odometría deja de llegar | Corregido | El temporizador del bloqueo corre antes de revisar la odometría. Si el detector se cae con el bloqueo puesto, el robot queda bloqueado (falla segura); documentado en `GEMELO_DIGITAL.md` |
| La pérdida de paquetes emulada puede perder un evento "fin" | Documentado | Solo afecta lo que muestra el panel |
| `pause_sync` no manda velocidad cero | Corregido | Manda `Twist()` a los dos seguidores |
| Un registro duplicado sin etiqueta se lee como experimento "2" | Corregido | Los duplicados se nombran `-2`, `-3`… y no se toman como experimento |
| `welch_t` con NaN | Corregido | Ver N-T1 |
| `finish()` deja las ganancias sintonizadas | Corregido | Restaura ganancias, red, banderas y sincronización, y cierra el registro |
| `feedforward_source: odom` hace que la calibración devuelva K ≈ 1, τ ≈ 0 | Corregido | El puente avisa al arrancar y `twin_calibrate dynamics` se niega a ajustar con esos registros |
| Un `configure` que tarda aborta sin informe | Corregido | Ante cualquier error, `twin_experiment` explica el motivo, restaura el puente, genera el informe con las corridas completas y termina con código 1 |
| `bt_xml` y `gui` no declarados | Corregido | Declarados en los launch del bringup y pasados explícitamente |
| Dependencias faltantes en `package.xml` | Corregido | `bumperbot_description` y `bumperbot_firmware` |
| `use_sim_time` declarado y sin uso en `controller.launch.py` | Corregido | Eliminado; el archivo de parámetros fusionado va a una ruta fija en el directorio temporal, así no se acumulan |
| `twin_calibrate arena` no actualiza `metas_arena.yaml` ni el valor por defecto | Documentado | README, sección del mundo |
| udev: LiDAR con `0777` | Corregido | `0666` y grupo `dialout`; `install_dependencies.sh robot` agrega al usuario a `dialout` e `i2c`. Nota sobre `ATTRS{serial}` para distinguir el CH340 |
| `rosdep -r` | Corregido | Eliminado de `install_dependencies.sh` |
| E7 con `--only` cuando no se calibró en la sesión | Corregido | Se omite con el motivo ("corre también el paso CAL") |

## Pruebas que faltaban (sección 3 de la revisión)

| Afirmación | Prueba nueva |
|---|---|
| #10, comparación de LiDAR condicionada al rumbo | `scan_gate` separado en `scan_compare.py`; `test_scan_compare.py` comprueba el rechazo por rumbo, posición y movimiento |
| `StallLock` como lo usa el nodo | La prueba sigue la misma secuencia que `anomaly_detector` en el tick de desbloqueo |
| Panel | Segunda navegación rechazada mientras corre la primera y `nav_busy` liberado al terminar; dos Start seguidos dejan un solo recorrido publicando |
| #9, informe | Protocolos del repositorio, registros ajenos al protocolo y caso sin protocolo |
| #8, runner | `--only E7` sin calibración en la sesión; cierre del registro en cada corrida |
| IMU | Lectura en ráfaga, giro durante la calibración, cero implausible |

## Resultados de CI con las correcciones

{{RESULTADOS}}

Los números de `GEMELO_DIGITAL.md` y del borrador de la tesis se actualizaron con esta corrida.
