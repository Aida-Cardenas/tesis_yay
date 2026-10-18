# Respuesta a la revisión de código (10 de octubre de 2026)

Qué se hizo con cada observación de la revisión, dónde está el cambio y cómo se comprobó. Todas las pruebas automáticas corren en ROS 2 Humble en GitHub Actions (`.github/workflows/build.yml`), incluidas las de punta a punta que en la máquina de revisión fallaban por tener Jazzy.

## Críticas: robot real

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| 1 | Ruedas izquierda y derecha invertidas | Corregido | `bumperbot_interface.cpp` busca cada rueda por el nombre del joint (`find_wheel_index`), no por su posición | gtest `test_serial_protocol`; comprobación de giro en la guía del robot |
| 2 | Giroscopio 8 veces más lento | Corregido | `mpu6050_math.py`: 939,65 LSB/(rad/s) para ±2000 °/s, rango del acelerómetro explícito, covarianzas reales, `>= 32768`. Además, se mide y resta el cero del giroscopio al arrancar | `test_imu_scaling.py` |
| 3 | Una línea serie corrupta cierra el controlador | Corregido | Parser aparte (`serial_protocol.cpp`) que descarta datos malos y avisa | gtest con tokens vacíos, cortos y no numéricos |

## Altas: navegación y lanzamiento

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| 4 | El costmap local no se carga | No aplicaba | El bloque `local_costmap` (con `global_frame: odom`, `rolling_window: true` y el LiDAR como fuente) está en `controller_server.yaml`, que sí se carga. `costmap.yaml` era un sobrante sin uso y se eliminó | Prueba de navegación con Gazebo y Nav2 en CI |
| 5 | Umbrales del map_saver ignorados | Corregido | `slam.launch.py` usa un diccionario | CI carga todos los launch |
| 6 | Mundo y mapa no coinciden por defecto | Corregido | Todos los launch usan `arena` por defecto; los mundos de demostración se eliminaron | CI |
| 7 | Dos `--param-file` en el spawner de Humble | Corregido | `controller.launch.py` fusiona EKF y calibración en un solo archivo | `ci/test_mock_stack.sh` lanza con calibración y comprueba `enable_odom_tf: false` y el multiplicador |

## Altas: resultados del gemelo

| # | Observación | Estado | Cambio | Comprobación |
|---|---|---|---|---|
| 8 | La calibración sin datos instala un modelo | Corregido | `experiment_runner.py` se detiene con error, no guarda ni carga el modelo y omite los experimentos que lo necesitan | `test_runner_logic.py` |
| 9 | La figura error vs latencia mezcla experimentos | Corregido | `report.py`: solo experimentos que difieren en el retardo (mismo líder, con realimentación, ganancias por defecto, sin fallas) y una sola métrica | `test_report.py` |
| 10 | El LiDAR se compara sin mirar el rumbo | Corregido | `twin_bridge.py`: también exige error de rumbo menor que `scan_heading_tolerance` (0,035 rad) | CI de sincronización con LiDAR |

## Medias

| Observación | Estado | Cambio |
|---|---|---|
| `lock_on_stall` se activa y desactiva durante un atasco | Corregido | `StallLock` en `anomaly.py`: mantiene el bloqueo, congela el detector y reintenta (`test_anomaly.py`). El bloqueo usa ahora su propio lock, `anomaly_stop`, para no liberar el de `safety_stop` |
| Carreras en el panel | Corregido | `TrajectoryPlayer.play` detiene el hilo anterior; `nav_busy` se marca antes de lanzar el hilo; el estado no pisa una casilla recién cambiada (`test_dashboard.py`) |
| Reglas udev | Corregido | Reglas por `KERNEL` y sin la línea que cambiaba todos los `tty` |
| `rosdep install -r` en CI | Corregido | Sin `-r`; además CI corre `colcon test` del firmware y compila el sketch para Uno y Nano |
| `safety_stop` baja la velocidad repetidamente | Corregido | Baja una vez por acercamiento y la sube al quedar libre |

## Bajas

| Observación | Estado | Cambio |
|---|---|---|
| Tabla de comparaciones siempre en cm | Corregido | Unidad y escala según la métrica |
| `final_error` NaN en el JSON | Corregido | `json_safe` (`test_nav_preview_json.py`) |
| `fake_navigator` no interrumpe metas concurrentes | Sin cambio | `twin_navigate` envía una meta a la vez por diseño; el navegador de mentira solo se usa en pruebas |
| `arena:=[0.0]` por defecto | Corregido | El valor por defecto es el recinto real |
| Tasa en 0 y registros que se pisan | Corregido | Validación de tasas y sufijo `-N` si el archivo existe |
| Ruta `/home/alien/...` | Corregido | Eliminada |
| `gz_frame_id` / `GZ_SIM_RESOURCE_PATH` en Fortress | Comprobado | Se agregó `IGN_GAZEBO_RESOURCE_PATH`; el registro de CI muestra el LiDAR de Gazebo en `laser_link` |

## Complementos del curso sin usar

Se eliminaron en lugar de documentarlos: `bumperbot_planning` (Dijkstra, A\*), `bumperbot_motion` (PD, Pure Pursuit), `simple_controller`, `robot_control_inverted.ino`, los mundos y mapas de demostración y otras demos. Nav2 usa SmacPlanner2D y RegulatedPurePursuit. Detalle en [`ORIGEN_DEL_CODIGO.md`](ORIGEN_DEL_CODIGO.md).

## Hallazgos adicionales después de la revisión

| Problema | Cambio |
|---|---|
| Si el software del robot se caía o se cerraba con el robot en movimiento, el Arduino seguía con la última velocidad | El firmware detiene las ruedas tras 500 ms sin comandos; la interfaz manda velocidad cero al cerrarse |
| Contadores de los encoders leídos sin proteger de las interrupciones, y `String` dentro de ellas | Lectura con interrupciones desactivadas y signos como `char` |
| El giroscopio no restaba su desviación del cero | Se mide durante 2 s al arrancar, con el robot quieto |
| El driver de la IMU se cerraba si la IMU no respondía al arrancar | Reintenta sin cerrarse |

Las pruebas con el robot físico hechas antes de estas correcciones deben repetirse (ruedas invertidas y escala de la IMU).
