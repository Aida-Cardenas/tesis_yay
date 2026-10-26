# Origen del código y relación con la tesis

## 1. Cómo se unificaron los tres repositorios

Los tres cursos construyen **el mismo workspace** (`bumperbot_ws`) de forma incremental: cada módulo arranca con el código final del anterior y le agrega paquetes. Por eso, el workspace final de **Plan & Navigation** (rama `humble`, `Section9_Build_the_Robot`) ya contiene todos los archivos que aparecen en cualquier sección de los tres repos, con una sola excepción: `robot_control_inverted.ino`, que se copió de los otros dos (y después se eliminó, ver la limpieza más abajo).

Se verificó comparando cada archivo de cada sección de los tres repos contra el workspace final. Los archivos que existen en varias versiones quedaron en su versión más reciente (la de Plan & Navigation), salvo los ajustes listados en [`NOTICE`](../NOTICE).

| Repo | Rama | Sección usada | Aporte al workspace unificado |
|---|---|---|---|
| Plan-Navigation | `humble` | Section9_Build_the_Robot | Base completa (13 paquetes) |
| Odometry-Control | `main` | Section12_Build_the_Robot | `robot_control_inverted.ino` |
| Map-Localization | `main` | Section11_Build_the_Robot | `robot_control_inverted.ino`, `safety_stop` en el bringup, puerto `/dev/arduino`, `python3-smbus` |

Las ramas `main` de Odometry-Control y Map-Localization ya usan el Gazebo nuevo (Fortress en Humble), igual que la rama `humble` de Plan-Navigation, así que no hubo conflicto de simulador. Las ramas `gz-classic` (Gazebo 11) no se usaron.

### Limpieza: solo lo esencial

El curso enseña cada concepto con una demo aislada y casi todo lo escribe dos veces (C++ y Python). Para la tesis se dejó **solo lo que usa el sistema completo** (el robot real, el gemelo en Gazebo y la navegación). Se quitaron:

| Quitado | Por qué |
|---|---|
| `bumperbot_cpp_examples`, `bumperbot_py_examples` | Demos de conceptos de ROS 2 (publicador, suscriptor, parámetros, TF, QoS, lifecycle…) que el sistema no usa |
| Mensajes `Fibonacci`, `AddTwoInts`, `GetTransform` | Solo los usaban esas demos |
| Versiones Python de `simple_controller`, `noisy_controller`, `kalman_filter`, `imu_republisher`, `odometry_motion_model`, `mapping_with_known_poses`, planificadores y controladores | Duplicados de la versión C++ o demos que reemplazan robot_localization, slam_toolbox y Nav2 |
| `noisy_controller`, `kalman_filter`, `local_localization.launch.py`, `odometry_motion_model`, `mapping_with_known_poses` (C++) | Demos de teoría reemplazadas por los paquetes estándar que sí se usan (EKF de robot_localization, slam_toolbox, AMCL) |
| Planificadores Dijkstra y A* y controladores PD y Pure Pursuit (paquetes `bumperbot_planning` y `bumperbot_motion`) | Nav2 usa SmacPlanner2D y RegulatedPurePursuit; los del curso no se usaban y además tenían errores (rutas incorrectas cuando la meta es inalcanzable, velocidad angular mal calculada, caídas por excepciones de TF) |
| `simple_controller` (C++) | Cinemática y odometría propias del curso; el sistema usa `diff_drive_controller` de ros2_controllers |
| `robot_control_inverted.ino` | Invertía lados y sentidos; con la interfaz que asigna las ruedas por nombre ya no hace falta |
| Mundos `empty`, `small_house` y `small_warehouse`, sus mapas, modelos 3D y fotos | Demos del curso; las pruebas de la tesis usan solo el recinto `arena` |
| `display.launch.py`, configuraciones de RViz sueltas y dos behavior trees sin usar | Demos del curso; el sistema usa la vista de Nav2 y un solo behavior tree |
| Sketches de prueba del Arduino y `simple_serial_*` | Pruebas de la conexión serie; el firmware real es `robot_control` |

Lo que queda es lo que se ejecuta: el modelo URDF y el recinto `arena`, `diff_drive_controller` con `twist_mux` y `twist_relay` (joystick), la interfaz de hardware y el firmware `robot_control.ino`, el driver de la IMU, el EKF, AMCL y slam_toolbox, Nav2 con su configuración, `safety_stop`, el gemelo digital y sus mensajes.

## 2. Qué archivo nació en qué sección del curso

Algunos de estos archivos se quitaron en la limpieza (ver arriba); la tabla se deja para ubicar cada tema en el curso.

### Módulo 1 — Odometry & Control

| Sección | Archivos principales | Paquete |
|---|---|---|
| 3 Introducción a ROS 2 | `simple_publisher`, `simple_subscriber` | `bumperbot_cpp_examples`, `bumperbot_py_examples` |
| 4 Locomoción | `bumperbot.urdf.xacro`, `bumperbot_gazebo.xacro`, `display.launch.py`, `gazebo.launch.py`, `simple_parameter` | `bumperbot_description` |
| 5 Control | `bumperbot_ros2_control.xacro`, `bumperbot_controllers.yaml`, `controller.launch.py` | `bumperbot_description`, `bumperbot_controller` |
| 6 Cinemática | `simple_turtlesim_kinematics` | ejemplos |
| 7 Cinemática diferencial | `simple_controller` (C++/Py), `joystick_teleop.launch.py`, `joy_*.yaml` | `bumperbot_controller` |
| 8 TF2 | `simple_tf_kinematics`, servicios `AddTwoInts`, `GetTransform` | ejemplos, `bumperbot_msgs` |
| 9 Odometría | odometría y TF `odom → base_footprint` dentro de `simple_controller` | `bumperbot_controller` |
| 10 Probabilidad | `noisy_controller` (odometría con ruido) | `bumperbot_controller` |
| 11 Fusión sensorial | `kalman_filter` (C++/Py), `imu_republisher`, `ekf.yaml`, `local_localization.launch.py` | `bumperbot_localization` |
| 12 Construir el robot | `bumperbot_interface` (hardware ros2_control), `mpu6050_driver.py`, sketches Arduino, `real_robot.launch.py`, `simulated_robot.launch.py` | `bumperbot_firmware`, `bumperbot_bringup` |

### Módulo 2 — Map & Localization

| Sección | Archivos principales | Paquete |
|---|---|---|
| 5 Localización global | `odometry_motion_model`, mundos `small_house`, `small_warehouse` | `bumperbot_localization`, `bumperbot_description` |
| 6 Sensores | `safety_stop`, `twist_relay`, `twist_mux_*.yaml`, acción `Fibonacci` | `bumperbot_utils`, `bumperbot_controller` |
| 7 Representación de mapas | mapas `small_house`, `small_warehouse`, `global_localization.launch.py`, QoS | `bumperbot_mapping`, `bumperbot_localization` |
| 8 Mapeo con poses conocidas | `mapping_with_known_poses` (C++/Py) | `bumperbot_mapping` |
| 9 Localización con mapa conocido | `amcl.yaml`, `global_localization.rviz` | `bumperbot_localization` |
| 10 SLAM | `slam_toolbox.yaml`, `slam.launch.py`, `slam.rviz` | `bumperbot_mapping` |
| 11 Construir el robot | `rplidar_a1.yaml`, LiDAR en bringup real | `bumperbot_bringup` |

### Módulo 3 — Plan & Navigation

| Sección | Archivos principales | Paquete |
|---|---|---|
| 4 Planificación de rutas | `dijkstra_planner`, `a_star_planner` (C++/Py) | `bumperbot_planning` |
| 5 Planificación de movimiento | `pd_motion_planner`, `pure_pursuit` (C++/Py) | `bumperbot_motion` |
| 6 Evasión de obstáculos | costmaps (el bloque `local_costmap` de `controller_server.yaml` y `global_costmap` de `planner_server.yaml`; el `costmap.yaml` de la demo se quitó) | `bumperbot_navigation` |
| 7 Navegación | `navigation.launch.py`, `planner_server.yaml`, `controller_server.yaml`, `smoother_server.yaml`, plugins Nav2 | `bumperbot_navigation` |
| 8 Toma de decisiones | behavior trees (`simple_navigation*.xml`), `bt_navigator.yaml`, `behavior_server.yaml` | `bumperbot_navigation` |
| 9 Construir el robot | Nav2 en ambos bringup | `bumperbot_bringup` |

## 3. Relación con las secciones de la tesis

| Sección de la tesis | Dónde está en el código | Estado |
|---|---|---|
| III.3.1 Modelo virtual (URDF/Xacro) | `bumperbot_description/urdf/*.xacro`, `meshes/` | Listo: las mallas y medidas corresponden al diseño del prototipo en Fusion 360 |
| III.3.2 Entorno en Gazebo | `bumperbot_description/worlds/`, `launch/gazebo.launch.py` | Listo |
| III.3.3 Control y odometría | `bumperbot_controller` (+ `bumperbot_firmware` en el robot real) | Listo |
| III.3.4 Comunicación bidireccional | `bumperbot_digital_twin` | Aporte propio (ver [`GEMELO_DIGITAL.md`](GEMELO_DIGITAL.md)) |
| III.1.1 Fusión sensorial (EKF) | `bumperbot_localization/config/ekf_odom.yaml` | Integrado al sistema (ver punto 2) |
| III.1.1 SLAM | `bumperbot_mapping` (`slam_toolbox`) | Listo |
| III.1.1 Nav2 | `bumperbot_navigation` | Listo |
| Prueba 1 — URDF en Gazebo | `ros2 launch bumperbot_description gazebo.launch.py` | — |
| Prueba 2 — Árbol TF | con el bringup corriendo: `ros2 run tf2_tools view_frames` | — |
| Prueba 3 — Movimiento por comando | `ros2 topic pub -r 10 /key_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` y `ros2 topic echo /bumperbot_controller/odom` | — |
| Prueba 4 — Encoders | `ros2 topic echo /joint_states` girando las ruedas a mano | — |
| Prueba 5 — Sincronización | `ros2 run bumperbot_digital_twin twin_experiment protocolo_real` | Protocolo automático con informe |

## 4. Diferencias entre el texto actual de la tesis y el código

1. **Comunicación bidireccional (III.3.4, Prueba 5).** El curso lanza el robot real **o** el simulado. La capa que los conecta y sincroniza es el aporte propio de la tesis: paquete `bumperbot_digital_twin`, descrito en [`GEMELO_DIGITAL.md`](GEMELO_DIGITAL.md).
2. **EKF.** En el curso el EKF era una demo aparte. Ahora está integrado al sistema completo y activo por defecto (`use_ekf:=true`): publica `/odometry/filtered` (no `/odom_filtered`) y la TF `odom → base_footprint` que usan SLAM, AMCL y Nav2.
3. **"Nodos propios en C++ para cinemática y odometría".** El sistema usa `diff_drive_controller` de ros2_controllers, que calcula la cinemática y la odometría a partir de los encoders. El nodo propio del curso (`simple_controller`) se eliminó porque no se usaba. El texto de III.1.1 debe decir que se usó `diff_drive_controller` configurado con las medidas del prototipo.
4. **Ruedas.** El modelo tiene 2 ruedas motrices + 2 ruedas locas (`caster_front_link`, `caster_rear_link`), es decir, tracción diferencial clásica.
5. **Medidas del robot.** Radio de rueda 0,033 m y separación 0,17 m están en `bumperbot_controllers.yaml` y en el URDF (`bumperbot.urdf.xacro`: posición de las ruedas y radio de su colisión). Las diferencias medidas en el robot real se corrigen sin tocar el URDF con `twin_calibrate wheels` (multiplicadores de `diff_drive_controller`). Coinciden con el diseño del prototipo; si alguna cambia, hay que actualizarla en los dos lugares.
6. **Simulador.** El capítulo III.1 menciona `gazebo_ros_pkgs`, que es del Gazebo viejo (Classic). El sistema usa Gazebo Fortress con `ros_gz`.
7. **Limitaciones (V.1).** El primer punto dice que no se abordó la navegación autónoma, pero el código sí incluye Nav2, planificación y evasión de obstáculos.

## 5. Errores del código del curso que se corrigieron (revisiones de octubre 2026)

| Problema | Dónde | Corrección |
|---|---|---|
| Ruedas izquierda y derecha invertidas: la interfaz suponía que el joint 0 era la rueda derecha, pero el URDF declara primero `wheel_left_joint` | `bumperbot_interface.cpp` | Las ruedas se buscan por nombre del joint (`wheel_right_joint`, `wheel_left_joint`) |
| Una línea corrupta del Arduino (`r`, `rp`, `rpabc`) lanzaba una excepción y cerraba `ros2_control_node` | `bumperbot_interface.cpp` | Parser aparte (`serial_protocol.cpp`) que descarta los datos malos y avisa; con prueba gtest |
| Giroscopio 8 veces más lento: se configuraba ±2000 °/s pero se dividía por el factor de ±250 °/s | `mpu6050_driver.py` | Factor 939,65 LSB/(rad/s), rango del acelerómetro explícito y covarianzas reales para el EKF |
| Umbrales del map_saver escritos como conjunto de Python y no como diccionario (se ignoraban) | `slam.launch.py` | `{"free_thresh_default": 0.196, "occupied_thresh_default": 0.65}` |
| `simulated_robot.launch.py` abría un mundo vacío y localizaba en otro mapa | `simulated_robot.launch.py` | Argumentos `world_name` y `map_name` con valores coherentes |
| Con EKF y calibración, el spawner de Humble recibía dos `--param-file` | `controller.launch.py` | Se fusionan en un solo archivo |
| `safety_stop` bajaba la velocidad del joystick cada vez que salía de la zona de peligro | `safety_stop.cpp` | Baja una sola vez por acercamiento y la sube al quedar libre |
| La regla udev del Arduino no filtraba por `ttyUSB*` y otra línea cambiaba los permisos de todas las terminales | `udev/90-bumperbot.rules` | Reglas por `KERNEL` y sin la línea global |
| Si el programa del robot se caía o se cerraba con el robot en movimiento, el Arduino seguía con la última velocidad indefinidamente | `robot_control.ino`, `bumperbot_interface.cpp` | El Arduino detiene las ruedas si pasa medio segundo sin recibir comandos, y la interfaz manda velocidad cero al cerrarse |
| Los contadores de los encoders se leían mientras las interrupciones podían modificarlos, y las interrupciones creaban objetos `String` (memoria dinámica) | `robot_control.ino` | Lectura con las interrupciones desactivadas y signos como `char` |
| El giroscopio se publicaba sin restar su desviación del cero (el MPU6050 marca algunos °/s estando quieto) | `mpu6050_driver.py` | Mide el cero durante 2 s al arrancar, con el robot quieto, y lo resta |
| Si la IMU no respondía al arrancar, el driver se cerraba con un error | `mpu6050_driver.py` | Reintenta en silencio hasta que la IMU responde |
| La IMU se leía byte por byte, en transacciones I2C separadas, mientras el sensor actualiza a 1 kHz: un valor cerca de cero podía saltar ±256 LSB (unos ±0,27 rad/s) | `mpu6050_driver.py` | Una sola lectura de 14 bytes; filtro interno a 44 Hz en lugar de 260 Hz |
| La interfaz integraba la posición de las ruedas con el reloj del sistema y `ReadLine` podía bloquear el lazo de control con una línea a medias | `bumperbot_interface.cpp` | Integra con el periodo de ros2_control y lee solo los bytes ya recibidos |
| Faltaban dependencias en `package.xml` (solo se notaba al compilar paquetes sueltos) | `bumperbot_description`, `bumperbot_firmware` | Agregadas |
| El bloqueo por atasco del gemelo usaba el mismo lock de `twist_mux` que `safety_stop`, así que uno podía liberar al otro | `anomaly_detector.py`, `twist_mux_locks.yaml` | Lock propio, `anomaly_stop` |
| `safety_stop` tomaba como obstáculo cualquier lectura finita, también las menores que `range_min` (el LiDAR viéndose a sí mismo), y podía dejar el robot detenido para siempre | `safety_stop.cpp` | Solo cuenta lecturas dentro de `[range_min, range_max]` |
| La posición de las ruedas se obtenía multiplicando la velocidad que manda el Arduino cada 100 ms por el periodo de la Pi; como esos intervalos no coinciden, la odometría acumulaba un error que no venía de las ruedas, y una línea perdida era distancia perdida | `robot_control.ino`, `serial_protocol.cpp` | El Arduino manda además el contador de cada encoder (`rc`, `lc`) y la posición sale de él. Cuenta los dos flancos de la fase A: 770 cuentas por vuelta en lugar de 385 |
| Un comando no finito (NaN) se enviaba como `nan`, y el sketch toma la `n` como "invertir sentido" | `serial_protocol.cpp` | Se envía 0; la magnitud se limita a 99,99 para que siempre quepa en los 5 caracteres que lee el sketch |

`robot_control_inverted.ino` se eliminó: además de cambiar los lados invertía el sentido de motores y encoders, y con la interfaz corregida ya no hace falta. El único firmware es `robot_control.ino`. Si el robot estaba cableado para el sketch invertido, con el actual puede andar hacia atrás aunque la odometría parezca correcta; el README explica cómo comprobarlo (paso 5 de "Uso con el robot real").

Los planificadores y controladores del curso (`DijkstraPlanner`, `AStarPlanner`, `PDMotionPlanner`, `PurePursuit`) tenían errores conocidos, pero no se corrigieron porque Nav2 nunca los usaba: se eliminaron del repositorio junto con el resto del código que el sistema no ejecuta (sección 1).
