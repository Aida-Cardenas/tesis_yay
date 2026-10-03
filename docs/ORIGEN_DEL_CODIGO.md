# Origen del código y relación con la tesis

## 1. Cómo se unificaron los tres repositorios

Los tres cursos construyen **el mismo workspace** (`bumperbot_ws`) de forma incremental: cada módulo arranca con el código final del anterior y le agrega paquetes. Por eso, el workspace final de **Plan & Navigation** (rama `humble`, `Section9_Build_the_Robot`) ya contiene todos los archivos que aparecen en cualquier sección de los tres repos, con una sola excepción: `robot_control_inverted.ino`, que se copió de los otros dos.

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
| Nodos sueltos de Dijkstra, A*, PD y Pure Pursuit | Se conservan como **plugins de Nav2** (misma lógica) |
| Sketches de prueba del Arduino y `simple_serial_*` | Pruebas de la conexión serie; el firmware real es `robot_control` |

Lo que queda: `simple_controller` y `twist_relay` (C++), la interfaz de hardware y el driver de la IMU, los plugins de Nav2 y todas las configuraciones y launch del sistema.

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
| 6 Evasión de obstáculos | `costmap.yaml` | `bumperbot_navigation` |
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
| III.1.1 Nav2 | `bumperbot_navigation`, `bumperbot_planning`, `bumperbot_motion` | Listo |
| Prueba 1 — URDF en Gazebo | `ros2 launch bumperbot_description gazebo.launch.py` | — |
| Prueba 2 — Árbol TF | con el bringup corriendo: `ros2 run tf2_tools view_frames` | — |
| Prueba 3 — Movimiento por comando | `ros2 topic pub -r 10 /key_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` y `ros2 topic echo /bumperbot_controller/odom` | — |
| Prueba 4 — Encoders | `ros2 topic echo /joint_states` girando las ruedas a mano | — |
| Prueba 5 — Sincronización | `ros2 run bumperbot_digital_twin twin_experiment protocolo_real` | Protocolo automático con informe |

## 4. Diferencias entre el texto actual de la tesis y el código

1. **Comunicación bidireccional (III.3.4, Prueba 5).** El curso lanza el robot real **o** el simulado. La capa que los conecta y sincroniza es el aporte propio de la tesis: paquete `bumperbot_digital_twin`, descrito en [`GEMELO_DIGITAL.md`](GEMELO_DIGITAL.md).
2. **EKF.** En el curso el EKF era una demo aparte. Ahora está integrado al sistema completo y activo por defecto (`use_ekf:=true`): publica `/odometry/filtered` (no `/odom_filtered`) y la TF `odom → base_footprint` que usan SLAM, AMCL y Nav2.
3. **"Nodos propios en C++ para cinemática y odometría".** Por defecto se usa `diff_drive_controller` de ros2_controllers. El nodo propio (C++) se usa con `use_simple_controller:=True`; en ese modo el EKF no se lanza porque esos nodos publican su propia TF.
4. **Ruedas.** El modelo tiene 2 ruedas motrices + 2 ruedas locas (`caster_front_link`, `caster_rear_link`), es decir, tracción diferencial clásica.
5. **Medidas del robot.** Radio de rueda 0,033 m y separación 0,17 m están en `bumperbot_controllers.yaml`, `controller.launch.py` y los `declare_parameter` de `simple_controller`. Las diferencias medidas en el robot real se corrigen sin tocar el URDF con `twin_calibrate wheels` (multiplicadores de `diff_drive_controller`). Coinciden con el diseño del prototipo; si alguna cambia, hay que actualizarla en todos esos lugares y en el URDF.
6. **Simulador.** El capítulo III.1 menciona `gazebo_ros_pkgs`, que es del Gazebo viejo (Classic). El sistema usa Gazebo Fortress con `ros_gz`.
7. **Limitaciones (V.1).** El primer punto dice que no se abordó la navegación autónoma, pero el código sí incluye Nav2, planificación y evasión de obstáculos.
