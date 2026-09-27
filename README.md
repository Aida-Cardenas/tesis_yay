# Gemelo digital de un robot móvil diferencial (WMR) — workspace ROS 2 Humble

Workspace único con todo el software del prototipo: descripción URDF y simulación en Gazebo, control y odometría, fusión sensorial, localización, SLAM, planificación, navegación con Nav2 y la interfaz de hardware (Raspberry Pi + Arduino).

Trabajo Especial de Grado — Ingeniería de Sistemas, Universidad Metropolitana. Autora: Aida Cárdenas.

> **Origen del código.** Este workspace unifica los tres repositorios de Antonio Brandi *Self-Driving and ROS 2 – Learn by Doing* (Odometry & Control, Map & Localization, Plan & Navigation), licenciados bajo Apache 2.0. Ver [`NOTICE`](NOTICE) y [`docs/ORIGEN_DEL_CODIGO.md`](docs/ORIGEN_DEL_CODIGO.md).

---

## Estructura

```
tesis-gemelo-digital-wmr/          <- esta carpeta ES el workspace (colcon build aquí)
├── src/
│   ├── bumperbot_description   URDF/xacro, mallas, mundos de Gazebo, launch de simulación
│   ├── bumperbot_controller    cinemática diferencial, odometría, teleoperación, twist_mux
│   ├── bumperbot_firmware      ros2_control hardware interface (serial), driver IMU MPU6050, sketches Arduino
│   ├── bumperbot_localization  filtro de Kalman propio, EKF (robot_localization), AMCL
│   ├── bumperbot_mapping       mapeo con poses conocidas, slam_toolbox, mapas guardados
│   ├── bumperbot_planning      planificadores globales Dijkstra y A* (nodos + plugins Nav2)
│   ├── bumperbot_motion        controladores PD y Pure Pursuit (nodos + plugins Nav2)
│   ├── bumperbot_navigation    configuración de Nav2 y behavior trees
│   ├── bumperbot_utils         safety_stop (parada de seguridad con LiDAR)
│   ├── bumperbot_bringup       launch principales: simulated_robot / real_robot
│   ├── bumperbot_msgs          servicios y acciones propias
│   ├── bumperbot_cpp_examples  ejemplos de conceptos ROS 2 en C++
│   └── bumperbot_py_examples   ejemplos de conceptos ROS 2 en Python
├── udev/90-bumperbot.rules     nombres fijos /dev/arduino y /dev/rplidar en la Raspberry Pi
├── install_dependencies.sh
└── docs/ORIGEN_DEL_CODIGO.md   qué vino de qué módulo y cómo se relaciona con la tesis
```

## Requisitos

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (`ros-humble-desktop` en la PC, `ros-humble-ros-base` en la Raspberry Pi)
- Gazebo Fortress (se instala solo con las dependencias; en Humble se usa `ign_ros2_control`)

## Instalación

```bash
git clone <URL-de-tu-repo> ~/tesis_ws
cd ~/tesis_ws
./install_dependencies.sh           # en la Raspberry Pi: ./install_dependencies.sh robot
colcon build
source install/setup.bash
```

Recomendado: agregar `source ~/tesis_ws/install/setup.bash` al final de `~/.bashrc`.

## Uso en simulación (gemelo digital en Gazebo)

**Mapeo con SLAM** en la casa:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

**Localización con AMCL + navegación** sobre un mapa ya guardado (el mundo y el mapa deben coincidir):

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house map_name:=small_house
```

En RViz: primero **2D Pose Estimate** (pose inicial para AMCL) y luego **Nav2 Goal** para enviar una meta.

Mundos disponibles: `empty`, `small_house`, `small_warehouse`. Mapas disponibles: `small_house`, `small_warehouse`.

**Solo ver el modelo** (RViz, sin física):

```bash
ros2 launch bumperbot_description display.launch.py
```

### Argumentos de `simulated_robot.launch.py` / `real_robot.launch.py`

| Argumento | Por defecto | Qué hace |
|---|---|---|
| `use_slam` | `false` | `true`: slam_toolbox (mapeo). `false`: map_server + AMCL |
| `world_name` | `empty` | Mundo de Gazebo (solo simulación) |
| `map_name` | `small_house` | Mapa para AMCL, en `bumperbot_mapping/maps/<map_name>/map.yaml` |
| `use_simple_controller` | `False` | `True`: nodo propio de cinemática/odometría (`simple_controller`). `False`: `diff_drive_controller` |
| `use_python` | `False` | Con `use_simple_controller:=True`, usa la versión Python en lugar de C++ |
| `use_safety_stop` | `false` | Lanza `safety_stop` (frena/detiene si el LiDAR ve un obstáculo cerca) |
| `bt_xml` | `simple_navigation_w_replanning_and_recovery.xml` | Behavior tree de Nav2 (ruta completa) |
| `arduino_port` | `/dev/arduino` | Puerto serie del Arduino (solo robot real) |

### Teleoperación

- Joystick: se lanza automáticamente con el bringup.
- Teclado: `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=false`

Prioridades en `twist_mux`: joystick (99) > teclado (90) > Nav2 (80). El lock `safety_stop` (255) bloquea todo.

### Guardar un mapa nuevo

Con el SLAM corriendo, después de recorrer el entorno:

```bash
mkdir -p src/bumperbot_mapping/maps/mi_mapa
ros2 run nav2_map_server map_saver_cli -f src/bumperbot_mapping/maps/mi_mapa/map
colcon build --packages-select bumperbot_mapping
```

Luego se usa con `map_name:=mi_mapa`.

## Uso con el robot real

1. **Firmware.** Cargar `src/bumperbot_firmware/firmware/robot_control/robot_control.ino` en el Arduino con el Arduino IDE. Si los motores o encoders giran al revés, usar `robot_control_inverted/robot_control_inverted.ino`.
2. **Reglas udev** (en la Raspberry Pi, una sola vez). Revisar los IDs con `lsusb` y ajustar el archivo si hace falta:
   ```bash
   sudo cp udev/90-bumperbot.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
   ls -l /dev/arduino /dev/rplidar
   ```
3. **Lanzar** en la Raspberry Pi:
   ```bash
   ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true
   ```
   Sin udev, se puede indicar el puerto a mano: `arduino_port:=/dev/ttyUSB0`.
4. **Visualizar** desde la laptop (misma red y mismo `ROS_DOMAIN_ID` en ambas máquinas):
   ```bash
   ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

## Demos individuales del curso

| Tema | Comando |
|---|---|
| Filtro de Kalman propio + EKF de robot_localization | `ros2 launch bumperbot_localization local_localization.launch.py` |
| Modelo de movimiento de odometría | `ros2 run bumperbot_localization odometry_motion_model` |
| Mapeo con poses conocidas | `ros2 run bumperbot_mapping mapping_with_known_poses` |
| Planificadores Dijkstra / A* (nodos) | `ros2 run bumperbot_planning dijkstra_planner.py` · `a_star_planner.py` |
| Controladores PD / Pure Pursuit (nodos) | `ros2 run bumperbot_motion pd_motion_planner.py` · `pure_pursuit.py` |
| Ejemplos ROS 2 (pub/sub, servicios, acciones, TF, QoS, lifecycle) | `ros2 run bumperbot_cpp_examples <nombre>` · `ros2 run bumperbot_py_examples <nombre>` |

Nav2 usa por defecto `SmacPlanner2D` y `RegulatedPurePursuitController`. Los plugins propios (`bumperbot_planning::DijkstraPlanner`, `AStarPlanner`, `bumperbot_motion::PDMotionPlanner`, `PurePursuit`) están comentados en `bumperbot_navigation/config/planner_server.yaml` y `controller_server.yaml`; se activan descomentándolos.

## Integración continua

`.github/workflows/build.yml` compila el workspace completo en ROS 2 Humble en cada push y verifica que el URDF y todos los launch carguen. Si el check sale en verde en GitHub, compila.
