# Gemelo digital de un robot móvil diferencial (WMR) — workspace ROS 2 Humble

Workspace único con todo el software del prototipo: descripción URDF y simulación en Gazebo, control y odometría, fusión sensorial (EKF), localización, SLAM, planificación, navegación con Nav2, la interfaz de hardware (Raspberry Pi + Arduino) y la **capa de comunicación bidireccional con el gemelo digital** ([`docs/GEMELO_DIGITAL.md`](docs/GEMELO_DIGITAL.md)).

Trabajo Especial de Grado — Ingeniería de Sistemas, Universidad Metropolitana. Autora: Aida Cárdenas.

> **Origen del código.** Este workspace unifica los tres repositorios de Antonio Brandi *Self-Driving and ROS 2 – Learn by Doing* (Odometry & Control, Map & Localization, Plan & Navigation), licenciados bajo Apache 2.0. Ver [`NOTICE`](NOTICE) y [`docs/ORIGEN_DEL_CODIGO.md`](docs/ORIGEN_DEL_CODIGO.md).

---

## Estructura

```
tesis-gemelo-digital-wmr/          <- esta carpeta ES el workspace (colcon build aquí)
├── src/
│   ├── bumperbot_digital_twin  GEMELO DIGITAL (propio): puente bidireccional, calibración, anomalías,
│   │                           red degradada, compensación de latencia, LiDAR, panel, experimentos
│   ├── bumperbot_description   URDF/xacro, mallas, mundos de Gazebo, launch de simulación
│   ├── bumperbot_controller    diff_drive_controller, cinemática propia (simple_controller), teleoperación, twist_mux
│   ├── bumperbot_firmware      ros2_control hardware interface (serial), driver IMU MPU6050, sketches Arduino
│   ├── bumperbot_localization  EKF (robot_localization) y AMCL
│   ├── bumperbot_mapping       slam_toolbox y mapas guardados
│   ├── bumperbot_planning      planificadores globales Dijkstra y A* (plugins de Nav2)
│   ├── bumperbot_motion        controladores PD y Pure Pursuit (plugins de Nav2)
│   ├── bumperbot_navigation    configuración de Nav2 y behavior trees
│   ├── bumperbot_utils         safety_stop (parada de seguridad con LiDAR)
│   ├── bumperbot_bringup       launch principales: simulated_robot / real_robot
│   └── bumperbot_msgs          mensajes y servicio del gemelo digital
├── udev/90-bumperbot.rules     nombres fijos /dev/arduino y /dev/rplidar en la Raspberry Pi
├── ci/                         scripts de pruebas automáticas (GitHub Actions)
├── install_dependencies.sh
├── docs/GEMELO_DIGITAL.md      arquitectura, uso y métricas del gemelo digital
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

Mundos disponibles: `empty`, `small_house`, `small_warehouse`, `arena` (recinto rectangular para comparar el LiDAR; se regenera con `twin_calibrate arena`). Mapas disponibles: `small_house`, `small_warehouse`.

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
| `calibration_file` | (vacío) | YAML con la calibración de ruedas (`twin_calibrate wheels`) |
| `use_ekf` | `true` | EKF (robot_localization) fusiona ruedas + IMU y publica `odom → base_footprint` |
| `use_safety_stop` | `false` | Lanza `safety_stop` (frena/detiene si el LiDAR ve un obstáculo cerca) |
| `use_rviz` / `gui` | `true` | Solo simulación: abrir RViz / la ventana de Gazebo |
| `use_mock_hardware` | `false` | Solo robot real: sin Arduino, ruedas simuladas por ros2_control (para probar) |
| `bt_xml` | `simple_navigation_w_replanning_and_recovery.xml` | Behavior tree de Nav2 (ruta completa) |
| `arduino_port` | `/dev/arduino` | Puerto serie del Arduino (solo robot real) |

### Teleoperación

- Joystick: se lanza automáticamente con el bringup.
- Teclado: `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=false`

Prioridades en `twist_mux`: joystick (99) > gemelo digital (95) > teclado (90) > Nav2 (80). El lock `safety_stop` (255) bloquea todo.

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

## Gemelo digital

Robot físico en la Raspberry Pi y gemelo en Gazebo en el PC, sincronizados:

```bash
# Raspberry Pi
ros2 launch bumperbot_digital_twin real_side.launch.py real_domain:=10
# PC
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_domain:=10 twin_domain:=20 dashboard:=true
```

Sin robot, todo en el PC: `ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake driver:=square`.

| Herramienta | Para qué |
|---|---|
| `twin_bridge` | Puente bidireccional (líder/seguidor, corrección, compensación de latencia, red emulada, comparación de LiDAR, registro CSV) |
| `anomaly_detector` | Detecta atascos, deslizamiento, empujones y desvíos del modelo en el robot real |
| `twin_dashboard` | Panel de control: métricas en vivo, gráficas, cambio de líder, red, recorridos |
| `twin_experiment` | Corre un protocolo YAML completo y genera el informe estadístico |
| `twin_calibrate` | Identifica el modelo dinámico (`dynamics`), calibra las ruedas (`wheels`), genera el recinto (`arena`) |
| `analyze_twin_log` / `twin_report` | Métricas y gráficas de una corrida / informe de varias con pruebas t |

Detalles, métricas y experimentos en [`docs/GEMELO_DIGITAL.md`](docs/GEMELO_DIGITAL.md).

## Navegación

Nav2 usa por defecto `SmacPlanner2D` y `RegulatedPurePursuitController`. Los plugins propios (`bumperbot_planning::DijkstraPlanner`, `AStarPlanner`, `bumperbot_motion::PDMotionPlanner`, `PurePursuit`) están comentados en `bumperbot_navigation/config/planner_server.yaml` y `controller_server.yaml`; se activan descomentándolos.

## Integración continua

`.github/workflows/build.yml` compila el workspace completo en ROS 2 Humble en cada push, verifica que el URDF y todos los launch carguen y prueba el gemelo digital sin hardware: pruebas unitarias, sincronización y LiDAR con robots de mentira, detección de fallas inyectadas, el protocolo de experimentos completo (calibración, red degradada, compensación), el panel, el robot real con hardware simulado y Gazebo sin ventana. Los resultados aparecen como anotaciones en la pestaña *Actions* de GitHub.
