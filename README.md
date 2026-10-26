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
│   │                           red degradada, compensación de latencia, LiDAR, panel, experimentos,
│   │                           navegación con vista previa, sintonización de ganancias
│   ├── bumperbot_description   URDF/xacro, mallas, el recinto de pruebas en Gazebo, launch de simulación
│   ├── bumperbot_controller    diff_drive_controller, teleoperación, twist_mux
│   ├── bumperbot_firmware      ros2_control hardware interface (serial), driver IMU MPU6050, firmware del Arduino
│   ├── bumperbot_localization  EKF (robot_localization) y AMCL
│   ├── bumperbot_mapping       slam_toolbox y mapas guardados
│   ├── bumperbot_navigation    configuración de Nav2 y su behavior tree
│   ├── bumperbot_utils         safety_stop (parada de seguridad con LiDAR)
│   ├── bumperbot_bringup       launch principales: simulated_robot / real_robot
│   └── bumperbot_msgs          mensajes y servicio del gemelo digital
├── udev/90-bumperbot.rules     nombres fijos /dev/arduino y /dev/rplidar en la Raspberry Pi
├── ci/                         scripts de pruebas automáticas (GitHub Actions)
├── install_dependencies.sh
├── docs/GEMELO_DIGITAL.md      arquitectura, uso y métricas del gemelo digital
├── docs/ORIGEN_DEL_CODIGO.md   qué vino de qué módulo y cómo se relaciona con la tesis
├── docs/RESPUESTA_REVISION.md  qué se hizo con cada punto de la primera revisión de código
├── docs/RESPUESTA_REVISION_2.md  ídem para la segunda revisión
├── docs/RESPUESTA_REVISION_3.md  ídem para la tercera revisión
├── docs/RESPUESTA_REVISION_4.md  ídem para la cuarta revisión
├── docs/REVISION_GENERAL.md     revisión de errores y seguridad antes de cargar el código en el robot
└── docs/RESPUESTA_OPTIMIZACION.md  optimizaciones posteriores (odometría con contadores, sintonización)
```

## Requisitos

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (`ros-humble-desktop` en la PC, `ros-humble-ros-base` en la Raspberry Pi)
- Gazebo Fortress (se instala solo con las dependencias; en Humble se usa `ign_ros2_control`)

## Instalación

```bash
git clone <URL-de-tu-repo> ~/tesis_ws
cd ~/tesis_ws
./install_dependencies.sh           # en la Raspberry Pi: ./install_dependencies.sh robot <IP del PC>
colcon build
source install/setup.bash
```

Recomendado: agregar `source ~/tesis_ws/install/setup.bash` al final de `~/.bashrc`.

### En la Raspberry Pi

Probado para Ubuntu Server 22.04 de 64 bits con `ros-humble-ros-base`.

1. **Instalar y compilar.** La compilación completa en paralelo puede quedarse sin memoria en una Pi (se cuelga o el compilador muere con `Killed`). Se compila de a un paquete y un proceso a la vez:
   ```bash
   ./install_dependencies.sh robot <IP del PC>
   MAKEFLAGS=-j1 colcon build --parallel-workers 1
   source install/setup.bash
   ```
   Con 2 GB de RAM o menos conviene además activar 2 GB de swap antes de compilar (`sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`).
2. **Cerrar la sesión y volver a entrar** (o reiniciar), para que el usuario quede de verdad en los grupos `dialout` (Arduino y LiDAR) e `i2c` (IMU). Comprobar con `groups`.
3. **I2C para la IMU.** Debe existir `/dev/i2c-1` y el MPU6050 debe responder en la dirección 68:
   ```bash
   ls /dev/i2c-1
   sudo apt install -y i2c-tools && i2cdetect -y 1
   ```
   Si no existe `/dev/i2c-1`, agregar `dtparam=i2c_arm=on` a `/boot/firmware/config.txt` y reiniciar. Si `i2cdetect` no muestra `68`, revisar SDA (GPIO 2, pin 3), SCL (GPIO 3, pin 5), 3,3 V y GND.
4. **Red.** El PC y la Pi en la misma red, con el mismo `ROS_DOMAIN_ID` y sin `ROS_LOCALHOST_ONLY=1`. Comprobar con `ros2 topic list` en el PC mientras corre algo en la Pi.
5. **Lanzar dentro de `tmux`.** Si el robot se lanza por SSH y el Wi-Fi se corta un momento, la sesión se cierra, el launch se detiene con ella y se pierde la corrida. Dentro de `tmux` sigue corriendo; después se vuelve a la misma sesión:
   ```bash
   sudo apt install -y tmux
   tmux new -s robot
   ros2 launch bumperbot_digital_twin real_side.launch.py real_domain:=10
   ```
   `Ctrl+b` y luego `d` deja la sesión corriendo en segundo plano; `tmux attach -t robot` vuelve a ella.

## Uso en simulación (gemelo digital en Gazebo)

**Localización con AMCL + navegación** en el recinto de pruebas `arena`:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

En RViz: **Nav2 Goal** para enviar una meta (AMCL arranca en la marca de salida; si se pierde, **2D Pose Estimate**).

**Mapeo con SLAM** en el mismo recinto:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

El único mundo es `arena`, el recinto rectangular de pruebas que también se arma en el laboratorio. Su mundo y su mapa se regeneran juntos con `twin_calibrate arena` si el recinto real mide otra cosa. Ese comando no toca dos cosas que hay que ajustar a mano: las metas de navegación de `bumperbot_digital_twin/config/metas_arena.yaml` (deben quedar dentro del recinto nuevo) y el argumento `arena` de `digital_twin.launch.py` (recinto del robot de prueba `fake_robot`), que tiene por defecto las medidas del recinto actual.

### Argumentos de `simulated_robot.launch.py` / `real_robot.launch.py`

| Argumento | Por defecto | Qué hace |
|---|---|---|
| `use_slam` | `false` | `true`: slam_toolbox (mapeo). `false`: map_server + AMCL |
| `world_name` | `arena` | Mundo de Gazebo (solo simulación) |
| `map_name` | `arena` | Mapa para AMCL, en `bumperbot_mapping/maps/<map_name>/map.yaml` |
| `calibration_file` | (vacío) | YAML con la calibración de ruedas (`twin_calibrate wheels`) |
| `use_ekf` | `true` | EKF (robot_localization) fusiona ruedas + IMU y publica `odom → base_footprint` |
| `use_safety_stop` | `false` | Lanza `safety_stop` (frena/detiene si el LiDAR ve un obstáculo cerca) |
| `use_rviz` | `true` | Solo simulación: abrir RViz |
| `gui` | `true` | Solo simulación: `false` corre Gazebo sin ventana |
| `use_mock_hardware` | `false` | Solo robot real: sin Arduino, ruedas simuladas por ros2_control (para probar) |
| `use_lidar` | `true` | Solo robot real: `false` no arranca el driver del RPLidar (pruebas en la mesa sin el sensor) |
| `bt_xml` | `bumperbot_navigation/behavior_tree/simple_navigation_w_replanning_and_recovery.xml` | Behavior tree de Nav2 (ruta completa al archivo) |
| `arduino_port` | `/dev/arduino` | Puerto serie del Arduino (solo robot real) |

### Teleoperación

- Joystick: se lanza automáticamente con el bringup.
- Teclado: `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=false`

Prioridades en `twist_mux`: joystick (99) > gemelo digital (95) > teclado (90) > Nav2 (80). Los locks `safety_stop` (parada por LiDAR) y `anomaly_stop` (atasco detectado por el gemelo), ambos con prioridad 255, bloquean todo.

### Guardar un mapa nuevo

Con el SLAM corriendo, después de recorrer el entorno:

```bash
mkdir -p src/bumperbot_mapping/maps/mi_mapa
ros2 run nav2_map_server map_saver_cli -f src/bumperbot_mapping/maps/mi_mapa/map
colcon build --packages-select bumperbot_mapping
```

Luego se usa con `map_name:=mi_mapa`.

## Uso con el robot real

1. **Firmware.** Cargar `src/bumperbot_firmware/firmware/robot_control/robot_control.ino` en el Arduino con el Arduino IDE (necesita la biblioteca **PID** de Brett Beauregard, desde el gestor de bibliotecas). En los Nano clones, si la carga falla con `not in sync`, elegir *Procesador: ATmega328P (Old Bootloader)*. El monitor serie va a 115200 baudios. Si un motor gira al revés, se invierten sus dos cables en el L298N; si un encoder cuenta al revés, se intercambian sus fases A y B.
2. **Reglas udev** (en la Raspberry Pi, una sola vez). Revisar los IDs con `lsusb` y ajustar el archivo si hace falta:
   ```bash
   sudo cp udev/90-bumperbot.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
   ls -l /dev/arduino /dev/rplidar
   ```
3. **Relojes.** `install_dependencies.sh` deja al PC como servidor de hora (chrony) y a la Raspberry Pi tomando la hora de él, si se le pasó la IP del PC. Hay que comprobarlo antes de cualquier toma de datos, porque el error "real" entre los robots compara instantes de los dos relojes: 50 ms de diferencia ya son 0,75 cm a 0,15 m/s.
   ```bash
   chronyc sources      # en la Raspberry Pi: la IP del PC debe aparecer con ^*
   chronyc tracking     # "System time" debe ser de pocos milisegundos
   ```
   Al empezar, `twin_experiment` mide la diferencia de relojes con el robot: avisa si supera 10 ms y no arranca si supera 50 ms.
4. **Lanzar** en la Raspberry Pi:
   ```bash
   ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true
   ```
   Sin udev, se puede indicar el puerto a mano: `arduino_port:=/dev/ttyUSB0`. El monitor serie del Arduino IDE tiene que estar cerrado: dos programas no pueden usar el puerto a la vez.

   Al arrancar es normal ver "Waiting for the first data from the Arduino" durante uno o dos segundos (abrir el puerto reinicia el Arduino). Si después aparece "No data from the Arduino", revisar el cable, que el sketch esté cargado y los 115200 baudios. Si el LiDAR no está conectado, su nodo se reinicia cada 3 s hasta que aparezca `/dev/rplidar`; el resto del robot sigue funcionando.
5. **Comprobar el sentido de giro, antes de cualquier otra prueba.** Si el robot estaba cableado para el sketch "invertido" del curso, con el sketch actual puede andar hacia atrás mientras la odometría de las ruedas parece correcta (los encoders se invierten junto con los motores). Solo la IMU y el LiDAR lo delatan.
   1. *Hacia adelante, sin ROS.* Con `real_robot.launch.py` cerrado, abrir el monitor serie del Arduino IDE a 115200 baudios y enviar `rp03.00,lp03.00,` varias veces seguidas (el Arduino frena las ruedas si pasa medio segundo sin comandos). El robot debe avanzar hacia su frente y las respuestas deben empezar con `rp` y traer `lp` (signo positivo en ambas ruedas), y los contadores `rc` y `lc` del final de cada línea deben ir aumentando. Si una rueda gira hacia atrás, se invierten los dos cables de ese motor en el L298N; si gira hacia adelante pero responde con `n`, se intercambian las fases A y B de su encoder.
   2. *Giro, con ROS.* Lanzar `real_robot.launch.py` y en otra terminal:
      ```bash
      ros2 topic pub -r 10 /key_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
      ```
      Visto desde arriba, el robot debe girar en sentido **antihorario**. Mientras gira, en otras dos terminales:
      ```bash
      ros2 topic echo /bumperbot_controller/odom --field twist.twist.angular.z
      ros2 topic echo /imu/out --field angular_velocity.z
      ```
      Los dos deben ser **positivos** y parecidos (unos 0,5 rad/s), y el rumbo de la odometría debe ir aumentando. Si el robot gira en sentido horario, o la IMU da negativo mientras la odometría da positivo, hay que corregir el cableado (paso anterior) antes de seguir.
6. **Visualizar** desde la laptop (misma red y mismo `ROS_DOMAIN_ID` en ambas máquinas):
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

Para la toma de datos, Gazebo sin ventana (`gui:=false`): la ventana es lo que más lo frena, y si el gemelo corre más lento que el tiempo real el informe lo marca como aviso de calidad. RViz o el panel bastan para mirar:

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_domain:=10 twin_domain:=20 gui:=false dashboard:=true
```

| Herramienta | Para qué |
|---|---|
| `twin_bridge` | Puente bidireccional (líder/seguidor, corrección, compensación de latencia, red emulada, comparación de LiDAR, registro CSV) |
| `anomaly_detector` | Detecta atascos, deslizamiento, empujones y desvíos del modelo en el robot real |
| `twin_dashboard` | Panel de control: métricas en vivo, gráficas, cambio de líder, red, recorridos |
| `twin_experiment` | Corre un protocolo YAML completo (condiciones intercaladas, sesión retomable con `--resume`) y genera el informe estadístico |
| `twin_calibrate` | Identifica el modelo dinámico (`dynamics`), calibra las ruedas (`wheels`), genera el recinto y su mapa (`arena`) |
| `twin_tune` | Sintoniza las ganancias de la corrección simulando con el modelo del gemelo |
| `twin_navigate` | Navegación con vista previa: el gemelo prueba la meta con Nav2 antes que el robot real |
| `twin_odom_meter` | Mide distancia y giro según la odometría (para calibrar las ruedas) |
| `analyze_twin_log` / `twin_report` | Métricas y gráficas de una corrida / informe de varias con pruebas t |

Detalles, métricas y experimentos en [`docs/GEMELO_DIGITAL.md`](docs/GEMELO_DIGITAL.md).

## Navegación

Nav2 usa `SmacPlanner2D` como planificador global y `RegulatedPurePursuitController` como controlador local, con el behavior tree `simple_navigation_w_replanning_and_recovery.xml` (replanifica y tiene maniobras de recuperación).

## Integración continua

`.github/workflows/build.yml` compila el workspace completo en ROS 2 Humble en cada push, verifica que el URDF y todos los launch carguen y prueba el gemelo digital sin hardware: pruebas unitarias, sincronización y LiDAR con robots de mentira, detección de fallas inyectadas, el protocolo de experimentos completo (calibración, red degradada, compensación, sintonización), la navegación con vista previa, el panel, el robot real con hardware simulado y Gazebo sin ventana. Los resultados aparecen como anotaciones en la pestaña *Actions* de GitHub.
