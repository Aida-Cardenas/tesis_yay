#!/usr/bin/env bash
# Gemelo en Gazebo (sin interfaz gráfica) siguiendo a un robot real de mentira, ambos
# en el mismo recinto para comparar el LiDAR simulado por Gazebo con el de mentira.

source /opt/ros/humble/setup.bash
source install/setup.bash
source ci/annotate.sh
export ROS_LOCALHOST_ONLY=1

mkdir -p /tmp/twin_gz_logs
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena arena:=[-0.5,2.0,-1.0,1.0] \
  leader:=real driver:=square driver_start_delay:=40.0 linear_speed:=0.2 distance:=0.8 \
  log_dir:=/tmp/twin_gz_logs log_tag:=gazebo > /tmp/twin_gz.log 2>&1 &
PID=$!
sleep 30
export ROS_DOMAIN_ID=20
INFO="$(timeout 20 ros2 control list_controllers 2>&1)"$'\n'
INFO+="scan: $(timeout 10 ros2 topic echo --once --field header.frame_id /scan 2>&1 | head -n1)"$'\n'
INFO+="imu: $(timeout 10 ros2 topic echo --once --field header.frame_id /imu/out 2>&1 | head -n1)"$'\n'
INFO+="odom filtered: $(timeout 10 ros2 topic echo --once --field pose.pose.position.x /odometry/filtered 2>&1 | head -n1)"$'\n'
sleep 60
stop_group $PID
CSV=$(ls /tmp/twin_gz_logs/*_gazebo.csv 2>/dev/null | tail -n1)
FAIL=1
if [ -n "$CSV" ]; then
  python3 -m bumperbot_digital_twin.analyze_log "$CSV" -o /tmp/twin_gz_res --no-plots > /tmp/twin_gz_summary.txt 2>&1
  INFO+="$(cat /tmp/twin_gz_summary.txt)"
  python3 -c "
import json,sys; s=json.load(open('/tmp/twin_gz_res/resumen.json'))[0]
sys.exit(0 if s['error_posicion_m']['n'] > 100 and s['error_final_m'] < 0.08 else 1)" && FAIL=0
fi
if [ $FAIL -eq 0 ]; then
  annotate notice "Gemelo en Gazebo" "$INFO"
else
  annotate warning "Gemelo en Gazebo" "$INFO"
  annotate warning "Log Gazebo (errores)" "$(grep -iE 'error|fail|exception|died' /tmp/twin_gz.log | tail -n 40)"
fi

# Navegación con vista previa: Nav2 + AMCL del gemelo en Gazebo (mapa arena) y robot real de mentira
mkdir -p /tmp/twin_gz_nav_logs
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena map_name:=arena arena:=[-0.5,2.0,-1.0,1.0] \
  log_dir:=/tmp/twin_gz_nav_logs log_tag:=gznav > /tmp/twin_gz_nav.log 2>&1 &
NPID=$!
sleep 90
timeout 300 ros2 run bumperbot_digital_twin twin_navigate 1.0 0.3 0 --name G1 --twin-sim-time \
  --out /tmp/twin_gz_nav_res --no-plots > /tmp/twin_gz_nav_run.txt 2>&1
NAV=$?
stop_group $NPID
if [ $NAV -eq 0 ] && grep -q '"aprobada": true' /tmp/twin_gz_nav_res/informe_navegacion.json 2>/dev/null; then
  annotate notice "Vista previa de navegación con Gazebo + Nav2" "$(cat /tmp/twin_gz_nav_res/informe_navegacion.md)"
else
  annotate warning "Vista previa de navegación con Gazebo + Nav2" "$(tail -n 30 /tmp/twin_gz_nav_run.txt)"
  annotate warning "Log Gazebo + Nav2 (errores)" "$(grep -iE 'error|fail|abort' /tmp/twin_gz_nav.log | tail -n 30)"
  FAIL=1
fi
exit $FAIL
