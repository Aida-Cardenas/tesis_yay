#!/usr/bin/env bash
# Gemelo en Gazebo (sin interfaz gráfica) siguiendo a un robot real de mentira.
set -u
source /opt/ros/humble/setup.bash
source install/setup.bash
source ci/annotate.sh
export ROS_LOCALHOST_ONLY=1

mkdir -p /tmp/twin_gz_logs
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
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
kill -INT $PID; wait $PID 2>/dev/null
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
exit $FAIL
