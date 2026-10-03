#!/usr/bin/env bash
# Robot "real" completo (bumperbot_bringup/real_robot) con ros2_control mock hardware,
# EKF y Nav2, y luego el gemelo digital conectado a ese robot.

source /opt/ros/humble/setup.bash
source install/setup.bash
source ci/annotate.sh
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=10

RESULTS=""
FAIL=0
check() {
  local name="$1"; shift
  local out
  if out=$(timeout 25 "$@" 2>&1); then
    RESULTS+="OK   ${name}"$'\n'
  else
    RESULTS+="FAIL ${name}: $(echo "$out" | tail -n 3 | tr '\n' ' ')"$'\n'
    FAIL=1
  fi
}

setsid ros2 launch bumperbot_digital_twin real_side.launch.py use_mock_hardware:=true use_ekf:=true real_domain:=10 \
  > /tmp/mock_stack.log 2>&1 &
PID=$!
sleep 30

check "controladores activos" bash -c "ros2 control list_controllers | grep -E 'bumperbot_controller.*active' && ros2 control list_controllers | grep -E 'joint_state_broadcaster.*active'"
check "enable_odom_tf false con EKF" bash -c "ros2 param get /bumperbot_controller enable_odom_tf | grep -i false"
check "EKF publica odometry filtered" bash -c "ros2 topic echo --once /odometry/filtered > /dev/null"
check "TF odom a base_footprint" bash -c "timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 Translation"
check "Nav2 controller_server activo" bash -c "ros2 lifecycle get /controller_server | grep -i active"
check "Nav2 bt_navigator activo" bash -c "ros2 lifecycle get /bt_navigator | grep -i active"
check "detector de anomalías activo" bash -c "ros2 node list | grep anomaly_detector"
check "twist_mux con entrada twin_vel" bash -c "ros2 param get /twist_mux topics.digital_twin.topic | grep twin_vel"
timeout 6 ros2 topic pub -r 10 /key_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" > /dev/null 2>&1
check "el robot avanza con key_vel" bash -c "x=\$(ros2 topic echo --once --field pose.pose.position.x /odometry/filtered | head -n1); echo x=\$x; python3 -c \"import sys; sys.exit(0 if float('\$x') > 0.3 else 1)\""

mkdir -p /tmp/twin_mock_logs
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=hardware twin_mode:=fake rviz:=false \
  leader:=real driver:=line_back driver_start_delay:=3.0 distance:=0.8 \
  log_dir:=/tmp/twin_mock_logs log_tag:=mock > /tmp/twin_mock.log 2>&1 &
TPID=$!
sleep 35
stop_group $TPID
CSV=$(ls /tmp/twin_mock_logs/*_mock.csv 2>/dev/null | tail -n1)
if [ -n "$CSV" ]; then
  python3 -m bumperbot_digital_twin.analyze_log "$CSV" -o /tmp/twin_mock_res --no-plots > /tmp/twin_mock_summary.txt 2>&1
  check "gemelo sigue al robot real (mock) error final < 5 cm" python3 -c "
import json,sys; s=json.load(open('/tmp/twin_mock_res/resumen.json'))[0]
print(s['error_final_m'], s['error_posicion_m'])
sys.exit(0 if s['error_final_m'] < 0.05 and s['error_posicion_m']['n'] > 100 else 1)"
  annotate notice "Gemelo con robot real simulado (mock)" "$(cat /tmp/twin_mock_summary.txt)"
else
  RESULTS+="FAIL el puente no escribió CSV"$'\n'
  FAIL=1
fi

stop_group $PID
if [ $FAIL -eq 0 ]; then
  annotate notice "Robot real con mock hardware" "$RESULTS"
else
  annotate error "Robot real con mock hardware" "$RESULTS"
  annotate error "Log real_side (final)" "$(grep -iE 'error|fail|exception' /tmp/mock_stack.log | grep -viE 'rplidar|mpu6050|i2c|smbus|joy' | tail -n 40)"
  annotate error "Log digital_twin (final)" "$(tail -n 40 /tmp/twin_mock.log)"
fi
exit $FAIL
