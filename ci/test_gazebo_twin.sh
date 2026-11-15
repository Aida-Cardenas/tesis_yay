#!/usr/bin/env bash
# Gemelo en Gazebo (sin interfaz gráfica) siguiendo a un robot real de mentira, ambos
# en el mismo recinto para comparar el LiDAR simulado por Gazebo con el de mentira.

source /opt/ros/humble/setup.bash
source install/setup.bash
source ci/annotate.sh
export ROS_LOCALHOST_ONLY=1

# Un reintento no debe analizar archivos del intento anterior
rm -rf /tmp/twin_gz_logs /tmp/twin_gz_res /tmp/twin_gz_nav_logs /tmp/twin_gz_nav_res
mkdir -p /tmp/twin_gz_logs
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena arena:=[-1.0,2.0,-0.5,1.5] \
  leader:=real driver:=square driver_start_delay:=40.0 linear_speed:=0.2 distance:=0.8 \
  log_dir:=/tmp/twin_gz_logs log_tag:=gazebo > /tmp/twin_gz.log 2>&1 &
PID=$!
sleep 30
export ROS_DOMAIN_ID=20
INFO="$(timeout 20 ros2 control list_controllers 2>&1)"$'\n'
SCAN_FRAME="$(timeout 10 ros2 topic echo --once --field header.frame_id /scan 2>&1 | head -n1)"
INFO+="scan: ${SCAN_FRAME}"$'\n'
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
if [ "$SCAN_FRAME" != "laser_link" ]; then
  INFO+="FAIL el LiDAR de Gazebo publica en '${SCAN_FRAME}' y no en laser_link"$'\n'
  FAIL=1
fi
if [ $FAIL -eq 0 ]; then
  annotate notice "Gemelo en Gazebo" "$INFO"
else
  annotate warning "Gemelo en Gazebo" "$INFO"
  annotate warning "Log Gazebo (errores)" "$(grep -iE 'error|fail|exception|died' /tmp/twin_gz.log | tail -n 40)"
fi

# Antes de cada corrida el gemelo vuelve a la marca: tres rectas de 1 m en el mismo
# sentido (sin volver, el gemelo chocaría con la pared este en la segunda) y el
# LiDAR del gemelo debe coincidir con el del robot de mentira en todas.
rm -rf /tmp/twin_gz_mark_logs /tmp/twin_gz_mark_res
mkdir -p /tmp/twin_gz_mark_logs
cat > /tmp/protocolo_marca.yaml <<'YAML'
name: ci_marca
defaults:
  repetitions: 3
  linear_speed: 0.2
  distance: 1.0
  settle: 2.0
experiments:
  - {name: LINEA, description: Tres rectas de 1 m en el mismo sentido, pattern: line}
YAML
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena arena:=[-1.0,2.0,-0.5,1.5] \
  log_dir:=/tmp/twin_gz_mark_logs > /tmp/twin_gz_mark.log 2>&1 &
MPID=$!
sleep 45
timeout 300 ros2 run bumperbot_digital_twin twin_experiment /tmp/protocolo_marca.yaml --auto --no-plots \
  --out /tmp/twin_gz_mark_res > /tmp/twin_gz_mark_run.txt 2>&1
MARK=$?
stop_group $MPID
MARK_INFO="$(grep -E 'marca|LINEA' /tmp/twin_gz_mark_run.txt | head -n 20)"$'\n'
if [ $MARK -eq 0 ] && python3 -c "
import json,sys
t=json.load(open('/tmp/twin_gz_mark_res/informe.json'))['experimentos']['LINEA']['lidar_mae']
runs=[r for r in json.load(open('/tmp/twin_gz_mark_res/sesion.json'))['corridas'] if r['completa']]
print('MAE del LiDAR [m]:', t, '· gemelo en la marca:', [r.get('gemelo_en_marca') for r in runs])
sys.exit(0 if t[2] == 3 and t[0] < 0.06 and len(runs) == 3 and all(r.get('gemelo_en_marca') for r in runs) else 1)
" >> /tmp/twin_gz_mark_check.txt 2>&1; then
  annotate notice "Gemelo de vuelta a la marca en cada corrida" "${MARK_INFO}$(cat /tmp/twin_gz_mark_check.txt)"
else
  annotate warning "Gemelo de vuelta a la marca en cada corrida" \
    "${MARK_INFO}$(cat /tmp/twin_gz_mark_check.txt 2>/dev/null)"$'\n'"$(tail -n 30 /tmp/twin_gz_mark_run.txt)"
  FAIL=1
fi

# El real de mentira sigue al gemelo con 300 ms: con las ganancias por defecto termina
# todavía corrigiendo. Antes de volver a la marca hay que pausar la sincronización; si no,
# sigue girando mientras vuelve, la alineación nueva lo empareja fuera de la marca y en la
# corrida siguiente (con las ganancias del protocolo de simulación) el LiDAR no coincide.
rm -rf /tmp/twin_gz_rt_logs /tmp/twin_gz_rt_res
mkdir -p /tmp/twin_gz_rt_logs
cat > /tmp/protocolo_rt.yaml <<'YAML'
name: ci_rt
defaults:
  leader: twin
  pattern: square
  net_delay_ms: 300
  linear_speed: 0.15
  angular_speed: 0.6
  distance: 1.0
  settle: 3.0
  gains: default
experiments:
  - {name: R, description: El real sigue al gemelo con 300 ms, repetitions: 2}
  - {name: RT, description: Igual con kθ bajo, repetitions: 2, gains: {kx: 0.9, ky: 8.3, ktheta: 0.2}}
YAML
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena arena:=[-1.0,2.0,-0.5,1.5] fake_real_tau:=0.3 fake_real_gain:=0.85 \
  log_dir:=/tmp/twin_gz_rt_logs > /tmp/twin_gz_rt.log 2>&1 &
RPID=$!
sleep 45
timeout 600 ros2 run bumperbot_digital_twin twin_experiment /tmp/protocolo_rt.yaml --auto --no-plots \
  --out /tmp/twin_gz_rt_res > /tmp/twin_gz_rt_run.txt 2>&1
RT=$?
stop_group $RPID
if [ $RT -eq 0 ] && python3 -c "
import json,sys
e=json.load(open('/tmp/twin_gz_rt_res/informe.json'))['experimentos']
t={k: e[k].get('lidar_mae') for k in ('R', 'RT')}
print('MAE del LiDAR [m] (media, desviación, corridas):', t)
sys.exit(0 if t['RT'] and t['RT'][2] >= 1 and t['RT'][0] < 0.06 and (not t['R'] or not t['R'][2] or t['R'][0] < 0.06) else 1)
" > /tmp/twin_gz_rt_check.txt 2>&1; then
  annotate notice "El real sigue al gemelo con 300 ms: LiDAR después de cambiar las ganancias" "$(cat /tmp/twin_gz_rt_check.txt)"
else
  annotate warning "El real sigue al gemelo con 300 ms: LiDAR después de cambiar las ganancias" \
    "$(cat /tmp/twin_gz_rt_check.txt 2>/dev/null)"$'\n'"$(tail -n 30 /tmp/twin_gz_rt_run.txt)"
  FAIL=1
fi

# Navegación con vista previa: Nav2 + AMCL del gemelo en Gazebo (mapa arena) y robot real de mentira
mkdir -p /tmp/twin_gz_nav_logs
setsid ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake twin_mode:=gazebo gui:=false rviz:=false \
  world_name:=arena map_name:=arena arena:=[-1.0,2.0,-0.5,1.5] \
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
