# Estado de sincronización entre el robot físico y su gemelo digital.
# Publicado por bumperbot_digital_twin/twin_bridge en ambos dominios ROS.

# Tiempo de pared (reloj del PC que corre el puente)
builtin_interfaces/Time stamp

# "real" o "twin": quién manda en este momento
string leader

# true cuando el puente está enviando comandos al seguidor
bool active
# true cuando el error superó max_correction_error y se suspendió la corrección
bool lost_sync
# true cuando los datos del líder o del seguidor llegaron tarde (watchdog)
bool stale

# Error del seguidor respecto al líder, en el marco del seguidor
float64 position_error      # [m]
float64 heading_error       # [rad]
float64 error_longitudinal  # [m] ex
float64 error_lateral       # [m] ey

# Poses en el marco odom del seguidor (el líder ya alineado)
geometry_msgs/Pose2D leader_pose
geometry_msgs/Pose2D follower_pose

# Comando enviado al seguidor y su parte de prealimentación
geometry_msgs/Twist feedforward
geometry_msgs/Twist command

# Latencias [ms]. NaN cuando no hay dato.
float64 rtt_ms              # ida y vuelta PC <-> Raspberry Pi (ping/pong)
float64 real_odom_age_ms    # edad de la odometría real al llegar (requiere relojes sincronizados)
float64 leader_cmd_age_ms   # edad del último comando del líder

# Configuración vigente
bool feedback
bool compensation
bool twin_model
float64 net_delay_ms
float64 net_loss
# Retardo de un sentido estimado para la compensación [ms]
float64 one_way_delay_ms
# Error sin predicción (lo que se ve con los datos tal como llegan) [m]
float64 position_error_raw
# Anomalías activas, separadas por "|"
string anomalies
# false mientras la sincronización está en pausa (por ejemplo, durante una vista previa de navegación)
bool sync
# Ganancias de la ley de seguimiento en uso
float64 kx
float64 ky
float64 ktheta
