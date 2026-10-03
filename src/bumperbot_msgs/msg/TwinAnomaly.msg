# Evento de anomalía detectado usando al gemelo como referencia.

builtin_interfaces/Time stamp
# "detector" (bumperbot_digital_twin/anomaly_detector) o "puente" (twin_bridge)
string source
# atasco | deslizamiento | movimiento_no_comandado | desviacion_modelo | latencia_alta | perdida_datos
string kind
# inicio | fin
string phase
# 1 = aviso, 2 = crítico
uint8 severity
float64 value
string description
