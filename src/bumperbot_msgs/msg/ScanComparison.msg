# Comparación entre el LiDAR del robot real y el del gemelo (rejilla de 1°).

builtin_interfaces/Time stamp
# Haces en los que ambos sensores ven un obstáculo
uint32 common_beams
# Diferencias de distancia real - gemelo [m]
float64 mae
float64 rmse
float64 bias
float64 p95
# Fracción de haces con lectura válida en cada sensor
float64 coverage_real
float64 coverage_twin
# Fracción de haces en que ambos coinciden en ver o no ver un obstáculo
float64 visibility_agreement
