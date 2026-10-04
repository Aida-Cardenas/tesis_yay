# Gemelo digital: comunicación bidireccional físico–virtual

Paquete: `src/bumperbot_digital_twin`. Es la parte original de la tesis; todo lo demás del workspace viene del curso.

## 1. Arquitectura

```
 Raspberry Pi  (ROS_DOMAIN_ID=10)                  PC  (ROS_DOMAIN_ID=20)
 ┌─────────────────────────────────┐              ┌─────────────────────────────────┐
 │ real_side.launch.py             │              │ digital_twin.launch.py          │
 │  real_robot.launch.py (curso)   │              │  simulated_robot.launch.py      │
 │   ros2_control + Arduino        │              │   (curso) Gazebo + ros2_control │
 │   EKF, SLAM/AMCL, Nav2          │              │   EKF, SLAM/AMCL, Nav2          │
 │   twist_mux ← twin_vel (95)     │              │   twist_mux ← twin_vel (95)     │
 │  latency_echo                   │              │  RViz (ambos robots)            │
 │  anomaly_detector               │              │  twin_dashboard (panel)         │
 └───────────────┬─────────────────┘              └───────────────┬─────────────────┘
                 │ dominio 10          twin_bridge         dominio 20 │
                 └────────────────►  (en el PC, unido  ◄──────────────┘
                                      a ambos dominios)
```

- **Dos dominios DDS.** El robot real y el gemelo corren exactamente el mismo software del curso, cada uno en su `ROS_DOMAIN_ID`. Así sus tópicos (`/tf`, `/joint_states`, `/scan`, `/cmd_vel`…) no se mezclan y no hubo que renombrar nada en los 13 paquetes del curso.
- **Un puente en los dos dominios.** `twin_bridge` crea un contexto de ROS 2 por dominio dentro del mismo proceso. Es el único nodo que ve a los dos robots.
- **Líder y seguidor.** Uno de los dos manda (`leader:=real` o `leader:=twin`) y se puede cambiar en caliente. Los comandos van del líder al seguidor; el estado (odometría, poses, trayectorias, métricas) viaja siempre en los dos sentidos. Eso es la comunicación bidireccional.
- **Entrada propia en twist_mux.** El puente no escribe directo al controlador: publica en `twin_vel`, una entrada nueva de `twist_mux` con prioridad 95. El joystick (99) siempre puede tomar el control, y el `safety_stop` sigue funcionando.

## 2. Sincronización

En cada ciclo (20 Hz) el puente:

1. **Alinea** las odometrías. Al arrancar supone que ambos robots están en el mismo sitio y calcula la transformación entre sus marcos `odom` (servicio `/digital_twin/align` para repetirlo).
2. **Prealimentación.** Toma el último comando del líder (`/bumperbot_controller/cmd_vel`, la salida de `twist_relay`).
3. **Realimentación.** Calcula el error de posición del seguidor respecto al líder y corrige con la ley de seguimiento de Kanayama et al. (1990):

   ```
   v = v_r · cos(eθ) + kx · ex
   ω = ω_r + v_r · ky · ey + kθ · sin(eθ)
   ```

   `ex`, `ey`, `eθ` son el error longitudinal, lateral y de orientación en el marco del seguidor. Las velocidades se saturan (más bajo si el seguidor es el robot real).
4. **Seguridad.** Si los datos de un robot tienen más de 0,5 s, deja de mandar (el seguidor se detiene). Si el error supera 0,75 m, suspende la corrección y marca `lost_sync`.

Con `feedback:=false` el puente solo copia comandos (espejo en lazo abierto). Comparar ambos modos es un buen experimento para la tesis. En las pruebas automáticas (dos robots simulados con 10 % de diferencia de velocidad, cuadrado de 0,6 m a 0,3 m/s):

| Líder | Corrección | Error medio | Error máximo | Error final |
|---|---|---|---|---|
| real | sí | 0,8 cm | 2,2 cm | 0,4 cm |
| real | no (espejo) | 14,7 cm | 23,7 cm | 23,6 cm |
| gemelo | sí | 1,1 cm | 2,6 cm | 0,9 cm |


## 3. Calibración del gemelo (identificación del sistema)

El gemelo en Gazebo es "ideal": acelera al instante y avanza exactamente lo que se le pide. El robot real tiene inercia, retardo de los motores y ruedas que no miden exactamente lo nominal. Para que el gemelo se comporte como el robot real se identifica un modelo de primer orden con tiempo muerto para cada velocidad (lineal y angular):

```
G(s) = K · e^(−L·s) / (τ·s + 1)
```

`K` es la ganancia (cuánto de lo pedido se logra), `τ` la constante de tiempo (qué tan rápido responde) y `L` el retardo. Se ajusta por mínimos cuadrados (`y[k+1] = a·y[k] + b·u[k−d]`, probando cada retardo `d`) sobre los registros del puente en los que el robot real es el líder: el comando del líder es la entrada y la velocidad medida por los encoders, la salida.

```bash
ros2 run bumperbot_digital_twin twin_calibrate dynamics ~/twin_logs/twin_*_E1_*.csv ~/twin_logs/twin_*_E3_*.csv -o modelo_gemelo.yaml
```

Genera el YAML con `K`, `τ`, `L` y `R²` de cada canal y una gráfica del ajuste. Con `use_twin_model:=true twin_model_file:=modelo_gemelo.yaml` (o el panel) el puente filtra los comandos que manda al gemelo con ese modelo, así que el gemelo reproduce la respuesta del robot real aun sin corrección.

**Calibración de ruedas.** Si la odometría del robot real no coincide con lo medido con cinta métrica:

```bash
ros2 run bumperbot_digital_twin twin_calibrate wheels --odom-distance 0.97 --real-distance 1.00 --odom-angle 6.10 --real-angle 6.283 -o ruedas.yaml
ros2 launch bumperbot_digital_twin real_side.launch.py calibration_file:=$PWD/ruedas.yaml
```

Calcula `wheel_radius_multiplier` y `wheel_separation_multiplier` de `diff_drive_controller`.

## 4. Detección de anomalías

`anomaly_detector` corre junto al robot real y usa el modelo del gemelo como referencia de "comportamiento esperado". Compara lo que el robot debería hacer ante cada comando con lo que miden los encoders y la IMU:

| Anomalía | Condición (sostenida, con histéresis) | Ejemplo |
|---|---|---|
| `atasco` | se ordena movimiento y los encoders no se mueven | rueda trabada, choque |
| `deslizamiento` | las ruedas dicen que gira y la IMU no lo confirma | piso liso |
| `movimiento_no_comandado` | el robot se mueve sin comando | lo empujan o lo levantan |
| `desviacion_modelo` | CUSUM de la diferencia entre la velocidad real y la del modelo | batería baja, carga extra |
| `latencia_alta` (puente) | RTT por encima de `latency_alert_ms` | Wi-Fi saturado |
| `perdida_datos` (puente) | dejan de llegar datos de un robot | robot apagado, red caída |

Los eventos (`bumperbot_msgs/TwinAnomaly`, con inicio y fin) se publican en `/digital_twin/events` en los dos dominios, aparecen en el panel y en RViz y quedan en `twin_<fecha>_<tag>_eventos.csv`. Con `lock_on_stall:=true` el detector además bloquea el robot (lock `safety_stop` de twist_mux) mientras dure un atasco.

## 5. Red degradada y compensación de latencia

**Emulación de red.** El puente puede degradar a propósito el enlace con el robot real, en los dos sentidos: retardo fijo, variación (jitter) y pérdida de mensajes (`net_delay_ms`, `net_jitter_ms`, `net_loss`, cambiables en caliente desde el panel o el servicio `/digital_twin/configure`). Así se mide cómo se degrada la sincronización con la latencia sin depender de cómo esté el Wi-Fi ese día.

**Compensación** (`compensation:=true`):

- Si el líder es el robot real, su pose llega con retardo. El puente la *extrapola* hacia el presente con su última velocidad y el retardo de un sentido (mitad del RTT medido).
- Si el seguidor es el robot real, sus comandos llegan tarde y su pose vuelve tarde: lazo cerrado con retardo, que oscila si la corrección es fuerte. El puente usa un **predictor de Smith**: integra los comandos que ya mandó y todavía no se ven reflejados para estimar dónde está el robot *ahora* y corrige sobre esa estimación.

## 6. Comparación de LiDAR real y simulado

El puente reenvía el `/scan` del robot real al dominio del gemelo como `/digital_twin/real_scan` (en RViz se ve en rojo sobre el del gemelo) y, cuando ambos robots están quietos, compara los dos barridos ángulo a ángulo (1°): error absoluto medio, RMSE, sesgo, percentil 95 y coincidencia de visibilidad (si ambos ven o no ven algo en cada ángulo). Resultado en `/digital_twin/scan_comparison` y en `_scan.csv`.

Para que la comparación tenga sentido el entorno debe ser el mismo. `twin_calibrate arena --bounds XMIN XMAX YMIN YMAX [--obstacle X Y ANCHO LARGO]` genera un mundo de Gazebo con un recinto rectangular igual al que se arme con cajas o tablas en el laboratorio (`world_name:=arena`).

## 7. Panel de control

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py dashboard:=true
# o por separado
ros2 run bumperbot_digital_twin twin_dashboard
```

Muestra en vivo líder, error, RTT, retardo, red, LiDAR y anomalías, con gráficas del error y la latencia. Desde ahí se cambia el líder, se realinea, se activan la corrección, la compensación y el gemelo calibrado, se degrada la red, se abre un registro nuevo y se lanzan recorridos automáticos en el robot líder, todo sin terminal.

## 8. Experimentos automáticos e informe

```bash
ros2 run bumperbot_digital_twin twin_experiment protocolo_real
```

Lee un protocolo YAML (`config/protocolo_real.yaml`, `protocolo_simulacion.yaml`), y para cada experimento y repetición configura el puente, realinea, abre un registro `<experimento>_r<n>`, hace el recorrido y espera. Entre corridas pide colocar el robot en la marca (con `--auto` no pregunta). A mitad del protocolo puede calibrar el gemelo con las corridas anteriores (`calibrate_from`). Al final genera `informe.md` / `informe.json`: media ± desviación por experimento, comparaciones con prueba t de Welch (¿la mejora es significativa?) y la gráfica de error contra latencia.

`protocolo_real` cubre toda la Prueba 5: E1–E6 (corrección, espejo, ambos sentidos), calibración, E7 (gemelo calibrado), L100/L250 con y sin compensación (y con el robot real como seguidor), TUNE (sintonización con el modelo de CAL) con G0T y L250RT para validarla, A1/A2 (atasco y empujón provocados a mano).

Informe de varias carpetas de registros: `ros2 run bumperbot_digital_twin twin_report ~/twin_logs/*.csv -o informe`.

## 9. Navegación con vista previa

`twin_navigate` usa al gemelo como **simulador predictivo**: antes de mandar al robot real a una meta, el gemelo la intenta con Nav2.

1. Pausa la sincronización (`pause_sync` en `/digital_twin/configure`) para que el puente no mueva a ninguno de los dos.
2. Manda la meta a la acción `navigate_to_pose` del gemelo y registra su ruta (TF `map → base_footprint`), el tiempo, la distancia mínima del LiDAR a obstáculos y las maniobras de recuperación.
3. **Aprueba** la meta si el gemelo llegó, sin bajar de `--min-clearance` (20 cm por defecto), en menos de `--max-time` y con pocas recuperaciones; si no, la **rechaza** con el motivo.
4. Devuelve al gemelo al punto de partida con Nav2.
5. Si se aprobó (y con `--execute auto`, o confirmando con `--execute ask`), realinea, reanuda la sincronización con el robot real como líder y le manda la misma meta; el gemelo lo sigue en vivo.
6. Compara la ruta real con la prevista: desviación media, p95 y máxima (distancia de cada punto real a la polilínea prevista), Hausdorff, longitudes, diferencia de tiempo y error final.

```bash
ros2 run bumperbot_digital_twin twin_navigate 1.2 0.5 90 --twin-sim-time
ros2 run bumperbot_digital_twin twin_navigate --goals metas_arena --twin-sim-time --execute ask
```

Para que las dos rutas sean comparables, ambos robots se localizan con AMCL en **el mismo mapa** (`map_name:=arena`) y el gemelo corre en el mismo recinto (`world_name:=arena`), los dos arrancando en la marca de salida (0, 0). `twin_calibrate arena --map-dir` genera el mapa junto con el mundo.

Sin Nav2 (`real_mode:=fake twin_mode:=fake`) se usa `fake_navigator`, que ofrece la misma acción `navigate_to_pose` y rechaza metas fuera del recinto.

Resultados: `informe_navegacion.md/json` y, por meta, `vista_previa_<meta>.png/.csv/.json`.

## 10. Sintonización automática de las ganancias

La ley de seguimiento tiene tres ganancias (kx, ky, kθ). Las de por defecto (1,5 / 6 / 3) son buenas sin retardo, pero con latencia y con un robot de respuesta lenta el lazo puede oscilar. `twin_tune` usa al gemelo como modelo para elegirlas:

1. Simula el lazo completo (líder → red → puente → seguidor) con el modelo identificado del robot real (`twin_calibrate dynamics`) y el del gemelo, en cada escenario: recorrido × quién sigue × retardo de red, con o sin compensación.
2. El costo es el RMSE de posición más una pequeña penalización al error de orientación y a los cambios bruscos de comando.
3. Busca en escala logarítmica: una rejilla de 4 × 4 × 4 combinaciones y luego Nelder-Mead desde las dos mejores. Nunca devuelve algo peor que las ganancias de partida.

```bash
ros2 run bumperbot_digital_twin twin_tune --real-model modelo_gemelo.yaml --follower twin,real --delay-ms 0,250
```

Genera `ganancias.yaml` (para `gains_file:=` del launch o `--apply` con el puente corriendo), `ganancias_informe.md` y `ganancias.png` (trayectorias y error antes/después). En los protocolos, el paso `TUNE` lo hace solo con el modelo de `CAL`, y los experimentos con `gains: tuned` usan el resultado.

## 11. Métricas

| Métrica | Cómo se mide | Requisito |
|---|---|---|
| Error de posición y orientación | Diferencia entre las poses alineadas, cada ciclo | — |
| Error "real" (mismo instante) | Igual, pero con la pose real en el instante en que se midió (sello de tiempo) | Relojes sincronizados (`chrony`) |
| RTT (ida y vuelta) | `twin_bridge` publica `/digital_twin/ping`, `latency_echo` en la Raspberry Pi lo devuelve | Ninguno (un solo reloj) |
| Edad de la odometría real | Hora del PC − sello de tiempo del mensaje | `chrony` |
| Retardo de sincronización | Desfase que maximiza la correlación entre la velocidad del líder y la del seguidor | — |
| LiDAR | MAE, RMSE, sesgo y coincidencia de visibilidad entre el barrido real y el simulado | Mismo entorno |
| Anomalías | Eventos por tipo | — |

Todo queda en `~/twin_logs/twin_<fecha>_<tag>.csv` (+ `_scan.csv`, `_eventos.csv` y `.json` con los parámetros usados) y se publica en vivo en `/digital_twin/status` (`bumperbot_msgs/TwinSyncStatus`) en ambos dominios.

## 12. Uso rápido

Robot físico (Raspberry Pi):

```bash
ros2 launch bumperbot_digital_twin real_side.launch.py real_domain:=10
```

PC:

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_domain:=10 twin_domain:=20 dashboard:=true
```

Sin hardware, todo en el PC (`real_mode:=fake`; con `twin_mode:=fake` tampoco hace falta Gazebo). Los robots de mentira aceptan `fake_real_gain`, `fake_real_tau`, `fake_real_delay` (para que se parezcan al real) y `fake_real_fault:=stall|slip|push` (fallas inyectadas):

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake driver:=square
```

Robot real completo sin Arduino (ros2_control con hardware simulado), en otra terminal del mismo PC:

```bash
ros2 launch bumperbot_digital_twin real_side.launch.py use_mock_hardware:=true
```

Recorridos automáticos (`driver:=`): `line`, `line_back`, `square`, `circle`, `rotate`, `figure8`, con `linear_speed`, `angular_speed`, `distance`.

Servicios (en cualquiera de los dos dominios):

```bash
ros2 service call /digital_twin/align std_srvs/srv/Trigger
ros2 service call /digital_twin/switch_leader std_srvs/srv/Trigger
ros2 service call /digital_twin/set_feedback std_srvs/srv/SetBool "{data: false}"
ros2 service call /digital_twin/new_log std_srvs/srv/Trigger
ros2 service call /digital_twin/configure bumperbot_msgs/srv/TwinConfigure "{leader: '', feedback: -1, compensation: 1, twin_model: -1, net_delay_ms: 150.0, net_jitter_ms: 10.0, net_loss: 0.0}"
ros2 service call /digital_twin/configure bumperbot_msgs/srv/TwinConfigure "{feedback: -1, compensation: -1, twin_model: -1, net_delay_ms: -1.0, net_jitter_ms: -1.0, net_loss: -1.0, kx: 0.8, ky: 3.0, ktheta: 1.5}"
```

Análisis de una corrida: `ros2 run bumperbot_digital_twin analyze_twin_log ~/twin_logs/*.csv -o resultados`.

## 13. Parámetros principales (`config/twin_bridge.yaml`)

| Parámetro | Defecto | Qué hace |
|---|---|---|
| `leader` | `real` | Quién manda |
| `feedback` | `true` | Corrección por odometría |
| `kx`, `ky`, `ktheta` | 1.5, 6.0, 3.0 | Ganancias de la ley de seguimiento |
| `max_linear`, `max_angular` | 0.5 m/s, 2.5 rad/s | Saturación general |
| `real_max_linear`, `real_max_angular` | 0.3 m/s, 1.5 rad/s | Saturación cuando el seguidor es el robot real |
| `position_tolerance`, `heading_tolerance` | 2 cm, 0.03 rad | Debajo de esto no corrige |
| `max_correction_error` | 0.75 m | Encima de esto suspende la corrección |
| `state_timeout`, `cmd_timeout` | 0.5 s, 0.3 s | Watchdog |
| `compensation` | `false` | Extrapolación del líder / predictor de Smith |
| `twin_model`, `twin_model_file` | `false`, — | Aplicar el modelo identificado al gemelo |
| `net_delay_ms`, `net_jitter_ms`, `net_loss` | 0 | Red emulada hacia el robot real |
| `scan_compare`, `relay_real_scan` | `true` | Comparar y reenviar el LiDAR real |
| `latency_alert_ms` | 250 | Umbral de la anomalía `latencia_alta` |
| `gains_file` | — | YAML con kx, ky, ktheta (`twin_tune`) |
| `sync` | `true` | `false` = sincronización en pausa (la usa `twin_navigate`) |
| `odom_topic` | `/bumperbot_controller/odom` | Odometría que se compara (`/odometry/filtered` para usar el EKF) |

## 14. Pruebas automáticas

`.github/workflows/build.yml` compila todo en ROS 2 Humble y ejecuta:

1. Pruebas unitarias: geometría y ley de control, predictor de Smith, identificación del modelo, emulador de red, detector de anomalías, comparación de LiDAR, análisis, informe, calibración y panel.
2. Sincronización con dos robots de mentira en dos dominios (líder real con y sin corrección, líder gemelo), con comparación de LiDAR en un recinto.
3. Fallas inyectadas en el robot "real" (atasco, deslizamiento, empujón): el detector debe reportar cada una y ninguna otra.
4. El protocolo `protocolo_ci` completo con `twin_experiment --auto`: robot "real" más lento y con retardo, calibración automática, red de 200 y 300 ms con y sin compensación, y sintonización automática de ganancias.
5. Navegación con vista previa con los navegadores de mentira: cuatro metas aprobadas y ejecutadas, una rechazada por pasar cerca de la pared y otra fuera del recinto.
6. El panel dibujado sin pantalla.
7. El robot real completo con hardware simulado (ros2_control mock) + EKF + Nav2 + detector + medidor de odometría, y el gemelo siguiéndolo.
8. El gemelo en Gazebo sin ventana, en el recinto, siguiendo a un robot real de mentira, y una meta de navegación con vista previa usando Nav2 y AMCL del gemelo (experimental).

Resultados de la corrida automática del 4 de octubre de 2026 (todas las pruebas aprobadas; todo simulado, en un solo computador):

**Sincronización y LiDAR** (robot "real" de mentira y gemelo con 10 % menos de velocidad, cuadrado de 0,6 m a 0,3 m/s, ambos en un recinto de 2 × 2 m):

| Líder | Corrección | Error medio | Error máximo | Error final | MAE del LiDAR | Anomalías |
|---|---|---|---|---|---|---|
| real | sí | 1,0 cm | 2,2 cm | 0,8 cm | 1,9 cm | ninguna |
| real | no (espejo) | 15,5 cm | 24,5 cm | 24,4 cm | 1,6 cm | ninguna |
| gemelo | sí | 1,0 cm | 2,2 cm | 0,9 cm | 2,2 cm | ninguna |

**Fallas inyectadas:** atasco, deslizamiento y empujón se detectaron cada uno como su tipo, sin falsas alarmas de los otros.

**Protocolo completo** (`protocolo_ci` con `twin_experiment --auto`, 2 repeticiones; el robot "real" de mentira tiene K = 0,85, τ = 0,3 s y 0,1 s de retardo, y el gemelo responde casi ideal):

| Experimento | RMSE de posición | Reducción |
|---|---|---|
| M: solo copia comandos | 24,2 cm | |
| MC: solo copia comandos, gemelo calibrado | 2,7 cm | −89 % |
| D: red de 200 ms, sin compensación | 3,3 cm | |
| DC: red de 200 ms, con compensación | 1,9 cm | −42 % |
| R: el real sigue al gemelo, red de 300 ms (error real en el mismo instante) | 24,2 cm | |
| RC: lo mismo con predictor de Smith | 13,2 cm | −45 % |
| RT: lo mismo con ganancias sintonizadas por el gemelo (kx 0,3 / ky 0,5 / kθ 0,3) | 12,4 cm | −49 % |

Las cuatro mejoras fueron significativas en la prueba t de Welch (p < 0,05). La sintonización sola logra lo mismo que el predictor de Smith: con 300 ms de retardo, lo mejor es corregir con suavidad. El modelo identificado a partir de M fue K = 0,86, τ = 0,34 s y L = 0,05 s (R² = 0,999), cercano a los valores con que se configuró el robot de mentira (el ajuste reparte parte del retardo en τ).

**Navegación con vista previa** (navegadores de mentira, recinto `arena`):

| Meta | Veredicto del gemelo | Tiempo gemelo / real | Desviación media / máxima entre rutas | Error final del real |
|---|---|---|---|---|
| N1 (1,2; 0) | Aprobada | 7,4 s / 6,7 s | 0,0 / 0,2 cm | 4,8 cm |
| N2 (1,0; 0,6) | Aprobada | 7,8 s / 6,1 s | 1,8 / 4,3 cm | 4,5 cm |
| N3 (0,3; −0,6) | Aprobada | 13,3 s / 11,2 s | 2,9 / 5,4 cm | 4,6 cm |
| N4 (0; 0) | Aprobada | 7,8 s / 7,2 s | 2,1 / 8,6 cm | 4,4 cm |
| X1 pegada a la pared | Rechazada: pasó a 18 cm (mínimo 20) | 11,8 s / — | — | el real no se movió |
| X2 fuera del recinto | Rechazada: no hay camino | 0,1 s / — | — | el real no se movió |

**Vista previa con Nav2 de verdad**: el gemelo en Gazebo (AMCL en el mapa `arena` y Nav2) aprobó una meta a (1,0; 0,3) m en 8,6 s; el robot real de mentira la ejecutó en 6,8 s con una desviación media de 4,5 cm respecto a la ruta prevista (máxima 8,4 cm).

**Gemelo en Gazebo** (física, EKF y Nav2) siguiendo al robot de mentira en el recinto `arena`: error medio 0,5 cm, máximo 2,7 cm; LiDAR de Gazebo frente al de mentira: MAE 2,2 cm, coincidencia de visibilidad 99,7 %.

**Robot real con hardware simulado** (ros2_control mock + EKF + Nav2 + detector, 10 comprobaciones aprobadas) y gemelo siguiéndolo en un ida y vuelta de 0,8 m: error medio 1,0 cm, máximo 1,5 cm.

La latencia de red en estas pruebas es local (menos de 2 ms); con la Raspberry Pi por Wi-Fi será mayor, y es lo que mide `protocolo_real`.

## 15. Limitaciones conocidas

- La alineación inicial supone que ambos robots arrancan en el mismo punto. Si no, hay que llamar a `/digital_twin/align` con los robots en posiciones equivalentes.
- Se comparan odometrías, que también derivan. Para comparar posiciones absolutas, ambos deben localizarse en el mismo mapa (AMCL) y usar `/amcl_pose`; queda como mejora.
- Con el robot parado no se puede corregir el error lateral (restricción no holonómica del robot diferencial); se corrige en cuanto vuelve a avanzar.
- El modelo de primer orden no representa la zona muerta de los motores a velocidades muy bajas; conviene identificar con las velocidades que se usarán en las pruebas.
- La compensación supone que el retardo es aproximadamente simétrico (un sentido = RTT/2).
- La vista previa supone que el gemelo y el robot real están localizados en el mismo mapa y en el mismo recinto; si el entorno real cambia (alguien pone una caja), el gemelo no lo sabe.
- La sintonización es tan buena como el modelo identificado; conviene validarla siempre con un experimento (`G0T`, `L250RT`).
- Todo lo anterior está probado con robots simulados; los valores con el robot físico salen de correr `protocolo_real` y `twin_navigate`.
