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

## 3. Métricas

| Métrica | Cómo se mide | Requisito |
|---|---|---|
| Error de posición y orientación | Diferencia entre las poses alineadas, cada ciclo | — |
| RTT (ida y vuelta) | `twin_bridge` publica `/digital_twin/ping`, `latency_echo` en la Raspberry Pi lo devuelve | Ninguno (un solo reloj) |
| Edad de la odometría real | Hora del PC − sello de tiempo del mensaje | Relojes sincronizados (`chrony`) |
| Retardo de sincronización | Desfase que maximiza la correlación entre la velocidad del líder y la del seguidor (análisis offline) | — |

Todo queda en `~/twin_logs/twin_<fecha>_<tag>.csv` (+ `.json` con los parámetros usados). Se publica también en vivo en `/digital_twin/status` (`bumperbot_msgs/TwinSyncStatus`) en ambos dominios.

## 4. Uso

Robot físico (Raspberry Pi):

```bash
ros2 launch bumperbot_digital_twin real_side.launch.py real_domain:=10
```

PC:

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_domain:=10 twin_domain:=20
```

Sin hardware, todo en el PC:

```bash
ros2 launch bumperbot_digital_twin digital_twin.launch.py real_mode:=fake driver:=square
```

Robot real completo sin Arduino (ros2_control con hardware simulado), en otra terminal del mismo PC:

```bash
ros2 launch bumperbot_digital_twin real_side.launch.py use_mock_hardware:=true
```

Recorridos automáticos para las pruebas (`driver:=`): `line`, `line_back`, `square`, `circle`, `rotate`, `figure8`, con `linear_speed`, `angular_speed`, `distance`.

Servicios (en cualquiera de los dos dominios):

```bash
ros2 service call /digital_twin/align std_srvs/srv/Trigger
ros2 service call /digital_twin/switch_leader std_srvs/srv/Trigger
ros2 service call /digital_twin/set_feedback std_srvs/srv/SetBool "{data: false}"
ros2 service call /digital_twin/new_log std_srvs/srv/Trigger
```

Análisis:

```bash
ros2 run bumperbot_digital_twin analyze_twin_log ~/twin_logs/*.csv -o resultados
```

Genera `resumen.md` (tabla lista para la tesis), `resumen.json` y, por cada experimento, gráficas de trayectorias, error, velocidades y latencia.

## 5. Parámetros principales (`config/twin_bridge.yaml`)

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
| `odom_topic` | `/bumperbot_controller/odom` | Odometría que se compara (`/odometry/filtered` para usar el EKF) |

## 6. Pruebas automáticas

`.github/workflows/build.yml` compila todo en ROS 2 Humble y ejecuta:

1. Pruebas unitarias de la geometría, la ley de control, los recorridos y el análisis.
2. El puente con dos robots de mentira en dos dominios: líder real con y sin realimentación, y líder gemelo.
3. El robot real completo con hardware simulado (ros2_control mock) + EKF + Nav2, y el gemelo siguiéndolo.
4. El gemelo en Gazebo sin ventana (experimental: depende de que el servidor de GitHub pueda renderizar el LiDAR).

Resultados de la corrida del 27 de septiembre de 2026 (todas con realimentación salvo la indicada):

| Prueba | Recorrido | Error medio | Error máximo | Error final | RTT medio |
|---|---|---|---|---|---|
| Robots de mentira, líder real | cuadrado 0,6 m | 1,0 cm | 2,2 cm | 0,8 cm | 1,7 ms |
| Robots de mentira, líder real, sin realimentación | cuadrado 0,6 m | 15,2 cm | 23,9 cm | 23,7 cm | 0,6 ms |
| Robots de mentira, líder gemelo | cuadrado 0,6 m | 1,0 cm | 2,6 cm | 0,9 cm | 0,6 ms |
| Robot real con mock hardware + EKF + Nav2, gemelo de mentira | ida y vuelta 0,8 m | 1,0 cm | 1,8 cm | 1,6 cm | 0,5 ms |
| **Gemelo en Gazebo** siguiendo a un robot real de mentira | cuadrado 0,8 m | 0,5 cm | 3,1 cm | 0,7 cm | 1,2 ms |

La latencia aquí es local (todo en una máquina); con la Raspberry Pi por Wi-Fi será mayor y es lo que mide la Prueba 5.

## 7. Limitaciones conocidas

- La alineación inicial supone que ambos robots arrancan en el mismo punto. Si no, hay que llamar a `/digital_twin/align` con los robots en posiciones equivalentes.
- Se comparan odometrías, que también derivan. Para comparar posiciones absolutas, ambos deben localizarse en el mismo mapa (AMCL) y usar `/amcl_pose`; queda como mejora.
- Con el robot parado no se puede corregir el error lateral (restricción no holonómica del robot diferencial); se corrige en cuanto vuelve a avanzar.
