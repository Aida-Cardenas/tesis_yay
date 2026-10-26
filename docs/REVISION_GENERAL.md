# Revisión general del código antes de cargarlo en el robot

Revisión propia del 26 de octubre de 2026, hecha después de la cuarta revisión del tutor y antes de cargar el código en la Raspberry Pi y el Arduino. Se buscaron errores que puedan aparecer al correr las pruebas, problemas de seguridad y código que sobra. No se quitó ni se cambió nada de lo que pidieron las revisiones anteriores ([1](RESPUESTA_REVISION.md), [2](RESPUESTA_REVISION_2.md), [3](RESPUESTA_REVISION_3.md), [4](RESPUESTA_REVISION_4.md)). Ningún cambio toca la ley de seguimiento, el modelo, la calibración ni la estadística, así que los resultados de CI no cambian.

## Despliegue en la Raspberry Pi y el Arduino

| Problema | Qué podía pasar | Cambio |
|---|---|---|
| La interfaz mandaba comandos mientras el Arduino arrancaba | Abrir el puerto reinicia el Arduino; los bytes que llegan durante el bootloader pueden dejar un Nano con bootloader viejo esperando una carga de firmware en vez de correr el sketch | No se envía nada hasta la primera línea del Arduino |
| Al reconectar el cable no se esperaba al Arduino | Igual que el anterior, después de un cable suelto | La reconexión vuelve a esperar la primera línea y conserva la posición de las ruedas, para que la odometría no salte (`WheelIntegrator::forget_reading`, gtest) |
| `scan_mode: Sensitivity` fijo para el RPLidar A1 | Un A1 que no tiene ese modo cierra el driver con "Can not start scan" | Modo típico del sensor (`scan_mode: ""`) |
| El driver del LiDAR termina si `/dev/rplidar` todavía no existe | Sin LiDAR durante toda la sesión | Se reinicia cada 3 s; `use_lidar:=false` para probar sin el sensor |
| La IMU reintentaba la conexión I2C 100 veces por segundo | Bus I2C saturado y sin explicación del fallo | Reintento cada segundo, cerrando el bus anterior, con un mensaje que dice qué revisar |
| Nano con chip FTDI y UNO original sin regla udev | Sin `/dev/arduino` | Reglas agregadas |

## Errores y riesgos corregidos en esta revisión

| Problema | Qué podía pasar | Cambio | Comprobación |
|---|---|---|---|
| Un comando no finito se enviaba al Arduino como `nan` | El sketch lee la `n` como "invertir sentido" y la rueda cambia de dirección | Se envía 0, y la magnitud se limita a 99,99 para que siempre quepa en los 5 caracteres que lee el sketch | gtest `CommandIsAlwaysReadableByTheSketch` |
| `safety_stop` tomaba como obstáculo cualquier lectura finita | Una lectura menor que `range_min`, como el LiDAR viendo el propio chasis, deja el robot detenido para siempre | Solo cuentan lecturas dentro de `[range_min, range_max]` | Compila en CI |
| La etiqueta del registro llega por un servicio y se usaba tal cual en el nombre del archivo | Una etiqueta como `../../x` escribe archivos fuera de `log_dir` | `safe_log_tag`: solo letras, números, `.`, `-` y `_` | `test_log_tags_cannot_leave_the_log_folder` |
| Ctrl-C en `twin_navigate` no cancelaba la meta de Nav2 | El robot real seguía navegando solo después de interrumpir | Ctrl-C cancela la meta antes de salir | `test_ctrl_c_cancels_the_nav2_goal` |
| La vista previa de una meta no ejecutada dejaba abierto su registro | El puente seguía escribiendo en `<meta>_vista` hasta el siguiente registro | Se cierra también cuando la meta no se ejecuta | — |
| `--resume` aceptaba la carpeta de una sesión de otro protocolo o con otras repeticiones | Orden de corridas distinto del guardado y corridas mezcladas | Rechaza otro protocolo y reutiliza las repeticiones de la sesión; la sesión se valida antes de unirse a los dominios | `test_resume_refuses_a_session_of_another_protocol` |
| Ctrl-C a mitad de un recorrido no mandaba velocidad cero | El robot seguía hasta que vencía la entrada de `twist_mux` (0,5 s) | `TrajectoryPlayer.stop()` y `scripted_driver` publican cero | — |
| Los nodos mostraban trazas y "process has died" al cerrar el launch | Errores falsos en los registros de CI y del robot | Un solo `spin_node` para todos los nodos | Registros de CI |
| `resumen.json` e `informe.json` con `NaN` | JSON inválido para otros programas | `json_safe` también en esos archivos | `test_json_safe_handles_numpy_values` |
| Permisos de CI por defecto y puertos serie `0666` | El token de CI podía escribir en el repositorio; cualquier usuario de la Pi podía mover el robot | CI con `contents: read`; puertos `0660` para el grupo `dialout` | — |

Además se quitó código que nada usaba: constantes de registros de la IMU, `last_values` del detector, una rama imposible de `trajectories`, la variable `is_cmd_complete` del sketch, imports sin uso, y las copias duplicadas de `require_positive` y de la búsqueda de protocolos.

## Lo que no se cambió y hay que tener en cuenta

- **La red no tiene autenticación.** DDS no pide permiso: cualquier computador en la misma red y con el mismo `ROS_DOMAIN_ID` puede mandar velocidades al robot o reconfigurar el puente. Para las pruebas conviene un router propio, sin otros equipos. SROS2 (DDS con certificados) queda fuera del alcance de la tesis.
- **El joystick es la parada de emergencia.** Tiene la prioridad más alta en `twist_mux` (99) y necesita el botón 5 apretado (deadman). Hay que tenerlo en la mano durante las pruebas físicas.
- **El bloqueo por atasco es de falla segura.** Si el detector se cae con el bloqueo puesto, el robot queda detenido hasta reiniciar el detector o publicar `false` en `/anomaly_stop`.
- **Arranque del Arduino.** Mientras arranca, sus pines quedan sin configurar y el LED del pin 13 (IN2 del motor derecho) parpadea. Si una rueda da un tirón al encender, una resistencia de 10 kΩ de ENA a GND y otra de ENB a GND lo evitan.
- **Alimentación.** Si la Raspberry Pi y los motores comparten batería sin un regulador propio para la Pi, un arranque brusco de los motores puede reiniciarla. La Pi necesita su regulador de 5 V (3 A) y tierra común con el Arduino y el L298N.
