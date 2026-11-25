from svglib import S
s=S(2400,1400)
# sources
src=[(60,["Joystick","joy_vel · prioridad 99"],"white"),(520,["twin_bridge","twin_vel · prioridad 95"],"twin"),(980,["Teclado","key_vel · prioridad 90"],"white"),(1440,["Nav2","cmd_vel · prioridad 80"],"nav")]
for x,l,k in src:
    s.box(x,40,400,100,l,k)
s.box(1900,40,440,100,["Bloqueos","safety_stop · anomaly_stop (255)"],"white")
s.o.append('<path d="M260,140 L260,190 L1640,190 L1640,140" fill="none" stroke="#424242" stroke-width="3"/>')
for x in [720,1180]: s.o.append(f'<path d="M{x},140 L{x},190" fill="none" stroke="#424242" stroke-width="3"/>')
s.arrow([(950,190),(950,240)]); s.arrow([(2120,140),(2120,290),(1500,290)]); s.text(2110,280,"detienen el robot",20,"end",italic=True)
s.box(400,240,1100,100,["twist_mux","pasa el comando de mayor prioridad que siga vigente (0,5 s)"],"ctl")
s.arrow([(950,340),(950,390)])
s.box(400,390,1100,100,["twist_relay","Twist → TwistStamped, en /bumperbot_controller/cmd_vel"],"ctl")
s.arrow([(950,490),(950,540)])
s.box(400,540,1100,110,["diff_drive_controller (ros2_control, 100 Hz)","v y ω → velocidad de cada rueda, en rad/s"],"ctl")
s.box(1700,540,640,110,["EKF (robot_localization)","/odometry/filtered · TF odom → base_footprint"],"ctl")
s.arrow([(1500,595),(1700,595)]); s.text(1600,580,"odometría",20,italic=True)
s.arrow([(2020,740),(2020,650)]); s.box(1880,740,280,80,["IMU","/imu/out (real o simulada)"],"hw",size=22)
# split
s.arrow([(700,650),(700,760)]); s.text(690,710,"consigna",20,"end",italic=True)
s.arrow([(1250,650),(1250,760)]); 
s.text(380,742,"Robot físico",28,"start",bold=True); s.text(1280,742,"Gemelo digital",28,"start",bold=True)
s.box(380,760,640,110,["BumperbotInterface","envía «rp05.20,ln03.10,» por USB, 115 200 baudios"],"ctl")
s.arrow([(600,870),(600,940)]); 
s.box(380,940,640,110,["Arduino Nano","PID por rueda cada 100 ms · PWM ≤ 150 de 255"],"hw")
s.arrow([(600,1050),(600,1120)])
s.box(380,1120,640,100,["Puente H L298N","alimenta los motores con la batería"],"hw")
s.arrow([(600,1220),(600,1290)])
s.box(380,1290,640,90,["Motores JGA25-371 y encoders"],"hw",size=24)
s.arrow([(1020,1335),(1080,1335),(1080,995),(1020,995)]); s.text(1092,1170,"cuentas del",20,"start",italic=True); s.text(1092,1196,"encoder",20,"start",italic=True)
s.arrow([(800,940),(800,870)]); s.text(812,912,"«rp4.55,ln0.12,rc…,lc…,t…,»",19,"start",italic=True)
s.box(1180,760,640,110,["ign_ros2_control","complemento de ros2_control en Gazebo"],"sim")
s.arrow([(1400,870),(1400,940)])
s.box(1180,940,640,110,["Gazebo Fortress","física de las ruedas y del contacto con el suelo"],"sim")
s.arrow([(1600,940),(1600,870)])
# feedback to diff drive
s.arrow([(380,815),(330,815),(330,595),(400,595)]); s.text(318,700,"estado de",20,"end",italic=True); s.text(318,726,"las ruedas",20,"end",italic=True)
s.arrow([(1820,815),(1860,815),(1860,700),(1450,700),(1450,650)]); s.text(1655,690,"estado de las ruedas",20,italic=True)
s.legend(240,1340,[],cols=1)
s.save("camino.svg")
