from svglib import S
s=S(1460,1420)
F=27; SB=25
src=[(20,["Joystick","joy_vel, prioridad 99"],"white"),(380,["twin_bridge","twin_vel, prioridad 95"],"twin"),(740,["Teclado","key_vel, prioridad 90"],"white"),(1100,["Nav2","cmd_vel, prioridad 80"],"nav")]
for x,l,k in src: s.box(x,20,340,95,l,k,size=F,sub=SB)
s.o.append('<path d="M190,115 L190,170 L1270,170 L1270,115 M550,115 L550,170 M910,115 L910,170" fill="none" stroke="#424242" stroke-width="3"/>')
s.arrow([(470,170),(470,220)])
s.box(20,220,900,110,["twist_mux","pasa el comando de mayor prioridad","que siga vigente (0,5 s)"],"ctl",size=F,sub=SB)
s.box(990,220,450,110,["Bloqueos (255)","safety_stop · anomaly_stop"],"white",size=F,sub=SB)
s.arrow([(990,275),(920,275)])
s.arrow([(470,330),(470,385)])
s.box(20,385,900,100,["twist_relay","Twist → TwistStamped"],"ctl",size=F,sub=SB)
s.box(990,385,450,130,["EKF","odometría + IMU →","/odometry/filtered"],"ctl",size=F,sub=SB)
s.arrow([(470,485),(470,545)])
s.box(20,545,1420,105,["diff_drive_controller (ros2_control, 100 Hz)","v y ω → velocidad de cada rueda, en rad/s"],"ctl",size=F,sub=SB)
s.arrow([(1215,545),(1215,515)]); s.text(1228,537,"odometría",22,"start",italic=True)
# columns
s.text(40,745,"Robot físico",30,"start",bold=True); s.text(780,745,"Gemelo digital",30,"start",bold=True)
for cx in [(330,560),(1090,1300)]:
    a,b=cx
    s.arrow([(a,650),(a,770)]); s.text(a-12,715,"consigna",22,"end",italic=True)
    s.arrow([(b,770),(b,650)]); s.text(b+12,715,"estado",22,"start",italic=True)
s.box(40,770,640,130,["BumperbotInterface","envía «rp05.20,ln03.10,»","recibe «rp4.55,ln0.12,rc…,lc…,t…,»"],"ctl",size=F,sub=24)
s.arrow([(445,900),(445,960)],both=True); s.text(457,938,"USB, 115 200 baudios",22,"start",italic=True)
s.box(40,960,640,105,["Arduino Nano","PID por rueda cada 100 ms","PWM hasta 150 de 255"],"hw",size=F,sub=24)
s.arrow([(250,1065),(250,1125)])
s.box(40,1125,640,95,["Puente H L298N","energía de la batería"],"hw",size=F,sub=SB)
s.arrow([(250,1220),(250,1280)])
s.box(40,1280,640,95,["Motores JGA25-371","con encoders Hall"],"hw",size=F,sub=SB)
s.arrow([(680,1327),(720,1327),(720,1012),(680,1012)]); s.text(735,1160,"cuentas de los encoders",22,"start",italic=True)
s.box(780,770,660,130,["ign_ros2_control","complemento de ros2_control","en Gazebo"],"sim",size=F,sub=SB)
s.arrow([(1195,900),(1195,960)],both=True)
s.box(780,960,660,105,["Gazebo Fortress","física de las ruedas y del suelo"],"sim",size=F,sub=SB)
s.save("camino.svg")
