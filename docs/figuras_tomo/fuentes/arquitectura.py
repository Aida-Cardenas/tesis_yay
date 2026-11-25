from svglib import S
s=S(2640,1480)
L,R,M=40,1640,1050
s.panel(L,30,960,1230,"Robot físico","Raspberry Pi 4 · ROS_DOMAIN_ID = 10")
s.panel(R,30,960,1230,"Gemelo digital","computador (Ubuntu en WSL) · ROS_DOMAIN_ID = 20")
s.panel(M,30,540,1230,"Capa bidireccional","computador · dominios 10 y 20")
for X,side in [(L,'r'),(R,'t')]:
    cx=X+180; w=600
    s.box(cx,150,w,100,["Localización y navegación","slam_toolbox o AMCL · Nav2"],"nav")
    s.box(cx,330,w,100,["Fusión sensorial","EKF (robot_localization)"],"ctl")
    s.box(cx,510,w,120,["Control","twist_mux · twist_relay","ros2_control · diff_drive_controller (100 Hz)"],"ctl")
    s.arrow([(cx+200,430),(cx+200,510)][::-1]); s.text(cx+190,478,"odometría",21,"end",italic=True)
    s.arrow([(cx+300,330),(cx+300,250)]); s.text(cx+312,298,"pose",21,"start",italic=True)
    s.arrow([(cx+w-40,250),(cx+w-40,510)]); s.text(cx+w-52,298,"cmd_vel",21,"end",italic=True)
# robot físico: hardware
cx=L+180
s.box(cx,700,600,100,["BumperbotInterface","puerto serie USB, 115 200 baudios"],"ctl")
s.arrow([(cx+300,630),(cx+300,700)],both=True)
s.box(L+60,870,400,130,["Arduino Nano","PID por rueda · L298N","motores y encoders"],"hw")
s.box(L+500,870,200,130,["IMU","MPU6050 · I²C"],"hw")
s.box(L+730,870,200,130,["LiDAR","A1M8 · USB"],"hw")
s.arrow([(L+260,800),(L+260,870)],both=True)
s.arrow([(L+600,870),(L+600,820),(L+120,820),(L+120,380),(cx,380)]); s.text(L+108,470,"/imu/out",21,"end",italic=True)
s.arrow([(L+830,870),(L+830,840),(L+900,840),(L+900,200),(cx+600,200)]); s.text(L+888,760,"/scan",21,"end",italic=True)
s.box(L+180,1080,600,100,["Apoyo del gemelo en el robot","latency_echo · anomaly_detector"],"twin")
# gemelo: simulación
cx=R+180
s.box(cx,700,600,100,["ign_ros2_control","complemento de ros2_control en Gazebo"],"sim")
s.arrow([(cx+300,630),(cx+300,700)],both=True)
s.box(R+60,870,420,130,["Gazebo Fortress","recinto de 3,0 × 2,0 m","ruedas, IMU y LiDAR simulados"],"sim")
s.box(R+520,870,380,130,["Modelo virtual","URDF/Xacro","mismas medidas que el robot"],"model")
s.arrow([(R+520,935),(R+480,935)])
s.arrow([(R+270,870),(R+270,800)],both=True)
s.arrow([(R+120,870),(R+120,380),(cx,380)]); s.text(R+108,470,"IMU",21,"end",italic=True)
s.arrow([(R+440,870),(R+440,840),(R+900,840),(R+900,200),(cx+600,200)]); s.text(R+888,760,"LiDAR",21,"end",italic=True)
s.box(R+180,1080,600,100,["Interfaz","RViz · panel twin_dashboard · experimentos"],"twin")
# bridge
s.box(M+50,380,440,420,["twin_bridge","un contexto de ROS 2 por dominio","ciclo de sincronización a 20 Hz","líder y seguidor","corrección de seguimiento","compensación de latencia","comparación de los LiDAR"],"twin",size=26)
s.arrow([(L+780,545),(M+50,545)],"#E65100",4); s.text((L+780+M+50)/2,530,"odometría, comandos, LiDAR",20,color="#BF360C")
s.arrow([(M+50,600),(L+780,600)],"#E65100",4); s.text((L+780+M+50)/2,630,"twin_vel (si sigue)",20,color="#BF360C")
s.arrow([(R+180,545),(M+490,545)],"#E65100",4); s.text((R+180+M+490)/2,530,"odometría, comandos, LiDAR",20,color="#BF360C")
s.arrow([(M+490,600),(R+180,600)],"#E65100",4); s.text((R+180+M+490)/2,630,"twin_vel (si sigue)",20,color="#BF360C")
s.arrow([(L+780,1130),(M+170,1130),(M+170,800)],"#E65100",4,both=True); s.text((L+780+M+170)/2,1115,"ping, pong y anomalías",20,color="#BF360C")
s.arrow([(R+180,1130),(M+370,1130),(M+370,800)],"#E65100",4,both=True); s.text((M+370+R+180)/2,1115,"estado y métricas",20,color="#BF360C")
s.text(M+270,880,"DDS sobre Wi-Fi",22,color="#555",italic=True); s.text(M+270,910,"(descubrimiento por unicast)",20,color="#555",italic=True)
s.legend(150,1330,[("model","Modelo virtual"),("sim","Entorno de simulación"),("ctl","Control, odometría y fusión sensorial"),("twin","Capa de comunicación bidireccional"),("nav","Localización y navegación"),("hw","Hardware del robot")],cols=3,colw=800)
s.save("arquitectura.svg")
