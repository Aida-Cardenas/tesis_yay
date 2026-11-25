from svglib import S
s=S(2600,1450)
s.panel(30,30,560,1230,"Dominio 10","robot físico (Raspberry Pi)")
s.panel(2010,30,560,1230,"Dominio 20","gemelo digital (Gazebo)")
s.panel(640,30,1320,1230,"Proceso twin_bridge","en el computador")
# domain topics
L=[(150,["Odometría de ruedas","/bumperbot_controller/odom"],"ctl"),(290,["Comando aplicado","/bumperbot_controller/cmd_vel"],"ctl"),(430,["LiDAR","/scan"],"ctl"),(570,["Entrada del seguidor","twin_vel → twist_mux (95)"],"ctl"),(760,["latency_echo","/digital_twin/ping → pong"],"twin"),(900,["anomaly_detector","/digital_twin/anomalies"],"twin")]
R=[(150,["Odometría de ruedas","/bumperbot_controller/odom"],"ctl"),(290,["Comando aplicado","/bumperbot_controller/cmd_vel"],"ctl"),(430,["LiDAR simulado","/scan"],"sim"),(570,["Entrada del seguidor","twin_vel → twist_mux (95)"],"ctl"),(760,["LiDAR real reenviado","/digital_twin/real_scan"],"sim"),(900,["Estado y trayectorias","/digital_twin/status, …_path"],"twin")]
for y,l,k in L: s.box(70,y,480,100,l,k,size=23)
for y,l,k in R: s.box(2050,y,480,100,l,k,size=23)
# contexts
s.box(690,150,380,520,["Contexto ROS 2","dominio 10","","suscripciones","y publicadores","del robot físico","","ejecutor en","su propio hilo"],"white",size=24)
s.box(1530,150,380,520,["Contexto ROS 2","dominio 20","","suscripciones","y publicadores","del gemelo","","ejecutor en","su propio hilo"],"white",size=24)
s.box(1110,150,380,190,["Estado compartido","de los dos robots","protegido con un cerrojo"],"twin",size=24)
for y in [200,340,480]:
    s.arrow([(550,y),(690,y)]); s.arrow([(2050,y),(1910,y)])
s.arrow([(690,620),(550,620)],"#E65100",4); s.arrow([(1910,620),(2050,620)],"#E65100",4)
s.arrow([(550,810),(690,640)],both=True); s.arrow([(550,950),(690,655)])
s.arrow([(1910,640),(2050,810)]); s.arrow([(1910,655),(2050,950)])
s.arrow([(1070,250),(1110,250)],both=True); s.arrow([(1530,250),(1490,250)],both=True)
s.arrow([(1300,340),(1300,400)],both=True)
# cycle
s.box(1110,400,380,610,["Ciclo a 20 Hz","(reloj de pared)","","1. Alinear las odometrías","2. Prealimentación:","comando del líder","3. Realimentación:","ley de Kanayama","4. Compensación de latencia:","predicción y predictor de Smith","5. Seguridad: datos viejos","o error mayor a 0,75 m","6. twin_vel al seguidor"],"twin",size=23)
s.arrow([(1110,940),(880,940),(880,670)],"#E65100",4); s.arrow([(1490,940),(1720,940),(1720,670)],"#E65100",4)
s.text(880,975,"si el líder es el gemelo",20,color="#BF360C",italic=True); s.text(1720,975,"si el líder es el robot",20,color="#BF360C",italic=True)
# services
s.box(690,1060,1220,150,["Servicios /digital_twin/…","align · set_feedback · switch_leader · configure · new_log","Registro de cada corrida en CSV (~/twin_logs)"],"white",size=24)
s.legend(200,1350,[("ctl","Tópicos del software base"),("twin","Capa bidireccional"),("sim","Gemelo y comparación de LiDAR")],cols=3,colw=780)
s.save("gemelo.svg")
