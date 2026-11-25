from svglib import S
s=S(1460,1310)
F=26; SB=24
s.panel(15,15,380,1180,"Dominio 10","robot físico")
s.panel(1065,15,380,1180,"Dominio 20","gemelo digital")
s.panel(415,15,630,1180,"twin_bridge","un solo proceso en el computador")
L=[(130,["Odometría","…/odom"],"ctl"),(255,["Comando aplicado","…/cmd_vel"],"ctl"),(380,["LiDAR","/scan"],"ctl"),(505,["Entrada del seguidor","twin_vel"],"ctl"),(680,["latency_echo","ping → pong"],"twin"),(805,["anomaly_detector","anomalías"],"twin")]
R=[(130,["Odometría","…/odom"],"ctl"),(255,["Comando aplicado","…/cmd_vel"],"ctl"),(380,["LiDAR simulado","/scan"],"sim"),(505,["Entrada del seguidor","twin_vel"],"ctl"),(680,["LiDAR real reenviado","…/real_scan"],"sim"),(805,["Estado y trayectorias","…/status, …_path"],"twin")]
for y,l,k in L: s.box(40,y,330,95,l,k,size=F,sub=SB)
for y,l,k in R: s.box(1090,y,330,95,l,k,size=F,sub=SB)
s.text(205,955,"…/ = /bumperbot_controller/",21,color="#555",italic=True); s.text(205,983,"o /digital_twin/",21,color="#555",italic=True)
s.box(440,130,280,470,["Contexto","dominio 10","","suscriptores","y publicadores","","ejecutor en","su hilo"],"white",size=F,sub=SB)
s.box(740,130,280,470,["Contexto","dominio 20","","suscriptores","y publicadores","","ejecutor en","su hilo"],"white",size=F,sub=SB)
for y in [177,302,427]:
    s.arrow([(370,y),(440,y)]); s.arrow([(1090,y),(1020,y)])
s.arrow([(440,552),(370,552)],"#E65100",4); s.arrow([(1020,552),(1090,552)],"#E65100",4)
s.arrow([(370,727),(400,727),(400,585),(440,585)],both=True); s.arrow([(370,852),(410,852),(410,595),(440,595)])
s.arrow([(1020,585),(1060,585),(1060,727),(1090,727)]); s.arrow([(1020,595),(1050,595),(1050,852),(1090,852)])
s.box(440,640,580,90,["Estado compartido de los dos robots","protegido con un cerrojo"],"twin",size=F,sub=SB)
s.arrow([(580,600),(580,640)],both=True); s.arrow([(880,600),(880,640)],both=True)
s.box(440,770,580,330,["Ciclo de sincronización a 20 Hz","1. alinea las odometrías","2. prealimentación: comando del líder","3. realimentación: ley de Kanayama","4. compensación de latencia","5. seguridad: datos viejos o error > 0,75 m","6. publica twin_vel para el seguidor"],"twin",size=F,sub=SB,left=30)
s.arrow([(730,730),(730,770)],both=True)
s.box(440,1120,580,0,[" "],"white")
s.text(730,1135,"Servicios: align · set_feedback · switch_leader · configure",22,color="#444")
s.text(730,1165,"Registro de cada corrida en CSV",22,color="#444")
s.legend(60,1260,[("ctl","Tópicos del software base"),("twin","Capa bidireccional"),("sim","Gemelo y LiDAR")],cols=3,colw=470)
s.save("gemelo.svg")
