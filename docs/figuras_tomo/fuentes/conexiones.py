W,H=2760,1600
C=dict(bat="#C62828",gnd="#212121",v5="#EF6C00",mot="#6D4C41",ctl="#1565C0",enc="#2E7D32",i2c="#00838F",usb="#6A1B9A")
o=[]
def box(x,y,w,h,title,sub=None,fill="#F7F7F7"):
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{fill}" stroke="#424242" stroke-width="3"/>')
    o.append(f'<text x="{x+w/2}" y="{y+42}" class="t">{title}</text>')
    if sub: o.append(f'<text x="{x+w/2}" y="{y+76}" class="s">{sub}</text>')
def pin(x,y,label,side):
    o.append(f'<circle cx="{x}" cy="{y}" r="6" fill="#fff" stroke="#424242" stroke-width="2.5"/>')
    if side=='l': o.append(f'<text x="{x+14}" y="{y+8}" class="p" text-anchor="start">{label}</text>')
    if side=='r': o.append(f'<text x="{x-14}" y="{y+8}" class="p" text-anchor="end">{label}</text>')
    if side=='b': o.append(f'<text x="{x}" y="{y-16}" class="p" text-anchor="middle">{label}</text>')
    if side=='t': o.append(f'<text x="{x+12}" y="{y-14}" class="p" text-anchor="start">{label}</text>')
    if side=='bs': o.append(f'<text x="{x}" y="{y-16}" class="ps" text-anchor="middle">{label}</text>')
def wire(pts,color,w=4):
    d="M"+" L".join(f"{x},{y}" for x,y in pts)
    o.insert(0,f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linejoin="round"/>')
def dot(x,y,color): o.append(f'<circle cx="{x}" cy="{y}" r="8" fill="{color}"/>')
def note(x,y,t,cls="n",anchor="start"): o.append(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}">{t}</text>')

# Battery, fuse, switch
box(60,520,300,200,"Batería LiPo 3S","11,1 V · 3300 mAh")
pin(360,600,"+","r"); pin(360,670,"−","r")
box(400,572,160,56,"",fill="#FFF"); note(480,609,"Fusible 7,5 A","s2","middle")
box(575,572,150,56,"",fill="#FFF"); note(650,609,"Interruptor","s2","middle")
wire([(360,600),(400,600)],C["bat"],6); wire([(560,600),(575,600)],C["bat"],6)
# + bus
wire([(725,600),(740,600)],C["bat"],6)
wire([(740,330),(740,780)],C["bat"],6); wire([(740,330),(820,330)],C["bat"],6); wire([(740,780),(820,780)],C["bat"],6); dot(740,600,C["bat"])
# GND bus
wire([(360,670),(770,670)],C["gnd"],6); wire([(770,370),(770,830)],C["gnd"],6); wire([(770,370),(820,370)],C["gnd"],6); wire([(770,830),(820,830)],C["gnd"],6); dot(770,670,C["gnd"])

# Regulator
box(820,200,340,220,"Regulador LM2596","reductor, ajustado a 5,1 V")
pin(820,330,"IN+","l"); pin(820,370,"IN−","l"); pin(1160,330,"OUT+","r"); pin(1160,370,"OUT−","r")
# L298N
box(820,720,340,440,"Puente H L298N")
pin(820,780,"+12V","l"); pin(820,830,"GND","l"); pin(820,880,"+5V","l")
note(830,935,"puente «5V EN» retirado","n2")
for y,l in zip([800,840,880,920,960,1000,1060],["ENA","IN1","IN2","IN3","IN4","ENB","GND"]): pin(1160,y,l,"r")
for x,l in zip([860,935,1025,1100],["OUT1","OUT2","OUT3","OUT4"]): pin(x,1160,l,"bs")
# Arduino
box(1320,650,330,540,"Arduino Nano","ATmega328P · CH340")
for y,l in zip([800,840,880,920,960,1000,1060],["D9","D12","D13","D7","D8","D11","GND"]): pin(1320,y,l,"l")
for y,(a,b) in zip([800,840,880,920,960,1000,1060],[("ENA","D9")]*7): pass
for y in [800,840,880,920,960,1000]: wire([(1160,y),(1320,y)],C["ctl"],4)
wire([(1160,1060),(1320,1060)],C["gnd"],4)
note(1240,780,"PWM y sentido","n3","middle")
bx=[1360,1410,1460,1510,1560,1610]; bl=["D2","D3","D4","D5","5V","GND"]
for x,l in zip(bx,bl): pin(x,1190,l,"b")
pin(1650,760,"USB","r")
# 5V reg -> L298N +5V and Pi
wire([(1160,330),(1820,330)],C["v5"],5); wire([(1160,370),(1820,370)],C["gnd"],5)
wire([(1200,330),(1200,470),(795,470),(795,880),(820,880)],C["v5"],4); dot(1200,330,C["v5"])
note(1490,318,"cable USB-C, 5,1 V","n3","middle")
# Raspberry Pi
box(1820,200,430,480,"Raspberry Pi 4","4 GB · Ubuntu 22.04 · ROS 2")
pin(1820,330,"USB-C +","l"); pin(1820,370,"USB-C −","l")
for y,l in zip([360,410,460,510],["pin 1 · 3,3 V","pin 3 · SDA","pin 5 · SCL","pin 6 · GND"]): pin(2250,y,l,"r")
pin(1930,680,"USB","b"); pin(2110,680,"USB","b")
wire([(1650,760),(1930,760),(1930,680)],C["usb"],5)
note(1790,748,"serie, 115 200 baudios","n3","middle")
# IMU
box(2400,250,260,300,"IMU","MPU6050")
for y,l in zip([360,410,460,510],["VCC","SDA","SCL","GND"]): pin(2400,y,l,"l")
wire([(2250,360),(2400,360)],C["v5"],4)
wire([(2250,410),(2400,410)],C["i2c"],4); wire([(2250,460),(2400,460)],C["i2c"],4); wire([(2250,510),(2400,510)],C["gnd"],4)
note(2325,440,"I²C","n3","middle")
# LiDAR
box(1820,880,430,200,"LiDAR RPLidar A1M8","con su adaptador USB")
pin(2110,880,"USB","t")
wire([(2110,680),(2110,880)],C["usb"],5)
# Motors
box(560,1300,420,150,"Motor derecho","JGA25-371 · 6 V · encoder Hall")
box(1000,1300,300,150,"Motor izquierdo","JGA25-371 · 6 V")
for x,l in zip([860,935],["M1","M2"]): pin(x,1300,l,"t")
for x,l in zip([1025,1100],["M1","M2"]): pin(x,1300,l,"t")
for x in [860,935,1025,1100]: wire([(x,1160),(x,1300)],C["mot"],5)
mdp=[(610,"C1"),(670,"C2"),(735,"VCC"),(805,"GND")]; mip=[(1030,"C1"),(1090,"C2"),(1155,"VCC"),(1225,"GND")]
for x,l in mdp+mip: pin(x,1450,l,"bs")
routes=[(1030,1360,C["enc"]),(610,1410,C["enc"]),(1090,1460,C["enc"]),(670,1510,C["enc"]),(1155,1560,C["v5"]),(735,1560,C["v5"]),(1225,1610,C["gnd"]),(805,1610,C["gnd"])]
ys=[1490,1515,1540,1565,1590,1615,1640,1665]
for (sx,ex,col),y in zip(routes,ys): wire([(sx,1450),(sx,y),(ex,y),(ex,1190)],col,4)
dot(1560,1590,C["v5"]); dot(1610,1640,C["gnd"])
note(1720,1300,"Encoder derecho: C1 → D3, C2 → D5","n4"); note(1720,1340,"Encoder izquierdo: C1 → D2, C2 → D4","n4")
note(1720,1380,"VCC de los dos encoders → 5V; GND → GND","n4")
# legend
lx,ly=1720,1450
items=[("bat","Batería, 11,1 V"),("gnd","Tierra (GND)"),("v5","5 V y 3,3 V"),("mot","Potencia de los motores"),("ctl","PWM y sentido de giro"),("enc","Fases A y B de los encoders"),("i2c","Bus I²C"),("usb","USB")]
for i,(k,t) in enumerate(items):
    xx=lx+(i//4)*480; yy=ly+(i%4)*48
    o.append(f'<line x1="{xx}" y1="{yy}" x2="{xx+60}" y2="{yy}" stroke="{C[k]}" stroke-width="7"/>'); note(xx+76,yy+9,t,"n4")
svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<style>text{{font-family:"Liberation Sans",Arial,sans-serif;fill:#212121}} .t{{font-size:34px;font-weight:bold;text-anchor:middle}} .s{{font-size:25px;text-anchor:middle;fill:#555}} .s2{{font-size:24px;font-weight:bold}} .p{{font-size:24px}} .ps{{font-size:21px}} .n{{font-size:24px}} .n2{{font-size:22px;fill:#666;font-style:italic}} .n3{{font-size:23px;fill:#444;font-style:italic}} .n4{{font-size:27px}}</style>
<rect width="{W}" height="{H}" fill="#fff"/>
<g transform="translate(0,-130)">{"".join(o)}</g></svg>'''
open("conexiones.svg","w").write(svg)
