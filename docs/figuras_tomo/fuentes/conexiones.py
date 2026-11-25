W,H=1460,1800
C=dict(bat="#C62828",gnd="#212121",v5="#EF6C00",mot="#6D4C41",ctl="#1565C0",enc="#2E7D32",i2c="#00838F",usb="#6A1B9A")
o=[]; back=[]
def box(x,y,w,h,title,sub=None,fill="#F7F7F7",ty=None):
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{fill}" stroke="#424242" stroke-width="3"/>')
    ty=ty if ty is not None else y+40
    o.append(f'<text x="{x+w/2}" y="{ty}" class="t">{title}</text>')
    if sub: o.append(f'<text x="{x+w/2}" y="{ty+32}" class="s">{sub}</text>')
def pin(x,y,label,side):
    o.append(f'<circle cx="{x}" cy="{y}" r="6" fill="#fff" stroke="#424242" stroke-width="2.5"/>')
    if side=='l': o.append(f'<text x="{x+13}" y="{y+8}" class="p" text-anchor="start">{label}</text>')
    if side=='r': o.append(f'<text x="{x-13}" y="{y+8}" class="p" text-anchor="end">{label}</text>')
    if side=='b': o.append(f'<text x="{x}" y="{y-14}" class="p" text-anchor="middle">{label}</text>')
    if side=='bs': o.append(f'<text x="{x}" y="{y-14}" class="ps" text-anchor="middle">{label}</text>')
    if side=='t': o.append(f'<text x="{x+12}" y="{y-12}" class="p" text-anchor="start">{label}</text>')
    if side=='ti': o.append(f'<text x="{x}" y="{y+30}" class="ps" text-anchor="middle">{label}</text>')
def wire(pts,color,w=4):
    d="M"+" L".join(f"{x},{y}" for x,y in pts); back.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linejoin="round"/>')
def dot(x,y,c): o.append(f'<circle cx="{x}" cy="{y}" r="7.5" fill="{c}"/>')
def note(x,y,t,cls="n",a="start"): o.append(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{a}">{t}</text>')
# band A
box(30,40,280,150,"Batería LiPo 3S","11,1 V · 3300 mAh")
pin(310,120,"+","r"); pin(310,170,"−","r")
box(345,97,170,46,"",fill="#fff"); note(430,129,"Fusible 7,5 A","s2","middle")
box(545,97,150,46,"",fill="#fff"); note(620,129,"Interruptor","s2","middle")
wire([(310,120),(345,120)],C["bat"],6); wire([(515,120),(545,120)],C["bat"],6); wire([(695,120),(735,120)],C["bat"],6)
wire([(735,120),(880,120)],C["bat"],6); wire([(735,120),(735,720),(200,720),(200,780)],C["bat"],6); dot(735,120,C["bat"])
wire([(310,170),(880,170)],C["gnd"],6); wire([(765,170),(765,740),(290,740),(290,780)],C["gnd"],6); dot(765,170,C["gnd"])
box(880,40,330,190,"Regulador LM2596","ajustado a 5,1 V")
pin(880,120,"IN+","l"); pin(880,170,"IN−","l"); pin(950,230,"OUT+","bs"); pin(1110,230,"OUT−","bs")
# Pi
box(820,320,380,360,"Raspberry Pi 4","4 GB · ROS 2 Humble")
pin(950,320,"","ti"); pin(1110,320,"","ti")
wire([(950,230),(950,320)],C["v5"],5); wire([(1110,230),(1110,320)],C["gnd"],5); note(1030,292,"cable USB-C","n3","middle")
wire([(950,262),(700,262),(700,700),(110,700),(110,780)],C["v5"],4); dot(950,262,C["v5"])
for y,l in zip([440,490,540,590],["pin 1 · 3,3 V","pin 3 · SDA","pin 5 · SCL","pin 6 · GND"]): pin(1200,y,l,"r")
box(1250,330,190,310,"IMU","MPU6050",ty=370)
for y,l in zip([440,490,540,590],["VCC","SDA","SCL","GND"]): pin(1250,y,l,"l")
wire([(1200,440),(1250,440)],C["v5"]); wire([(1200,490),(1250,490)],C["i2c"]); wire([(1200,540),(1250,540)],C["i2c"]); wire([(1200,590),(1250,590)],C["gnd"])
pin(820,500,"USB","l")
box(40,420,300,130,"LiDAR","RPLidar A1M8")
pin(340,500,"USB","r")
wire([(340,500),(820,500)],C["usb"],5); note(540,488,"USB","n3","middle")
pin(980,680,"USB","b")
# L298N
box(60,780,380,460,"Puente H L298N",ty=835)
note(250,875,"puente «5V EN» retirado","n2","middle")
pin(110,780,"+5V","t"); pin(200,780,"+12V","t"); pin(290,780,"GND","t")
for y,l in zip([930,970,1010,1050,1090,1130,1190],["ENA","IN1","IN2","IN3","IN4","ENB","GND"]): pin(440,y,l,"r")
for x,l in zip([100,170,340,410],["OUT1","OUT2","OUT3","OUT4"]): pin(x,1240,l,"bs")
# Arduino
box(640,780,400,460,"Arduino Nano","ATmega328P · CH340",ty=835)
pin(980,780,"USB","t"); wire([(980,680),(980,780)],C["usb"],5); note(992,735,"serie, 115 200 baudios","n3")
for y,l in zip([930,970,1010,1050,1090,1130,1190],["D9","D12","D13","D7","D8","D11","GND"]): pin(640,y,l,"l")
for y in [930,970,1010,1050,1090,1130]: wire([(440,y),(640,y)],C["ctl"])
wire([(440,1190),(640,1190)],C["gnd"]); note(540,915,"PWM y sentido","n3","middle")
bx=[680,740,800,860,930,1000]
for x,l in zip(bx,["D2","D3","D4","D5","5V","GND"]): pin(x,1240,l,"bs")
# motors
box(30,1330,270,150,"Motor derecho",ty=1395); box(310,1330,270,150,"Motor izquierdo",ty=1395)
note(165,1425,"JGA25-371 · 6 V","n2","middle"); note(445,1425,"JGA25-371 · 6 V","n2","middle")
for x,l in zip([100,170],["M1","M2"]): pin(x,1330,l,"t")
for x,l in zip([340,410],["M1","M2"]): pin(x,1330,l,"t")
for x in [100,170,340,410]: wire([(x,1240),(x,1330)],C["mot"],5)
md=[(60,"C1"),(120,"C2"),(185,"VCC"),(250,"GND")]; mi=[(340,"C1"),(400,"C2"),(465,"VCC"),(530,"GND")]
for x,l in md+mi: pin(x,1480,l,"bs")
routes=[(340,680,C["enc"]),(60,740,C["enc"]),(400,800,C["enc"]),(120,860,C["enc"]),(465,930,C["v5"]),(185,930,C["v5"]),(530,1000,C["gnd"]),(250,1000,C["gnd"])]
for (sx,ex,col),y in zip(routes,[1515,1540,1565,1590,1620,1645,1675,1700]): wire([(sx,1480),(sx,y),(ex,y),(ex,1240)],col,4)
dot(930,1620,C["v5"]); dot(1000,1675,C["gnd"])
note(1080,1290,"Encoder derecho:","n4b"); note(1080,1322,"C1 → D3, C2 → D5","n4")
note(1080,1362,"Encoder izquierdo:","n4b"); note(1080,1394,"C1 → D2, C2 → D4","n4")
note(1080,1434,"VCC de los dos → 5V,","n4"); note(1080,1466,"GND de los dos → GND","n4")
items=[("bat","Batería, 11,1 V"),("gnd","Tierra (GND)"),("v5","5 V y 3,3 V"),("mot","Motores"),("ctl","PWM y sentido"),("enc","Encoders, fases A y B"),("i2c","Bus I²C"),("usb","USB")]
for i,(k,t) in enumerate(items):
    yy=1530+i*34; o.append(f'<line x1="1080" y1="{yy}" x2="1130" y2="{yy}" stroke="{C[k]}" stroke-width="7"/>'); note(1142,yy+8,t,"n5")
svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<style>text{{font-family:"Liberation Sans",Arial,sans-serif;fill:#212121}} .t{{font-size:29px;font-weight:bold;text-anchor:middle}} .s{{font-size:24px;text-anchor:middle;fill:#555}} .s2{{font-size:23px;font-weight:bold}} .p{{font-size:24px}} .ps{{font-size:22px}} .n2{{font-size:22px;fill:#555;font-style:italic}} .n3{{font-size:22px;fill:#444;font-style:italic}} .n4{{font-size:24px}} .n4b{{font-size:24px;font-weight:bold}} .n5{{font-size:23px}}</style>
<rect width="{W}" height="{H}" fill="#fff"/>{"".join(back)}{"".join(o)}</svg>'''
open("conexiones.svg","w").write(svg)
