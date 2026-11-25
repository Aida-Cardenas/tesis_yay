import sys
from lib import Canvas
RED, BLK = "#D32F2F", "#263238"
USB = "#6A1B9A"
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)
S = 3.0
c = Canvas(2950, 1700, S=S)
c.rect(30, 30, 1500, 1640, "#FFF3E8", stroke="#F4A261", rx=40, sw=4)
c.text(70, 92, "5 · TOPE: Raspberry Pi y LiDAR", 36, bold=True, color="#BF360C")
px, py = 330, 560
c.place("pi", "pi", px, py)
P = lambda p: c.p("pi", p)
lx, ly = 1020, 290
c.circle(lx, ly, 150, "#37474F", stroke="#212121", sw=4, layer="parts")
c.circle(lx, ly, 95, "#263238", stroke="#455A64", sw=4, layer="parts")
c.circle(lx, ly, 26, "#90A4AE", stroke="#455A64", sw=3, layer="parts")
c.text(lx, ly - 170, "LiDAR RPLidar A1: arriba de todo, en el centro", 24, "middle", bold=True)
c.rect(lx + 190, ly + 40, 130, 70, "#1565C0", stroke="#0D47A1", rx=8, sw=3, layer="parts")
c.text(lx + 255, ly + 84, "adaptador", 19, "middle", color="#fff", bold=True)
c.curve((lx + 140, ly + 40), RIGHT, (lx + 190, ly + 75), RIGHT, "#212121", 5, 40, 40, dots=False)
c.curve((lx + 320, ly + 75), RIGHT, P("USB3a"), RIGHT, USB, 10, 140, 120)
c.tag(1490, 520, "USB del LiDAR", USB, "end", 22)
imu = c.cable_end(170, 1480, [RED, BLK, "#00838F", "#26C6DA"], ["VCC", "GND", "SCL", "SDA"], "de la IMU (sube del Medio)", pitch=48)
c.curve(imu[0], UP, P("P1"), UP, RED, 7, 900, 150)
c.curve(imu[1], UP, P("P6"), UP, BLK, 7, 960, 230)
c.curve(imu[2], UP, P("P5"), UP, "#00838F", 7, 1020, 190)
c.curve(imu[3], UP, P("P3"), UP, "#26C6DA", 7, 1080, 110)
ix, iy = 80, 150
c.rect(ix, iy, 560, 300, "#FFFFFF", stroke="#00838F", rx=16, sw=3, layer="top")
c.text(ix + 280, iy + 40, "Zoom: esquina del conector GPIO", 24, "middle", bold=True)
pins = {2: ("5 V", None), 4: ("5 V", None), 6: ("GND", BLK), 1: ("3,3 V", RED), 3: ("SDA", "#26C6DA"), 5: ("SCL", "#00838F")}
for n, (lab, col) in pins.items():
    k = (n - 1) // 2
    x = ix + 120 + k * 150
    y = iy + (110 if n % 2 == 0 else 200)
    c.raw(f'<rect x="{x-24}" y="{y-24}" width="48" height="48" rx="6" fill="#212121"/>', "top")
    c.raw(f'<circle cx="{x}" cy="{y}" r="11" fill="{col or '#BDBDBD'}" stroke="#fff" stroke-width="2"/>', "top")
    c.text(x + 34, y + 9, f"{n}", 24, "start", bold=True)
    c.text(x, y + (-36 if n % 2 == 0 else 54), lab, 22, "middle", bold=bool(col), color=("#C62828" if col is None else "#212121"))
    if col is None:
        c.raw(f'<path d="M{x-16},{y-16} L{x+16},{y+16} M{x+16},{y-16} L{x-16},{y+16}" stroke="#C62828" stroke-width="5"/>', "top")
c.text(ix + 280, iy + 290, "pin 1: el de la esquina, lejos de los USB", 20, "middle", italic=True)
nano = c.cable_end(1180, 1480, [USB], [""], "USB del Nano (Medio)", pitch=40)
c.curve(nano[0], UP, P("USB2a"), RIGHT, USB, 10, 200, 120)
pwr = c.cable_end(700, 1480, [RED], [""], "USB-C del regulador (Medio)", pitch=40)
c.curve(pwr[0], UP, P("PWR"), DOWN, "#424242", 10, 120, 120)
c.tag(P("PWR")[0] + 40, P("PWR")[1] + 60, "entrada de alimentación (USB-C)", "#424242", "start", 21)
c.steps(1600, 110, 1300, "5 · Tope: paso a paso", [
    ("h", "Montaje"),
    ("", ["1. Raspberry Pi con sus separadores: nada metálico debajo.", "Que la microSD y los USB queden accesibles."]),
    ("", ["2. LiDAR arriba de todo, en el centro, con sus 4 tornillos.", "Nada más alto alrededor: tiene que ver 360°."]),
    ("", ["3. IMU pegada plana en el centro del Medio, flecha X adelante.", "Sus 4 cables (dupont hembra-hembra) suben a la Pi:"]),
    ("", "    • VCC → pin 1 (3,3 V)     • SDA → pin 3"),
    ("", "    • SCL → pin 5                • GND → pin 6"),
    ("", "4. USB del Nano → un USB de la Pi."),
    ("", "5. Adaptador del LiDAR → otro USB de la Pi."),
    ("", ["6. USB-C del regulador → Pi, SOLO después de medir", "5,05–5,2 V con el multímetro."]),
    ("h", "Pruebas (por ssh en la Pi)"),
    ("", "✔ El LED rojo de la Pi queda fijo (si parpadea: falta voltaje)."),
    ("", "✔ ls /dev/arduino /dev/rplidar   → aparecen los dos"),
    ("", "✔ python3 probar_arduino.py hola   → «El Arduino responde»"),
    ("", "✔ i2cdetect -y 1   → aparece 68 (la IMU)"),
    ("", ["✔ Si no aparece 68: revisa SDA (pin 3), SCL (pin 5),", "3,3 V (pin 1) y GND (pin 6)."]),
], size=29)
c.save(sys.argv[1])
