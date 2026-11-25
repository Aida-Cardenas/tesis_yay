import sys
from lib import Canvas
RED, BLK, YEL, GRN, BLU, WHT = "#D32F2F", "#263238", "#FDD835", "#2E7D32", "#1565C0", "#F5F5F5"
CTL = {"ENA": "#00ACC1", "IN1": "#8D6E63", "IN2": "#9E9E9E", "IN3": "#EF6C00", "IN4": "#EC407A", "ENB": "#7E57C2"}
V5, USB = "#FF7043", "#6A1B9A"
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)
c = Canvas(3300, 1900, S=3.0)
c.rect(30, 30, 2230, 1840, "#FFF3E8", stroke="#F4A261", rx=40, sw=4)
c.text(70, 92, "MEDIO · vista desde arriba", 36, bold=True, color="#BF360C")
c.place("l298n", "l298n", 170, 190)
c.place("lm", "lm2596", 1180, 170)
c.place("bb", "mini", 900, 1020)
c.place("nano", "nano", 900 + 3.72 * 3, 1020 + 21.66 * 3, rot=-90)
c.place("imu", "imu", 1960, 640)
L = lambda p: c.p("l298n", p); M = lambda p: c.p("lm", p)
h = lambda col, row: c.p("bb", (col, row))
# power from the base
c.text(560, 1735, "de la Base:", 24, "middle", italic=True)
pw = c.cable_end(530, 1770, [RED, BLK], ["+", "−"], "", pitch=60)
c.curve(pw[0], UP, L("12V"), DOWN, RED, 9, 380, 160)
c.curve(pw[0], UP, M("IN+"), LEFT, RED, 9, 700, 260)
c.curve(pw[1], UP, L("GND"), DOWN, BLK, 9, 300, 200)
c.curve(pw[1], UP, M("IN-"), LEFT, BLK, 9, 560, 200)
c.tag(450, 1700, "+ del interruptor", RED, "end", 22); c.tag(660, 1700, "− de la batería", BLK, "start", 22)
c.curve(M("OUT+"), RIGHT, L("5V"), DOWN, V5, 6, 140, 260)
c.tag(1060, 690, "5V del regulador → +5V del L298N", V5, "start", 22)
c.curve(M("OUT+"), RIGHT, (1930, 70), UP, RED, 7, 110, 120, dots=False)
c.curve(M("OUT-"), RIGHT, (1960, 70), UP, BLK, 7, 140, 120, dots=False)
c.tag(1985, 104, "USB-C → Pi (Tope)", USB, "start", 22)
# jumpers removed
for j in ("J5VEN", "JENA", "JENB"):
    x, y = L(j); c.raw(f'<circle cx="{x}" cy="{y}" r="17" fill="none" stroke="#C62828" stroke-width="4" stroke-dasharray="6,4"/>')
c.tag(120, 600, "quitar los 3 puentes", "#C62828", "start", 22)
# control L298N -> Nano
ctl = [("ENA", 13, "J", "D9"), ("IN1", 16, "J", "D12"), ("IN3", 11, "J", "D7"), ("IN4", 12, "J", "D8"), ("ENB", 15, "J", "D11")]
for i, (pin, col, row, dn) in enumerate(ctl):
    c.curve(L(pin), DOWN, h(col, row), DOWN, CTL[pin], 6, 260 + 40 * i, 260 + 30 * i)
c.curve(L("IN2"), DOWN, h(16, "A"), UP, CTL["IN2"], 6, 140, 200)
c.curve(L("GND"), DOWN, h(5, "H"), DOWN, BLK, 6, 220, 330)
# motor cables
cols = [RED, WHT, BLU, BLK, YEL, GRN]
labs = ["M+", "M−", "VCC", "GND", "A", "B"]
der = c.cable_end(120, 1500, cols, labs, "Motor DERECHO (sube de la Base)")
izq = c.cable_end(1700, 1500, cols, labs, "Motor IZQUIERDO (sube de la Base)")
c.curve(der[0], UP, L("OUT1"), LEFT, RED, 7, 500, 80)
c.curve(der[1], UP, L("OUT2"), LEFT, WHT, 7, 450, 90, edge="#555")
c.curve(der[2], UP, h(5, "A"), UP, BLU, 6, 700, 300)
c.curve(der[3], UP, h(5, "J"), DOWN, BLK, 6, 160, 300)
c.curve(der[4], UP, h(7, "J"), DOWN, YEL, 6, 140, 260)
c.curve(der[5], UP, h(9, "J"), DOWN, GRN, 6, 120, 220)
c.curve(izq[0], UP, L("OUT3"), RIGHT, RED, 7, 600, 260)
c.curve(izq[1], UP, L("OUT4"), RIGHT, WHT, 7, 650, 230, edge="#555")
c.curve(izq[2], UP, h(5, "B"), UP, BLU, 6, 760, 340)
c.curve(izq[3], UP, h(5, "I"), DOWN, BLK, 6, 140, 360)
c.curve(izq[4], UP, h(6, "J"), DOWN, YEL, 6, 120, 300)
c.curve(izq[5], UP, h(8, "J"), DOWN, GRN, 6, 100, 260)
# Nano USB and IMU to the top level
nx, ny = 900 + 3.72 * 3 + 127 * 3, 1020 + 21.66 * 3 + 25.2 * 3
c.curve((nx - 6, ny), RIGHT, (2080, 70), UP, USB, 9, 240, 300, dots=False)
c.tag(2105, 150, "USB del Nano → Pi", USB, "start", 22)
for k, (pin, col) in enumerate([("VCC", RED), ("GND", BLK), ("SCL", "#00838F"), ("SDA", "#26C6DA")]):
    p = c.p("imu", pin)
    c.curve(p, LEFT, (1770 + k * 26, 70), UP, col, 6, 70 + 10 * k, 200, dots=False)
    c.dot(1770 + k * 26, 70, col, 6)
c.tag(1640, 40, "IMU → pines 1, 6, 5 y 3 de la Pi", "#00838F", "start", 22)
c.text(2055, 920, "IMU en el centro, sobre el eje", 22, "middle", italic=True)
c.text(2055, 948, "de las ruedas; flecha X adelante", 22, "middle", italic=True)
# breadboard notes
c.text(1095, 1370, "Nano pinchado en la protoboard mini (fila C y fila G)", 22, "middle", italic=True)
c.save(sys.argv[1])
