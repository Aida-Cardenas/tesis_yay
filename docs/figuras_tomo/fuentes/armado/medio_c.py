import sys
from lib import Canvas
RED, BLK = "#D32F2F", "#263238"
V5, USB = "#FF7043", "#6A1B9A"
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)
S = 3.0
c = Canvas(2950, 1700, S=S)
c.rect(30, 30, 1500, 1640, "#FFF3E8", stroke="#F4A261", rx=40, sw=4)
c.text(70, 92, "4 · MEDIO: alimentación", 36, bold=True, color="#BF360C")
c.place("l298n", "l298n", 980, 150)
c.place("lm", "lm2596", 110, 250)
c.rect(70, 1230, 230, 300, "#B0BEC5", stroke="#37474F", rx=16, sw=4, layer="parts")
c.text(185, 1340, "Batería", 28, "middle", bold=True)
c.text(185, 1380, "LiPo 3S", 26, "middle")
c.text(185, 1416, "11,1–12,6 V", 26, "middle")
c.text(185, 1480, "(en la Base)", 22, "middle", italic=True)
c.place("xm", "xt60m", 360, 1300)
c.place("xf", "xt60f", 360 + 97.8 * S + 6, 1300)
c.place("sw", "switch", 1180, 1150, scale=5.0)
L = lambda p: c.p("l298n", p); M = lambda p: c.p("lm", p)
bp, bn = (300, 1420), (300, 1350)
c.curve(bp, RIGHT, c.p("xm", "VCC"), LEFT, RED, 9, 30, 30)
c.curve(bn, RIGHT, c.p("xm", "GND"), LEFT, BLK, 9, 30, 30)
c.text(510, 1285, "XT60: el de la batería y el de tu cable", 22, "middle", italic=True)
fp, fn = c.p("xf", "VCC"), c.p("xf", "GND")
com, l1 = c.p("sw", "COM"), c.p("sw", "L1")
c.curve(fp, RIGHT, com, DOWN, RED, 9, 80, 120)
c.text(1270, 1120, "interruptor", 24, "middle", bold=True)
c.text(1270, 1090, "(fusible opcional entre XT60 e interruptor)", 20, "middle", italic=True)
c.curve(l1, DOWN, L("12V"), DOWN, RED, 9, 260, 420)
c.curve(l1, DOWN, M("IN+"), LEFT, RED, 9, 300, 120, dots=False)
c.curve(fn, RIGHT, L("GND"), DOWN, BLK, 9, 380, 440)
c.curve(fn, RIGHT, M("IN-"), LEFT, BLK, 9, 260, 160, dots=False)
c.dot(*M("IN+"), RED, 7); c.dot(*M("IN-"), BLK, 7)
c.curve(M("OUT+"), RIGHT, L("5V"), DOWN, V5, 6, 160, 380)
c.tag(640, 600, "OUT+ → +5V del L298N (cable fino)", V5, "start", 22)
op, on = M("OUT+"), M("OUT-")
c.curve(op, RIGHT, (640, 860), DOWN, RED, 7, 70, 160, dots=False)
c.curve(on, RIGHT, (670, 860), DOWN, BLK, 7, 40, 140, dots=False)
c.rect(610, 860, 90, 54, "#424242", stroke="#212121", rx=8, sw=2, layer="top")
c.text(655, 896, "USB-C", 20, "middle", bold=True, color="#fff")
c.tag(720, 896, "→ Pi (Tope), sin enchufar hasta medir", USB, "start", 22)
for p, t, a, dx in (("IN+", "IN+", "end", -14), ("IN-", "IN−", "end", -14), ("OUT+", "OUT+", "start", 14), ("OUT-", "OUT−", "start", 14)):
    x, y = M(p); c.tag(x + dx, y + 8, t, "#37474F", a, 21)
for p, t, a, dy in (("12V", "+12V", "end", 50), ("GND", "GND", "middle", 95), ("5V", "+5V", "start", 50)):
    x, y = L(p); c.tag(x + (12 if a == "start" else (-12 if a == "end" else 0)), y + dy, t, "#37474F", a, 21)
x, y = L("J5VEN"); c.raw(f'<circle cx="{x}" cy="{y}" r="17" fill="none" stroke="#C62828" stroke-width="4.5" stroke-dasharray="7,4"/>')
c.tag(x + 30, y - 30, "sin puente 5V EN", "#C62828", "start", 21)
c.steps(1600, 110, 1300, "4 · Alimentación: paso a paso", [
    ("h", "Antes de cablear: ajustar el regulador"),
    ("", ["1. Batería (o fuente de laboratorio a 12 V) a IN+ e IN−.", "Nada conectado en la salida."]),
    ("", ["2. Multímetro en V⎓ (20 V) en OUT+ y OUT−. Gira el tornillito", "dorado hasta que marque entre 5,05 y 5,2 V. Desconecta."]),
    ("h", "Montaje (interruptor apagado, batería desconectada)"),
    ("", ["3. Fija el regulador en el Medio, cerca del L298N.", "El interruptor, en un borde donde lo alcances."]),
    ("", ["4. + (rojo, 18–20 AWG): XT60 → interruptor → dos cables:", "uno a +12V del L298N y otro a IN+ del regulador."]),
    ("", ["5. − (negro): XT60 → dos cables: GND del L298N", "y IN− del regulador."]),
    ("", "6. Cable fino de OUT+ del regulador a +5V del L298N."),
    ("", ["7. Cable USB-C a OUT+ (rojo) y OUT− (negro).", "Todavía no lo enchufes a la Pi."]),
    ("h", "Pruebas"),
    ("", "✔ Apagado: + y − NO suenan en continuidad."),
    ("", ["✔ Encendido: LED rojo del L298N; entre +12V y GND", "marca el voltaje de la batería; en el USB-C, 5,05–5,2 V."]),
    ("", ["✔ Ruedas en el aire, Nano por USB:", "python3 probar_arduino.py girar   y después   maxima"]),
], size=29)
c.save(sys.argv[1])
