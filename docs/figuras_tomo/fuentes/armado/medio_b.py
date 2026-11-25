import sys
from lib import Canvas
RED, BLK, YEL, GRN, BLU, WHT = "#D32F2F", "#263238", "#FDD835", "#2E7D32", "#1565C0", "#F5F5F5"
CTL = {"ENA": "#00ACC1", "IN1": "#8D6E63", "IN2": "#9E9E9E", "IN3": "#EF6C00", "IN4": "#EC407A", "ENB": "#7E57C2"}
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)
S = 3.6
c = Canvas(2950, 1700, S=S)
c.rect(30, 30, 1500, 1640, "#FFF3E8", stroke="#F4A261", rx=40, sw=4)
c.text(70, 92, "3 · MEDIO: L298N, motores y control", 36, bold=True, color="#BF360C")
c.place("l298n", "l298n", 170, 150)
bx, by = 930, 820
c.place("bb", "mini", bx, by)
c.place("nano", "nano", bx + 3.72 * S, by + 21.66 * S, rot=-90)
L = lambda p: c.p("l298n", p)
h = lambda col, row: c.p("bb", (col, row))
for j in ("J5VEN", "JENA", "JENB"):
    x, y = L(j); c.raw(f'<circle cx="{x}" cy="{y}" r="19" fill="none" stroke="#C62828" stroke-width="4.5" stroke-dasharray="7,4"/>')
c.tag(60, 820, "✗ quitar los 3 puentes (5V EN, ENA, ENB)", "#C62828", "start", 22)
cols = [RED, WHT, BLU, BLK, YEL, GRN]
labs = ["M+", "M−", "VCC", "GND", "A", "B"]
der = c.cable_end(110, 1430, cols, labs, "Motor DERECHO", pitch=44)
izq = c.cable_end(560, 1430, cols, labs, "Motor IZQUIERDO", pitch=44)
c.curve(der[0], UP, L("OUT1"), LEFT, RED, 9, 420, 120)
c.curve(der[1], UP, L("OUT2"), LEFT, WHT, 9, 360, 90, edge="#616161")
c.curve(izq[0], UP, L("OUT3"), RIGHT, RED, 9, 520, 200)
c.curve(izq[1], UP, L("OUT4"), RIGHT, WHT, 9, 580, 260, edge="#616161")
for e in (der, izq):
    for k in range(2, 6):
        x, y = e[k]
        c.raw(f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y-45}" stroke="{cols[k]}" stroke-width="7" stroke-dasharray="8,6"/>')
c.text(560, 1555, "VCC, GND, A y B ya quedaron en la protoboard (diagrama 2)", 21, "middle", italic=True)
for p, t in (("OUT1", "OUT1"), ("OUT2", "OUT2")):
    x, y = L(p); c.tag(x - 16, y + 6, t, RED if p == "OUT1" else "#616161", "end", 21)
for p, t in (("OUT3", "OUT3"), ("OUT4", "OUT4")):
    x, y = L(p); c.tag(x + 16, y + 6, t, RED if p == "OUT3" else "#616161", "start", 21)
ctl = [("ENA", 13, "J", "D9"), ("IN1", 16, "J", "D12"), ("IN3", 11, "J", "D7"), ("IN4", 12, "J", "D8"), ("ENB", 15, "J", "D11")]
for i, (pin, col, row, dn) in enumerate(ctl):
    c.curve(L(pin), DOWN, h(col, row), DOWN, CTL[pin], 7, 300 + 30 * i, 260 + 40 * i)
c.curve(L("IN2"), DOWN, h(16, "A"), UP, CTL["IN2"], 7, 120, 160)
c.curve(L("GND"), DOWN, h(5, "H"), DOWN, BLK, 7, 260, 300)
tags = [("D9", 13, "J", CTL["ENA"]), ("D12", 16, "J", CTL["IN1"]), ("D7", 11, "J", CTL["IN3"]), ("D8", 12, "J", CTL["IN4"]), ("D11", 15, "J", CTL["ENB"])]
yb = h(5, "J")[1] + 300
c.text(1130, yb - 40, "Llegan a la protoboard:", 22, "middle", italic=True)
legend = [("ENA → D9", CTL["ENA"]), ("IN1 → D12", CTL["IN1"]), ("IN2 → D13", CTL["IN2"]), ("IN3 → D7", CTL["IN3"]), ("IN4 → D8", CTL["IN4"]), ("ENB → D11", CTL["ENB"]), ("GND → GND", BLK)]
for k, (t, col) in enumerate(legend):
    xx = 960 + (k % 2) * 260; yy = yb + (k // 2) * 46
    c.raw(f'<line x1="{xx}" y1="{yy-8}" x2="{xx+40}" y2="{yy-8}" stroke="{col}" stroke-width="9"/>')
    c.text(xx + 52, yy, t, 23, bold=True)
c.steps(1600, 110, 1300, "3 · L298N: paso a paso", [
    ("h", "Montaje"),
    ("", ["1. Fija el L298N en el Medio. Su disipador (la pieza", "negra con aletas) tiene que quedar con aire alrededor."]),
    ("", "2. Quita los 3 puentes: 5V EN, ENA y ENB. Guárdalos."),
    ("", "3. Motores a las borneras de tornillo:"),
    ("", "    • DERECHO: rojo → OUT1, blanco → OUT2"),
    ("", "    • IZQUIERDO: rojo → OUT3, blanco → OUT4"),
    ("", ["4. Control con dupont macho-hembra (la hembra en el L298N,", "el macho en la protoboard): ver la tabla del dibujo."]),
    ("", ["5. ENA y ENB tienen 2 pines cada uno: usa el que NO suena", "en continuidad con la bornera +5V."]),
    ("", ["6. Tierra común: del GND de la bornera del L298N a la columna", "GND del Nano. Sin este cable el L298N no entiende al Nano."]),
    ("h", "Pruebas (sin batería)"),
    ("", ["✔ Multímetro en continuidad: cada cable suena entre", "su pin del L298N y su pin del Nano."]),
    ("", "✔ Tira suave de cada cable: ninguno se sale."),
    ("", ["✔ Los motores todavía no giran: eso se prueba con la", "alimentación (diagrama 4)."]),
], size=29)
c.save(sys.argv[1])
