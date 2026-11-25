import sys
from lib import Canvas
RED, BLK, YEL, GRN, BLU, WHT = "#D32F2F", "#263238", "#FDD835", "#2E7D32", "#1565C0", "#F5F5F5"
UP, DOWN, LEFT, RIGHT = (0, -1), (0, 1), (-1, 0), (1, 0)
S = 5.0
c = Canvas(2950, 1700, S=S)
c.rect(30, 30, 1500, 1640, "#FFF3E8", stroke="#F4A261", rx=40, sw=4)
c.text(70, 92, "2 · MEDIO: Arduino Nano y encoders", 36, bold=True, color="#BF360C")
bx, by = 420, 420
c.place("bb", "mini", bx, by)
c.place("nano", "nano", bx + 3.72 * S, by + 21.66 * S, rot=-90)
h = lambda col, row: c.p("bb", (col, row))
cols = [RED, WHT, BLU, BLK, YEL, GRN]
labs = ["M+", "M−", "VCC", "GND", "A", "B"]
der = c.cable_end(150, 1250, cols, labs, "Motor DERECHO", pitch=44)
izq = c.cable_end(1030, 1250, cols, labs, "Motor IZQUIERDO", pitch=44)
for e in (der, izq):
    for k in (0, 1):
        x, y = e[k]
        c.raw(f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y-50}" stroke="{cols[k]}" stroke-width="7" stroke-dasharray="8,6"/>')
for x0 in (150, 1030):
    c.text(x0 + 22, 1385, "M+ y M− van al L298N", 21, "middle", italic=True)
    c.text(x0 + 22, 1410, "(diagrama 3)", 21, "middle", italic=True)
c.curve(der[2], UP, h(5, "A"), UP, BLU, 7, 700, 260)
c.curve(der[3], UP, h(5, "J"), DOWN, BLK, 7, 220, 260)
c.curve(der[4], UP, h(7, "J"), DOWN, YEL, 7, 240, 300)
c.curve(der[5], UP, h(9, "J"), DOWN, GRN, 7, 260, 340)
c.curve(izq[2], UP, h(5, "B"), UP, BLU, 7, 760, 300)
c.curve(izq[3], UP, h(5, "I"), DOWN, BLK, 7, 200, 200)
c.curve(izq[4], UP, h(6, "J"), DOWN, YEL, 7, 220, 240)
c.curve(izq[5], UP, h(8, "J"), DOWN, GRN, 7, 240, 280)
def lab(col, row, t, dy, color):
    x, y = h(col, row); c.tag(x, y + dy, t, color, "middle", 22)
lab(5, "A", "5V", -70, BLU)
lab(5, "J", "GND", 120, BLK)
lab(6, "J", "D2", 160, YEL)
lab(7, "J", "D3", 120, YEL)
lab(8, "J", "D4", 160, GRN)
lab(9, "J", "D5", 120, GRN)
nx, ny = bx + 3.72 * S + 127 * S, by + 21.66 * S + 25.2 * S
c.curve((nx - 8, ny), RIGHT, (1460, 330), UP, "#6A1B9A", 10, 200, 120, dots=False)
c.tag(1460, 300, "USB: laptop para probar; luego la Pi", "#6A1B9A", "end", 22)
c.text(780, 1530, "Cada columna de 5 agujeros de la protoboard está unida por dentro:", 23, "middle", italic=True)
c.text(780, 1562, "un cable en la misma columna que un pin del Nano queda conectado a ese pin.", 23, "middle", italic=True)
c.steps(1600, 110, 1300, "2 · Nano y encoders: paso a paso", [
    ("h", "Montaje"),
    ("", ["1. Pega la protoboard mini en el Medio con cinta doble faz,", "cerca del borde por donde subirá el USB hacia la Pi."]),
    ("", ["2. Pincha el Nano en la protoboard a lo largo, cruzando el canal", "del centro: una fila de pines a cada lado, sin doblarlos."]),
    ("", "3. Encoders: la punta del cable directo al agujero, sin soldar."),
    ("", "    • Derecho: amarillo → D3, verde → D5"),
    ("", "    • Izquierdo: amarillo → D2, verde → D4"),
    ("", "    • Azules (VCC) de los dos → columna del 5V"),
    ("", "    • Negros (GND) de los dos → columna del GND (abajo)"),
    ("", "4. Rojo y blanco (M+ y M−) NO van aquí: van al L298N."),
    ("h", "Prueba (sin batería, solo el USB)"),
    ("", "✔ Nano a tu laptop por USB, Monitor Serie a 115200."),
    ("", ["✔ Gira la rueda derecha 5 vueltas hacia adelante:", "rc cambia unos 3850 (770 por vuelta)."]),
    ("", "✔ Lo mismo con la izquierda, mirando lc."),
    ("", ["✔ Si baja en vez de subir: intercambia amarillo y verde", "de ese motor. Si no cambia: revisa azul y negro."]),
    ("", "✔ Pásame los dos números."),
], size=29)
c.save(sys.argv[1])
