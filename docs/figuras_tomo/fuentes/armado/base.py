import sys
import numpy as np
import trimesh
from lib import Canvas
mesh = trimesh.load(sys.argv[2])
c = Canvas(2950, 1700, S=3.0)
K, CX, CY = 7.0, 760, 860
def P(x, y): return (CX - y * 1000 * K, CY - x * 1000 * K)
def poly_path(polys):
    d = ""
    for p in polys:
        pts = [P(a, b) for a, b in p[:, :2]]
        d += "M" + " L".join(f"{u:.1f},{v:.1f}" for u, v in pts) + " Z "
    return d
c.rect(30, 30, 1500, 1640, "#FFFFFF", stroke="#BDBDBD", rx=40, sw=3)
c.text(70, 92, "1 · BASE · vista desde arriba", 36, bold=True, color="#BF360C")
from shapely.geometry import Polygon
from shapely.ops import unary_union
sec = mesh.section(plane_origin=[0, 0, -0.020], plane_normal=[0, 0, 1])
loops = [Polygon(p[:, :2]) for p in sec.discrete if len(p) > 3]
loops.sort(key=lambda g: -g.area)
outer = unary_union([g for g in loops if g.area > 0.002]).buffer(0.0006).buffer(-0.0006)
holes = [g for g in loops if g.area <= 0.002 and outer.contains(g.centroid)]
plate = outer.difference(unary_union(holes))
geoms = plate.geoms if hasattr(plate, "geoms") else [plate]
rings = []
for g in geoms:
    rings.append(np.array(g.exterior.coords)); rings += [np.array(i.coords) for i in g.interiors]
c.raw(f'<path d="{poly_path(rings)}" fill="#FF8A3D" fill-rule="evenodd" stroke="#BF5B16" stroke-width="2"/>', "bg")
pads = [p for p in mesh.section(plane_origin=[0, 0, -0.014], plane_normal=[0, 0, 1]).discrete if np.ptp(p[:, 1]) > 0.02]
c.raw(f'<path d="{poly_path(pads)}" fill="#E8712A" fill-rule="evenodd" stroke="#8D3F0F" stroke-width="3"/>', "bg")
# casters (under the plate)
for x in (0.04755, -0.04755):
    u, v = P(x, 0)
    c.circle(u, v, 9 * K, "none", stroke="#455A64", sw=4, layer="parts")
    c.raw(f'<circle cx="{u}" cy="{v}" r="{9*K}" fill="#90A4AE55" stroke="#37474F" stroke-width="4" stroke-dasharray="10,6"/>')
# motors, brackets, wheels
for sgn, name in ((-1, "DERECHO"), (1, "IZQUIERDO")):
    x0, y0 = P(0.012, sgn * 0.012); x1, y1 = P(-0.012, sgn * 0.064)
    c.rect(min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0), "#B0BEC5", stroke="#37474F", rx=8, sw=3, layer="parts")
    e0, f0 = P(0.010, sgn * 0.012); e1, f1 = P(-0.010, sgn * 0.024)
    c.rect(min(e0, e1), min(f0, f1), abs(e1 - e0), abs(f1 - f0), "#263238", stroke="#000", rx=4, sw=2, layer="parts")
    b0, g0 = P(0.016, sgn * 0.064); b1, g1 = P(-0.016, sgn * 0.070)
    c.rect(min(b0, b1), min(g0, g1), abs(b1 - b0), abs(g1 - g0), "#78909C", stroke="#263238", rx=3, sw=2, layer="parts")
    w0, h0 = P(0.033, sgn * 0.072); w1, h1 = P(-0.033, sgn * 0.098)
    c.rect(min(w0, w1), min(h0, h1), abs(w1 - w0), abs(h1 - h0), "#212121", stroke="#000", rx=10, sw=2, layer="parts")
    mx, my = P(0, sgn * 0.040)
    c.text(mx, my - 25, "motor", 24, "middle", bold=True)
    c.text(mx, my + 5, name, 22, "middle", bold=True, color="#BF360C")
# standoffs
for x, y in [(-24.0, -64.2), (64.1, -24.2), (-63.9, -12.2), (64.0, 23.8), (-64.1, 23.8), (24.0, 63.8)]:
    u, v = P(x / 1000, y / 1000)
    pts = " ".join(f"{u+18*np.cos(a):.1f},{v+18*np.sin(a):.1f}" for a in np.linspace(0, 2 * np.pi, 7)[:-1])
    c.raw(f'<polygon points="{pts}" fill="#CFD8DC" stroke="#37474F" stroke-width="3"/>')
# battery area
b0, g0 = P(0.058, 0.045); b1, g1 = P(0.022, -0.045)
c.raw(f'<rect x="{min(b0,b1)}" y="{min(g0,g1)}" width="{abs(b1-b0)}" height="{abs(g1-g0)}" rx="10" fill="#FFF59D99" stroke="#F9A825" stroke-width="4" stroke-dasharray="14,8"/>')
bx, by = P(0.040, 0)
c.text(bx, by - 4, "batería (cuando la tengas)", 24, "middle", bold=True)
c.text(bx, by + 26, "fija con velcro o brida", 22, "middle", italic=True)
# front arrow
ax, ay = P(0.085, 0)
c.raw(f'<path d="M{ax-40},{ay+30} L{ax},{ay-20} L{ax+40},{ay+30} Z" fill="#1565C0"/>')
c.text(ax, ay - 34, "ADELANTE", 30, "middle", bold=True, color="#1565C0")
lx, ly = P(-0.110, 0)
c.text(lx, ly, "(con el robot de espaldas a ti: DERECHO a tu derecha)", 22, "middle", italic=True)
# dimensions
y1 = P(-0.085, 0.085); y2 = P(-0.085, -0.085)
yy = P(-0.088, 0)[1] + 70
for (u, v) in (y1, y2):
    c.raw(f'<line x1="{u}" y1="{v-140}" x2="{u}" y2="{yy+14}" stroke="#1565C0" stroke-width="2.5" stroke-dasharray="6,5"/>')
c.raw(f'<line x1="{y1[0]+6}" y1="{yy}" x2="{y2[0]-6}" y2="{yy}" stroke="#1565C0" stroke-width="3" marker-start="url(#ar)" marker-end="url(#ar)"/>')
c.text((y1[0]+y2[0])/2, yy - 12, "17 cm de centro a centro de las ruedas", 24, "middle", bold=True, color="#1565C0")
wx, wy = P(0.040, 0.085)
c.text(wx, wy - 18, "rueda", 24, "middle", bold=True)
c.text(wx, wy + 10, "Ø 66 mm", 24, "middle", bold=True, color="#1565C0")
wx, wy = P(0.040, -0.085)
c.text(wx, wy - 18, "rueda", 24, "middle", bold=True)
c.text(wx, wy + 10, "Ø 66 mm", 24, "middle", bold=True, color="#1565C0")
for x, t in ((0.04755, "rueda loca (por debajo)"), (-0.04755, "rueda loca (por debajo)")):
    u, v = P(x, 0); c.tag(u + 75, v + 8, t, "#37474F", "start", 21)
u, v = P(-0.064, 0.024); c.tag(u - 30, v + 60, "separador (son 6)", "#37474F", "start", 21)
u, v = P(0.0, -0.067); c.tag(u - 10, v - 70, "soporte metálico", "#37474F", "start", 21)
c.raw('<defs><marker id="ar" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="12" markerHeight="12" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#1565C0"/></marker></defs>')
c.steps(1600, 110, 1650, "1 · Base: paso a paso", [
    ("h", "Montaje"),
    ("", "1. Quita las paredes diagonales (listo) y limpia los agujeros."),
    ("", "2. Marca ADELANTE con cinta en uno de los bordes sin rectángulos."),
    ("", ["3. Ruedas locas por debajo: una adelante y otra atrás,", "en la línea del centro, a unos 4,75 cm del centro."]),
    ("", "4. Atornilla el soporte metálico en la cara del motor por donde sale el eje."),
    ("", ["5. Cada motor sobre su rectángulo: eje hacia afuera, encoder hacia", "el centro, tuercas por debajo. Cinta DERECHO o IZQUIERDO."]),
    ("", ["6. Conecta ya el cable de 6 hilos a cada motor:", "con todo montado cuesta llegar al conector."]),
    ("", "7. Ruedas en los ejes; el tornillito de la rueda contra la cara plana del eje."),
    ("", "8. Separadores en los 6 agujeros hexagonales."),
    ("", "9. Sube los cables de los motores por las aberturas del centro, sin tensión."),
    ("h", "Pruebas antes de poner el Medio"),
    ("", "✔ En una mesa plana las 4 ruedas tocan a la vez (si cojea: arandelas)."),
    ("", "✔ Gira cada rueda a mano: gira libre y no roza la placa."),
    ("", ["✔ Mide: rueda de 66 mm de diámetro y 17 cm entre centros.", "Si algo difiere más de 2 mm, pásame tus medidas."]),
    ("", "✔ Foto desde arriba antes de seguir."),
    ("h", "Todavía sin batería"),
    ("", ["La batería, el interruptor y el conector XT60 se ponen al final", "(diagrama 4, alimentación). Deja libre su espacio."]),
], size=27)
c.save(sys.argv[1])
