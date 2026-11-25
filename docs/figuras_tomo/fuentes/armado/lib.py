import base64, os
FZ = os.environ.get("FZ_PARTS", "")
CURSO = os.environ.get("CURSO_PARTS", "")
PARTS = {
    "nano": ("core", "Arduino Nano3_breadboard.svg", (50.4, 127.584)),
    "lm2596": ("core", "RioRand_LM2596_94ce9810d0922bc731e2f315085d46ab_1_breadboard.svg", (181.459, 86.917)),
    "mini": ("core", "miniBreadboard.svg", (129.839, 100.914)),
    "pi": ("core", "raspberry-pi-4B_1_breadboard.svg", (329.28, 218.949)),
    "imu": ("core", "MPU6050_GY521_9.svg", (58.809, 76.573)),
    "switch": ("core", "basic_toggle_switch.svg", (35.518, 36.088)),
    "l298n": ("curso", "svg.breadboard.L298N motor driver_ed29594eba4fbd330bd92441a17764ea_17_breadboard.svg", (161.833, 160.132)),
    "xt60m": ("curso", "svg.breadboard.XT60-Connector-male_1_breadboard.svg", (97.838, 58.583)),
    "xt60f": ("curso", "svg.breadboard.XT60-Connector-female_1_breadboard.svg", (97.838, 58.583)),
}
PINS = {
    "nano": {"TX": (3.6, 10.8), "RX": (3.6, 18.0), "GND_L": (3.6, 32.4), "D2": (3.6, 39.6), "D3": (3.6, 46.8), "D4": (3.6, 54.0), "D5": (3.6, 61.2), "D6": (3.6, 68.4), "D7": (3.6, 75.6), "D8": (3.6, 82.8), "D9": (3.6, 90.0), "D10": (3.6, 97.2), "D11": (3.6, 104.4), "D12": (3.6, 111.6), "VIN": (46.8, 10.8), "GND_R": (46.8, 18.0), "5V": (46.8, 32.4), "D13": (46.8, 111.6)},
    "lm2596": {"IN+": (4.8, 4.837), "IN-": (4.8, 81.637), "OUT+": (177.6, 4.837), "OUT-": (177.6, 81.637)},
    "pi": {"P1": (31.62, 18.03), "P2": (31.62, 8.43), "P3": (41.22, 18.03), "P5": (50.82, 18.03), "P6": (50.82, 8.43), "USB3a": (325, 92), "USB3b": (325, 118), "USB2a": (325, 160), "USB2b": (325, 186), "PWR": (43, 216)},
    "imu": {"VCC": (4.8, 4.8), "GND": (4.8, 14.39), "SCL": (4.8, 23.99), "SDA": (4.8, 33.59)},
    "switch": {"L1": (7.0, 31.72), "COM": (16.6, 31.72), "L2": (26.2, 31.72)},
    "l298n": {"OUT1": (1.43, 98.75), "OUT2": (1.43, 116.70), "OUT3": (160.5, 113.17), "OUT4": (160.5, 95.23), "12V": (33.57, 158.8), "GND": (51.52, 158.8), "5V": (69.46, 158.8), "ENA": (89.44, 148.98), "IN1": (99.04, 148.98), "IN2": (108.64, 148.98), "IN3": (118.24, 148.98), "IN4": (127.84, 148.98), "ENB": (137.44, 148.98), "J5VEN": (52.46, 114.49), "JENA": (89.44, 144.2), "JENB": (137.44, 144.2)},
    "xt60m": {"GND": (1.22, 18.53), "VCC": (1.22, 37.73)},
    "xt60f": {"GND": (96.62, 20.85), "VCC": (96.62, 40.05)},
}
def hole(col, row):
    rows = dict(A=10.86, B=18.06, C=25.26, D=32.46, E=39.66, F=61.26, G=68.46, H=75.66, I=82.86, J=90.06)
    return (7.32 + 7.2 * (col - 1), rows[row])
def _data(kind):
    where, fn, _ = PARTS[kind]
    path = os.path.join(FZ, fn) if where == "core" else os.path.join(CURSO, fn)
    return "data:image/svg+xml;base64," + base64.b64encode(open(path, "rb").read()).decode()
class Canvas:
    def __init__(s, W, H, S=3.0):
        s.W, s.H, s.S = W, H, S
        s.bg, s.parts, s.wires, s.top = [], [], [], []
        s.placed = {}
    def place(s, name, kind, x, y, rot=0, scale=None):
        S = scale or s.S
        w, h = PARTS[kind][2]
        if rot == 0:
            tr = f"translate({x},{y}) scale({S})"
        else:
            tr = f"translate({x},{y}) scale({S}) translate(0,{w}) rotate(-90)"
        s.parts.append(f'<g transform="{tr}"><image href="{_data(kind)}" x="0" y="0" width="{w}" height="{h}"/></g>')
        s.placed[name] = (kind, x, y, rot, S)
    def p(s, name, pin):
        kind, x, y, rot, S = s.placed[name]
        if kind == "mini":
            px, py = hole(*pin)
        else:
            px, py = PINS[kind][pin]
        w, h = PARTS[kind][2]
        if rot:
            px, py = py, w - px
        return (x + px * S, y + py * S)
    def wire(s, pts, color, w=7, dots=True, edge="#00000055"):
        d = "M" + " L".join(f"{a:.1f},{b:.1f}" for a, b in pts)
        s.wires.append(f'<path d="{d}" fill="none" stroke="{edge}" stroke-width="{w+3}" stroke-linejoin="round" stroke-linecap="round"/>')
        s.wires.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"/>')
        if dots:
            for a, b in (pts[0], pts[-1]):
                s.top.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="{w*0.75:.1f}" fill="{color}" stroke="#222" stroke-width="1.5"/>')
    def dot(s, x, y, c, r=9):
        s.top.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{c}" stroke="#222" stroke-width="1.5"/>')
    def rect(s, x, y, w, h, fill, stroke="#424242", rx=10, sw=3, layer="bg", dash=None):
        da = f' stroke-dasharray="{dash}"' if dash else ""
        getattr(s, layer).append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{da}/>')
    def circle(s, x, y, r, fill, stroke="#424242", sw=3, layer="bg"):
        getattr(s, layer).append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')
    def raw(s, txt, layer="top"):
        getattr(s, layer).append(txt)
    def text(s, x, y, t, size=26, anchor="start", color="#212121", bold=False, layer="top", italic=False):
        b = ' font-weight="bold"' if bold else ""
        it = ' font-style="italic"' if italic else ""
        getattr(s, layer).append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{color}"{b}{it}>{t}</text>')
    def tag(s, x, y, t, color, anchor="start", size=24):
        w = len(t) * size * 0.56 + 18
        xx = x if anchor == "start" else (x - w if anchor == "end" else x - w / 2)
        s.top.append(f'<rect x="{xx:.1f}" y="{y-size*0.95:.1f}" width="{w:.1f}" height="{size*1.35:.1f}" rx="7" fill="#fff" stroke="{color}" stroke-width="3"/>')
        s.top.append(f'<text x="{xx+9:.1f}" y="{y:.1f}" font-size="{size}" fill="#212121" font-weight="bold">{t}</text>')
    def steps(s, x, y, w, title, items, size=25, lh=1.38):
        out = [f'<text x="{x}" y="{y}" font-size="{size+9}" font-weight="bold" fill="#212121">{title}</text>']
        yy = y + size * 2.0
        for kind, t in items:
            if kind == "h":
                yy += size * 0.6
                out.append(f'<text x="{x}" y="{yy}" font-size="{size+3}" font-weight="bold" fill="#BF360C">{t}</text>')
                yy += size * lh * 1.1
                continue
            lines = t if isinstance(t, list) else [t]
            for i, ln in enumerate(lines):
                ind = 0 if i == 0 else 42
                out.append(f'<text x="{x+ind}" y="{yy}" font-size="{size}" fill="#212121" xml:space="preserve">{ln}</text>')
                yy += size * lh
            yy += size * 0.25
        s.top.extend(out)
        return yy
    def save(s, path):
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{s.W}" height="{s.H}" viewBox="0 0 {s.W} {s.H}">'
               f'<style>text{{font-family:"Liberation Sans",Arial,sans-serif}}</style><rect width="{s.W}" height="{s.H}" fill="#fff"/>'
               + "".join(s.bg) + "".join(s.parts) + "".join(s.wires) + "".join(s.top) + "</svg>")
        open(path, "w").write(svg)

def _bez(p0, d0, p1, d1, k0, k1):
    c0 = (p0[0] + d0[0] * k0, p0[1] + d0[1] * k0)
    c1 = (p1[0] + d1[0] * k1, p1[1] + d1[1] * k1)
    return f"M{p0[0]:.1f},{p0[1]:.1f} C{c0[0]:.1f},{c0[1]:.1f} {c1[0]:.1f},{c1[1]:.1f} {p1[0]:.1f},{p1[1]:.1f}"
def curve(self, p0, d0, p1, d1, color, w=7, k0=160, k1=160, dots=True, edge="#00000066"):
    dd = _bez(p0, d0, p1, d1, k0, k1)
    self.wires.append(f'<path d="{dd}" fill="none" stroke="{edge}" stroke-width="{w+3}" stroke-linecap="round"/>')
    self.wires.append(f'<path d="{dd}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linecap="round"/>')
    if dots:
        for a, b in (p0, p1):
            self.top.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="{w*0.7:.1f}" fill="{color}" stroke="#222" stroke-width="1.5"/>')
Canvas.curve = curve
def cable_end(self, x, y, colors, labels, title, pitch=26, up=True):
    n = len(colors)
    w = pitch * (n - 1) + 40
    self.rect(x - 20, y, w, 46, "#37474F", stroke="#263238", rx=8, sw=2, layer="top")
    self.text(x - 20 + w / 2, y + 82, title, 24, "middle", bold=True)
    pts = []
    for i, (col, lb) in enumerate(zip(colors, labels)):
        xx = x + i * pitch
        self.top.append(f'<line x1="{xx}" y1="{y}" x2="{xx}" y2="{y-18}" stroke="#00000088" stroke-width="10"/>')
        self.top.append(f'<line x1="{xx}" y1="{y}" x2="{xx}" y2="{y-18}" stroke="{col}" stroke-width="7"/>')
        self.top.append(f'<text x="{xx}" y="{y+32}" font-size="17" fill="#fff" text-anchor="middle" font-weight="bold">{lb}</text>')
        pts.append((xx, y - 18))
    return pts
Canvas.cable_end = cable_end
