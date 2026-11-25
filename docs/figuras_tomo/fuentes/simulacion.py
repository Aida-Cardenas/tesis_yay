import sys, json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

src, out = Path(sys.argv[1]), Path(sys.argv[2])

a = Image.open(next(src.glob("*_S2_r1_trayectorias.png"))).convert("RGB")
b = Image.open(next(src.glob("*_S1_r1_trayectorias.png"))).convert("RGB")
strip = 80
canvas = Image.new("RGB", (a.width + b.width, a.height + strip), "white")
canvas.paste(a, (0, 0))
canvas.paste(b, (a.width, 0))
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 34)
draw = ImageDraw.Draw(canvas)
for x0, label in ((0, "(a) Sin corrección (S2)"), (a.width, "(b) Con corrección (S1)")):
    w = draw.textlength(label, font=font)
    draw.text((x0 + (a.width - w) / 2, a.height + 15), label, fill="black", font=font)
canvas.save(out / "trayectorias_cuadrado.png")

Image.open(src / "modelo_gemelo_ajuste.png").convert("RGB").save(out / "modelo_gemelo_ajuste.png")

g = Image.open(src / "ganancias.png").convert("RGB")
g.crop((0, 40, g.width, g.height)).save(out / "ganancias_sintonizacion.png")

exp = json.load(open(src / "informe.json"))["experimentos"]
def cm(k):
    return exp[k]["rmse_real"][0] * 100
comma = FuncFormatter(lambda v, _: f"{v:g}".replace(".", ","))
fig, ax = plt.subplots(figsize=(7, 4), dpi=200)
ax.plot([0, 150, 300], [cm("L0"), cm("L150"), cm("L300")], "o-", color="#d62728", label="Sin compensación")
ax.plot([150, 300], [cm("L150C"), cm("L300C")], "s--", color="#1f77b4", label="Con compensación")
for x, k, dy in ((0, "L0", -16), (150, "L150", -16), (300, "L300", 9), (150, "L150C", 9), (300, "L300C", -16)):
    color = "#1f77b4" if k.endswith("C") else "#d62728"
    ax.annotate(f"{cm(k):.1f}".replace(".", ","), (x, cm(k)), textcoords="offset points", xytext=(0, dy), ha="center", fontsize=9, color=color)
ax.set_xlabel("Retardo de la red emulada, en cada sentido [ms]")
ax.set_ylabel("RMSE real entre los robots [cm]")
ax.set_xticks([0, 150, 300])
ax.set_xlim(-25, 325)
ax.set_ylim(0, 2.5)
ax.yaxis.set_major_formatter(comma)
ax.grid(alpha=0.3)
ax.legend(loc="upper left")
fig.tight_layout()
fig.savefig(out / "error_vs_latencia.png")
