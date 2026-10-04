#!/usr/bin/env python3
"""Informe estadístico de una campaña de experimentos del gemelo digital.

    ros2 run bumperbot_digital_twin twin_report ~/twin_logs/campaña_x -p protocolo.yaml -o informe

Agrupa las corridas por experimento según su etiqueta (E3_r1, E3_r2... → E3),
calcula media y desviación estándar de cada métrica, compara pares de experimentos
con la prueba t de Welch y, si hay experimentos con distintas latencias de red,
dibuja el error en función de la latencia.
"""
import argparse
import glob
import json
import math
import os
import re
from collections import OrderedDict

import numpy as np

from bumperbot_digital_twin.analyze_log import analyze

TAG_RE = re.compile(r"twin_\d{8}_\d{6}_(?P<tag>.+)\.csv$")
RUN_RE = re.compile(r"^(?P<exp>.+?)_r(?P<rep>\d+)$")


def experiment_of(path):
    m = TAG_RE.search(os.path.basename(path))
    if not m:
        return None, None
    tag = m.group("tag")
    r = RUN_RE.match(tag)
    return (r.group("exp"), int(r.group("rep"))) if r else (tag, 1)


def welch_t(a, b):
    """Prueba t de Welch de dos colas. Devuelve (t, gl, p)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.size < 2 or b.size < 2:
        return math.nan, math.nan, math.nan
    va, vb = a.var(ddof=1) / a.size, b.var(ddof=1) / b.size
    if va + vb == 0:
        return math.inf if a.mean() != b.mean() else 0.0, math.nan, 0.0 if a.mean() != b.mean() else 1.0
    t = (a.mean() - b.mean()) / math.sqrt(va + vb)
    df = (va + vb) ** 2 / (va ** 2 / (a.size - 1) + vb ** 2 / (b.size - 1))
    try:
        from scipy import stats as st
        p = float(2 * st.t.sf(abs(t), df))
    except Exception:
        p = math.erfc(abs(t) / math.sqrt(2))
    return float(t), float(df), p


def metric(summary, key):
    s = summary
    if key == "rmse":
        return s["error_posicion_m"].get("rmse", math.nan)
    if key == "rmse_real":
        return s["error_real_m"].get("rmse", math.nan)
    if key == "max":
        return s["error_posicion_m"].get("max", math.nan)
    if key == "final":
        return s["error_final_m"]
    if key == "rtt":
        return s["rtt_ms"].get("media", math.nan)
    if key == "lag":
        return s["retardo_sincronizacion_ms"]
    if key == "lidar_mae":
        return s.get("lidar", {}).get("mae_m", math.nan)
    raise KeyError(key)


METRICS = OrderedDict([
    ("rmse", ("RMSE de posición", 100, "cm")),
    ("rmse_real", ("RMSE real (mismo instante)", 100, "cm")),
    ("max", ("Error máximo", 100, "cm")),
    ("final", ("Error final", 100, "cm")),
    ("rtt", ("RTT medio", 1, "ms")),
    ("lag", ("Retardo de sincronización", 1, "ms")),
    ("lidar_mae", ("MAE del LiDAR", 100, "cm")),
])


def mean_std(values):
    v = np.asarray([x for x in values if x is not None and math.isfinite(x)], float)
    if v.size == 0:
        return math.nan, math.nan, 0
    return float(v.mean()), float(v.std(ddof=1)) if v.size > 1 else 0.0, int(v.size)


def fmt_ms(m, s, scale, n):
    if not math.isfinite(m):
        return "—"
    return f"{m * scale:.1f} ± {s * scale:.1f}" if n > 1 else f"{m * scale:.1f}"


def build_report(csv_paths, out_dir, protocol=None, plots=True):
    os.makedirs(out_dir, exist_ok=True)
    groups = OrderedDict()
    for path in sorted(csv_paths):
        exp, rep = experiment_of(path)
        if exp is None:
            continue
        summary = analyze(path, out_dir, plots=plots)
        summary["repeticion"] = rep
        groups.setdefault(exp, []).append(summary)
    if not groups:
        raise ValueError("No se encontraron corridas twin_*_<experimento>_r<n>.csv")

    descriptions = {}
    comparisons = []
    if protocol:
        for e in protocol.get("experiments", []):
            descriptions[e["name"]] = e.get("description", "")
        comparisons = protocol.get("comparisons", [])

    table = OrderedDict()
    for exp, runs in groups.items():
        first = runs[0]
        row = OrderedDict(
            experimento=exp, descripcion=descriptions.get(exp, ""), corridas=len(runs),
            lider=first["lider"], realimentacion=first["realimentacion"], compensacion=first["compensacion"],
            modelo=first["modelo_gemelo"], ganancias=first.get("ganancias", "—"),
            red_ms=first["red_retardo_ms"], perdida=first["red_perdida"])
        for key in METRICS:
            row[key] = mean_std(metric(r, key) for r in runs)
        anomalies = {}
        for r in runs:
            for k, v in r.get("anomalias", {}).items():
                anomalies[k] = anomalies.get(k, 0) + v
        row["anomalias"] = anomalies
        table[exp] = row

    lines = ["# Informe de experimentos del gemelo digital", "",
             f"{sum(len(r) for r in groups.values())} corridas en {len(groups)} experimentos. "
             "Valores: media ± desviación estándar entre repeticiones.", ""]
    head = "| Experimento | Descripción | n | Líder | Realim. | Comp. | Modelo | Ganancias kx/ky/kθ | Red [ms] | " + \
        " | ".join(f"{name} [{unit}]" for name, _, unit in METRICS.values()) + " |"
    lines += [head, "|" + "---|" * (9 + len(METRICS))]
    for exp, row in table.items():
        cells = [fmt_ms(*row[k][:2], METRICS[k][1], row[k][2]) for k in METRICS]
        lines.append(f"| {exp} | {row['descripcion']} | {row['corridas']} | {row['lider']} | {row['realimentacion']} "
                     f"| {row['compensacion']} | {row['modelo']} | {row['ganancias']} | {row['red_ms']:.0f} | "
                     + " | ".join(cells) + " |")

    results_cmp = []
    if comparisons:
        lines += ["", "## Comparaciones (prueba t de Welch sobre el RMSE de posición)", "",
                  "| A | B | RMSE A [cm] | RMSE B [cm] | Diferencia [cm] | t | gl | p | ¿Significativa (α=0,05)? |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for pair in comparisons:
            a_name, b_name = pair[0], pair[1]
            key = pair[2] if len(pair) > 2 else "rmse"
            if a_name not in groups or b_name not in groups:
                continue
            a = [metric(r, key) for r in groups[a_name]]
            b = [metric(r, key) for r in groups[b_name]]
            t, df, p = welch_t(a, b)
            ma, mb = np.nanmean(a), np.nanmean(b)
            sig = "sí" if math.isfinite(p) and p < 0.05 else "no"
            results_cmp.append({"a": a_name, "b": b_name, "metrica": key, "media_a": ma, "media_b": mb,
                                "t": t, "gl": df, "p": p, "significativa": sig == "sí"})
            lines.append(f"| {a_name} | {b_name} | {ma * 100:.2f} | {mb * 100:.2f} | {(ma - mb) * 100:+.2f} "
                         f"| {t:.2f} | {df:.1f} | {p:.4f} | {sig} |")

    sweep = [(row["red_ms"], row["compensacion"], row["rmse_real"][0] if math.isfinite(row["rmse_real"][0])
              else row["rmse"][0], exp) for exp, row in table.items()]
    delays = sorted({s[0] for s in sweep})
    if plots and len(delays) >= 3:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        for comp, color, label in (("no", "#d62728", "Sin compensación"), ("sí", "#1f77b4", "Con compensación")):
            pts = sorted((d, e) for d, c, e, _ in sweep if c == comp and math.isfinite(e))
            if pts:
                ax.plot([p[0] for p in pts], [p[1] * 100 for p in pts], "o-", color=color, label=label)
        ax.set_xlabel("Retardo de red emulado, un sentido [ms]")
        ax.set_ylabel("RMSE de posición [cm]")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "error_vs_latencia.png"), dpi=150)
        plt.close(fig)
        lines += ["", "## Error en función de la latencia", "", "![Error vs latencia](error_vs_latencia.png)"]

    anomalies = [(exp, row["anomalias"]) for exp, row in table.items() if row["anomalias"]]
    if anomalies:
        lines += ["", "## Anomalías detectadas", "", "| Experimento | Eventos |", "|---|---|"]
        for exp, a in anomalies:
            lines.append(f"| {exp} | " + ", ".join(f"{k} ×{v}" for k, v in a.items()) + " |")

    report_md = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, "informe.md"), "w") as f:
        f.write(report_md)
    with open(os.path.join(out_dir, "informe.json"), "w") as f:
        json.dump({"experimentos": table, "comparaciones": results_cmp,
                   "corridas": {k: v for k, v in groups.items()}}, f, indent=2, ensure_ascii=False, default=str)
    return report_md, table, results_cmp


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", help="Carpetas o archivos twin_*.csv")
    parser.add_argument("-p", "--protocol", help="YAML del protocolo (descripciones y comparaciones)")
    parser.add_argument("-o", "--out", default="informe_gemelo")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    paths = []
    for item in args.inputs:
        if os.path.isdir(item):
            paths += glob.glob(os.path.join(item, "twin_*.csv"))
        else:
            paths.append(item)
    paths = [p for p in paths if not p.endswith(("_scan.csv", "_eventos.csv"))]
    protocol = None
    if args.protocol:
        import yaml
        with open(args.protocol) as f:
            protocol = yaml.safe_load(f)
    report_md, _, _ = build_report(paths, args.out, protocol, plots=not args.no_plots)
    print(report_md)
    print(f"Informe en {os.path.abspath(os.path.join(args.out, 'informe.md'))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
