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

from bumperbot_digital_twin.analyze_log import analyze, json_safe

TAG_RE = re.compile(r"twin_\d{8}_\d{6}_(?P<tag>.+)\.csv$")
RUN_RE = re.compile(r"^(?P<exp>.+?)_r(?P<rep>\d+)(?:-\d+)?$")


def experiment_of(path):
    m = TAG_RE.search(os.path.basename(path))
    if not m:
        return None, None
    tag = m.group("tag")
    r = RUN_RE.match(tag)
    return (r.group("exp"), int(r.group("rep"))) if r else (tag, 1)


SERIOUS_ANOMALIES = {"atasco", "deslizamiento", "movimiento_no_comandado"}
DEFAULT_GAINS_TEXT = "1.5/6.0/3.0"


def protocol_steps(protocol):
    """Parámetros de cada experimento del protocolo (defaults + los propios)."""
    if not protocol:
        return {}
    defaults = dict(protocol.get("defaults") or {})
    steps = {}
    for item in protocol.get("experiments", []):
        if "name" in item and "calibrate_from" not in item and "tune" not in item:
            steps[item["name"]] = dict(defaults, **item)
    return steps


def latency_series(table, protocol=None):
    """Curvas de error contra retardo, usando solo experimentos comparables.

    Necesita el protocolo: sin él no se sabe qué recorrido hizo cada experimento y se
    mezclarían líneas, cuadrados y círculos. Se toman solo los experimentos del
    protocolo con corrección, sin gemelo calibrado, con las ganancias por defecto, sin
    fallas provocadas y con el mismo recorrido; se separan por líder y por compensación.
    La métrica es siempre el error real (rmse_real): el error del puente se mide contra
    las poses compensadas y favorecería a la compensación por construcción.
    """
    empty = {"metric": "rmse_real", "pattern": "", "series": {}}
    steps = protocol_steps(protocol)
    if not steps:
        return dict(empty, note="sin protocolo (-p) no se puede saber qué recorrido hizo cada experimento")
    eligible = []
    for exp, row in table.items():
        step = steps.get(exp)
        if step is None:
            continue
        if row["realimentacion"] != "sí" or row["modelo"] != "no":
            continue
        if row.get("ganancias", "—") not in ("—", DEFAULT_GAINS_TEXT) or step.get("gains") == "tuned":
            continue
        if step.get("pattern") == "free" or step.get("instructions"):
            continue
        if SERIOUS_ANOMALIES & set(row.get("anomalias", {})):
            continue
        if "," in str(row["lider"]):
            continue
        if not math.isfinite(row["rmse_real"][0]):
            continue
        eligible.append((exp, row, step.get("pattern", "square")))
    delayed_patterns = [pat for _, row, pat in eligible if row["red_ms"] > 0]
    if not delayed_patterns:
        return empty
    pattern = max(set(delayed_patterns), key=delayed_patterns.count)
    grouped = {}
    for exp, row, pat in eligible:
        if pat != pattern:
            continue
        grouped.setdefault((row["lider"], row["compensacion"]), {}).setdefault(row["red_ms"], []).append(
            (row["rmse_real"][0], exp))
    series = {}
    for k, by_delay in grouped.items():
        pts = [(d, float(np.mean([v for v, _ in vals])), [e for _, e in vals]) for d, vals in sorted(by_delay.items())]
        if len(pts) >= 2:
            series[k] = pts
    if len({p[0] for pts in series.values() for p in pts}) < 3:
        series = {}
    return {"metric": "rmse_real", "pattern": pattern, "series": series}


MIN_RUNS_FOR_TEST = 5


def _clean(x):
    x = np.asarray(x, float)
    return x[np.isfinite(x)]


def welch_t(a, b):
    """Prueba t de Welch de dos colas. Devuelve (t, gl, p).

    Sin scipy no se informa p: la aproximación normal da valores demasiado
    pequeños con los pocos grados de libertad de estas comparaciones.
    """
    a, b = _clean(a), _clean(b)
    if a.size < 2 or b.size < 2:
        return math.nan, math.nan, math.nan
    va, vb = a.var(ddof=1) / a.size, b.var(ddof=1) / b.size
    try:
        from scipy import stats as st
    except ImportError:
        st = None
    if va + vb == 0:
        diff = a.mean() - b.mean()
        t = math.copysign(math.inf, diff) if diff != 0 else 0.0
        p = math.nan if st is None else (0.0 if diff != 0 else 1.0)
        return t, math.nan, p
    t = (a.mean() - b.mean()) / math.sqrt(va + vb)
    df = (va + vb) ** 2 / (va ** 2 / (a.size - 1) + vb ** 2 / (b.size - 1))
    if st is None:
        return float(t), float(df), math.nan
    return float(t), float(df), float(2 * st.t.sf(abs(t), df))


def holm(pvalues):
    """p ajustados por Holm-Bonferroni (los NaN quedan NaN y no cuentan)."""
    idx = [i for i, p in enumerate(pvalues) if math.isfinite(p)]
    ordered = sorted(idx, key=lambda i: pvalues[i])
    m = len(ordered)
    adjusted = [math.nan] * len(pvalues)
    running = 0.0
    for rank, i in enumerate(ordered):
        running = max(running, min(1.0, (m - rank) * pvalues[i]))
        adjusted[i] = running
    return adjusted


def welch_ci(a, b, level=0.95):
    """Diferencia de medias (a − b) y su intervalo de confianza de Welch."""
    a, b = _clean(a), _clean(b)
    if a.size < 2 or b.size < 2:
        return math.nan, math.nan, math.nan
    diff = float(a.mean() - b.mean())
    va, vb = a.var(ddof=1) / a.size, b.var(ddof=1) / b.size
    se = math.sqrt(va + vb)
    if se == 0:
        return diff, diff, diff
    df = (va + vb) ** 2 / (va ** 2 / (a.size - 1) + vb ** 2 / (b.size - 1))
    try:
        from scipy import stats as st
    except ImportError:
        return diff, math.nan, math.nan
    half = float(st.t.ppf(0.5 + level / 2, df)) * se
    return diff, diff - half, diff + half


def compare(a, b):
    """Resumen de una comparación entre dos grupos de corridas."""
    na, nb = int(_clean(a).size), int(_clean(b).size)
    t, df, p = welch_t(a, b)
    diff, lo, hi = welch_ci(a, b)
    enough = na >= MIN_RUNS_FOR_TEST and nb >= MIN_RUNS_FOR_TEST
    if not enough:
        verdict = f"no concluyente (n < {MIN_RUNS_FOR_TEST})"
    elif not math.isfinite(p):
        verdict = "sin p"
    else:
        verdict = "sí" if p < 0.05 else "no"
    return {"n_a": na, "n_b": nb, "t": t, "gl": df, "p": p, "diferencia": diff, "ic95": [lo, hi],
            "concluyente": enough and math.isfinite(p), "significativa": verdict == "sí", "veredicto": verdict}


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
        for pair in comparisons:
            a_name, b_name = pair[0], pair[1]
            key = pair[2] if len(pair) > 2 else "rmse"
            missing = [n for n in (a_name, b_name) if n not in groups]
            if missing:
                results_cmp.append({"a": a_name, "b": b_name, "metrica": key, "media_a": math.nan,
                                    "media_b": math.nan, "n_a": len(groups.get(a_name, [])),
                                    "n_b": len(groups.get(b_name, [])), "t": math.nan, "gl": math.nan,
                                    "p": math.nan, "diferencia": math.nan, "ic95": [math.nan, math.nan],
                                    "concluyente": False, "significativa": False,
                                    "veredicto": "sin datos (no hay corridas de " + ", ".join(missing) + ")"})
                continue
            a = [metric(r, key) for r in groups[a_name]]
            b = [metric(r, key) for r in groups[b_name]]
            c = compare(a, b)
            if key == "rmse_real" and (c["n_a"] == 0 or c["n_b"] == 0):
                reasons = {r.get("error_real_sin_datos") for n in (a_name, b_name) for r in groups[n]}
                if "relojes" in reasons:
                    c["veredicto"] = "sin datos (relojes desincronizados)"
                elif "sin sellos" in reasons:
                    c["veredicto"] = "sin datos (sin sellos de tiempo)"
            results_cmp.append({"a": a_name, "b": b_name, "metrica": key,
                                "media_a": np.nanmean(a) if np.isfinite(a).any() else math.nan,
                                "media_b": np.nanmean(b) if np.isfinite(b).any() else math.nan, **c})
        # Holm-Bonferroni sobre las comparaciones concluyentes del protocolo
        adjusted = holm([c["p"] if c["concluyente"] else math.nan for c in results_cmp])
        for c, p_holm in zip(results_cmp, adjusted):
            c["p_holm"] = p_holm
            if c["concluyente"]:
                c["significativa"] = p_holm < 0.05
                c["veredicto"] = "sí" if c["significativa"] else "no"
        lines += ["", "## Comparaciones (prueba t de Welch, corrección de Holm)", "",
                  "| A | B | Métrica | A | B | Diferencia A − B | IC 95 % | n A/B | t | gl | p | p (Holm) "
                  "| ¿Diferencia significativa (α=0,05, Holm)? |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in results_cmp:
            name, scale, unit = METRICS[c["metrica"]]
            lo, hi = c["ic95"]
            ci = f"[{lo * scale:+.2f}; {hi * scale:+.2f}] {unit}" if math.isfinite(lo) else "—"
            num = (lambda v, f: format(v, f) if math.isfinite(v) else "—")
            lines.append(f"| {c['a']} | {c['b']} | {name} | {num(c['media_a'] * scale, '.2f')} {unit} "
                         f"| {num(c['media_b'] * scale, '.2f')} {unit} | {num(c['diferencia'] * scale, '+.2f')} {unit} "
                         f"| {ci} | {c['n_a']}/{c['n_b']} | {num(c['t'], '.2f')} | {num(c['gl'], '.1f')} "
                         f"| {num(c['p'], '.4f')} | {num(c['p_holm'], '.4f')} | {c['veredicto']} |")
        m = sum(1 for c in results_cmp if c["concluyente"])
        lines += ["", "Si el intervalo de confianza de la diferencia contiene el cero, los datos no muestran "
                  "una diferencia clara entre A y B. El p de Holm corrige por hacer varias comparaciones a la "
                  f"vez: sin corregir, con {m} comparaciones a α = 0,05 la probabilidad de al menos un falso "
                  f"positivo sería de hasta un {100 * (1 - 0.95 ** m):.0f} %."]
        if any(c["veredicto"].startswith("no concluyente") for c in results_cmp):
            lines += [f"Las comparaciones con menos de {MIN_RUNS_FOR_TEST} corridas por grupo se marcan como no "
                      "concluyentes: con tan pocas corridas, el valor de p depende más de cuánto se parecieron "
                      "esas corridas que del efecto."]

    warnings = []
    for exp, runs in groups.items():
        for r in runs:
            tag = f"{exp} r{r.get('repeticion', '?')}"
            comp = r.get("compensacion_sin_rtt", math.nan)
            if math.isfinite(comp) and comp > 0.1:
                warnings.append(f"{tag}: la compensación estuvo activa sin latencia medida en el {comp:.0%} de "
                                "la corrida (en ese tramo no compensó nada)")
            rtf = r.get("factor_tiempo_real", math.nan)
            if math.isfinite(rtf) and rtf < 0.95:
                warnings.append(f"{tag}: el gemelo corrió a {rtf:.2f} veces el tiempo real")
            off = r.get("desfase_relojes_ms", math.nan)
            if math.isfinite(off) and abs(off) > 10:
                warnings.append(f"{tag}: los relojes del robot y del PC difieren en {off:.0f} ms; el error real "
                                f"se desplaza unos {abs(off) * 0.15 / 10:.1f} cm a 0,15 m/s")
    if warnings:
        lines += ["", "## Avisos de calidad de los datos", ""] + [f"- {w}" for w in warnings]

    sweep = latency_series(table, protocol)
    if plots and sweep["series"]:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        colors = {"no": "#d62728", "sí": "#1f77b4"}
        styles = {"real": "-", "twin": "--"}
        for (leader, comp), pts in sorted(sweep["series"].items()):
            label = (f"{'Líder real' if leader == 'real' else 'Líder gemelo'}, "
                     f"{'con' if comp == 'sí' else 'sin'} compensación")
            ax.plot([p[0] for p in pts], [p[1] * 100 for p in pts], "o", ls=styles.get(leader, "-"),
                    color=colors.get(comp, "#444444"), label=label)
        ax.set_xlabel("Retardo nominal de la red emulada, un sentido [ms]")
        ax.set_ylabel(f"{METRICS[sweep['metric']][0]} [cm]")
        ax.set_title(f"Recorrido: {sweep['pattern']} · con corrección, sin modelo calibrado, ganancias por defecto",
                     fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "error_vs_latencia.png"), dpi=150)
        plt.close(fig)
        used = ", ".join(e for pts in sweep["series"].values() for p in pts for e in p[2])
        lines += ["", "## Error en función de la latencia", "",
                  f"Solo experimentos que difieren únicamente en el retardo ({used}); métrica: "
                  f"{METRICS[sweep['metric']][0]}.", "", "![Error vs latencia](error_vs_latencia.png)"]
    elif sweep.get("note"):
        lines += ["", f"No se dibuja el error en función de la latencia: {sweep['note']}."]

    anomalies = [(exp, row["anomalias"]) for exp, row in table.items() if row["anomalias"]]
    if anomalies:
        lines += ["", "## Anomalías detectadas", "", "| Experimento | Eventos |", "|---|---|"]
        for exp, a in anomalies:
            lines.append(f"| {exp} | " + ", ".join(f"{k} ×{v}" for k, v in a.items()) + " |")

    report_md = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, "informe.md"), "w") as f:
        f.write(report_md)
    with open(os.path.join(out_dir, "informe.json"), "w") as f:
        json.dump(json_safe({"experimentos": table, "comparaciones": results_cmp, "corridas": dict(groups)}),
                  f, indent=2, ensure_ascii=False, default=str)
    return report_md, table, results_cmp


def resolve_protocol_path(name):
    """Acepta una ruta a un YAML o el nombre de un protocolo incluido (protocolo_real)."""
    if os.path.isfile(name):
        return name
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("bumperbot_digital_twin")
    candidate = os.path.join(share, "config", name if name.endswith(".yaml") else name + ".yaml")
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(f"No encuentro el protocolo {name}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+",
                        help="Carpeta de una sesión de twin_experiment (usa solo sus corridas válidas), "
                             "carpetas de registros o archivos twin_*.csv")
    parser.add_argument("-p", "--protocol", help="YAML del protocolo (descripciones y comparaciones)")
    parser.add_argument("-o", "--out", default="informe_gemelo")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    paths = []
    for item in args.inputs:
        if os.path.isdir(item) and os.path.exists(os.path.join(item, "sesion.json")):
            from bumperbot_digital_twin.session import Session
            session = Session.load(item)
            paths += [r["registro"] for r in session.good_runs() if os.path.exists(r["registro"])]
            print(f"Sesión {item}: {len(session.good_runs())} corridas completas y válidas, "
                  f"{len(session.discarded())} descartadas")
        elif os.path.isdir(item):
            paths += glob.glob(os.path.join(item, "twin_*.csv"))
        else:
            paths.append(item)
    paths = [p for p in paths if not p.endswith(("_scan.csv", "_eventos.csv"))]
    protocol = None
    if args.protocol:
        import yaml
        with open(resolve_protocol_path(args.protocol)) as f:
            protocol = yaml.safe_load(f)
    else:
        print("Aviso: sin -p no se dibuja la figura de error contra latencia ni se hacen las comparaciones "
              "del protocolo. Ejemplo: twin_report ~/twin_logs -p protocolo_real")
    report_md, _, _ = build_report(paths, args.out, protocol, plots=not args.no_plots)
    print(report_md)
    print(f"Informe en {os.path.abspath(os.path.join(args.out, 'informe.md'))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
