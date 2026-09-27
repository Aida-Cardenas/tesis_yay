#!/usr/bin/env python3
"""Analiza los registros CSV del puente y produce las métricas y gráficas de la Prueba 5.

    ros2 run bumperbot_digital_twin analyze_twin_log ~/twin_logs/twin_*.csv -o resultados

Por cada CSV genera:
  - <nombre>_trayectorias.png  trayectoria del líder y del seguidor
  - <nombre>_error.png         error de posición y de orientación en el tiempo
  - <nombre>_velocidades.png   velocidad del líder y del seguidor (muestra el retardo)
  - <nombre>_latencia.png      histograma del tiempo de ida y vuelta de la red
y un resumen.md / resumen.json con todas las métricas, listo para copiar a la tesis.
"""
import argparse
import csv
import json
import math
import os

import numpy as np


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} está vacío")
    data = {}
    for key in rows[0]:
        col = [r[key] for r in rows]
        try:
            data[key] = np.array([float(x) if x not in ("", None) else math.nan for x in col])
        except ValueError:
            data[key] = np.array(col)
    return data


def finite(x):
    return x[np.isfinite(x)]


def stats(x):
    x = finite(np.asarray(x, dtype=float))
    if x.size == 0:
        return {"n": 0}
    return {
        "n": int(x.size),
        "media": float(np.mean(x)),
        "rmse": float(np.sqrt(np.mean(x ** 2))),
        "p50": float(np.percentile(x, 50)),
        "p95": float(np.percentile(x, 95)),
        "max": float(np.max(x)),
    }


def estimate_lag(t, leader_speed, follower_speed, max_lag=1.5):
    """Retardo [s] que maximiza la correlación entre las velocidades del líder y del seguidor."""
    ok = np.isfinite(t) & np.isfinite(leader_speed) & np.isfinite(follower_speed)
    t, a, b = t[ok], leader_speed[ok], follower_speed[ok]
    if t.size < 20 or np.std(a) < 1e-6 or np.std(b) < 1e-6:
        return math.nan
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return math.nan
    grid = np.arange(t[0], t[-1], dt)
    a = np.interp(grid, t, a) - np.mean(a)
    b = np.interp(grid, t, b) - np.mean(b)
    best, best_lag = -np.inf, math.nan
    for k in range(0, int(max_lag / dt) + 1):
        if k >= len(a) - 10:
            break
        c = np.dot(a[:len(a) - k], b[k:]) / (np.linalg.norm(a[:len(a) - k]) * np.linalg.norm(b[k:]) + 1e-12)
        if c > best:
            best, best_lag = c, k * dt
    return best_lag


def speeds(d):
    """Velocidad lineal del líder y del seguidor según quién era líder en cada fila."""
    leader_is_real = d["leader"] == "real"
    lv = np.where(leader_is_real, d["real_v"], d["twin_v"])
    fv = np.where(leader_is_real, d["twin_v"], d["real_v"])
    return lv, fv


def analyze(path, out_dir, plots=True):
    d = load(path)
    t = d["t"] - d["t"][0]
    valid = (d["stale"] == 0) & np.isfinite(d["pos_error"])
    moving = valid & ((np.abs(d["ff_v"]) > 1e-3) | (np.abs(d["ff_w"]) > 1e-3) | (d["active"] == 1))
    lv, fv = speeds(d)
    leaders = sorted(set(d["leader"].tolist()))
    summary = {
        "archivo": os.path.basename(path),
        "duracion_s": float(t[-1]),
        "muestras": int(len(t)),
        "lider": ", ".join(leaders),
        "realimentacion": "sí" if np.nanmean(d["feedback"]) > 0.5 else "no",
        "error_posicion_m": stats(d["pos_error"][valid]),
        "error_posicion_en_movimiento_m": stats(d["pos_error"][moving]),
        "error_final_m": float(d["pos_error"][valid][-1]) if valid.any() else math.nan,
        "error_orientacion_deg": stats(np.degrees(np.abs(d["yaw_error"][valid]))),
        "rtt_ms": stats(d["rtt_ms"]),
        "edad_odom_real_ms": stats(d["real_odom_age_ms"]),
        "edad_cmd_lider_ms": stats(d["leader_cmd_age_ms"]),
        "retardo_sincronizacion_ms": float(estimate_lag(t, lv, fv) * 1000.0),
        "tiempo_desincronizado_s": float(np.sum(d["lost_sync"] == 1) * np.median(np.diff(t))) if len(t) > 1 else 0.0,
        "tiempo_datos_viejos_s": float(np.sum(d["stale"] == 1) * np.median(np.diff(t))) if len(t) > 1 else 0.0,
    }
    if plots:
        make_plots(d, t, lv, fv, valid, path, out_dir)
    return summary


def make_plots(d, t, lv, fv, valid, path, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    base = os.path.join(out_dir, os.path.splitext(os.path.basename(path))[0])

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(d["leader_x"][valid], d["leader_y"][valid], label="Líder", color="#d62728", lw=2)
    ax.plot(d["follower_x"][valid], d["follower_y"][valid], label="Seguidor", color="#1f77b4", lw=1.5, ls="--")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("Trayectorias (marco odom del seguidor)")
    fig.tight_layout()
    fig.savefig(base + "_trayectorias.png", dpi=150)
    plt.close(fig)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    a1.plot(t[valid], d["pos_error"][valid] * 100.0, color="#1f77b4")
    a1.set_ylabel("Error de posición [cm]")
    a1.grid(alpha=0.3)
    a2.plot(t[valid], np.degrees(d["yaw_error"][valid]), color="#ff7f0e")
    a2.set_ylabel("Error de orientación [°]")
    a2.set_xlabel("Tiempo [s]")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(base + "_error.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(t, lv, label="Líder", color="#d62728")
    ax.plot(t, fv, label="Seguidor", color="#1f77b4", ls="--")
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Velocidad lineal [m/s]")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(base + "_velocidades.png", dpi=150)
    plt.close(fig)

    rtt = finite(d["rtt_ms"])
    if rtt.size:
        fig, ax = plt.subplots(figsize=(6, 3.5))
        ax.hist(rtt, bins=40, color="#2ca02c")
        ax.set_xlabel("Tiempo de ida y vuelta [ms]")
        ax.set_ylabel("Muestras")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(base + "_latencia.png", dpi=150)
        plt.close(fig)


def fmt(x, scale=1.0, digits=1):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    return f"{x * scale:.{digits}f}"


def markdown(summaries):
    lines = [
        "| Experimento | Líder | Realim. | Duración [s] | Error medio [cm] | RMSE [cm] | p95 [cm] | Máx [cm] "
        "| Error final [cm] | Orientación RMSE [°] | RTT medio [ms] | RTT p95 [ms] | Retardo [ms] |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for s in summaries:
        e = s["error_posicion_m"]
        o = s["error_orientacion_deg"]
        r = s["rtt_ms"]
        lines.append(
            f"| {s['archivo']} | {s['lider']} | {s['realimentacion']} | {fmt(s['duracion_s'])} "
            f"| {fmt(e.get('media'), 100)} | {fmt(e.get('rmse'), 100)} | {fmt(e.get('p95'), 100)} "
            f"| {fmt(e.get('max'), 100)} | {fmt(s['error_final_m'], 100)} | {fmt(o.get('rmse'))} "
            f"| {fmt(r.get('media'))} | {fmt(r.get('p95'))} | {fmt(s['retardo_sincronizacion_ms'], 1, 0)} |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="Registros twin_*.csv del puente")
    parser.add_argument("-o", "--out", default="resultados_gemelo", help="Carpeta de salida")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    summaries = [analyze(p, args.out, plots=not args.no_plots) for p in args.csv]
    with open(os.path.join(args.out, "resumen.json"), "w") as f:
        json.dump(summaries, f, indent=2, ensure_ascii=False)
    table = markdown(summaries)
    with open(os.path.join(args.out, "resumen.md"), "w") as f:
        f.write(table)
    print(table)
    print(f"Gráficas y resumen en {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
