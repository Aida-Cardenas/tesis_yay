#!/usr/bin/env python3
"""Analiza los registros del puente y produce las métricas y gráficas de la Prueba 5.

    ros2 run bumperbot_digital_twin analyze_twin_log ~/twin_logs/twin_*.csv -o resultados

Por cada corrida (twin_<fecha>_<etiqueta>.csv y sus archivos _scan.csv y _eventos.csv):
  - <nombre>_trayectorias.png  trayectoria del líder y del seguidor
  - <nombre>_error.png         error de posición (visto por el puente y real) y de orientación
  - <nombre>_velocidades.png   velocidad del líder y del seguidor (muestra el retardo)
  - <nombre>_latencia.png      histograma del tiempo de ida y vuelta de la red
y un resumen.md / resumen.json con todas las métricas.

Error "real" (sincronizado por reloj): el puente ve al robot real con el retraso de
la red. Usando el sello de tiempo de cada odometría, se reconstruye dónde estaba el
robot real en el mismo instante que el gemelo. Requiere relojes sincronizados
(chrony) entre la Raspberry Pi y el PC.
"""
import argparse
import csv
import json
import math
import os
from collections import Counter

import numpy as np

NUMERIC_TEXT = ("leader", "anomalies")


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} está vacío")
    data = {}
    for key in rows[0]:
        col = [r[key] for r in rows]
        if key in NUMERIC_TEXT:
            data[key] = np.array(col)
            continue
        try:
            data[key] = np.array([float(x) if x not in ("", None) else math.nan for x in col])
        except ValueError:
            data[key] = np.array(col)
    return data


def load_optional(path):
    if not os.path.exists(path):
        return None
    try:
        return load(path)
    except ValueError:
        return None


def finite(x):
    x = np.asarray(x, dtype=float)
    return x[np.isfinite(x)]


def stats(x):
    x = finite(x)
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
    leader_is_real = d["leader"] == "real"
    lv = np.where(leader_is_real, d["real_v"], d["twin_v"])
    fv = np.where(leader_is_real, d["twin_v"], d["real_v"])
    return lv, fv


def true_error(d):
    """Distancia entre el gemelo y el robot real en el mismo instante de reloj.

    Devuelve NaN si no hay sellos de tiempo del robot real o si los relojes no
    están sincronizados (diferencia mediana mayor a 2 s).
    """
    n = len(d["t"])
    out = np.full(n, math.nan)
    if "real_stamp" not in d:
        return out
    stamp = d["real_stamp"]
    ok = np.isfinite(stamp) & np.isfinite(d["real_x"]) & np.isfinite(d["twin_x"])
    if ok.sum() < 10 or abs(np.median(d["t"][ok] - stamp[ok])) > 2.0:
        return out
    s, idx = np.unique(stamp[ok], return_index=True)
    rx = d["real_x"][ok][idx]
    ry = d["real_y"][ok][idx]
    inside = ok & (d["t"] >= s[0]) & (d["t"] <= s[-1])
    tx = np.interp(d["t"][inside], s, rx)
    ty = np.interp(d["t"][inside], s, ry)
    out[inside] = np.hypot(d["twin_x"][inside] - tx, d["twin_y"][inside] - ty)
    return out


def flag(d, key):
    if key not in d:
        return "—"
    vals = finite(d[key])
    if vals.size == 0:
        return "—"
    return "sí" if np.mean(vals) > 0.5 else "no"


def analyze(path, out_dir, plots=True):
    d = load(path)
    t = d["t"] - d["t"][0]
    valid = (d["stale"] == 0) & np.isfinite(d["pos_error"])
    moving = valid & ((np.abs(d["ff_v"]) > 1e-3) | (np.abs(d["ff_w"]) > 1e-3) | (d["active"] == 1))
    lv, fv = speeds(d)
    err_true = true_error(d)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    base = os.path.splitext(path)[0]
    scan = load_optional(base + "_scan.csv")
    events = load_optional(base + "_eventos.csv")
    leaders = sorted(set(d["leader"].tolist()))
    summary = {
        "archivo": os.path.basename(path),
        "duracion_s": float(t[-1]),
        "muestras": int(len(t)),
        "lider": ", ".join(leaders),
        "realimentacion": flag(d, "feedback"),
        "compensacion": flag(d, "compensation"),
        "modelo_gemelo": flag(d, "twin_model"),
        "red_retardo_ms": float(np.nanmedian(d["net_delay_ms"])) if "net_delay_ms" in d else 0.0,
        "red_perdida": float(np.nanmedian(d["net_loss"])) if "net_loss" in d else 0.0,
        "error_posicion_m": stats(d["pos_error"][valid]),
        "error_posicion_en_movimiento_m": stats(d["pos_error"][moving]),
        "error_real_m": stats(err_true),
        "error_final_m": float(d["pos_error"][valid][-1]) if valid.any() else math.nan,
        "error_orientacion_deg": stats(np.degrees(np.abs(d["yaw_error"][valid]))),
        "rtt_ms": stats(d["rtt_ms"]),
        "edad_odom_real_ms": stats(d["real_odom_age_ms"]),
        "edad_cmd_lider_ms": stats(d["leader_cmd_age_ms"]),
        "retardo_sincronizacion_ms": float(estimate_lag(t, lv, fv) * 1000.0),
        "tiempo_desincronizado_s": float(np.sum(d["lost_sync"] == 1) * dt),
        "tiempo_datos_viejos_s": float(np.sum(d["stale"] == 1) * dt),
    }
    if scan is not None and len(scan["t"]):
        summary["lidar"] = {
            "comparaciones": int(len(scan["t"])),
            "mae_m": float(np.nanmean(scan["mae"])),
            "rmse_m": float(np.sqrt(np.nanmean(scan["rmse"] ** 2))),
            "sesgo_m": float(np.nanmean(scan["bias"])),
            "coincidencia_visibilidad": float(np.nanmean(scan["visibility_agreement"])),
        }
    if events is not None and len(events["t"]):
        starts = events["phase"] == "inicio" if events["phase"].dtype.kind in "US" else np.zeros(0, bool)
        kinds = events["kind"][starts] if starts.size else []
        summary["anomalias"] = dict(Counter(str(k) for k in kinds))
    if plots:
        make_plots(d, t, lv, fv, valid, err_true, path, out_dir)
    return summary


def make_plots(d, t, lv, fv, valid, err_true, path, out_dir):
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
    a1.plot(t[valid], d["pos_error"][valid] * 100.0, color="#1f77b4", label="Visto por el puente")
    if np.isfinite(err_true).any():
        a1.plot(t, err_true * 100.0, color="#2ca02c", lw=1, label="Real (mismo instante)")
        a1.legend(fontsize=8)
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
        "| Experimento | Líder | Realim. | Comp. | Modelo | Red [ms] | Error medio [cm] | RMSE [cm] | p95 [cm] "
        "| Máx [cm] | Error final [cm] | Error real RMSE [cm] | RTT medio [ms] | Retardo [ms] |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for s in summaries:
        e, r, tr = s["error_posicion_m"], s["rtt_ms"], s["error_real_m"]
        lines.append(
            f"| {s['archivo']} | {s['lider']} | {s['realimentacion']} | {s['compensacion']} | {s['modelo_gemelo']} "
            f"| {fmt(s['red_retardo_ms'], 1, 0)} | {fmt(e.get('media'), 100)} | {fmt(e.get('rmse'), 100)} "
            f"| {fmt(e.get('p95'), 100)} | {fmt(e.get('max'), 100)} | {fmt(s['error_final_m'], 100)} "
            f"| {fmt(tr.get('rmse'), 100)} | {fmt(r.get('media'))} | {fmt(s['retardo_sincronizacion_ms'], 1, 0)} |")
    lidar = [s for s in summaries if "lidar" in s]
    if lidar:
        lines += ["", "| Experimento | Comparaciones LiDAR | MAE [cm] | RMSE [cm] | Sesgo [cm] | Coincidencia de visibilidad |",
                  "|---|---|---|---|---|---|"]
        for s in lidar:
            li = s["lidar"]
            lines.append(f"| {s['archivo']} | {li['comparaciones']} | {fmt(li['mae_m'], 100)} | {fmt(li['rmse_m'], 100)} "
                         f"| {fmt(li['sesgo_m'], 100)} | {fmt(li['coincidencia_visibilidad'], 100)} % |")
    anomalies = [s for s in summaries if s.get("anomalias")]
    if anomalies:
        lines += ["", "| Experimento | Anomalías detectadas |", "|---|---|"]
        for s in anomalies:
            lines.append(f"| {s['archivo']} | " + ", ".join(f"{k} ×{v}" for k, v in s["anomalias"].items()) + " |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="Registros twin_*.csv del puente")
    parser.add_argument("-o", "--out", default="resultados_gemelo", help="Carpeta de salida")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    paths = [p for p in args.csv if not p.endswith(("_scan.csv", "_eventos.csv"))]
    os.makedirs(args.out, exist_ok=True)
    summaries = [analyze(p, args.out, plots=not args.no_plots) for p in paths]
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
