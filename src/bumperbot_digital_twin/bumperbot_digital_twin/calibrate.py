#!/usr/bin/env python3
"""Calibración del gemelo digital a partir del robot real.

    ros2 run bumperbot_digital_twin twin_calibrate dynamics ~/twin_logs/twin_*_E1*.csv -o ~/modelo_robot.yaml
    ros2 run bumperbot_digital_twin twin_calibrate wheels --odom-distance 0.97 --real-distance 1.00 \\
         --odom-angle 6.10 --real-angle 6.283 -o ~/calibracion_ruedas.yaml
    ros2 run bumperbot_digital_twin twin_calibrate arena --bounds -0.5 1.5 -0.5 1.5 -o arena.world

dynamics: identifica K, τ y L de la velocidad lineal y angular del robot real a partir
  de los registros del puente (comando del líder real frente a la velocidad medida).
  El YAML resultante se usa con twin_model_file (puente) y model_file (detector).
wheels:   calcula los multiplicadores de radio y separación de ruedas de
  diff_drive_controller a partir de mediciones con cinta métrica. El YAML se pasa a
  bumperbot_bringup con calibration_file.
arena:    genera un mundo de Gazebo con un recinto rectangular igual al de tus
  pruebas, para comparar el LiDAR real con el simulado.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

from bumperbot_digital_twin.dynamics import TwinModel, fit_error, fit_first_order, simulate, wheel_calibration
from bumperbot_digital_twin.scan_compare import arena_world_sdf


def read_identification_data(paths):
    """Concatena (t, comando, velocidad medida) del robot real de los CSV del puente."""
    chunks = {"linear": [], "angular": []}
    for path in paths:
        with open(path, newline="") as f:
            rows = [r for r in csv.DictReader(f) if r.get("leader") == "real" and r.get("stale") == "0"]
        if len(rows) < 20:
            continue
        t = np.array([float(r["t"]) for r in rows])
        for key, u, y in (("linear", "ff_v", "real_v"), ("angular", "ff_w", "real_w")):
            chunks[key].append((t, np.array([float(r[u]) for r in rows]), np.array([float(r[y]) for r in rows])))
    return chunks


def fit_channel(chunks, max_delay):
    """Ajusta un modelo con todas las corridas, separadas por huecos para no mezclar dinámicas."""
    if not chunks:
        raise ValueError("No hay filas con líder real en los registros")
    offset = 0.0
    T, U, Y = [], [], []
    for t, u, y in chunks:
        t = t - t[0] + offset
        T.append(t)
        U.append(u)
        Y.append(y)
        offset = t[-1] + 5.0
        T.append(np.arange(t[-1] + 0.05, offset, 0.05))
        U.append(np.zeros_like(T[-1]))
        Y.append(np.zeros_like(T[-1]))
    t, u, y = np.concatenate(T), np.concatenate(U), np.concatenate(Y)
    model = fit_first_order(t, u, y, max_delay=max_delay)
    return model, t, u, y


def plot_fit(model, data, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5))
    for ax, (name, (m, t, u, y)) in zip(axes, data.items()):
        ax.plot(t, u, color="#999999", lw=1, label="Comando")
        ax.plot(t, y, color="#d62728", lw=1.2, label="Robot real")
        ax.plot(t, simulate(m, t, u), color="#1f77b4", lw=1.2, ls="--", label="Modelo identificado")
        unit = "m/s" if name == "linear" else "rad/s"
        ax.set_ylabel(f"{'Lineal' if name == 'linear' else 'Angular'} [{unit}]")
        ax.set_title(f"K={m.gain:.3f}  τ={m.tau:.3f} s  L={m.delay:.3f} s  R²={m.r2:.3f}", fontsize=10)
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Tiempo [s]")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def cmd_dynamics(args):
    chunks = read_identification_data(args.csv)
    model = TwinModel(source=", ".join(os.path.basename(p) for p in args.csv))
    data = {}
    for name in ("linear", "angular"):
        try:
            m, t, u, y = fit_channel(chunks[name], args.max_delay)
        except ValueError as exc:
            print(f"{name}: {exc}; se deja el modelo ideal (K=1, τ=0, L=0)")
            continue
        m.samples = int(len(t))
        setattr(model, name, m)
        data[name] = (m, t, u, y)
        print(f"{name:8s} K={m.gain:.3f}  τ={m.tau:.3f} s  L={m.delay:.3f} s  R²={m.r2:.3f}  "
              f"RMSE={fit_error(m, t, u, y):.4f}")
    model.save(args.output)
    print(f"Modelo guardado en {args.output}")
    if data and not args.no_plot:
        png = os.path.splitext(args.output)[0] + "_ajuste.png"
        plot_fit(model, data, png)
        print(f"Gráfica del ajuste en {png}")
    return 0


def cmd_wheels(args):
    import yaml
    values = wheel_calibration(args.odom_distance, args.real_distance, args.odom_angle, args.real_angle)
    with open(args.output, "w") as f:
        yaml.safe_dump({"bumperbot_controller": {"ros__parameters": values}}, f, sort_keys=False)
    for k, v in values.items():
        print(f"{k} = {v:.4f}")
    print(f"Guardado en {args.output}. Úsalo con: calibration_file:={os.path.abspath(args.output)}")
    return 0


def cmd_arena(args):
    obstacles = [tuple(args.obstacle[i:i + 4]) for i in range(0, len(args.obstacle or []), 4)]
    with open(args.output, "w") as f:
        f.write(arena_world_sdf(args.bounds, args.wall_height, args.wall_thickness, obstacles))
    print(f"Mundo guardado en {args.output}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    d = sub.add_parser("dynamics", help="Identifica la dinámica del robot real")
    d.add_argument("csv", nargs="+")
    d.add_argument("-o", "--output", default="modelo_robot.yaml")
    d.add_argument("--max-delay", type=float, default=0.6)
    d.add_argument("--no-plot", action="store_true")
    d.set_defaults(func=cmd_dynamics)

    w = sub.add_parser("wheels", help="Calibra radio y separación de ruedas")
    w.add_argument("--odom-distance", type=float, required=True, help="Distancia según la odometría [m]")
    w.add_argument("--real-distance", type=float, required=True, help="Distancia medida con cinta [m]")
    w.add_argument("--odom-angle", type=float, help="Giro según la odometría [rad]")
    w.add_argument("--real-angle", type=float, help="Giro real [rad] (2π = una vuelta)")
    w.add_argument("-o", "--output", default="calibracion_ruedas.yaml")
    w.set_defaults(func=cmd_wheels)

    a = sub.add_parser("arena", help="Genera un mundo de Gazebo con un recinto rectangular")
    a.add_argument("--bounds", type=float, nargs=4, required=True, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    a.add_argument("--wall-height", type=float, default=0.3)
    a.add_argument("--wall-thickness", type=float, default=0.05)
    a.add_argument("--obstacle", type=float, nargs=4, action="extend", metavar=("X", "Y", "ANCHO", "LARGO"))
    a.add_argument("-o", "--output", default="arena.world")
    a.set_defaults(func=cmd_arena)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
