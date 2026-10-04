#!/usr/bin/env python3
"""Sintoniza las ganancias del puente (kx, ky, kθ) simulando con el modelo del gemelo.

    ros2 run bumperbot_digital_twin twin_tune --real-model modelo_gemelo.yaml
    ros2 run bumperbot_digital_twin twin_tune --real-model modelo_gemelo.yaml --follower real --delay-ms 0,150,300
    ros2 run bumperbot_digital_twin twin_tune --real-model modelo_gemelo.yaml --apply

Usa el modelo dinámico identificado del robot real (twin_calibrate dynamics) y el del
gemelo para simular el lazo completo en cada escenario (recorrido × quién sigue ×
retardo de red), busca las ganancias que minimizan el error y guarda:
  ganancias.yaml          para el puente (gains_file:=... o --apply)
  ganancias_informe.md    antes/después por escenario
  ganancias.png           trayectorias y error antes/después
"""
import argparse
import os
import sys

from bumperbot_digital_twin.dynamics import NOMINAL_MODEL, TwinModel
from bumperbot_digital_twin.tuning import (
    DEFAULT_GAINS, CostWeights, Limits, save_gains, scenarios_from, simulate, tune,
)


def parse_list(text, cast=str):
    return [cast(x) for x in str(text).split(",") if x.strip()]


def report_markdown(result, scenarios, real_model, twin_model):
    g, d = result.gains, DEFAULT_GAINS
    lines = [
        "# Sintonización de la ley de seguimiento", "",
        f"Modelo del robot real: lineal K={real_model.linear.gain:.3f} τ={real_model.linear.tau:.3f} s "
        f"L={real_model.linear.delay:.3f} s · angular K={real_model.angular.gain:.3f} "
        f"τ={real_model.angular.tau:.3f} s L={real_model.angular.delay:.3f} s",
        f"Modelo del gemelo: lineal K={twin_model.linear.gain:.3f} τ={twin_model.linear.tau:.3f} s · "
        f"angular K={twin_model.angular.gain:.3f} τ={twin_model.angular.tau:.3f} s", "",
        "| | kx | ky | kθ | Costo |", "|---|---|---|---|---|",
        f"| Antes | {d['kx']:.2f} | {d['ky']:.2f} | {d['ktheta']:.2f} | {result.default_cost:.4f} |",
        f"| Después | {g['kx']:.2f} | {g['ky']:.2f} | {g['ktheta']:.2f} | {result.cost:.4f} |", "",
        f"{result.evaluations} combinaciones simuladas en {len(scenarios)} escenarios.", "",
        "| Escenario | RMSE antes [cm] | RMSE después [cm] | Máx. antes [cm] | Máx. después [cm] |",
        "|---|---|---|---|---|",
    ]
    for r in result.per_scenario:
        lines.append(f"| {r['escenario']} | {r['rmse_antes'] * 100:.1f} | {r['rmse_despues'] * 100:.1f} | "
                     f"{r['max_antes'] * 100:.1f} | {r['max_despues'] * 100:.1f} |")
    return "\n".join(lines) + "\n"


def plot(result, scenarios, real_model, twin_model, limits, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gain = [r["rmse_antes"] - r["rmse_despues"] for r in result.per_scenario]
    order = sorted(range(len(scenarios)), key=lambda i: -abs(gain[i]))[:3]
    chosen = [scenarios[i] for i in sorted(order)]
    n = len(chosen)
    fig, axes = plt.subplots(2, n, figsize=(4.5 * n, 7.5), squeeze=False)
    for i, s in enumerate(chosen):
        before = simulate(DEFAULT_GAINS, s, real_model, twin_model, limits, keep=True)
        after = simulate(result.gains, s, real_model, twin_model, limits, keep=True)
        ax = axes[0][i]
        ax.plot(before.leader[:, 0], before.leader[:, 1], color="#444444", lw=1.5, label="Líder")
        ax.plot(before.follower[:, 0], before.follower[:, 1], color="#d62728", lw=1.2, label="Seguidor (antes)")
        ax.plot(after.follower[:, 0], after.follower[:, 1], color="#1f77b4", lw=1.2, label="Seguidor (después)")
        ax.set_aspect("equal")
        ax.set_title(s.name, fontsize=10)
        ax.grid(alpha=0.3)
        ax2 = axes[1][i]
        ax2.plot(before.t, before.error * 100, color="#d62728", lw=1.2, label="Antes")
        ax2.plot(after.t, after.error * 100, color="#1f77b4", lw=1.2, label="Después")
        ax2.set_xlabel("Tiempo [s]")
        ax2.set_ylabel("Error de posición [cm]")
        ax2.grid(alpha=0.3)
    axes[0][0].legend(fontsize=8)
    axes[1][0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def apply_to_bridge(gains, real_domain, twin_domain):
    from bumperbot_msgs.srv import TwinConfigure
    from bumperbot_digital_twin.ros_util import DualDomain

    dual = DualDomain("twin_tune", real_domain, twin_domain)
    try:
        client = dual.nodes["twin"].create_client(TwinConfigure, "/digital_twin/configure")
        req = TwinConfigure.Request()
        req.feedback = req.compensation = req.twin_model = -1
        req.net_delay_ms = req.net_jitter_ms = req.net_loss = -1.0
        req.kx, req.ky, req.ktheta = gains["kx"], gains["ky"], gains["ktheta"]
        return dual.call("twin", client, req).message
    finally:
        dual.shutdown()


def run_tuning(real_model, twin_model, followers, delays_ms, patterns, compensation=False,
               linear_speed=0.15, angular_speed=0.6, distance=1.0, radius=0.5, pause=0.5,
               limits=Limits(), weights=CostWeights(), output="ganancias.yaml", plots=True, quiet=False):
    scenarios = scenarios_from(patterns, followers, delays_ms, compensation, linear_speed,
                               angular_speed, distance, radius, pause)

    def progress(i, g, c):
        if not quiet and i % 25 == 0:
            print(f"   {i} combinaciones… mejor hasta ahora {c:.4f}", flush=True)

    result = tune(scenarios, real_model, twin_model, limits, weights, progress=progress)
    md = report_markdown(result, scenarios, real_model, twin_model)
    out_dir = os.path.dirname(os.path.abspath(output))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(output)[0]
    save_gains(output, result.gains,
               info=f"Generado por twin_tune. Costo {result.default_cost:.4f} -> {result.cost:.4f}")
    with open(base + "_informe.md", "w") as f:
        f.write(md)
    if plots:
        plot(result, scenarios, real_model, twin_model, limits, base + ".png")
    return result, md


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--real-model", help="Modelo del robot real (twin_calibrate dynamics). Por defecto, ideal")
    parser.add_argument("--twin-model", help="Modelo del gemelo. Por defecto, ideal")
    parser.add_argument("--follower", default="twin,real", help="Quién sigue: twin, real o ambos")
    parser.add_argument("--delay-ms", default="0", help="Retardos de red de un sentido, separados por comas")
    parser.add_argument("--patterns", default="square,circle", help="Recorridos, separados por comas")
    parser.add_argument("--compensation", action="store_true", help="Sintonizar con la compensación activa")
    parser.add_argument("--linear-speed", type=float, default=0.15)
    parser.add_argument("--angular-speed", type=float, default=0.6)
    parser.add_argument("--distance", type=float, default=1.0)
    parser.add_argument("--radius", type=float, default=0.5)
    parser.add_argument("--effort-weight", type=float, default=CostWeights.effort,
                        help="Penalización por comandos bruscos (más alto = más suave)")
    parser.add_argument("-o", "--output", default="ganancias.yaml")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--apply", action="store_true", help="Enviar las ganancias al puente que está corriendo")
    parser.add_argument("--real-domain", type=int, default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int, default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    args = parser.parse_args(argv)

    real_model = TwinModel.load(os.path.expanduser(args.real_model)) if args.real_model else NOMINAL_MODEL
    twin_model = TwinModel.load(os.path.expanduser(args.twin_model)) if args.twin_model else NOMINAL_MODEL
    print("Sintonizando con el gemelo…")
    result, md = run_tuning(
        real_model, twin_model, parse_list(args.follower), parse_list(args.delay_ms, float),
        parse_list(args.patterns), args.compensation, args.linear_speed, args.angular_speed,
        args.distance, args.radius, weights=CostWeights(effort=args.effort_weight),
        output=args.output, plots=not args.no_plot)
    print()
    print(md)
    print(f"Ganancias guardadas en {args.output}. Úsalas con gains_file:={os.path.abspath(args.output)}")
    if args.apply:
        print(f"Puente: {apply_to_bridge(result.gains, args.real_domain, args.twin_domain)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
