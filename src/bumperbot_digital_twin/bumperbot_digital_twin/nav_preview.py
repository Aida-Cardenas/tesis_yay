#!/usr/bin/env python3
"""Navegación con vista previa: el gemelo prueba la meta antes que el robot real.

    ros2 run bumperbot_digital_twin twin_navigate 1.2 0.5 90
    ros2 run bumperbot_digital_twin twin_navigate 1.2 0.5 90 --execute ask
    ros2 run bumperbot_digital_twin twin_navigate --goals metas_arena

Para cada meta (x, y en metros y orientación en grados, en el marco del mapa):
  1. Pausa la sincronización del puente y manda la meta a Nav2 del gemelo.
  2. Registra la ruta, el tiempo, la distancia mínima a obstáculos (LiDAR del gemelo)
     y las maniobras de recuperación.
  3. Aprueba la meta si el gemelo llegó, sin acercarse demasiado a nada y a tiempo.
  4. Devuelve el gemelo al punto de partida.
  5. Si se aprobó, la manda a Nav2 del robot real con la sincronización activa (el
     gemelo lo sigue) y compara la ruta real con la que predijo el gemelo.
Guarda un JSON, un CSV con las rutas y una gráfica por meta, y un informe con todas.

Necesita el puente corriendo (digital_twin.launch.py) y Nav2 en ambos robots (o los
navegadores de mentira con real_mode:=fake twin_mode:=fake).
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

from bumperbot_digital_twin.navigation import ApprovalCriteria, TwinRun, approve, compare_paths, min_valid_range

STATUS_NAMES = {0: "desconocido", 1: "aceptada", 2: "en curso", 3: "cancelando", 4: "lograda",
                5: "cancelada", 6: "abortada"}


def wait_future(future, timeout):
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return False
        time.sleep(0.02)
    return True


class NavSide:
    """Lo que la vista previa necesita de un robot: Nav2, su pose en el mapa y su LiDAR."""

    def __init__(self, name, node, frame):
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.name = name
        self.node = node
        self.frame = frame
        self.NavigateToPose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.lock = threading.Lock()
        self.scan_min = math.inf
        self.ignore_below = 0.0
        self.recoveries = 0
        node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def _scan_cb(self, msg):
        m = min_valid_range(msg.ranges, msg.range_min, msg.range_max, self.ignore_below)
        with self.lock:
            self.scan_min = min(self.scan_min, m)

    def reset_scan_min(self):
        with self.lock:
            self.scan_min = math.inf

    def pose(self, timeout=2.0):
        from rclpy.time import Time
        from bumperbot_digital_twin.geometry import yaw_from_quaternion
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                t = self.tf_buffer.lookup_transform(self.frame, "base_footprint", Time())
                q = t.transform.rotation
                return (t.transform.translation.x, t.transform.translation.y,
                        yaw_from_quaternion(q.x, q.y, q.z, q.w))
            except Exception:
                time.sleep(0.1)
        return None

    def publish_initial_pose(self, x, y, yaw):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.0025
        msg.pose.covariance[35] = 0.0076
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.2)

    def navigate(self, x, y, yaw, timeout, log=print):
        """Manda la meta y espera. Devuelve TwinRun con la ruta recorrida."""
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.client.wait_for_server(timeout_sec=15.0):
            return TwinRun(False, "Nav2 no responde (navigate_to_pose)", 0.0)
        self.reset_scan_min()
        self.recoveries = 0

        def feedback_cb(fb):
            self.recoveries = max(self.recoveries, int(fb.feedback.number_of_recoveries))

        t0 = time.monotonic()
        send = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        if not wait_future(send, 10.0) or not send.result().accepted:
            return TwinRun(False, "Nav2 rechazó la meta", time.monotonic() - t0)
        handle = send.result()
        result = handle.get_result_async()
        path = []
        status = None
        while True:
            p = self.pose(timeout=0.5)
            if p is not None:
                path.append((time.monotonic() - t0, *p))
            if result.done():
                status = result.result().status
                break
            if time.monotonic() - t0 > timeout:
                log(f"   [{self.name}] se acabó el tiempo; cancelo la meta")
                cancel = handle.cancel_goal_async()
                wait_future(cancel, 5.0)
                wait_future(result, 5.0)
                status = 5
                break
            time.sleep(0.1)
        end = self.pose(timeout=1.0)
        if end is not None:
            path.append((time.monotonic() - t0, *end))
        with self.lock:
            clearance = self.scan_min
        final_error = math.hypot(end[0] - x, end[1] - y) if end else math.nan
        return TwinRun(status == 4, STATUS_NAMES.get(status, str(status)), time.monotonic() - t0,
                       path, clearance, self.recoveries, final_error)


class NavPreview:
    def __init__(self, dual, configure, frame="map", criteria=ApprovalCriteria(), ignore_below=0.0, log=print):
        self.dual = dual
        self.configure = configure
        self.criteria = criteria
        self.log = log
        self.sides = {s: NavSide(s, dual.nodes[s], frame) for s in ("real", "twin")}
        for side in self.sides.values():
            side.ignore_below = ignore_below

    def set_initial_pose(self, x, y, yaw):
        for side in self.sides.values():
            side.publish_initial_pose(x, y, yaw)

    def run(self, name, x, y, yaw, execute="auto", return_twin=True, out_dir=None, plots=True):
        log = self.log
        twin, real = self.sides["twin"], self.sides["real"]
        log(f"── Meta {name}: ({x:.2f}, {y:.2f}) m, {math.degrees(yaw):.0f}°")
        start_twin = twin.pose()
        start_real = real.pose()
        if start_twin is None or start_real is None:
            missing = " y ".join(s for s, p in (("gemelo", start_twin), ("real", start_real)) if p is None)
            raise RuntimeError(f"No encuentro la pose en el mapa del robot {missing} (¿Nav2/AMCL corriendo?)")

        self.configure(pause_sync=True, new_log=True, log_tag=f"{name}_vista")
        log("   1/3 El gemelo prueba la meta (sincronización en pausa)…")
        preview = twin.navigate(x, y, yaw, self.criteria.max_time * 1.5, log)
        ok, reasons = approve(preview, self.criteria)
        log(f"   Gemelo: {preview.status} en {preview.duration:.1f} s, distancia mínima a obstáculos "
            f"{preview.min_clearance * 100:.0f} cm → {'APROBADA' if ok else 'RECHAZADA: ' + '; '.join(reasons)}")

        returned = None
        if return_twin:
            log("   2/3 El gemelo vuelve al punto de partida…")
            back = twin.navigate(*start_twin, self.criteria.max_time * 1.5, log)
            returned = back.status
            if not back.succeeded:
                log(f"   ADVERTENCIA: el gemelo no pudo volver ({back.status})")

        execution = None
        do_execute = ok and execute != "never"
        if do_execute and execute == "ask":
            do_execute = input("   ¿Ejecutar en el robot real? [s/N]: ").strip().lower().startswith("s")
        if do_execute:
            self.configure(leader="real", align=True, resume_sync=True, new_log=True, log_tag=f"{name}_real")
            log("   3/3 El robot real ejecuta la meta (el gemelo lo sigue)…")
            execution = real.navigate(x, y, yaw, self.criteria.max_time * 1.5, log)
            log(f"   Real: {execution.status} en {execution.duration:.1f} s, quedó a "
                f"{execution.final_error * 100:.1f} cm de la meta")
        else:
            self.configure(align=True, resume_sync=True)
            log("   3/3 No se ejecuta en el robot real")

        result = {
            "meta": name, "x": x, "y": y, "yaw_grados": math.degrees(yaw), "aprobada": ok, "motivos": reasons,
            "inicio_gemelo": start_twin, "inicio_real": start_real, "vuelta_gemelo": returned,
            "gemelo": run_dict(preview), "real": run_dict(execution) if execution else None,
            "ejecutada": execution is not None,
        }
        if execution is not None and preview.path and execution.path:
            result["comparacion"] = compare_paths([p[1:3] for p in preview.path], [p[1:3] for p in execution.path])
            result["comparacion"]["diferencia_tiempo_s"] = execution.duration - preview.duration
            c = result["comparacion"]
            log(f"   Ruta real vs prevista: desviación media {c['desviacion_media_m'] * 100:.1f} cm, "
                f"máxima {c['desviacion_max_m'] * 100:.1f} cm, diferencia de tiempo {c['diferencia_tiempo_s']:+.1f} s")
        if out_dir:
            save_result(result, preview, execution, out_dir, plots)
        return result


def run_dict(run):
    return {"lograda": run.succeeded, "estado": run.status, "duracion_s": run.duration,
            "distancia_minima_m": run.min_clearance if math.isfinite(run.min_clearance) else None,
            "recuperaciones": run.recoveries, "error_final_m": run.final_error,
            "puntos": len(run.path)}


def save_result(result, preview, execution, out_dir, plots):
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, f"vista_previa_{result['meta']}")
    with open(base + ".json", "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False, default=float)
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t", "x", "y", "yaw"])
        for label, run in (("gemelo", preview), ("real", execution)):
            for p in (run.path if run else []):
                w.writerow([label, *(f"{v:.4f}" for v in p)])
    if plots:
        plot_result(result, preview, execution, base + ".png")


def plot_result(result, preview, execution, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    if preview.path:
        xs, ys = zip(*[(p[1], p[2]) for p in preview.path])
        ax.plot(xs, ys, color="#1f77b4", lw=2, label="Prevista por el gemelo")
    if execution and execution.path:
        xs, ys = zip(*[(p[1], p[2]) for p in execution.path])
        ax.plot(xs, ys, color="#d62728", lw=1.5, ls="--", label="Recorrida por el robot real")
    sx, sy, _ = result["inicio_real"]
    ax.plot([sx], [sy], "o", color="#444444", label="Salida")
    ax.plot([result["x"]], [result["y"]], "*", color="#2ca02c", ms=14, label="Meta")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = f"Meta {result['meta']}: {'aprobada' if result['aprobada'] else 'rechazada'}"
    if "comparacion" in result:
        title += f" · desviación media {result['comparacion']['desviacion_media_m'] * 100:.1f} cm"
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def report_markdown(results):
    lines = ["# Navegación con vista previa en el gemelo", "",
             "| Meta | x [m] | y [m] | Aprobada | Motivo | t gemelo [s] | t real [s] | Desv. media [cm] | "
             "Desv. máx. [cm] | Hausdorff [cm] | Error final real [cm] |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        g, re, c = r["gemelo"], r["real"] or {}, r.get("comparacion", {})

        def cm(v):
            return f"{v * 100:.1f}" if isinstance(v, (int, float)) and math.isfinite(v) else "—"

        lines.append(
            f"| {r['meta']} | {r['x']:.2f} | {r['y']:.2f} | {'sí' if r['aprobada'] else 'no'} | "
            f"{'; '.join(r['motivos']) or '—'} | {g['duracion_s']:.1f} | "
            f"{re.get('duracion_s', float('nan')):.1f} | {cm(c.get('desviacion_media_m'))} | "
            f"{cm(c.get('desviacion_max_m'))} | {cm(c.get('hausdorff_m'))} | {cm(re.get('error_final_m'))} |")
    approved = [r for r in results if r["aprobada"]]
    executed = [r for r in results if "comparacion" in r]
    lines += ["", f"{len(approved)} de {len(results)} metas aprobadas por el gemelo; {len(executed)} ejecutadas."]
    if executed:
        import numpy as np
        dm = np.mean([r["comparacion"]["desviacion_media_m"] for r in executed])
        lines.append(f"Desviación media entre la ruta prevista y la real: {dm * 100:.1f} cm.")
    return "\n".join(lines) + "\n"


def load_goals(spec):
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = spec
    if not os.path.isfile(path):
        share = get_package_share_directory("bumperbot_digital_twin")
        path = os.path.join(share, "config", spec if spec.endswith(".yaml") else spec + ".yaml")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    goals = []
    for i, g in enumerate(data.get("goals", []), 1):
        goals.append((str(g.get("name", f"M{i}")), float(g["x"]), float(g["y"]), math.radians(float(g.get("yaw", 0.0)))))
    return data, goals


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("x", nargs="?", type=float)
    parser.add_argument("y", nargs="?", type=float)
    parser.add_argument("yaw", nargs="?", type=float, default=0.0, help="Orientación final en grados")
    parser.add_argument("--goals", help="YAML con una lista de metas (ver config/metas_arena.yaml)")
    parser.add_argument("--name", default="M1")
    parser.add_argument("--execute", choices=("auto", "ask", "never"), default="auto",
                        help="auto: ejecutar si se aprueba · ask: preguntar · never: solo vista previa")
    parser.add_argument("--no-return", action="store_true", help="No devolver el gemelo al punto de partida")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--min-clearance", type=float, default=ApprovalCriteria.min_clearance,
                        help="Distancia mínima del LiDAR a obstáculos para aprobar [m]")
    parser.add_argument("--max-time", type=float, default=ApprovalCriteria.max_time)
    parser.add_argument("--ignore-below", type=float, default=0.0,
                        help="Ignorar lecturas del LiDAR más cortas que esto (partes del propio robot) [m]")
    parser.add_argument("--initial-pose", type=float, nargs=3, metavar=("X", "Y", "YAW_GRADOS"),
                        help="Publicar esta pose inicial a AMCL en ambos robots antes de empezar")
    parser.add_argument("--return-home", action="store_true",
                        help="Al terminar, llevar el robot real (y el gemelo) de vuelta a la salida")
    parser.add_argument("--out", help="Carpeta de resultados (por defecto ~/twin_resultados/navegacion_<fecha>)")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--real-domain", type=int, default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int, default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    parser.add_argument("--twin-sim-time", action="store_true", help="El gemelo corre en Gazebo")
    args = parser.parse_args(argv)

    if args.goals:
        _, goals = load_goals(args.goals)
    elif args.x is not None and args.y is not None:
        goals = [(args.name, args.x, args.y, math.radians(args.yaw))]
    else:
        parser.error("indica x y [yaw] o --goals")
    if not goals:
        parser.error("no hay metas")

    from bumperbot_msgs.srv import TwinConfigure
    from bumperbot_digital_twin.ros_util import DualDomain

    out_dir = os.path.expanduser(args.out or os.path.join("~/twin_resultados",
                                                          f"navegacion_{datetime.now():%Y%m%d_%H%M%S}"))
    dual = DualDomain("twin_navigate", args.real_domain, args.twin_domain, twin_sim_time=args.twin_sim_time)
    client = dual.nodes["twin"].create_client(TwinConfigure, "/digital_twin/configure")

    def configure(**kw):
        req = TwinConfigure.Request()
        req.leader = kw.get("leader", "")
        req.feedback = req.compensation = req.twin_model = -1
        req.net_delay_ms = req.net_jitter_ms = req.net_loss = -1.0
        req.align = bool(kw.get("align", False))
        req.new_log = bool(kw.get("new_log", False))
        req.log_tag = kw.get("log_tag", "")
        req.pause_sync = bool(kw.get("pause_sync", False))
        req.resume_sync = bool(kw.get("resume_sync", False))
        res = dual.call("twin", client, req, timeout=15.0)
        if not res.success:
            raise RuntimeError(res.message)
        return res

    criteria = ApprovalCriteria(min_clearance=args.min_clearance, max_time=args.max_time)
    preview = NavPreview(dual, configure, args.frame, criteria, args.ignore_below)
    results = []
    code = 0
    try:
        time.sleep(1.0)
        if args.initial_pose:
            x0, y0, yaw0 = args.initial_pose
            preview.set_initial_pose(x0, y0, math.radians(yaw0))
            time.sleep(2.0)
        home = {s: preview.sides[s].pose() for s in ("real", "twin")}
        for name, x, y, yaw in goals:
            results.append(preview.run(name, x, y, yaw, args.execute, not args.no_return, out_dir,
                                       not args.no_plots))
        if args.return_home and all(home.values()):
            print("── Vuelta a la salida")
            configure(pause_sync=True)
            for side in ("real", "twin"):
                preview.sides[side].navigate(*home[side], args.max_time * 1.5)
            configure(align=True, resume_sync=True)
    except KeyboardInterrupt:
        print("\nInterrumpido")
        code = 1
    except RuntimeError as exc:
        print(f"Error: {exc}")
        code = 1
    finally:
        try:
            configure(resume_sync=True)
        except Exception:
            pass
        if results:
            md = report_markdown(results)
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, "informe_navegacion.md"), "w") as f:
                f.write(md)
            with open(os.path.join(out_dir, "informe_navegacion.json"), "w") as f:
                json.dump(results, f, indent=2, ensure_ascii=False, default=float)
            print("\n" + md)
            print(f"Resultados en {out_dir}")
        dual.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
