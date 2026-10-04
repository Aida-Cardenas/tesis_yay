#!/usr/bin/env python3
"""Ejecutor automático de experimentos del gemelo digital.

    ros2 run bumperbot_digital_twin twin_experiment protocolo_real
    ros2 run bumperbot_digital_twin twin_experiment protocolo_simulacion --auto
    ros2 run bumperbot_digital_twin twin_experiment mi_protocolo.yaml --only E1,E3

Lee un protocolo YAML (ver config/protocolo_*.yaml), y para cada experimento y
repetición: configura el puente (líder, corrección, compensación, modelo, red),
realinea, abre un registro con etiqueta <experimento>_r<n>, reproduce el recorrido
en el robot líder y espera a que el seguidor se detenga. Puede calibrar el modelo
del gemelo a mitad del protocolo. Al final genera el informe estadístico.

Necesita el puente corriendo (digital_twin.launch.py).
"""
import argparse
import os
import sys
import time
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory

from bumperbot_msgs.srv import TwinConfigure
from bumperbot_digital_twin.calibrate import fit_channel, read_identification_data
from bumperbot_digital_twin.dynamics import TwinModel
from bumperbot_digital_twin.report import build_report
from bumperbot_digital_twin.tuning import DEFAULT_GAINS
from bumperbot_digital_twin.ros_util import DualDomain, TrajectoryPlayer

DEFAULTS = {
    "repetitions": 3,
    "leader": "real",
    "pattern": "square",
    "linear_speed": 0.15,
    "angular_speed": 0.6,
    "distance": 1.0,
    "radius": 0.5,
    "laps": 1,
    "pause": 1.0,
    "feedback": True,
    "compensation": False,
    "twin_model": False,
    "net_delay_ms": 0.0,
    "net_jitter_ms": 0.0,
    "net_loss": 0.0,
    "settle": 3.0,
    "duration": 60.0,
    "instructions": "",
    "gains": None,
}


def resolve_protocol(name):
    if os.path.isfile(name):
        return name
    share = get_package_share_directory("bumperbot_digital_twin")
    candidate = os.path.join(share, "config", name if name.endswith(".yaml") else name + ".yaml")
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(f"No encuentro el protocolo {name}")


def load_protocol(path):
    with open(path) as f:
        protocol = yaml.safe_load(f)
    defaults = dict(DEFAULTS, **(protocol.get("defaults") or {}))
    steps = []
    for item in protocol.get("experiments", []):
        if "calibrate_from" in item:
            steps.append({"type": "calibrate", **item})
        elif "tune" in item:
            steps.append({"type": "tune", "defaults": defaults, **item})
        else:
            steps.append({"type": "experiment", **defaults, **item})
    return protocol, steps


class Runner:
    def __init__(self, args, protocol, steps):
        self.args = args
        self.protocol = protocol
        self.steps = steps
        self.dual = DualDomain("twin_experiment", args.real_domain, args.twin_domain)
        self.client = self.dual.nodes["twin"].create_client(TwinConfigure, "/digital_twin/configure")
        self.player = TrajectoryPlayer(self.dual)
        self.logs = {}
        self.model_path = None
        self.tuned_gains = None
        self.session_dir = os.path.expanduser(args.out or os.path.join(
            "~/twin_resultados", f"{protocol.get('name', 'protocolo')}_{datetime.now():%Y%m%d_%H%M%S}"))

    def configure(self, **kwargs):
        req = TwinConfigure.Request()
        req.leader = kwargs.get("leader", "")
        for key in ("feedback", "compensation", "twin_model"):
            setattr(req, key, -1 if kwargs.get(key) is None else int(bool(kwargs[key])))
        for key in ("net_delay_ms", "net_jitter_ms", "net_loss"):
            value = kwargs.get(key)
            setattr(req, key, -1.0 if value is None else float(value))
        req.align = bool(kwargs.get("align", False))
        req.new_log = bool(kwargs.get("new_log", False))
        req.log_tag = kwargs.get("log_tag", "")
        req.twin_model_file = kwargs.get("twin_model_file", "")
        gains = kwargs.get("gains") or {}
        req.kx, req.ky, req.ktheta = (float(gains.get(k, 0.0)) for k in ("kx", "ky", "ktheta"))
        req.pause_sync = bool(kwargs.get("pause_sync", False))
        req.resume_sync = bool(kwargs.get("resume_sync", False))
        res = self.dual.call("twin", self.client, req, timeout=15.0)
        if not res.success:
            raise RuntimeError(res.message)
        return res

    def ask(self, text):
        if self.args.auto:
            return "c"
        answer = input(f"\n{text}\n[Enter] continuar · [s] saltar · [q] terminar: ").strip().lower()
        return {"": "c"}.get(answer, answer[:1])

    def run_experiment(self, step, rep):
        tag = f"{step['name']}_r{rep}"
        prompt = f"── {tag}: {step.get('description', '')}"
        if step.get("instructions"):
            prompt += f"\n   {step['instructions']}"
        if not self.args.auto:
            prompt += "\n   Coloca el robot en la marca de salida."
        choice = self.ask(prompt)
        if choice == "q":
            return False
        if choice == "s":
            return True
        res = self.configure(
            leader=step["leader"], feedback=step["feedback"], compensation=step["compensation"],
            twin_model=step["twin_model"], net_delay_ms=step["net_delay_ms"],
            net_jitter_ms=step["net_jitter_ms"], net_loss=step["net_loss"],
            gains=self.gains_for(step), align=True, new_log=True, log_tag=tag)
        self.logs.setdefault(step["name"], []).append(res.log_path)
        print(f"   {res.message}")
        time.sleep(1.5)
        if step["pattern"] == "free":
            print(f"   Movimiento libre durante {step['duration']:.0f} s (joystick o teclado).")
            time.sleep(step["duration"])
        else:
            total = self.player.play(step["leader"], step["pattern"], step["linear_speed"], step["angular_speed"],
                                     step["distance"], step["radius"], step["laps"], step["pause"])
            print(f"   Recorrido {step['pattern']} terminado ({total:.1f} s)")
        time.sleep(step["settle"])
        return True

    def gains_for(self, step):
        choice = step.get("gains")
        if choice is None:
            return None
        if choice == "default":
            return DEFAULT_GAINS
        if choice == "tuned":
            if self.tuned_gains is None:
                print("   ADVERTENCIA: no hay ganancias sintonizadas todavía; uso las de por defecto")
                return DEFAULT_GAINS
            return self.tuned_gains
        if isinstance(choice, dict):
            return choice
        raise ValueError(f"gains debe ser default, tuned o un diccionario con kx, ky, ktheta: {choice}")

    def tune(self, step):
        from bumperbot_digital_twin.dynamics import NOMINAL_MODEL
        from bumperbot_digital_twin.tune import run_tuning
        cfg = step["tune"] or {}
        d = step["defaults"]
        real_model = TwinModel.load(self.model_path) if self.model_path else NOMINAL_MODEL
        if not self.model_path:
            print("   ADVERTENCIA: no hay modelo calibrado; sintonizo con el modelo ideal")

        def as_list(v, default):
            v = cfg.get(v, default)
            return v if isinstance(v, list) else [v]

        output = os.path.join(self.session_dir, step.get("output", "ganancias.yaml"))
        result, _ = run_tuning(
            real_model, NOMINAL_MODEL, as_list("follower", ["twin", "real"]),
            [float(x) for x in as_list("delay_ms", [0.0])], as_list("patterns", [d["pattern"]]),
            bool(cfg.get("compensation", False)), d["linear_speed"], d["angular_speed"], d["distance"],
            d["radius"], d["pause"], output=output, plots=not self.args.no_plots, quiet=True)
        self.tuned_gains = result.gains
        g = result.gains
        print(f"   kx={g['kx']:.2f} ky={g['ky']:.2f} kθ={g['ktheta']:.2f} "
              f"(costo simulado {result.default_cost:.4f} → {result.cost:.4f}); guardadas en {output}")
        return True

    def calibrate(self, step):
        self.configure(new_log=True, log_tag="calibracion")
        sources = []
        for name in step["calibrate_from"]:
            sources += self.logs.get(name, [])
        output = os.path.join(self.session_dir, step.get("output", "modelo_gemelo.yaml"))
        chunks = read_identification_data(sources)
        model = TwinModel(source=", ".join(os.path.basename(s) for s in sources))
        for channel in ("linear", "angular"):
            try:
                m, t, _, _ = fit_channel(chunks[channel], 0.6)
                m.samples = int(len(t))
                setattr(model, channel, m)
                print(f"   {channel}: K={m.gain:.3f} τ={m.tau:.3f}s L={m.delay:.3f}s R²={m.r2:.3f}")
            except ValueError as exc:
                print(f"   {channel}: {exc}")
        model.save(output)
        self.model_path = output
        res = self.configure(twin_model_file=output)
        print(f"   Modelo calibrado guardado en {output} ({res.message})")
        return True

    def run(self):
        os.makedirs(self.session_dir, exist_ok=True)
        only = set(self.args.only.split(",")) if self.args.only else None
        print(f"Protocolo: {self.protocol.get('name', '')} · resultados en {self.session_dir}")
        self.configure()
        for step in self.steps:
            if step["type"] == "tune":
                if only is None or step.get("name") in only:
                    print(f"\n── Sintonización de ganancias con el gemelo ({step.get('name', 'TUNE')})")
                    self.tune(step)
                continue
            if step["type"] == "calibrate":
                if only is None or step.get("name") in only or any(s in only for s in step["calibrate_from"]):
                    print(f"\n── Calibración del gemelo con {', '.join(step['calibrate_from'])}")
                    self.calibrate(step)
                continue
            if only is not None and step["name"] not in only:
                continue
            reps = self.args.repetitions or step["repetitions"]
            for rep in range(1, reps + 1):
                if not self.run_experiment(step, rep):
                    return self.finish()
        return self.finish()

    def finish(self):
        self.configure(new_log=True, log_tag="fin", feedback=True, compensation=False, twin_model=False,
                       net_delay_ms=0.0, net_jitter_ms=0.0, net_loss=0.0, resume_sync=True)
        paths = [p for runs in self.logs.values() for p in runs if os.path.exists(p)]
        if not paths:
            print("No hay corridas para analizar")
            return 1
        report_md, _, _ = build_report(paths, self.session_dir, self.protocol, plots=not self.args.no_plots)
        print("\n" + report_md)
        print(f"Informe: {os.path.join(self.session_dir, 'informe.md')}")
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("protocol", help="Archivo YAML o nombre de un protocolo incluido")
    parser.add_argument("--real-domain", type=int, default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int, default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    parser.add_argument("--auto", action="store_true", help="No pedir confirmación entre corridas")
    parser.add_argument("--only", help="Solo estos experimentos, separados por comas")
    parser.add_argument("--repetitions", type=int, help="Sobrescribe las repeticiones")
    parser.add_argument("--out", help="Carpeta de resultados")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    protocol, steps = load_protocol(resolve_protocol(args.protocol))
    runner = Runner(args, protocol, steps)
    try:
        return runner.run()
    except KeyboardInterrupt:
        print("\nInterrumpido; genero el informe con lo que hay")
        return runner.finish()
    finally:
        runner.player.stop()
        runner.dual.shutdown()


if __name__ == "__main__":
    sys.exit(main())
