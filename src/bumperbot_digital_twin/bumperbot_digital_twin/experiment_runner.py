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

Las condiciones que se comparan se intercalan (ver session.py) y cada corrida
queda anotada en <carpeta de la sesión>/sesion.json. Una sesión interrumpida se
retoma con --resume <carpeta>; solo cuentan las corridas completas.

Necesita el puente corriendo (digital_twin.launch.py).
"""
import argparse
import math
import os
import random
import sys
import time
from datetime import datetime

import yaml

from bumperbot_msgs.srv import TwinConfigure
from bumperbot_digital_twin.calibrate import fit_channel, plot_fit, read_identification_data
from bumperbot_digital_twin.dynamics import TwinModel
from bumperbot_digital_twin.report import build_report, resolve_protocol_path
from bumperbot_digital_twin.session import RunQueue, Session, build_schedule, parse_answer
from bumperbot_digital_twin.tuning import DEFAULT_GAINS, load_gains_file
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
        self.model_path = None
        self.calibration_failed = False
        self.tuned_gains = None
        self.tuning_failed = False
        if getattr(args, "resume", None):
            self.session_dir = os.path.expanduser(args.resume)
            self.session = Session.load(self.session_dir)
            started_with = self.session.get("protocolo")
            if started_with and started_with != protocol.get("name", ""):
                raise ValueError(f"La sesión {self.session_dir} es del protocolo {started_with}, no de "
                                 f"{protocol.get('name', '')}: el orden de las corridas no coincidiría")
            if args.repetitions is None:
                args.repetitions = self.session.get("repeticiones")
        else:
            self.session_dir = os.path.expanduser(args.out or os.path.join(
                "~/twin_resultados", f"{protocol.get('name', 'protocolo')}_{datetime.now():%Y%m%d_%H%M%S}"))
            self.session = Session(self.session_dir)
        self.logs = self.session.logs()
        self.last_item = None
        self.dual = DualDomain("twin_experiment", args.real_domain, args.twin_domain)
        self.client = self.dual.nodes["twin"].create_client(TwinConfigure, "/digital_twin/configure")
        self.player = TrajectoryPlayer(self.dual)

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
        req.close_log = bool(kwargs.get("close_log", False))
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
            return "c", None
        while True:
            answer = input(f"\n{text}\n[Enter] continuar · voltaje de la batería (p. ej. 11.8) y Enter · "
                           "[s] saltar · [x] la corrida anterior salió mal · [q] terminar: ")
            choice, volts = parse_answer(answer)
            if choice in ("c", "s", "x", "q"):
                return choice, volts
            print("   No entendí la respuesta.")

    def skip_reason(self, step):
        if step.get("twin_model") and self.calibration_failed:
            return "necesita el gemelo calibrado y la calibración falló"
        if step.get("twin_model") and self.model_path is None:
            return "necesita el gemelo calibrado y en esta sesión no se calibró (corre también el paso CAL)"
        if step.get("gains") == "tuned" and (self.tuning_failed or self.tuned_gains is None):
            return "necesita las ganancias sintonizadas y no hay"
        return None

    def run_experiment(self, step, rep):
        """Una corrida. Devuelve "ok", "skip", "redo" (repetir la anterior) o "quit"."""
        tag = f"{step['name']}_r{rep}"
        prompt = f"── {tag}: {step.get('description', '')}"
        if step.get("instructions"):
            prompt += f"\n   {step['instructions']}"
        if not self.args.auto:
            prompt += "\n   Coloca el robot en la marca de salida."
        choice, volts = self.ask(prompt)
        if choice == "q":
            return "quit"
        if choice == "s":
            return "skip"
        if choice == "x":
            return "redo"
        started = datetime.now().isoformat(timespec="seconds")
        res = self.configure(
            leader=step["leader"], feedback=step["feedback"], compensation=step["compensation"],
            twin_model=step["twin_model"], net_delay_ms=step["net_delay_ms"],
            net_jitter_ms=step["net_jitter_ms"], net_loss=step["net_loss"],
            gains=self.gains_for(step), align=True, new_log=True, log_tag=tag)
        run = {"experimento": step["name"], "repeticion": rep, "registro": res.log_path, "inicio": started,
               "voltaje": volts, "desfase_relojes_ms": _finite(getattr(res, "clock_offset_ms", math.nan)),
               "rtt_ms": _finite(getattr(res, "rtt_ms", math.nan)), "completa": False}
        print(f"   {res.message}")
        try:
            time.sleep(1.5)
            if step["pattern"] == "free":
                print(f"   Movimiento libre durante {step['duration']:.0f} s (joystick o teclado).")
                time.sleep(step["duration"])
            else:
                total = self.player.play(step["leader"], step["pattern"], step["linear_speed"],
                                         step["angular_speed"], step["distance"], step["radius"], step["laps"],
                                         step["pause"])
                print(f"   Recorrido {step['pattern']} terminado ({total:.1f} s)")
            time.sleep(step["settle"])
        except KeyboardInterrupt:
            self.player.stop()
            try:
                self.configure(close_log=True)
            finally:
                run["motivo"] = "interrumpida con Ctrl-C"
                self.session.record(**run)
                print(f"\n   {tag} interrumpida: no cuenta como corrida.")
            raise
        self.configure(close_log=True)
        run["completa"] = True
        run["fin"] = datetime.now().isoformat(timespec="seconds")
        self.session.record(**run)
        if res.log_path:
            self.logs.setdefault(step["name"], []).append(res.log_path)
        return "ok"

    def gains_for(self, step):
        choice = step.get("gains")
        if choice is None:
            return None
        if choice == "default":
            return DEFAULT_GAINS
        if choice == "tuned":
            return self.tuned_gains
        if isinstance(choice, dict):
            return choice
        raise ValueError(f"gains debe ser default, tuned o un diccionario con kx, ky, ktheta: {choice}")

    def tune(self, step):
        from bumperbot_digital_twin.dynamics import NOMINAL_MODEL
        from bumperbot_digital_twin.tune import run_tuning
        cfg = step["tune"] or {}
        d = step["defaults"]
        if not self.model_path:
            self.tuning_failed = True
            print("   ERROR: no hay un modelo calibrado en esta sesión (paso CAL); sin él la "
                  "sintonización no tiene sentido. Se salta, junto con los experimentos con gains: tuned.")
            return False
        real_model = TwinModel.load(self.model_path)

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
        self.session.set("ganancias", output)
        g = result.gains
        print(f"   kx={g['kx']:.2f} ky={g['ky']:.2f} kθ={g['ktheta']:.2f} "
              f"(costo simulado {result.default_cost:.4f} → {result.cost:.4f}); guardadas en {output}")
        if result.at_bounds:
            print(f"   ADVERTENCIA: {', '.join(result.at_bounds)} quedó en el límite de la búsqueda; el óptimo "
                  "real podría estar más allá (ver BOUNDS en tuning.py).")
        return True

    def calibrate(self, step):
        self.configure(close_log=True)
        sources = []
        for name in step["calibrate_from"]:
            sources += self.logs.get(name, [])
        output = os.path.join(self.session_dir, step.get("output", "modelo_gemelo.yaml"))
        chunks = read_identification_data(sources)
        model = TwinModel(source=", ".join(os.path.basename(s) for s in sources))
        fits = {}
        for channel in ("linear", "angular"):
            try:
                m, t, u, y = fit_channel(chunks[channel], 0.6)
                m.samples = int(len(t))
                setattr(model, channel, m)
                fits[channel] = (m, t, u, y)
                print(f"   {channel}: K={m.gain:.3f} τ={m.tau:.3f}s L={m.delay:.3f}s R² (respuesta simulada)={m.r2:.3f}")
            except ValueError as exc:
                print(f"   {channel}: {exc}")
        missing = [c for c in ("linear", "angular") if c not in fits]
        if missing:
            self.calibration_failed = True
            self.model_path = None
            print(f"   ERROR: no se pudo calibrar ({', '.join(missing)}): faltan corridas de "
                  f"{', '.join(step['calibrate_from'])} en esta sesión. No se carga ningún modelo; "
                  "se saltan los experimentos con gemelo calibrado y la sintonización.")
            return False
        if not self.args.no_plots:
            plot_fit(model, fits, os.path.splitext(output)[0] + "_ajuste.png")
        model.save(output)
        self.model_path = output
        self.session.set("modelo", output)
        res = self.configure(twin_model_file=output)
        print(f"   Modelo calibrado guardado en {output} ({res.message})")
        return True

    def check_clocks(self):
        """Mide la diferencia de relojes con el robot antes de empezar (R4-M1)."""
        offset = math.nan
        for _ in range(10):
            res = self.configure()
            offset = getattr(res, "clock_offset_ms", math.nan)
            if math.isfinite(offset):
                break
            time.sleep(1.0)
        if not math.isfinite(offset):
            print("ADVERTENCIA: no se pudo medir la diferencia de relojes con el robot (¿corre latency_echo "
                  "actualizado en la Raspberry Pi?). El error real puede quedar sesgado o vacío.")
            return True
        print(f"Diferencia de relojes robot − PC: {offset:+.1f} ms")
        if abs(offset) > self.args.max_clock_offset:
            print(f"ERROR: los relojes difieren en más de {self.args.max_clock_offset:.0f} ms; el error real "
                  "saldría sesgado. Sincronízalos con chrony (README) o usa --ignore-clock.")
            return self.args.ignore_clock
        if abs(offset) > 10.0:
            print("ADVERTENCIA: más de 10 ms de diferencia; revisa chrony antes de la toma de datos final.")
        return True

    def restore(self):
        """Reutiliza la calibración y las ganancias de la sesión o de --model / --gains."""
        model = self.args.model or self.session.get("modelo")
        if model and os.path.exists(model):
            res = self.configure(twin_model_file=model)
            self.model_path = model
            self.session.data["modelo"] = model
            print(f"Modelo del gemelo reutilizado: {model} ({res.message})")
        gains = self.args.gains or self.session.get("ganancias")
        if gains and os.path.exists(gains):
            self.tuned_gains = load_gains_file(gains)
            self.session.data["ganancias"] = gains
            print(f"Ganancias sintonizadas reutilizadas: {gains}")

    def run(self):
        os.makedirs(self.session_dir, exist_ok=True)
        seed = self.session.get("semilla")
        if seed is None:
            seed = self.args.seed if self.args.seed is not None else random.SystemRandom().randrange(10 ** 6)
        self.session.data.update({"protocolo": self.protocol.get("name", ""), "semilla": seed,
                                  "repeticiones": self.args.repetitions})
        self.session.save()
        only = set(self.args.only.split(",")) if self.args.only else None
        print(f"Protocolo: {self.protocol.get('name', '')} · semilla {seed} · resultados en {self.session_dir}")
        if self.session.good_runs():
            print(f"Se retoma la sesión: {len(self.session.good_runs())} corridas completas.")
        self.configure()
        if not self.check_clocks():
            return self.finish()
        self.restore()
        done = self.session.done()
        for segment in build_schedule(self.steps, self.protocol, seed, self.args.repetitions):
            step = segment.get("step")
            if segment["kind"] == "tune":
                if self.tuned_gains is None and (only is None or step.get("name") in only):
                    print(f"\n── Sintonización de ganancias con el gemelo ({step.get('name', 'TUNE')})")
                    self.tune(step)
                continue
            if segment["kind"] == "calibrate":
                wanted = only is None or step.get("name") in only or any(s in only for s in step["calibrate_from"])
                if self.model_path is None and wanted:
                    print(f"\n── Calibración del gemelo con {', '.join(step['calibrate_from'])}")
                    self.calibrate(step)
                continue
            items = [(st, k) for st, k in segment["items"] if only is None or st["name"] in only]
            if not items:
                continue
            queue = RunQueue(items, done)
            if queue:
                print(f"\n══ {segment['name']}")
            while queue:
                item = queue.pop()
                step, rep = item
                reason = self.skip_reason(step)
                if reason:
                    print(f"── {step['name']}_r{rep}: se salta ({reason})")
                    continue
                result = self.run_experiment(step, rep)
                if result == "quit":
                    return self.finish()
                if result == "skip":
                    if queue.skip(item):
                        print("   Se repetirá al final de este grupo.")
                    else:
                        print("   Saltada por segunda vez: queda fuera de la sesión.")
                    continue
                if result == "redo":
                    queue.push_front(item)
                    if self.last_item is None:
                        print("   No hay una corrida anterior de esta ejecución que repetir.")
                        continue
                    bad = self.session.invalidate_last("el operador la marcó como mal hecha")
                    prev = self.logs.get(bad["experimento"], [])
                    if bad["registro"] in prev:
                        prev.remove(bad["registro"])
                    queue.push_front(self.last_item)
                    self.last_item = None
                    print(f"   {bad['experimento']}_r{bad['repeticion']} descartada; se repite ahora.")
                    continue
                self.last_item = item
        return self.finish()

    def finish(self):
        try:
            self.configure(close_log=True, feedback=True, compensation=False, twin_model=False,
                           net_delay_ms=0.0, net_jitter_ms=0.0, net_loss=0.0, resume_sync=True,
                           gains=DEFAULT_GAINS)
        except Exception as exc:
            print(f"No se pudo dejar el puente en su configuración normal: {exc}")
        discarded = self.session.discarded()
        if discarded:
            print("\nCorridas que no cuentan:")
            for r in discarded:
                print(f"   {r['experimento']}_r{r['repeticion']}: {r.get('motivo', 'incompleta')}")
        paths = [p for runs in self.logs.values() for p in runs if os.path.exists(p)]
        if not paths:
            print("No hay corridas para analizar")
            return 1
        report_md, _, _ = build_report(paths, self.session_dir, self.protocol, plots=not self.args.no_plots)
        print("\n" + report_md)
        print(f"Informe: {os.path.join(self.session_dir, 'informe.md')}")
        print(f"Para retomar esta sesión: twin_experiment {self.protocol.get('name', '<protocolo>')} "
              f"--resume {self.session_dir}")
        return 0


def _finite(value):
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("protocol", help="Archivo YAML o nombre de un protocolo incluido")
    parser.add_argument("--real-domain", type=int, default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int, default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    parser.add_argument("--auto", action="store_true", help="No pedir confirmación entre corridas")
    parser.add_argument("--only", help="Solo estos experimentos, separados por comas")
    parser.add_argument("--repetitions", type=int, help="Sobrescribe las repeticiones")
    parser.add_argument("--out", help="Carpeta de resultados")
    parser.add_argument("--resume", metavar="CARPETA", help="Retomar una sesión interrumpida")
    parser.add_argument("--model", help="Usar este modelo del gemelo en vez de calibrar (salida de CAL)")
    parser.add_argument("--gains", help="Usar estas ganancias en vez de sintonizar (salida de TUNE)")
    parser.add_argument("--seed", type=int, help="Semilla del orden intercalado (por defecto, al azar)")
    parser.add_argument("--max-clock-offset", type=float, default=50.0,
                        help="Diferencia de relojes máxima en ms (por defecto 50)")
    parser.add_argument("--ignore-clock", action="store_true", help="Seguir aunque los relojes difieran")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    protocol, steps = load_protocol(resolve_protocol_path(args.protocol))
    try:
        runner = Runner(args, protocol, steps)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    try:
        return runner.run()
    except KeyboardInterrupt:
        print("\nInterrumpido; genero el informe con las corridas completas")
        return runner.finish()
    except Exception as exc:
        print(f"\nERROR: {exc}\nGenero el informe con las corridas completas hasta ahora")
        runner.finish()
        return 1
    finally:
        runner.player.stop()
        runner.dual.shutdown()


if __name__ == "__main__":
    sys.exit(main())
