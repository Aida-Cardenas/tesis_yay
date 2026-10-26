"""Orden de las corridas y registro de una sesión de experimentos (sin dependencias de ROS).

Orden intercalado. Las condiciones que se comparan entre sí no se corren en
bloques (las 5 de A y después las 5 de B), porque todo lo que cambia con el
tiempo (batería, temperatura de los motores, red, el operador) se confundiría
con el efecto de la condición. El protocolo declara grupos en `interleave`; cada
grupo se corre por rondas, una corrida de cada miembro por ronda, en un orden
aleatorio dentro de la ronda. La semilla queda guardada en la sesión, así que
el mismo protocolo con la misma semilla da siempre el mismo orden.

Registro de la sesión. `sesion.json` guarda cada corrida (completa o no, válida
o descartada) con su registro, la diferencia de relojes y el voltaje anotado.
Con él se puede retomar una sesión interrumpida y el informe usa solo las
corridas completas y válidas.
"""
import json
import os
import random
from collections import deque
from datetime import datetime

SESSION_FILE = "sesion.json"


def repetitions(step, override=None):
    return int(override or step["repetitions"])


def build_schedule(steps, protocol, seed, override_reps=None):
    """Lista de segmentos en el orden en que se ejecutan.

    Cada segmento es {"kind": "calibrate" | "tune", "step": ...} o
    {"kind": "runs", "name": ..., "items": [(step, repetición), ...]}. Un grupo
    intercalado se ejecuta en la posición de su último miembro en el YAML, así
    todos sus miembros ya tienen disponibles la calibración y las ganancias que
    piden los pasos anteriores.
    """
    groups = [list(g) for g in (protocol.get("interleave") or [])]
    experiments = {s["name"]: s for s in steps if s["type"] == "experiment"}
    member_of = {}
    for gi, group in enumerate(groups):
        for name in group:
            if name not in experiments:
                raise ValueError(f"interleave nombra a {name}, que no es un experimento del protocolo")
            if name in member_of:
                raise ValueError(f"{name} aparece en dos grupos de interleave")
            member_of[name] = gi
    last_index = {gi: max(i for i, s in enumerate(steps) if s.get("name") in group)
                  for gi, group in enumerate(groups)}
    rng = random.Random(seed)
    segments = []
    for i, step in enumerate(steps):
        if step["type"] in ("calibrate", "tune"):
            segments.append({"kind": step["type"], "step": step})
            continue
        name = step["name"]
        if name in member_of:
            gi = member_of[name]
            if i != last_index[gi]:
                continue
            members = [experiments[n] for n in groups[gi]]
            items = []
            for k in range(1, max(repetitions(m, override_reps) for m in members) + 1):
                round_items = [(m, k) for m in members if repetitions(m, override_reps) >= k]
                rng.shuffle(round_items)
                items += round_items
            segments.append({"kind": "runs", "name": " + ".join(groups[gi]), "items": items})
        else:
            items = [(step, k) for k in range(1, repetitions(step, override_reps) + 1)]
            segments.append({"kind": "runs", "name": name, "items": items})
    return segments


def parse_answer(text):
    """Respuesta del operador antes de una corrida.

    Enter → continuar; un número → voltaje de la batería y continuar; s → saltar;
    x → la corrida anterior salió mal, repetirla; q → terminar.
    """
    text = (text or "").strip().lower().replace(",", ".")
    if not text:
        return "c", None
    try:
        volts = float(text)
    except ValueError:
        return text[:1], None
    if not 0.0 < volts < 60.0:
        return "?", None
    return "c", volts


class RunQueue:
    """Cola de un segmento: una corrida saltada vuelve una vez al final del segmento."""

    def __init__(self, items, done=()):
        done = set(done)
        self.queue = deque((s, k) for s, k in items if (s["name"], k) not in done)
        self.skipped = set()

    def __bool__(self):
        return bool(self.queue)

    def pop(self):
        return self.queue.popleft()

    def push_front(self, item):
        self.queue.appendleft(item)

    def skip(self, item):
        """Devuelve True si la corrida se reprograma al final, False si se abandona."""
        key = (item[0]["name"], item[1])
        if key in self.skipped:
            return False
        self.skipped.add(key)
        self.queue.append(item)
        return True


class Session:
    def __init__(self, folder, data=None):
        self.folder = folder
        self.data = data or {"creada": datetime.now().isoformat(timespec="seconds"), "corridas": []}

    @property
    def path(self):
        return os.path.join(self.folder, SESSION_FILE)

    @classmethod
    def load(cls, folder):
        with open(os.path.join(folder, SESSION_FILE)) as f:
            return cls(folder, json.load(f))

    def save(self):
        os.makedirs(self.folder, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value
        self.save()

    def record(self, **run):
        run.setdefault("valida", True)
        self.data["corridas"].append(run)
        self.save()
        return run

    def invalidate_last(self, reason):
        for run in reversed(self.data["corridas"]):
            if run.get("completa") and run.get("valida"):
                run["valida"] = False
                run["motivo"] = reason
                self.save()
                return run
        return None

    def good_runs(self):
        return [r for r in self.data["corridas"] if r.get("completa") and r.get("valida")]

    def done(self):
        return {(r["experimento"], r["repeticion"]) for r in self.good_runs()}

    def logs(self):
        logs = {}
        for r in self.good_runs():
            logs.setdefault(r["experimento"], []).append(r["registro"])
        return logs

    def discarded(self):
        return [r for r in self.data["corridas"] if not (r.get("completa") and r.get("valida"))]
