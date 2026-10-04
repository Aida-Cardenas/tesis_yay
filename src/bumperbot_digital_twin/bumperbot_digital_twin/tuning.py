"""Sintonización automática de la ley de seguimiento usando el gemelo como simulador.

El modelo dinámico identificado del robot real (twin_calibrate dynamics) y el del
gemelo permiten simular, muy rápido y sin mover ningún robot, cómo se comportaría el
lazo de sincronización completo con unas ganancias (kx, ky, kθ): el líder recorre una
trayectoria, la red introduce su retardo, el puente calcula la corrección y el
seguidor responde con su propia dinámica. Se busca la combinación que minimiza el
error de seguimiento (más una penalización por comandos bruscos) en un conjunto de
escenarios, primero con una rejilla y luego con Nelder-Mead.
"""
import itertools
import math
from dataclasses import dataclass, field

import numpy as np

from bumperbot_digital_twin.dynamics import NOMINAL_MODEL, FirstOrderFilter, TwinModel
from bumperbot_digital_twin.geometry import (
    Pose2D, TrackingGains, integrate_unicycle, smith_predict, tracking_command, tracking_error,
)
from bumperbot_digital_twin.trajectories import build_segments

DEFAULT_GAINS = {"kx": 1.5, "ky": 6.0, "ktheta": 3.0}
BOUNDS = {"kx": (0.2, 6.0), "ky": (0.5, 30.0), "ktheta": (0.3, 10.0)}


@dataclass
class Scenario:
    pattern: str = "square"
    follower: str = "twin"
    delay: float = 0.0
    compensation: bool = False
    linear_speed: float = 0.15
    angular_speed: float = 0.6
    distance: float = 1.0
    radius: float = 0.5
    pause: float = 0.5

    @property
    def name(self):
        comp = "+comp" if self.compensation else ""
        return f"{self.pattern}/{self.follower}/{self.delay * 1000:.0f}ms{comp}"


@dataclass
class Limits:
    max_linear: float = 0.5
    max_angular: float = 2.5
    real_max_linear: float = 0.3
    real_max_angular: float = 1.5


@dataclass
class CostWeights:
    heading: float = 0.05
    effort: float = 0.01


@dataclass
class SimResult:
    rmse: float
    max_error: float
    final_error: float
    rmse_heading: float
    effort: float
    cost: float
    t: np.ndarray = field(repr=False, default=None)
    leader: np.ndarray = field(repr=False, default=None)
    follower: np.ndarray = field(repr=False, default=None)
    error: np.ndarray = field(repr=False, default=None)


class _Robot:
    def __init__(self, model):
        self.pose = Pose2D()
        self.fv = FirstOrderFilter(model.linear)
        self.fw = FirstOrderFilter(model.angular)
        self.v = self.w = 0.0

    def step(self, t, cmd, h):
        self.v = self.fv.update(t, cmd[0])
        self.w = self.fw.update(t, cmd[1])
        self.pose = integrate_unicycle(self.pose, self.v, self.w, h)


def simulate(gains, scenario, real_model=NOMINAL_MODEL, twin_model=NOMINAL_MODEL, limits=Limits(),
             weights=CostWeights(), rate=20.0, h=0.01, settle=3.0, keep=False):
    """Simula el lazo líder → puente → seguidor con las ganancias dadas."""
    follower_real = scenario.follower == "real"
    leader_model, follower_model = (twin_model, real_model) if follower_real else (real_model, twin_model)
    leader, follower = _Robot(leader_model), _Robot(follower_model)
    segments = build_segments(scenario.pattern, scenario.linear_speed, scenario.angular_speed,
                              scenario.distance, scenario.radius, 1, scenario.pause)
    edges = np.cumsum([0.0] + [s[0] for s in segments])
    total = edges[-1] + settle
    d = scenario.delay
    lag = int(round(d / h))
    tg = TrackingGains(gains["kx"], gains["ky"], gains["ktheta"],
                       min(limits.max_linear, limits.real_max_linear) if follower_real else limits.max_linear,
                       min(limits.max_angular, limits.real_max_angular) if follower_real else limits.max_angular)
    period = 1.0 / rate
    steps_per_tick = max(1, int(round(period / h)))

    hist_leader, hist_follower = [], []
    in_flight = []
    sent = []
    cmd_follower = (0.0, 0.0)
    out_t, out_l, out_f, out_e, out_he, out_w = [], [], [], [], [], []
    n_steps = int(total / h)
    seg_idx = 0
    for k in range(n_steps):
        t = k * h
        while seg_idx < len(segments) and t >= edges[seg_idx + 1]:
            seg_idx += 1
        leader_cmd = (segments[seg_idx][1], segments[seg_idx][2]) if seg_idx < len(segments) else (0.0, 0.0)

        hist_leader.append((leader.pose, leader.v, leader.w, leader_cmd))
        hist_follower.append(follower.pose)

        if k % steps_per_tick == 0:
            li = len(hist_leader) - 1 - (0 if follower_real else lag)
            fi = len(hist_follower) - 1 - (lag if follower_real else 0)
            L_pose, L_v, L_w, L_cmd = hist_leader[max(li, 0)]
            F_raw = hist_follower[max(fi, 0)]
            ref, F_pose = L_pose, F_raw
            if scenario.compensation and d > 0:
                cmd_delay = d if follower_real else 0.0
                ref = integrate_unicycle(L_pose, L_v, L_w, cmd_delay + (0.0 if follower_real else d))
                if follower_real:
                    F_pose = smith_predict(F_raw, sent, t, d, d)
            err = tracking_error(ref, F_pose)
            cmd = tracking_command(L_cmd[0], L_cmd[1], err, tg, True)
            if follower_real:
                in_flight.append((t + d, cmd))
                sent.append((t, cmd[0], cmd[1]))
                if len(sent) > 200:
                    del sent[0]
            else:
                cmd_follower = cmd
            out_w.append(cmd[1])

        while in_flight and in_flight[0][0] <= t + 1e-9:
            cmd_follower = in_flight.pop(0)[1]

        leader.step(t, leader_cmd, h)
        follower.step(t, cmd_follower, h)

        if k % steps_per_tick == 0:
            e = tracking_error(leader.pose, follower.pose)
            out_t.append(t)
            out_e.append(math.hypot(e[0], e[1]))
            out_he.append(e[2])
            if keep:
                out_l.append((leader.pose.x, leader.pose.y))
                out_f.append((follower.pose.x, follower.pose.y))

    e = np.asarray(out_e)
    he = np.asarray(out_he)
    w = np.asarray(out_w)
    if not np.all(np.isfinite(e)):
        return SimResult(math.inf, math.inf, math.inf, math.inf, math.inf, math.inf)
    rmse = float(np.sqrt(np.mean(e ** 2)))
    rmse_h = float(np.sqrt(np.mean(he ** 2)))
    effort = float(np.mean(np.abs(np.diff(w)))) * rate if len(w) > 1 else 0.0
    cost = rmse + weights.heading * rmse_h + weights.effort * effort
    res = SimResult(rmse, float(e.max()), float(e[-1]), rmse_h, effort, cost)
    if keep:
        res.t, res.error = np.asarray(out_t), e
        res.leader, res.follower = np.asarray(out_l), np.asarray(out_f)
    return res


def evaluate(gains, scenarios, real_model, twin_model, limits=Limits(), weights=CostWeights()):
    results = [simulate(gains, s, real_model, twin_model, limits, weights) for s in scenarios]
    return float(np.mean([r.cost for r in results])), results


def _to_gains(x):
    return {k: float(math.exp(v)) for k, v in zip(("kx", "ky", "ktheta"), x)}


def _clip_log(x):
    lo = np.log([BOUNDS[k][0] for k in ("kx", "ky", "ktheta")])
    hi = np.log([BOUNDS[k][1] for k in ("kx", "ky", "ktheta")])
    return np.clip(x, lo, hi)


def _nelder_mead(f, x0, step=0.4, iters=60, tol=1e-4):
    n = len(x0)
    simplex = [np.asarray(x0, float)] + [np.asarray(x0, float) + step * np.eye(n)[i] for i in range(n)]
    simplex = [_clip_log(s) for s in simplex]
    values = [f(s) for s in simplex]
    for _ in range(iters):
        order = np.argsort(values)
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        if abs(values[-1] - values[0]) < tol:
            break
        centroid = np.mean(simplex[:-1], axis=0)
        xr = _clip_log(centroid + (centroid - simplex[-1]))
        fr = f(xr)
        if fr < values[0]:
            xe = _clip_log(centroid + 2.0 * (centroid - simplex[-1]))
            fe = f(xe)
            simplex[-1], values[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < values[-2]:
            simplex[-1], values[-1] = xr, fr
        else:
            xc = _clip_log(centroid + 0.5 * (simplex[-1] - centroid))
            fc = f(xc)
            if fc < values[-1]:
                simplex[-1], values[-1] = xc, fc
            else:
                best = simplex[0]
                simplex = [best] + [_clip_log(best + 0.5 * (s - best)) for s in simplex[1:]]
                values = [values[0]] + [f(s) for s in simplex[1:]]
    i = int(np.argmin(values))
    return simplex[i], values[i]


@dataclass
class TuningResult:
    gains: dict
    cost: float
    default_cost: float
    evaluations: int
    per_scenario: list
    history: list = field(repr=False, default_factory=list)


def tune(scenarios, real_model=NOMINAL_MODEL, twin_model=NOMINAL_MODEL, limits=Limits(),
         weights=CostWeights(), grid=4, iters=60, start=None, progress=None):
    """Busca kx, ky, kθ que minimizan el costo medio en los escenarios."""
    start = dict(start or DEFAULT_GAINS)
    history = []
    cache = {}

    def f(x):
        key = tuple(np.round(x, 4))
        if key not in cache:
            g = _to_gains(x)
            cost, _ = evaluate(g, scenarios, real_model, twin_model, limits, weights)
            cache[key] = cost
            history.append((g, cost))
            if progress:
                progress(len(history), g, cost)
        return cache[key]

    x_default = np.log([start["kx"], start["ky"], start["ktheta"]])
    default_cost = f(x_default)
    axes = [np.linspace(math.log(BOUNDS[k][0]), math.log(BOUNDS[k][1]), grid + 2)[1:-1]
            for k in ("kx", "ky", "ktheta")]
    candidates = [(default_cost, x_default)]
    for x in itertools.product(*axes):
        candidates.append((f(np.asarray(x)), np.asarray(x)))
    candidates.sort(key=lambda c: c[0])
    best_x, best_cost = candidates[0][1], candidates[0][0]
    for _, x0 in candidates[:2]:
        x, c = _nelder_mead(f, x0, iters=iters)
        if c < best_cost:
            best_x, best_cost = x, c
    gains = _to_gains(best_x)
    if default_cost <= best_cost:
        gains, best_cost = dict(start), default_cost
    _, per = evaluate(gains, scenarios, real_model, twin_model, limits, weights)
    _, per_default = evaluate(start, scenarios, real_model, twin_model, limits, weights)
    per_scenario = [{"escenario": s.name, "rmse_antes": a.rmse, "rmse_despues": b.rmse,
                     "max_antes": a.max_error, "max_despues": b.max_error}
                    for s, a, b in zip(scenarios, per_default, per)]
    return TuningResult(gains, best_cost, default_cost, len(history), per_scenario, history)


def save_gains(path, gains, info=None):
    import yaml
    data = {"twin_bridge": {"ros__parameters": {k: round(float(gains[k]), 4) for k in ("kx", "ky", "ktheta")}}}
    with open(path, "w") as f:
        if info:
            for line in info.splitlines():
                f.write(f"# {line}\n")
        yaml.safe_dump(data, f, sort_keys=False)


def load_gains_file(path):
    """Lee kx, ky, ktheta de un YAML de twin_tune (o de uno plano con esas claves)."""
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if "twin_bridge" in data:
        data = data["twin_bridge"].get("ros__parameters", {})
    gains = {k: float(data[k]) for k in ("kx", "ky", "ktheta") if k in data}
    if len(gains) != 3:
        raise ValueError("el archivo debe tener kx, ky y ktheta")
    return gains


def scenarios_from(patterns, followers, delays_ms, compensation=False, linear_speed=0.15,
                   angular_speed=0.6, distance=1.0, radius=0.5, pause=0.5):
    out = []
    for p, fol, dl in itertools.product(patterns, followers, delays_ms):
        out.append(Scenario(p, fol, dl / 1000.0, compensation, linear_speed, angular_speed, distance, radius, pause))
    return out


__all__ = ["Scenario", "Limits", "CostWeights", "simulate", "evaluate", "tune", "save_gains",
           "load_gains_file", "scenarios_from", "DEFAULT_GAINS", "BOUNDS", "TwinModel"]
