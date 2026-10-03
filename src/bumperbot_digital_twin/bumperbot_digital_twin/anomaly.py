"""Detección de anomalías usando al gemelo como referencia de comportamiento esperado.

El modelo del gemelo (dynamics.TwinModel) predice qué velocidad debería tener el
robot real ante cada comando. Las anomalías son diferencias sostenidas entre esa
predicción, lo que miden los encoders y lo que mide la IMU:

- atasco: se ordena movimiento y el robot no se mueve (rueda trabada, choque).
- deslizamiento: las ruedas dicen que el robot gira pero la IMU no lo confirma.
- movimiento_no_comandado: el robot se mueve sin comando (empujado, levantado).
- desviacion_modelo: la velocidad real se aparta del modelo de forma acumulada
  (CUSUM), aunque cada muestra por sí sola parezca normal.

Cada condición tiene histéresis temporal para no disparar por ruido.
"""
from dataclasses import dataclass

from bumperbot_digital_twin.dynamics import NOMINAL_MODEL, FirstOrderFilter

DESCRIPTIONS = {
    "atasco": "Se ordena movimiento pero el robot no se mueve",
    "deslizamiento": "Las ruedas giran pero la IMU no registra el giro",
    "movimiento_no_comandado": "El robot se mueve sin que se le ordene",
    "desviacion_modelo": "La respuesta del robot se aleja de la predicha por el gemelo",
    "latencia_alta": "La latencia de la red supera el umbral",
    "perdida_datos": "Dejaron de llegar datos de uno de los robots",
}
SEVERITY = {
    "atasco": 2,
    "deslizamiento": 1,
    "movimiento_no_comandado": 2,
    "desviacion_modelo": 1,
    "latencia_alta": 1,
    "perdida_datos": 2,
}


@dataclass
class DetectorConfig:
    stall_min_v: float = 0.05
    stall_min_w: float = 0.4
    stall_ratio: float = 0.25
    stall_time: float = 0.6
    slip_abs: float = 0.4
    slip_ratio: float = 0.5
    slip_time: float = 0.4
    idle_time: float = 0.5
    unexpected_v: float = 0.05
    unexpected_w: float = 0.35
    unexpected_time: float = 0.5
    cusum_k_v: float = 0.03
    cusum_h_v: float = 0.06
    cusum_k_w: float = 0.2
    cusum_h_w: float = 0.4
    clear_time: float = 0.5


@dataclass
class AnomalyEvent:
    t: float
    kind: str
    phase: str
    value: float
    severity: int
    description: str


class Hysteresis:
    """Una condición se activa si se cumple durante on_time y se apaga tras off_time sin cumplirse."""

    def __init__(self, on_time, off_time):
        self.on_time = on_time
        self.off_time = off_time
        self.active = False
        self.since = None

    def update(self, t, condition):
        if condition != self.active:
            if self.since is None:
                self.since = t
            if t - self.since >= (self.on_time if condition else self.off_time):
                self.active = condition
                self.since = None
                return "inicio" if condition else "fin"
        else:
            self.since = None
        return None


class AnomalyDetector:
    def __init__(self, model=None, config=None):
        self.model = model or NOMINAL_MODEL
        self.cfg = config or DetectorConfig()
        c = self.cfg
        self.pred_v = FirstOrderFilter(self.model.linear)
        self.pred_w = FirstOrderFilter(self.model.angular)
        self.conditions = {
            "atasco": Hysteresis(c.stall_time, c.clear_time),
            "deslizamiento": Hysteresis(c.slip_time, c.clear_time),
            "movimiento_no_comandado": Hysteresis(c.unexpected_time, c.clear_time),
            "desviacion_modelo": Hysteresis(0.0, c.clear_time),
        }
        self.last_cmd_t = None
        self.t_prev = None
        self.cusum = {"v+": 0.0, "v-": 0.0, "w+": 0.0, "w-": 0.0}
        self.last_values = {}

    @property
    def active(self):
        return sorted(k for k, h in self.conditions.items() if h.active)

    def _cusum(self, key_pos, key_neg, r, k, dt):
        self.cusum[key_pos] = max(0.0, self.cusum[key_pos] + (r - k) * dt)
        self.cusum[key_neg] = max(0.0, self.cusum[key_neg] + (-r - k) * dt)
        return max(self.cusum[key_pos], self.cusum[key_neg])

    def update(self, t, cmd_v, cmd_w, meas_v, meas_w, imu_w=None):
        c = self.cfg
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        pv = self.pred_v.update(t, cmd_v)
        pw = self.pred_w.update(t, cmd_w)
        if abs(cmd_v) > 1e-3 or abs(cmd_w) > 1e-3:
            self.last_cmd_t = t
        idle = self.last_cmd_t is None or t - self.last_cmd_t > c.idle_time

        stall_v = abs(pv) > c.stall_min_v and abs(meas_v) < c.stall_ratio * abs(pv)
        stall_w = abs(pw) > c.stall_min_w and abs(meas_w) < c.stall_ratio * abs(pw)
        stall = stall_v or stall_w

        slip = False
        if imu_w is not None:
            diff = abs(meas_w - imu_w)
            slip = diff > c.slip_abs and diff > c.slip_ratio * max(abs(meas_w), abs(imu_w))

        unexpected = idle and (abs(meas_v) > c.unexpected_v or abs(meas_w) > c.unexpected_w)

        sv = self._cusum("v+", "v-", meas_v - pv, c.cusum_k_v, dt)
        sw = self._cusum("w+", "w-", meas_w - pw, c.cusum_k_w, dt)
        other_fault = (stall or unexpected or self.conditions["atasco"].active
                       or self.conditions["movimiento_no_comandado"].active)
        if other_fault:
            for key in self.cusum:
                self.cusum[key] = 0.0
            sv = sw = 0.0
        deviation = sv > c.cusum_h_v or sw > c.cusum_h_w

        values = {
            "atasco": abs(meas_v - pv) if stall_v else abs(meas_w - pw),
            "deslizamiento": abs(meas_w - imu_w) if imu_w is not None else 0.0,
            "movimiento_no_comandado": max(abs(meas_v), abs(meas_w)),
            "desviacion_modelo": max(sv / c.cusum_h_v, sw / c.cusum_h_w),
        }
        self.last_values = values
        events = []
        for kind, cond in (("atasco", stall), ("deslizamiento", slip),
                           ("movimiento_no_comandado", unexpected), ("desviacion_modelo", deviation)):
            phase = self.conditions[kind].update(t, cond)
            if phase:
                events.append(AnomalyEvent(t, kind, phase, values[kind], SEVERITY[kind], DESCRIPTIONS[kind]))
                if kind == "desviacion_modelo" and phase == "fin":
                    for key in self.cusum:
                        self.cusum[key] = 0.0
        return events
