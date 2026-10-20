"""Modelo dinámico del robot e identificación de sistemas para calibrar el gemelo.

Cada canal (velocidad lineal y angular) se modela como un sistema de primer orden
con retardo puro, el modelo más usado para identificar actuadores:

    G(s) = K · e^(-L s) / (τ s + 1)

K es la ganancia (cuánto de la velocidad pedida se consigue), τ la constante de
tiempo (qué tan rápido responde) y L el retardo (cuánto tarda en empezar). Los
parámetros se estiman por mínimos cuadrados a partir de los registros del robot
real, y el gemelo los usa para responder igual que él.
"""
import math
from collections import deque
from dataclasses import asdict, dataclass, field

import numpy as np


@dataclass
class FirstOrderModel:
    gain: float = 1.0
    tau: float = 0.0
    delay: float = 0.0
    r2: float = float("nan")
    samples: int = 0
    r2_step: float = float("nan")

    def discrete(self, dt):
        """Coeficientes (a, b) de y[k+1] = a·y[k] + b·u[k-d] con retención de orden cero."""
        a = 0.0 if self.tau <= 1e-9 else math.exp(-dt / self.tau)
        return a, self.gain * (1.0 - a)


@dataclass
class TwinModel:
    linear: FirstOrderModel = field(default_factory=FirstOrderModel)
    angular: FirstOrderModel = field(default_factory=FirstOrderModel)
    source: str = ""

    def to_dict(self):
        return {"linear": asdict(self.linear), "angular": asdict(self.angular), "source": self.source}

    @classmethod
    def from_dict(cls, data):
        def channel(d):
            d = dict(d or {})
            return FirstOrderModel(**{k: float(v) if k != "samples" else int(v)
                                      for k, v in d.items() if k in FirstOrderModel.__dataclass_fields__})
        return cls(channel(data.get("linear")), channel(data.get("angular")), str(data.get("source", "")))

    def save(self, path):
        import yaml
        with open(path, "w") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False, allow_unicode=True)

    @classmethod
    def load(cls, path):
        import yaml
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f) or {})


NOMINAL_MODEL = TwinModel(FirstOrderModel(1.0, 0.1, 0.0), FirstOrderModel(1.0, 0.1, 0.0), "nominal")


class FirstOrderFilter:
    """Aplica un FirstOrderModel en línea, con pasos de tiempo variables."""

    def __init__(self, model):
        self.model = model
        self.y = 0.0
        self.t_prev = None
        self.buffer = deque()
        self.u_delayed = 0.0

    def reset(self):
        self.y = 0.0
        self.t_prev = None
        self.buffer.clear()
        self.u_delayed = 0.0

    def update(self, t, u):
        """Avanza el modelo hasta t y registra la entrada u aplicada desde t.

        Entre el paso anterior y t actúa la entrada aplicada L segundos antes del paso
        anterior (retención de orden cero), igual que en y[k+1] = a·y[k] + b·u[k-d],
        la forma con la que fit_first_order identifica el modelo. La entrada nueva se
        registra después, así el retardo efectivo es exactamente L.
        """
        if self.t_prev is None:
            self.t_prev = t
            self.buffer.append((t, u))
            return self.y
        limit = self.t_prev - self.model.delay + 1e-9
        while self.buffer and self.buffer[0][0] <= limit:
            self.u_delayed = self.buffer.popleft()[1]
        dt = max(0.0, t - self.t_prev)
        alpha = 1.0 if self.model.tau <= 1e-9 else 1.0 - math.exp(-dt / self.model.tau)
        self.y += alpha * (self.model.gain * self.u_delayed - self.y)
        self.buffer.append((t, u))
        self.t_prev = t
        return self.y


def simulate(model, t, u):
    f = FirstOrderFilter(model)
    return np.array([f.update(ti, ui) for ti, ui in zip(t, u)])


def resample(t, *signals, dt=None):
    t = np.asarray(t, dtype=float)
    ok = np.isfinite(t)
    for s in signals:
        ok &= np.isfinite(np.asarray(s, dtype=float))
    t = t[ok]
    if t.size < 3:
        raise ValueError("Muy pocas muestras válidas")
    order = np.argsort(t)
    t = t[order]
    if dt is None:
        dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    return grid, dt, [np.interp(grid, t, np.asarray(s, dtype=float)[ok][order]) for s in signals]


def fit_first_order(t, u, y, max_delay=0.6, dt=None):
    """Identifica (K, τ, L) por mínimos cuadrados probando cada retardo posible.

    Ajusta y[k+1] = a·y[k] + b·u[k-d] para d = 0..max_delay/dt y se queda con el de
    menor error. K = b/(1-a), τ = -dt/ln(a), L = d·dt.
    """
    grid, dt, (U, Y) = resample(t, u, y, dt=dt)
    n = len(grid)
    if np.std(U) < 1e-6:
        raise ValueError("La entrada no varía: el robot no recibió comandos en este registro")
    best = None
    for d in range(0, int(round(max_delay / dt)) + 1):
        if n - 1 - d < 10:
            break
        X = np.column_stack([Y[d:n - 1], U[0:n - 1 - d]])
        target = Y[d + 1:n]
        coef, *_ = np.linalg.lstsq(X, target, rcond=None)
        a, b = float(coef[0]), float(coef[1])
        if not (0.0 <= a < 1.0) or abs(b) < 1e-9:
            continue
        sse = float(np.sum((target - X @ coef) ** 2))
        if best is None or sse < best[0]:
            best = (sse, a, b, d, target)
    if best is None:
        raise ValueError("No se encontró un modelo de primer orden estable")
    sse, a, b, d, target = best
    sst = float(np.sum((target - np.mean(target)) ** 2)) or 1e-12
    tau = 0.0 if a <= 1e-9 else -dt / math.log(a)
    model = FirstOrderModel(gain=b / (1.0 - a), tau=tau, delay=d * dt, samples=n, r2_step=1.0 - sse / sst)
    model.r2 = simulated_r2(model, grid, U, Y)
    return model


def simulated_r2(model, t, u, y):
    """R² de la respuesta simulada completa frente a la medida.

    A diferencia del R² de un paso (r2_step), que predice cada muestra a partir de la
    medida anterior y sale cercano a 1 casi siempre que dt es mucho menor que τ, este
    compara la respuesta que produce el modelo solo con los comandos.
    """
    y = np.asarray(y, float)
    sim = simulate(model, np.asarray(t, float), np.asarray(u, float))
    sst = float(np.sum((y - np.mean(y)) ** 2)) or 1e-12
    return 1.0 - float(np.sum((y - sim) ** 2)) / sst


def fit_error(model, t, u, y):
    """RMSE entre la respuesta medida y la que predice el modelo."""
    grid, _, (U, Y) = resample(t, u, y)
    return float(np.sqrt(np.mean((simulate(model, grid, U) - Y) ** 2)))


def wheel_calibration(odom_distance, real_distance, odom_angle=None, real_angle=None):
    """Multiplicadores de diff_drive_controller a partir de mediciones con cinta métrica.

    Si la odometría dice que avanzó odom_distance pero en realidad avanzó real_distance,
    el radio real es real/odom veces el nominal. Con el radio ya corregido, la diferencia
    entre el giro medido y el real se atribuye a la separación entre ruedas.
    """
    if odom_distance <= 0 or real_distance <= 0:
        raise ValueError("Las distancias deben ser positivas")
    radius = real_distance / odom_distance
    separation = 1.0
    if odom_angle and real_angle:
        if odom_angle <= 0 or real_angle <= 0:
            raise ValueError("Los ángulos deben ser positivos")
        separation = radius * odom_angle / real_angle
    return {
        "left_wheel_radius_multiplier": radius,
        "right_wheel_radius_multiplier": radius,
        "wheel_separation_multiplier": separation,
    }
