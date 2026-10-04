"""Cálculos de la vista previa de navegación (sin dependencias de ROS).

- Métricas para comparar la ruta que predijo el gemelo con la que hizo el robot real.
- Criterios para aprobar o rechazar una meta según lo que pasó en el gemelo.
- El controlador sencillo del navegador de mentira (para probar sin Nav2).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from bumperbot_digital_twin.geometry import wrap_angle


def as_xy(path):
    a = np.asarray(path, float)
    if a.ndim != 2 or a.shape[0] == 0:
        return np.zeros((0, 2))
    return a[:, :2]


def path_length(path):
    p = as_xy(path)
    if len(p) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(p, axis=0).T)))


def point_to_polyline(points, polyline):
    """Distancia de cada punto al segmento más cercano de la polilínea."""
    pts, poly = as_xy(points), as_xy(polyline)
    if len(pts) == 0 or len(poly) == 0:
        return np.zeros(0)
    if len(poly) == 1:
        return np.hypot(*(pts - poly[0]).T)
    a, b = poly[:-1], poly[1:]
    ab = b - a
    len2 = np.maximum(np.sum(ab ** 2, axis=1), 1e-12)
    d = np.empty(len(pts))
    for i, p in enumerate(pts):
        t = np.clip(np.sum((p - a) * ab, axis=1) / len2, 0.0, 1.0)
        proj = a + ab * t[:, None]
        d[i] = float(np.min(np.hypot(*(p - proj).T)))
    return d


def compare_paths(predicted, actual):
    """Qué tanto se parece la ruta real a la que predijo el gemelo."""
    pred, act = as_xy(predicted), as_xy(actual)
    if len(pred) == 0 or len(act) == 0:
        return {}
    d_ap = point_to_polyline(act, pred)
    d_pa = point_to_polyline(pred, act)
    return {
        "desviacion_media_m": float(np.mean(d_ap)),
        "desviacion_p95_m": float(np.percentile(d_ap, 95)),
        "desviacion_max_m": float(np.max(d_ap)),
        "hausdorff_m": float(max(np.max(d_ap), np.max(d_pa))),
        "longitud_prevista_m": path_length(pred),
        "longitud_real_m": path_length(act),
        "diferencia_final_m": float(np.hypot(*(act[-1] - pred[-1]))),
    }


@dataclass
class ApprovalCriteria:
    min_clearance: float = 0.20
    max_time: float = 120.0
    max_recoveries: int = 2
    max_final_error: float = 0.25


@dataclass
class TwinRun:
    succeeded: bool
    status: str
    duration: float
    path: list = field(default_factory=list)
    min_clearance: float = math.inf
    recoveries: int = 0
    final_error: float = math.nan


def approve(run, criteria=ApprovalCriteria()):
    """(aprobada, motivos) para ejecutar la meta en el robot real."""
    reasons = []
    if not run.succeeded:
        reasons.append(f"el gemelo no llegó a la meta ({run.status})")
    if math.isfinite(run.min_clearance) and run.min_clearance < criteria.min_clearance:
        reasons.append(f"pasó a {run.min_clearance * 100:.0f} cm de un obstáculo "
                       f"(mínimo {criteria.min_clearance * 100:.0f} cm)")
    if run.duration > criteria.max_time:
        reasons.append(f"tardó {run.duration:.0f} s (máximo {criteria.max_time:.0f} s)")
    if run.recoveries > criteria.max_recoveries:
        reasons.append(f"necesitó {run.recoveries} maniobras de recuperación (máximo {criteria.max_recoveries})")
    if run.succeeded and math.isfinite(run.final_error) and run.final_error > criteria.max_final_error:
        reasons.append(f"quedó a {run.final_error * 100:.0f} cm de la meta")
    return not reasons, reasons


def min_valid_range(ranges, range_min, range_max, ignore_below=0.0):
    r = np.asarray(ranges, float)
    ok = np.isfinite(r) & (r >= max(range_min, ignore_below)) & (r <= range_max)
    return float(r[ok].min()) if ok.any() else math.inf


@dataclass
class GoToGoalGains:
    max_linear: float = 0.2
    max_angular: float = 1.0
    k_linear: float = 0.8
    k_angular: float = 2.0
    turn_in_place: float = 0.5
    position_tolerance: float = 0.05
    yaw_tolerance: float = 0.05


def go_to_goal(x, y, yaw, gx, gy, gyaw, gains=GoToGoalGains()):
    """(v, ω, terminado): gira hacia la meta, avanza y al final se orienta."""
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist > gains.position_tolerance:
        heading_err = wrap_angle(math.atan2(dy, dx) - yaw)
        w = max(-gains.max_angular, min(gains.max_angular, gains.k_angular * heading_err))
        if abs(heading_err) > gains.turn_in_place:
            return 0.0, w, False
        v = min(gains.max_linear, gains.k_linear * dist) * math.cos(heading_err)
        return v, w, False
    yaw_err = wrap_angle(gyaw - yaw)
    if abs(yaw_err) > gains.yaw_tolerance:
        w = max(-gains.max_angular, min(gains.max_angular, gains.k_angular * yaw_err))
        if abs(w) < 0.15:
            w = math.copysign(0.15, w)
        return 0.0, w, False
    return 0.0, 0.0, True


def inside_arena(x, y, arena, margin):
    if len(arena) != 4:
        return True
    xmin, xmax, ymin, ymax = arena
    return xmin + margin <= x <= xmax - margin and ymin + margin <= y <= ymax - margin
