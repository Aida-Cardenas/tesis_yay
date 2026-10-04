"""Comparación entre el LiDAR real y el simulado, y LiDAR sintético para pruebas.

Los dos escaneos se llevan a una rejilla común de 1° (el RPLidar y el sensor de
Gazebo no tienen el mismo número de muestras ni el mismo ángulo inicial) y se
comparan solo donde ambos ven un obstáculo. Así se valida el modelo de sensor del
gemelo: si el error es pequeño, el LiDAR simulado representa bien al real.
"""
import math

import numpy as np

TWO_PI = 2.0 * math.pi


def bin_scan(angle_min, angle_increment, ranges, range_min, range_max, bins=360, angle_offset=0.0):
    """Rejilla angular fija: en cada sector, el obstáculo más cercano (NaN si no hay)."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_offset + angle_increment * np.arange(r.size)
    idx = np.rint(np.mod(angles, TWO_PI) / TWO_PI * bins).astype(int) % bins
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    out = np.full(bins, np.inf)
    np.minimum.at(out, idx[valid], r[valid])
    out[np.isinf(out)] = np.nan
    return out


def compare_scans(real, twin):
    real = np.asarray(real, dtype=float)
    twin = np.asarray(twin, dtype=float)
    a, b = np.isfinite(real), np.isfinite(twin)
    both = a & b
    n = int(both.sum())
    result = {
        "bins": int(real.size),
        "common": n,
        "coverage_real": float(a.mean()),
        "coverage_twin": float(b.mean()),
        "visibility_agreement": float((a == b).mean()),
        "mae": float("nan"),
        "rmse": float("nan"),
        "bias": float("nan"),
        "p95": float("nan"),
    }
    if n:
        d = real[both] - twin[both]
        result.update(
            mae=float(np.mean(np.abs(d))),
            rmse=float(np.sqrt(np.mean(d ** 2))),
            bias=float(np.mean(d)),
            p95=float(np.percentile(np.abs(d), 95)),
        )
    return result


def raycast_rectangle(x, y, theta, bounds, angles, range_max):
    """Distancias desde (x, y, θ) a las paredes de un rectángulo [xmin, xmax, ymin, ymax]."""
    xmin, xmax, ymin, ymax = bounds
    out = np.full(len(angles), np.inf)
    for i, a in enumerate(angles):
        dx, dy = math.cos(theta + a), math.sin(theta + a)
        best = math.inf
        for wall, d, origin, lo, hi, other_d, other_o in (
            (xmin, dx, x, ymin, ymax, dy, y), (xmax, dx, x, ymin, ymax, dy, y),
            (ymin, dy, y, xmin, xmax, dx, x), (ymax, dy, y, xmin, xmax, dx, x),
        ):
            if abs(d) < 1e-12:
                continue
            t = (wall - origin) / d
            if t <= 0:
                continue
            hit = other_o + t * other_d
            if lo - 1e-9 <= hit <= hi + 1e-9:
                best = min(best, t)
        out[i] = best if best <= range_max else np.inf
    return out


def arena_world_sdf(bounds, wall_height=0.3, wall_thickness=0.05, obstacles=(), name="arena"):
    """Mundo de Gazebo con un recinto rectangular y obstáculos de caja opcionales.

    bounds: [xmin, xmax, ymin, ymax] (cara interior de las paredes), en metros.
    obstacles: lista de (x, y, ancho, largo) de cajas de la misma altura.
    """
    xmin, xmax, ymin, ymax = bounds
    if xmax <= xmin or ymax <= ymin:
        raise ValueError("bounds inválidos")
    t, h = wall_thickness, wall_height
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    lx, ly = xmax - xmin + 2 * t, ymax - ymin + 2 * t
    boxes = [
        ("pared_oeste", xmin - t / 2, cy, t, ly),
        ("pared_este", xmax + t / 2, cy, t, ly),
        ("pared_sur", cx, ymin - t / 2, lx, t),
        ("pared_norte", cx, ymax + t / 2, lx, t),
    ]
    boxes += [(f"obstaculo_{i}", ox, oy, sx, sy) for i, (ox, oy, sx, sy) in enumerate(obstacles)]

    def box(name_, px, py, sx, sy):
        geom = f"<geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>"
        return (f'<model name="{name_}"><static>true</static><pose>{px:.3f} {py:.3f} {h / 2:.3f} 0 0 0</pose>'
                f'<link name="link"><collision name="collision">{geom}</collision>'
                f'<visual name="visual">{geom}<material><ambient>0.6 0.45 0.3 1</ambient>'
                f'<diffuse>0.6 0.45 0.3 1</diffuse></material></visual></link></model>')

    models = "\n    ".join(box(*b) for b in boxes)
    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{name}">
    <physics type="ode"><max_step_size>0.01</max_step_size></physics>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>
    {models}
  </world>
</sdf>
"""


def arena_map(bounds, wall_thickness=0.05, resolution=0.05, margin=0.5, obstacles=()):
    """Mapa de ocupación (PGM + YAML de map_server) del mismo recinto que arena_world_sdf.

    Devuelve (imagen uint8, origen [x, y, 0]). 0 = ocupado, 254 = libre, 205 = desconocido.
    """
    xmin, xmax, ymin, ymax = bounds
    t = wall_thickness
    ox, oy = xmin - t - margin, ymin - t - margin
    w = int(math.ceil((xmax - xmin + 2 * (t + margin)) / resolution))
    h = int(math.ceil((ymax - ymin + 2 * (t + margin)) / resolution))
    cx = ox + (np.arange(w) + 0.5) * resolution
    cy = oy + (np.arange(h) + 0.5) * resolution
    X, Y = np.meshgrid(cx, cy)
    img = np.full((h, w), 205, np.uint8)
    inside = (X >= xmin) & (X <= xmax) & (Y >= ymin) & (Y <= ymax)
    walls = (X >= xmin - t) & (X <= xmax + t) & (Y >= ymin - t) & (Y <= ymax + t) & ~inside
    img[inside] = 254
    img[walls] = 0
    for (bx, by, sx, sy) in obstacles:
        img[(np.abs(X - bx) <= sx / 2) & (np.abs(Y - by) <= sy / 2)] = 0
    return img[::-1], [round(ox, 4), round(oy, 4), 0.0]


def write_map(directory, image, origin, resolution=0.05):
    import os
    os.makedirs(directory, exist_ok=True)
    h, w = image.shape
    with open(os.path.join(directory, "map.pgm"), "wb") as f:
        f.write(f"P5\n# mapa generado por twin_calibrate arena\n{w} {h}\n255\n".encode())
        f.write(image.tobytes())
    with open(os.path.join(directory, "map.yaml"), "w") as f:
        f.write(f"image: map.pgm\nmode: trinary\nresolution: {resolution:.6f}\n"
                f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.000000]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
