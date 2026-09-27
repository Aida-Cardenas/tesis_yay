"""Recorridos de prueba como listas de tramos (sin dependencias de ROS)."""
import math


PATTERNS = ("line", "line_back", "square", "circle", "rotate", "figure8")


def build_segments(pattern, v, w, distance, radius, laps, pause):
    """Lista de (duración [s], v [m/s], ω [rad/s])."""
    stop = [(pause, 0.0, 0.0)] if pause > 0 else []
    if pattern == "line":
        one = [(distance / v, v, 0.0)]
    elif pattern == "line_back":
        one = [(distance / v, v, 0.0), *stop, (math.pi / w, 0.0, w), *stop, (distance / v, v, 0.0)]
    elif pattern == "square":
        one = []
        for _ in range(4):
            one += [(distance / v, v, 0.0), *stop, ((math.pi / 2) / w, 0.0, w), *stop]
    elif pattern == "circle":
        one = [(2 * math.pi * radius / v, v, v / radius)]
    elif pattern == "rotate":
        one = [(2 * math.pi / w, 0.0, w)]
    elif pattern == "figure8":
        one = [(2 * math.pi * radius / v, v, v / radius), *stop, (2 * math.pi * radius / v, v, -v / radius)]
    else:
        raise ValueError(f"pattern debe ser uno de {PATTERNS}")
    segments = []
    for _ in range(laps):
        segments += one + stop
    return segments
