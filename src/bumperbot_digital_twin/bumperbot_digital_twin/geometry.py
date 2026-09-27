"""Geometría 2D (SE(2)) y ley de seguimiento para sincronizar dos robots diferenciales.

No depende de ROS: se puede probar con pytest y reutilizar en el análisis.
"""
import math
from dataclasses import dataclass


def wrap_angle(a):
    """Lleva un ángulo al intervalo (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def compose(self, other):
        """self ⊕ other: aplica `other` expresado en el marco de `self`."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap_angle(self.theta + other.theta),
        )

    def inverse(self):
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            -c * self.x - s * self.y,
            s * self.x - c * self.y,
            wrap_angle(-self.theta),
        )

    def distance_to(self, other):
        return math.hypot(other.x - self.x, other.y - self.y)


def alignment_transform(follower_pose, leader_pose):
    """Transformación T tal que T ⊕ leader_pose = follower_pose.

    Lleva poses del marco odom del líder al marco odom del seguidor.
    """
    return follower_pose.compose(leader_pose.inverse())


def tracking_error(reference, follower):
    """Error de la referencia vista desde el seguidor: (ex, ey, etheta)."""
    dx = reference.x - follower.x
    dy = reference.y - follower.y
    c, s = math.cos(follower.theta), math.sin(follower.theta)
    ex = c * dx + s * dy
    ey = -s * dx + c * dy
    etheta = wrap_angle(reference.theta - follower.theta)
    return ex, ey, etheta


@dataclass
class TrackingGains:
    kx: float = 1.5
    ky: float = 6.0
    ktheta: float = 3.0
    max_linear: float = 0.5
    max_angular: float = 2.5


def clamp(value, limit):
    return max(-limit, min(limit, value))


def tracking_command(v_ref, w_ref, error, gains, feedback=True):
    """Ley de seguimiento de trayectoria (Kanayama et al., 1990) con saturación.

    v = v_r cos(eθ) + kx ex
    ω = ω_r + v_r ky ey + kθ sin(eθ)

    Con feedback=False devuelve solo la prealimentación (espejo de comandos).
    """
    ex, ey, etheta = error
    if not feedback:
        return clamp(v_ref, gains.max_linear), clamp(w_ref, gains.max_angular)
    v = v_ref * math.cos(etheta) + gains.kx * ex
    w = w_ref + v_ref * gains.ky * ey + gains.ktheta * math.sin(etheta)
    return clamp(v, gains.max_linear), clamp(w, gains.max_angular)


def integrate_unicycle(pose, v, w, dt):
    """Integra la cinemática de un robot diferencial durante dt (exacta para v, ω constantes)."""
    if abs(w) < 1e-9:
        return Pose2D(
            pose.x + v * dt * math.cos(pose.theta),
            pose.y + v * dt * math.sin(pose.theta),
            pose.theta,
        )
    r = v / w
    theta_new = pose.theta + w * dt
    return Pose2D(
        pose.x + r * (math.sin(theta_new) - math.sin(pose.theta)),
        pose.y - r * (math.cos(theta_new) - math.cos(pose.theta)),
        wrap_angle(theta_new),
    )
