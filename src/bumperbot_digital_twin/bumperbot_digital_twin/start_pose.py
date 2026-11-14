"""Devuelve los robots a la marca de salida antes de cada corrida.

El puente realinea las poses al empezar una corrida, pero eso solo cambia la
cuenta: el gemelo de Gazebo seguía donde terminó la corrida anterior. Entonces
su LiDAR veía las paredes desde otro lugar (el MAE del LiDAR salía de decenas de
centímetros) y, después de varias rectas en el mismo sentido, podía chocar con
una pared en medio de una corrida.

Antes de cada corrida:
- el gemelo en Gazebo se lleva a la marca con el servicio set_pose del mundo,
- un robot de mentira (real o gemelo) vuelve a la marca con su servicio
  /fake_robot/return_to_start, como si alguien lo colocara ahí,
- el robot físico lo coloca la persona (el ejecutor se lo pide).

La odometría de cada robot no se toca: igual que con el robot físico, solo cambia
dónde está de verdad. Por eso el puente vuelve a alinear después.
"""
import math
import shutil
import subprocess

FAKE_SERVICE = "/fake_robot/return_to_start"
SET_POSE_TYPES = (
    ("ign", "ignition.msgs.Pose", "ignition.msgs.Boolean"),
    ("gz", "gz.msgs.Pose", "gz.msgs.Boolean"),
)


def set_pose_request(model, x=0.0, y=0.0, z=0.0, yaw=0.0):
    """Texto del mensaje Pose para el servicio set_pose de Gazebo."""
    return (f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0):.6f}, w: {math.cos(yaw / 2.0):.6f}}}')


def gazebo_set_pose(world, model, x=0.0, y=0.0, z=0.0, yaw=0.0, timeout_ms=3000,
                    run=subprocess.run, which=shutil.which):
    """Mueve un modelo de Gazebo a una pose. Devuelve (éxito, detalle)."""
    detail = "no está instalada la herramienta ign ni gz"
    for tool, req_type, rep_type in SET_POSE_TYPES:
        if which(tool) is None:
            continue
        cmd = [tool, "service", "-s", f"/world/{world}/set_pose", "--reqtype", req_type,
               "--reptype", rep_type, "--timeout", str(timeout_ms),
               "--req", set_pose_request(model, x, y, z, yaw)]
        try:
            out = run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5.0)
        except (OSError, subprocess.TimeoutExpired) as exc:
            detail = f"{tool}: {exc}"
            continue
        if "data: true" in out.stdout:
            return True, tool
        detail = f"{tool}: {(out.stdout + out.stderr).strip() or 'sin respuesta'}"
    return False, detail


class StartReset:
    """Lleva los dos robots a la marca de salida (los que se pueden mover solos)."""

    def __init__(self, dual, world="arena", model="bumperbot", settle_s=1.5, set_pose=gazebo_set_pose):
        from std_srvs.srv import Trigger
        self.Trigger = Trigger
        self.dual = dual
        self.world = world
        self.model = model
        self.settle_s = settle_s
        self.set_pose = set_pose
        self.clients = {side: dual.nodes[side].create_client(Trigger, FAKE_SERVICE) for side in ("real", "twin")}

    def _fake(self, side):
        """True/False si hay un robot de mentira en ese dominio, None si no lo hay."""
        client = self.clients[side]
        if not client.wait_for_service(timeout_sec=0.5):
            return None
        return bool(self.dual.call(side, client, self.Trigger.Request(), timeout=5.0).success)

    def run(self):
        """Devuelve (el gemelo quedó en la marca, texto para la persona)."""
        notes = []
        if self._fake("real"):
            notes.append("robot real de mentira en la marca")
        twin = self._fake("twin")
        if twin is None:
            twin, detail = self.set_pose(self.world, self.model)
            notes.append("gemelo en la marca (Gazebo)" if twin else
                         f"ADVERTENCIA: no pude llevar el gemelo a la marca ({detail}); su LiDAR no se "
                         "puede comparar y podría chocar con una pared")
        else:
            notes.append("gemelo de mentira en la marca" if twin else
                         "ADVERTENCIA: el gemelo de mentira no volvió a la marca")
        return bool(twin), " · ".join(notes)
