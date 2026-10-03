#!/usr/bin/env python3
"""Capa de comunicación bidireccional entre el robot físico y su gemelo digital.

El robot físico y el gemelo corren el mismo software (bumperbot_bringup) en dos
dominios DDS distintos (ROS_DOMAIN_ID), así sus tópicos no se mezclan. Este nodo
se une a los dos dominios a la vez y:

1. Toma el comando de velocidad del líder y lo envía al seguidor (prealimentación).
2. Compara las odometrías y corrige al seguidor con una ley de seguimiento
   (Kanayama et al., 1990) para que no se aleje por deriva o diferencias del modelo.
3. Compensa la latencia de la red: predice la pose del líder por el retardo medido
   y, cuando el seguidor es el robot real, usa un predictor de Smith con los
   comandos que todavía van "en vuelo".
4. Puede dar al gemelo la dinámica identificada del robot real (calibración),
   para que responda a los comandos igual que él.
5. Puede degradar artificialmente el enlace con el robot real (retardo, variación
   y pérdida) para estudiar los límites de la sincronización.
6. Reenvía el LiDAR real al dominio del gemelo y compara ambos escaneos.
7. Reúne las anomalías del detector y las suyas propias (latencia alta, pérdida
   de datos) y las publica en ambos dominios.
8. Publica el estado y las trayectorias en ambos dominios y lo registra todo en CSV.

El líder puede ser "real" o "twin" y se puede cambiar en caliente: los comandos
fluyen en el sentido que se elija y el estado de ambos robots fluye siempre en
los dos sentidos.
"""
import argparse
import copy
import csv
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rcl_interfaces.msg import SetParametersResult

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger

from bumperbot_msgs.msg import ScanComparison, TwinAnomaly, TwinSyncStatus
from bumperbot_msgs.srv import TwinConfigure
from bumperbot_digital_twin.anomaly import DESCRIPTIONS, SEVERITY, Hysteresis
from bumperbot_digital_twin.dynamics import FirstOrderFilter, TwinModel
from bumperbot_digital_twin.geometry import (
    Pose2D,
    TrackingGains,
    alignment_transform,
    clamp,
    integrate_unicycle,
    smith_predict,
    tracking_command,
    tracking_error,
    yaw_from_quaternion,
)
from bumperbot_digital_twin.netem import NetworkEmulator
from bumperbot_digital_twin.ros_util import spin_resilient
from bumperbot_digital_twin.scan_compare import bin_scan, compare_scans

SIDES = ("real", "twin")
NAN = float("nan")

CSV_COLUMNS = [
    "t", "leader", "feedback", "compensation", "twin_model", "net_delay_ms", "net_loss",
    "active", "lost_sync", "stale",
    "pos_error", "pos_error_raw", "yaw_error", "ex", "ey",
    "leader_x", "leader_y", "leader_yaw",
    "follower_x", "follower_y", "follower_yaw",
    "real_x", "real_y", "real_yaw", "real_v", "real_w", "real_stamp",
    "twin_x", "twin_y", "twin_yaw", "twin_v", "twin_w",
    "ff_v", "ff_w", "cmd_v", "cmd_w",
    "rtt_ms", "one_way_ms", "real_odom_age_ms", "leader_cmd_age_ms",
    "anomalies",
]
SCAN_COLUMNS = ["t", "common", "mae", "rmse", "bias", "p95",
                "coverage_real", "coverage_twin", "visibility_agreement", "pos_error"]
EVENT_COLUMNS = ["t", "source", "kind", "phase", "severity", "value", "description"]

DEFAULTS = {
    "leader": "real",
    "odom_topic": "/bumperbot_controller/odom",
    "cmd_topic": "/bumperbot_controller/cmd_vel",
    "follow_topic": "twin_vel",
    "scan_topic": "/scan",
    "odom_frame": "odom",
    "rate": 20.0,
    "feedback": True,
    "feedforward_source": "cmd",
    "kx": 1.5,
    "ky": 6.0,
    "ktheta": 3.0,
    "max_linear": 0.5,
    "max_angular": 2.5,
    "real_max_linear": 0.3,
    "real_max_angular": 1.5,
    "state_timeout": 0.5,
    "cmd_timeout": 0.3,
    "position_tolerance": 0.02,
    "heading_tolerance": 0.03,
    "max_correction_error": 0.75,
    "auto_align": True,
    "measure_rtt": True,
    "ping_rate": 5.0,
    "compensation": False,
    "compensation_extra_ms": 0.0,
    "twin_model": False,
    "twin_model_file": "",
    "net_delay_ms": 0.0,
    "net_jitter_ms": 0.0,
    "net_loss": 0.0,
    "net_seed": 7,
    "scan_compare": True,
    "relay_real_scan": True,
    "real_scan_frame": "laser_link",
    "scan_compare_rate": 2.0,
    "scan_sync_tolerance": 0.05,
    "scan_max_linear": 0.05,
    "scan_max_angular": 0.3,
    "latency_alert_ms": 250.0,
    "log_enabled": True,
    "log_dir": "~/twin_logs",
    "log_tag": "",
    "path_max_len": 5000,
}
RUNTIME_FLOATS = ("kx", "ky", "ktheta", "max_linear", "max_angular", "real_max_linear",
                  "real_max_angular", "position_tolerance", "heading_tolerance",
                  "max_correction_error", "compensation_extra_ms", "latency_alert_ms",
                  "scan_sync_tolerance")


def other(side):
    return "twin" if side == "real" else "real"


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wall_time_msg():
    now = time.time()
    msg = Time()
    msg.sec = int(now)
    msg.nanosec = int((now - int(now)) * 1e9)
    return msg


def is_true(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes")


class RobotState:
    def __init__(self):
        self.pose = None
        self.v = 0.0
        self.w = 0.0
        self.odom_rx = None
        self.odom_stamp = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.cmd_rx = None
        self.cmd_stamp = None
        self.scan = None
        self.scan_rx = None


class Side:
    """Todo lo que el puente hace dentro de un dominio (real o twin)."""

    def __init__(self, name, node, bridge):
        self.name = name
        self.node = node
        self.bridge = bridge
        self.state = RobotState()
        p = bridge.p
        node.create_subscription(Odometry, p["odom_topic"], bridge.incoming(name, self._odom_cb), 10)
        node.create_subscription(TwistStamped, p["cmd_topic"], bridge.incoming(name, self._cmd_cb), 10)
        if p["scan_compare"] or (name == "real" and p["relay_real_scan"]):
            node.create_subscription(LaserScan, p["scan_topic"], bridge.incoming(name, self._scan_cb),
                                     qos_profile_sensor_data)
        self.follow_pub = node.create_publisher(Twist, p["follow_topic"], 10)
        self.status_pub = node.create_publisher(TwinSyncStatus, "/digital_twin/status", 10)
        self.event_pub = node.create_publisher(TwinAnomaly, "/digital_twin/events", 10)
        self.scan_cmp_pub = node.create_publisher(ScanComparison, "/digital_twin/scan_comparison", 10)
        self.path_pubs = {s: node.create_publisher(Path, f"/digital_twin/{s}_path", 10) for s in SIDES}
        self.pose_pubs = {s: node.create_publisher(PoseStamped, f"/digital_twin/{s}_pose", 10) for s in SIDES}
        self.real_scan_pub = (node.create_publisher(LaserScan, "/digital_twin/real_scan", qos_profile_sensor_data)
                              if name == "twin" else None)
        self.paths = {s: Path() for s in SIDES}
        node.create_service(Trigger, "/digital_twin/align", bridge.srv_align)
        node.create_service(SetBool, "/digital_twin/set_feedback", bridge.srv_set_feedback)
        node.create_service(Trigger, "/digital_twin/switch_leader", bridge.srv_switch_leader)
        node.create_service(Trigger, "/digital_twin/new_log", bridge.srv_new_log)
        node.create_service(TwinConfigure, "/digital_twin/configure", bridge.srv_configure)

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        with self.bridge.lock:
            s = self.state
            s.pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                            yaw_from_quaternion(q.x, q.y, q.z, q.w))
            s.v = msg.twist.twist.linear.x
            s.w = msg.twist.twist.angular.z
            s.odom_rx = time.monotonic()
            s.odom_stamp = stamp_to_sec(msg.header.stamp)
            if self.name == "real":
                self.bridge.real_odom_age_ms = (time.time() - s.odom_stamp) * 1000.0

    def _cmd_cb(self, msg):
        with self.bridge.lock:
            s = self.state
            s.cmd_v = msg.twist.linear.x
            s.cmd_w = msg.twist.angular.z
            s.cmd_rx = time.monotonic()
            s.cmd_stamp = stamp_to_sec(msg.header.stamp)

    def _scan_cb(self, msg):
        binned = bin_scan(msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max)
        with self.bridge.lock:
            self.state.scan = binned
            self.state.scan_rx = time.monotonic()
        if self.name == "real" and self.bridge.p["relay_real_scan"]:
            self.bridge.relay_real_scan(msg)

    def pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = self.bridge.p["odom_frame"]
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation.z = math.sin(pose.theta / 2.0)
        msg.pose.orientation.w = math.cos(pose.theta / 2.0)
        return msg

    def publish_poses(self, poses_in_this_frame):
        for s, pose in poses_in_this_frame.items():
            if pose is None:
                continue
            ps = self.pose_stamped(pose)
            self.pose_pubs[s].publish(ps)
            path = self.paths[s]
            last = path.poses[-1].pose.position if path.poses else None
            if last is None or math.hypot(last.x - pose.x, last.y - pose.y) > 0.01:
                path.poses.append(ps)
                if len(path.poses) > self.bridge.p["path_max_len"]:
                    del path.poses[0]
            path.header = ps.header
            self.path_pubs[s].publish(path)


class TwinBridge:
    def __init__(self, real_node, twin_node):
        self.lock = threading.RLock()
        self.main = real_node
        self.p = self._declare_parameters(real_node)
        self._declare_mirror_parameters(twin_node)
        self.leader = self.p["leader"]
        self.T_real_to_twin = None
        self.rtt_ms = NAN
        self.rtt_ema_ms = NAN
        self.real_odom_age_ms = NAN
        self.last_active = -1e9
        self.tick_count = 0
        self.sent_to_real = deque(maxlen=200)
        self.active_anomalies = {}
        self.comm_conditions = {"latencia_alta": Hysteresis(1.0, 1.0), "perdida_datos": Hysteresis(0.5, 0.5)}
        self.last_scan_cmp = 0.0
        self.net_in = NetworkEmulator(seed=self.p["net_seed"])
        self.net_out = NetworkEmulator(seed=self.p["net_seed"] + 1)
        self._configure_network(self.p["net_delay_ms"], self.p["net_jitter_ms"], self.p["net_loss"])
        self.twin_model = None
        self.twin_filters = None
        self._load_twin_model(self.p["twin_model_file"])

        self.sides = {}
        self.sides["real"] = Side("real", real_node, self)
        self.sides["twin"] = Side("twin", twin_node, self)

        self.ping_seq = 0
        self.ping_sent = {}
        if self.p["measure_rtt"]:
            self.ping_pub = real_node.create_publisher(Header, "/digital_twin/ping", 10)
            real_node.create_subscription(Header, "/digital_twin/pong", self.incoming("real", self._pong_cb), 10)
            real_node.create_timer(1.0 / self.p["ping_rate"], self._ping)
        real_node.create_subscription(TwinAnomaly, "/digital_twin/anomalies",
                                      self.incoming("real", self._detector_event_cb), 10)

        self.csv_file = self.csv_writer = None
        self.scan_file = self.scan_writer = None
        self.event_file = self.event_writer = None
        self.csv_path = ""
        if self.p["log_enabled"]:
            self._open_log()

        for node in (real_node, twin_node):
            node.add_on_set_parameters_callback(self._on_params)
        real_node.create_timer(1.0 / self.p["rate"], self.tick)
        real_node.create_timer(0.005, self._pump_network)
        real_node.get_logger().info(
            f"Puente listo. Líder: {self.leader}. Realimentación: {self.p['feedback']}. "
            f"Compensación: {self.p['compensation']}. Modelo del gemelo: {self.p['twin_model']}. "
            f"Dominios real={self.p['real_domain']} twin={self.p['twin_domain']}")

    # ------------------------------------------------------------------ setup
    def _declare_parameters(self, node):
        defaults = dict(DEFAULTS, real_domain=0, twin_domain=0)
        p = {k: node.declare_parameter(k, v).value for k, v in defaults.items()}
        if p["leader"] not in SIDES:
            raise ValueError("leader debe ser 'real' o 'twin'")
        return p

    def _declare_mirror_parameters(self, node):
        """El nodo del dominio del gemelo acepta los mismos parámetros ajustables."""
        for k in ("leader", "feedback", "compensation", "twin_model", "net_delay_ms",
                  "net_jitter_ms", "net_loss", *RUNTIME_FLOATS):
            node.declare_parameter(k, self.p[k])

    def _configure_network(self, delay_ms, jitter_ms, loss):
        self.p["net_delay_ms"], self.p["net_jitter_ms"], self.p["net_loss"] = delay_ms, jitter_ms, loss
        self.net_in.configure(delay_ms, jitter_ms, loss)
        self.net_out.configure(delay_ms, jitter_ms, loss)

    def _load_twin_model(self, path):
        path = os.path.expanduser(path or "")
        if not path:
            return
        try:
            self.twin_model = TwinModel.load(path)
            self.twin_filters = (FirstOrderFilter(self.twin_model.linear),
                                 FirstOrderFilter(self.twin_model.angular))
            self.main.get_logger().info(
                f"Modelo del gemelo: lineal K={self.twin_model.linear.gain:.3f} "
                f"τ={self.twin_model.linear.tau:.3f}s L={self.twin_model.linear.delay:.3f}s · "
                f"angular K={self.twin_model.angular.gain:.3f} τ={self.twin_model.angular.tau:.3f}s "
                f"L={self.twin_model.angular.delay:.3f}s")
        except Exception as exc:
            self.main.get_logger().error(f"No se pudo cargar twin_model_file {path}: {exc}")
            self.twin_model = None
            self.twin_filters = None

    def gains_for(self, follower):
        g = TrackingGains(self.p["kx"], self.p["ky"], self.p["ktheta"],
                          self.p["max_linear"], self.p["max_angular"])
        if follower == "real":
            g.max_linear = min(g.max_linear, self.p["real_max_linear"])
            g.max_angular = min(g.max_angular, self.p["real_max_angular"])
        return g

    # ------------------------------------------------------------------ network
    def incoming(self, side, handler):
        """Envuelve un callback para que los mensajes del robot real pasen por la red emulada."""
        def deliver(msg):
            if side == "real":
                with self.lock:
                    if self.net_in.enabled or self.net_in.queue:
                        self.net_in.send((handler, msg), time.monotonic())
                        return
            handler(msg)
        return deliver

    def send_to(self, side, publisher, msg):
        if side == "real" and (self.net_out.enabled or self.net_out.queue):
            self.net_out.send((publisher, msg), time.monotonic())
        else:
            publisher.publish(msg)

    def _pump_network(self):
        now = time.monotonic()
        with self.lock:
            inbound = self.net_in.receive(now)
            outbound = self.net_out.receive(now)
        for handler, msg in inbound:
            handler(msg)
        for publisher, msg in outbound:
            publisher.publish(msg)

    def one_way_delay(self):
        base = self.rtt_ema_ms / 2.0 if math.isfinite(self.rtt_ema_ms) else 0.0
        return max(0.0, base + self.p["compensation_extra_ms"]) / 1000.0

    # ------------------------------------------------------------------ logging
    def _open_log(self):
        log_dir = os.path.expanduser(self.p["log_dir"])
        os.makedirs(log_dir, exist_ok=True)
        tag = f"_{self.p['log_tag']}" if self.p["log_tag"] else ""
        base = os.path.join(log_dir, f"twin_{datetime.now().strftime('%Y%m%d_%H%M%S')}{tag}")
        self.csv_path = base + ".csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_COLUMNS)
        self.scan_file = open(base + "_scan.csv", "w", newline="")
        self.scan_writer = csv.writer(self.scan_file)
        self.scan_writer.writerow(SCAN_COLUMNS)
        self.event_file = open(base + "_eventos.csv", "w", newline="")
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(EVENT_COLUMNS)
        with open(base + ".json", "w") as f:
            meta = {"started": datetime.now().isoformat(), "parameters": self.p,
                    "twin_model": self.twin_model.to_dict() if self.twin_model else None}
            json.dump(meta, f, indent=2, ensure_ascii=False)
        self.main.get_logger().info(f"Registrando en {self.csv_path}")

    def _close_log(self):
        for f in (self.csv_file, self.scan_file, self.event_file):
            if f:
                f.close()
        self.csv_file = self.scan_file = self.event_file = None
        self.csv_writer = self.scan_writer = self.event_writer = None

    def _new_log(self, tag=None):
        if tag is not None:
            self.p["log_tag"] = tag
        self._close_log()
        self._open_log()
        for side in self.sides.values():
            for path in side.paths.values():
                path.poses.clear()

    # ------------------------------------------------------------------ latency
    def _ping(self):
        self.ping_seq += 1
        msg = Header()
        msg.frame_id = str(self.ping_seq)
        msg.stamp = wall_time_msg()
        with self.lock:
            self.ping_sent[self.ping_seq] = time.monotonic()
            for old in [s for s in self.ping_sent if s < self.ping_seq - 100]:
                del self.ping_sent[old]
            self.send_to("real", self.ping_pub, msg)

    def _pong_cb(self, msg):
        try:
            seq = int(msg.frame_id)
        except ValueError:
            return
        with self.lock:
            sent = self.ping_sent.pop(seq, None)
            if sent is not None:
                self.rtt_ms = (time.monotonic() - sent) * 1000.0
                self.rtt_ema_ms = (self.rtt_ms if not math.isfinite(self.rtt_ema_ms)
                                   else 0.8 * self.rtt_ema_ms + 0.2 * self.rtt_ms)

    # ------------------------------------------------------------------ events
    def emit_event(self, source, kind, phase, severity, value, description):
        msg = TwinAnomaly()
        msg.stamp = wall_time_msg()
        msg.source, msg.kind, msg.phase = source, kind, phase
        msg.severity, msg.value, msg.description = int(severity), float(value), description
        if phase == "inicio":
            self.active_anomalies[kind] = source
        else:
            self.active_anomalies.pop(kind, None)
        for side in self.sides.values():
            side.event_pub.publish(msg)
        if self.event_writer:
            self.event_writer.writerow([f"{time.time():.4f}", source, kind, phase, severity,
                                        f"{value:.4f}", description])
            self.event_file.flush()
        text = f"Anomalía [{source}] {kind} {phase}: {description} ({value:.3f})"
        if phase == "inicio":
            self.main.get_logger().warn(text)
        else:
            self.main.get_logger().info(text)

    def _detector_event_cb(self, msg):
        with self.lock:
            self.emit_event(msg.source or "detector", msg.kind, msg.phase, msg.severity, msg.value, msg.description)

    def _check_comm_anomalies(self, now, stale):
        t = time.monotonic()
        high = math.isfinite(self.rtt_ema_ms) and self.rtt_ema_ms > self.p["latency_alert_ms"]
        for kind, cond in (("latencia_alta", high), ("perdida_datos", stale and self.T_real_to_twin is not None)):
            phase = self.comm_conditions[kind].update(t, cond)
            if phase:
                value = self.rtt_ema_ms if kind == "latencia_alta" else 0.0
                self.emit_event("puente", kind, phase, SEVERITY[kind], value if math.isfinite(value) else 0.0,
                                DESCRIPTIONS[kind])

    # ------------------------------------------------------------------ scans
    def relay_real_scan(self, msg):
        twin = self.sides.get("twin")
        if twin is None:
            return
        out = copy.copy(msg)
        out.header = Header()
        out.header.frame_id = self.p["real_scan_frame"]
        out.header.stamp = twin.node.get_clock().now().to_msg()
        twin.real_scan_pub.publish(out)

    def _compare_scans(self, now, pos_err):
        if not self.p["scan_compare"] or now - self.last_scan_cmp < 1.0 / self.p["scan_compare_rate"]:
            return
        r, t = self.sides["real"].state, self.sides["twin"].state
        if r.scan is None or t.scan is None or now - r.scan_rx > 0.5 or now - t.scan_rx > 0.5:
            return
        if not math.isfinite(pos_err) or pos_err > self.p["scan_sync_tolerance"]:
            return
        if any(abs(s.v) > self.p["scan_max_linear"] or abs(s.w) > self.p["scan_max_angular"] for s in (r, t)):
            return
        self.last_scan_cmp = now
        res = compare_scans(r.scan, t.scan)
        msg = ScanComparison()
        msg.stamp = wall_time_msg()
        msg.common_beams = res["common"]
        for k in ("mae", "rmse", "bias", "p95", "coverage_real", "coverage_twin", "visibility_agreement"):
            setattr(msg, k, float(res[k]))
        for side in self.sides.values():
            side.scan_cmp_pub.publish(msg)
        if self.scan_writer:
            self.scan_writer.writerow([f"{time.time():.4f}", res["common"]] +
                                      [f"{res[k]:.5f}" for k in SCAN_COLUMNS[2:9]] + [f"{pos_err:.5f}"])

    # ------------------------------------------------------------------ configuration
    def align(self):
        real = self.sides["real"].state.pose
        twin = self.sides["twin"].state.pose
        if real is None or twin is None:
            return False
        self.T_real_to_twin = alignment_transform(twin, real)
        for side in self.sides.values():
            for path in side.paths.values():
                path.poses.clear()
        if self.twin_filters:
            for f in self.twin_filters:
                f.reset()
        return True

    def set_leader(self, leader):
        with self.lock:
            if leader != self.leader:
                self.leader = leader
                self.last_active = -1e9
                self.sent_to_real.clear()
        self.main.get_logger().info(f"Nuevo líder: {leader}")

    def srv_align(self, request, response):
        with self.lock:
            response.success = self.align()
        response.message = "Poses alineadas" if response.success else "Falta odometría de algún robot"
        return response

    def srv_set_feedback(self, request, response):
        with self.lock:
            self.p["feedback"] = bool(request.data)
        response.success = True
        response.message = f"Realimentación {'activada' if self.p['feedback'] else 'desactivada'}"
        return response

    def srv_switch_leader(self, request, response):
        self.set_leader(other(self.leader))
        response.success = True
        response.message = f"Líder: {self.leader}"
        return response

    def srv_new_log(self, request, response):
        with self.lock:
            self._new_log()
            response.success = True
            response.message = self.csv_path
        return response

    def srv_configure(self, request, response):
        notes = []
        if request.leader:
            if request.leader not in SIDES:
                response.success = False
                response.message = "leader debe ser real o twin"
                return response
            self.set_leader(request.leader)
            notes.append(f"líder={request.leader}")
        with self.lock:
            for field, key in (("feedback", "feedback"), ("compensation", "compensation"),
                               ("twin_model", "twin_model")):
                value = getattr(request, field)
                if value >= 0:
                    self.p[key] = bool(value)
                    notes.append(f"{key}={self.p[key]}")
            if request.twin_model_file:
                self._load_twin_model(request.twin_model_file)
                if self.twin_model is not None:
                    self.p["twin_model_file"] = request.twin_model_file
                    notes.append(f"modelo={os.path.basename(request.twin_model_file)}")
                else:
                    notes.append("ERROR: no se pudo cargar el modelo")
            if self.p["twin_model"] and self.twin_model is None:
                notes.append("ADVERTENCIA: no hay twin_model_file cargado")
            net = [request.net_delay_ms, request.net_jitter_ms, request.net_loss]
            if any(v >= 0 for v in net):
                current = [self.p["net_delay_ms"], self.p["net_jitter_ms"], self.p["net_loss"]]
                self._configure_network(*[n if n >= 0 else c for n, c in zip(net, current)])
                notes.append(f"red={self.p['net_delay_ms']:.0f}ms±{self.p['net_jitter_ms']:.0f} "
                             f"pérdida={self.p['net_loss']:.2f}")
                self.rtt_ema_ms = NAN
            if request.align:
                ok = self.align()
                notes.append("alineado" if ok else "sin alinear (falta odometría)")
            if request.new_log and self.p["log_enabled"]:
                self._new_log(request.log_tag or self.p["log_tag"])
                notes.append(f"registro={os.path.basename(self.csv_path)}")
            response.log_path = self.csv_path
        response.success = True
        response.message = ", ".join(notes) or "sin cambios"
        self.main.get_logger().info(f"Configuración: {response.message}")
        return response

    def _on_params(self, params):
        for prm in params:
            if prm.name == "leader":
                if prm.value not in SIDES:
                    return SetParametersResult(successful=False, reason="leader: real | twin")
                self.set_leader(prm.value)
            elif prm.name in ("feedback", "compensation", "twin_model"):
                with self.lock:
                    self.p[prm.name] = is_true(prm.value)
            elif prm.name in ("net_delay_ms", "net_jitter_ms", "net_loss"):
                with self.lock:
                    values = {k: self.p[k] for k in ("net_delay_ms", "net_jitter_ms", "net_loss")}
                    values[prm.name] = float(prm.value)
                    self._configure_network(values["net_delay_ms"], values["net_jitter_ms"], values["net_loss"])
            elif prm.name in RUNTIME_FLOATS:
                with self.lock:
                    self.p[prm.name] = float(prm.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ main loop
    def pose_in_frame(self, side_frame, source_side, pose=None):
        pose = pose if pose is not None else self.sides[source_side].state.pose
        if pose is None:
            return None
        if side_frame == source_side:
            return pose
        if self.T_real_to_twin is None:
            return None
        if side_frame == "twin":
            return self.T_real_to_twin.compose(pose)
        return self.T_real_to_twin.inverse().compose(pose)

    def tick(self):
        with self.lock:
            self._tick_locked()

    def _shape_for_twin(self, now, cmd):
        if not (self.p["twin_model"] and self.twin_filters):
            return cmd
        fv, fw = self.twin_filters
        return fv.update(now, cmd[0]), fw.update(now, cmd[1])

    def _tick_locked(self):
        now = time.monotonic()
        self.tick_count += 1
        leader, follower = self.leader, other(self.leader)
        L = self.sides[leader].state
        F = self.sides[follower].state

        def fresh(rx, timeout):
            return rx is not None and now - rx < timeout

        if self.T_real_to_twin is None and self.p["auto_align"]:
            if self.align():
                self.main.get_logger().info("Alineación inicial hecha")

        stale = not (fresh(L.odom_rx, self.p["state_timeout"]) and fresh(F.odom_rx, self.p["state_timeout"]))
        tau = self.one_way_delay()
        cmd_age = NAN
        if leader == "real" and L.cmd_stamp is not None:
            cmd_age = (time.time() - L.cmd_stamp) * 1000.0

        ref = F_pose = None
        err = err_raw = (NAN, NAN, NAN)
        ff = (0.0, 0.0)
        cmd = (0.0, 0.0)
        active = lost = False
        if not stale and self.T_real_to_twin is not None:
            ref_raw = self.pose_in_frame(follower, leader)
            F_raw = F.pose
            err_raw = tracking_error(ref_raw, F_raw)
            ref, F_pose = ref_raw, F_raw
            if self.p["compensation"]:
                cmd_delay = tau if follower == "real" else 0.0
                ref = integrate_unicycle(ref_raw, L.v, L.w, cmd_delay + (tau if leader == "real" else 0.0))
                if follower == "real":
                    F_pose = smith_predict(F_raw, self.sent_to_real, now, tau, tau)
            err = tracking_error(ref, F_pose)
            pos_err = math.hypot(err[0], err[1])
            if self.p["feedforward_source"] == "odom":
                ff = (L.v, L.w)
            elif fresh(L.cmd_rx, self.p["cmd_timeout"]):
                ff = (L.cmd_v, L.cmd_w)
            lost = pos_err > self.p["max_correction_error"]
            use_fb = self.p["feedback"] and not lost
            cmd = tracking_command(ff[0], ff[1], err, self.gains_for(follower), use_fb)
            leader_moving = abs(ff[0]) > 1e-3 or abs(ff[1]) > 1e-3
            needs_correction = use_fb and (abs(err[0]) > self.p["position_tolerance"]
                                           or abs(err[2]) > self.p["heading_tolerance"])
            active = leader_moving or needs_correction
            if active:
                self.last_active = now
            elif now - self.last_active < 1.0:
                cmd = (0.0, 0.0)
            else:
                cmd = None
            if cmd is not None:
                if follower == "twin":
                    cmd = self._shape_for_twin(now, cmd)
                t = Twist()
                t.linear.x = float(clamp(cmd[0], 10.0))
                t.angular.z = float(clamp(cmd[1], 10.0))
                self.send_to(follower, self.sides[follower].follow_pub, t)
                if follower == "real":
                    self.sent_to_real.append((now, t.linear.x, t.angular.z))
            else:
                cmd = (0.0, 0.0)
                if self.twin_filters:
                    for f in self.twin_filters:
                        f.reset()
        elif stale and now - self.last_active < 1.0:
            self.send_to(follower, self.sides[follower].follow_pub, Twist())

        self._check_comm_anomalies(now, stale)

        pos_error = math.hypot(err[0], err[1]) if ref is not None else NAN
        pos_error_raw = math.hypot(err_raw[0], err_raw[1]) if ref is not None else NAN
        status = TwinSyncStatus()
        status.stamp = wall_time_msg()
        status.leader = leader
        status.stale = stale
        status.active = active
        status.lost_sync = lost
        status.position_error = pos_error
        status.position_error_raw = pos_error_raw
        status.heading_error = err[2]
        status.error_longitudinal = err[0]
        status.error_lateral = err[1]
        if ref is not None:
            status.leader_pose = Pose2DMsg(x=ref.x, y=ref.y, theta=ref.theta)
            status.follower_pose = Pose2DMsg(x=F_pose.x, y=F_pose.y, theta=F_pose.theta)
        status.feedforward.linear.x, status.feedforward.angular.z = float(ff[0]), float(ff[1])
        status.command.linear.x, status.command.angular.z = float(cmd[0]), float(cmd[1])
        status.rtt_ms = self.rtt_ms
        status.real_odom_age_ms = self.real_odom_age_ms
        status.leader_cmd_age_ms = cmd_age
        status.feedback = bool(self.p["feedback"])
        status.compensation = bool(self.p["compensation"])
        status.twin_model = bool(self.p["twin_model"] and self.twin_filters is not None)
        status.net_delay_ms = float(self.p["net_delay_ms"])
        status.net_loss = float(self.p["net_loss"])
        status.one_way_delay_ms = tau * 1000.0
        status.anomalies = "|".join(sorted(self.active_anomalies))
        for side in self.sides.values():
            side.status_pub.publish(status)

        self._compare_scans(now, pos_error)

        if self.T_real_to_twin is not None and self.tick_count % 2 == 0:
            for name, side in self.sides.items():
                side.publish_poses({s: self.pose_in_frame(name, s) for s in SIDES})

        if self.csv_writer is not None:
            self._log_row(status, ref, F_pose, err, ff, cmd, active, lost, stale, cmd_age, tau)

    def _log_row(self, status, ref, F_pose, err, ff, cmd, active, lost, stale, cmd_age, tau):
        def pose_cols(pose):
            return [pose.x, pose.y, pose.theta] if pose is not None else [NAN, NAN, NAN]

        real = self.sides["real"].state
        twin = self.sides["twin"].state
        row = [
            f"{time.time():.4f}", self.leader, int(self.p["feedback"]), int(self.p["compensation"]),
            int(status.twin_model), self.p["net_delay_ms"], self.p["net_loss"],
            int(active), int(lost), int(stale),
            status.position_error, status.position_error_raw, err[2], err[0], err[1],
            *pose_cols(ref), *pose_cols(F_pose),
            *pose_cols(self.pose_in_frame("twin", "real")), real.v, real.w,
            real.odom_stamp if real.odom_stamp is not None else NAN,
            *pose_cols(twin.pose), twin.v, twin.w,
            ff[0], ff[1], cmd[0], cmd[1],
            self.rtt_ms, tau * 1000.0, self.real_odom_age_ms, cmd_age,
            status.anomalies,
        ]
        self.csv_writer.writerow([f"{x:.5f}" if isinstance(x, float) else x for x in row])
        if self.tick_count % 20 == 0:
            self.csv_file.flush()
            self.scan_file.flush()


def main():
    argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(description="Puente del gemelo digital")
    parser.add_argument("--real-domain", type=int,
                        default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int,
                        default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    parser.add_argument("--twin-sim-time", default="true")
    args = parser.parse_args(argv[1:])
    if args.real_domain == args.twin_domain:
        print("real_domain y twin_domain deben ser distintos", file=sys.stderr)
        return 1

    ctx_real, ctx_twin = Context(), Context()
    rclpy.init(args=sys.argv, context=ctx_real, domain_id=args.real_domain)
    rclpy.init(args=None, context=ctx_twin, domain_id=args.twin_domain)
    real_node = Node("twin_bridge", context=ctx_real, parameter_overrides=[
        Parameter("real_domain", value=args.real_domain),
        Parameter("twin_domain", value=args.twin_domain),
    ])
    twin_node = Node("twin_bridge", context=ctx_twin, parameter_overrides=[
        Parameter("use_sim_time", value=is_true(args.twin_sim_time)),
    ])
    bridge = TwinBridge(real_node, twin_node)

    executors = []
    for node, ctx in ((real_node, ctx_real), (twin_node, ctx_twin)):
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        threading.Thread(target=spin_resilient, args=(ex, ctx, node.get_logger()), daemon=True).start()
        executors.append(ex)
    try:
        while rclpy.ok(context=ctx_real) and rclpy.ok(context=ctx_twin):
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        with bridge.lock:
            bridge._close_log()
        for ex in executors:
            ex.shutdown(timeout_sec=1.0)
        for ctx in (ctx_real, ctx_twin):
            if ctx.ok():
                rclpy.shutdown(context=ctx)
    return 0


if __name__ == "__main__":
    sys.exit(main())
