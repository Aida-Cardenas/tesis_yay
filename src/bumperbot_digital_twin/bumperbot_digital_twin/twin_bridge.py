#!/usr/bin/env python3
"""Capa de comunicación bidireccional entre el robot físico y su gemelo digital.

El robot físico y el gemelo corren el mismo software (bumperbot_bringup) en dos
dominios DDS distintos (ROS_DOMAIN_ID), así sus tópicos no se mezclan. Este nodo
se une a los dos dominios a la vez y:

1. Toma el comando de velocidad del líder y lo envía al seguidor (prealimentación).
2. Compara las odometrías y corrige al seguidor con una ley de seguimiento
   (Kanayama et al., 1990) para que no se aleje por deriva o diferencias del modelo.
3. Publica en ambos dominios el estado de sincronización, las poses y trayectorias
   de los dos robots (para verlas juntas en RViz) y mide la latencia de la red.
4. Registra todo en un CSV por experimento para el análisis de la tesis.

El líder puede ser "real" o "twin" y se puede cambiar en caliente, por eso la
comunicación es bidireccional: los comandos fluyen en el sentido que se elija y
el estado de ambos robots fluye siempre en los dos sentidos.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from rcl_interfaces.msg import SetParametersResult

from builtin_interfaces.msg import Time

from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger

from bumperbot_msgs.msg import TwinSyncStatus
from bumperbot_digital_twin.geometry import (
    Pose2D,
    TrackingGains,
    alignment_transform,
    clamp,
    tracking_command,
    tracking_error,
    yaw_from_quaternion,
)

SIDES = ("real", "twin")
NAN = float("nan")

CSV_COLUMNS = [
    "t", "leader", "feedback", "active", "lost_sync", "stale",
    "pos_error", "yaw_error", "ex", "ey",
    "leader_x", "leader_y", "leader_yaw",
    "follower_x", "follower_y", "follower_yaw",
    "real_x", "real_y", "real_yaw", "real_v", "real_w",
    "twin_x", "twin_y", "twin_yaw", "twin_v", "twin_w",
    "ff_v", "ff_w", "cmd_v", "cmd_w",
    "rtt_ms", "real_odom_age_ms", "leader_cmd_age_ms",
]


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


class Side:
    """Todo lo que el puente hace dentro de un dominio (real o twin)."""

    def __init__(self, name, node, bridge):
        self.name = name
        self.node = node
        self.bridge = bridge
        self.state = RobotState()
        p = bridge.p
        node.create_subscription(Odometry, p["odom_topic"], self._odom_cb, 10)
        node.create_subscription(TwistStamped, p["cmd_topic"], self._cmd_cb, 10)
        self.follow_pub = node.create_publisher(Twist, p["follow_topic"], 10)
        self.status_pub = node.create_publisher(TwinSyncStatus, "/digital_twin/status", 10)
        self.path_pubs = {s: node.create_publisher(Path, f"/digital_twin/{s}_path", 10) for s in SIDES}
        self.pose_pubs = {s: node.create_publisher(PoseStamped, f"/digital_twin/{s}_pose", 10) for s in SIDES}
        self.paths = {s: Path() for s in SIDES}
        node.create_service(Trigger, "/digital_twin/align", bridge.srv_align)
        node.create_service(SetBool, "/digital_twin/set_feedback", bridge.srv_set_feedback)
        node.create_service(Trigger, "/digital_twin/switch_leader", bridge.srv_switch_leader)
        node.create_service(Trigger, "/digital_twin/new_log", bridge.srv_new_log)

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

    def pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = self.bridge.p["odom_frame"]
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation.z = math.sin(pose.theta / 2.0)
        msg.pose.orientation.w = math.cos(pose.theta / 2.0)
        return msg

    def publish_poses(self, poses_in_this_frame, append_path):
        for s, pose in poses_in_this_frame.items():
            if pose is None:
                continue
            ps = self.pose_stamped(pose)
            self.pose_pubs[s].publish(ps)
            path = self.paths[s]
            if append_path:
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
        self.leader = self.p["leader"]
        self.feedback = self.p["feedback"]
        self.T_real_to_twin = None
        self.rtt_ms = NAN
        self.real_odom_age_ms = NAN
        self.last_active = -1e9
        self.tick_count = 0
        self.sides = {"real": Side("real", real_node, self), "twin": Side("twin", twin_node, self)}

        self.ping_seq = 0
        self.ping_sent = {}
        if self.p["measure_rtt"]:
            self.ping_pub = real_node.create_publisher(Header, "/digital_twin/ping", 10)
            real_node.create_subscription(Header, "/digital_twin/pong", self._pong_cb, 10)
            real_node.create_timer(1.0 / self.p["ping_rate"], self._ping)

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = ""
        if self.p["log_enabled"]:
            self._open_log()

        real_node.add_on_set_parameters_callback(self._on_params)
        real_node.create_timer(1.0 / self.p["rate"], self.tick)
        real_node.get_logger().info(
            f"Puente listo. Líder: {self.leader}. Realimentación: {self.feedback}. "
            f"Dominios real={self.p['real_domain']} twin={self.p['twin_domain']}")

    # ------------------------------------------------------------------ setup
    def _declare_parameters(self, node):
        defaults = {
            "leader": "real",
            "odom_topic": "/bumperbot_controller/odom",
            "cmd_topic": "/bumperbot_controller/cmd_vel",
            "follow_topic": "twin_vel",
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
            "log_enabled": True,
            "log_dir": "~/twin_logs",
            "log_tag": "",
            "path_max_len": 5000,
            "real_domain": 0,
            "twin_domain": 0,
        }
        p = {}
        for k, v in defaults.items():
            p[k] = node.declare_parameter(k, v).value
        if p["leader"] not in SIDES:
            raise ValueError("leader debe ser 'real' o 'twin'")
        return p

    def gains_for(self, follower):
        g = TrackingGains(self.p["kx"], self.p["ky"], self.p["ktheta"],
                          self.p["max_linear"], self.p["max_angular"])
        if follower == "real":
            g.max_linear = min(g.max_linear, self.p["real_max_linear"])
            g.max_angular = min(g.max_angular, self.p["real_max_angular"])
        return g

    # ------------------------------------------------------------------ logging
    def _open_log(self):
        log_dir = os.path.expanduser(self.p["log_dir"])
        os.makedirs(log_dir, exist_ok=True)
        tag = f"_{self.p['log_tag']}" if self.p["log_tag"] else ""
        base = f"twin_{datetime.now().strftime('%Y%m%d_%H%M%S')}{tag}"
        self.csv_path = os.path.join(log_dir, base + ".csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_COLUMNS)
        with open(os.path.join(log_dir, base + ".json"), "w") as f:
            json.dump({"started": datetime.now().isoformat(), "parameters": self.p}, f, indent=2)
        self.main.get_logger().info(f"Registrando en {self.csv_path}")

    def _close_log(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    # ------------------------------------------------------------------ latency
    def _ping(self):
        self.ping_seq += 1
        msg = Header()
        msg.frame_id = str(self.ping_seq)
        msg.stamp = wall_time_msg()
        with self.lock:
            self.ping_sent[self.ping_seq] = time.monotonic()
            for old in [s for s in self.ping_sent if s < self.ping_seq - 50]:
                del self.ping_sent[old]
        self.ping_pub.publish(msg)

    def _pong_cb(self, msg):
        try:
            seq = int(msg.frame_id)
        except ValueError:
            return
        with self.lock:
            sent = self.ping_sent.pop(seq, None)
            if sent is not None:
                self.rtt_ms = (time.monotonic() - sent) * 1000.0

    # ------------------------------------------------------------------ services
    def align(self):
        real = self.sides["real"].state.pose
        twin = self.sides["twin"].state.pose
        if real is None or twin is None:
            return False
        self.T_real_to_twin = alignment_transform(twin, real)
        for side in self.sides.values():
            for path in side.paths.values():
                path.poses.clear()
        return True

    def srv_align(self, request, response):
        with self.lock:
            response.success = self.align()
        response.message = "Poses alineadas" if response.success else "Falta odometría de algún robot"
        return response

    def srv_set_feedback(self, request, response):
        with self.lock:
            self.feedback = bool(request.data)
        response.success = True
        response.message = f"Realimentación {'activada' if self.feedback else 'desactivada'}"
        return response

    def set_leader(self, leader):
        with self.lock:
            if leader != self.leader:
                self.leader = leader
                self.last_active = -1e9
        self.main.get_logger().info(f"Nuevo líder: {leader}")

    def srv_switch_leader(self, request, response):
        self.set_leader(other(self.leader))
        response.success = True
        response.message = f"Líder: {self.leader}"
        return response

    def srv_new_log(self, request, response):
        with self.lock:
            self._close_log()
            self._open_log()
            response.success = True
            response.message = self.csv_path
        return response

    def _on_params(self, params):
        for prm in params:
            if prm.name == "leader":
                if prm.value not in SIDES:
                    return SetParametersResult(successful=False, reason="leader: real | twin")
                self.set_leader(prm.value)
            elif prm.name == "feedback":
                with self.lock:
                    self.feedback = bool(prm.value)
            elif prm.name in ("kx", "ky", "ktheta", "max_linear", "max_angular",
                              "position_tolerance", "heading_tolerance", "max_correction_error"):
                with self.lock:
                    self.p[prm.name] = float(prm.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ main loop
    def pose_in_frame(self, side_frame, source_side):
        pose = self.sides[source_side].state.pose
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
        status = TwinSyncStatus()
        status.stamp = wall_time_msg()
        status.leader = leader
        status.stale = stale
        status.rtt_ms = self.rtt_ms
        status.real_odom_age_ms = self.real_odom_age_ms
        cmd_age = NAN
        if leader == "real" and L.cmd_stamp is not None:
            cmd_age = (time.time() - L.cmd_stamp) * 1000.0
        status.leader_cmd_age_ms = cmd_age

        ref = F_pose = None
        err = (NAN, NAN, NAN)
        ff = (0.0, 0.0)
        cmd = (0.0, 0.0)
        active = False
        lost = False
        if not stale and self.T_real_to_twin is not None:
            ref = self.pose_in_frame(follower, leader)
            F_pose = F.pose
            err = tracking_error(ref, F_pose)
            pos_err = math.hypot(err[0], err[1])
            if self.p["feedforward_source"] == "odom":
                ff = (L.v, L.w)
            elif fresh(L.cmd_rx, self.p["cmd_timeout"]):
                ff = (L.cmd_v, L.cmd_w)
            gains = self.gains_for(follower)
            lost = pos_err > self.p["max_correction_error"]
            use_fb = self.feedback and not lost
            cmd = tracking_command(ff[0], ff[1], err, gains, use_fb)
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
                t = Twist()
                t.linear.x = float(clamp(cmd[0], 10.0))
                t.angular.z = float(clamp(cmd[1], 10.0))
                self.sides[follower].follow_pub.publish(t)
            else:
                cmd = (0.0, 0.0)
        elif stale and now - self.last_active < 1.0:
            self.sides[follower].follow_pub.publish(Twist())

        status.active = active
        status.lost_sync = lost
        status.position_error = math.hypot(err[0], err[1]) if ref is not None else NAN
        status.heading_error = err[2]
        status.error_longitudinal = err[0]
        status.error_lateral = err[1]
        if ref is not None:
            status.leader_pose = Pose2DMsg(x=ref.x, y=ref.y, theta=ref.theta)
            status.follower_pose = Pose2DMsg(x=F_pose.x, y=F_pose.y, theta=F_pose.theta)
        status.feedforward.linear.x, status.feedforward.angular.z = float(ff[0]), float(ff[1])
        status.command.linear.x, status.command.angular.z = float(cmd[0]), float(cmd[1])
        for side in self.sides.values():
            side.status_pub.publish(status)

        if self.T_real_to_twin is not None and self.tick_count % 2 == 0:
            for name, side in self.sides.items():
                poses = {s: self.pose_in_frame(name, s) for s in SIDES}
                side.publish_poses(poses, append_path=True)

        if self.csv_writer is not None:
            self._log_row(status, ref, F_pose, err, ff, cmd, active, lost, stale, cmd_age)

    def _log_row(self, status, ref, F_pose, err, ff, cmd, active, lost, stale, cmd_age):
        def pose_cols(pose):
            return [pose.x, pose.y, pose.theta] if pose is not None else [NAN, NAN, NAN]

        real = self.sides["real"].state
        twin = self.sides["twin"].state
        real_in_follower = self.pose_in_frame(other(self.leader), "real")
        twin_in_follower = self.pose_in_frame(other(self.leader), "twin")
        row = [
            f"{time.time():.4f}", self.leader, int(self.feedback), int(active), int(lost), int(stale),
            status.position_error, err[2], err[0], err[1],
            *pose_cols(ref), *pose_cols(F_pose),
            *pose_cols(real_in_follower), real.v, real.w,
            *pose_cols(twin_in_follower), twin.v, twin.w,
            ff[0], ff[1], cmd[0], cmd[1],
            self.rtt_ms, self.real_odom_age_ms, cmd_age,
        ]
        self.csv_writer.writerow([f"{x:.5f}" if isinstance(x, float) else x for x in row])
        if self.tick_count % 20 == 0:
            self.csv_file.flush()


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
    twin_sim_time = args.twin_sim_time.lower() in ("1", "true", "yes")
    real_node = Node("twin_bridge", context=ctx_real, parameter_overrides=[
        Parameter("real_domain", value=args.real_domain),
        Parameter("twin_domain", value=args.twin_domain),
    ])
    twin_node = Node("twin_bridge", context=ctx_twin, parameter_overrides=[
        Parameter("use_sim_time", value=twin_sim_time),
    ])
    bridge = TwinBridge(real_node, twin_node)

    executors = []
    for node, ctx in ((real_node, ctx_real), (twin_node, ctx_twin)):
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
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
