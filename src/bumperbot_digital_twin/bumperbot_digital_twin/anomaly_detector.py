#!/usr/bin/env python3
"""Detector de anomalías del robot real, usando el modelo del gemelo como referencia.

Corre en el dominio del robot real (en la Raspberry Pi o en el PC). Compara lo que
el robot debería hacer según el modelo identificado (twin_calibrate dynamics) con
lo que miden los encoders y la IMU, y publica los eventos en
/digital_twin/anomalies. Opcionalmente bloquea el robot (lock "safety_stop" de
twist_mux) mientras dure un atasco.
"""
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from bumperbot_msgs.msg import TwinAnomaly
from bumperbot_digital_twin.anomaly import AnomalyDetector, DetectorConfig
from bumperbot_digital_twin.dynamics import NOMINAL_MODEL, TwinModel


class AnomalyDetectorNode(Node):
    def __init__(self):
        super().__init__("anomaly_detector")
        model_file = os.path.expanduser(self.declare_parameter("model_file", "").value)
        self.rate = self.declare_parameter("rate", 50.0).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.3).value
        self.state_timeout = self.declare_parameter("state_timeout", 0.5).value
        self.use_imu = self.declare_parameter("use_imu", True).value
        self.lock_on_stall = self.declare_parameter("lock_on_stall", False).value
        log_dir = self.declare_parameter("log_dir", "").value
        cfg = DetectorConfig()
        for field in cfg.__dataclass_fields__:
            setattr(cfg, field, float(self.declare_parameter(field, getattr(cfg, field)).value))

        model = NOMINAL_MODEL
        if model_file:
            try:
                model = TwinModel.load(model_file)
                self.get_logger().info(f"Modelo cargado de {model_file}")
            except Exception as exc:
                self.get_logger().error(f"No se pudo leer {model_file}: {exc}. Uso el modelo nominal.")
        self.detector = AnomalyDetector(model, cfg)

        self.cmd = (0.0, 0.0)
        self.cmd_rx = None
        self.meas = (0.0, 0.0)
        self.odom_rx = None
        self.imu_w = None
        self.imu_rx = None
        self.create_subscription(TwistStamped, "/bumperbot_controller/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Odometry, "/bumperbot_controller/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/imu/out", self._imu_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(TwinAnomaly, "/digital_twin/anomalies", 10)
        self.lock_pub = self.create_publisher(Bool, "safety_stop", 10)
        self.writer = None
        if log_dir:
            path = os.path.join(os.path.expanduser(log_dir),
                                f"anomalias_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.log_file = open(path, "w", newline="")
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(["t", "kind", "phase", "severity", "value", "description"])
        self.create_timer(1.0 / self.rate, self._step)
        self.get_logger().info("Detector de anomalías listo")

    def _cmd_cb(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)
        self.cmd_rx = time.monotonic()

    def _odom_cb(self, msg):
        self.meas = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.odom_rx = time.monotonic()

    def _imu_cb(self, msg):
        self.imu_w = msg.angular_velocity.z
        self.imu_rx = time.monotonic()

    def _step(self):
        now = time.monotonic()
        if self.odom_rx is None or now - self.odom_rx > self.state_timeout:
            return
        cmd = self.cmd if self.cmd_rx is not None and now - self.cmd_rx < self.cmd_timeout else (0.0, 0.0)
        imu = None
        if self.use_imu and self.imu_rx is not None and now - self.imu_rx < self.state_timeout:
            imu = self.imu_w
        for ev in self.detector.update(now, cmd[0], cmd[1], self.meas[0], self.meas[1], imu):
            msg = TwinAnomaly()
            t = time.time()
            msg.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            msg.source = "detector"
            msg.kind, msg.phase, msg.severity = ev.kind, ev.phase, ev.severity
            msg.value, msg.description = float(ev.value), ev.description
            self.pub.publish(msg)
            log = self.get_logger().warn if ev.phase == "inicio" else self.get_logger().info
            log(f"{ev.kind} {ev.phase}: {ev.description} ({ev.value:.3f})")
            if self.writer:
                self.writer.writerow([f"{t:.4f}", ev.kind, ev.phase, ev.severity, f"{ev.value:.4f}", ev.description])
                self.log_file.flush()
            if self.lock_on_stall and ev.kind == "atasco":
                self.lock_pub.publish(Bool(data=ev.phase == "inicio"))


def main():
    rclpy.init()
    node = AnomalyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
