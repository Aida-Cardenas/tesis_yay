#!/usr/bin/env python3
"""Robot de mentira para probar el gemelo digital sin hardware ni Gazebo.

Imita la interfaz del bumperbot: recibe velocidades por las mismas entradas que
twist_mux (joy_vel, twin_vel, key_vel, cmd_vel, con sus prioridades), publica el
comando aplicado en /bumperbot_controller/cmd_vel, la odometría de ruedas en
/bumperbot_controller/odom y la IMU en /imu/out. Opcionalmente simula un LiDAR
2D dentro de un recinto rectangular (/scan).

Permite reproducir diferencias con el robot ideal (ganancia, constante de tiempo,
retardo, ruido) y fallos para probar el detector de anomalías:
- stall: el robot queda trabado (ni las ruedas ni el robot se mueven),
- slip: las ruedas giran pero el robot no rota (la IMU no lo registra),
- push: alguien empuja el robot sin que haya comando.
"""
import math
import random
import time
from collections import deque

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

from bumperbot_digital_twin.geometry import Pose2D, integrate_unicycle
from bumperbot_digital_twin.scan_compare import raycast_rectangle

MUX_INPUTS = (("joy_vel", 99), ("twin_vel", 95), ("key_vel", 90), ("cmd_vel", 80))
SCAN_BEAMS = 360


def quaternion_z(theta):
    return math.sin(theta / 2.0), math.cos(theta / 2.0)


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        dp = self.declare_parameter
        self.rate = dp("rate", 50.0).value
        self.linear_gain = dp("linear_gain", 1.0).value
        self.angular_gain = dp("angular_gain", 1.0).value
        self.tau = dp("tau", 0.1).value
        self.cmd_delay = dp("cmd_delay", 0.0).value
        self.noise_std = dp("noise_std", 0.0).value
        self.mux_timeout = dp("mux_timeout", 0.5).value
        self.publish_tf = dp("publish_tf", True).value
        self.fault = dp("fault", "none").value
        self.fault_start = dp("fault_start", 5.0).value
        self.fault_duration = dp("fault_duration", 3.0).value
        self.arena = [float(x) for x in dp("arena", [0.0], ParameterDescriptor(dynamic_typing=True)).value]
        self.scan_noise = dp("scan_noise", 0.0).value
        self.scan_rate = dp("scan_rate", 5.0).value
        self.rng = random.Random(dp("seed", 1).value)

        self.inputs = {}
        for topic, prio in MUX_INPUTS:
            self.create_subscription(Twist, topic, lambda m, t=topic, p=prio: self._input(t, p, m), 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.odom_pub = self.create_publisher(Odometry, "/bumperbot_controller/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/out", qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        self.odom_pose = Pose2D()
        self.true_pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self.delayed = deque()
        self.applied = (0.0, 0.0)
        self.first_cmd_t = None
        self.last = time.monotonic()
        self.fault_t0 = None
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(1.0 / self.rate, self._step)
        if len(self.arena) == 4:
            self.scan_angles = np.arange(SCAN_BEAMS) * 2.0 * math.pi / SCAN_BEAMS
            self.create_timer(1.0 / self.scan_rate, self._scan)

    def _input(self, topic, prio, msg):
        self.inputs[topic] = (prio, time.monotonic(), msg.linear.x, msg.angular.z)

    def _active_command(self, now):
        best = None
        for prio, rx, v, w in self.inputs.values():
            if now - rx < self.mux_timeout and (best is None or prio > best[0]):
                best = (prio, v, w)
        return best

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name in ("fault", "fault_start", "fault_duration"):
                setattr(self, prm.name, prm.value)
                self.fault_t0 = time.monotonic()
            elif prm.name in ("linear_gain", "angular_gain", "tau", "cmd_delay", "noise_std"):
                setattr(self, prm.name, float(prm.value))
        return SetParametersResult(successful=True)

    def _fault_active(self, now):
        start = self.fault_t0 if self.fault_t0 is not None else self.first_cmd_t
        if self.fault == "none" or start is None:
            return False
        t = now - start
        return self.fault_start <= t < self.fault_start + self.fault_duration

    def _step(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        active = self._active_command(now)
        v_cmd, w_cmd = (active[1], active[2]) if active else (0.0, 0.0)
        if active:
            if self.first_cmd_t is None and (abs(v_cmd) > 1e-3 or abs(w_cmd) > 1e-3):
                self.first_cmd_t = now
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_footprint"
            cmd.twist.linear.x = v_cmd
            cmd.twist.angular.z = w_cmd
            self.cmd_pub.publish(cmd)

        self.delayed.append((now, v_cmd, w_cmd))
        while self.delayed and self.delayed[0][0] <= now - self.cmd_delay:
            self.applied = self.delayed.popleft()[1:]
        applied = self.applied

        alpha = 1.0 if self.tau <= 0 else min(1.0, dt / self.tau)
        self.v += alpha * (self.linear_gain * applied[0] - self.v)
        self.w += alpha * (self.angular_gain * applied[1] - self.w)
        noise = (lambda: self.rng.gauss(0.0, self.noise_std)) if self.noise_std > 0 else (lambda: 0.0)
        wheel_v, wheel_w = self.v + noise(), self.w + noise()
        true_v, true_w = wheel_v, wheel_w
        if self._fault_active(now):
            if self.fault == "stall":
                self.v = self.w = 0.0
                wheel_v = wheel_w = true_v = true_w = 0.0
            elif self.fault == "slip":
                true_w = 0.2 * wheel_w
            elif self.fault == "push":
                wheel_v = true_v = 0.12

        self.odom_pose = integrate_unicycle(self.odom_pose, wheel_v, wheel_w, dt)
        self.true_pose = integrate_unicycle(self.true_pose, true_v, true_w, dt)

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = quaternion_z(self.odom_pose.theta)
        odom.twist.twist.linear.x = wheel_v
        odom.twist.twist.angular.z = wheel_w
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = true_w + noise()
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        if self.tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.odom_pose.x
            t.transform.translation.y = self.odom_pose.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)

    def _scan(self):
        p = self.true_pose
        ranges = raycast_rectangle(p.x, p.y, p.theta, self.arena, self.scan_angles, 12.0)
        if self.scan_noise > 0:
            ranges = ranges + np.array([self.rng.gauss(0.0, self.scan_noise) for _ in ranges])
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / SCAN_BEAMS
        msg.angle_max = msg.angle_increment * (SCAN_BEAMS - 1)
        msg.scan_time = 1.0 / self.scan_rate
        msg.range_min = 0.12
        msg.range_max = 12.0
        msg.ranges = [float(r) if math.isfinite(r) else float("inf") for r in ranges]
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeRobot()
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
