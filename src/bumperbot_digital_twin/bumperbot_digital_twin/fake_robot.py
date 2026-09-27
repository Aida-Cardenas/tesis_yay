#!/usr/bin/env python3
"""Robot de mentira para probar el gemelo digital sin hardware ni Gazebo.

Imita la interfaz del bumperbot: recibe velocidades por las mismas entradas que
twist_mux (joy_vel, twin_vel, key_vel, cmd_vel, con sus prioridades), publica el
comando aplicado en /bumperbot_controller/cmd_vel y la odometría en
/bumperbot_controller/odom. Permite simular diferencias con el robot ideal:
ganancia de velocidad, retardo de respuesta (constante de tiempo) y ruido.
"""
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from bumperbot_digital_twin.geometry import Pose2D, integrate_unicycle

MUX_INPUTS = (("joy_vel", 99), ("twin_vel", 95), ("key_vel", 90), ("cmd_vel", 80))


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.rate = self.declare_parameter("rate", 50.0).value
        self.linear_gain = self.declare_parameter("linear_gain", 1.0).value
        self.angular_gain = self.declare_parameter("angular_gain", 1.0).value
        self.tau = self.declare_parameter("tau", 0.1).value
        self.noise_std = self.declare_parameter("noise_std", 0.0).value
        self.mux_timeout = self.declare_parameter("mux_timeout", 0.5).value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.seed = self.declare_parameter("seed", 1).value
        random.seed(self.seed)

        self.inputs = {}
        for topic, prio in MUX_INPUTS:
            self.create_subscription(Twist, topic, lambda m, t=topic, p=prio: self._input(t, p, m), 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.odom_pub = self.create_publisher(Odometry, "/bumperbot_controller/odom", 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self.last = time.monotonic()
        self.create_timer(1.0 / self.rate, self._step)

    def _input(self, topic, prio, msg):
        self.inputs[topic] = (prio, time.monotonic(), msg.linear.x, msg.angular.z)

    def _active_command(self, now):
        best = None
        for prio, rx, v, w in self.inputs.values():
            if now - rx < self.mux_timeout and (best is None or prio > best[0]):
                best = (prio, v, w)
        return best

    def _step(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        active = self._active_command(now)
        v_cmd, w_cmd = (active[1], active[2]) if active else (0.0, 0.0)
        if active:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_footprint"
            cmd.twist.linear.x = v_cmd
            cmd.twist.angular.z = w_cmd
            self.cmd_pub.publish(cmd)
        alpha = 1.0 if self.tau <= 0 else min(1.0, dt / self.tau)
        self.v += alpha * (self.linear_gain * v_cmd - self.v)
        self.w += alpha * (self.angular_gain * w_cmd - self.w)
        v = self.v + (random.gauss(0.0, self.noise_std) if self.noise_std > 0 else 0.0)
        w = self.w + (random.gauss(0.0, self.noise_std) if self.noise_std > 0 else 0.0)
        self.pose = integrate_unicycle(self.pose, v, w, dt)

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.z = math.sin(self.pose.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.pose.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)
        if self.tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.pose.x
            t.transform.translation.y = self.pose.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)


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
