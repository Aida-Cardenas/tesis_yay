#!/usr/bin/env python3
"""Navegador de mentira con la misma interfaz que Nav2 (acción navigate_to_pose).

Sirve para probar la vista previa de navegación sin Nav2 ni mapa, junto con
fake_robot. Publica map → odom fijo (identidad), gira hacia la meta, avanza en línea
recta publicando en cmd_vel (la entrada de Nav2 en twist_mux) y se orienta al final.
Rechaza (aborta) las metas fuera del recinto, como haría Nav2 sin un camino posible.
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster

from bumperbot_digital_twin.geometry import yaw_from_quaternion
from bumperbot_digital_twin.navigation import GoToGoalGains, go_to_goal, inside_arena


class FakeNavigator(Node):
    def __init__(self):
        super().__init__("fake_navigator")
        dp = self.declare_parameter
        self.arena = [float(x) for x in dp("arena", [0.0], ParameterDescriptor(dynamic_typing=True)).value]
        self.margin = dp("margin", 0.1).value
        self.timeout = dp("timeout", 60.0).value
        self.gains = GoToGoalGains(max_linear=dp("max_linear", 0.2).value,
                                   max_angular=dp("max_angular", 1.0).value)
        self.group = ReentrantCallbackGroup()
        self.pose = None
        self.create_subscription(Odometry, "/bumperbot_controller/odom", self._odom_cb, 10,
                                 callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)
        self.server = ActionServer(self, NavigateToPose, "navigate_to_pose", self._execute,
                                   goal_callback=lambda g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.get_logger().info("Navegador de mentira listo (acción navigate_to_pose)")

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        g = goal_handle.request.pose.pose
        gx, gy = g.position.x, g.position.y
        gyaw = yaw_from_quaternion(g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w)
        self.get_logger().info(f"Meta ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)")
        if not inside_arena(gx, gy, self.arena, self.margin):
            self.get_logger().warn("La meta está fuera del recinto: no hay camino")
            goal_handle.abort()
            return NavigateToPose.Result()
        t0 = time.monotonic()
        feedback = NavigateToPose.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return NavigateToPose.Result()
            if time.monotonic() - t0 > self.timeout:
                self._stop()
                goal_handle.abort()
                return NavigateToPose.Result()
            if self.pose is None:
                time.sleep(0.05)
                continue
            x, y, yaw = self.pose
            v, w, done = go_to_goal(x, y, yaw, gx, gy, gyaw, self.gains)
            if done:
                self._stop()
                goal_handle.succeed()
                return NavigateToPose.Result()
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.cmd_pub.publish(cmd)
            feedback.current_pose.header.frame_id = "map"
            feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y = x, y
            feedback.distance_remaining = float(math.hypot(gx - x, gy - y))
            feedback.number_of_recoveries = 0
            elapsed = time.monotonic() - t0
            feedback.navigation_time.sec = int(elapsed)
            feedback.navigation_time.nanosec = int((elapsed % 1) * 1e9)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNavigator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
