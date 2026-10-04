#!/usr/bin/env python3
"""Mide cuánto avanzó y giró el robot según su odometría, para calibrar las ruedas.

    ros2 run bumperbot_digital_twin twin_odom_meter

Desde que se lanza, muestra la distancia en línea recta desde el punto de partida y el
giro acumulado (sin dar la vuelta en ±180°). Mueve el robot y lee los números al final;
son los valores --odom-distance y --odom-angle de twin_calibrate wheels.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from bumperbot_digital_twin.geometry import wrap_angle, yaw_from_quaternion


class OdomMeter(Node):
    def __init__(self):
        super().__init__("odom_meter")
        topic = self.declare_parameter("odom_topic", "/bumperbot_controller/odom").value
        self.start = None
        self.last_yaw = None
        self.turn = 0.0
        self.path = 0.0
        self.last_xy = None
        self.create_subscription(Odometry, topic, self._cb, 10)
        self.create_timer(0.5, self._print)
        self.get_logger().info(f"Midiendo {topic}. Mueve el robot; Ctrl+C para terminar.")

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self.start is None:
            self.start = (p.x, p.y)
            self.last_yaw = yaw
            self.last_xy = (p.x, p.y)
            return
        self.turn += wrap_angle(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.path += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def summary(self):
        if self.start is None:
            return "Sin datos de odometría todavía"
        d = math.hypot(self.last_xy[0] - self.start[0], self.last_xy[1] - self.start[1])
        return (f"distancia desde la salida: {d:.3f} m · recorrido total: {self.path:.3f} m · "
                f"giro acumulado: {self.turn:.3f} rad ({math.degrees(self.turn):.1f}°)")

    def _print(self):
        print("\r" + self.summary() + "   ", end="", flush=True)


def main():
    rclpy.init()
    node = OdomMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\nResultado final: " + node.summary())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
