#!/usr/bin/env python3
"""Eco para medir la latencia de ida y vuelta entre el PC y la Raspberry Pi.

Corre en el robot real. Devuelve cada mensaje de /digital_twin/ping por
/digital_twin/pong sin tocarlo; el puente calcula el tiempo de ida y vuelta con
su propio reloj, así que no hace falta sincronizar relojes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class LatencyEcho(Node):
    def __init__(self):
        super().__init__("latency_echo")
        self.pub = self.create_publisher(Header, "/digital_twin/pong", 10)
        self.create_subscription(Header, "/digital_twin/ping", self.pub.publish, 10)


def main():
    rclpy.init()
    node = LatencyEcho()
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
