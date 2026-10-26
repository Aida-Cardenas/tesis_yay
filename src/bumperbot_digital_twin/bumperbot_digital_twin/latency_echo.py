#!/usr/bin/env python3
"""Eco para medir la latencia de ida y vuelta entre el PC y la Raspberry Pi.

Corre en el robot real. Devuelve cada mensaje de /digital_twin/ping por
/digital_twin/pong. El puente calcula el tiempo de ida y vuelta con su propio
reloj; además, el eco escribe en el sello la hora de la Raspberry Pi, y con eso
el puente estima la diferencia entre los relojes de los dos computadores.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class LatencyEcho(Node):
    def __init__(self):
        super().__init__("latency_echo")
        self.pub = self.create_publisher(Header, "/digital_twin/pong", 10)
        self.create_subscription(Header, "/digital_twin/ping", self.echo, 10)

    def echo(self, msg):
        now = time.time()
        msg.stamp.sec = int(now)
        msg.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.frame_id = msg.frame_id.split(";")[0] + ";t"
        self.pub.publish(msg)


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
