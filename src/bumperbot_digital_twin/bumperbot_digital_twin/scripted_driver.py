#!/usr/bin/env python3
"""Recorridos repetibles para las pruebas (línea recta, cuadrado, círculo...).

Publica velocidades en key_vel, la entrada de teclado de twist_mux, así que
funciona igual en el robot real, en Gazebo o en fake_robot. Al terminar publica
ceros y se cierra solo. El joystick (prioridad 99) siempre puede interrumpirlo.

Con `duration` > 0 no hace un recorrido: publica linear_speed y angular_speed
durante esos segundos y después cero (Prueba 3, movimiento por comando):

    ros2 run bumperbot_digital_twin scripted_driver --ros-args -p duration:=5.0 \
        -p linear_speed:=0.1 -p angular_speed:=0.0
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

from bumperbot_digital_twin.trajectories import build_segments


class ScriptedDriver(Node):
    def __init__(self):
        super().__init__("scripted_driver")
        pattern = self.declare_parameter("pattern", "square").value
        v = self.declare_parameter("linear_speed", 0.15).value
        w = self.declare_parameter("angular_speed", 0.6).value
        distance = self.declare_parameter("distance", 1.0).value
        radius = self.declare_parameter("radius", 0.5).value
        laps = self.declare_parameter("laps", 1).value
        pause = self.declare_parameter("pause", 1.0).value
        self.start_delay = self.declare_parameter("start_delay", 3.0).value
        topic = self.declare_parameter("topic", "key_vel").value
        duration = self.declare_parameter("duration", 0.0).value
        if duration > 0.0:
            pattern = "constante"
            self.segments = [(duration, v, w)]
        else:
            self.segments = build_segments(pattern, v, w, distance, radius, laps, pause)
        self.total = sum(s[0] for s in self.segments)
        self.pub = self.create_publisher(Twist, topic, 10)
        self.t0 = None
        self.done = False
        self.stop_ticks = 0
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Recorrido '{pattern}': {len(self.segments)} tramos, {self.total:.1f} s, "
            f"empieza en {self.start_delay:.1f} s")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.t0 is None:
            if self._now() == 0.0:
                return
            self.t0 = self._now() + self.start_delay
        t = self._now() - self.t0
        msg = Twist()
        if t < 0:
            return
        if t >= self.total:
            self.pub.publish(msg)
            self.stop_ticks += 1
            if self.stop_ticks > 10:
                self.get_logger().info("Recorrido terminado")
                self.done = True
            return
        acc = 0.0
        for duration, v, w in self.segments:
            if t < acc + duration:
                msg.linear.x = v
                msg.angular.z = w
                break
            acc += duration
        self.pub.publish(msg)


def main():
    # Ctrl-C is handled here and not by rclpy: its handler shuts the context down
    # first, and the zero speed in `finally` could no longer be published.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ScriptedDriver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido: velocidad cero")
    finally:
        for _ in range(3):
            node.pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
