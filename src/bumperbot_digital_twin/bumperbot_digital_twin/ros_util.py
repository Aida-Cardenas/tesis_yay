"""Utilidades de ROS 2 compartidas por el ejecutor de experimentos y el panel.

Ambos programas, como el puente, necesitan estar en los dos dominios a la vez:
publican recorridos en el dominio del líder y hablan con el puente.
"""
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

from bumperbot_digital_twin.trajectories import build_segments


def spin_resilient(executor, context, logger):
    """Como executor.spin(), pero una excepción en un callback se registra en vez de matar el hilo."""
    while rclpy.ok(context=context):
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            if not rclpy.ok(context=context):
                break
            logger.error(f"Error en un callback: {exc!r}")


class DualDomain:
    """Un nodo en el dominio del robot real y otro en el del gemelo, cada uno con su ejecutor."""

    def __init__(self, name, real_domain, twin_domain, twin_sim_time=False, args=None):
        if real_domain == twin_domain:
            raise ValueError("real_domain y twin_domain deben ser distintos")
        self.contexts = {"real": Context(), "twin": Context()}
        rclpy.init(args=args, context=self.contexts["real"], domain_id=real_domain)
        rclpy.init(args=None, context=self.contexts["twin"], domain_id=twin_domain)
        self.nodes = {
            "real": Node(name, context=self.contexts["real"]),
            "twin": Node(name, context=self.contexts["twin"], parameter_overrides=[
                Parameter("use_sim_time", value=twin_sim_time)]),
        }
        self.executors = []
        for side in ("real", "twin"):
            ex = SingleThreadedExecutor(context=self.contexts[side])
            ex.add_node(self.nodes[side])
            threading.Thread(target=spin_resilient, daemon=True,
                             args=(ex, self.contexts[side], self.nodes[side].get_logger())).start()
            self.executors.append(ex)

    def ok(self):
        return all(rclpy.ok(context=c) for c in self.contexts.values())

    def call(self, side, client, request, timeout=10.0):
        """Llamada bloqueante a un servicio desde un hilo que no es el del ejecutor."""
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"El servicio {client.srv_name} no responde en el dominio {side}")
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise TimeoutError(f"Sin respuesta de {client.srv_name}")
            time.sleep(0.01)
        return future.result()

    def shutdown(self):
        for ex in self.executors:
            ex.shutdown(timeout_sec=1.0)
        for c in self.contexts.values():
            if c.ok():
                rclpy.shutdown(context=c)


class TrajectoryPlayer:
    """Reproduce un recorrido publicando en key_vel (entrada de teclado de twist_mux)."""

    def __init__(self, dual, topic="key_vel"):
        self.pubs = {side: dual.nodes[side].create_publisher(Twist, topic, 10) for side in ("real", "twin")}
        self.thread = None
        self.stop_flag = threading.Event()

    def play(self, side, pattern, linear_speed=0.15, angular_speed=0.6, distance=1.0, radius=0.5,
             laps=1, pause=1.0, rate=20.0, blocking=True):
        segments = build_segments(pattern, linear_speed, angular_speed, distance, radius, laps, pause)
        self.stop_flag.clear()

        def run():
            pub = self.pubs[side]
            period = 1.0 / rate
            t0 = time.monotonic()
            acc = 0.0
            for duration, v, w in segments:
                end = acc + duration
                while time.monotonic() - t0 < end:
                    if self.stop_flag.is_set():
                        pub.publish(Twist())
                        return
                    msg = Twist()
                    msg.linear.x, msg.angular.z = float(v), float(w)
                    pub.publish(msg)
                    time.sleep(period)
                acc = end
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(period)

        if blocking:
            run()
            return sum(s[0] for s in segments)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        return sum(s[0] for s in segments)

    def stop(self):
        self.stop_flag.set()

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()
