"""Emulación de una red degradada (retardo, variación y pérdida de paquetes).

Se usa dentro del puente para estudiar cómo se degrada la sincronización cuando la
red empeora, sin depender de la calidad real del Wi-Fi. Los mensajes conservan su
orden, como en DDS confiable.
"""
import random
from collections import deque


class NetworkEmulator:
    def __init__(self, delay_ms=0.0, jitter_ms=0.0, loss=0.0, seed=0):
        self.rng = random.Random(seed)
        self.queue = deque()
        self.last_release = 0.0
        self.sent = 0
        self.dropped = 0
        self.configure(delay_ms, jitter_ms, loss)

    def configure(self, delay_ms=None, jitter_ms=None, loss=None):
        if delay_ms is not None:
            self.delay_ms = max(0.0, float(delay_ms))
        if jitter_ms is not None:
            self.jitter_ms = max(0.0, float(jitter_ms))
        if loss is not None:
            self.loss = min(1.0, max(0.0, float(loss)))

    @property
    def enabled(self):
        return self.delay_ms > 0 or self.jitter_ms > 0 or self.loss > 0

    def send(self, item, now):
        """Encola un mensaje. Devuelve False si la red lo pierde."""
        self.sent += 1
        if self.loss > 0 and self.rng.random() < self.loss:
            self.dropped += 1
            return False
        delay = self.delay_ms + (self.rng.gauss(0.0, self.jitter_ms) if self.jitter_ms > 0 else 0.0)
        release = max(now + max(0.0, delay) / 1000.0, self.last_release)
        self.last_release = release
        self.queue.append((release, item))
        return True

    def receive(self, now):
        """Mensajes cuyo tiempo de llegada ya pasó, en orden."""
        out = []
        while self.queue and self.queue[0][0] <= now:
            out.append(self.queue.popleft()[1])
        return out

    @property
    def loss_rate(self):
        return self.dropped / self.sent if self.sent else 0.0
