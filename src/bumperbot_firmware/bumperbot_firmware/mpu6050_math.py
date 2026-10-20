"""Conversión de las lecturas crudas del MPU6050 a unidades SI (sin dependencias de ROS).

El factor depende del rango configurado en el sensor:
  giroscopio ±250/±500/±1000/±2000 °/s  -> 131/65.5/32.8/16.4 LSB por °/s
  acelerómetro ±2/±4/±8/±16 g          -> 16384/8192/4096/2048 LSB por g
"""
import math

GYRO_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)
ACCEL_LSB_PER_G = (16384.0, 8192.0, 4096.0, 2048.0)
STANDARD_GRAVITY = 9.80665

GYRO_RANGE = 3
ACCEL_RANGE = 0

GYRO_CONFIG_VALUE = GYRO_RANGE << 3
ACCEL_CONFIG_VALUE = ACCEL_RANGE << 3

# Filtro paso bajo interno (DLPF_CFG=3, unos 44 Hz). El nodo lee a 100 Hz, así
# que con el valor 0 (260 Hz) el ruido por encima de 50 Hz se doblaba.
DLPF_CONFIG_VALUE = 3

# Rango plausible del cero de un MPU6050 (unos ±20 °/s). Un cero mayor casi
# siempre significa que el robot estaba girando mientras se medía.
MAX_PLAUSIBLE_BIAS = 0.35

GYRO_LSB_PER_RAD_S = GYRO_LSB_PER_DPS[GYRO_RANGE] * 180.0 / math.pi
ACCEL_LSB_PER_M_S2 = ACCEL_LSB_PER_G[ACCEL_RANGE] / STANDARD_GRAVITY

GYRO_VARIANCE = 0.02 ** 2
ACCEL_VARIANCE = 0.1 ** 2


def to_signed(high, low):
    value = (high << 8) | low
    if value >= 32768:
        value -= 65536
    return value


def parse_burst(data):
    """Separa las 14 lecturas de una sola transacción desde ACCEL_XOUT_H (0x3B).

    Orden en el MPU6050: acelerómetro x, y, z, temperatura, giroscopio x, y, z,
    cada uno con el byte alto primero. Leerlos juntos evita mezclar el byte alto
    de una muestra con el bajo de la siguiente.
    """
    if len(data) < 14:
        raise ValueError(f"se esperaban 14 bytes y llegaron {len(data)}")
    words = [to_signed(data[i], data[i + 1]) for i in range(0, 14, 2)]
    return tuple(words[0:3]), tuple(words[4:7])


def gyro_to_rad_s(raw):
    return raw / GYRO_LSB_PER_RAD_S


def accel_to_m_s2(raw):
    return raw / ACCEL_LSB_PER_M_S2


def covariance_diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class GyroBias:
    """Desviación del cero del giroscopio, medida con el robot quieto al arrancar.

    El MPU6050 marca algunos °/s aunque no gire. Sin restarlo, el EKF integra un
    giro que no existe y el detector de anomalías ve deslizamientos falsos.
    Si durante la medición el giro varía más que `max_spread` (rad/s), se supone
    que alguien movió el robot y se empieza de nuevo. Un giro constante no cambia
    la dispersión, por eso el driver también llama a `restart()` cuando las
    ruedas se mueven, y `suspicious` avisa si el cero medido es demasiado grande.
    """

    def __init__(self, samples, max_spread=0.05):
        self.samples = int(samples)
        self.max_spread = max_spread
        self.bias = (0.0, 0.0, 0.0)
        self.restarts = 0
        self._reset()
        self.ready = self.samples <= 0

    def _reset(self):
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._low = [math.inf] * 3
        self._high = [-math.inf] * 3

    def restart(self):
        if self.ready:
            return
        if self._count > 0:
            self.restarts += 1
        self._reset()

    @property
    def suspicious(self):
        return self.ready and max(abs(b) for b in self.bias) > MAX_PLAUSIBLE_BIAS

    def add(self, x, y, z):
        if self.ready:
            return True
        values = (x, y, z)
        for i, v in enumerate(values):
            self._sum[i] += v
            self._low[i] = min(self._low[i], v)
            self._high[i] = max(self._high[i], v)
        self._count += 1
        if max(h - l for h, l in zip(self._high, self._low)) > self.max_spread:
            self.restarts += 1
            self._reset()
            return False
        if self._count >= self.samples:
            self.bias = tuple(s / self._count for s in self._sum)
            self.ready = True
        return self.ready

    def correct(self, x, y, z):
        return x - self.bias[0], y - self.bias[1], z - self.bias[2]
