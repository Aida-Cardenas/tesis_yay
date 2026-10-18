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

GYRO_LSB_PER_RAD_S = GYRO_LSB_PER_DPS[GYRO_RANGE] * 180.0 / math.pi
ACCEL_LSB_PER_M_S2 = ACCEL_LSB_PER_G[ACCEL_RANGE] / STANDARD_GRAVITY

GYRO_VARIANCE = 0.02 ** 2
ACCEL_VARIANCE = 0.1 ** 2


def to_signed(high, low):
    value = (high << 8) | low
    if value >= 32768:
        value -= 65536
    return value


def gyro_to_rad_s(raw):
    return raw / GYRO_LSB_PER_RAD_S


def accel_to_m_s2(raw):
    return raw / ACCEL_LSB_PER_M_S2


def covariance_diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
