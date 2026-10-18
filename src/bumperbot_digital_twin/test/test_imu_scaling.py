import math
import os
import sys

import pytest

try:
    from bumperbot_firmware import mpu6050_math as imu
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "bumperbot_firmware"))
    from bumperbot_firmware import mpu6050_math as imu


def test_gyro_config_selects_2000_dps_and_scale_matches():
    assert imu.GYRO_CONFIG_VALUE == 24
    assert imu.GYRO_LSB_PER_RAD_S == pytest.approx(939.65, abs=0.01)


def test_one_rad_per_second_reads_as_one():
    raw = round(16.4 * math.degrees(1.0))
    assert imu.gyro_to_rad_s(raw) == pytest.approx(1.0, rel=1e-3)


def test_gravity_reads_as_9_8():
    assert imu.ACCEL_CONFIG_VALUE == 0
    assert imu.accel_to_m_s2(16384) == pytest.approx(9.80665)


def test_signed_conversion_edges():
    assert imu.to_signed(0x7F, 0xFF) == 32767
    assert imu.to_signed(0x80, 0x00) == -32768
    assert imu.to_signed(0xFF, 0xFF) == -1
    assert imu.to_signed(0x00, 0x00) == 0


def test_covariances_are_not_zero():
    cov = imu.covariance_diagonal(imu.GYRO_VARIANCE)
    assert cov[0] > 0 and cov[4] > 0 and cov[8] > 0 and cov[1] == 0
