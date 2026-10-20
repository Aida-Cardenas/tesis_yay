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


def test_gyro_bias_is_measured_and_removed():
    bias = imu.GyroBias(samples=100)
    for _ in range(99):
        assert not bias.add(0.01, -0.02, 0.03)
    assert bias.add(0.01, -0.02, 0.03)
    assert bias.bias == pytest.approx((0.01, -0.02, 0.03))
    assert bias.correct(0.51, -0.02, 1.03) == pytest.approx((0.5, 0.0, 1.0))


def test_gyro_bias_restarts_if_robot_moves():
    bias = imu.GyroBias(samples=10)
    for _ in range(5):
        bias.add(0.0, 0.0, 0.03)
    bias.add(0.0, 0.0, 0.5)
    assert bias.restarts == 1 and not bias.ready
    for _ in range(10):
        bias.add(0.0, 0.0, 0.03)
    assert bias.ready and bias.bias[2] == pytest.approx(0.03)


def test_gyro_bias_can_be_disabled():
    bias = imu.GyroBias(samples=0)
    assert bias.ready and bias.correct(1.0, 2.0, 3.0) == (1.0, 2.0, 3.0)


def test_burst_read_splits_accel_and_gyro_and_skips_temperature():
    from bumperbot_firmware.mpu6050_math import parse_burst
    data = [0x00, 0x10, 0xFF, 0xF0, 0x40, 0x00, 0x12, 0x34, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00]
    accel, gyro = parse_burst(data)
    assert accel == (16, -16, 16384)
    assert gyro == (1, -1, -32768)


def test_burst_read_rejects_short_transfers():
    from bumperbot_firmware.mpu6050_math import parse_burst
    with pytest.raises(ValueError):
        parse_burst([0] * 12)


def test_constant_rotation_while_wheels_move_is_not_taken_as_bias():
    from bumperbot_firmware.mpu6050_math import GyroBias
    bias = GyroBias(50)
    for i in range(200):
        if i < 120:
            bias.restart()
        else:
            bias.add(0.0, 0.0, 0.01)
    assert bias.ready and bias.bias[2] == pytest.approx(0.01)
    assert bias.restarts == 0 and not bias.suspicious


def test_restart_counts_only_when_samples_were_discarded():
    from bumperbot_firmware.mpu6050_math import GyroBias
    bias = GyroBias(50)
    for _ in range(10):
        bias.add(0.0, 0.0, 0.0)
    bias.restart()
    bias.restart()
    assert bias.restarts == 1


def test_large_steady_bias_is_flagged():
    from bumperbot_firmware.mpu6050_math import GyroBias
    bias = GyroBias(20)
    for _ in range(20):
        bias.add(0.0, 0.0, 0.8)
    assert bias.ready and bias.suspicious
