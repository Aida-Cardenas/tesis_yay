import numpy as np
import pytest

from bumperbot_digital_twin.dynamics import (
    FirstOrderFilter, FirstOrderModel, TwinModel, fit_first_order, simulate, wheel_calibration)


def excitation(t):
    return np.where((t % 8) < 4, 0.2, 0.0) + np.where((t % 5) < 2, 0.1, 0.0)


def test_step_response_reaches_gain():
    m = FirstOrderModel(0.8, 0.2, 0.0)
    t = np.arange(0, 3, 0.01)
    y = simulate(m, t, np.ones_like(t))
    assert y[-1] == pytest.approx(0.8, abs=1e-3)
    k = np.searchsorted(t, 0.2)
    assert y[k] == pytest.approx(0.8 * (1 - np.exp(-1)), abs=0.03)


def test_delay_is_respected():
    f = FirstOrderFilter(FirstOrderModel(1.0, 0.0, 0.3))
    out = [f.update(t, 1.0) for t in np.arange(0, 1, 0.05)]
    assert out[4] == 0.0 and out[-1] == 1.0


@pytest.mark.parametrize("gain,tau,delay", [(0.85, 0.35, 0.15), (1.1, 0.1, 0.0), (0.6, 0.6, 0.3)])
def test_fit_recovers_parameters(gain, tau, delay):
    t = np.arange(0, 40, 0.05)
    u = excitation(t)
    y = simulate(FirstOrderModel(gain, tau, delay), t, u)
    y = y + np.random.default_rng(0).normal(0, 0.002, y.size)
    m = fit_first_order(t, u, y)
    assert m.gain == pytest.approx(gain, rel=0.05)
    assert m.tau == pytest.approx(tau, abs=0.08)
    assert m.delay == pytest.approx(delay, abs=0.1)
    assert m.r2 > 0.98


def test_fit_rejects_constant_input():
    t = np.arange(0, 5, 0.05)
    with pytest.raises(ValueError):
        fit_first_order(t, np.zeros_like(t), np.zeros_like(t))


def test_model_roundtrip(tmp_path):
    m = TwinModel(FirstOrderModel(0.9, 0.3, 0.1, 0.99, 100), FirstOrderModel(0.8, 0.2, 0.05), "x")
    path = tmp_path / "m.yaml"
    m.save(path)
    m2 = TwinModel.load(path)
    assert m2.linear.gain == pytest.approx(0.9) and m2.angular.delay == pytest.approx(0.05)
    assert m2.source == "x"


def test_wheel_calibration():
    c = wheel_calibration(0.97, 1.0, 6.0, 6.0)
    assert c["left_wheel_radius_multiplier"] == pytest.approx(1.0 / 0.97)
    assert c["wheel_separation_multiplier"] == pytest.approx(1.0 / 0.97)
    assert wheel_calibration(1.0, 1.0)["wheel_separation_multiplier"] == 1.0
    with pytest.raises(ValueError):
        wheel_calibration(0.0, 1.0)
