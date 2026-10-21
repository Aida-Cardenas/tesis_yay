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


@pytest.mark.parametrize("delay", [0.0, 0.02, 0.05, 0.07, 0.1, 0.13, 0.3])
def test_fit_recovers_the_filter_delay_exactly(delay):
    t = np.arange(0, 40, 0.05)
    u = excitation(t)
    y = simulate(FirstOrderModel(0.85, 0.3, delay), t, u)
    m = fit_first_order(t, u, y)
    assert m.delay == pytest.approx(delay, abs=1e-6)
    assert m.gain == pytest.approx(0.85, rel=1e-3)
    assert m.tau == pytest.approx(0.3, rel=1e-3)


def test_filter_matches_discrete_model():
    model = FirstOrderModel(0.8, 0.25, 0.1)
    dt = 0.05
    t = np.arange(0, 10, dt)
    u = excitation(t)
    y = simulate(model, t, u)
    a, b = model.discrete(dt)
    d = int(round(model.delay / dt))
    for k in range(d + 1, len(t) - 1):
        assert y[k + 1] == pytest.approx(a * y[k] + b * u[k - d], abs=1e-9)


def test_simulated_r2_is_stricter_than_one_step_r2():
    t = np.arange(0, 40, 0.05)
    u = excitation(t)
    y = simulate(FirstOrderModel(0.8, 0.3, 0.1), t, u)
    wrong = FirstOrderModel(1.0, 0.3, 0.1)
    from bumperbot_digital_twin.dynamics import simulated_r2
    assert simulated_r2(wrong, t, u, y) < 0.95
    m = fit_first_order(t, u, y)
    assert m.r2 == pytest.approx(1.0, abs=1e-6) and m.r2_step == pytest.approx(1.0, abs=1e-6)


def test_filter_delay_does_not_depend_on_the_step():
    model = FirstOrderModel(0.9, 0.2, 0.07)
    fine = np.arange(0, 10, 0.001)
    coarse = np.arange(0, 10, 0.05)
    exc = lambda t: excitation(np.atleast_1d(t))
    y_fine = simulate(model, fine, exc(fine))
    y_coarse = simulate(model, coarse, exc(coarse))
    idx = np.searchsorted(fine, coarse - 1e-9)
    assert np.max(np.abs(y_fine[idx] - y_coarse)) < 0.01


def test_fit_recovers_a_delay_between_samples_from_a_faster_robot():
    model = FirstOrderModel(0.85, 0.3, 0.1)
    fast = np.arange(0, 60, 0.02)
    y = simulate(model, fast, excitation(fast))
    log = np.arange(0.013, 59, 0.05)
    idx = np.searchsorted(fast, log, side="right") - 1
    m = fit_first_order(log, excitation(fast)[idx], y[idx])
    assert m.delay == pytest.approx(0.1, abs=0.02)
    assert m.tau == pytest.approx(0.3, rel=0.1)
