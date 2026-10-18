import math

import numpy as np

from bumperbot_digital_twin.anomaly import AnomalyDetector, Hysteresis


def run(signal, n=400, dt=0.02):
    d = AnomalyDetector()
    events = []
    for k in range(n):
        t = k * dt
        cmd_v, cmd_w, meas_v, meas_w, imu = signal(t, d)
        events += d.update(t, cmd_v, cmd_w, meas_v, meas_w, imu)
    return [(e.kind, e.phase, round(e.t, 2)) for e in events]


def starts(events):
    return [e[0] for e in events if e[1] == "inicio"]


def test_hysteresis():
    h = Hysteresis(0.5, 0.5)
    assert h.update(0.0, True) is None
    assert h.update(0.6, True) == "inicio"
    assert h.update(0.7, False) is None
    assert h.update(1.3, False) == "fin"


def test_nominal_has_no_events():
    rng = np.random.default_rng(1)
    ev = run(lambda t, d: (0.2 * math.sin(t), 0.5 * math.cos(t), d.pred_v.y + rng.normal(0, 0.01),
                           d.pred_w.y, d.pred_w.y), n=1500)
    assert ev == []


def test_stall():
    ev = run(lambda t, d: (0.2 if t < 6 else 0.0, 0.0, d.pred_v.y if t < 3 else 0.0, 0.0, 0.0))
    assert starts(ev) == ["atasco"]
    assert ("atasco", "fin") in [(k, p) for k, p, _ in ev]


def test_slip():
    ev = run(lambda t, d: (0.15, 0.5, d.pred_v.y, d.pred_w.y, d.pred_w.y if t < 3 else 0.0))
    assert starts(ev) == ["deslizamiento"]


def test_unexpected_motion():
    ev = run(lambda t, d: (0.0, 0.0, 0.1 if 2 < t < 4 else 0.0, 0.0, 0.0), n=300)
    assert starts(ev) == ["movimiento_no_comandado"]


def test_model_deviation():
    ev = run(lambda t, d: (0.2, 0.0, 0.6 * d.pred_v.y, 0.0, 0.0), n=600)
    assert starts(ev) == ["desviacion_modelo"]


def test_stall_lock_holds_and_retries_without_flapping():
    from bumperbot_digital_twin.anomaly import StallLock
    lock = StallLock(hold_time=3.0, retry_time=1.5)
    assert lock.update(0.0, False) is None
    assert lock.update(1.0, True) == "lock"
    assert lock.update(2.0, False) is None and lock.locked
    assert lock.update(4.0, True) == "unlock"
    assert lock.update(4.5, True) is None
    assert lock.update(5.6, True) == "lock"
    assert lock.update(8.7, False) == "unlock"
    assert lock.update(20.0, False) is None and not lock.locked


def test_frozen_detector_keeps_stall_active_until_robot_really_moves():
    from bumperbot_digital_twin.anomaly import AnomalyDetector, StallLock
    det = AnomalyDetector()
    lock = StallLock(hold_time=3.0, retry_time=1.5)
    events = []
    t = 0.0
    while t < 12.0:
        jammed = t < 7.0
        if lock.update(t, "atasco" in det.active) or lock.locked:
            t += 0.02
            continue
        meas = 0.0 if jammed else 0.2
        events += [(round(e.t, 2), e.kind, e.phase) for e in det.update(t, 0.2, 0.0, meas, 0.0, 0.0)]
        t += 0.02
    stall = [e for e in events if e[1] == "atasco"]
    assert [e[2] for e in stall] == ["inicio", "fin"]
    assert stall[1][0] > 7.0
