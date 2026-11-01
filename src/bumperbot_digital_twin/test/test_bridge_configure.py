import math
import threading
import types
from collections import deque

import pytest

pytest.importorskip("bumperbot_msgs")

from bumperbot_msgs.srv import TwinConfigure

from bumperbot_digital_twin.netem import NetworkEmulator
from bumperbot_digital_twin.twin_bridge import TwinBridge


def make_bridge():
    b = TwinBridge.__new__(TwinBridge)
    b.lock = threading.RLock()
    b.p = {"net_delay_ms": 0.0, "net_jitter_ms": 0.0, "net_loss": 0.0, "sync": True, "feedback": True,
           "compensation": False, "twin_model": False, "log_enabled": False, "log_tag": ""}
    b.net_in, b.net_out = NetworkEmulator(seed=1), NetworkEmulator(seed=2)
    b.ping_seq, b.ping_epoch, b.ping_sent = 10, 0, {}
    b.rtt_ms = b.rtt_ema_ms = 10.1
    b.clock_samples = deque([(10.1, 30.0)], maxlen=30)
    b.clock_offset_ms = 30.0
    b.twin_model = b.twin_filters = b.csv_file = None
    b.csv_path = ""
    b.main = types.SimpleNamespace(get_logger=lambda: types.SimpleNamespace(info=lambda m: None))
    return b


def request(**kw):
    req = TwinConfigure.Request()
    req.feedback = req.compensation = req.twin_model = -1
    req.net_delay_ms = req.net_jitter_ms = req.net_loss = -1.0
    for k, v in kw.items():
        setattr(req, k, v)
    return req


def test_same_network_keeps_the_rtt_and_the_clock_offset():
    b = make_bridge()
    res = b.srv_configure(request(net_delay_ms=0.0, net_jitter_ms=0.0, net_loss=0.0), TwinConfigure.Response())
    assert res.clock_offset_ms == 30.0 and res.rtt_ms == pytest.approx(10.1)
    assert b.ping_epoch == 0


def test_a_new_network_resets_the_rtt_but_not_the_clock_offset():
    b = make_bridge()
    res = b.srv_configure(request(net_delay_ms=250.0, net_jitter_ms=20.0, net_loss=0.0), TwinConfigure.Response())
    assert math.isnan(res.rtt_ms) and math.isnan(b.rtt_ms)
    assert res.clock_offset_ms == 30.0
    assert b.ping_epoch == b.ping_seq


def test_clock_offset_is_only_measured_without_emulated_delay(monkeypatch):
    import bumperbot_digital_twin.twin_bridge as tb
    b = make_bridge()
    b.srv_configure(request(net_delay_ms=250.0, net_jitter_ms=20.0), TwinConfigure.Response())

    def pong(seq, sent_wall, echo_wall, now_mono):
        b.ping_sent[seq] = (now_mono - 0.5, sent_wall)
        monkeypatch.setattr(tb.time, "monotonic", lambda: now_mono)
        msg = types.SimpleNamespace(frame_id=f"{seq};t",
                                    stamp=types.SimpleNamespace(sec=int(echo_wall), nanosec=0))
        b._pong_cb(msg)

    pong(11, 1000.0, 1000.0, 50.0)
    assert b.clock_offset_ms == 30.0 and b.rtt_ms == pytest.approx(500.0)
    b.srv_configure(request(net_delay_ms=0.0, net_jitter_ms=0.0), TwinConfigure.Response())
    pong(20, 2000.0, 2000.0, 60.0)
    assert len(b.clock_samples) == 2
