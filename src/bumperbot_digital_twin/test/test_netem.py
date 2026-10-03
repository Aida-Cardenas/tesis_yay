from bumperbot_digital_twin.netem import NetworkEmulator


def test_disabled_by_default():
    n = NetworkEmulator()
    assert not n.enabled
    n.send("a", 0.0)
    assert n.receive(0.0) == ["a"]


def test_fixed_delay():
    n = NetworkEmulator(delay_ms=100)
    n.send(1, 0.0)
    assert n.receive(0.09) == []
    assert n.receive(0.1) == [1]


def test_order_and_loss_rate():
    n = NetworkEmulator(delay_ms=50, jitter_ms=30, loss=0.2, seed=3)
    got = []
    for k in range(2000):
        n.send(k, k * 0.01)
        got += n.receive(k * 0.01)
    got += n.receive(1e9)
    assert got == sorted(got)
    assert 0.17 < n.loss_rate < 0.23
    assert len(got) == n.sent - n.dropped


def test_reconfigure():
    n = NetworkEmulator()
    n.configure(delay_ms=10, loss=1.0)
    assert n.enabled
    assert n.send(1, 0.0) is False
