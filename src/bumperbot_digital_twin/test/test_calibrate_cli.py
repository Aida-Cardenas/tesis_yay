import csv

import numpy as np
import pytest
import yaml

from conftest import bridge_columns

from bumperbot_digital_twin import calibrate
from bumperbot_digital_twin.dynamics import FirstOrderModel, TwinModel, simulate


def test_dynamics_cli(tmp_path):
    cols = bridge_columns()
    t = np.arange(0, 40, 0.05)
    u = np.where((t % 8) < 4, 0.2, 0.0)
    uw = np.where((t % 6) < 3, 0.6, -0.3)
    y = simulate(FirstOrderModel(0.85, 0.3, 0.1), t, u)
    yw = simulate(FirstOrderModel(0.9, 0.2, 0.05), t, uw)
    path = tmp_path / "twin_20260101_000000_M_r1.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(t.size):
            row = {c: 0 for c in cols}
            row.update(t=1000 + t[i], leader="real", stale=0, ff_v=u[i], real_v=y[i], ff_w=uw[i], real_w=yw[i],
                       anomalies="")
            w.writerow([row[c] for c in cols])
    out = tmp_path / "modelo.yaml"
    assert calibrate.main(["dynamics", str(path), "-o", str(out)]) == 0
    m = TwinModel.load(out)
    assert m.linear.gain == pytest.approx(0.85, rel=0.05)
    assert m.angular.gain == pytest.approx(0.9, rel=0.05)
    assert (tmp_path / "modelo_ajuste.png").exists()


def test_wheels_cli(tmp_path):
    out = tmp_path / "ruedas.yaml"
    assert calibrate.main(["wheels", "--odom-distance", "0.95", "--real-distance", "1.0",
                           "--odom-angle", "6.0", "--real-angle", "6.2832", "-o", str(out)]) == 0
    data = yaml.safe_load(open(out))["bumperbot_controller"]["ros__parameters"]
    assert data["left_wheel_radius_multiplier"] == pytest.approx(1 / 0.95)


def test_arena_cli(tmp_path):
    out = tmp_path / "arena.world"
    assert calibrate.main(["arena", "--bounds", "-0.5", "2", "-1", "1", "--obstacle", "1", "0", "0.2", "0.2",
                           "-o", str(out)]) == 0
    assert "obstaculo_0" in out.read_text()


def test_arena_cli_writes_world_and_map(tmp_path):
    world = tmp_path / "arena.world"
    maps = tmp_path / "mapa"
    assert calibrate.main(["arena", "--bounds", "-0.5", "2.0", "-1.0", "1.0", "-o", str(world),
                           "--map-dir", str(maps)]) == 0
    assert world.exists()
    meta = yaml.safe_load(open(maps / "map.yaml"))
    assert meta["image"] == "map.pgm" and meta["resolution"] == pytest.approx(0.05)
    raw = open(maps / "map.pgm", "rb").read()
    assert raw.startswith(b"P5")
    header = raw.split(b"\n")
    w, h = (int(v) for v in header[2].split())
    pixels = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)[::-1]
    ox, oy = meta["origin"][:2]

    def cell(x, y):
        return pixels[int((y - oy) / 0.05), int((x - ox) / 0.05)]

    assert cell(0.0, 0.0) == 254
    assert cell(2.02, 0.0) == 0
    assert cell(-0.52, 0.0) == 0
    assert cell(0.5, 1.02) == 0
