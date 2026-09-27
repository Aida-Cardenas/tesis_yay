import csv
import math
import os

from bumperbot_digital_twin import analyze_log

COLUMNS = [
    "t", "leader", "feedback", "active", "lost_sync", "stale",
    "pos_error", "yaw_error", "ex", "ey",
    "leader_x", "leader_y", "leader_yaw",
    "follower_x", "follower_y", "follower_yaw",
    "real_x", "real_y", "real_yaw", "real_v", "real_w",
    "twin_x", "twin_y", "twin_yaw", "twin_v", "twin_w",
    "ff_v", "ff_w", "cmd_v", "cmd_w",
    "rtt_ms", "real_odom_age_ms", "leader_cmd_age_ms",
]


def test_columns_match_bridge():
    import re
    here = os.path.dirname(__file__)
    src = open(os.path.join(here, "..", "bumperbot_digital_twin", "twin_bridge.py")).read()
    cols = eval(re.search(r"CSV_COLUMNS = (\[.*?\])", src, re.S).group(1))
    assert cols == COLUMNS


def test_analyze_synthetic(tmp_path):
    path = tmp_path / "twin_test.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i in range(400):
            t = i * 0.05
            v = 0.2 if 2 < t < 15 else 0.0
            fv = 0.2 if 2.2 < t < 15.2 else 0.0
            err = 0.01
            row = {c: 0.0 for c in COLUMNS}
            row.update(t=1000 + t, leader="real", feedback=1, active=1, pos_error=err, yaw_error=0.01,
                       leader_x=t * 0.1, follower_x=t * 0.1 - err, real_v=v, twin_v=fv, ff_v=v,
                       rtt_ms=10.0 + (i % 5), real_odom_age_ms=math.nan, leader_cmd_age_ms=5.0)
            w.writerow([row[c] for c in COLUMNS])
    out = tmp_path / "out"
    assert analyze_log.main([str(path), "-o", str(out)]) == 0
    s = analyze_log.analyze(str(path), str(out), plots=False)
    assert abs(s["error_posicion_m"]["media"] - 0.01) < 1e-9
    assert 150 <= s["retardo_sincronizacion_ms"] <= 250
    assert (out / "resumen.md").exists()
    assert (out / "twin_test_trayectorias.png").exists()
