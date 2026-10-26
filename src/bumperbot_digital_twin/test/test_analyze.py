import csv
import math

from conftest import bridge_columns

from bumperbot_digital_twin import analyze_log


def write_run(path, lag_s=0.2, err=0.01, n=400, leader="real", feedback=1, extra=None):
    cols = bridge_columns()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(n):
            t = i * 0.05
            v = 0.2 if 2 < t < 15 else 0.0
            fv = 0.2 if 2 + lag_s < t < 15 + lag_s else 0.0
            row = {c: 0.0 for c in cols}
            row.update(t=1000 + t, leader=leader, feedback=feedback, active=1, pos_error=err, pos_error_raw=err,
                       yaw_error=0.01, leader_x=t * 0.1, follower_x=t * 0.1 - err,
                       real_x=t * 0.1, twin_x=t * 0.1 - err, real_stamp=1000 + t,
                       real_v=v if leader == "real" else fv, twin_v=fv if leader == "real" else v,
                       ff_v=v, rtt_ms=10.0 + (i % 5), real_odom_age_ms=math.nan, leader_cmd_age_ms=5.0,
                       anomalies="", twin_rx=1000 + t, rtt_ema_ms=12.0, clock_offset_ms=0.5,
                       twin_clock=1000 + t)
            row.update(extra or {})
            w.writerow([row[c] for c in cols])
    return path


def test_analyze_synthetic(tmp_path):
    path = write_run(tmp_path / "twin_20260101_000000_test.csv")
    out = tmp_path / "out"
    assert analyze_log.main([str(path), "-o", str(out)]) == 0
    s = analyze_log.analyze(str(path), str(out), plots=False)
    assert abs(s["error_posicion_m"]["media"] - 0.01) < 1e-9
    assert abs(s["error_real_m"]["media"] - 0.01) < 1e-6
    assert 150 <= s["retardo_sincronizacion_ms"] <= 250
    assert (out / "resumen.md").exists()
    assert (out / "twin_20260101_000000_test_trayectorias.png").exists()


def test_true_error_uses_timestamps(tmp_path):
    path = write_run(tmp_path / "twin_20260101_000000_x.csv")
    d = analyze_log.load(str(path))
    d["real_stamp"] = d["real_stamp"] - 0.5
    e = analyze_log.true_error(d)
    finite = e[~__import__("numpy").isnan(e)]
    assert finite.size > 100
    assert abs(finite.mean() - (0.01 + 0.05)) < 0.01


def test_scan_and_events_summary(tmp_path):
    path = write_run(tmp_path / "twin_20260101_000000_y.csv")
    with open(tmp_path / "twin_20260101_000000_y_scan.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(bridge_columns("SCAN_COLUMNS"))
        for i in range(5):
            w.writerow([1000 + i, 300, 0.02, 0.025, 0.001, 0.04, 0.95, 0.96, 0.98, 0.01])
    with open(tmp_path / "twin_20260101_000000_y_eventos.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(bridge_columns("EVENT_COLUMNS"))
        w.writerow([1001, "detector", "atasco", "inicio", 2, 0.1, "x"])
        w.writerow([1003, "detector", "atasco", "fin", 2, 0.1, "x"])
    s = analyze_log.analyze(str(path), str(tmp_path), plots=False)
    assert s["lidar"]["comparaciones"] == 5
    assert abs(s["lidar"]["mae_m"] - 0.02) < 1e-9
    assert s["anomalias"] == {"atasco": 1}
    assert "atasco" in analyze_log.markdown([s])


def test_true_error_interpolates_the_twin_by_arrival_time(tmp_path):
    import numpy as np
    path = write_run(tmp_path / "twin_20260101_000000_z.csv", err=0.0)
    d = analyze_log.load(str(path))
    held = d["twin_x"].copy()
    held[1::2] = held[0::2][: len(held[1::2])]
    rx = d["twin_rx"].copy()
    rx[1::2] = rx[0::2][: len(rx[1::2])]
    d["twin_x"], d["twin_rx"] = held, rx
    e = analyze_log.true_error(d)
    assert np.nanmax(e) < 1e-9
    del d["twin_rx"]
    assert np.nanmax(analyze_log.true_error(d)) > 0.004


def test_clock_problem_is_reported_as_such(tmp_path):
    import numpy as np
    path = write_run(tmp_path / "twin_20260101_000000_w.csv")
    d = analyze_log.load(str(path))
    assert analyze_log.true_error_status(d) is None
    d["real_stamp"] = d["real_stamp"] + 5.0
    assert analyze_log.true_error_status(d) == "relojes"
    assert np.isnan(analyze_log.true_error(d)).all()


def test_compensation_without_rtt_and_real_time_factor(tmp_path):
    import numpy as np
    path = write_run(tmp_path / "twin_20260101_000000_v.csv", extra={"compensation": 1})
    d = analyze_log.load(str(path))
    assert analyze_log.compensation_without_rtt(d) == 0.0
    d["rtt_ema_ms"][:100] = np.nan
    assert analyze_log.compensation_without_rtt(d) == 0.25
    assert abs(analyze_log.real_time_factor(d) - 1.0) < 1e-9
    d["twin_clock"] = 1000 + (d["t"] - 1000) * 0.8
    assert abs(analyze_log.real_time_factor(d) - 0.8) < 1e-9
