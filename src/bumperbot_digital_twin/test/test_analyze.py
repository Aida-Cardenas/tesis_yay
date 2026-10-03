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
                       anomalies="")
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
