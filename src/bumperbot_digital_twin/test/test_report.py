import math

import pytest

from test_analyze import write_run

from bumperbot_digital_twin import report


def test_welch_t():
    t, df, p = report.welch_t([1.0, 1.1, 0.9, 1.05], [2.0, 2.1, 1.9, 2.05])
    assert t < 0 and p < 0.001
    assert math.isnan(report.welch_t([1.0], [2.0])[2])


def test_experiment_name_from_file():
    assert report.experiment_of("/x/twin_20260101_120000_E3_r2.csv") == ("E3", 2)
    assert report.experiment_of("/x/twin_20260101_120000_libre.csv") == ("libre", 1)
    assert report.experiment_of("/x/otro.csv") == (None, None)


def test_build_report(tmp_path):
    paths = []
    for rep in range(1, 4):
        paths.append(write_run(tmp_path / f"twin_20260101_00000{rep}_A_r{rep}.csv", err=0.01 + rep * 0.001))
        paths.append(write_run(tmp_path / f"twin_20260101_00001{rep}_B_r{rep}.csv", err=0.05 + rep * 0.001,
                               feedback=0))
    protocol = {"experiments": [{"name": "A", "description": "con corrección"}, {"name": "B"}],
                "comparisons": [["A", "B"]]}
    md, table, cmp = report.build_report([str(p) for p in paths], str(tmp_path / "inf"), protocol, plots=False)
    assert table["A"]["corridas"] == 3
    assert table["A"]["rmse"][0] == pytest.approx(0.012, abs=1e-6)
    assert cmp[0]["significativa"] and cmp[0]["p"] < 0.01
    assert "con corrección" in md and (tmp_path / "inf" / "informe.md").exists()


def _row(leader="real", fb="sí", comp="no", model="no", gains="1.5/6.0/3.0", delay=0.0, rmse=0.02,
         rmse_real=0.021, anomalias=None):
    return {"lider": leader, "realimentacion": fb, "compensacion": comp, "modelo": model, "ganancias": gains,
            "red_ms": delay, "rmse": (rmse, 0.0, 2), "rmse_real": (rmse_real, 0.0, 2),
            "anomalias": anomalias or {}}


def test_latency_series_keeps_only_comparable_experiments():
    from bumperbot_digital_twin.report import latency_series
    protocol = {"defaults": {"pattern": "square", "gains": "default"}, "experiments": [
        {"name": "E1", "pattern": "line"}, {"name": "E2", "pattern": "line", "feedback": False},
        {"name": "E3"}, {"name": "E6", "leader": "twin"}, {"name": "E7", "feedback": False, "twin_model": True},
        {"name": "L100", "net_delay_ms": 100}, {"name": "L250", "net_delay_ms": 250},
        {"name": "L250C", "net_delay_ms": 250, "compensation": True},
        {"name": "L250R", "leader": "twin", "net_delay_ms": 250},
        {"name": "G0T", "gains": "tuned"}, {"name": "A1", "pattern": "line", "instructions": "sujétalo"},
        {"name": "A2", "pattern": "free"}, {"name": "CAL", "calibrate_from": ["E1"]}, {"name": "TUNE", "tune": {}},
    ]}
    table = {
        "E1": _row(rmse_real=0.5), "E2": _row(fb="no", rmse_real=0.9), "E3": _row(rmse_real=0.01),
        "E6": _row(leader="twin", rmse_real=0.02), "E7": _row(fb="no", model="sí", rmse_real=0.7),
        "L100": _row(delay=100, rmse_real=0.02), "L250": _row(delay=250, rmse_real=0.04),
        "L250C": _row(delay=250, comp="sí", rmse_real=0.03), "L250R": _row(leader="twin", delay=250, rmse_real=0.2),
        "G0T": _row(gains="0.3/1.0/0.3", rmse_real=0.8), "A1": _row(anomalias={"atasco": 1}, rmse_real=0.9),
        "A2": _row(rmse_real=0.9),
    }
    s = latency_series(table, protocol)
    assert s["metric"] == "rmse_real" and s["pattern"] == "square"
    used = {e for pts in s["series"].values() for p in pts for e in p[2]}
    assert used == {"E3", "L100", "L250", "E6", "L250R"}
    assert [p[0] for p in s["series"][("real", "no")]] == [0.0, 100, 250]


def test_latency_series_uses_one_metric():
    from bumperbot_digital_twin.report import latency_series
    table = {"A": _row(delay=0), "B": _row(delay=100), "C": _row(delay=250, rmse_real=float("nan"))}
    assert latency_series(table)["metric"] == "rmse"


def test_deduplicated_log_name_keeps_experiment():
    from bumperbot_digital_twin.report import experiment_of
    assert experiment_of("/x/twin_20261001_101010_E3_r1-2.csv") == ("E3", 1)
