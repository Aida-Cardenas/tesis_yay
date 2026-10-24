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
    for rep in range(1, 6):
        paths.append(write_run(tmp_path / f"twin_20260101_00000{rep}_A_r{rep}.csv", err=0.01 + rep * 0.001))
        paths.append(write_run(tmp_path / f"twin_20260101_00001{rep}_B_r{rep}.csv", err=0.05 + rep * 0.001,
                               feedback=0))
    protocol = {"experiments": [{"name": "A", "description": "con corrección"}, {"name": "B"}],
                "comparisons": [["A", "B"]]}
    md, table, cmp = report.build_report([str(p) for p in paths], str(tmp_path / "inf"), protocol, plots=False)
    assert table["A"]["corridas"] == 5
    assert table["A"]["rmse"][0] == pytest.approx(0.013, abs=1e-6)
    assert cmp[0]["significativa"] and cmp[0]["p"] < 0.01
    lo, hi = cmp[0]["ic95"]
    assert lo < cmp[0]["diferencia"] < hi < 0
    assert "IC 95 %" in md
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


def test_latency_series_needs_the_protocol():
    from bumperbot_digital_twin.report import latency_series
    table = {"A": _row(delay=0), "B": _row(delay=100), "C": _row(delay=250)}
    result = latency_series(table)
    assert result["series"] == {} and "protocolo" in result["note"]


def test_latency_series_always_uses_the_real_error_and_skips_runs_without_it():
    from bumperbot_digital_twin.report import latency_series
    protocol = {"experiments": [{"name": n, "net_delay_ms": d} for n, d in
                                (("A", 0), ("B", 100), ("C", 250), ("D", 400))]}
    table = {"A": _row(delay=0, rmse=0.01, rmse_real=0.02), "B": _row(delay=100, rmse=0.01, rmse_real=0.03),
             "C": _row(delay=250, rmse=0.01, rmse_real=0.05), "D": _row(delay=400, rmse_real=float("nan"))}
    result = latency_series(table, protocol)
    assert result["metric"] == "rmse_real"
    assert [(p[0], p[1]) for p in result["series"][("real", "no")]] == [(0, 0.02), (100, 0.03), (250, 0.05)]


def test_latency_series_ignores_logs_outside_the_protocol():
    from bumperbot_digital_twin.report import latency_series
    protocol = {"experiments": [{"name": n, "net_delay_ms": d} for n, d in (("A", 0), ("B", 100), ("C", 250))]}
    table = {"A": _row(delay=0), "B": _row(delay=100), "C": _row(delay=250),
             "fin": _row(delay=0, rmse_real=0.9), "calibracion": _row(delay=0, rmse_real=0.9),
             "N1": _row(delay=0, rmse_real=0.9)}
    used = {e for pts in latency_series(table, protocol)["series"].values() for p in pts for e in p[2]}
    assert used == {"A", "B", "C"}


@pytest.mark.parametrize("name", ["protocolo_real", "protocolo_simulacion"])
def test_shipped_protocols_give_a_clean_latency_figure(name):
    import os
    import yaml
    from bumperbot_digital_twin.report import latency_series, protocol_steps
    path = os.path.join(os.path.dirname(__file__), "..", "config", name + ".yaml")
    protocol = yaml.safe_load(open(path))
    table = {}
    for exp, step in protocol_steps(protocol).items():
        table[exp] = _row(leader=step.get("leader", "real"), fb="sí" if step.get("feedback", True) else "no",
                          comp="sí" if step.get("compensation") else "no",
                          model="sí" if step.get("twin_model") else "no",
                          gains="0.3/1.0/0.3" if step.get("gains") == "tuned" else "1.5/6.0/3.0",
                          delay=step.get("net_delay_ms", 0), rmse_real=0.02 + step.get("net_delay_ms", 0) / 1e4)
    result = latency_series(table, protocol)
    assert result["series"], name
    steps = protocol_steps(protocol)
    for (leader, comp), pts in result["series"].items():
        for _, _, exps in pts:
            for e in exps:
                assert steps[e].get("pattern", protocol["defaults"].get("pattern")) == result["pattern"]
                assert steps[e].get("gains") != "tuned" and not steps[e].get("twin_model")


def test_comparisons_of_compensation_use_the_real_error():
    import os
    import yaml
    for name in ("protocolo_real", "protocolo_simulacion", "protocolo_ci"):
        path = os.path.join(os.path.dirname(__file__), "..", "config", name + ".yaml")
        protocol = yaml.safe_load(open(path))
        steps = {s["name"]: s for s in protocol["experiments"] if "name" in s}
        for pair in protocol["comparisons"]:
            a, b = steps[pair[0]], steps[pair[1]]
            delayed = any(s.get("net_delay_ms", 0) > 0 for s in (a, b))
            if a.get("compensation") != b.get("compensation") or delayed:
                assert len(pair) > 2 and pair[2] == "rmse_real", (name, pair)


def test_welch_ignores_nan_runs():
    from bumperbot_digital_twin.report import welch_t
    t, df, p = welch_t([0.10, 0.11, 0.12, float("nan")], [0.20, 0.21, 0.22])
    assert p < 0.05


def test_untagged_duplicate_log_is_not_an_experiment():
    from bumperbot_digital_twin.report import experiment_of
    assert experiment_of("/x/twin_20261001_101010-2.csv") == (None, None)


def test_deduplicated_log_name_keeps_experiment():
    from bumperbot_digital_twin.report import experiment_of
    assert experiment_of("/x/twin_20261001_101010_E3_r1-2.csv") == ("E3", 1)


def test_few_runs_are_not_conclusive_even_with_small_p():
    from bumperbot_digital_twin.report import compare
    c = compare([0.019, 0.0191], [0.017, 0.0171])
    assert c["p"] < 0.05
    assert not c["significativa"] and not c["concluyente"] and c["veredicto"].startswith("no concluyente")


def test_welch_interval_matches_scipy():
    import numpy as np
    from scipy import stats
    from bumperbot_digital_twin.report import welch_ci
    a, b = [0.12, 0.10, 0.11, 0.13, 0.12], [0.08, 0.09, 0.07, 0.08, 0.10]
    diff, lo, hi = welch_ci(a, b)
    ref = stats.ttest_ind(a, b, equal_var=False).confidence_interval(0.95)
    assert diff == pytest.approx(np.mean(a) - np.mean(b))
    assert (lo, hi) == pytest.approx((ref.low, ref.high), rel=1e-9)


def test_welch_gives_no_p_without_scipy(monkeypatch):
    import builtins
    from bumperbot_digital_twin.report import welch_t
    real_import = builtins.__import__

    def no_scipy(name, *args, **kwargs):
        if name.startswith("scipy"):
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_scipy)
    t, df, p = welch_t([0.1, 0.11, 0.12], [0.2, 0.21, 0.22])
    assert t < 0 and math.isnan(p)


@pytest.mark.parametrize("name", ["protocolo_real", "protocolo_simulacion"])
def test_compared_experiments_have_enough_runs(name):
    import os
    import yaml
    from bumperbot_digital_twin.report import MIN_RUNS_FOR_TEST
    path = os.path.join(os.path.dirname(__file__), "..", "config", name + ".yaml")
    protocol = yaml.safe_load(open(path))
    default = protocol["defaults"].get("repetitions", 1)
    steps = {s["name"]: s for s in protocol["experiments"] if "name" in s}
    for pair in protocol["comparisons"]:
        for exp in pair[:2]:
            assert steps[exp].get("repetitions", default) >= MIN_RUNS_FOR_TEST, (name, exp)
