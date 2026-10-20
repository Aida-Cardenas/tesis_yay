import argparse

import pytest

pytest.importorskip("bumperbot_msgs")

from bumperbot_digital_twin.experiment_runner import Runner


def make_runner(tmp_path):
    r = Runner.__new__(Runner)
    r.args = argparse.Namespace(no_plots=True, auto=True, only=None, repetitions=None)
    r.logs = {}
    r.model_path = None
    r.calibration_failed = False
    r.tuned_gains = None
    r.tuning_failed = False
    r.session_dir = str(tmp_path)
    r.calls = []
    r.configure = lambda **kw: r.calls.append(kw)
    return r


def test_calibration_without_runs_installs_nothing(tmp_path):
    r = make_runner(tmp_path)
    assert r.calibrate({"calibrate_from": ["E1", "E2"]}) is False
    assert r.calibration_failed and r.model_path is None
    assert not (tmp_path / "modelo_gemelo.yaml").exists()
    assert not any(c.get("twin_model_file") for c in r.calls)
    assert r.skip_reason({"name": "E7", "twin_model": True}) is not None
    assert r.skip_reason({"name": "E3", "twin_model": False}) is None


def test_tuning_without_model_is_skipped(tmp_path):
    r = make_runner(tmp_path)
    step = {"tune": {}, "defaults": {"pattern": "square", "linear_speed": 0.15, "angular_speed": 0.6,
                                     "distance": 1.0, "radius": 0.5, "pause": 0.5}}
    assert r.tune(step) is False
    assert r.tuning_failed and r.tuned_gains is None
    assert r.skip_reason({"name": "G0T", "gains": "tuned"}) is not None


def test_calibrated_experiments_are_skipped_without_cal_in_this_session(tmp_path):
    r = make_runner(tmp_path)
    assert "CAL" in r.skip_reason({"name": "E7", "twin_model": True})
    r.model_path = str(tmp_path / "modelo.yaml")
    assert r.skip_reason({"name": "E7", "twin_model": True}) is None


def test_each_run_closes_its_log_after_settling(tmp_path, monkeypatch):
    import types
    import bumperbot_digital_twin.experiment_runner as er
    monkeypatch.setattr(er.time, "sleep", lambda s: None)
    r = make_runner(tmp_path)
    calls = []

    def configure(**kw):
        calls.append(kw)
        return types.SimpleNamespace(log_path=str(tmp_path / "x.csv"), message="ok")

    r.configure = configure
    r.player = types.SimpleNamespace(play=lambda *a: calls.append({"play": True}) or 5.0)
    step = {"name": "E1", "description": "", "leader": "real", "feedback": True, "compensation": False,
            "twin_model": False, "net_delay_ms": 0.0, "net_jitter_ms": 0.0, "net_loss": 0.0,
            "pattern": "line", "linear_speed": 0.15, "angular_speed": 0.6, "distance": 1.0, "radius": 0.5,
            "laps": 1, "pause": 0.5, "settle": 2.0}
    assert r.run_experiment(step, 1)
    assert calls[0].get("new_log") and calls[1].get("play") and calls[-1] == {"close_log": True}
