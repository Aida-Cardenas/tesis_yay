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
