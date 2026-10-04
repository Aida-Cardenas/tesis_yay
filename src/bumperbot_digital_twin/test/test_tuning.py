import pytest

from bumperbot_digital_twin.dynamics import FirstOrderModel, TwinModel
from bumperbot_digital_twin.tuning import (
    DEFAULT_GAINS, Scenario, load_gains_file, save_gains, scenarios_from, simulate, tune,
)

REAL = TwinModel(FirstOrderModel(0.85, 0.3, 0.1), FirstOrderModel(0.85, 0.3, 0.1))
TWIN = TwinModel(FirstOrderModel(1.0, 0.05, 0.0), FirstOrderModel(1.0, 0.05, 0.0))


def test_feedback_beats_nothing_and_delay_hurts():
    s0 = Scenario("square", "twin", 0.0, False, 0.3, 1.0, 0.6)
    weak = simulate({"kx": 0.01, "ky": 0.01, "ktheta": 0.01}, s0, REAL, TWIN)
    good = simulate(DEFAULT_GAINS, s0, REAL, TWIN)
    assert good.rmse < weak.rmse
    delayed = simulate(DEFAULT_GAINS, Scenario("square", "real", 0.3, False, 0.3, 1.0, 0.6), REAL, TWIN)
    assert delayed.rmse > good.rmse


def test_compensation_helps_with_delay():
    s = Scenario("square", "real", 0.3, False, 0.3, 1.0, 0.6)
    sc = Scenario("square", "real", 0.3, True, 0.3, 1.0, 0.6)
    assert simulate(DEFAULT_GAINS, sc, REAL, TWIN).rmse < simulate(DEFAULT_GAINS, s, REAL, TWIN).rmse


def test_tuning_improves_delayed_real_follower():
    scenarios = [Scenario("square", "real", 0.3, False, 0.3, 1.0, 0.6)]
    result = tune(scenarios, REAL, TWIN)
    assert result.cost < result.default_cost * 0.8
    before, after = result.per_scenario[0]["rmse_antes"], result.per_scenario[0]["rmse_despues"]
    assert after < before * 0.8
    for k in ("kx", "ky", "ktheta"):
        assert result.gains[k] > 0


def test_tuning_never_worse_than_start():
    scenarios = scenarios_from(["line"], ["twin"], [0.0])
    result = tune(scenarios, TWIN, TWIN, grid=2, iters=10)
    assert result.cost <= result.default_cost + 1e-9


def test_gains_file_roundtrip(tmp_path):
    path = tmp_path / "g.yaml"
    save_gains(path, {"kx": 0.5, "ky": 2.0, "ktheta": 1.25}, info="prueba")
    assert load_gains_file(path) == pytest.approx({"kx": 0.5, "ky": 2.0, "ktheta": 1.25})
    flat = tmp_path / "flat.yaml"
    flat.write_text("kx: 1\nky: 2\nktheta: 3\n")
    assert load_gains_file(flat) == {"kx": 1.0, "ky": 2.0, "ktheta": 3.0}
    bad = tmp_path / "bad.yaml"
    bad.write_text("kx: 1\n")
    with pytest.raises(ValueError):
        load_gains_file(bad)


def test_tune_cli(tmp_path):
    from bumperbot_digital_twin import tune as tune_cli
    REAL.save(tmp_path / "modelo.yaml")
    out = tmp_path / "ganancias.yaml"
    assert tune_cli.main(["--real-model", str(tmp_path / "modelo.yaml"), "--follower", "real", "--delay-ms", "300",
                          "--patterns", "square", "--linear-speed", "0.3", "--angular-speed", "1.0",
                          "--distance", "0.6", "-o", str(out)]) == 0
    assert load_gains_file(out)
    assert (tmp_path / "ganancias_informe.md").read_text().count("square/real/300ms") == 1
    assert (tmp_path / "ganancias.png").stat().st_size > 10000
