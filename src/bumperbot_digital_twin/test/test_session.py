import json

import pytest

from bumperbot_digital_twin.session import RunQueue, Session, build_schedule, parse_answer


def _steps():
    exp = lambda name, reps=5, **kw: {"type": "experiment", "name": name, "repetitions": reps, **kw}
    return [exp("E1"), exp("E2"), {"type": "calibrate", "name": "CAL", "calibrate_from": ["E1", "E2"]},
            {"type": "tune", "name": "TUNE"}, exp("E3"), exp("E4"), exp("E7", twin_model=True),
            exp("G0T", gains="tuned"), exp("A1", reps=2)]


PROTOCOL = {"interleave": [["E1", "E2"], ["E3", "E4", "E7", "G0T"]]}


def test_compared_conditions_alternate_round_by_round():
    segments = build_schedule(_steps(), PROTOCOL, seed=7)
    kinds = [s["kind"] if s["kind"] != "runs" else s["name"] for s in segments]
    assert kinds == ["E1 + E2", "calibrate", "tune", "E3 + E4 + E7 + G0T", "A1"]
    squares = segments[3]["items"]
    assert len(squares) == 20
    for k in range(5):
        round_ = squares[4 * k: 4 * k + 4]
        assert {s["name"] for s, _ in round_} == {"E3", "E4", "E7", "G0T"}
        assert {rep for _, rep in round_} == {k + 1}


def test_same_seed_same_order_and_different_seed_other_order():
    order = lambda seed: [(s["name"], k) for s, k in build_schedule(_steps(), PROTOCOL, seed)[3]["items"]]
    assert order(3) == order(3)
    assert any(order(3) != order(s) for s in range(4, 10))


def test_group_runs_after_its_dependencies():
    steps = _steps()
    steps.insert(0, steps.pop(4))
    segments = build_schedule(steps, PROTOCOL, seed=1)
    names = [s["kind"] if s["kind"] != "runs" else s["name"] for s in segments]
    assert names.index("E3 + E4 + E7 + G0T") > names.index("tune")


def test_bad_interleave_is_rejected():
    with pytest.raises(ValueError):
        build_schedule(_steps(), {"interleave": [["E1", "XX"]]}, seed=1)
    with pytest.raises(ValueError):
        build_schedule(_steps(), {"interleave": [["E1"], ["E1", "E2"]]}, seed=1)


def test_operator_answers():
    assert parse_answer("") == ("c", None)
    assert parse_answer("11,8") == ("c", 11.8)
    assert parse_answer(" s ") == ("s", None)
    assert parse_answer("x") == ("x", None)
    assert parse_answer("q") == ("q", None)
    assert parse_answer("118") == ("?", None)


def test_a_skipped_run_comes_back_once_at_the_end():
    a, b = {"name": "A"}, {"name": "B"}
    q = RunQueue([(a, 1), (b, 1)])
    item = q.pop()
    assert q.skip(item)
    assert q.pop() == (b, 1) and q.pop() == (a, 1)
    assert not q.skip((a, 1))
    assert not q


def test_session_resume_keeps_only_complete_valid_runs(tmp_path):
    s = Session(str(tmp_path))
    s.record(experimento="E1", repeticion=1, registro="a.csv", completa=True)
    s.record(experimento="E1", repeticion=2, registro="b.csv", completa=False, motivo="interrumpida")
    s.record(experimento="E2", repeticion=1, registro="c.csv", completa=True)
    s.invalidate_last("mal hecha")
    s.set("semilla", 42)
    again = Session.load(str(tmp_path))
    assert again.done() == {("E1", 1)}
    assert again.logs() == {"E1": ["a.csv"]}
    assert again.get("semilla") == 42
    assert [r["registro"] for r in again.discarded()] == ["b.csv", "c.csv"]
    assert json.load(open(tmp_path / "sesion.json"))["corridas"][2]["motivo"] == "mal hecha"
    q = RunQueue([({"name": "E1"}, 1), ({"name": "E1"}, 2)], again.done())
    assert q.pop() == ({"name": "E1"}, 2) and not q


def _load(name):
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), "..", "config", name + ".yaml")
    protocol = yaml.safe_load(open(path))
    return protocol


@pytest.mark.parametrize("name", ["protocolo_real", "protocolo_simulacion", "protocolo_ci"])
def test_shipped_protocols_build_a_schedule(name):
    protocol = _load(name)
    defaults = dict({"repetitions": 3}, **protocol.get("defaults", {}))
    steps = []
    for item in protocol["experiments"]:
        if "calibrate_from" in item:
            steps.append({"type": "calibrate", **item})
        elif "tune" in item:
            steps.append({"type": "tune", **item})
        else:
            steps.append({"type": "experiment", **defaults, **item})
    segments = build_schedule(steps, protocol, seed=1)
    order = []
    for seg in segments:
        order += [seg["kind"]] if seg["kind"] != "runs" else [s["name"] for s, _ in seg["items"]]
    for step in steps:
        if step["type"] != "experiment":
            continue
        assert order.count(step["name"]) == step["repetitions"]
        if step.get("twin_model"):
            assert order.index(step["name"]) > order.index("calibrate")
        if step.get("gains") == "tuned":
            assert order.index(step["name"]) > order.index("tune")


@pytest.mark.parametrize("name", ["protocolo_real", "protocolo_simulacion"])
def test_compared_pairs_are_interleaved_and_calibration_is_tested_out_of_sample(name):
    protocol = _load(name)
    group_of = {n: i for i, g in enumerate(protocol["interleave"]) for n in g}
    steps = {s["name"]: s for s in protocol["experiments"]}
    cal = next(s for s in protocol["experiments"] if "calibrate_from" in s)
    for a, b, *_ in protocol["comparisons"]:
        assert a in group_of and group_of.get(a) == group_of.get(b), (name, a, b)
        if steps[a].get("twin_model") or steps[b].get("twin_model"):
            assert not {a, b} & set(cal["calibrate_from"]), (name, a, b)
