import math

import numpy as np
import pytest

from bumperbot_digital_twin.geometry import Pose2D, integrate_unicycle
from bumperbot_digital_twin.navigation import (
    ApprovalCriteria, TwinRun, approve, compare_paths, go_to_goal, inside_arena, min_valid_range,
    path_length, point_to_polyline,
)


def test_path_length_and_distance_to_polyline():
    line = [(0, 0), (1, 0), (1, 1)]
    assert path_length(line) == pytest.approx(2.0)
    d = point_to_polyline([(0.5, 0.2), (1.3, 0.5), (2, 2)], line)
    assert d == pytest.approx([0.2, 0.3, math.hypot(1, 1)])


def test_compare_identical_and_shifted_paths():
    pred = [(x, 0.0) for x in np.linspace(0, 1, 21)]
    same = compare_paths(pred, pred)
    assert same["desviacion_media_m"] == pytest.approx(0.0, abs=1e-9)
    shifted = compare_paths(pred, [(x, 0.05) for x in np.linspace(0, 1, 21)])
    assert shifted["desviacion_media_m"] == pytest.approx(0.05)
    assert shifted["hausdorff_m"] == pytest.approx(0.05)
    assert shifted["longitud_real_m"] == pytest.approx(1.0)


def test_approval_rules():
    good = TwinRun(True, "lograda", 12.0, min_clearance=0.4, recoveries=0, final_error=0.03)
    assert approve(good) == (True, [])
    for run, text in [
        (TwinRun(False, "abortada", 2.0), "no llegó"),
        (TwinRun(True, "lograda", 12.0, min_clearance=0.1), "obstáculo"),
        (TwinRun(True, "lograda", 500.0, min_clearance=0.4), "tardó"),
        (TwinRun(True, "lograda", 12.0, min_clearance=0.4, recoveries=5), "recuperación"),
        (TwinRun(True, "lograda", 12.0, min_clearance=0.4, final_error=0.6), "meta"),
    ]:
        ok, reasons = approve(run, ApprovalCriteria())
        assert not ok and any(text in r for r in reasons), reasons
    ok, reasons = approve(TwinRun(False, "abortada", 0.1, final_error=3.0))
    assert len(reasons) == 1


def test_min_valid_range_ignores_invalid_and_body():
    assert min_valid_range([0.05, float("inf"), 0.8, 0.3, float("nan")], 0.12, 12.0) == pytest.approx(0.3)
    assert min_valid_range([0.2, 0.9], 0.12, 12.0, ignore_below=0.25) == pytest.approx(0.9)
    assert min_valid_range([], 0.12, 12.0) == math.inf


@pytest.mark.parametrize("goal", [(1.0, 0.0, 0.0), (0.5, 0.8, math.pi / 2), (-0.6, -0.4, math.pi), (0.0, 0.0, -2.0)])
def test_go_to_goal_reaches_goal(goal):
    pose = Pose2D()
    for _ in range(2000):
        v, w, done = go_to_goal(pose.x, pose.y, pose.theta, *goal)
        if done:
            break
        pose = integrate_unicycle(pose, v, w, 0.05)
    assert done
    assert math.hypot(pose.x - goal[0], pose.y - goal[1]) < 0.06
    assert abs(math.atan2(math.sin(pose.theta - goal[2]), math.cos(pose.theta - goal[2]))) < 0.06


def test_inside_arena():
    arena = [-0.5, 2.0, -1.0, 1.0]
    assert inside_arena(1.0, 0.0, arena, 0.1)
    assert not inside_arena(3.0, 0.0, arena, 0.1)
    assert not inside_arena(1.95, 0.0, arena, 0.1)
    assert inside_arena(50.0, 50.0, [0.0], 0.1)
