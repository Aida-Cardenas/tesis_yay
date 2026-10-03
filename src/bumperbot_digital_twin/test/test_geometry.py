import math

import pytest

from bumperbot_digital_twin.geometry import (
    Pose2D, TrackingGains, alignment_transform, integrate_unicycle,
    tracking_command, tracking_error, wrap_angle, yaw_from_quaternion,
)


def close(a, b, tol=1e-9):
    return abs(a.x - b.x) < tol and abs(a.y - b.y) < tol and abs(wrap_angle(a.theta - b.theta)) < tol


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)


def test_yaw_from_quaternion():
    yaw = 0.7
    assert yaw_from_quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


def test_compose_inverse_is_identity():
    p = Pose2D(1.2, -0.4, 0.9)
    assert close(p.compose(p.inverse()), Pose2D())
    assert close(p.inverse().compose(p), Pose2D())


def test_alignment_maps_leader_onto_follower():
    leader = Pose2D(2.0, 1.0, 0.5)
    follower = Pose2D(-1.0, 0.3, -1.2)
    T = alignment_transform(follower, leader)
    assert close(T.compose(leader), follower)
    moved = leader.compose(Pose2D(1.0, 0.0, 0.0))
    assert close(T.compose(moved), follower.compose(Pose2D(1.0, 0.0, 0.0)))


def test_tracking_error_in_follower_frame():
    follower = Pose2D(0.0, 0.0, math.pi / 2)
    ref = Pose2D(0.0, 1.0, math.pi / 2)
    ex, ey, eth = tracking_error(ref, follower)
    assert ex == pytest.approx(1.0)
    assert ey == pytest.approx(0.0, abs=1e-12)
    assert eth == pytest.approx(0.0)


def test_zero_error_gives_feedforward():
    v, w = tracking_command(0.2, 0.3, (0.0, 0.0, 0.0), TrackingGains())
    assert (v, w) == pytest.approx((0.2, 0.3))


def test_feedback_pushes_towards_reference():
    v, w = tracking_command(0.0, 0.0, (0.1, 0.0, 0.0), TrackingGains())
    assert v > 0
    v, w = tracking_command(0.1, 0.0, (0.0, 0.05, 0.0), TrackingGains())
    assert w > 0


def test_saturation_and_mirror_mode():
    g = TrackingGains(max_linear=0.3, max_angular=1.0)
    v, w = tracking_command(1.0, 5.0, (2.0, 0.0, 0.0), g)
    assert v == pytest.approx(0.3) and w == pytest.approx(1.0)
    assert tracking_command(0.1, 0.2, (1.0, 1.0, 1.0), g, feedback=False) == pytest.approx((0.1, 0.2))


def test_integrate_unicycle_circle():
    p = Pose2D()
    for _ in range(1000):
        p = integrate_unicycle(p, 0.5, 1.0, 2 * math.pi / 1000)
    assert close(p, Pose2D(), tol=1e-6)


def test_closed_loop_converges_despite_model_error():
    leader, follower = Pose2D(), Pose2D(0.05, -0.05, 0.1)
    T = alignment_transform(Pose2D(), Pose2D())
    dt = 0.05
    for _ in range(600):
        leader = integrate_unicycle(leader, 0.2, 0.3, dt)
        err = tracking_error(T.compose(leader), follower)
        v, w = tracking_command(0.2, 0.3, err, TrackingGains())
        follower = integrate_unicycle(follower, 0.9 * v, 0.9 * w, dt)
    assert follower.distance_to(leader) < 0.03


def test_smith_predictor_replays_commands_in_flight():
    from bumperbot_digital_twin.geometry import smith_predict
    sent = [(k * 0.05, 0.2, 0.0) for k in range(21)]
    p = smith_predict(Pose2D(), sent, 1.0, 0.2, 0.2)
    assert p.x == pytest.approx(0.08, abs=1e-6)
    sent = [(k * 0.05, 0.2 if k * 0.05 < 0.9 else 0.0, 0.0) for k in range(21)]
    assert smith_predict(Pose2D(), sent, 1.0, 0.2, 0.2).x == pytest.approx(0.06, abs=1e-6)
    assert smith_predict(Pose2D(1, 2, 0.5), [], 1.0, 0.0, 0.0) == Pose2D(1, 2, 0.5)
