import math

import pytest

from bumperbot_digital_twin.trajectories import PATTERNS, build_segments


@pytest.mark.parametrize("pattern", PATTERNS)
def test_every_pattern_builds(pattern):
    segs = build_segments(pattern, 0.15, 0.6, 1.0, 0.5, 1, 0.5)
    assert segs and all(d > 0 for d, _, _ in segs)


def test_line_distance():
    segs = build_segments("line", 0.2, 0.6, 1.0, 0.5, 1, 0.0)
    assert sum(d * v for d, v, _ in segs) == pytest.approx(1.0)


def test_square_turns_full_circle():
    segs = build_segments("square", 0.2, 0.5, 1.0, 0.5, 1, 0.0)
    assert sum(d * w for d, _, w in segs) == pytest.approx(2 * math.pi)


def test_unknown_pattern():
    with pytest.raises(ValueError):
        build_segments("zigzag", 0.1, 0.1, 1, 1, 1, 0)


@pytest.mark.parametrize("pattern,args", [
    ("line", (0.0, 0.6, 1.0, 0.5)), ("rotate", (0.15, 0.0, 1.0, 0.5)),
    ("circle", (0.15, 0.6, 1.0, 0.0)), ("square", (0.15, -0.5, 1.0, 0.5)),
])
def test_zero_speed_or_size_is_rejected_instead_of_dividing_by_zero(pattern, args):
    with pytest.raises(ValueError):
        build_segments(pattern, *args, 1, 0.0)


def test_unused_speed_may_be_zero():
    assert build_segments("rotate", 0.0, 0.6, 0.0, 0.0, 1, 0.0)
    assert build_segments("line", 0.2, 0.0, 1.0, 0.0, 1, 0.0)
