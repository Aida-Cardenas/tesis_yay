import math
import xml.dom.minidom

import numpy as np
import pytest

from bumperbot_digital_twin.scan_compare import arena_world_sdf, bin_scan, compare_scans, raycast_rectangle

ANG = np.arange(360) * 2 * math.pi / 360


def test_raycast_inside_square():
    r = raycast_rectangle(0.0, 0.0, 0.0, [-1, 1, -1, 1], np.array([0.0, math.pi / 2, math.pi / 4]), 12)
    assert r[0] == pytest.approx(1.0) and r[1] == pytest.approx(1.0) and r[2] == pytest.approx(math.sqrt(2))


def test_bin_scan_different_layouts_agree():
    r = raycast_rectangle(0.2, 0.1, 0.3, [-0.8, 1.4, -0.8, 1.4], ANG, 12)
    a = bin_scan(0.0, 2 * math.pi / 360, r, 0.12, 12)
    angles720 = -math.pi + np.arange(720) * math.pi / 360
    r720 = raycast_rectangle(0.2, 0.1, 0.3, [-0.8, 1.4, -0.8, 1.4], angles720, 12)
    b = bin_scan(-math.pi, math.pi / 360, r720, 0.12, 12)
    assert np.isnan(a).sum() == 0
    res = compare_scans(a, b)
    assert res["common"] == 360 and res["mae"] < 0.01


def test_compare_detects_noise_and_bias():
    r = raycast_rectangle(0, 0, 0, [-1, 2, -1, 1], ANG, 12)
    a = bin_scan(0, 2 * math.pi / 360, r, 0.12, 12)
    res = compare_scans(a + 0.05, a)
    assert res["bias"] == pytest.approx(0.05) and res["mae"] == pytest.approx(0.05)
    b = a.copy()
    b[:90] = np.nan
    assert compare_scans(a, b)["visibility_agreement"] == pytest.approx(0.75)


def test_invalid_ranges_are_ignored():
    out = bin_scan(0, 2 * math.pi / 4, [float("inf"), 0.05, 1.0, float("nan")], 0.12, 12, bins=4)
    assert np.isnan(out[0]) and np.isnan(out[1]) and out[2] == 1.0


def test_arena_world_is_valid_sdf():
    sdf = arena_world_sdf([-0.5, 2, -1, 1], obstacles=[(1, 0, 0.2, 0.2)])
    doc = xml.dom.minidom.parseString(sdf)
    assert len(doc.getElementsByTagName("model")) == 6
    with pytest.raises(ValueError):
        arena_world_sdf([1, 0, 0, 1])
