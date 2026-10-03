import os

import pytest

pytest.importorskip("PyQt5")


def test_dashboard_selftest(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from bumperbot_digital_twin import dashboard
    out = tmp_path / "panel.png"
    assert dashboard.selftest(str(out)) == 0
    assert out.stat().st_size > 10000
