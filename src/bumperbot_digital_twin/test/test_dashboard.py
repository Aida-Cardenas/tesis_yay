import os

import pytest

pytest.importorskip("PyQt5")


def test_dashboard_selftest(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from bumperbot_digital_twin import dashboard
    out = tmp_path / "panel.png"
    assert dashboard.selftest(str(out)) == 0
    assert out.stat().st_size > 10000


def test_late_status_does_not_undo_user_checkbox(tmp_path):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5 import QtWidgets
    from bumperbot_digital_twin.dashboard import FakeActions
    from bumperbot_digital_twin.dashboard_ui import DashboardWindow
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = DashboardWindow(FakeActions())
    status = {"leader": "real", "position_error": 0.01, "feedback": True, "compensation": True, "twin_model": False}
    win.update_status(status)
    assert win.chk_comp.isChecked()
    win.chk_comp.setChecked(False)
    win.update_status(status)
    assert not win.chk_comp.isChecked()
    win._flags_changed_at -= 5.0
    win.update_status(status)
    assert win.chk_comp.isChecked()
