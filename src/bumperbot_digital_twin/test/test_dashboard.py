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


def test_second_navigation_is_refused_while_the_first_runs(monkeypatch):
    import sys
    import threading
    import types
    from bumperbot_digital_twin.dashboard import RosActions
    release = threading.Event()
    started = []

    class FakePreview:
        def __init__(self, *a, **k):
            pass

        def run(self, *a, **k):
            started.append(a)
            release.wait(5.0)
            return {"aprobada": True, "motivos": []}

    monkeypatch.setitem(sys.modules, "bumperbot_digital_twin.nav_preview", types.SimpleNamespace(NavPreview=FakePreview))
    monkeypatch.setattr("bumperbot_digital_twin.dashboard.time.sleep", lambda s: None)
    messages, nav = [], []
    win = types.SimpleNamespace(show_message=messages.append)
    actions = RosActions.__new__(RosActions)
    actions.window_ref = lambda: win
    actions.preview = None
    actions.nav_busy = False
    actions.dual = actions._configure = None
    actions._nav_text = nav.append
    actions.navigate(1.0, 0.0, 0.0, False)
    assert actions.nav_busy
    actions.navigate(2.0, 0.0, 0.0, False)
    assert messages == ["Ya hay una navegación en curso"]
    release.set()
    for _ in range(100):
        if not actions.nav_busy:
            break
        threading.Event().wait(0.02)
    assert not actions.nav_busy and len(started) == 1 and nav == ["Aprobada"]


def test_pressing_start_twice_leaves_a_single_trajectory_running():
    pytest.importorskip("rclpy")
    import threading
    import types
    from bumperbot_digital_twin.ros_util import TrajectoryPlayer
    published = []
    lock = threading.Lock()

    class Pub:
        def __init__(self, side):
            self.side = side

        def publish(self, msg):
            with lock:
                published.append((self.side, threading.get_ident(), msg.linear.x))

    node = lambda side: types.SimpleNamespace(create_publisher=lambda *a: Pub(side))
    dual = types.SimpleNamespace(nodes={"real": node("real"), "twin": node("twin")})
    player = TrajectoryPlayer(dual)
    player.play("real", "line", linear_speed=0.1, distance=1.0, pause=0.0, rate=50.0, blocking=False)
    first = player.thread
    threading.Event().wait(0.1)
    player.play("real", "line", linear_speed=0.2, distance=1.0, pause=0.0, rate=50.0, blocking=False)
    assert not first.is_alive() and player.running
    threading.Event().wait(0.1)
    with lock:
        cut = len(published)
    threading.Event().wait(0.1)
    with lock:
        later = {(tid, v) for _, tid, v in published[cut:]}
    player.stop()
    player.thread.join(2.0)
    assert later and all(tid == player.thread.ident and v in (0.2, 0.0) for tid, v in later)
