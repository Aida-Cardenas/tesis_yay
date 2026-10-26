import json
import math

from bumperbot_digital_twin.nav_preview import json_safe


def test_json_safe_replaces_nan_and_inf():
    data = {"a": math.nan, "b": [1.0, math.inf, {"c": -math.inf}], "d": "x", "e": 2}
    text = json.dumps(json_safe(data), allow_nan=False)
    assert json.loads(text) == {"a": None, "b": [1.0, None, {"c": None}], "d": "x", "e": 2}


def test_json_safe_handles_numpy_values():
    import numpy as np
    text = json.dumps(json_safe({"a": np.float64("nan"), "b": np.float32(1.5), "c": np.int64(3)}), allow_nan=False)
    assert json.loads(text) == {"a": None, "b": 1.5, "c": 3}


def test_ctrl_c_cancels_the_nav2_goal():
    import types
    import pytest
    from bumperbot_digital_twin.nav_preview import NavSide

    class Done:
        def __init__(self, value=None):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class Pending:
        def done(self):
            return False

    cancelled = []
    handle = types.SimpleNamespace(accepted=True, get_result_async=Pending,
                                   cancel_goal_async=lambda: cancelled.append(True) or Done())
    side = NavSide.__new__(NavSide)
    side.name = "real"
    side.frame = "map"
    side.lock = __import__("threading").Lock()
    side.scan_min = math.inf
    side.NavigateToPose = types.SimpleNamespace(Goal=lambda: types.SimpleNamespace(
        pose=types.SimpleNamespace(header=types.SimpleNamespace(frame_id=""),
                                   pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0, y=0),
                                                              orientation=types.SimpleNamespace(z=0, w=1)))))
    side.client = types.SimpleNamespace(wait_for_server=lambda timeout_sec: True,
                                        send_goal_async=lambda goal, feedback_callback: Done(handle))

    def pose(timeout=0.5):
        raise KeyboardInterrupt

    side.pose = pose
    with pytest.raises(KeyboardInterrupt):
        side._navigate_once(1.0, 0.0, 0.0, 60.0, log=lambda *a: None)
    assert cancelled == [True]
