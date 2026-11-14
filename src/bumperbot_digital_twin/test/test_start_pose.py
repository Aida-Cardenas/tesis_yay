import subprocess
import types

from bumperbot_digital_twin.start_pose import gazebo_set_pose, set_pose_request


def test_request_puts_the_model_at_the_pose():
    req = set_pose_request("bumperbot", 0.0, 0.0, 0.0, 0.0)
    assert req.startswith('name: "bumperbot", position: {x: 0.0000, y: 0.0000, z: 0.0000}')
    assert "w: 1.000000" in req and "z: 0.000000, w" in req


def test_set_pose_uses_ign_on_fortress():
    calls = []

    def run(cmd, **kw):
        calls.append(cmd)
        return types.SimpleNamespace(stdout="data: true\n", stderr="")

    ok, tool = gazebo_set_pose("arena", "bumperbot", run=run, which=lambda t: "/usr/bin/" + t)
    assert ok and tool == "ign"
    cmd = calls[0]
    assert cmd[:4] == ["ign", "service", "-s", "/world/arena/set_pose"]
    assert "ignition.msgs.Pose" in cmd and "ignition.msgs.Boolean" in cmd


def test_set_pose_reports_why_it_failed():
    def run(cmd, **kw):
        if cmd[0] == "ign":
            raise subprocess.TimeoutExpired(cmd, 8)
        return types.SimpleNamespace(stdout="", stderr="Service call timed out")

    ok, detail = gazebo_set_pose("arena", "bumperbot", run=run, which=lambda t: "/usr/bin/" + t)
    assert not ok and detail.startswith("gz:") and "timed out" in detail
    ok, detail = gazebo_set_pose("arena", "bumperbot", run=run, which=lambda t: None)
    assert not ok and "ign" in detail
