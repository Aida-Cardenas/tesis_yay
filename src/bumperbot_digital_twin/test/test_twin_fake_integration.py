"""Prueba de punta a punta sin hardware: dos robots de mentira en dos dominios DDS.

El gemelo de mentira tiene 10 % menos de velocidad que el real (error de modelo).
Con realimentación el puente debe mantenerlos juntos; sin ella deben separarse.
Se prueba en los dos sentidos (líder real y líder gemelo).
"""
import glob
import os
import shutil
import signal
import subprocess
import time

import pytest

from bumperbot_digital_twin import analyze_log

pytestmark = pytest.mark.skipif(shutil.which("ros2") is None, reason="ROS 2 no disponible")

CASES = [
    # id, leader, feedback, dominios
    ("real_fb", "real", "true", 61, 62),
    ("real_mirror", "real", "false", 63, 64),
    ("twin_fb", "twin", "true", 65, 66),
]


def run_case(tmp_path, leader, feedback, rd, td):
    log_dir = tmp_path / "logs"
    cmd = [
        "ros2", "launch", "bumperbot_digital_twin", "digital_twin.launch.py",
        "real_mode:=fake", "twin_mode:=fake", "rviz:=false",
        f"leader:={leader}", f"feedback:={feedback}",
        f"real_domain:={rd}", f"twin_domain:={td}",
        "driver:=square", "driver_start_delay:=3.0",
        "linear_speed:=0.3", "angular_speed:=1.0", "distance:=0.6",
        f"log_dir:={log_dir}", "log_tag:=ci",
    ]
    out = open(tmp_path / "launch.log", "w")
    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(40)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        out.close()
    csvs = sorted(glob.glob(str(log_dir / "twin_*_ci.csv")))
    assert csvs, "El puente no escribió ningún CSV:\n" + (tmp_path / "launch.log").read_text()[-3000:]
    return analyze_log.analyze(csvs[-1], str(tmp_path), plots=False), csvs[-1]


@pytest.mark.parametrize("case_id,leader,feedback,rd,td", CASES)
def test_sync(tmp_path, case_id, leader, feedback, rd, td):
    s, path = run_case(tmp_path, leader, feedback, rd, td)
    report = os.environ.get("TWIN_TEST_REPORT")
    if report:
        e = s["error_posicion_m"]
        with open(report, "a") as f:
            f.write(f"{case_id}: lider={leader} realim={feedback} dur={s['duracion_s']:.1f}s "
                    f"err_medio={e.get('media', float('nan')) * 100:.1f}cm "
                    f"max={e.get('max', float('nan')) * 100:.1f}cm "
                    f"final={s['error_final_m'] * 100:.1f}cm "
                    f"rtt={s['rtt_ms'].get('media', float('nan')):.1f}ms "
                    f"retardo={s['retardo_sincronizacion_ms']:.0f}ms n={s['muestras']}\n")
    assert s["muestras"] > 200
    assert s["error_posicion_m"]["n"] > 100, "Nunca hubo datos de ambos robots"
    assert s["rtt_ms"]["n"] > 10, "No se midió la latencia"
    if feedback == "true":
        assert s["error_posicion_m"]["max"] < 0.10
        assert s["error_final_m"] < 0.05
    else:
        assert s["error_final_m"] > 0.10
