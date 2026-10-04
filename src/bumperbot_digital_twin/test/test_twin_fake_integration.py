"""Pruebas de punta a punta sin hardware: dos robots de mentira en dos dominios DDS.

- Sincronización: el gemelo de mentira tiene 10 % menos de velocidad que el real
  (error de modelo). Con realimentación el puente debe mantenerlos juntos; sin ella
  deben separarse. Se prueba en los dos sentidos. Ambos robots están en el mismo
  recinto, así que además se comparan sus LiDAR.
- Anomalías: se inyectan fallas en el robot "real" (atasco, deslizamiento, empujón)
  y el detector debe reportarlas.
- Protocolo: el ejecutor automático corre el protocolo de CI completo (calibración,
  red degradada, compensación) y el informe debe mostrar las mejoras esperadas.
"""
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import time

import pytest

from bumperbot_digital_twin import analyze_log

pytestmark = pytest.mark.skipif(shutil.which("ros2") is None, reason="ROS 2 no disponible")

ARENA = "[-0.5,1.5,-0.5,1.5]"
SERIOUS = {"atasco", "deslizamiento", "movimiento_no_comandado", "perdida_datos"}


def report(line):
    path = os.environ.get("TWIN_TEST_REPORT")
    if path:
        with open(path, "a") as f:
            f.write(line + "\n")


def start(cmd, log_path):
    out = open(log_path, "w")
    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, out


def stop(proc, out):
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)
    out.close()


def twin_launch(log_dir, rd, td, *extra):
    return [
        "ros2", "launch", "bumperbot_digital_twin", "digital_twin.launch.py",
        "real_mode:=fake", "twin_mode:=fake", "rviz:=false",
        f"real_domain:={rd}", f"twin_domain:={td}",
        f"log_dir:={log_dir}", "log_tag:=ci", *extra,
    ]


def run_launch(tmp_path, rd, td, duration, *extra):
    log_dir = tmp_path / "logs"
    launch_log = tmp_path / "launch.log"
    proc, out = start(twin_launch(log_dir, rd, td, *extra), launch_log)
    try:
        time.sleep(duration)
    finally:
        stop(proc, out)
    csvs = sorted(glob.glob(str(log_dir / "twin_*_ci.csv")))
    assert csvs, "El puente no escribió ningún CSV:\n" + launch_log.read_text()[-3000:]
    return analyze_log.analyze(csvs[-1], str(tmp_path), plots=False), launch_log


SYNC_CASES = [
    ("real_fb", "real", "true", 61, 62),
    ("real_mirror", "real", "false", 63, 64),
    ("twin_fb", "twin", "true", 65, 66),
]


@pytest.mark.parametrize("case_id,leader,feedback,rd,td", SYNC_CASES)
def test_sync(tmp_path, case_id, leader, feedback, rd, td):
    s, _ = run_launch(tmp_path, rd, td, 40,
                      f"leader:={leader}", f"feedback:={feedback}",
                      "driver:=square", "driver_start_delay:=3.0",
                      "linear_speed:=0.3", "angular_speed:=1.0", "distance:=0.6",
                      f"arena:={ARENA}")
    e = s["error_posicion_m"]
    lidar = s.get("lidar", {})
    report(f"{case_id}: lider={leader} realim={feedback} dur={s['duracion_s']:.1f}s "
           f"err_medio={e.get('media', float('nan')) * 100:.1f}cm "
           f"max={e.get('max', float('nan')) * 100:.1f}cm "
           f"final={s['error_final_m'] * 100:.1f}cm "
           f"rtt={s['rtt_ms'].get('media', float('nan')):.1f}ms "
           f"retardo={s['retardo_sincronizacion_ms']:.0f}ms n={s['muestras']} "
           f"lidar_mae={lidar.get('mae_m', float('nan')) * 100:.1f}cm "
           f"lidar_n={lidar.get('comparaciones', 0)} anomalias={s.get('anomalias', {})}")
    assert s["muestras"] > 200
    assert e["n"] > 100, "Nunca hubo datos de ambos robots"
    assert s["rtt_ms"]["n"] > 10, "No se midió la latencia"
    assert not SERIOUS & set(s.get("anomalias", {})), f"Anomalías en una corrida normal: {s['anomalias']}"
    if feedback == "true":
        assert e["max"] < 0.10
        assert s["error_final_m"] < 0.05
        assert lidar.get("comparaciones", 0) > 5, "No se compararon los LiDAR"
        assert lidar["mae_m"] < 0.05
        assert lidar["coincidencia_visibilidad"] > 0.9
    else:
        assert s["error_final_m"] > 0.10


FAULT_CASES = [
    ("atasco", "stall", "square", "2.5", "2.0", 71, 72),
    ("deslizamiento", "slip", "rotate", "1.0", "3.0", 73, 74),
    ("movimiento_no_comandado", "push", "line", "4.0", "3.0", 75, 76),
]


@pytest.mark.parametrize("kind,fault,pattern,fault_start,fault_duration,rd,td", FAULT_CASES)
def test_anomaly_detection(tmp_path, kind, fault, pattern, fault_start, fault_duration, rd, td):
    s, launch_log = run_launch(tmp_path, rd, td, 25,
                               "leader:=real", "driver_start_delay:=3.0", f"driver:={pattern}",
                               "linear_speed:=0.3", "angular_speed:=1.0", "distance:=0.6",
                               f"fake_real_fault:={fault}", f"fake_real_fault_start:={fault_start}",
                               f"fake_real_fault_duration:={fault_duration}")
    found = s.get("anomalias", {})
    report(f"falla {fault}: anomalias detectadas={found}")
    assert kind in found, f"No se detectó {kind}: {found}\n" + launch_log.read_text()[-2000:]
    others = SERIOUS - {kind, "perdida_datos"}
    assert not others & set(found), f"Falsas alarmas: {found}"


def test_experiment_protocol(tmp_path):
    rd, td = 81, 82
    out_dir = tmp_path / "resultados"
    proc, out = start(twin_launch(tmp_path / "logs", rd, td,
                                  "fake_real_gain:=0.85", "fake_real_tau:=0.3", "fake_real_delay:=0.1",
                                  "fake_twin_gain:=1.0", "fake_twin_tau:=0.05"),
                      tmp_path / "launch.log")
    try:
        time.sleep(6)
        runner = subprocess.run(
            ["ros2", "run", "bumperbot_digital_twin", "twin_experiment", "protocolo_ci", "--auto",
             "--real-domain", str(rd), "--twin-domain", str(td), "--out", str(out_dir), "--no-plots"],
            capture_output=True, text=True, timeout=900)
    finally:
        stop(proc, out)
    (tmp_path / "runner.log").write_text(runner.stdout + runner.stderr)
    sizes = {os.path.basename(p)[16:]: sum(1 for _ in open(p)) - 1
             for p in sorted(glob.glob(str(tmp_path / "logs" / "twin_*.csv")))
             if not p.endswith(("_scan.csv", "_eventos.csv"))}
    report(f"protocolo_ci filas por registro: {sizes}")
    assert runner.returncode == 0, runner.stdout[-3000:] + runner.stderr[-3000:]
    data = json.load(open(out_dir / "informe.json"))
    table = data["experimentos"]
    assert (out_dir / "modelo_gemelo.yaml").exists()
    for exp in ("M", "MC", "D", "DC", "R", "RC", "RT"):
        assert table[exp]["rmse"][2] == 2, f"{exp}: faltan corridas"
    assert (out_dir / "ganancias.yaml").exists()

    def m(exp, key="rmse"):
        return table[exp][key][0]

    report("protocolo_ci: " + " ".join(f"{e}={m(e) * 100:.1f}cm" for e in ("M", "MC", "D", "DC", "R", "RC", "RT"))
           + " · real R=" + f"{m('R', 'rmse_real') * 100:.1f}cm RC={m('RC', 'rmse_real') * 100:.1f}cm"
           + f" RT={m('RT', 'rmse_real') * 100:.1f}cm · ganancias RT={table['RT']['ganancias']}")
    report("comparaciones: " + "; ".join(f"{c['a']} vs {c['b']} p={c['p']:.3f}" for c in data["comparaciones"]))
    report("modelo identificado: " + (out_dir / "modelo_gemelo.yaml").read_text().replace("\n", " "))
    assert m("MC") < 0.6 * m("M"), "El gemelo calibrado no redujo el error"
    assert m("DC") < m("D"), "La compensación no redujo el error con 200 ms"
    assert m("RC", "rmse_real") < m("R", "rmse_real"), "El predictor de Smith no redujo el error"
    assert m("RT", "rmse_real") < m("R", "rmse_real"), "Las ganancias sintonizadas no redujeron el error"


def test_navigation_preview(tmp_path):
    rd, td = 91, 92
    out_dir = tmp_path / "navegacion"
    proc, out = start(twin_launch(tmp_path / "logs", rd, td, "arena:=[-0.5,2.0,-1.0,1.0]"), tmp_path / "launch.log")
    try:
        time.sleep(6)
        nav = subprocess.run(
            ["ros2", "run", "bumperbot_digital_twin", "twin_navigate", "--goals", "metas_arena",
             "--real-domain", str(rd), "--twin-domain", str(td), "--out", str(out_dir)],
            capture_output=True, text=True, timeout=900)
    finally:
        stop(proc, out)
    (tmp_path / "runner.log").write_text(nav.stdout + nav.stderr)
    assert nav.returncode == 0, nav.stdout[-3000:] + nav.stderr[-3000:]
    results = {r["meta"]: r for r in json.load(open(out_dir / "informe_navegacion.json"))}
    for name, r in results.items():
        c = r.get("comparacion", {})
        report(f"navegacion {name}: aprobada={r['aprobada']} motivos={r['motivos']} "
               f"t_gemelo={r['gemelo']['duracion_s']:.1f}s t_real={(r['real'] or {}).get('duracion_s', float('nan')):.1f}s "
               f"desv_media={c.get('desviacion_media_m', float('nan')) * 100:.1f}cm "
               f"desv_max={c.get('desviacion_max_m', float('nan')) * 100:.1f}cm "
               f"err_final_real={(r['real'] or {}).get('error_final_m', float('nan')) * 100:.1f}cm")
    for name in ("N1", "N2", "N3", "N4"):
        r = results[name]
        assert r["aprobada"] and r["ejecutada"], (name, r["motivos"])
        assert r["real"]["lograda"], name
        assert r["real"]["error_final_m"] < 0.1, name
        assert r["comparacion"]["desviacion_media_m"] < 0.05, name
    for name, text in (("X1", "obstáculo"), ("X2", "no llegó")):
        r = results[name]
        assert not r["aprobada"] and not r["ejecutada"], name
        assert any(text in m for m in r["motivos"]), (name, r["motivos"])
    x1, x2 = results["X1"]["inicio_real"], results["X2"]["inicio_real"]
    assert math.hypot(x1[0] - x2[0], x1[1] - x2[1]) < 0.02, "El robot real se movió con una meta rechazada"
    assert (out_dir / "vista_previa_N1.png").stat().st_size > 10000
    assert "N1" in (out_dir / "informe_navegacion.md").read_text()
