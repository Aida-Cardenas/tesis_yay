#!/usr/bin/env python3
"""Panel de control del gemelo digital.

    ros2 run bumperbot_digital_twin twin_dashboard
    ros2 run bumperbot_digital_twin twin_dashboard --selftest captura.png

Muestra en vivo el estado de la sincronización (error, latencia, líder, red,
LiDAR, anomalías) y permite manejar el puente sin terminal: cambiar el líder,
realinear, activar la corrección, la compensación de latencia y el gemelo
calibrado, degradar la red, abrir registros y lanzar recorridos automáticos.
"""
import argparse
import math
import os
import random
import sys
import threading
import time

from PyQt5 import QtCore, QtWidgets

from bumperbot_digital_twin.dashboard_ui import DashboardWindow


class RosActions:
    """Conecta los botones con el puente (servicio /digital_twin/configure)."""

    def __init__(self, args, window_ref):
        from bumperbot_msgs.msg import ScanComparison, TwinAnomaly, TwinSyncStatus
        from bumperbot_msgs.srv import TwinConfigure
        from std_srvs.srv import Trigger
        from bumperbot_digital_twin.ros_util import DualDomain, TrajectoryPlayer

        self.TwinConfigure = TwinConfigure
        self.Trigger = Trigger
        self.window_ref = window_ref
        self.dual = DualDomain("twin_dashboard", args.real_domain, args.twin_domain,
                               twin_sim_time=args.twin_sim_time)
        node = self.dual.nodes["twin"]
        self.lock = threading.Lock()
        self.status = None
        self.scan = None
        self.events = []
        node.create_subscription(TwinSyncStatus, "/digital_twin/status", self._status_cb, 10)
        node.create_subscription(TwinAnomaly, "/digital_twin/events", self._event_cb, 50)
        node.create_subscription(ScanComparison, "/digital_twin/scan_comparison", self._scan_cb, 10)
        self.configure_client = node.create_client(TwinConfigure, "/digital_twin/configure")
        self.switch_client = node.create_client(Trigger, "/digital_twin/switch_leader")
        self.player = TrajectoryPlayer(self.dual)
        self.preview = None
        self.nav_busy = False

    def _status_cb(self, msg):
        s = {
            "leader": msg.leader, "stale": msg.stale, "lost_sync": msg.lost_sync, "active": msg.active,
            "position_error": msg.position_error, "heading_error": msg.heading_error,
            "rtt_ms": msg.rtt_ms, "one_way_delay_ms": msg.one_way_delay_ms,
            "cmd_v": msg.command.linear.x, "cmd_w": msg.command.angular.z,
            "net_delay_ms": msg.net_delay_ms, "net_loss": msg.net_loss, "anomalies": msg.anomalies,
            "feedback": msg.feedback, "compensation": msg.compensation, "twin_model": msg.twin_model,
            "sync": msg.sync, "kx": msg.kx, "ky": msg.ky, "ktheta": msg.ktheta,
        }
        with self.lock:
            self.status = s

    def _event_cb(self, msg):
        with self.lock:
            self.events.append({"t": msg.stamp.sec + msg.stamp.nanosec * 1e-9, "source": msg.source,
                                "kind": msg.kind, "phase": msg.phase, "severity": msg.severity,
                                "description": msg.description})

    def _scan_cb(self, msg):
        with self.lock:
            self.scan = {"mae": msg.mae, "visibility_agreement": msg.visibility_agreement}

    def poll(self):
        with self.lock:
            status, scan, events = self.status, self.scan, self.events
            self.status, self.scan, self.events = None, None, []
        return status, scan, events

    def _async(self, fn):
        def run():
            try:
                message = fn()
            except Exception as exc:
                message = f"Error: {exc}"
            win = self.window_ref()
            if win is not None:
                QtCore.QMetaObject.invokeMethod(win, "show_message", QtCore.Qt.QueuedConnection,
                                                QtCore.Q_ARG(str, message))
        threading.Thread(target=run, daemon=True).start()

    def _configure(self, **kw):
        req = self.TwinConfigure.Request()
        req.leader = kw.get("leader", "")
        req.feedback = req.compensation = req.twin_model = -1
        req.net_delay_ms = req.net_jitter_ms = req.net_loss = -1.0
        for k, v in kw.items():
            setattr(req, k, v)
        res = self.dual.call("twin", self.configure_client, req, timeout=15.0)
        if not res.success:
            raise RuntimeError(res.message)
        return res

    def set_gains(self, kx, ky, ktheta):
        self._async(lambda: self._configure(kx=float(kx), ky=float(ky), ktheta=float(ktheta)).message)

    def _nav_text(self, text):
        win = self.window_ref()
        if win is not None:
            QtCore.QMetaObject.invokeMethod(win, "show_nav", QtCore.Qt.QueuedConnection, QtCore.Q_ARG(str, text))

    def navigate(self, x, y, yaw_deg, execute):
        if self.nav_busy:
            self.window_ref().show_message("Ya hay una navegación en curso")
            return
        from bumperbot_digital_twin.nav_preview import NavPreview

        def run():
            self.nav_busy = True
            try:
                if self.preview is None:
                    self.preview = NavPreview(self.dual, self._configure, log=self._nav_text)
                    time.sleep(1.0)
                r = self.preview.run("panel", x, y, math.radians(yaw_deg), "auto" if execute else "never",
                                     out_dir=os.path.expanduser("~/twin_resultados/navegacion_panel"))
                text = "Aprobada" if r["aprobada"] else "Rechazada: " + "; ".join(r["motivos"])
                if "comparacion" in r:
                    c = r["comparacion"]
                    text += (f". Ruta real vs prevista: desviación media {c['desviacion_media_m'] * 100:.1f} cm, "
                             f"máxima {c['desviacion_max_m'] * 100:.1f} cm")
                self._nav_text(text)
            except Exception as exc:
                self._nav_text(f"Error: {exc}")
            finally:
                self.nav_busy = False

        threading.Thread(target=run, daemon=True).start()

    def switch_leader(self):
        self._async(lambda: self.dual.call("twin", self.switch_client, self.Trigger.Request()).message)

    def align(self):
        self._async(lambda: self._configure(align=True, resume_sync=True).message)

    def set_flags(self, feedback, compensation, twin_model):
        self._async(lambda: self._configure(feedback=int(feedback), compensation=int(compensation),
                                            twin_model=int(twin_model)).message)

    def set_network(self, delay, jitter, loss):
        self._async(lambda: self._configure(net_delay_ms=float(delay), net_jitter_ms=float(jitter),
                                            net_loss=float(loss)).message)

    def new_log(self, tag):
        self._async(lambda: self._configure(new_log=True, log_tag=tag).message)

    def start_trajectory(self, pattern, speed, distance):
        leader = (self.window_ref().last_status or {}).get("leader", "real")
        total = self.player.play(leader, pattern, linear_speed=speed, distance=distance, blocking=False)
        self.window_ref().show_message(f"Recorrido '{pattern}' en el {leader} ({total:.0f} s)")

    def stop_trajectory(self):
        self.player.stop()

    def shutdown(self):
        self.player.stop()
        self.dual.shutdown()


class FakeActions:
    """Acciones de prueba: registran las llamadas sin ROS."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *a, **k: self.calls.append((name, a, k))


def selftest(path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    actions = FakeActions()
    win = DashboardWindow(actions)
    win.t0 = time.monotonic() - 60
    rng = random.Random(3)
    for i in range(240):
        win.update_status({
            "leader": "real", "stale": False, "lost_sync": False, "active": True,
            "position_error": 0.012 + 0.008 * math.sin(i / 10) + rng.gauss(0, 0.002),
            "heading_error": 0.02, "rtt_ms": 18 + rng.gauss(0, 3), "one_way_delay_ms": 9.0,
            "cmd_v": 0.15, "cmd_w": 0.3, "net_delay_ms": 0.0, "net_loss": 0.0, "anomalies": "",
            "feedback": True, "compensation": True, "twin_model": False,
            "sync": True, "kx": 1.5, "ky": 6.0, "ktheta": 3.0,
        })
        win.err_hist[-1] = (i * 0.25, win.err_hist[-1][1])
        win.rtt_hist[-1] = (i * 0.25, win.rtt_hist[-1][1])
    win.update_scan({"mae": 0.021, "visibility_agreement": 0.97})
    win.add_event({"source": "detector", "kind": "atasco", "phase": "inicio", "severity": 2,
                   "description": "Se ordena movimiento pero el robot no se mueve"})
    win.add_event({"source": "detector", "kind": "atasco", "phase": "fin", "severity": 2,
                   "description": "Se ordena movimiento pero el robot no se mueve"})
    win.redraw()
    win.show()
    app.processEvents()
    win.btn_align.click()
    win.chk_comp.setChecked(False)
    win.btn_nav.click()
    win.show_nav("Aprobada. Ruta real vs prevista: desviación media 1.2 cm, máxima 3.4 cm")
    app.processEvents()
    names = [c[0] for c in actions.calls]
    ok = all(n in names for n in ("align", "set_flags", "navigate"))
    win.grab().save(path)
    print(f"Captura en {path}; acciones registradas: {[c[0] for c in actions.calls]}")
    return 0 if ok and os.path.exists(path) else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--real-domain", type=int, default=int(os.environ.get("REAL_DOMAIN_ID", "10")))
    parser.add_argument("--twin-domain", type=int, default=int(os.environ.get("TWIN_DOMAIN_ID", "20")))
    parser.add_argument("--twin-sim-time", action="store_true",
                        help="El gemelo corre en Gazebo (usa el reloj de simulación)")
    parser.add_argument("--selftest", metavar="PNG", help="Dibuja el panel con datos de prueba y guarda una captura")
    args, _ = parser.parse_known_args(argv)
    if args.selftest:
        return selftest(args.selftest)

    app = QtWidgets.QApplication(sys.argv)
    actions = RosActions(args, lambda: None)
    win = DashboardWindow(actions)
    actions.window_ref = lambda: win

    def refresh():
        status, scan, events = actions.poll()
        if status:
            win.update_status(status)
        if scan:
            win.update_scan(scan)
        for e in events:
            win.add_event(e)
        if not actions.dual.ok():
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(100)
    plot_timer = QtCore.QTimer()
    plot_timer.timeout.connect(win.redraw)
    plot_timer.start(500)
    win.show()
    try:
        code = app.exec_()
    finally:
        actions.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
