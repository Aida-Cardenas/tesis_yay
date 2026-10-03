"""Interfaz gráfica del panel de control del gemelo digital (sin dependencias de ROS).

La ventana recibe datos con update_status / add_event / update_scan y llama a los
métodos de un objeto `actions` cuando se pulsan los controles. Así se puede probar
sin ROS (modo --selftest) y la parte de ROS queda en dashboard.py.
"""
import math
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

PATTERNS = [("Línea recta", "line"), ("Ida y vuelta", "line_back"), ("Cuadrado", "square"),
            ("Círculo", "circle"), ("Giro en el sitio", "rotate"), ("Ocho", "figure8")]
GOOD, WARN, BAD, MUTED = "#2e7d32", "#b26a00", "#c62828", "#757575"


def fmt(value, scale=1.0, digits=1, unit=""):
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "—"
    return f"{value * scale:.{digits}f}{unit}"


class Badge(QtWidgets.QLabel):
    def __init__(self, text=""):
        super().__init__(text)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumWidth(130)
        self.set_state(text, MUTED)

    def set_state(self, text, color):
        self.setText(text)
        self.setStyleSheet(f"QLabel {{ color: white; background: {color}; border-radius: 9px; "
                           f"padding: 4px 12px; font-weight: 600; }}")


class Metric(QtWidgets.QFrame):
    def __init__(self, title):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet(f"color: {MUTED}; font-size: 11px;")
        self.title.setWordWrap(True)
        self.value = QtWidgets.QLabel("—")
        self.value.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set(self, text, color=None):
        self.value.setText(text)
        self.value.setStyleSheet(f"font-size: 18px; font-weight: 600; color: {color or 'palette(text)'};")


class DashboardWindow(QtWidgets.QMainWindow):
    def __init__(self, actions, history_s=60.0):
        super().__init__()
        self.actions = actions
        self.history_s = history_s
        self.t0 = time.monotonic()
        self.err_hist = deque()
        self.rtt_hist = deque()
        self.last_status = {}
        self._updating_flags = False
        self.setWindowTitle("Gemelo digital · panel de control")
        self.resize(1320, 900)

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Gemelo digital del robot")
        title.setStyleSheet("font-size: 20px; font-weight: 700;")
        header.addWidget(title)
        header.addStretch(1)
        self.leader_badge = Badge("Líder: —")
        self.sync_badge = Badge("Sin datos")
        header.addWidget(self.leader_badge)
        header.addWidget(self.sync_badge)
        outer.addLayout(header)

        body = QtWidgets.QHBoxLayout()
        outer.addLayout(body, 1)

        metrics = QtWidgets.QGridLayout()
        self.m = {}
        for i, (key, title) in enumerate([
            ("pos", "Error de posición"), ("yaw", "Error de orientación"),
            ("rtt", "Latencia ida y vuelta"), ("delay", "Retardo estimado (un sentido)"),
            ("cmd", "Comando al seguidor [m/s · rad/s]"), ("net", "Red emulada"),
            ("lidar", "LiDAR real vs. simulado [MAE · coincidencia]"), ("anom", "Anomalías activas"),
        ]):
            self.m[key] = Metric(title)
            metrics.addWidget(self.m[key], i // 2, i % 2)
        left = QtWidgets.QWidget()
        left.setLayout(metrics)
        left.setFixedWidth(420)
        body.addWidget(left)

        self.figure = Figure(figsize=(6, 5), tight_layout=True)
        self.ax_err = self.figure.add_subplot(211)
        self.ax_rtt = self.figure.add_subplot(212, sharex=self.ax_err)
        self.canvas = FigureCanvasQTAgg(self.figure)
        body.addWidget(self.canvas, 1)

        body.addWidget(self._controls())

        self.events = QtWidgets.QTableWidget(0, 5)
        self.events.setHorizontalHeaderLabels(["Hora", "Origen", "Anomalía", "Fase", "Descripción"])
        self.events.horizontalHeader().setStretchLastSection(True)
        self.events.verticalHeader().setVisible(False)
        self.events.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.events.setMinimumHeight(190)
        self.events.setMaximumHeight(220)
        outer.addWidget(self.events)
        self.statusBar().showMessage("Esperando datos del puente…")

    # ------------------------------------------------------------------ controls
    def _controls(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(330)
        v = QtWidgets.QVBoxLayout(panel)

        g = QtWidgets.QGroupBox("Sincronización")
        gl = QtWidgets.QVBoxLayout(g)
        self.btn_leader = QtWidgets.QPushButton("Cambiar líder")
        self.btn_align = QtWidgets.QPushButton("Realinear robots")
        self.chk_feedback = QtWidgets.QCheckBox("Corrección por odometría")
        self.chk_comp = QtWidgets.QCheckBox("Compensación de latencia")
        self.chk_model = QtWidgets.QCheckBox("Gemelo calibrado")
        for w in (self.btn_leader, self.btn_align, self.chk_feedback, self.chk_comp, self.chk_model):
            gl.addWidget(w)
        self.btn_leader.clicked.connect(lambda: self.actions.switch_leader())
        self.btn_align.clicked.connect(lambda: self.actions.align())
        for chk in (self.chk_feedback, self.chk_comp, self.chk_model):
            chk.toggled.connect(self._flags_changed)
        v.addWidget(g)

        g = QtWidgets.QGroupBox("Red emulada (enlace con el robot)")
        gl = QtWidgets.QFormLayout(g)
        self.sp_delay = QtWidgets.QSpinBox()
        self.sp_delay.setRange(0, 2000)
        self.sp_delay.setSuffix(" ms")
        self.sp_jitter = QtWidgets.QSpinBox()
        self.sp_jitter.setRange(0, 500)
        self.sp_jitter.setSuffix(" ms")
        self.sp_loss = QtWidgets.QDoubleSpinBox()
        self.sp_loss.setRange(0, 90)
        self.sp_loss.setSuffix(" %")
        gl.addRow("Retardo", self.sp_delay)
        gl.addRow("Variación", self.sp_jitter)
        gl.addRow("Pérdida", self.sp_loss)
        btn = QtWidgets.QPushButton("Aplicar red")
        btn.clicked.connect(lambda: self.actions.set_network(self.sp_delay.value(), self.sp_jitter.value(),
                                                             self.sp_loss.value() / 100.0))
        gl.addRow(btn)
        v.addWidget(g)

        g = QtWidgets.QGroupBox("Recorrido automático del líder")
        gl = QtWidgets.QFormLayout(g)
        self.cb_pattern = QtWidgets.QComboBox()
        for label, key in PATTERNS:
            self.cb_pattern.addItem(label, key)
        self.cb_pattern.setCurrentIndex(2)
        self.sp_speed = QtWidgets.QDoubleSpinBox()
        self.sp_speed.setRange(0.05, 0.5)
        self.sp_speed.setSingleStep(0.05)
        self.sp_speed.setValue(0.15)
        self.sp_speed.setSuffix(" m/s")
        self.sp_dist = QtWidgets.QDoubleSpinBox()
        self.sp_dist.setRange(0.2, 5.0)
        self.sp_dist.setValue(1.0)
        self.sp_dist.setSuffix(" m")
        gl.addRow("Figura", self.cb_pattern)
        gl.addRow("Velocidad", self.sp_speed)
        gl.addRow("Lado / distancia", self.sp_dist)
        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Iniciar")
        b2 = QtWidgets.QPushButton("Detener")
        b1.clicked.connect(lambda: self.actions.start_trajectory(self.cb_pattern.currentData(),
                                                                 self.sp_speed.value(), self.sp_dist.value()))
        b2.clicked.connect(lambda: self.actions.stop_trajectory())
        row.addWidget(b1)
        row.addWidget(b2)
        gl.addRow(row)
        v.addWidget(g)

        g = QtWidgets.QGroupBox("Registro")
        gl = QtWidgets.QFormLayout(g)
        self.ed_tag = QtWidgets.QLineEdit("prueba")
        btn = QtWidgets.QPushButton("Nuevo registro")
        btn.clicked.connect(lambda: self.actions.new_log(self.ed_tag.text().strip()))
        gl.addRow("Etiqueta", self.ed_tag)
        gl.addRow(btn)
        v.addWidget(g)
        v.addStretch(1)
        return panel

    def _flags_changed(self):
        if not self._updating_flags:
            self.actions.set_flags(self.chk_feedback.isChecked(), self.chk_comp.isChecked(),
                                   self.chk_model.isChecked())

    # ------------------------------------------------------------------ data
    @QtCore.pyqtSlot(str)
    def show_message(self, text):
        self.statusBar().showMessage(text, 8000)

    def update_status(self, s):
        self.last_status = s
        now = time.monotonic() - self.t0
        leader = s.get("leader", "")
        self.leader_badge.set_state(f"Líder: {'robot real' if leader == 'real' else 'gemelo'}", "#37474f")
        pos = s.get("position_error", math.nan)
        if s.get("stale"):
            self.sync_badge.set_state("Datos atrasados", BAD)
        elif s.get("lost_sync"):
            self.sync_badge.set_state("Desincronizado", BAD)
        elif math.isfinite(pos) and pos < 0.05:
            self.sync_badge.set_state("Sincronizado", GOOD)
        else:
            self.sync_badge.set_state("Corrigiendo", WARN)
        color = GOOD if math.isfinite(pos) and pos < 0.03 else WARN if math.isfinite(pos) and pos < 0.10 else BAD
        self.m["pos"].set(fmt(pos, 100, 1, " cm"), color if math.isfinite(pos) else None)
        self.m["yaw"].set(fmt(abs(s.get("heading_error", math.nan)), 180 / math.pi, 1, "°"))
        rtt = s.get("rtt_ms", math.nan)
        self.m["rtt"].set(fmt(rtt, 1, 0, " ms"), GOOD if math.isfinite(rtt) and rtt < 100 else WARN)
        self.m["delay"].set(fmt(s.get("one_way_delay_ms", math.nan), 1, 0, " ms"))
        self.m["cmd"].set(f"{fmt(s.get('cmd_v', math.nan), 1, 2)} · {fmt(s.get('cmd_w', math.nan), 1, 2)}")
        net = s.get("net_delay_ms", 0.0)
        loss = s.get("net_loss", 0.0)
        self.m["net"].set("Sin degradar" if not net and not loss else f"{net:.0f} ms · {loss * 100:.0f} %")
        anomalies = s.get("anomalies", "")
        self.m["anom"].set(anomalies.replace("|", ", ").replace("_", " ") or "Ninguna", BAD if anomalies else GOOD)
        self._updating_flags = True
        self.chk_feedback.setChecked(bool(s.get("feedback")))
        self.chk_comp.setChecked(bool(s.get("compensation")))
        self.chk_model.setChecked(bool(s.get("twin_model")))
        self._updating_flags = False
        if math.isfinite(pos):
            self.err_hist.append((now, pos * 100))
        if math.isfinite(rtt):
            self.rtt_hist.append((now, rtt))
        for h in (self.err_hist, self.rtt_hist):
            while h and h[0][0] < now - self.history_s:
                h.popleft()

    def update_scan(self, sc):
        self.m["lidar"].set(f"{fmt(sc.get('mae'), 100, 1)} cm · {fmt(sc.get('visibility_agreement'), 100, 0)} %")

    def add_event(self, e):
        self.events.insertRow(0)
        stamp = time.strftime("%H:%M:%S", time.localtime(e.get("t", time.time())))
        values = [stamp, e.get("source", ""), e.get("kind", "").replace("_", " "), e.get("phase", ""),
                  e.get("description", "")]
        color = QtGui.QColor(BAD if e.get("severity", 1) >= 2 else WARN) if e.get("phase") == "inicio" \
            else QtGui.QColor(MUTED)
        for col, value in enumerate(values):
            item = QtWidgets.QTableWidgetItem(value)
            item.setForeground(color)
            self.events.setItem(0, col, item)
        while self.events.rowCount() > 50:
            self.events.removeRow(self.events.rowCount() - 1)

    def redraw(self):
        for ax, hist, label, color in ((self.ax_err, self.err_hist, "Error de posición [cm]", "#1565c0"),
                                       (self.ax_rtt, self.rtt_hist, "Latencia ida y vuelta [ms]", "#2e7d32")):
            ax.clear()
            if hist:
                ax.plot([h[0] for h in hist], [h[1] for h in hist], color=color, lw=1.4)
            ax.set_ylabel(label, fontsize=9)
            ax.grid(alpha=0.3)
        self.ax_rtt.set_xlabel("Tiempo [s]", fontsize=9)
        self.canvas.draw_idle()
