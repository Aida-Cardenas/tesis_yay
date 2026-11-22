"""Pruebas de probar_arduino.py contra un Arduino de mentira en un pty."""
import io
import os
import pty
import sys
import threading
import time
import tty
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import probar_arduino as pa  # noqa: E402


class FakeArduino:
    """Habla el protocolo de robot_control.ino: una línea cada 100 ms."""

    def __init__(self, top_speed=10.0, right_sign=1, fault=None):
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        self.slave = slave
        self.top_speed = top_speed
        self.right_sign = right_sign
        self.fault = fault
        self.setpoint = {"r": 0.0, "l": 0.0}
        self.count = {"r": 65000, "l": 10}
        self.commands = []
        self.stop_event = threading.Event()
        self.buffer = ""
        self.t = 0
        threading.Thread(target=self._run, daemon=True).start()

    def _read_commands(self):
        import select
        while select.select([self.master], [], [], 0)[0]:
            self.buffer += os.read(self.master, 1024).decode()
        while "," in self.buffer:
            field, self.buffer = self.buffer.split(",", 1)
            if len(field) == 7 and field[0] in "rl":
                value = float(field[2:])
                self.setpoint[field[0]] = value if field[1] == "p" else -value
                self.commands.append(field)

    def _run(self):
        while not self.stop_event.is_set():
            self._read_commands()
            fields = []
            for wheel in ("r", "l"):
                speed = max(-self.top_speed, min(self.top_speed, self.setpoint[wheel]))
                if wheel == "r":
                    speed *= self.right_sign
                self.count[wheel] = (self.count[wheel] + int(speed * 12.3)) % 65536
                fields.append(f"{wheel}{'p' if speed >= 0 else 'n'}{abs(speed):.2f}")
            line = ",".join(fields) + f",rc{self.count['r']},lc{self.count['l']},t{self.t},"
            if self.fault and any(self.setpoint.values()):
                line += f"f{self.fault},"
            os.write(self.master, (line + "\r\n").encode())
            self.t += 100
            time.sleep(0.1)

    def close(self):
        self.stop_event.set()
        time.sleep(0.15)
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def fake():
    pytest.importorskip("serial")
    made = []

    def make(**kwargs):
        made.append(FakeArduino(**kwargs))
        return made[-1]
    yield make
    for f in made:
        f.close()


def test_parse_line_reads_signed_speeds_counters_and_faults():
    r = pa.parse_line("rp4.55,ln03.10,rc100,lc65535,t812,fr,\r\n")
    assert r["speed"] == {"r": 4.55, "l": -3.1}
    assert r["count"] == {"r": 100, "l": 65535}
    assert r["t"] == 812
    assert r["fault"] == {"r"}
    assert pa.parse_line("garbage") is None
    assert pa.parse_line("rp1.00,") is None


def test_format_command_matches_the_firmware_buffer():
    assert pa.format_command(5.2, -3.1) == "rp05.20,ln03.10,"
    assert pa.format_command(0.0, 150.0) == "rp00.00,lp99.99,"


def test_count_tracker_unwraps_the_16_bit_counter():
    t = pa.CountTracker()
    t.update({"r": 65530, "l": 5})
    t.update({"r": 4, "l": 65533})
    assert t.total == {"r": 10, "l": -8}


def test_hola(fake, capsys):
    f = fake()
    assert pa.main(["--puerto", f.port, "--espera", "0.2", "hola"]) == 0
    assert "responde" in capsys.readouterr().out


def test_girar_reports_both_wheels_and_stops_them(fake, capsys):
    f = fake()
    assert pa.main(["--puerto", f.port, "--espera", "0.2", "girar", "--segundos", "1"]) == 0
    out = capsys.readouterr().out
    assert "Rueda derecha: 3.00" in out and "Rueda izquierda: 3.00" in out
    assert f.commands[-2:] == ["rp00.00", "lp00.00"]


def test_girar_detects_an_encoder_counting_backwards(fake, capsys):
    f = fake(right_sign=-1)
    pa.main(["--puerto", f.port, "--espera", "0.2", "girar", "--segundos", "1"])
    assert "derecha midió hacia atrás" in capsys.readouterr().out


def test_girar_stops_on_a_fault(fake, capsys):
    f = fake(fault="l")
    assert pa.main(["--puerto", f.port, "--espera", "0.2", "girar", "--segundos", "3"]) == 1
    assert "FALLA en la rueda izquierda" in capsys.readouterr().out


def test_maxima_converts_to_meters_per_second(fake, capsys):
    f = fake(top_speed=8.0)
    pa.main(["--puerto", f.port, "--espera", "0.2", "maxima", "--segundos", "1.5"])
    out = capsys.readouterr().out
    assert "0.26 m/s" in out
    assert "No llega a 0.3" in out


def test_contar_divides_by_the_turns(fake, capsys, monkeypatch):
    f = fake()
    f.setpoint = {"r": 5.0, "l": 5.0}
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    real_thread = threading.Thread

    class LateEnter(real_thread):
        def __init__(self, target=None, daemon=None):
            super().__init__(target=lambda: (time.sleep(1.0), target()), daemon=daemon)
    monkeypatch.setattr(pa.threading, "Thread", LateEnter)
    assert pa.main(["--puerto", f.port, "--espera", "0.2", "contar", "--vueltas", "2"]) == 0
    out = capsys.readouterr().out
    assert "Promedio:" in out
