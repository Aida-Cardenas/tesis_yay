#!/usr/bin/env python3
"""Pruebas del Arduino y los motores sin ROS, con las ruedas en el aire.

    python3 probar_arduino.py hola                    el Arduino responde
    python3 probar_arduino.py contar                  cuentas del encoder por vuelta
    python3 probar_arduino.py girar                   sentido de giro de las ruedas
    python3 probar_arduino.py maxima                  velocidad máxima de las ruedas

Por defecto usa /dev/arduino (las reglas udev); con --puerto se elige otro
(/dev/ttyUSB0 en Linux, COM3 en Windows). real_robot.launch.py y el monitor serie
del Arduino IDE tienen que estar cerrados: dos programas no pueden usar el puerto.
"""
import argparse
import sys
import threading
import time

WHEEL_RADIUS = 0.0325
TARGET_SPEED = 0.3
COUNTER_MOD = 65536
NAMES = {"r": "derecha", "l": "izquierda"}


def parse_line(line):
    """Una línea del Arduino: velocidades con signo (rad/s), contadores, millis y fallas."""
    out = {"speed": {}, "count": {}, "t": None, "fault": set()}
    for field in line.strip().split(","):
        if len(field) < 2:
            continue
        head, rest = field[0], field[1:]
        try:
            if head in "rl" and rest[0] in "pn":
                value = float(rest[1:])
                out["speed"][head] = value if rest[0] == "p" else -value
            elif head in "rl" and rest[0] == "c":
                out["count"][head] = int(rest[1:])
            elif head == "t":
                out["t"] = int(rest)
            elif head == "f" and rest in ("r", "l"):
                out["fault"].add(rest)
        except (ValueError, IndexError):
            continue
    if set(out["speed"]) != {"r", "l"}:
        return None
    return out


def format_command(right, left):
    """Velocidades en rad/s con signo, como las manda la Pi: "rp05.20,ln03.10,"."""
    def one(wheel, value):
        value = max(-99.99, min(99.99, value))
        return f"{wheel}{'p' if value >= 0 else 'n'}{abs(value):05.2f},"
    return one("r", right) + one("l", left)


class CountTracker:
    """Suma las diferencias de los contadores de 16 bits (dan la vuelta en 65535)."""

    def __init__(self):
        self.last = {}
        self.total = {"r": 0, "l": 0}

    def update(self, counts):
        for wheel, value in counts.items():
            if wheel in self.last:
                delta = (value - self.last[wheel]) % COUNTER_MOD
                if delta >= COUNTER_MOD // 2:
                    delta -= COUNTER_MOD
                self.total[wheel] += delta
            self.last[wheel] = value

    def reset(self):
        self.total = {"r": 0, "l": 0}


class Arduino:
    def __init__(self, port, reset_wait):
        import serial
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.3)
        except serial.SerialException as exc:
            sys.exit(f"No se pudo abrir {port}: {exc}\n"
                     "¿Está conectado el Arduino? ¿Está cerrado real_robot.launch.py y el monitor serie?")
        time.sleep(reset_wait)
        self.ser.reset_input_buffer()
        self.buffer = b""

    def read(self, timeout=1.0):
        """La próxima línea completa y válida, o None si no llega ninguna a tiempo."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            self.buffer += chunk
            while b"\n" in self.buffer:
                raw, self.buffer = self.buffer.split(b"\n", 1)
                parsed = parse_line(raw.decode("ascii", errors="ignore"))
                if parsed is not None:
                    return parsed
        return None

    def send(self, right, left):
        self.ser.write(format_command(right, left).encode("ascii"))

    def stop(self):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def close(self):
        self.ser.close()


def first_reading(arduino):
    reading = arduino.read(timeout=3.0)
    if reading is None:
        sys.exit("El Arduino no manda datos. Revisa que el programa robot_control esté cargado "
                 "y que el cable USB sea de datos (algunos solo cargan).")
    return reading


def cmd_hola(arduino, args):
    reading = first_reading(arduino)
    print("El Arduino responde ✔")
    print(f"  contadores: derecha {reading['count'].get('r')}, izquierda {reading['count'].get('l')}")
    if reading["fault"]:
        print("  Ojo: el Arduino está en falla (" + ", ".join(NAMES[w] for w in sorted(reading["fault"]))
              + "). Apriétale el botón de reset.")
    return 0


def cmd_contar(arduino, args):
    first_reading(arduino)
    tracker = CountTracker()
    done = threading.Event()
    print(f"Marca cada rueda con cinta. Gira las dos ruedas HACIA ADELANTE exactamente "
          f"{args.vueltas} vueltas, a mano y despacio, y después aprieta Enter.")
    threading.Thread(target=lambda: (sys.stdin.readline(), done.set()), daemon=True).start()
    while not done.is_set():
        reading = arduino.read(timeout=0.5)
        if reading is not None:
            tracker.update(reading["count"])
            print(f"\r  derecha {tracker.total['r']:7d}   izquierda {tracker.total['l']:7d}   ",
                  end="", flush=True)
    print()
    print(f"Cuentas por vuelta (después de {args.vueltas} vueltas):")
    results = {}
    for wheel in ("r", "l"):
        per_rev = tracker.total[wheel] / args.vueltas
        results[wheel] = per_rev
        print(f"  {NAMES[wheel]:9s} {per_rev:8.1f}")
    for wheel in ("r", "l"):
        if results[wheel] < 0:
            print(f"Ojo: la rueda {NAMES[wheel]} cuenta hacia atrás. Intercambia los cables de las "
                  "fases A y B de su encoder (o la giraste al revés).")
        elif results[wheel] == 0:
            print(f"Ojo: la rueda {NAMES[wheel]} no contó nada. Revisa los cables de su encoder.")
    if all(v > 0 for v in results.values()):
        mean = (results["r"] + results["l"]) / 2
        print(f"Promedio: {mean:.0f}. Ese número va en counts_per_rev (robot_control.ino) "
              "y en encoder_counts_per_rev (bumperbot_ros2_control.xacro).")
    return 0


def drive(arduino, right, left, seconds):
    """Manda la velocidad cada 100 ms y devuelve las lecturas."""
    readings = []
    end = time.monotonic() + seconds
    next_send = 0.0
    try:
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                arduino.send(right, left)
                next_send = now + 0.1
            reading = arduino.read(timeout=0.1)
            if reading is not None:
                readings.append(reading)
                print(f"\r  derecha {reading['speed']['r']:6.2f} rad/s   "
                      f"izquierda {reading['speed']['l']:6.2f} rad/s   ", end="", flush=True)
                if reading["fault"]:
                    break
    finally:
        arduino.stop()
        print()
    return readings


def summarize(readings):
    tail = readings[len(readings) // 2:] or readings
    mean = {w: sum(r["speed"][w] for r in tail) / max(1, len(tail)) for w in ("r", "l")}
    faults = set().union(*(r["fault"] for r in readings)) if readings else set()
    return mean, faults


def report_faults(faults):
    for wheel in sorted(faults):
        print(f"FALLA en la rueda {NAMES[wheel]}: su encoder cuenta al revés respecto de su motor, "
              "o no cuenta. Intercambia las fases A y B de su encoder (o revisa sus cables) "
              "y apriétale el botón de reset al Arduino.")


def cmd_girar(arduino, args):
    first_reading(arduino)
    right = args.velocidad if args.rueda in ("ambas", "derecha") else 0.0
    left = args.velocidad if args.rueda in ("ambas", "izquierda") else 0.0
    print(f"Girando hacia adelante a {args.velocidad} rad/s durante {args.segundos} s "
          "(Ctrl+C para frenar)...")
    mean, faults = summarize(drive(arduino, right, left, args.segundos))
    report_faults(faults)
    if faults:
        return 1
    for wheel, asked in (("r", right), ("l", left)):
        if asked == 0:
            continue
        got = mean[wheel]
        if got < 0:
            print(f"La rueda {NAMES[wheel]} midió hacia atrás ({got:.2f} rad/s). Si la viste girar "
                  "hacia adelante, intercambia las fases A y B de su encoder.")
        elif got < 0.2 * asked:
            print(f"La rueda {NAMES[wheel]} casi no se movió ({got:.2f} rad/s). ¿Está conectada la "
                  "batería al L298N? ¿Están puestos los cables del motor?")
        else:
            print(f"Rueda {NAMES[wheel]}: {got:.2f} rad/s (pedido {asked:.2f}) ✔ "
                  "¿La viste girar HACIA ADELANTE? Si no, invierte los dos cables de ese motor en el L298N.")
    return 0


def cmd_maxima(arduino, args):
    first_reading(arduino)
    print(f"Las dos ruedas al máximo durante {args.segundos} s (Ctrl+C para frenar)...")
    mean, faults = summarize(drive(arduino, 30.0, 30.0, args.segundos))
    report_faults(faults)
    if faults:
        return 1
    slowest = min(mean.values())
    for wheel in ("r", "l"):
        print(f"  {NAMES[wheel]:9s} {mean[wheel]:6.2f} rad/s = {mean[wheel] * WHEEL_RADIUS:.2f} m/s")
    top = slowest * WHEEL_RADIUS
    print(f"Velocidad máxima del robot: {top:.2f} m/s (la de la rueda más lenta, sin peso encima).")
    if top < TARGET_SPEED:
        print(f"No llega a {TARGET_SPEED} m/s: las pruebas se hacen a menos velocidad "
              "(en los dos robots, el real y el gemelo).")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--puerto", default="/dev/arduino")
    parser.add_argument("--espera", type=float, default=2.0, help=argparse.SUPPRESS)
    sub = parser.add_subparsers(dest="prueba", required=True)
    sub.add_parser("hola")
    contar = sub.add_parser("contar")
    contar.add_argument("--vueltas", type=int, default=5)
    girar = sub.add_parser("girar")
    girar.add_argument("--rueda", choices=("ambas", "derecha", "izquierda"), default="ambas")
    girar.add_argument("--velocidad", type=float, default=3.0)
    girar.add_argument("--segundos", type=float, default=3.0)
    maxima = sub.add_parser("maxima")
    maxima.add_argument("--segundos", type=float, default=4.0)
    args = parser.parse_args(argv)

    arduino = Arduino(args.puerto, args.espera)
    commands = {"hola": cmd_hola, "contar": cmd_contar, "girar": cmd_girar, "maxima": cmd_maxima}
    try:
        return commands[args.prueba](arduino, args)
    except KeyboardInterrupt:
        arduino.stop()
        print("\nFrenado.")
        return 130
    finally:
        arduino.close()


if __name__ == "__main__":
    sys.exit(main())
