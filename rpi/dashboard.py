#!/usr/bin/env python3
"""
dashboard.py — Web dashboard for ESP32-S3 wake-on-radio power monitoring.

Combines serial logging, INA219 power sampling, and a live web UI with
a trigger button. Replaces serial_logger.py for interactive use.

Usage:
    python3 dashboard.py --port /dev/ttyAMA0 --ina-channel 1 --sample-rate 100

    # Without INA219 (serial + trigger only)
    python3 dashboard.py --port /dev/ttyAMA0 --no-ina

    # Custom web port
    python3 dashboard.py --port /dev/ttyAMA0 --web-port 8080

Requires:
    pip install flask pyserial smbus2

Open http://axon-command.local:5000 in a browser.
"""

import argparse
import csv
import json
import queue
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import re

import serial
from flask import Flask, Response, render_template, request, jsonify

# ── INA219 registers and constants ──────────────────────────────────────────

_REG_CONFIG = 0x00
_REG_SHUNT_VOLTAGE = 0x01
_REG_BUS_VOLTAGE = 0x02
_REG_POWER = 0x03
_REG_CURRENT = 0x04
_REG_CALIBRATION = 0x05

CHANNEL_ADDRS = {1: 0x40, 2: 0x41, 3: 0x42, 4: 0x43}
SHUNT_RESISTANCE_OHM = 0.1


class INA219:
    """Minimal INA219 driver for the Waveshare Current/Power Monitor HAT."""

    def __init__(self, bus_num=1, address=0x40, shunt_ohms=0.1, max_current_a=0.4):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        self._configure(max_current_a)

    def _write_register(self, reg, value):
        buf = [(value >> 8) & 0xFF, value & 0xFF]
        self.bus.write_i2c_block_data(self.addr, reg, buf)

    def _read_register(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (data[0] << 8) | data[1]

    def _read_register_signed(self, reg):
        raw = self._read_register(reg)
        if raw & 0x8000:
            raw -= 1 << 16
        return raw

    def _configure(self, max_current_a):
        config = (
            (0 << 13)
            | (0b00 << 11)
            | (0b0011 << 7)
            | (0b0011 << 3)
            | 0b111
        )
        self._write_register(_REG_CONFIG, config)
        self.current_lsb_a = max_current_a / 32768.0
        cal = int(0.04096 / (self.current_lsb_a * self.shunt_ohms))
        self._write_register(_REG_CALIBRATION, cal)

    def read_all(self):
        voltage_mv = ((self._read_register(_REG_BUS_VOLTAGE) >> 3) & 0x1FFF) * 4.0
        current_ua = self._read_register_signed(_REG_CURRENT) * self.current_lsb_a * 1_000_000
        power_uw = voltage_mv * current_ua / 1000.0
        return voltage_mv, current_ua, power_uw


# ── Shared state ────────────────────────────────────────────────────────────

# SSE subscribers: list of queue.Queue, one per connected client
sse_clients: list[queue.Queue] = []
sse_lock = threading.Lock()

# Ring buffer for chart history (last 120 seconds at ~10 Hz = 1200 points)
MAX_HISTORY = 1200
power_history: list[dict] = []
history_lock = threading.Lock()

# Ring buffer for 2-second rolling average at full sample rate
AVG_WINDOW_S = 2.0
avg_samples: list[tuple[float, float]] = []  # (timestamp, current_ua)
avg_lock = threading.Lock()

# State log (last 200 entries)
MAX_STATE_LOG = 200
state_log: list[dict] = []
state_lock = threading.Lock()

# ESP32 target for trigger
esp32_host = "esp32-wor.local"
esp32_port = 7777
esp32_ip_detected = None  # auto-detected from serial log


def broadcast_sse(event: str, data: dict):
    """Send an SSE event to all connected clients."""
    msg = f"event: {event}\ndata: {json.dumps(data)}\n\n"
    with sse_lock:
        dead = []
        for q in sse_clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)
        for q in dead:
            sse_clients.remove(q)


# ── INA219 sampling thread ─────────────────────────────────────────────────

def ina_thread(ina, sample_rate, csv_writer, csv_lock, csv_file, stop_event):
    sample_interval = 1.0 / sample_rate
    # Downsample SSE to ~10 Hz to avoid flooding browsers
    sse_interval = 0.1
    last_sse = 0

    while not stop_event.is_set():
        t0 = time.monotonic()
        try:
            voltage_mv, current_ua, power_uw = ina.read_all()
            now = datetime.now(timezone.utc).isoformat()
            ts = time.time()

            with csv_lock:
                csv_writer.writerow([
                    now, "INA219", "", "",
                    f"{voltage_mv:.1f}", f"{current_ua:.1f}", f"{power_uw:.1f}",
                ])
                csv_file.flush()

            # Update rolling average window with every sample
            with avg_lock:
                avg_samples.append((ts, current_ua))
                cutoff = ts - AVG_WINDOW_S
                while avg_samples and avg_samples[0][0] < cutoff:
                    avg_samples.pop(0)
                avg_current_ua = sum(s[1] for s in avg_samples) / len(avg_samples)

            point = {
                "ts": round(ts, 3),
                "voltage_mv": round(voltage_mv, 1),
                "current_ua": round(current_ua, 1),
                "power_uw": round(power_uw, 1),
                "avg_current_ua": round(avg_current_ua, 1),
            }

            with history_lock:
                power_history.append(point)
                if len(power_history) > MAX_HISTORY:
                    del power_history[:len(power_history) - MAX_HISTORY]

            if ts - last_sse >= sse_interval:
                broadcast_sse("power", point)
                last_sse = ts

        except OSError:
            pass

        elapsed = time.monotonic() - t0
        remaining = sample_interval - elapsed
        if remaining > 0:
            stop_event.wait(remaining)


# ── Serial reader thread ───────────────────────────────────────────────────

_IP_RE = re.compile(r"Got IP:\s*(\d+\.\d+\.\d+\.\d+)")


def serial_thread(ser, csv_writer, csv_lock, csv_file, stop_event):
    global esp32_ip_detected
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        now = datetime.now(timezone.utc).isoformat()

        # Auto-detect ESP32 IP from its log output
        m = _IP_RE.search(line)
        if m:
            esp32_ip_detected = m.group(1)
            broadcast_sse("esp32_ip", {"ip": esp32_ip_detected})
            print(f"ESP32 IP detected: {esp32_ip_detected}")

        if line.startswith("PWR|"):
            parts = line.split("|")
            if len(parts) == 3:
                with csv_lock:
                    csv_writer.writerow([now, "STATE", parts[1], parts[2], "", "", ""])
                    csv_file.flush()

                entry = {"ts": now, "esp_ts_us": parts[1], "state": parts[2]}
                with state_lock:
                    state_log.append(entry)
                    if len(state_log) > MAX_STATE_LOG:
                        del state_log[:len(state_log) - MAX_STATE_LOG]

                broadcast_sse("state", entry)
        else:
            # Forward non-PWR serial lines as log messages
            broadcast_sse("log", {"ts": now, "line": line})


# ── Flask app ───────────────────────────────────────────────────────────────

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("dashboard.html",
                           esp32_host=esp32_host,
                           esp32_port=esp32_port)


@app.route("/api/trigger", methods=["POST"])
def trigger():
    """Send UDP wake trigger to ESP32."""
    default_host = esp32_ip_detected or esp32_host
    host = request.json.get("host", default_host) if request.is_json else default_host
    port = request.json.get("port", esp32_port) if request.is_json else esp32_port

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(b"WAKE", (host, int(port)))
        sock.close()
        broadcast_sse("trigger", {"ts": datetime.now(timezone.utc).isoformat()})
        return jsonify({"status": "ok", "host": host, "port": port})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500


@app.route("/api/history")
def history():
    """Return recent power history for initial chart load."""
    with history_lock:
        return jsonify(list(power_history))


@app.route("/api/esp32")
def esp32_info():
    """Return detected ESP32 connection info."""
    return jsonify({
        "ip": esp32_ip_detected,
        "host": esp32_host,
        "port": esp32_port,
    })


@app.route("/api/states")
def states():
    """Return recent state transitions."""
    with state_lock:
        return jsonify(list(state_log))


@app.route("/api/stream")
def stream():
    """SSE endpoint for real-time updates."""
    q = queue.Queue(maxsize=256)
    with sse_lock:
        sse_clients.append(q)

    def generate():
        try:
            while True:
                try:
                    msg = q.get(timeout=30)
                    yield msg
                except queue.Empty:
                    yield ": keepalive\n\n"
        except GeneratorExit:
            with sse_lock:
                if q in sse_clients:
                    sse_clients.remove(q)

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


# ── Main ────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Wake-on-Radio Dashboard")
    p.add_argument("--port", default="/dev/ttyAMA0",
                   help="Serial port for ESP32 UART (default: /dev/ttyAMA0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--out", default="power_log.csv", help="Output CSV path")
    p.add_argument("--ina-channel", type=int, default=1, choices=[1, 2, 3, 4],
                   help="Waveshare HAT channel (1-4, default: 1)")
    p.add_argument("--sample-rate", type=int, default=100,
                   help="INA219 samples per second (default: 100)")
    p.add_argument("--no-ina", action="store_true",
                   help="Disable INA219 reading (serial only)")
    p.add_argument("--web-port", type=int, default=5000,
                   help="Web dashboard port (default: 5000)")
    p.add_argument("--esp32-host", default="esp32-wor.local",
                   help="ESP32 hostname/IP for trigger (default: esp32-wor.local)")
    p.add_argument("--esp32-port", type=int, default=7777,
                   help="ESP32 UDP trigger port (default: 7777)")
    return p.parse_args()


def main():
    global esp32_host, esp32_port
    args = parse_args()
    esp32_host = args.esp32_host
    esp32_port = args.esp32_port

    # CSV setup
    out_path = Path(args.out)
    write_header = not out_path.exists()
    csv_file = open(out_path, "a", newline="")
    csv_writer = csv.writer(csv_file)
    csv_lock_obj = threading.Lock()

    if write_header:
        csv_writer.writerow([
            "rpi_timestamp", "type", "esp_timestamp_us",
            "state", "voltage_mv", "current_ua", "power_uw",
        ])
        csv_file.flush()

    stop_event = threading.Event()

    # Start INA219
    if not args.no_ina:
        addr = CHANNEL_ADDRS[args.ina_channel]
        try:
            ina = INA219(address=addr)
            print(f"INA219 found on CH{args.ina_channel} (0x{addr:02x}), "
                  f"sampling at {args.sample_rate} Hz")
            t = threading.Thread(target=ina_thread, daemon=True,
                                 args=(ina, args.sample_rate,
                                       csv_writer, csv_lock_obj, csv_file,
                                       stop_event))
            t.start()
        except OSError as e:
            print(f"Warning: INA219 not found on 0x{addr:02x}: {e}")
            print("  Continuing without power monitoring. Use --no-ina to suppress.")

    # Start serial reader
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Serial: {args.port} @ {args.baud} baud")
    t = threading.Thread(target=serial_thread, daemon=True,
                         args=(ser, csv_writer, csv_lock_obj, csv_file, stop_event))
    t.start()

    print(f"Logging to: {out_path}")
    print(f"ESP32 target: {esp32_host}:{esp32_port}")
    print(f"Dashboard: http://0.0.0.0:{args.web_port}")
    print("-" * 60)

    try:
        app.run(host="0.0.0.0", port=args.web_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()
        csv_file.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
