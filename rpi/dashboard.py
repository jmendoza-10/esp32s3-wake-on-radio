#!/usr/bin/env python3
"""
dashboard.py — Web dashboard for ESP32-S3 wake-on-radio power monitoring.

Combines serial logging, INA219 power sampling, and a live web UI with
a trigger button. Replaces serial_logger.py for interactive use.

Usage:
    python3 dashboard.py --port /dev/ttyS0 --ina-channel 1 --sample-rate 100

    # Without INA219 (serial + trigger only)
    python3 dashboard.py --port /dev/ttyS0 --no-ina

    # Custom web port
    python3 dashboard.py --port /dev/ttyS0 --web-port 8080

Requires:
    pip install flask pyserial smbus2

Open http://axon-command.local:5000 in a browser.
"""

import argparse
import csv
import glob as glob_mod
import json
import queue
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import re

import serial
from flask import Flask, Response, render_template, request, jsonify, send_file

# ── INA219 registers and constants ──────────────────────────────────────────

_REG_CONFIG = 0x00
_REG_SHUNT_VOLTAGE = 0x01
_REG_BUS_VOLTAGE = 0x02
_REG_POWER = 0x03
_REG_CURRENT = 0x04
_REG_CALIBRATION = 0x05

CHANNEL_ADDRS = {1: 0x40, 2: 0x41, 3: 0x42, 4: 0x43}
DEFAULT_SHUNT_OHMS = 0.1
DEFAULT_MAX_CURRENT_A = 1.6
CALIBRATION_CONFIG_PATH = Path("ina219_calibration.json")


class INA219:
    """Minimal INA219 driver for the Waveshare Current/Power Monitor HAT."""

    def __init__(self, bus_num=1, address=0x40, shunt_ohms=0.1,
                 max_current_a=1.6, channel=None):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.channel = channel
        self.shunt_ohms = shunt_ohms
        self._configure(max_current_a)
        self._last_voltage_mv = 0.0
        self._last_current_ua = 0.0
        self._last_power_uw = 0.0
        self._sample_count = 0

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
        if max_current_a <= 0:
            raise ValueError("max_current_a must be positive")
        if self.shunt_ohms <= 0:
            raise ValueError("shunt_ohms must be positive")

        target_shunt_v = max_current_a * self.shunt_ohms
        pga_choices = (
            (0b00, 0.040),
            (0b01, 0.080),
            (0b10, 0.160),
            (0b11, 0.320),
        )
        pga_bits, self.shunt_range_v = pga_choices[-1]
        for candidate_bits, shunt_range_v in pga_choices:
            if target_shunt_v <= shunt_range_v:
                pga_bits = candidate_bits
                self.shunt_range_v = shunt_range_v
                break

        self.shunt_range_a = self.shunt_range_v / self.shunt_ohms
        self.overrange_expected = target_shunt_v > self.shunt_range_v

        self.max_current_a = max_current_a
        config = (
            (0 << 13)
            | (pga_bits << 11)
            | (0b0000 << 7)
            | (0b0000 << 3)
            | 0b111
        )
        self._write_register(_REG_CONFIG, config)
        requested_lsb_a = max_current_a / 32768.0
        min_lsb_for_cal_a = 0.04096 / (65535 * self.shunt_ohms)
        self.current_lsb_a = max(requested_lsb_a, min_lsb_for_cal_a)
        self.calibration = int(0.04096 / (self.current_lsb_a * self.shunt_ohms))
        self._write_register(_REG_CALIBRATION, self.calibration)

    def configure(self, shunt_ohms=None, max_current_a=None):
        if shunt_ohms is not None:
            shunt_ohms = float(shunt_ohms)
            if shunt_ohms <= 0:
                raise ValueError("shunt_ohms must be positive")
            self.shunt_ohms = shunt_ohms
        if max_current_a is None:
            max_current_a = self.max_current_a
        max_current_a = float(max_current_a)
        self._configure(max_current_a)

    def read_all(self):
        self._sample_count += 1
        if self._sample_count == 1 or self._sample_count % 100 == 0:
            self._last_voltage_mv = ((self._read_register(_REG_BUS_VOLTAGE) >> 3) & 0x1FFF) * 4.0
            if self._read_register(_REG_CALIBRATION) == 0:
                label = f"CH{self.channel}" if self.channel is not None else f"0x{self.addr:02x}"
                print(f"INA219 {label} calibration register cleared; reconfiguring", flush=True)
                self._configure(self.max_current_a)
        voltage_mv = self._last_voltage_mv
        current_ua = self._read_register_signed(_REG_CURRENT) * self.current_lsb_a * 1_000_000
        power_uw = voltage_mv * current_ua / 1000.0
        self._last_current_ua = current_ua
        self._last_power_uw = power_uw
        return voltage_mv, current_ua, power_uw

    def status(self):
        calibration = self._read_register(_REG_CALIBRATION)
        bus_raw = self._read_register(_REG_BUS_VOLTAGE)
        current_raw = self._read_register_signed(_REG_CURRENT)
        power_raw = self._read_register(_REG_POWER)
        voltage_mv = ((bus_raw >> 3) & 0x1FFF) * 4.0
        current_ua = current_raw * self.current_lsb_a * 1_000_000
        return {
            "available": True,
            "channel": self.channel,
            "address": f"0x{self.addr:02x}",
            "shunt_ohms": self.shunt_ohms,
            "max_current_a": self.max_current_a,
            "shunt_range_a": self.shunt_range_a,
            "shunt_range_mv": self.shunt_range_v * 1000.0,
            "current_lsb_ua": self.current_lsb_a * 1_000_000,
            "calibration": calibration,
            "expected_calibration": self.calibration,
            "calibrated": calibration != 0,
            "overrange_expected": self.overrange_expected,
            "voltage_mv": voltage_mv,
            "current_ua": current_ua,
            "power_uw": voltage_mv * current_ua / 1000.0,
            "power_raw": power_raw,
        }


# ── Shared state ────────────────────────────────────────────────────────────

# SSE subscribers: list of queue.Queue, one per connected client
sse_clients: list[queue.Queue] = []
sse_lock = threading.Lock()

# Ring buffer for chart history (high-resolution rolling window)
MAX_HISTORY = 10000
power_history: list[dict] = []
history_lock = threading.Lock()

# Active INA219 instances for status and manual reconfiguration.
ina_sensors: dict[int, INA219] = {}
ina_channel_active = None
ina_highspeed_channels: list[int] = []
ina_lock = threading.Lock()

# Ring buffer for rolling average at full sample rate
AVG_WINDOW_S = 3.0
avg_samples: dict[int, list[tuple[float, float]]] = {}  # channel -> [(timestamp, current_ua)]
avg_lock = threading.Lock()

# State log (last 200 entries)
MAX_STATE_LOG = 200
state_log: list[dict] = []
state_lock = threading.Lock()

# ESP32 target for trigger
esp32_host = "esp32-wor.local"
esp32_port = 7777
esp32_ip_detected = None  # auto-detected from serial log

# Kernel driver sysfs state
SYSFS_BASE = "/sys/devices/platform/esp32-wor"
driver_state = {
    "available": False,
    "wake_count": 0,
    "active": False,
    "last_wake_ns": 0,
    "last_duration_ns": 0,
}
driver_lock = threading.Lock()

# RTT tracking: UDP send → GPIO rising edge
# Both use CLOCK_MONOTONIC (Python time.monotonic_ns == kernel ktime_get)
last_trigger_mono_ns = 0  # set when UDP packet is sent
trigger_ns_lock = threading.Lock()

MAX_RTT_SAMPLES = 200
rtt_samples: list[dict] = []  # {"rtt_ms": float, "ts": iso_string}
rtt_lock = threading.Lock()

# Burst test state
burst_state = {
    "running": False,
    "total": 0,
    "completed": 0,
    "errors": 0,
}
burst_lock = threading.Lock()
burst_stop = threading.Event()


# ── Logic analyzer state ───────────────────────────────────────────────────

CAPTURE_DIR = Path("/tmp/wor-captures")
CAPTURE_DIR.mkdir(exist_ok=True)

la_state = {
    "available": False,
    "device": None,        # e.g. "fx2lafw"
    "capturing": False,
    "sample_rate": "1m",   # sigrok format: 1m = 1 MHz
    "file": None,          # current/last capture file path
    "started_at": None,
    "error": None,
}
la_lock = threading.Lock()
la_process: subprocess.Popen | None = None


def _detect_fx2() -> str | None:
    """Run sigrok-cli --scan and return driver name if FX2 device found."""
    try:
        result = subprocess.run(
            ["sigrok-cli", "--scan"],
            capture_output=True, text=True, timeout=10,
        )
        for line in result.stdout.splitlines():
            if "fx2lafw" in line.lower():
                return "fx2lafw"
        return None
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


def la_detect_thread(stop_event):
    """Periodically check if FX2 logic analyzer is connected."""
    while not stop_event.is_set():
        driver = _detect_fx2()
        with la_lock:
            was_available = la_state["available"]
            la_state["available"] = driver is not None
            la_state["device"] = driver
            if not driver and la_state["capturing"]:
                la_state["capturing"] = False
                la_state["error"] = "Device disconnected"
            snapshot = dict(la_state)

        if snapshot["available"] != was_available:
            broadcast_sse("logic_analyzer", snapshot)

        stop_event.wait(5.0)


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

def _channel_status_locked(channel, ina):
    try:
        status = ina.status()
        status["channel"] = channel
        return status
    except OSError as exc:
        return {
            "available": False,
            "channel": channel,
            "address": f"0x{ina.addr:02x}",
            "error": str(exc),
        }


def _ina_status_payload_locked():
    if not ina_sensors:
        return {"available": False, "error": "INA219 disabled", "channels": []}
    channels = [
        _channel_status_locked(channel, ina)
        for channel, ina in sorted(ina_sensors.items())
    ]
    return {
        "available": any(ch.get("available") for ch in channels),
        "active_channel": ina_channel_active,
        "highspeed_channels": list(ina_highspeed_channels),
        "channels": channels,
    }


def _parse_optional_positive_float(payload, key):
    if key not in payload or payload[key] in (None, ""):
        return None
    value = float(payload[key])
    if value <= 0:
        raise ValueError(f"{key} must be positive")
    return value


def _load_calibration_config():
    try:
        with CALIBRATION_CONFIG_PATH.open() as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Warning: could not load {CALIBRATION_CONFIG_PATH}: {exc}", flush=True)
        return {}

    channels = data.get("channels", data)
    if not isinstance(channels, dict):
        return {}
    loaded = {}
    for key, cfg in channels.items():
        try:
            channel = int(key)
            if channel not in CHANNEL_ADDRS or not isinstance(cfg, dict):
                continue
            loaded[channel] = {
                "shunt_ohms": float(cfg["shunt_ohms"]),
                "max_current_a": float(cfg["max_current_a"]),
            }
        except (KeyError, TypeError, ValueError):
            continue
    return loaded


def _load_highspeed_config():
    try:
        with CALIBRATION_CONFIG_PATH.open() as f:
            data = json.load(f)
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None

    raw_channels = data.get("highspeed_channels")
    if not isinstance(raw_channels, list):
        return None

    channels = []
    for raw_channel in raw_channels:
        try:
            channel = int(raw_channel)
        except (TypeError, ValueError):
            continue
        if channel in CHANNEL_ADDRS and channel not in channels:
            channels.append(channel)
    return channels or None


def _save_calibration_config_locked():
    data = {
        "highspeed_channels": list(ina_highspeed_channels),
        "channels": {
            str(channel): {
                "shunt_ohms": ina.shunt_ohms,
                "max_current_a": ina.max_current_a,
            }
            for channel, ina in sorted(ina_sensors.items())
        }
    }
    tmp_path = CALIBRATION_CONFIG_PATH.with_suffix(".json.tmp")
    tmp_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    tmp_path.replace(CALIBRATION_CONFIG_PATH)


def ina_thread(highspeed_channels, sample_rate, csv_writer, csv_lock, csv_file, stop_event):
    highspeed_channels = list(highspeed_channels)
    sample_interval = 1.0 / sample_rate
    # Downsample SSE to ~10 Hz to avoid flooding browsers
    sse_interval = 0.1
    # Low-speed channel plots are fed from the status stream at ~10 Hz.
    status_sse_interval = 0.1
    last_sse = 0
    last_status_sse = 0
    last_csv_flush = time.monotonic()
    display_sum_ua = {}
    display_count = {}
    batch_points = {}
    channel_index = 0

    while not stop_event.is_set():
        t0 = time.monotonic()
        try:
            with ina_lock:
                current_highspeed_channels = [
                    channel for channel in ina_highspeed_channels
                    if channel in ina_sensors
                ]
                if not current_highspeed_channels:
                    current_highspeed_channels = [
                        channel for channel in highspeed_channels
                        if channel in ina_sensors
                    ]
                if not current_highspeed_channels:
                    raise OSError("No INA219 high-speed channels available")
                if channel_index >= len(current_highspeed_channels):
                    channel_index = 0
                active_channel = current_highspeed_channels[channel_index]
                channel_index = (channel_index + 1) % len(current_highspeed_channels)
                ina = ina_sensors.get(active_channel)
                if ina is None:
                    raise OSError(f"INA219 CH{active_channel} unavailable")
                voltage_mv, current_ua, power_uw = ina.read_all()
            ts = time.time()
            now = f"{ts:.6f}"

            with csv_lock:
                csv_writer.writerow([
                    now, f"INA219_CH{active_channel}", "", "",
                    f"{voltage_mv:.1f}", f"{current_ua:.1f}", f"{power_uw:.1f}",
                ])
                if time.monotonic() - last_csv_flush >= 0.1:
                    csv_file.flush()
                    last_csv_flush = time.monotonic()

            point = {
                "ts": round(ts, 6),
                "channel": active_channel,
                "voltage_mv": round(voltage_mv, 1),
                "current_ua": round(current_ua, 1),
                "power_uw": round(power_uw, 1),
            }
            batch_points.setdefault(active_channel, []).append(point)
            display_sum_ua[active_channel] = display_sum_ua.get(active_channel, 0.0) + current_ua
            display_count[active_channel] = display_count.get(active_channel, 0) + 1

            with avg_lock:
                samples = avg_samples.setdefault(active_channel, [])
                samples.append((ts, current_ua))
                cutoff = ts - AVG_WINDOW_S
                while samples and samples[0][0] < cutoff:
                    samples.pop(0)
                window_avg_current_ua = (
                    sum(sample[1] for sample in samples) / len(samples)
                    if samples else current_ua
                )

            point["window_avg_current_ua"] = round(window_avg_current_ua, 1)

            if ts - last_sse >= sse_interval:
                for channel in sorted(batch_points):
                    points = batch_points.get(channel, [])
                    if not points:
                        continue
                    avg_current_ua = (
                        display_sum_ua.get(channel, 0.0) / display_count[channel]
                        if display_count[channel] else points[-1]["current_ua"]
                    )
                    latest = dict(points[-1])
                    latest["avg_current_ua"] = round(avg_current_ua, 1)
                    latest["avg_window_s"] = AVG_WINDOW_S
                    latest["highspeed_channels"] = current_highspeed_channels

                    with history_lock:
                        power_history.extend(points)
                        if len(power_history) > MAX_HISTORY:
                            del power_history[:len(power_history) - MAX_HISTORY]

                    payload = dict(latest)
                    payload["samples"] = points
                    broadcast_sse("power", payload)
                    batch_points[channel] = []
                    display_sum_ua[channel] = 0.0
                    display_count[channel] = 0

                last_sse = ts

            if ts - last_status_sse >= status_sse_interval:
                with ina_lock:
                    broadcast_sse("ina219", _ina_status_payload_locked())
                last_status_sse = ts

        except OSError:
            pass

        elapsed = time.monotonic() - t0
        remaining = sample_interval - elapsed
        if remaining > 0:
            stop_event.wait(remaining)


# ── Driver sysfs polling thread ────────────────────────────────────────────

def _read_sysfs(name):
    """Read a single sysfs attribute, return string or None on failure."""
    try:
        with open(f"{SYSFS_BASE}/{name}") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


def _compute_rtt_stats():
    """Compute avg and p90 from collected RTT samples. Caller holds rtt_lock."""
    if not rtt_samples:
        return 0.0, 0.0
    values = [s["rtt_ms"] for s in rtt_samples]
    avg = sum(values) / len(values)
    sorted_v = sorted(values)
    p90_idx = int(len(sorted_v) * 0.9)
    p90 = sorted_v[min(p90_idx, len(sorted_v) - 1)]
    return round(avg, 2), round(p90, 2)


def driver_thread(stop_event):
    """Poll the esp32_wor kernel driver sysfs at ~10 Hz."""
    global last_trigger_mono_ns
    prev_wake_count = 0

    while not stop_event.is_set():
        wc = _read_sysfs("wake_count")
        if wc is None:
            # Driver not loaded
            with driver_lock:
                if driver_state["available"]:
                    driver_state["available"] = False
                    broadcast_sse("driver", dict(driver_state))
            stop_event.wait(2.0)
            continue

        active = _read_sysfs("active")
        last_wake = _read_sysfs("last_wake_ns")
        last_dur = _read_sysfs("last_duration_ns")

        with driver_lock:
            driver_state["available"] = True
            driver_state["wake_count"] = int(wc)
            driver_state["active"] = active == "1"
            driver_state["last_wake_ns"] = int(last_wake or 0)
            driver_state["last_duration_ns"] = int(last_dur or 0)
            snapshot = dict(driver_state)

        # Broadcast on every poll so the UI stays current
        broadcast_sse("driver", snapshot)

        # Detect new wake and compute RTT
        new_wc = int(wc)
        if new_wc > prev_wake_count:
            dur_ms = int(last_dur or 0) / 1_000_000
            wake_ns = int(last_wake or 0)

            # Compute RTT: kernel rising-edge timestamp minus UDP send timestamp
            # Both are CLOCK_MONOTONIC nanoseconds
            with trigger_ns_lock:
                send_ns = last_trigger_mono_ns

            if send_ns > 0 and wake_ns > send_ns:
                rtt_ms = (wake_ns - send_ns) / 1_000_000
                now_iso = datetime.now(timezone.utc).isoformat()

                with rtt_lock:
                    rtt_samples.append({"rtt_ms": round(rtt_ms, 2), "ts": now_iso})
                    if len(rtt_samples) > MAX_RTT_SAMPLES:
                        del rtt_samples[:len(rtt_samples) - MAX_RTT_SAMPLES]
                    avg_ms, p90_ms = _compute_rtt_stats()

                rtt_event = {
                    "rtt_ms": round(rtt_ms, 2),
                    "avg_ms": avg_ms,
                    "p90_ms": p90_ms,
                    "count": len(rtt_samples),
                    "ts": now_iso,
                }
                broadcast_sse("rtt", rtt_event)
                print(f"Driver: wake #{new_wc} RTT={rtt_ms:.1f}ms "
                      f"(avg={avg_ms:.1f}ms p90={p90_ms:.1f}ms "
                      f"n={len(rtt_samples)}, pulse={dur_ms:.1f}ms)")

                # Clear so we don't re-match a stale trigger
                with trigger_ns_lock:
                    last_trigger_mono_ns = 0
            else:
                print(f"Driver: wake #{new_wc} (duration={dur_ms:.1f}ms, "
                      f"no trigger timestamp for RTT)")

            prev_wake_count = new_wc

        stop_event.wait(0.1)


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
    host = esp32_ip_detected or esp32_host
    if _send_one_trigger():
        return jsonify({"status": "ok", "host": host, "port": esp32_port})
    else:
        return jsonify({"status": "error", "error": "send failed"}), 500


def _send_one_trigger():
    """Send a single UDP trigger and record monotonic time for RTT. Returns True on success."""
    global last_trigger_mono_ns
    host = esp32_ip_detected or esp32_host
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        with trigger_ns_lock:
            last_trigger_mono_ns = time.monotonic_ns()
        sock.sendto(b"WAKE", (host, int(esp32_port)))
        sock.close()
        broadcast_sse("trigger", {"ts": datetime.now(timezone.utc).isoformat()})
        return True
    except OSError:
        return False


def burst_thread(count, interval):
    """Run a burst of triggers, waiting for each wake cycle to complete."""
    global last_trigger_mono_ns

    with burst_lock:
        burst_state["running"] = True
        burst_state["total"] = count
        burst_state["completed"] = 0
        burst_state["errors"] = 0

    broadcast_sse("burst", dict(burst_state))

    for i in range(count):
        if burst_stop.is_set():
            break

        # Wait for ESP32 to be in idle state (GPIO LOW, not active)
        # before sending the next trigger
        for _ in range(100):  # up to 10 seconds
            if burst_stop.is_set():
                break
            with driver_lock:
                active = driver_state.get("active", False)
            if not active:
                break
            time.sleep(0.1)

        if burst_stop.is_set():
            break

        # Small delay to let ESP32 finish rebooting and reach DTIM idle
        time.sleep(interval)

        if burst_stop.is_set():
            break

        wc_before = 0
        with driver_lock:
            wc_before = driver_state.get("wake_count", 0)

        ok = _send_one_trigger()

        with burst_lock:
            if ok:
                burst_state["completed"] += 1
            else:
                burst_state["errors"] += 1
            snapshot = dict(burst_state)

        broadcast_sse("burst", snapshot)
        print(f"Burst: {snapshot['completed']}/{count}"
              f"{' (error)' if not ok else ''}")

        # Wait for this wake cycle to complete (GPIO goes HIGH then LOW)
        # Timeout after 15 seconds
        for _ in range(150):
            if burst_stop.is_set():
                break
            with driver_lock:
                wc_now = driver_state.get("wake_count", 0)
            if wc_now > wc_before:
                break
            time.sleep(0.1)

    with burst_lock:
        burst_state["running"] = False
    broadcast_sse("burst", dict(burst_state))
    burst_stop.clear()
    print(f"Burst complete: {burst_state['completed']}/{count} "
          f"({burst_state['errors']} errors)")


@app.route("/api/burst/start", methods=["POST"])
def burst_start():
    """Start a burst test: POST {"count": 10, "interval": 5}"""
    with burst_lock:
        if burst_state["running"]:
            return jsonify({"status": "error", "error": "burst already running"}), 409

    data = request.get_json(force=True)
    count = int(data.get("count", 10))
    interval = float(data.get("interval", 5))

    burst_stop.clear()
    t = threading.Thread(target=burst_thread, daemon=True,
                         args=(count, interval))
    t.start()
    return jsonify({"status": "ok", "count": count, "interval": interval})


@app.route("/api/burst/stop", methods=["POST"])
def burst_stop_api():
    """Stop a running burst test."""
    burst_stop.set()
    return jsonify({"status": "ok"})


@app.route("/api/burst")
def burst_info():
    """Return current burst test state."""
    with burst_lock:
        return jsonify(dict(burst_state))


@app.route("/api/logic-analyzer")
def la_info():
    """Return current logic analyzer state."""
    with la_lock:
        return jsonify(dict(la_state))


@app.route("/api/logic-analyzer/start", methods=["POST"])
def la_start():
    """Start a logic analyzer capture."""
    global la_process
    with la_lock:
        if not la_state["available"]:
            return jsonify({"status": "error", "error": "No device detected"}), 404
        if la_state["capturing"]:
            return jsonify({"status": "error", "error": "Already capturing"}), 409

    data = request.get_json(force=True) if request.data else {}
    sample_rate = data.get("sample_rate", "1m")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    capture_file = str(CAPTURE_DIR / f"capture_{ts}.sr")

    cmd = [
        "sigrok-cli", "-d", "fx2lafw",
        "-c", f"samplerate={sample_rate}",
        "--continuous",
        "-o", capture_file,
    ]

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
    except FileNotFoundError:
        return jsonify({"status": "error", "error": "sigrok-cli not found"}), 500

    la_process = proc

    with la_lock:
        la_state["capturing"] = True
        la_state["sample_rate"] = sample_rate
        la_state["file"] = capture_file
        la_state["started_at"] = datetime.now(timezone.utc).isoformat()
        la_state["error"] = None
        snapshot = dict(la_state)

    broadcast_sse("logic_analyzer", snapshot)
    print(f"Logic Analyzer: capture started → {capture_file} @ {sample_rate}Hz")

    # Monitor process in background
    def _monitor():
        global la_process
        proc.wait()
        with la_lock:
            la_state["capturing"] = False
            # Check if it exited with error (vs normal SIGTERM from stop)
            if proc.returncode not in (0, -15, -2):  # 0, SIGTERM, SIGINT
                stderr = proc.stderr.read().decode(errors="replace").strip()
                la_state["error"] = stderr or f"Exit code {proc.returncode}"
            snapshot = dict(la_state)
        la_process = None
        broadcast_sse("logic_analyzer", snapshot)
        print(f"Logic Analyzer: capture stopped (exit={proc.returncode})")

    threading.Thread(target=_monitor, daemon=True).start()

    return jsonify({"status": "ok", "file": capture_file,
                    "sample_rate": sample_rate})


@app.route("/api/logic-analyzer/stop", methods=["POST"])
def la_stop():
    """Stop a running capture."""
    global la_process
    if la_process is None:
        return jsonify({"status": "error", "error": "No capture running"}), 404

    la_process.terminate()
    return jsonify({"status": "ok"})


@app.route("/api/logic-analyzer/captures")
def la_captures():
    """List available capture files."""
    files = sorted(CAPTURE_DIR.glob("capture_*.sr"), reverse=True)
    result = []
    for f in files[:20]:
        stat = f.stat()
        result.append({
            "name": f.name,
            "size_kb": round(stat.st_size / 1024, 1),
            "created": datetime.fromtimestamp(
                stat.st_mtime, tz=timezone.utc).isoformat(),
        })
    return jsonify(result)


@app.route("/api/logic-analyzer/captures/<filename>")
def la_download(filename):
    """Download a capture file."""
    # Sanitize filename to prevent path traversal
    safe_name = Path(filename).name
    filepath = CAPTURE_DIR / safe_name
    if not filepath.exists():
        return jsonify({"error": "Not found"}), 404
    return send_file(filepath, as_attachment=True)


@app.route("/api/history")
def history():
    """Return recent power history for initial chart load."""
    with history_lock:
        return jsonify(list(power_history))


@app.route("/api/ina219")
def ina219_info():
    """Return live Waveshare/INA219 status."""
    if not ina_sensors:
        return jsonify({"available": False, "error": "INA219 disabled", "channels": []})
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"available": False, "error": "INA219 reader busy"}), 503
    try:
        return jsonify(_ina_status_payload_locked())
    except OSError as exc:
        return jsonify({"available": False, "error": str(exc), "channels": []}), 503
    finally:
        ina_lock.release()


@app.route("/api/ina219/<int:channel>")
def ina219_channel_info(channel):
    """Return one Waveshare/INA219 channel status."""
    if channel not in CHANNEL_ADDRS:
        return jsonify({"available": False, "error": "invalid channel"}), 404
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"available": False, "error": "INA219 reader busy"}), 503
    try:
        ina = ina_sensors.get(channel)
        if ina is None:
            return jsonify({"available": False, "channel": channel, "error": "channel disabled"}), 404
        return jsonify(_channel_status_locked(channel, ina))
    finally:
        ina_lock.release()


def _reconfigure_channel_locked(channel, ina, shunt_ohms=None, max_current_a=None):
    ina.configure(shunt_ohms=shunt_ohms, max_current_a=max_current_a)
    status = ina.status()
    status["channel"] = channel
    status["status"] = "ok"
    return status


def _parse_highspeed_payload_channels(payload):
    raw_channels = payload.get("channels", [])
    if isinstance(raw_channels, dict):
        raw_channels = [
            key for key, enabled in raw_channels.items()
            if enabled
        ]
    elif isinstance(raw_channels, str):
        raw_channels = parse_channel_list(raw_channels)
    elif not isinstance(raw_channels, list):
        raise ValueError("channels must be a list, object, or comma-separated string")

    channels = []
    for raw_channel in raw_channels:
        channel = int(raw_channel)
        if channel not in CHANNEL_ADDRS:
            raise ValueError(f"invalid INA219 channel {channel}")
        if channel not in ina_sensors:
            raise ValueError(f"INA219 CH{channel} is not configured")
        if channel not in channels:
            channels.append(channel)
    if not channels:
        raise ValueError("at least one high-speed channel is required")
    return channels


@app.route("/api/ina219/highspeed", methods=["POST"])
def ina219_highspeed():
    """Set INA219 channels sampled by the high-speed acquisition loop."""
    global ina_highspeed_channels, ina_channel_active
    if not ina_sensors:
        return jsonify({"status": "error", "error": "INA219 disabled"}), 404
    payload = request.get_json(silent=True) or {}
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "INA219 reader busy"}), 503
    try:
        channels = _parse_highspeed_payload_channels(payload)
        ina_highspeed_channels = channels
        if ina_channel_active not in channels:
            ina_channel_active = channels[0]
        _save_calibration_config_locked()
        status = _ina_status_payload_locked()
        status["status"] = "ok"
        broadcast_sse("ina219", status)
        print(
            "INA219 high-speed channels set to "
            + ",".join(f"CH{channel}" for channel in channels),
            flush=True,
        )
        return jsonify(status)
    except ValueError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    finally:
        ina_lock.release()


@app.route("/api/ina219/reconfigure", methods=["POST"])
def ina219_reconfigure():
    """Reapply INA219 configuration/calibration on all configured channels."""
    if not ina_sensors:
        return jsonify({"status": "error", "error": "INA219 disabled"}), 404
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "INA219 reader busy"}), 503
    try:
        payload_in = request.get_json(silent=True) or {}
        channel_updates = payload_in.get("channels", {})
        if channel_updates and not isinstance(channel_updates, dict):
            return jsonify({"status": "error", "error": "channels must be an object"}), 400

        channels = []
        for channel, ina in sorted(ina_sensors.items()):
            update = channel_updates.get(str(channel), channel_updates.get(channel, {}))
            if update is None:
                update = {}
            if not isinstance(update, dict):
                return jsonify({"status": "error", "error": f"CH{channel} update must be an object"}), 400
            shunt_ohms = _parse_optional_positive_float(update, "shunt_ohms")
            max_current_a = _parse_optional_positive_float(update, "max_current_a")
            channels.append(_reconfigure_channel_locked(
                channel, ina,
                shunt_ohms=shunt_ohms,
                max_current_a=max_current_a,
            ))
        _save_calibration_config_locked()
        payload = {
            "status": "ok",
            "available": True,
            "active_channel": ina_channel_active,
            "highspeed_channels": list(ina_highspeed_channels),
            "channels": channels,
        }
        broadcast_sse("ina219", payload)
        print("INA219 all channels calibrated manually", flush=True)
        return jsonify(payload)
    except ValueError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    except OSError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 503
    finally:
        ina_lock.release()


@app.route("/api/ina219/<int:channel>/reconfigure", methods=["POST"])
def ina219_channel_reconfigure(channel):
    """Reapply INA219 configuration/calibration on one channel."""
    if channel not in CHANNEL_ADDRS:
        return jsonify({"status": "error", "error": "invalid channel"}), 404
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "INA219 reader busy"}), 503
    try:
        ina = ina_sensors.get(channel)
        if ina is None:
            return jsonify({"status": "error", "channel": channel, "error": "channel disabled"}), 404
        payload = request.get_json(silent=True) or {}
        shunt_ohms = _parse_optional_positive_float(payload, "shunt_ohms")
        max_current_a = _parse_optional_positive_float(payload, "max_current_a")
        status = _reconfigure_channel_locked(
            channel, ina,
            shunt_ohms=shunt_ohms,
            max_current_a=max_current_a,
        )
        _save_calibration_config_locked()
        broadcast_sse("ina219", _ina_status_payload_locked())
        print(f"INA219 CH{channel} calibrated manually", flush=True)
        return jsonify(status)
    except ValueError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    except OSError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 503
    finally:
        ina_lock.release()


@app.route("/api/esp32")
def esp32_info():
    """Return detected ESP32 connection info."""
    return jsonify({
        "ip": esp32_ip_detected,
        "host": esp32_host,
        "port": esp32_port,
    })


@app.route("/api/driver")
def driver_info():
    """Return current kernel driver state."""
    with driver_lock:
        return jsonify(dict(driver_state))


@app.route("/api/rtt")
def rtt_info():
    """Return RTT measurement history and stats."""
    with rtt_lock:
        avg_ms, p90_ms = _compute_rtt_stats()
        return jsonify({
            "samples": list(rtt_samples),
            "avg_ms": avg_ms,
            "p90_ms": p90_ms,
            "count": len(rtt_samples),
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
    p.add_argument("--port", default="/dev/ttyS0",
                   help="Serial port for ESP32 UART (default: /dev/ttyS0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--out", default="power_log.csv", help="Output CSV path")
    p.add_argument("--ina-channel", type=int, default=1, choices=[1, 2, 3, 4],
                   help="Waveshare HAT channel used for the high-rate chart (1-4, default: 1)")
    p.add_argument("--ina-channels", default=None,
                   help="Comma-separated Waveshare HAT channels to monitor, e.g. 1,2,3,4")
    p.add_argument("--highspeed-channels", default=None,
                   help="Comma-separated INA219 channels sampled for high-speed plots, e.g. 1,2")
    p.add_argument("--sample-rate", type=int, default=100,
                   help="Total INA219 high-speed samples per second across high-speed channels (default: 100)")
    p.add_argument("--max-current", default=str(DEFAULT_MAX_CURRENT_A),
                   help=("Expected max current in A for INA219 calibration/range. "
                         "Use a single value or channel map like 1:16,2:3"))
    p.add_argument("--shunt-ohms", default=str(DEFAULT_SHUNT_OHMS),
                   help=("Shunt resistance in ohms. Use a single value or channel map "
                         "like 1:0.02,2:0.1"))
    p.add_argument("--no-ina", action="store_true",
                   help="Disable INA219 reading (serial only)")
    p.add_argument("--web-port", type=int, default=5000,
                   help="Web dashboard port (default: 5000)")
    p.add_argument("--esp32-host", default="esp32-wor.local",
                   help="ESP32 hostname/IP for trigger (default: esp32-wor.local)")
    p.add_argument("--esp32-port", type=int, default=7777,
                   help="ESP32 UDP trigger port (default: 7777)")
    return p.parse_args()


def parse_channel_list(raw):
    channels = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        channel = int(part)
        if channel not in CHANNEL_ADDRS:
            raise ValueError(f"invalid INA219 channel {channel}")
        if channel not in channels:
            channels.append(channel)
    if not channels:
        raise ValueError("at least one INA219 channel is required")
    return channels


def parse_channel_values(raw, channels, default):
    values = {channel: float(default) for channel in channels}
    raw = str(raw).strip()
    if not raw:
        return values
    if ":" not in raw and "=" not in raw:
        shared = float(raw)
        return {channel: shared for channel in channels}

    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            key, value = part.split(":", 1)
        elif "=" in part:
            key, value = part.split("=", 1)
        else:
            raise ValueError(f"invalid channel value {part!r}")
        channel = int(key.strip())
        if channel not in channels:
            raise ValueError(f"value provided for disabled INA219 channel {channel}")
        values[channel] = float(value.strip())
    return values


def main():
    global esp32_host, esp32_port, ina_sensors, ina_channel_active, ina_highspeed_channels
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
        channel_spec = args.ina_channels or str(args.ina_channel)
        try:
            channels = parse_channel_list(channel_spec)
            highspeed_channels = parse_channel_list(args.highspeed_channels or str(args.ina_channel))
            for channel in highspeed_channels:
                if channel not in channels:
                    raise ValueError(f"high-speed channel {channel} is not in --ina-channels")
            shunt_values = parse_channel_values(args.shunt_ohms, channels, DEFAULT_SHUNT_OHMS)
            max_current_values = parse_channel_values(args.max_current, channels, DEFAULT_MAX_CURRENT_A)
            persisted_calibration = _load_calibration_config()
            persisted_highspeed_channels = _load_highspeed_config()
            for channel, cfg in persisted_calibration.items():
                if channel in channels:
                    shunt_values[channel] = cfg["shunt_ohms"]
                    max_current_values[channel] = cfg["max_current_a"]
            if persisted_highspeed_channels:
                highspeed_channels = [
                    channel for channel in persisted_highspeed_channels
                    if channel in channels
                ] or highspeed_channels
        except ValueError as e:
            raise SystemExit(f"Invalid INA219 configuration: {e}") from e

        detected = {}
        for channel in channels:
            addr = CHANNEL_ADDRS[channel]
            try:
                ina = INA219(
                    address=addr,
                    shunt_ohms=shunt_values[channel],
                    max_current_a=max_current_values[channel],
                    channel=channel,
                )
                detected[channel] = ina
                warning = " overrange-risk" if ina.overrange_expected else ""
                print(f"INA219 found on CH{channel} (0x{addr:02x}), "
                      f"shunt {ina.shunt_ohms:g} ohm, "
                      f"calibrated for {ina.max_current_a:g} A, "
                      f"shunt range +/-{ina.shunt_range_a:g} A{warning}")
            except OSError as e:
                print(f"Warning: INA219 not found on CH{channel} (0x{addr:02x}): {e}")

        if detected:
            detected_highspeed_channels = [
                channel for channel in highspeed_channels
                if channel in detected
            ]
            if not detected_highspeed_channels:
                detected_highspeed_channels = [sorted(detected)[0]]
            active_channel = (
                args.ina_channel
                if args.ina_channel in detected_highspeed_channels
                else detected_highspeed_channels[0]
            )
            with ina_lock:
                ina_sensors = detected
                ina_channel_active = active_channel
                ina_highspeed_channels = detected_highspeed_channels
            print(f"INA219 active chart channel CH{active_channel}, "
                  f"high-speed channels "
                  f"{','.join(f'CH{ch}' for ch in detected_highspeed_channels)}, "
                  f"sampling at {args.sample_rate} Hz total")
            t = threading.Thread(target=ina_thread, daemon=True,
                                 args=(detected_highspeed_channels, args.sample_rate,
                                       csv_writer, csv_lock_obj, csv_file,
                                       stop_event))
            t.start()
        else:
            print("Warning: no INA219 channels found.")
            print("  Continuing without power monitoring. Use --no-ina to suppress.")

    # Start serial reader
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Serial: {args.port} @ {args.baud} baud")
    t = threading.Thread(target=serial_thread, daemon=True,
                         args=(ser, csv_writer, csv_lock_obj, csv_file, stop_event))
    t.start()

    # Start driver sysfs poller
    t = threading.Thread(target=driver_thread, daemon=True, args=(stop_event,))
    t.start()
    if Path(SYSFS_BASE).exists():
        print(f"Driver: monitoring {SYSFS_BASE}")
    else:
        print(f"Driver: {SYSFS_BASE} not found (module not loaded?)")

    # Start logic analyzer detection
    t = threading.Thread(target=la_detect_thread, daemon=True, args=(stop_event,))
    t.start()
    initial_la = _detect_fx2()
    if initial_la:
        with la_lock:
            la_state["available"] = True
            la_state["device"] = initial_la
        print(f"Logic Analyzer: {initial_la} detected")
    else:
        print("Logic Analyzer: no FX2 device found (will keep checking)")

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
