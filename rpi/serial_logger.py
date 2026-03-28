#!/usr/bin/env python3
"""
serial_logger.py — RPi-side serial + INA219 power logger for ESP32-S3
wake-on-radio telemetry.

Captures:
  - PWR| state transitions from ESP32-S3 UART (via RPi GPIO15 RX)
  - Continuous current/voltage readings from Waveshare 4-Ch Current/Power
    Monitor HAT (INA219 over I2C)

Writes everything to a single timestamped CSV for analysis.

Usage:
    python3 serial_logger.py --port /dev/ttyAMA0 --ina-channel 1 --sample-rate 100

Requires:
    pip install pyserial smbus2

Hardware setup:
    - Waveshare 4-Ch HAT stacked on RPi 4 GPIO header
    - ESP32-S3 powered through HAT channel (IN+ <- 3.3V, IN- -> ESP32 3V3)
    - ESP32-S3 TX (GPIO43) -> RPi RX (GPIO15, pin 10)
    - ESP32-S3 GND -> RPi GND
    - USB NOT connected to ESP32-S3
"""

import argparse
import csv
import struct
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import smbus2

# ── INA219 registers and constants ──────────────────────────────────────────

_REG_CONFIG = 0x00
_REG_SHUNT_VOLTAGE = 0x01
_REG_BUS_VOLTAGE = 0x02
_REG_POWER = 0x03
_REG_CURRENT = 0x04
_REG_CALIBRATION = 0x05

# Waveshare 4-Ch HAT I2C addresses (detected on this HAT: 0x40-0x43)
CHANNEL_ADDRS = {
    1: 0x40,
    2: 0x41,
    3: 0x42,
    4: 0x43,
}

# Waveshare HAT uses 0.1 ohm shunt resistors
SHUNT_RESISTANCE_OHM = 0.1


class INA219:
    """Minimal INA219 driver for the Waveshare Current/Power Monitor HAT."""

    def __init__(self, bus_num=1, address=0x40, shunt_ohms=0.1, max_current_a=0.4):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        self._configure(max_current_a)

    def _write_register(self, reg, value):
        buf = [(value >> 8) & 0xFF, value & 0xFF]
        self.bus.write_i2c_block_data(self.addr, reg, buf)

    def _read_register(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        raw = (data[0] << 8) | data[1]
        return raw

    def _read_register_signed(self, reg):
        raw = self._read_register(reg)
        if raw & 0x8000:
            raw -= 1 << 16
        return raw

    def _configure(self, max_current_a):
        # Config: bus voltage range 16V, PGA /1 (40mV), 12-bit samples,
        # continuous shunt + bus measurement
        # BRNG=0 (16V), PG=00 (40mV/±0.4A with 0.1R), BADC=1111 (128 samples avg),
        # SADC=1111 (128 samples avg), MODE=111 (continuous shunt+bus)
        # For fast sampling use BADC=0011 (12-bit no avg), SADC=0011
        config = (
            (0 << 13)     # BRNG: 16V bus range
            | (0b00 << 11)  # PGA: /1, 40mV range (±400mA with 0.1R shunt)
            | (0b0011 << 7) # BADC: 12-bit, no averaging (532 us)
            | (0b0011 << 3) # SADC: 12-bit, no averaging (532 us)
            | 0b111         # MODE: continuous shunt + bus
        )
        self._write_register(_REG_CONFIG, config)

        # Calibration register: Cal = trunc(0.04096 / (current_lsb * shunt_ohms))
        # Choose current_lsb for good resolution: max_current / 2^15
        self.current_lsb_a = max_current_a / 32768.0
        cal = int(0.04096 / (self.current_lsb_a * self.shunt_ohms))
        self._write_register(_REG_CALIBRATION, cal)

    def read_current_ua(self):
        """Read current in microamps."""
        raw = self._read_register_signed(_REG_CURRENT)
        return raw * self.current_lsb_a * 1_000_000

    def read_bus_voltage_mv(self):
        """Read bus voltage in millivolts."""
        raw = self._read_register(_REG_BUS_VOLTAGE)
        # Bits [15:3] contain the voltage, LSB = 4mV
        return ((raw >> 3) & 0x1FFF) * 4.0

    def read_shunt_voltage_uv(self):
        """Read shunt voltage in microvolts (for debug)."""
        raw = self._read_register_signed(_REG_SHUNT_VOLTAGE)
        return raw * 10.0  # LSB = 10uV

    def read_all(self):
        """Return (bus_voltage_mv, current_ua, power_uw)."""
        voltage_mv = self.read_bus_voltage_mv()
        current_ua = self.read_current_ua()
        power_uw = voltage_mv * current_ua / 1000.0
        return voltage_mv, current_ua, power_uw


# ── Argument parsing ────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="ESP32-S3 serial + INA219 power logger")
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
    return p.parse_args()


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    out_path = Path(args.out)
    write_header = not out_path.exists()
    sample_interval = 1.0 / args.sample_rate

    csv_lock = threading.Lock()
    csv_file = open(out_path, "a", newline="")
    writer = csv.writer(csv_file)

    if write_header:
        writer.writerow([
            "rpi_timestamp", "type", "esp_timestamp_us",
            "state", "voltage_mv", "current_ua", "power_uw",
        ])
        csv_file.flush()

    # ── INA219 sampling thread ──────────────────────────────────────────
    stop_event = threading.Event()

    def ina_thread(ina):
        while not stop_event.is_set():
            t0 = time.monotonic()
            try:
                voltage_mv, current_ua, power_uw = ina.read_all()
                now = datetime.now(timezone.utc).isoformat()

                with csv_lock:
                    writer.writerow([
                        now, "INA219", "", "",
                        f"{voltage_mv:.1f}", f"{current_ua:.1f}",
                        f"{power_uw:.1f}",
                    ])
                    csv_file.flush()

                # Print a compact line — current is the most useful at a glance
                print(f"\033[2m[INA219] {voltage_mv:7.1f} mV  "
                      f"{current_ua:10.1f} uA  "
                      f"{power_uw:10.1f} uW\033[0m", end="\r")

            except OSError:
                pass  # I2C bus busy, skip sample

            elapsed = time.monotonic() - t0
            remaining = sample_interval - elapsed
            if remaining > 0:
                stop_event.wait(remaining)

    # ── Start INA219 ────────────────────────────────────────────────────
    ina = None
    if not args.no_ina:
        addr = CHANNEL_ADDRS[args.ina_channel]
        try:
            ina = INA219(address=addr)
            print(f"INA219 found on CH{args.ina_channel} (0x{addr:02x}), "
                  f"sampling at {args.sample_rate} Hz")
            t = threading.Thread(target=ina_thread, args=(ina,), daemon=True)
            t.start()
        except OSError as e:
            print(f"Warning: INA219 not found on 0x{addr:02x}: {e}")
            print("  Continuing with serial only. Use --no-ina to suppress.")

    # ── Serial reader (main thread) ─────────────────────────────────────
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Serial: {args.port} @ {args.baud} baud")
    print(f"Logging to: {out_path}")
    print("-" * 60)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Newline to clear the INA219 \r line, then print serial data
            print(f"\n{line}")

            now = datetime.now(timezone.utc).isoformat()

            if line.startswith("PWR|"):
                parts = line.split("|")
                if len(parts) == 3:
                    with csv_lock:
                        writer.writerow([
                            now, "STATE", parts[1], parts[2],
                            "", "", "",
                        ])
                        csv_file.flush()

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        stop_event.set()
        ser.close()
        csv_file.close()
        print("Done.")


if __name__ == "__main__":
    main()
