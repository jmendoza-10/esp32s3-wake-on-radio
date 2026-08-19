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

# ── External INA226/INA228 monitor registers and constants ──────────────────

INA226_REG_CONFIG = 0x00
INA226_REG_SHUNT_VOLTAGE = 0x01
INA226_REG_BUS_VOLTAGE = 0x02
INA226_REG_POWER = 0x03
INA226_REG_CURRENT = 0x04
INA226_REG_CALIBRATION = 0x05
INA226_REG_MASK_ENABLE = 0x06
INA226_REG_ALERT_LIMIT = 0x07
INA226_REG_MANUFACTURER_ID = 0xFE
INA226_REG_DIE_ID = 0xFF

INA228_REG_CONFIG = 0x00
INA228_REG_ADC_CONFIG = 0x01
INA228_REG_SHUNT_CAL = 0x02
INA228_REG_SHUNT_VOLTAGE = 0x04
INA228_REG_BUS_VOLTAGE = 0x05
INA228_REG_CURRENT = 0x07
INA228_REG_POWER = 0x08
INA228_REG_MANUFACTURER_ID = 0x3E
INA228_REG_DEVICE_ID = 0x3F

DEFAULT_INA226_ADDRESS = 0x44
DEFAULT_INA228_ADDRESS = 0x45
DEFAULT_INA226_SHUNT_OHMS = 0.002
DEFAULT_INA226_MAX_CURRENT_A = 6.0
DEFAULT_INA226_SAMPLE_RATE_HZ = 1000.0
DEFAULT_INA226_AVERAGES = 64
DEFAULT_INA226_CONVERSION_TIME_US = 1100
DEFAULT_INA228_CONVERSION_TIME_US = 1052
DEFAULT_INA228_ADC_RANGE_MV = 40.96
DEFAULT_INA226_RAIL_MODE = "4v"
DEFAULT_EXTERNAL_MONITOR_TYPE = "auto"
EXTERNAL_MONITOR_CONFIG_PATH = Path("external_monitor_config.json")
INA226_CONFIG_PATH = Path("ina226_config.json")
SYSTEM_STATS_INTERVAL_S = 5.0
INA226_RAIL_MODES = {"4v", "12v", "auto"}
EXTERNAL_MONITOR_TYPES = {"auto", "ina226", "ina228"}
DEFAULT_POWER_CAPTURE_DIR = Path("/tmp/wor-power-captures")
DEFAULT_POWER_CAPTURE_MAX_MB = 512
DEFAULT_POWER_CAPTURE_MAX_SECONDS = 0
DEFAULT_POWER_CAPTURE_QUEUE_SIZE = 50000
POWER_CAPTURE_COLUMNS = [
    "epoch_s",
    "iso_time",
    "source",
    "channel",
    "device_type",
    "address",
    "sample_mode",
    "rail_mode",
    "rail_label",
    "voltage_mv",
    "shunt_uv",
    "current_ua",
    "power_uw",
    "window_avg_current_ua",
    "avg_window_s",
    "cpu_percent",
    "memory_percent",
    "load_1",
]
INA226_AVERAGE_BITS = {
    1: 0b000,
    4: 0b001,
    16: 0b010,
    64: 0b011,
    128: 0b100,
    256: 0b101,
    512: 0b110,
    1024: 0b111,
}
INA226_CONVERSION_TIME_BITS = {
    140: 0b000,
    204: 0b001,
    332: 0b010,
    588: 0b011,
    1100: 0b100,
    2116: 0b101,
    4156: 0b110,
    8244: 0b111,
}
INA228_AVERAGE_BITS = dict(INA226_AVERAGE_BITS)
INA228_CONVERSION_TIME_BITS = {
    50: 0b000,
    84: 0b001,
    150: 0b010,
    280: 0b011,
    540: 0b100,
    1052: 0b101,
    2074: 0b110,
    4120: 0b111,
}
INA228_ADC_RANGE_MV = (40.96, 163.84)


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


class INA226:
    """Minimal INA226 driver for an external high-side current monitor."""

    device_type = "ina226"

    def __init__(self, bus_num=1, address=DEFAULT_INA226_ADDRESS,
                 shunt_ohms=DEFAULT_INA226_SHUNT_OHMS,
                 max_current_a=DEFAULT_INA226_MAX_CURRENT_A,
                 averages=DEFAULT_INA226_AVERAGES,
                 conversion_time_us=DEFAULT_INA226_CONVERSION_TIME_US):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        self.averages = averages
        self.conversion_time_us = conversion_time_us
        self._configure(max_current_a, averages, conversion_time_us)
        self.manufacturer_id = None
        self.die_id = None
        self._read_identity()

    def _write_register(self, reg, value):
        value = int(value) & 0xFFFF
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

    def _read_identity(self):
        try:
            self.manufacturer_id = self._read_register(INA226_REG_MANUFACTURER_ID)
            self.die_id = self._read_register(INA226_REG_DIE_ID)
        except OSError:
            self.manufacturer_id = None
            self.die_id = None

    def reset(self):
        self._write_register(INA226_REG_CONFIG, 1 << 15)
        time.sleep(0.002)
        self._configure(self.max_current_a, self.averages, self.conversion_time_us)
        self._read_identity()

    def _configure(self, max_current_a, averages, conversion_time_us):
        if max_current_a <= 0:
            raise ValueError("max_current_a must be positive")
        if self.shunt_ohms <= 0:
            raise ValueError("shunt_ohms must be positive")
        averages = int(averages)
        conversion_time_us = int(conversion_time_us)
        if averages not in INA226_AVERAGE_BITS:
            raise ValueError("averages must be one of " + ",".join(map(str, INA226_AVERAGE_BITS)))
        if conversion_time_us not in INA226_CONVERSION_TIME_BITS:
            raise ValueError(
                "conversion_time_us must be one of "
                + ",".join(map(str, INA226_CONVERSION_TIME_BITS))
            )

        self.max_current_a = float(max_current_a)
        self.averages = averages
        self.conversion_time_us = conversion_time_us
        self.shunt_range_a = 0.08192 / self.shunt_ohms
        self.overrange_expected = (self.max_current_a * self.shunt_ohms) > 0.08192

        requested_lsb_a = self.max_current_a / 32768.0
        min_lsb_for_cal_a = 0.00512 / (65535 * self.shunt_ohms)
        self.current_lsb_a = max(requested_lsb_a, min_lsb_for_cal_a)
        self.power_lsb_w = 25.0 * self.current_lsb_a
        self.calibration = int(0.00512 / (self.current_lsb_a * self.shunt_ohms))
        self.calibration = max(1, min(65535, self.calibration))

        avg_bits = INA226_AVERAGE_BITS[averages]
        ct_bits = INA226_CONVERSION_TIME_BITS[conversion_time_us]
        config = (avg_bits << 9) | (ct_bits << 6) | (ct_bits << 3) | 0b111
        self._write_register(INA226_REG_CONFIG, config)
        self._write_register(INA226_REG_CALIBRATION, self.calibration)

    def configure(self, address=None, shunt_ohms=None, max_current_a=None,
                  averages=None, conversion_time_us=None):
        if address is not None:
            self.addr = int(address)
        if shunt_ohms is not None:
            shunt_ohms = float(shunt_ohms)
            if shunt_ohms <= 0:
                raise ValueError("shunt_ohms must be positive")
            self.shunt_ohms = shunt_ohms
        if max_current_a is None:
            max_current_a = self.max_current_a
        if averages is None:
            averages = self.averages
        if conversion_time_us is None:
            conversion_time_us = self.conversion_time_us
        self._configure(float(max_current_a), int(averages), int(conversion_time_us))
        self._read_identity()

    def read_all(self):
        if self._read_register(INA226_REG_CALIBRATION) == 0:
            print(f"INA226 0x{self.addr:02x} calibration register cleared; reconfiguring", flush=True)
            self._configure(self.max_current_a, self.averages, self.conversion_time_us)
        voltage_mv = self._read_register(INA226_REG_BUS_VOLTAGE) * 1.25
        current_ua = self._read_register_signed(INA226_REG_CURRENT) * self.current_lsb_a * 1_000_000
        power_uw = self._read_register(INA226_REG_POWER) * self.power_lsb_w * 1_000_000
        shunt_uv = self._read_register_signed(INA226_REG_SHUNT_VOLTAGE) * 2.5
        return voltage_mv, current_ua, power_uw, shunt_uv

    def status(self):
        calibration = self._read_register(INA226_REG_CALIBRATION)
        config = self._read_register(INA226_REG_CONFIG)
        voltage_mv, current_ua, power_uw, shunt_uv = self.read_all()
        return {
            "available": True,
            "device_type": self.device_type,
            "address": f"0x{self.addr:02x}",
            "address_int": self.addr,
            "shunt_ohms": self.shunt_ohms,
            "max_current_a": self.max_current_a,
            "shunt_range_a": self.shunt_range_a,
            "shunt_range_mv": 81.92,
            "current_lsb_ua": self.current_lsb_a * 1_000_000,
            "power_lsb_uw": self.power_lsb_w * 1_000_000,
            "calibration": calibration,
            "expected_calibration": self.calibration,
            "calibrated": calibration != 0,
            "overrange_expected": self.overrange_expected,
            "averages": self.averages,
            "conversion_time_us": self.conversion_time_us,
            "sample_rate_hz": ina226_sample_rate_hz,
            "config": config,
            "voltage_mv": voltage_mv,
            "current_ua": current_ua,
            "power_uw": power_uw,
            "shunt_uv": shunt_uv,
            "manufacturer_id": (
                f"0x{self.manufacturer_id:04x}"
                if self.manufacturer_id is not None else None
            ),
            "die_id": f"0x{self.die_id:04x}" if self.die_id is not None else None,
        }


class INA228:
    """Minimal INA228 driver for an external high-side current monitor."""

    device_type = "ina228"

    def __init__(self, bus_num=1, address=DEFAULT_INA228_ADDRESS,
                 shunt_ohms=DEFAULT_INA226_SHUNT_OHMS,
                 max_current_a=DEFAULT_INA226_MAX_CURRENT_A,
                 averages=DEFAULT_INA226_AVERAGES,
                 conversion_time_us=DEFAULT_INA228_CONVERSION_TIME_US,
                 adc_range_mv=DEFAULT_INA228_ADC_RANGE_MV):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        self.averages = averages
        self.conversion_time_us = conversion_time_us
        self.adc_range_mv = adc_range_mv
        self.manufacturer_id = None
        self.device_id = None
        self._read_identity()
        if self.manufacturer_id != 0x5449 or self.device_id is None or (self.device_id >> 4) != 0x228:
            try:
                self.bus.close()
            except AttributeError:
                pass
            raise OSError(
                f"INA228 identity not detected at 0x{self.addr:02x} "
                f"(mfg={self.manufacturer_id!r}, device={self.device_id!r})"
            )
        self._configure(max_current_a, averages, conversion_time_us, adc_range_mv)

    def _write_register(self, reg, value):
        value = int(value) & 0xFFFF
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

    def _read_u24(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]

    def _read_u20_from_24(self, reg):
        return self._read_u24(reg) >> 4

    def _read_s20_from_24(self, reg):
        raw = self._read_u20_from_24(reg)
        if raw & (1 << 19):
            raw -= 1 << 20
        return raw

    def _read_identity(self):
        try:
            self.manufacturer_id = self._read_register(INA228_REG_MANUFACTURER_ID)
            self.device_id = self._read_register(INA228_REG_DEVICE_ID)
        except OSError:
            self.manufacturer_id = None
            self.device_id = None

    def reset(self):
        self._write_register(INA228_REG_CONFIG, 1 << 15)
        time.sleep(0.002)
        self._read_identity()
        self._configure(
            self.max_current_a, self.averages,
            self.conversion_time_us, self.adc_range_mv,
        )

    def _configure(self, max_current_a, averages, conversion_time_us, adc_range_mv):
        if max_current_a <= 0:
            raise ValueError("max_current_a must be positive")
        if self.shunt_ohms <= 0:
            raise ValueError("shunt_ohms must be positive")
        averages = int(averages)
        conversion_time_us = int(conversion_time_us)
        adc_range_mv = parse_ina228_adc_range_mv(adc_range_mv, DEFAULT_INA228_ADC_RANGE_MV)
        if averages not in INA228_AVERAGE_BITS:
            raise ValueError("averages must be one of " + ",".join(map(str, INA228_AVERAGE_BITS)))
        if conversion_time_us not in INA228_CONVERSION_TIME_BITS:
            raise ValueError(
                "conversion_time_us must be one of "
                + ",".join(map(str, INA228_CONVERSION_TIME_BITS))
            )

        self.max_current_a = float(max_current_a)
        self.averages = averages
        self.conversion_time_us = conversion_time_us
        self.adc_range_mv = adc_range_mv
        self.shunt_range_a = (adc_range_mv / 1000.0) / self.shunt_ohms
        self.overrange_expected = (self.max_current_a * self.shunt_ohms) > (adc_range_mv / 1000.0)

        self.current_lsb_a = self.max_current_a / float(1 << 19)
        self.power_lsb_w = 3.2 * self.current_lsb_a
        calibration = 13107.2e6 * self.current_lsb_a * self.shunt_ohms
        if adc_range_mv == 40.96:
            calibration *= 4
        self.calibration = int(round(calibration))
        self.calibration = max(1, min(0x7FFF, self.calibration))

        range_bit = 1 if adc_range_mv == 40.96 else 0
        avg_bits = INA228_AVERAGE_BITS[averages]
        ct_bits = INA228_CONVERSION_TIME_BITS[conversion_time_us]
        config = range_bit << 4
        adc_config = (0x0B << 12) | (ct_bits << 9) | (ct_bits << 6) | (ct_bits << 3) | avg_bits
        self._write_register(INA228_REG_CONFIG, config)
        self._write_register(INA228_REG_ADC_CONFIG, adc_config)
        self._write_register(INA228_REG_SHUNT_CAL, self.calibration)

    def configure(self, address=None, shunt_ohms=None, max_current_a=None,
                  averages=None, conversion_time_us=None, adc_range_mv=None):
        if address is not None:
            self.addr = int(address)
        if shunt_ohms is not None:
            shunt_ohms = float(shunt_ohms)
            if shunt_ohms <= 0:
                raise ValueError("shunt_ohms must be positive")
            self.shunt_ohms = shunt_ohms
        if max_current_a is None:
            max_current_a = self.max_current_a
        if averages is None:
            averages = self.averages
        if conversion_time_us is None:
            conversion_time_us = self.conversion_time_us
        if adc_range_mv is None:
            adc_range_mv = self.adc_range_mv
        self._configure(float(max_current_a), int(averages), int(conversion_time_us), adc_range_mv)

    def read_all(self):
        if self._read_register(INA228_REG_SHUNT_CAL) == 0:
            print(f"INA228 0x{self.addr:02x} calibration register cleared; reconfiguring", flush=True)
            self._configure(
                self.max_current_a, self.averages,
                self.conversion_time_us, self.adc_range_mv,
            )
        voltage_mv = self._read_u20_from_24(INA228_REG_BUS_VOLTAGE) * 195.3125 / 1000.0
        current_ua = self._read_s20_from_24(INA228_REG_CURRENT) * self.current_lsb_a * 1_000_000
        power_uw = self._read_u24(INA228_REG_POWER) * self.power_lsb_w * 1_000_000
        shunt_lsb_uv = 0.078125 if self.adc_range_mv == 40.96 else 0.3125
        shunt_uv = self._read_s20_from_24(INA228_REG_SHUNT_VOLTAGE) * shunt_lsb_uv
        return voltage_mv, current_ua, power_uw, shunt_uv

    def read_current_ua(self):
        return self._read_s20_from_24(INA228_REG_CURRENT) * self.current_lsb_a * 1_000_000

    def status(self):
        calibration = self._read_register(INA228_REG_SHUNT_CAL)
        config = self._read_register(INA228_REG_CONFIG)
        adc_config = self._read_register(INA228_REG_ADC_CONFIG)
        voltage_mv, current_ua, power_uw, shunt_uv = self.read_all()
        return {
            "available": True,
            "device_type": self.device_type,
            "address": f"0x{self.addr:02x}",
            "address_int": self.addr,
            "shunt_ohms": self.shunt_ohms,
            "max_current_a": self.max_current_a,
            "shunt_range_a": self.shunt_range_a,
            "shunt_range_mv": self.adc_range_mv,
            "adc_range_mv": self.adc_range_mv,
            "current_lsb_ua": self.current_lsb_a * 1_000_000,
            "power_lsb_uw": self.power_lsb_w * 1_000_000,
            "calibration": calibration,
            "expected_calibration": self.calibration,
            "calibrated": calibration != 0,
            "overrange_expected": self.overrange_expected,
            "averages": self.averages,
            "conversion_time_us": self.conversion_time_us,
            "sample_rate_hz": ina226_sample_rate_hz,
            "config": config,
            "adc_config": adc_config,
            "voltage_mv": voltage_mv,
            "current_ua": current_ua,
            "power_uw": power_uw,
            "shunt_uv": shunt_uv,
            "manufacturer_id": (
                f"0x{self.manufacturer_id:04x}"
                if self.manufacturer_id is not None else None
            ),
            "device_id": f"0x{self.device_id:04x}" if self.device_id is not None else None,
            "die_id": f"0x{self.device_id:04x}" if self.device_id is not None else None,
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
ina_enabled_channels: list[int] = []
ina_lock = threading.Lock()

# Optional external INA226/INA228 monitor for the modem rail.
ina226_sensor: INA226 | INA228 | None = None
ina226_enabled = False
ina226_target_address = DEFAULT_INA226_ADDRESS
ina226_device_type = DEFAULT_EXTERNAL_MONITOR_TYPE
ina226_shunt_ohms = DEFAULT_INA226_SHUNT_OHMS
ina226_max_current_a = DEFAULT_INA226_MAX_CURRENT_A
ina226_averages = DEFAULT_INA226_AVERAGES
ina226_conversion_time_us = DEFAULT_INA226_CONVERSION_TIME_US
ina226_adc_range_mv = DEFAULT_INA228_ADC_RANGE_MV
ina226_sample_rate_hz = DEFAULT_INA226_SAMPLE_RATE_HZ
ina226_rail_mode = DEFAULT_INA226_RAIL_MODE
ina226_last_error = None
ina226_lock = threading.Lock()
ina226_history: list[dict] = []
ina226_avg_samples: list[tuple[float, float]] = []

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

# Raspberry Pi host health, sampled from /proc.
system_stats = {
    "available": False,
    "cpu_percent": 0.0,
    "memory_used_mb": 0.0,
    "memory_total_mb": 0.0,
    "memory_percent": 0.0,
    "load_1": 0.0,
    "uptime_s": 0.0,
}
system_lock = threading.Lock()

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


def broadcast_external_monitor_status(data: dict):
    broadcast_sse("external_monitor", data)
    broadcast_sse("ina226", data)


def broadcast_external_monitor_power(data: dict):
    broadcast_sse("external_monitor_power", data)
    broadcast_sse("ina226_power", data)


# ── On-demand power capture logger ────────────────────────────────────────

class PowerCaptureManager:
    """Queue-backed CSV capture for long power logs started from the dashboard."""

    def __init__(self):
        self.lock = threading.Lock()
        self.capture_dir = DEFAULT_POWER_CAPTURE_DIR
        self.max_file_bytes = DEFAULT_POWER_CAPTURE_MAX_MB * 1024 * 1024
        self.max_duration_s = DEFAULT_POWER_CAPTURE_MAX_SECONDS
        self.queue_size = DEFAULT_POWER_CAPTURE_QUEUE_SIZE
        self.state = "idle"
        self.active = False
        self.capture_id = None
        self.started_at = None
        self.started_mono = None
        self.stopped_at = None
        self.tmp_path = None
        self.final_path = None
        self.rows = 0
        self.dropped = 0
        self.size_bytes = 0
        self.error = None
        self.stop_reason = None
        self._queue = None
        self._stop_event = None
        self._writer_thread = None

    def configure(self, capture_dir, max_mb, max_seconds, queue_size):
        with self.lock:
            self.capture_dir = Path(capture_dir)
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            self.max_file_bytes = int(float(max_mb) * 1024 * 1024) if max_mb else 0
            self.max_duration_s = float(max_seconds or 0)
            self.queue_size = max(1000, int(queue_size or DEFAULT_POWER_CAPTURE_QUEUE_SIZE))

    def start(self):
        with self.lock:
            if self.state in ("running", "stopping"):
                raise RuntimeError("power capture already running")

            self.capture_dir.mkdir(parents=True, exist_ok=True)
            started = datetime.now(timezone.utc)
            stamp = started.strftime("%Y%m%dT%H%M%S") + f"{started.microsecond // 1000:03d}Z"
            capture_id = f"power_{stamp}"
            self.capture_id = capture_id
            self.started_at = started
            self.started_mono = time.monotonic()
            self.stopped_at = None
            self.tmp_path = self.capture_dir / f"{capture_id}.tmp.csv"
            self.final_path = self.capture_dir / f"{capture_id}.csv"
            self.rows = 0
            self.dropped = 0
            self.size_bytes = 0
            self.error = None
            self.stop_reason = None
            self._queue = queue.Queue(maxsize=self.queue_size)
            self._stop_event = threading.Event()
            self.state = "running"
            self.active = True
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                args=(capture_id, self.tmp_path, self.final_path, self._queue, self._stop_event),
                daemon=True,
            )
            self._writer_thread.start()
            snapshot = self._snapshot_locked()

        broadcast_sse("power_capture", snapshot)
        return snapshot

    def stop(self, wait_s=2.0):
        with self.lock:
            if self.state not in ("running", "stopping"):
                snapshot = self._snapshot_locked()
                broadcast_sse("power_capture", snapshot)
                return snapshot
            self._request_stop_locked("user")
            thread = self._writer_thread

        if thread is not None and wait_s:
            thread.join(timeout=wait_s)

        snapshot = self.status()
        broadcast_sse("power_capture", snapshot)
        return snapshot

    def record(self, row):
        if not self.active:
            return False

        with self.lock:
            if self.state != "running" or self._queue is None:
                return False
            if self.max_duration_s and self.started_mono is not None:
                if time.monotonic() - self.started_mono >= self.max_duration_s:
                    self._request_stop_locked("max duration")
                    return False
            q = self._queue

        try:
            q.put_nowait(row)
            return True
        except queue.Full:
            with self.lock:
                self.dropped += 1
            return False

    def status(self):
        with self.lock:
            return self._snapshot_locked()

    def list_files(self, limit=20):
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        files = sorted(self.capture_dir.glob("power_*.csv"), reverse=True)
        result = []
        for path in files[:limit]:
            stat = path.stat()
            result.append({
                "id": path.stem,
                "name": path.name,
                "size_bytes": stat.st_size,
                "size_kb": round(stat.st_size / 1024, 1),
                "created": datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc).isoformat(),
                "download_url": f"/api/captures/{path.stem}/download",
            })
        return result

    def download_path(self, capture_id):
        safe_id = Path(str(capture_id)).name
        if safe_id.endswith(".csv"):
            safe_id = safe_id[:-4]
        path = self.capture_dir / f"{safe_id}.csv"
        if not path.exists():
            return None
        return path

    def _request_stop_locked(self, reason):
        self.active = False
        self.stop_reason = reason
        if self.state == "running":
            self.state = "stopping"
        if self._stop_event is not None:
            self._stop_event.set()

    def _snapshot_locked(self):
        elapsed_s = 0.0
        if self.started_mono is not None and self.state in ("running", "stopping"):
            elapsed_s = max(0.0, time.monotonic() - self.started_mono)
        elif self.started_at is not None and self.stopped_at is not None:
            elapsed_s = max(0.0, (self.stopped_at - self.started_at).total_seconds())

        qsize = self._queue.qsize() if self._queue is not None else 0
        filename = self.final_path.name if self.final_path else None
        if self.state in ("running", "stopping") and self.tmp_path is not None:
            filename = self.tmp_path.name

        return {
            "state": self.state,
            "running": self.state in ("running", "stopping"),
            "capture_id": self.capture_id,
            "started_at": self.started_at.isoformat() if self.started_at else None,
            "stopped_at": self.stopped_at.isoformat() if self.stopped_at else None,
            "elapsed_s": round(elapsed_s, 1),
            "rows": self.rows,
            "dropped": self.dropped,
            "queue_depth": qsize,
            "size_bytes": self.size_bytes,
            "filename": filename,
            "file": str(self.final_path) if self.final_path else None,
            "download_url": (
                f"/api/captures/{self.capture_id}/download"
                if self.capture_id and self.state == "complete" else None
            ),
            "capture_dir": str(self.capture_dir),
            "max_file_bytes": self.max_file_bytes,
            "max_duration_s": self.max_duration_s,
            "error": self.error,
            "stop_reason": self.stop_reason,
        }

    def _writer_loop(self, capture_id, tmp_path, final_path, q, stop_event):
        last_flush = time.monotonic()
        snapshot = None
        try:
            with tmp_path.open("w", newline="") as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=POWER_CAPTURE_COLUMNS,
                    extrasaction="ignore",
                )
                writer.writeheader()
                while True:
                    if stop_event.is_set() and q.empty():
                        break

                    try:
                        row = q.get(timeout=0.25)
                    except queue.Empty:
                        row = None

                    if row is not None:
                        writer.writerow(_format_power_capture_row(row))
                        q.task_done()
                        with self.lock:
                            self.rows += 1

                    now = time.monotonic()
                    should_flush = (now - last_flush >= 1.0) or q.empty()
                    if should_flush:
                        csv_file.flush()
                        size = csv_file.tell()
                        with self.lock:
                            self.size_bytes = size
                            if (
                                self.state == "running"
                                and self.max_file_bytes
                                and size >= self.max_file_bytes
                            ):
                                self._request_stop_locked("max file size")
                        last_flush = now

                csv_file.flush()
                size = csv_file.tell()
                with self.lock:
                    self.size_bytes = size

            tmp_path.replace(final_path)
            with self.lock:
                if self.capture_id == capture_id:
                    self.state = "complete"
                    self.stopped_at = datetime.now(timezone.utc)
                    self.size_bytes = final_path.stat().st_size
                    self.active = False
                    snapshot = self._snapshot_locked()
        except Exception as exc:
            with self.lock:
                if self.capture_id == capture_id:
                    self.state = "error"
                    self.error = str(exc)
                    self.stopped_at = datetime.now(timezone.utc)
                    self.active = False
                    snapshot = self._snapshot_locked()

        if snapshot is not None:
            broadcast_sse("power_capture", snapshot)


def _format_power_capture_row(row):
    return {
        column: "" if row.get(column) is None else row.get(column)
        for column in POWER_CAPTURE_COLUMNS
    }


power_capture = PowerCaptureManager()


def record_power_capture_sample(source, ts, channel=None, device_type=None,
                                address=None, sample_mode=None, rail_mode=None,
                                rail_label=None, voltage_mv=None, shunt_uv=None,
                                current_ua=None, power_uw=None,
                                window_avg_current_ua=None, avg_window_s=None):
    if not power_capture.active:
        return

    with system_lock:
        system_snapshot = dict(system_stats)

    power_capture.record({
        "epoch_s": f"{ts:.6f}",
        "iso_time": datetime.fromtimestamp(ts, tz=timezone.utc).isoformat(),
        "source": source,
        "channel": "" if channel is None else channel,
        "device_type": device_type or source,
        "address": address,
        "sample_mode": sample_mode,
        "rail_mode": rail_mode,
        "rail_label": rail_label,
        "voltage_mv": None if voltage_mv is None else f"{float(voltage_mv):.3f}",
        "shunt_uv": None if shunt_uv is None else f"{float(shunt_uv):.3f}",
        "current_ua": None if current_ua is None else f"{float(current_ua):.3f}",
        "power_uw": None if power_uw is None else f"{float(power_uw):.3f}",
        "window_avg_current_ua": (
            None if window_avg_current_ua is None else f"{float(window_avg_current_ua):.3f}"
        ),
        "avg_window_s": avg_window_s,
        "cpu_percent": system_snapshot.get("cpu_percent"),
        "memory_percent": system_snapshot.get("memory_percent"),
        "load_1": system_snapshot.get("load_1"),
    })


# ── INA219 sampling thread ─────────────────────────────────────────────────

def _channel_status_locked(channel, ina):
    try:
        status = ina.status()
        status["channel"] = channel
        status["enabled"] = True
        return status
    except OSError as exc:
        return {
            "available": False,
            "enabled": True,
            "channel": channel,
            "address": f"0x{ina.addr:02x}",
            "error": str(exc),
        }


def _channel_disabled_status_locked(channel, ina):
    return {
        "available": False,
        "enabled": False,
        "channel": channel,
        "address": f"0x{ina.addr:02x}",
        "error": "disabled",
    }


def _ina_status_payload_locked():
    if not ina_sensors:
        return {"available": False, "error": "INA219 disabled", "channels": []}
    channels = [
        (
            _channel_status_locked(channel, ina)
            if channel in ina_enabled_channels
            else _channel_disabled_status_locked(channel, ina)
        )
        for channel, ina in sorted(ina_sensors.items())
    ]
    return {
        "available": any(ch.get("available") and ch.get("enabled") for ch in channels),
        "active_channel": ina_channel_active,
        "enabled_channels": list(ina_enabled_channels),
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


def parse_i2c_address(raw):
    if isinstance(raw, int):
        address = raw
    else:
        text = str(raw).strip().lower()
        if not text:
            raise ValueError("address is required")
        address = int(text, 0)
    if address < 0x03 or address > 0x77:
        raise ValueError("address must be a 7-bit I2C address from 0x03 to 0x77")
    return address


def parse_ina226_rail_mode(raw):
    text = str(raw).strip().lower().replace(" ", "")
    aliases = {
        "4": "4v",
        "4v": "4v",
        "4.0": "4v",
        "4.0v": "4v",
        "12": "12v",
        "12v": "12v",
        "12.0": "12v",
        "12.0v": "12v",
        "auto": "auto",
        "dynamic": "auto",
    }
    mode = aliases.get(text)
    if mode not in INA226_RAIL_MODES:
        raise ValueError("rail_mode must be 4v, 12v, or auto")
    return mode


def _ina226_rail_label(voltage_mv=None):
    if ina226_rail_mode == "4v":
        return "4V"
    if ina226_rail_mode == "12v":
        return "12V"
    if voltage_mv is None:
        return "AUTO"
    voltage_v = float(voltage_mv) / 1000.0
    if voltage_v < 6.5:
        return "4V"
    if voltage_v >= 8.0:
        return "12V"
    return "VIN"


def parse_external_monitor_type(raw):
    text = str(raw if raw is not None else DEFAULT_EXTERNAL_MONITOR_TYPE).strip().lower()
    aliases = {
        "": DEFAULT_EXTERNAL_MONITOR_TYPE,
        "auto": "auto",
        "detect": "auto",
        "external": "auto",
        "ina226": "ina226",
        "226": "ina226",
        "ina228": "ina228",
        "228": "ina228",
    }
    device_type = aliases.get(text)
    if device_type not in EXTERNAL_MONITOR_TYPES:
        raise ValueError("device_type must be auto, ina226, or ina228")
    return device_type


def parse_ina228_adc_range_mv(raw, current):
    if raw in (None, ""):
        return float(current)
    text = str(raw).strip().lower().replace("mv", "")
    value = float(text)
    for choice in INA228_ADC_RANGE_MV:
        if abs(value - choice) < 0.01:
            return choice
    raise ValueError("adc_range_mv must be 40.96 or 163.84")


def _monitor_device_label(device_type):
    if device_type == "ina226":
        return "INA226"
    if device_type == "ina228":
        return "INA228"
    return "External"


def _monitor_conversion_choices(device_type):
    return INA228_CONVERSION_TIME_BITS if device_type == "ina228" else INA226_CONVERSION_TIME_BITS


def _monitor_default_conversion_time(device_type):
    return DEFAULT_INA228_CONVERSION_TIME_US if device_type == "ina228" else DEFAULT_INA226_CONVERSION_TIME_US


def _coerce_monitor_conversion_time(device_type, conversion_time_us):
    choices = _monitor_conversion_choices(device_type)
    conversion_time_us = int(conversion_time_us)
    if conversion_time_us in choices:
        return conversion_time_us
    return min(choices, key=lambda choice: abs(choice - conversion_time_us))


def _monitor_shunt_range_mv(device_type):
    if device_type == "ina228":
        return ina226_adc_range_mv
    return 81.92


def _read_i2c_register16(bus_num, address, reg):
    import smbus2
    bus = smbus2.SMBus(bus_num)
    try:
        data = bus.read_i2c_block_data(address, reg, 2)
        return (data[0] << 8) | data[1]
    finally:
        try:
            bus.close()
        except AttributeError:
            pass


def _detect_external_monitor_type(address, bus_num=1):
    try:
        manufacturer_id = _read_i2c_register16(bus_num, address, INA228_REG_MANUFACTURER_ID)
        device_id = _read_i2c_register16(bus_num, address, INA228_REG_DEVICE_ID)
        if manufacturer_id == 0x5449 and (device_id >> 4) == 0x228:
            return "ina228"
    except OSError:
        pass

    try:
        manufacturer_id = _read_i2c_register16(bus_num, address, INA226_REG_MANUFACTURER_ID)
        die_id = _read_i2c_register16(bus_num, address, INA226_REG_DIE_ID)
        if manufacturer_id == 0x5449 and (die_id >> 4) == 0x226:
            return "ina226"
    except OSError:
        pass

    return None


def _parse_choice(payload, key, choices, current):
    if key not in payload or payload[key] in (None, ""):
        return current
    value = int(payload[key])
    if value not in choices:
        raise ValueError(f"{key} must be one of " + ",".join(map(str, choices)))
    return value


def _parse_positive_float(payload, key, current):
    parsed = _parse_optional_positive_float(payload, key)
    return current if parsed is None else parsed


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


def _load_enabled_config():
    try:
        with CALIBRATION_CONFIG_PATH.open() as f:
            data = json.load(f)
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None

    raw_channels = data.get("enabled_channels")
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
    return channels


def _save_calibration_config_locked():
    data = {
        "enabled_channels": list(ina_enabled_channels),
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


def _ina226_address_warning(address):
    if address in CHANNEL_ADDRS.values():
        return "address overlaps Waveshare HAT range 0x40-0x43"
    return None


def _load_ina226_config():
    config_path = (
        EXTERNAL_MONITOR_CONFIG_PATH
        if EXTERNAL_MONITOR_CONFIG_PATH.exists()
        else INA226_CONFIG_PATH
    )
    try:
        with config_path.open() as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Warning: could not load {config_path}: {exc}", flush=True)
        return {}

    if not isinstance(data, dict):
        return {}
    loaded = {}
    try:
        if "enabled" in data:
            loaded["enabled"] = bool(data["enabled"])
        if "device_type" in data:
            loaded["device_type"] = parse_external_monitor_type(data["device_type"])
        elif "monitor_type" in data:
            loaded["device_type"] = parse_external_monitor_type(data["monitor_type"])
        if "address" in data:
            loaded["address"] = parse_i2c_address(data["address"])
        if "shunt_ohms" in data:
            loaded["shunt_ohms"] = float(data["shunt_ohms"])
        if "max_current_a" in data:
            loaded["max_current_a"] = float(data["max_current_a"])
        if "averages" in data:
            loaded["averages"] = int(data["averages"])
        if "conversion_time_us" in data:
            loaded["conversion_time_us"] = int(data["conversion_time_us"])
        if "sample_rate_hz" in data:
            loaded["sample_rate_hz"] = float(data["sample_rate_hz"])
        if "rail_mode" in data:
            loaded["rail_mode"] = parse_ina226_rail_mode(data["rail_mode"])
        if "adc_range_mv" in data:
            loaded["adc_range_mv"] = parse_ina228_adc_range_mv(
                data["adc_range_mv"], DEFAULT_INA228_ADC_RANGE_MV,
            )
    except (TypeError, ValueError) as exc:
        print(f"Warning: invalid {config_path}: {exc}", flush=True)
        return {}
    return loaded


def _save_ina226_config_locked():
    data = {
        "enabled": ina226_enabled,
        "device_type": ina226_device_type,
        "address": f"0x{ina226_target_address:02x}",
        "shunt_ohms": ina226_shunt_ohms,
        "max_current_a": ina226_max_current_a,
        "averages": ina226_averages,
        "conversion_time_us": ina226_conversion_time_us,
        "adc_range_mv": ina226_adc_range_mv,
        "sample_rate_hz": ina226_sample_rate_hz,
        "rail_mode": ina226_rail_mode,
    }
    tmp_path = EXTERNAL_MONITOR_CONFIG_PATH.with_suffix(".json.tmp")
    tmp_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    tmp_path.replace(EXTERNAL_MONITOR_CONFIG_PATH)


def _ina226_unavailable_payload_locked(error=None):
    address = ina226_target_address
    warning = _ina226_address_warning(address)
    device_type = ina226_device_type
    shunt_range_mv = _monitor_shunt_range_mv(device_type)
    payload = {
        "available": False,
        "enabled": ina226_enabled,
        "device_type": device_type,
        "requested_device_type": ina226_device_type,
        "device_label": _monitor_device_label(device_type),
        "address": f"0x{address:02x}",
        "address_int": address,
        "shunt_ohms": ina226_shunt_ohms,
        "max_current_a": ina226_max_current_a,
        "shunt_range_a": (shunt_range_mv / 1000.0) / ina226_shunt_ohms if ina226_shunt_ohms > 0 else None,
        "shunt_range_mv": shunt_range_mv,
        "adc_range_mv": ina226_adc_range_mv,
        "averages": ina226_averages,
        "conversion_time_us": ina226_conversion_time_us,
        "sample_rate_hz": ina226_sample_rate_hz,
        "rail_mode": ina226_rail_mode,
        "rail_label": _ina226_rail_label(),
        "error": error or ina226_last_error or (
            "External monitor disabled" if not ina226_enabled else "External monitor not detected"
        ),
    }
    if warning:
        payload["warning"] = warning
    return payload


def _ina226_status_payload_locked():
    if not ina226_enabled:
        return _ina226_unavailable_payload_locked("External monitor disabled")
    if ina226_sensor is None:
        return _ina226_unavailable_payload_locked()
    status = ina226_sensor.status()
    actual_device_type = status.get("device_type", ina226_device_type)
    status["enabled"] = True
    status["device_type"] = actual_device_type
    status["requested_device_type"] = ina226_device_type
    status["device_label"] = _monitor_device_label(actual_device_type)
    status.setdefault("adc_range_mv", ina226_adc_range_mv if actual_device_type == "ina228" else None)
    status["rail_mode"] = ina226_rail_mode
    status["rail_label"] = _ina226_rail_label(status.get("voltage_mv"))
    warning = _ina226_address_warning(ina226_sensor.addr)
    if warning:
        status["warning"] = warning
    return status


def _configure_ina226_locked(payload, reset=False):
    global ina226_sensor, ina226_enabled, ina226_target_address, ina226_shunt_ohms
    global ina226_max_current_a, ina226_averages, ina226_conversion_time_us
    global ina226_sample_rate_hz, ina226_rail_mode, ina226_last_error
    global ina226_device_type, ina226_adc_range_mv

    if payload.get("enabled") is False:
        ina226_enabled = False
        ina226_sensor = None
        ina226_last_error = None
        _save_ina226_config_locked()
        return _ina226_unavailable_payload_locked("External monitor disabled")

    address = parse_i2c_address(payload.get("address", ina226_target_address))
    requested_device_type = parse_external_monitor_type(payload.get("device_type", ina226_device_type))
    shunt_ohms = _parse_positive_float(payload, "shunt_ohms", ina226_shunt_ohms)
    max_current_a = _parse_positive_float(payload, "max_current_a", ina226_max_current_a)
    averages = _parse_choice(payload, "averages", INA226_AVERAGE_BITS, ina226_averages)
    candidate_device_type = requested_device_type
    if candidate_device_type == "auto":
        candidate_device_type = getattr(ina226_sensor, "device_type", "ina226")
    raw_conversion_time_us = payload.get("conversion_time_us", ina226_conversion_time_us)
    if raw_conversion_time_us in (None, ""):
        raw_conversion_time_us = ina226_conversion_time_us
    conversion_time_us = _coerce_monitor_conversion_time(
        candidate_device_type,
        int(raw_conversion_time_us),
    )
    adc_range_mv = parse_ina228_adc_range_mv(payload.get("adc_range_mv", ina226_adc_range_mv), ina226_adc_range_mv)
    sample_rate_hz = _parse_positive_float(payload, "sample_rate_hz", ina226_sample_rate_hz)
    rail_mode = parse_ina226_rail_mode(payload.get("rail_mode", ina226_rail_mode))
    if sample_rate_hz > 1000:
        raise ValueError("sample_rate_hz must be <= 1000")

    ina226_enabled = True
    ina226_target_address = address
    ina226_device_type = requested_device_type
    ina226_shunt_ohms = shunt_ohms
    ina226_max_current_a = max_current_a
    ina226_averages = averages
    ina226_conversion_time_us = conversion_time_us
    ina226_adc_range_mv = adc_range_mv
    ina226_sample_rate_hz = sample_rate_hz
    ina226_rail_mode = rail_mode

    try:
        actual_device_type = requested_device_type
        if actual_device_type == "auto":
            actual_device_type = _detect_external_monitor_type(address)
            if actual_device_type is None:
                raise OSError(f"No INA226/INA228 detected at 0x{address:02x}")
        conversion_time_us = _coerce_monitor_conversion_time(actual_device_type, conversion_time_us)
        ina226_conversion_time_us = conversion_time_us

        needs_new_sensor = (
            ina226_sensor is None
            or ina226_sensor.addr != address
            or getattr(ina226_sensor, "device_type", None) != actual_device_type
        )
        if needs_new_sensor:
            if actual_device_type == "ina228":
                ina226_sensor = INA228(
                    address=address,
                    shunt_ohms=shunt_ohms,
                    max_current_a=max_current_a,
                    averages=averages,
                    conversion_time_us=conversion_time_us,
                    adc_range_mv=adc_range_mv,
                )
            else:
                ina226_sensor = INA226(
                    address=address,
                    shunt_ohms=shunt_ohms,
                    max_current_a=max_current_a,
                    averages=averages,
                    conversion_time_us=conversion_time_us,
                )
        else:
            if actual_device_type == "ina228":
                ina226_sensor.configure(
                    shunt_ohms=shunt_ohms,
                    max_current_a=max_current_a,
                    averages=averages,
                    conversion_time_us=conversion_time_us,
                    adc_range_mv=adc_range_mv,
                )
            else:
                ina226_sensor.configure(
                    shunt_ohms=shunt_ohms,
                    max_current_a=max_current_a,
                    averages=averages,
                    conversion_time_us=conversion_time_us,
                )
        if reset:
            ina226_sensor.reset()
        ina226_last_error = None
        _save_ina226_config_locked()
        status = _ina226_status_payload_locked()
        status["status"] = "ok"
        return status
    except OSError as exc:
        ina226_sensor = None
        ina226_last_error = str(exc)
        _save_ina226_config_locked()
        payload_out = _ina226_unavailable_payload_locked(str(exc))
        payload_out["status"] = "error"
        return payload_out


def _trimmed_mean(values, trim_fraction=0.1):
    if not values:
        return 0.0
    ordered = sorted(values)
    trim = int(len(ordered) * trim_fraction)
    if trim > 0 and len(ordered) > trim * 2:
        ordered = ordered[trim:-trim]
    return sum(ordered) / len(ordered)


def _read_external_current_samples_locked(sample_count, sample_delay_s):
    values = []
    for index in range(sample_count):
        if hasattr(ina226_sensor, "read_current_ua"):
            values.append(float(ina226_sensor.read_current_ua()))
        else:
            values.append(float(ina226_sensor.read_all()[1]))
        if sample_delay_s and index < sample_count - 1:
            time.sleep(sample_delay_s)
    return values


def _parse_reference_current_ua(payload):
    if "reference_current_ua" in payload and payload["reference_current_ua"] not in (None, ""):
        reference_current_ua = float(payload["reference_current_ua"])
    elif "reference_current_ma" in payload and payload["reference_current_ma"] not in (None, ""):
        reference_current_ua = float(payload["reference_current_ma"]) * 1000.0
    else:
        raise ValueError("reference_current_ma is required")
    if not (0.0 < abs(reference_current_ua) < 10_000_000.0):
        raise ValueError("reference current must be nonzero and less than 10 A")
    return reference_current_ua


def _reconfigure_existing_external_monitor_locked():
    actual_device_type = getattr(ina226_sensor, "device_type", ina226_device_type)
    if actual_device_type == "ina228":
        ina226_sensor.configure(
            shunt_ohms=ina226_shunt_ohms,
            max_current_a=ina226_max_current_a,
            averages=ina226_averages,
            conversion_time_us=ina226_conversion_time_us,
            adc_range_mv=ina226_adc_range_mv,
        )
    else:
        ina226_sensor.configure(
            shunt_ohms=ina226_shunt_ohms,
            max_current_a=ina226_max_current_a,
            averages=ina226_averages,
            conversion_time_us=ina226_conversion_time_us,
        )


def _calibrate_ina226_shunt_locked(payload):
    global ina226_shunt_ohms
    if not ina226_enabled or ina226_sensor is None:
        status = _ina226_unavailable_payload_locked()
        status["status"] = "error"
        return status

    reference_current_ua = _parse_reference_current_ua(payload)
    sample_count = int(payload.get("samples", 128))
    sample_count = max(16, min(512, sample_count))
    sample_delay_s = float(payload.get("sample_delay_s", 0.001))
    sample_delay_s = max(0.0, min(0.02, sample_delay_s))

    old_shunt_ohms = ina226_shunt_ohms
    values = _read_external_current_samples_locked(sample_count, sample_delay_s)
    measured_current_ua = _trimmed_mean(values)
    if abs(measured_current_ua) < 1000.0:
        raise ValueError("measured current is too close to zero for shunt scale calibration")

    new_shunt_ohms = old_shunt_ohms * (abs(measured_current_ua) / abs(reference_current_ua))
    if not (0.00001 <= new_shunt_ohms <= 1.0):
        raise ValueError(f"calculated shunt {new_shunt_ohms:g} ohm is out of range")

    ina226_shunt_ohms = new_shunt_ohms
    _reconfigure_existing_external_monitor_locked()
    with avg_lock:
        ina226_avg_samples.clear()
    _save_ina226_config_locked()
    status = _ina226_status_payload_locked()
    status["status"] = "ok"
    status["calibration_sample_count"] = sample_count
    status["calibration_reference_current_ua"] = reference_current_ua
    status["calibration_measured_current_ua"] = measured_current_ua
    status["calibration_old_shunt_ohms"] = old_shunt_ohms
    status["calibration_new_shunt_ohms"] = new_shunt_ohms
    if measured_current_ua * reference_current_ua < 0:
        status["calibration_warning"] = "measured polarity is opposite the reference current"
    return status


def ina226_thread(csv_writer, csv_lock, csv_file, stop_event):
    global ina226_last_error
    sse_interval = 0.1
    status_sse_interval = 1.0
    aux_read_interval = 0.25
    last_status_sse = 0
    last_power_sse = 0
    last_aux_read = 0
    cached_voltage_mv = None
    cached_power_uw = None
    cached_shunt_uv = None
    last_csv_flush = time.monotonic()
    batch_points = []

    while not stop_event.is_set():
        t0 = time.monotonic()
        sample_rate = max(1.0, ina226_sample_rate_hz)
        stop_wait = None
        try:
            with ina226_lock:
                if not ina226_enabled or ina226_sensor is None:
                    if time.time() - last_status_sse >= 2.0:
                        broadcast_external_monitor_status(_ina226_status_payload_locked())
                        last_status_sse = time.time()
                    stop_wait = min(1.0, 1.0 / sample_rate)
                else:
                    device_type = getattr(ina226_sensor, "device_type", ina226_device_type)
                    device_label = _monitor_device_label(device_type)
                    read_aux = (
                        cached_voltage_mv is None
                        or device_type != "ina228"
                        or time.monotonic() - last_aux_read >= aux_read_interval
                    )
                    if read_aux:
                        voltage_mv, current_ua, power_uw, shunt_uv = ina226_sensor.read_all()
                        cached_voltage_mv = voltage_mv
                        cached_power_uw = power_uw
                        cached_shunt_uv = shunt_uv
                        last_aux_read = time.monotonic()
                        sample_mode = "full"
                    else:
                        current_ua = ina226_sensor.read_current_ua()
                        voltage_mv = cached_voltage_mv
                        power_uw = (
                            voltage_mv * current_ua / 1000.0
                            if voltage_mv is not None else cached_power_uw
                        )
                        shunt_uv = (
                            current_ua * ina226_sensor.shunt_ohms
                            if getattr(ina226_sensor, "shunt_ohms", None) else cached_shunt_uv
                        )
                        sample_mode = "fast_current"
                    ts = time.time()
                    now = f"{ts:.6f}"

                    with avg_lock:
                        ina226_avg_samples.append((ts, current_ua))
                        cutoff = ts - AVG_WINDOW_S
                        while ina226_avg_samples and ina226_avg_samples[0][0] < cutoff:
                            ina226_avg_samples.pop(0)
                        window_avg_current_ua = (
                            sum(sample[1] for sample in ina226_avg_samples) / len(ina226_avg_samples)
                            if ina226_avg_samples else current_ua
                        )

                    point = {
                        "ts": round(ts, 6),
                        "monitor": device_type,
                        "device_type": device_type,
                        "requested_device_type": ina226_device_type,
                        "device_label": device_label,
                        "sample_mode": sample_mode,
                        "address": f"0x{ina226_sensor.addr:02x}",
                        "voltage_mv": round(voltage_mv, 1),
                        "current_ua": round(current_ua, 1),
                        "power_uw": round(power_uw, 1),
                        "shunt_uv": round(shunt_uv, 1),
                        "adc_range_mv": getattr(ina226_sensor, "adc_range_mv", None),
                        "window_avg_current_ua": round(window_avg_current_ua, 1),
                        "avg_window_s": AVG_WINDOW_S,
                        "rail_mode": ina226_rail_mode,
                        "rail_label": _ina226_rail_label(voltage_mv),
                    }
                    record_power_capture_sample(
                        "external_monitor",
                        ts,
                        device_type=device_type,
                        address=point["address"],
                        sample_mode=sample_mode,
                        rail_mode=ina226_rail_mode,
                        rail_label=point["rail_label"],
                        voltage_mv=voltage_mv,
                        shunt_uv=shunt_uv,
                        current_ua=current_ua,
                        power_uw=power_uw,
                        window_avg_current_ua=window_avg_current_ua,
                        avg_window_s=AVG_WINDOW_S,
                    )

                    with csv_lock:
                        csv_writer.writerow([
                            now, device_label, "", "",
                            f"{voltage_mv:.1f}", f"{current_ua:.1f}", f"{power_uw:.1f}",
                        ])
                        if time.monotonic() - last_csv_flush >= 0.1:
                            csv_file.flush()
                            last_csv_flush = time.monotonic()

                    batch_points.append(point)

                    if ts - last_power_sse >= sse_interval:
                        points = batch_points
                        batch_points = []
                        with history_lock:
                            ina226_history.extend(points)
                            if len(ina226_history) > MAX_HISTORY:
                                del ina226_history[:len(ina226_history) - MAX_HISTORY]

                        payload = dict(points[-1])
                        payload["samples"] = points
                        broadcast_external_monitor_power(payload)
                        last_power_sse = ts

                    if ts - last_status_sse >= status_sse_interval:
                        broadcast_external_monitor_status(_ina226_status_payload_locked())
                        last_status_sse = ts

        except OSError as exc:
            with ina226_lock:
                ina226_last_error = str(exc)
                broadcast_external_monitor_status(_ina226_unavailable_payload_locked(str(exc)))

        if stop_wait is not None:
            stop_event.wait(stop_wait)
            continue

        elapsed = time.monotonic() - t0
        remaining = (1.0 / max(1.0, sample_rate)) - elapsed
        if remaining > 0:
            stop_event.wait(remaining)


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
                    if channel in ina_sensors and channel in ina_enabled_channels
                ]
                if not current_highspeed_channels:
                    current_highspeed_channels = [
                        channel for channel in highspeed_channels
                        if channel in ina_sensors and channel in ina_enabled_channels
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
            point["avg_window_s"] = AVG_WINDOW_S
            record_power_capture_sample(
                "ina219",
                ts,
                channel=active_channel,
                device_type="ina219",
                address=f"0x{ina.addr:02x}",
                sample_mode="highspeed",
                voltage_mv=voltage_mv,
                current_ua=current_ua,
                power_uw=power_uw,
                window_avg_current_ua=window_avg_current_ua,
                avg_window_s=AVG_WINDOW_S,
            )

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
            stop_event.wait(0.1)

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


def _read_cpu_totals():
    with open("/proc/stat") as f:
        fields = f.readline().split()
    if not fields or fields[0] != "cpu":
        raise OSError("invalid /proc/stat")
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def _read_meminfo():
    data = {}
    with open("/proc/meminfo") as f:
        for line in f:
            key, value = line.split(":", 1)
            data[key] = int(value.strip().split()[0])
    total_kb = data.get("MemTotal", 0)
    available_kb = data.get("MemAvailable", 0)
    used_kb = max(0, total_kb - available_kb)
    return total_kb, used_kb


def _read_system_stats(prev_cpu):
    total, idle = _read_cpu_totals()
    prev_total, prev_idle = prev_cpu
    delta_total = total - prev_total
    delta_idle = idle - prev_idle
    cpu_percent = 0.0
    if delta_total > 0:
        cpu_percent = max(0.0, min(100.0, (1.0 - (delta_idle / delta_total)) * 100.0))

    mem_total_kb, mem_used_kb = _read_meminfo()
    memory_percent = (mem_used_kb / mem_total_kb * 100.0) if mem_total_kb else 0.0

    with open("/proc/loadavg") as f:
        load_1 = float(f.read().split()[0])
    with open("/proc/uptime") as f:
        uptime_s = float(f.read().split()[0])

    return {
        "available": True,
        "cpu_percent": round(cpu_percent, 1),
        "memory_used_mb": round(mem_used_kb / 1024.0, 1),
        "memory_total_mb": round(mem_total_kb / 1024.0, 1),
        "memory_percent": round(memory_percent, 1),
        "load_1": round(load_1, 2),
        "uptime_s": round(uptime_s, 1),
    }, (total, idle)


def system_stats_thread(stop_event):
    """Publish Raspberry Pi CPU/memory statistics periodically."""
    try:
        prev_cpu = _read_cpu_totals()
    except OSError as exc:
        with system_lock:
            system_stats.update({"available": False, "error": str(exc)})
        return

    while not stop_event.is_set():
        stop_event.wait(SYSTEM_STATS_INTERVAL_S)
        if stop_event.is_set():
            break
        try:
            snapshot, prev_cpu = _read_system_stats(prev_cpu)
        except (OSError, ValueError) as exc:
            snapshot = {"available": False, "error": str(exc)}
        with system_lock:
            system_stats.update(snapshot)
            payload = dict(system_stats)
        broadcast_sse("system", payload)


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


@app.route("/api/captures/status")
def power_capture_status():
    """Return current on-demand power capture state."""
    return jsonify(power_capture.status())


@app.route("/api/captures/start", methods=["POST"])
def power_capture_start():
    """Start a queue-backed power CSV capture."""
    try:
        status = power_capture.start()
        print(f"Power capture started: {status.get('filename')}", flush=True)
        return jsonify(status)
    except RuntimeError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 409
    except OSError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 500


@app.route("/api/captures/stop", methods=["POST"])
def power_capture_stop():
    """Stop the active power CSV capture and finalize the file."""
    status = power_capture.stop()
    print(f"Power capture stop requested: {status.get('filename')}", flush=True)
    return jsonify(status)


@app.route("/api/captures")
def power_captures():
    """List finalized power capture CSV files."""
    return jsonify(power_capture.list_files())


@app.route("/api/captures/<capture_id>/download")
def power_capture_download(capture_id):
    """Download a finalized power capture CSV."""
    filepath = power_capture.download_path(capture_id)
    if filepath is None:
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
        if channel not in ina_enabled_channels:
            raise ValueError(f"INA219 CH{channel} is disabled")
        if channel not in channels:
            channels.append(channel)
    if not channels:
        raise ValueError("at least one high-speed channel is required")
    return channels


def _parse_enabled_payload_channels(payload):
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


@app.route("/api/ina219/enabled", methods=["POST"])
def ina219_enabled():
    """Set INA219 channels that are actively sampled/status-polled."""
    global ina_enabled_channels, ina_highspeed_channels, ina_channel_active
    if not ina_sensors:
        return jsonify({"status": "error", "error": "INA219 disabled"}), 404
    payload = request.get_json(silent=True) or {}
    if not ina_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "INA219 reader busy"}), 503
    try:
        channels = _parse_enabled_payload_channels(payload)
        ina_enabled_channels = channels
        ina_highspeed_channels = [
            channel for channel in ina_highspeed_channels
            if channel in ina_enabled_channels
        ]
        if ina_enabled_channels and not ina_highspeed_channels:
            ina_highspeed_channels = [ina_enabled_channels[0]]
        if ina_channel_active not in ina_enabled_channels:
            ina_channel_active = ina_enabled_channels[0] if ina_enabled_channels else None
        _save_calibration_config_locked()
        status = _ina_status_payload_locked()
        status["status"] = "ok"
        broadcast_sse("ina219", status)
        enabled_label = (
            ",".join(f"CH{channel}" for channel in channels)
            if channels else "none"
        )
        print(f"INA219 enabled channels set to {enabled_label}", flush=True)
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
            "enabled_channels": list(ina_enabled_channels),
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


@app.route("/api/external-monitor")
@app.route("/api/ina226")
def ina226_info():
    """Return live status for the external INA226/INA228 monitor."""
    if not ina226_lock.acquire(timeout=1.0):
        return jsonify({"available": False, "error": "External monitor reader busy"}), 503
    try:
        return jsonify(_ina226_status_payload_locked())
    except OSError as exc:
        return jsonify(_ina226_unavailable_payload_locked(str(exc))), 503
    finally:
        ina226_lock.release()


@app.route("/api/external-monitor/history")
@app.route("/api/ina226/history")
def ina226_history_info():
    """Return recent external monitor power history for initial chart load."""
    with history_lock:
        return jsonify(list(ina226_history))


@app.route("/api/external-monitor/configure", methods=["POST"])
@app.route("/api/ina226/configure", methods=["POST"])
def ina226_configure():
    """Apply external monitor address, calibration, conversion, and polling settings."""
    payload = request.get_json(silent=True) or {}
    reset = bool(payload.pop("reset", False))
    if not ina226_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "External monitor reader busy"}), 503
    try:
        status = _configure_ina226_locked(payload, reset=reset)
        broadcast_external_monitor_status(status)
        if status.get("status") == "ok":
            device_label = status.get("device_label", "External monitor")
            print(
                f"{device_label} configured at {status['address']}, "
                f"shunt {status['shunt_ohms']:g} ohm, "
                f"max {status['max_current_a']:g} A, "
                f"{status['sample_rate_hz']:g} Hz",
                flush=True,
            )
            return jsonify(status)
        print(f"Warning: external monitor configure failed: {status.get('error')}", flush=True)
        return jsonify(status), 503
    except ValueError as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    finally:
        ina226_lock.release()


@app.route("/api/external-monitor/calibrate-shunt", methods=["POST"])
@app.route("/api/ina226/calibrate-shunt", methods=["POST"])
def ina226_calibrate_shunt():
    """Adjust external monitor shunt scale from a known nonzero reference current."""
    payload = request.get_json(silent=True) or {}
    if not ina226_lock.acquire(timeout=1.0):
        return jsonify({"status": "error", "error": "External monitor reader busy"}), 503
    try:
        status = _calibrate_ina226_shunt_locked(payload)
        broadcast_external_monitor_status(status)
        if status.get("status") == "ok":
            print(
                f"External monitor shunt calibrated "
                f"{status.get('calibration_old_shunt_ohms', 0):g} -> "
                f"{status.get('calibration_new_shunt_ohms', 0):g} ohm",
                flush=True,
            )
            return jsonify(status)
        return jsonify(status), 503
    except (ValueError, OSError) as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    finally:
        ina226_lock.release()


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


@app.route("/api/system")
def system_info():
    """Return current Raspberry Pi CPU/memory status."""
    with system_lock:
        return jsonify(dict(system_stats))


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
    p.add_argument("--no-ina226", action="store_true",
                   help="Disable optional external INA226/INA228 monitor")
    p.add_argument("--external-monitor-type", default=DEFAULT_EXTERNAL_MONITOR_TYPE,
                   choices=sorted(EXTERNAL_MONITOR_TYPES),
                   help="External monitor device type: auto, ina226, or ina228")
    p.add_argument("--external-monitor-address", default=None,
                   help="External monitor I2C address; overrides --ina226-address when set")
    p.add_argument("--ina226-address", default=f"0x{DEFAULT_INA226_ADDRESS:02x}",
                   help="Legacy external monitor I2C address, e.g. 0x44 (default: 0x44)")
    p.add_argument("--ina226-shunt-ohms", type=float, default=DEFAULT_INA226_SHUNT_OHMS,
                   help="External monitor shunt resistance in ohms (default: 0.002)")
    p.add_argument("--ina226-max-current", type=float, default=DEFAULT_INA226_MAX_CURRENT_A,
                   help="External monitor expected max current in A for calibration (default: 6.0)")
    p.add_argument("--ina226-sample-rate", type=float, default=DEFAULT_INA226_SAMPLE_RATE_HZ,
                   help="External monitor polling samples per second (default: 1000)")
    p.add_argument("--ina226-averages", type=int, default=DEFAULT_INA226_AVERAGES,
                   choices=sorted(INA226_AVERAGE_BITS),
                   help="External monitor hardware averaging sample count")
    p.add_argument("--ina226-conversion-time-us", type=int,
                   default=DEFAULT_INA226_CONVERSION_TIME_US,
                   choices=sorted(set(INA226_CONVERSION_TIME_BITS) | set(INA228_CONVERSION_TIME_BITS)),
                   help="External monitor shunt and bus conversion time in microseconds")
    p.add_argument("--ina228-adc-range-mv", type=float, default=DEFAULT_INA228_ADC_RANGE_MV,
                   choices=sorted(INA228_ADC_RANGE_MV),
                   help="INA228 shunt ADC range in mV (default: 40.96)")
    p.add_argument("--ina226-rail", default=DEFAULT_INA226_RAIL_MODE,
                   choices=sorted(INA226_RAIL_MODES),
                   help="External monitor rail label mode: 4v, 12v, or auto")
    p.add_argument("--capture-dir", default=str(DEFAULT_POWER_CAPTURE_DIR),
                   help="Directory for on-demand power capture CSV files")
    p.add_argument("--capture-max-mb", type=float, default=DEFAULT_POWER_CAPTURE_MAX_MB,
                   help="Maximum on-demand power capture size in MB (0 disables limit)")
    p.add_argument("--capture-max-seconds", type=float,
                   default=DEFAULT_POWER_CAPTURE_MAX_SECONDS,
                   help="Maximum on-demand power capture duration in seconds (0 disables limit)")
    p.add_argument("--capture-queue-size", type=int,
                   default=DEFAULT_POWER_CAPTURE_QUEUE_SIZE,
                   help="Maximum queued power samples before capture rows are dropped")
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
    global ina_enabled_channels
    global ina226_enabled, ina226_target_address, ina226_shunt_ohms
    global ina226_max_current_a, ina226_averages, ina226_conversion_time_us
    global ina226_sample_rate_hz, ina226_sensor, ina226_last_error
    global ina226_device_type, ina226_adc_range_mv
    args = parse_args()
    esp32_host = args.esp32_host
    esp32_port = args.esp32_port
    power_capture.configure(
        args.capture_dir,
        args.capture_max_mb,
        args.capture_max_seconds,
        args.capture_queue_size,
    )

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
            persisted_enabled_channels = _load_enabled_config()
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
            detected_enabled_channels = [
                channel for channel in (persisted_enabled_channels if persisted_enabled_channels is not None else channels)
                if channel in detected
            ]
            detected_highspeed_channels = [
                channel for channel in highspeed_channels
                if channel in detected and channel in detected_enabled_channels
            ]
            if detected_enabled_channels and not detected_highspeed_channels:
                detected_highspeed_channels = [sorted(detected)[0]]
            active_channel = (
                args.ina_channel
                if args.ina_channel in detected_highspeed_channels
                else (detected_highspeed_channels[0] if detected_highspeed_channels else None)
            )
            with ina_lock:
                ina_sensors = detected
                ina_enabled_channels = detected_enabled_channels
                ina_channel_active = active_channel
                ina_highspeed_channels = detected_highspeed_channels
            enabled_label = (
                ",".join(f"CH{ch}" for ch in detected_enabled_channels)
                if detected_enabled_channels else "none"
            )
            highspeed_label = (
                ",".join(f"CH{ch}" for ch in detected_highspeed_channels)
                if detected_highspeed_channels else "none"
            )
            active_label = f"CH{active_channel}" if active_channel is not None else "none"
            print(f"INA219 active chart channel {active_label}, "
                  f"enabled channels {enabled_label}, "
                  f"high-speed channels {highspeed_label}, "
                  f"sampling at {args.sample_rate} Hz total")
            t = threading.Thread(target=ina_thread, daemon=True,
                                 args=(detected_highspeed_channels, args.sample_rate,
                                       csv_writer, csv_lock_obj, csv_file,
                                       stop_event))
            t.start()
        else:
            print("Warning: no INA219 channels found.")
            print("  Continuing without power monitoring. Use --no-ina to suppress.")

    # Start optional INA226/INA228 external rail monitor. This can be absent at
    # boot and later configured from the dashboard without disturbing the HAT.
    if not args.no_ina226:
        try:
            external_address = args.external_monitor_address or args.ina226_address
            ina226_defaults = {
                "enabled": True,
                "device_type": args.external_monitor_type,
                "address": parse_i2c_address(external_address),
                "shunt_ohms": args.ina226_shunt_ohms,
                "max_current_a": args.ina226_max_current,
                "averages": args.ina226_averages,
                "conversion_time_us": args.ina226_conversion_time_us,
                "adc_range_mv": args.ina228_adc_range_mv,
                "sample_rate_hz": args.ina226_sample_rate,
                "rail_mode": args.ina226_rail,
            }
            persisted_ina226 = _load_ina226_config()
            ina226_defaults.update(persisted_ina226)
            with ina226_lock:
                status = _configure_ina226_locked(ina226_defaults)
            if status.get("available"):
                device_label = status.get("device_label", "External monitor")
                print(f"{device_label} found at {status['address']}, "
                      f"shunt {status['shunt_ohms']:g} ohm, "
                      f"calibrated for {status['max_current_a']:g} A, "
                      f"polling at {status['sample_rate_hz']:g} Hz")
            else:
                print(f"Warning: external monitor not available at {status['address']}: "
                      f"{status.get('error')}")
                if status.get("warning"):
                    print(f"  external monitor config warning: {status['warning']}")
        except ValueError as e:
            ina226_enabled = False
            ina226_sensor = None
            ina226_last_error = str(e)
            print(f"Invalid external monitor configuration: {e}")

        t = threading.Thread(target=ina226_thread, daemon=True,
                             args=(csv_writer, csv_lock_obj, csv_file, stop_event))
        t.start()
    else:
        ina226_enabled = False
        print("External monitor: disabled")

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

    # Start Raspberry Pi host health sampler
    t = threading.Thread(target=system_stats_thread, daemon=True, args=(stop_event,))
    t.start()
    print(f"RPi system stats: sampling every {SYSTEM_STATS_INTERVAL_S:g}s")

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

    print(f"Legacy stream logging to: {out_path}")
    print(f"On-demand power captures: {power_capture.capture_dir}")
    print(f"ESP32 target: {esp32_host}:{esp32_port}")
    print(f"Dashboard: http://0.0.0.0:{args.web_port}")
    print("-" * 60)

    try:
        app.run(host="0.0.0.0", port=args.web_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        power_capture.stop(wait_s=5.0)
        ser.close()
        csv_file.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
