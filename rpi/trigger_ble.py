#!/usr/bin/env python3
"""
trigger_ble.py — Broadcast BLE advertisements with the trigger device name
so the ESP32-S3 BLE wake strategy detects it.

Usage:
    python3 trigger_ble.py --name WOR_TRIG --duration 30

Requires: bleak (pip install bleak) and bluez on the RPi
"""

import argparse
import asyncio
import struct
import subprocess
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="BLE trigger advertiser")
    p.add_argument("--name", default="WOR_TRIG",
                   help="BLE device name to advertise (default: WOR_TRIG)")
    p.add_argument("--duration", type=int, default=30,
                   help="Advertisement duration in seconds (default: 30)")
    p.add_argument("--interval-ms", type=int, default=100,
                   help="Advertisement interval in ms (default: 100)")
    return p.parse_args()


def build_adv_data(name):
    """Build raw HCI advertisement data with the complete local name."""
    name_bytes = name.encode("utf-8")
    # AD structure: length, type (0x09 = Complete Local Name), data
    ad = bytes([len(name_bytes) + 1, 0x09]) + name_bytes
    # Flags: General Discoverable + BR/EDR Not Supported
    flags = bytes([2, 0x01, 0x06])
    return flags + ad


def advertise_hcitool(args):
    """Use hcitool and hciconfig for BLE advertising (no bleak dependency)."""
    adv_data = build_adv_data(args.name)

    # Enable LE advertising
    print(f"Starting BLE advertisement: name='{args.name}', "
          f"duration={args.duration}s")

    # Reset adapter
    subprocess.run(["sudo", "hciconfig", "hci0", "down"], capture_output=True)
    subprocess.run(["sudo", "hciconfig", "hci0", "up"], capture_output=True)

    # Set advertisement data
    # HCI command: OGF 0x08, OCF 0x0008 (LE Set Advertising Data)
    hex_data = adv_data.hex()
    padded = hex_data + "00" * (31 - len(adv_data))
    cmd = f"sudo hcitool -i hci0 cmd 0x08 0x0008 {len(adv_data):02x} " + \
          " ".join(padded[i:i+2] for i in range(0, len(padded), 2))
    subprocess.run(cmd, shell=True, capture_output=True)

    # Set advertisement parameters (min/max interval, type=ADV_IND)
    min_int = args.interval_ms * 16 // 10  # Convert ms to 0.625ms units
    max_int = min_int
    min_lo = min_int & 0xFF
    min_hi = (min_int >> 8) & 0xFF
    max_lo = max_int & 0xFF
    max_hi = (max_int >> 8) & 0xFF
    subprocess.run(
        f"sudo hcitool -i hci0 cmd 0x08 0x0006 {min_lo:02x} {min_hi:02x} "
        f"{max_lo:02x} {max_hi:02x} 00 00 00 00 00 00 00 00 00 07 00",
        shell=True, capture_output=True
    )

    # Enable advertising
    subprocess.run(
        "sudo hcitool -i hci0 cmd 0x08 0x000a 01",
        shell=True, capture_output=True
    )

    print(f"Advertising '{args.name}'...")

    try:
        for remaining in range(args.duration, 0, -1):
            print(f"\r  {remaining}s remaining", end="", flush=True)
            time.sleep(1)
        print()
    except KeyboardInterrupt:
        print("\nInterrupted.")

    # Disable advertising
    subprocess.run(
        "sudo hcitool -i hci0 cmd 0x08 0x000a 00",
        shell=True, capture_output=True
    )
    print("Advertising stopped.")


def main():
    args = parse_args()
    advertise_hcitool(args)


if __name__ == "__main__":
    main()
