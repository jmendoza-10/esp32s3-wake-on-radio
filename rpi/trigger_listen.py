#!/usr/bin/env python3
"""
trigger_listen.py — Create a temporary soft-AP with the magic SSID so
the ESP32-S3 periodic listen strategy detects it during a Wi-Fi scan.

Uses NetworkManager (nmcli) on the RPi to create a hotspot.

Usage:
    sudo python3 trigger_listen.py --ssid WOR_TRIGGER --duration 30

Requires: NetworkManager (nmcli)
"""

import argparse
import subprocess
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="Listen strategy trigger (soft-AP)")
    p.add_argument("--ssid", default="WOR_TRIGGER",
                   help="Magic SSID to broadcast (default: WOR_TRIGGER)")
    p.add_argument("--duration", type=int, default=30,
                   help="How long to broadcast in seconds (default: 30)")
    p.add_argument("--channel", type=int, default=1,
                   help="Wi-Fi channel (default: 1)")
    p.add_argument("--iface", default="wlan0",
                   help="Wireless interface (default: wlan0)")
    return p.parse_args()


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Command failed: {' '.join(cmd)}")
        print(result.stderr)
    return result


def main():
    args = parse_args()
    conn_name = "wor-trigger"

    print(f"Creating soft-AP: SSID={args.ssid}, channel={args.channel}, "
          f"duration={args.duration}s")

    # Remove any existing trigger connection
    run(["nmcli", "connection", "delete", conn_name])

    # Create a hotspot with the magic SSID
    result = run([
        "nmcli", "connection", "add",
        "type", "wifi",
        "ifname", args.iface,
        "con-name", conn_name,
        "autoconnect", "no",
        "ssid", args.ssid,
        "wifi.mode", "ap",
        "wifi.band", "bg",
        "wifi.channel", str(args.channel),
        "ipv4.method", "shared",
    ])
    if result.returncode != 0:
        sys.exit(1)

    # Bring it up
    result = run(["nmcli", "connection", "up", conn_name])
    if result.returncode != 0:
        run(["nmcli", "connection", "delete", conn_name])
        sys.exit(1)

    print(f"Broadcasting '{args.ssid}' on channel {args.channel}...")

    try:
        for remaining in range(args.duration, 0, -1):
            print(f"\r  {remaining}s remaining", end="", flush=True)
            time.sleep(1)
        print()
    except KeyboardInterrupt:
        print("\nInterrupted.")

    # Clean up
    print("Stopping soft-AP...")
    run(["nmcli", "connection", "down", conn_name])
    run(["nmcli", "connection", "delete", conn_name])
    print("Done.")


if __name__ == "__main__":
    main()
