#!/usr/bin/env python3
"""
trigger_espnow.py — Send ESP-NOW trigger frames via a second ESP32 connected
to the RPi over USB serial.

The companion ESP32 runs a simple firmware that listens for a 'T' byte on
UART and responds by broadcasting an ESP-NOW frame with the magic byte.

If no companion ESP32 is available, this script can also use scapy to inject
raw frames (requires monitor mode on the RPi Wi-Fi interface).

Usage:
    # Via companion ESP32 on USB serial
    python3 trigger_espnow.py --mode serial --port /dev/ttyUSB0

    # Via scapy raw injection (requires sudo + monitor mode)
    sudo python3 trigger_espnow.py --mode inject --iface wlan0 --channel 1

Requires: pyserial (serial mode) or scapy (inject mode)
"""

import argparse
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="ESP-NOW trigger sender")
    p.add_argument("--mode", choices=["serial", "inject"], default="serial",
                   help="Trigger mode (default: serial)")
    p.add_argument("--port", default="/dev/ttyUSB0",
                   help="Serial port for companion ESP32 (default: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--magic", type=lambda x: int(x, 0), default=0xA5,
                   help="Magic trigger byte (default: 0xA5)")
    p.add_argument("--count", type=int, default=10,
                   help="Number of trigger frames to send (default: 10)")
    p.add_argument("--interval", type=float, default=0.1,
                   help="Interval between frames in seconds (default: 0.1)")
    # For inject mode
    p.add_argument("--iface", default="wlan0", help="Wi-Fi interface (inject mode)")
    p.add_argument("--channel", type=int, default=1, help="Wi-Fi channel (inject mode)")
    return p.parse_args()


def trigger_serial(args):
    import serial
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Sending {args.count} triggers via {args.port}")

    for i in range(args.count):
        ser.write(b'T')
        print(f"  [{i+1}/{args.count}] Sent trigger command")
        time.sleep(args.interval)

    ser.close()
    print("Done.")


def trigger_inject(args):
    try:
        from scapy.all import RadioTap, Dot11, Dot11Deauth, sendp, conf
    except ImportError:
        print("Error: scapy required for inject mode. pip install scapy")
        sys.exit(1)

    # Build an ESP-NOW-like action frame
    # ESP-NOW uses vendor-specific action frames (category 127, OUI 18:fe:34)
    broadcast = b'\xff' * 6
    src_mac = b'\xaa\xbb\xcc\xdd\xee\xff'

    # Vendor-specific action frame for ESP-NOW
    espnow_oui = bytes([0x18, 0xfe, 0x34])
    payload = bytes([args.magic]) + b'\x00' * 7

    from scapy.all import Dot11, RadioTap
    frame = (
        RadioTap() /
        Dot11(type=0, subtype=13,  # Action frame
              addr1=broadcast.hex(':'),
              addr2=src_mac.hex(':'),
              addr3=broadcast.hex(':')) /
        bytes([127, 4]) + espnow_oui + payload  # Category=vendor, Action=4
    )

    conf.iface = args.iface
    print(f"Injecting {args.count} ESP-NOW frames on {args.iface} ch{args.channel}")

    for i in range(args.count):
        sendp(frame, verbose=False)
        print(f"  [{i+1}/{args.count}] Injected")
        time.sleep(args.interval)

    print("Done.")


def main():
    args = parse_args()

    if args.mode == "serial":
        trigger_serial(args)
    else:
        trigger_inject(args)


if __name__ == "__main__":
    main()
