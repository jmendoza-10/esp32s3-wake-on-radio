#!/usr/bin/env python3
"""
trigger_dtim.py — Send a UDP packet to wake the ESP32-S3 from DTIM power save.

The ESP32 stays associated with the AP and listens at DTIM beacons. The AP
buffers this UDP packet and delivers it at the next DTIM interval.

Usage:
    python3 trigger_dtim.py --host 192.168.0.203 --port 7777

    # Repeated triggers for latency testing
    python3 trigger_dtim.py --host 192.168.0.203 --count 10 --interval 5
"""

import argparse
import socket
import time


def parse_args():
    p = argparse.ArgumentParser(description="DTIM trigger (UDP)")
    p.add_argument("--host", required=True,
                   help="ESP32-S3 IP address")
    p.add_argument("--port", type=int, default=7777,
                   help="UDP trigger port (default: 7777)")
    p.add_argument("--count", type=int, default=1,
                   help="Number of triggers to send (default: 1)")
    p.add_argument("--interval", type=float, default=1.0,
                   help="Interval between triggers in seconds (default: 1.0)")
    return p.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    payload = b"WAKE"

    print(f"Sending UDP triggers to {args.host}:{args.port}")

    for i in range(args.count):
        sock.sendto(payload, (args.host, args.port))
        print(f"  [{i+1}/{args.count}] Sent '{payload.decode()}' to "
              f"{args.host}:{args.port}")
        if i < args.count - 1:
            time.sleep(args.interval)

    sock.close()
    print("Done.")


if __name__ == "__main__":
    main()
