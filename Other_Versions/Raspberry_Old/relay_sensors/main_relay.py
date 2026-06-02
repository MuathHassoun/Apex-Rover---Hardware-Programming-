#!/usr/bin/env python3
"""
main_relay.py — Apex Rover Raspberry Pi Sensor Relay Entry Point

Current architecture:
  Mega -> Raspberry Pi USB Serial -> ESP32 HTTP /sensor_update -> Mobile App WebSocket

Raspberry Pi does NOT drive the robot.
All commands come from:
  Mobile App -> ESP32 -> Mega / UNO
"""

import time
import signal
import sys

from sensor_relay import SensorRelay


def main():
    print("=" * 55)
    print("Apex Rover — Raspberry Pi Sensor Relay")
    print("Manual-only system")
    print("=" * 55)

    relay = SensorRelay()

    if not relay.start():
        print("[ERROR] Could not start sensor relay.")
        print("[HINT] Check Mega USB cable and MEGA_PORT in config.py")
        sys.exit(1)

    def shutdown(sig, frame):
        print("\n[INFO] Shutting down sensor relay...")
        relay.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("[OK] Sensor relay running.")
    print("[INFO] Waiting for SENSOR lines from Mega...")
    print("[INFO] Press Ctrl+C to stop.")

    while True:
        time.sleep(1)

        if not relay.is_running():
            print("[ERROR] Sensor relay stopped unexpectedly.")
            sys.exit(1)


if __name__ == "__main__":
    main()