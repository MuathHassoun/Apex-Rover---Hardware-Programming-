#!/usr/bin/env python3
"""
main_relay.py — Apex Rover Raspberry Pi entry point.

Responsibilities (simplified, no AUTO mode):
  1. Connect to Mega USB Serial.
  2. Read sensor data pushed by the Mega (SENSOR: lines).
  3. Forward every SENSOR: line to the ESP32 via HTTP POST /sensor.
  4. The ESP32 broadcasts the data to the mobile app via WebSocket.

The Raspberry Pi no longer drives the robot autonomously.
All motion commands come from the mobile app -> ESP32 -> Mega/UNO.
"""

import time
import signal
import sys
from sensor_relay import SensorRelay


def main():
    print("=" * 50)
    print("Apex Rover — Raspberry Pi Sensor Relay")
    print("=" * 50)

    relay = SensorRelay()

    if not relay.start():
        print("[ERROR] Could not start sensor relay. Check USB cable to Mega.")
        sys.exit(1)

    # Keep running until Ctrl+C or SIGTERM.
    def shutdown(sig, frame):
        print("\n[INFO] Shutting down...")
        relay.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("[INFO] Running. Waiting for sensor data from Mega...")
    print("[INFO] Press Ctrl+C to stop.")

    while True:
        time.sleep(1)
        if not relay.is_running():
            print("[ERROR] Relay stopped unexpectedly.")
            sys.exit(1)


if __name__ == "__main__":
    main()
