#!/usr/bin/env python3
"""
sensor_bridge.py - Apex Rover Manual Sensor Bridge

This file runs in MANUAL mode and also stays running in AUTO mode.

Responsibilities:
  1. Read SENSOR lines from Arduino Mega over USB Serial.
  2. Normalize sensor data:
       UF -> FRONT
       UR -> REAR
       ALERT -> BALANCE
  3. Save latest sensor data to:
       /tmp/apex_last_sensor.json
     so Auto brain can read MPU safely without opening Serial again.
  4. Forward sensor data to ESP32 so the mobile app can display it.

Important:
  Only this file opens the Mega USB Serial.
  auto_stair_climb.py must NOT open Serial directly.
"""

import json
import os
import time
from pathlib import Path

import requests
import serial


# ============================================================
# PATH FIX
# Allows importing config.py from Raspberry/ even when this file
# is inside Raspberry/Manual/
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
PARENT_DIR = BASE_DIR.parent

import sys
sys.path.insert(0, str(PARENT_DIR))


# ============================================================
# CONFIG WITH SAFE DEFAULTS
# ============================================================

try:
    from config import (
        ESP32_IP,
        MEGA_PORT,
        BAUD_RATE,
        SERIAL_TIMEOUT,
        SERIAL_WRITE_TIMEOUT,
        PITCH_WARNING_DEG,
        ROLL_WARNING_DEG,
        PITCH_DANGER_DEG,
        ROLL_DANGER_DEG,
    )
except Exception:
    ESP32_IP = "192.168.4.1"

    MEGA_PORT = "/dev/ttyACM0"
    BAUD_RATE = 9600
    SERIAL_TIMEOUT = 0.2
    SERIAL_WRITE_TIMEOUT = 0.2

    PITCH_WARNING_DEG = 15.0
    ROLL_WARNING_DEG = 12.0
    PITCH_DANGER_DEG = 30.0
    ROLL_DANGER_DEG = 25.0


ESP32_SENSOR_UPDATE_URL = f"http://{ESP32_IP}/sensor_update"
ESP32_SENSOR_POST_URL = f"http://{ESP32_IP}/sensor"

LAST_SENSOR_FILE = "/tmp/apex_last_sensor.json"
LAST_SENSOR_TEXT_FILE = "/tmp/apex_last_sensor.txt"

FORWARD_TIMEOUT = 0.4


# ============================================================
# SENSOR PARSING
# ============================================================

def compute_balance(pitch, roll, alert):
    alert = str(alert).upper().strip()

    if alert == "TILT_DANGER":
        return "DANGER"

    if abs(pitch) >= PITCH_DANGER_DEG or abs(roll) >= ROLL_DANGER_DEG:
        return "DANGER"

    if abs(pitch) >= PITCH_WARNING_DEG or abs(roll) >= ROLL_WARNING_DEG:
        return "WARNING"

    return "STABLE"


def parse_sensor_line(line):
    """
    Input from Mega example:
      SENSOR:PITCH=2.10;ROLL=-1.20;UF=55.00;UR=18.00;ALERT=NONE

    Output dict:
      {
        "pitch": 2.10,
        "roll": -1.20,
        "front": 55.00,
        "rear": 18.00,
        "balance": "STABLE",
        "alert": "NONE",
        "timestamp": ...
      }
    """

    line = line.strip()

    if not line.startswith("SENSOR:"):
        return None

    payload = line[len("SENSOR:"):]
    parts = payload.split(";")

    values = {}

    for part in parts:
        if "=" not in part:
            continue

        key, value = part.split("=", 1)
        values[key.strip().upper()] = value.strip()

    try:
        pitch = float(values.get("PITCH", "0"))
        roll = float(values.get("ROLL", "0"))

        front = float(
            values.get(
                "FRONT",
                values.get("UF", values.get("FRONT_DISTANCE", "-1")),
            )
        )

        rear = float(
            values.get(
                "REAR",
                values.get("UR", values.get("REAR_DISTANCE", "-1")),
            )
        )

        alert = values.get("ALERT", "NONE").upper()
        balance = values.get("BALANCE", "").upper()

        if not balance:
            balance = compute_balance(pitch, roll, alert)

        sensor = {
            "pitch": pitch,
            "roll": roll,
            "front": front,
            "rear": rear,
            "balance": balance,
            "alert": alert,
            "timestamp": time.time(),
            "raw": line,
        }

        return sensor

    except Exception as e:
        print(f"[PARSE ERROR] {e} | line={line}")
        return None


def build_mobile_sensor_message(sensor):
    """
    Message format expected by mobile Flutter:
      SENSOR:PITCH=2.4;ROLL=-1.1;FRONT=35.6;REAR=18.2;BALANCE=STABLE
    """

    return (
        "SENSOR:"
        f"PITCH={sensor['pitch']:.2f};"
        f"ROLL={sensor['roll']:.2f};"
        f"FRONT={sensor['front']:.2f};"
        f"REAR={sensor['rear']:.2f};"
        f"BALANCE={sensor['balance']}"
    )


# ============================================================
# LOCAL SAVE FOR AUTO MODE
# ============================================================

def save_latest_sensor(sensor):
    try:
        temp_file = LAST_SENSOR_FILE + ".tmp"

        with open(temp_file, "w", encoding="utf-8") as f:
            json.dump(sensor, f)

        os.replace(temp_file, LAST_SENSOR_FILE)

        with open(LAST_SENSOR_TEXT_FILE, "w", encoding="utf-8") as f:
            f.write(build_mobile_sensor_message(sensor) + "\n")

    except Exception as e:
        print(f"[SAVE ERROR] {e}")


# ============================================================
# FORWARD TO ESP32
# ============================================================

def forward_to_esp32(sensor):
    """
    Primary:
      GET /sensor_update?pitch=...&roll=...&front=...&rear=...&balance=...

    Fallback:
      POST /sensor with data=<SENSOR:...>
    """

    mobile_message = build_mobile_sensor_message(sensor)

    # Primary GET endpoint
    try:
        r = requests.get(
            ESP32_SENSOR_UPDATE_URL,
            params={
                "pitch": f"{sensor['pitch']:.2f}",
                "roll": f"{sensor['roll']:.2f}",
                "front": f"{sensor['front']:.2f}",
                "rear": f"{sensor['rear']:.2f}",
                "balance": sensor["balance"],
            },
            timeout=FORWARD_TIMEOUT,
        )

        if r.status_code == 200:
            return True

        print(f"[ESP32 WARN] /sensor_update HTTP {r.status_code}")

    except Exception as e:
        print(f"[ESP32 WARN] /sensor_update failed: {e}")

    # Fallback POST endpoint
    try:
        r = requests.post(
            ESP32_SENSOR_POST_URL,
            data={"data": mobile_message},
            timeout=FORWARD_TIMEOUT,
        )

        if r.status_code == 200:
            return True

        print(f"[ESP32 WARN] /sensor HTTP {r.status_code}")

    except Exception as e:
        print(f"[ESP32 ERROR] /sensor failed: {e}")

    return False


# ============================================================
# SERIAL LOOP
# ============================================================

def open_mega_serial():
    print(f"[SERIAL] Opening Mega on {MEGA_PORT} @ {BAUD_RATE}")

    ser = serial.Serial(
        port=MEGA_PORT,
        baudrate=BAUD_RATE,
        timeout=SERIAL_TIMEOUT,
        write_timeout=SERIAL_WRITE_TIMEOUT,
    )

    time.sleep(2)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    print("[SERIAL] Mega connected")
    return ser


def main():
    print("==========================================")
    print("Apex Rover Sensor Bridge")
    print("Mode: Manual services / Auto support")
    print("Reads Mega Serial once and shares MPU data")
    print(f"Mega port: {MEGA_PORT}")
    print(f"ESP32: {ESP32_IP}")
    print(f"Latest sensor JSON: {LAST_SENSOR_FILE}")
    print("==========================================")

    while True:
        ser = None

        try:
            ser = open_mega_serial()

            while True:
                raw = ser.readline()

                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()

                if not line:
                    continue

                sensor = parse_sensor_line(line)

                if sensor is None:
                    print(f"[MEGA] {line}")
                    continue

                save_latest_sensor(sensor)
                forward_to_esp32(sensor)

                print(
                    "[SENSOR] "
                    f"pitch={sensor['pitch']:.2f} "
                    f"roll={sensor['roll']:.2f} "
                    f"front={sensor['front']:.2f} "
                    f"rear={sensor['rear']:.2f} "
                    f"balance={sensor['balance']}"
                )

        except KeyboardInterrupt:
            print("\n[INFO] Sensor bridge stopped by user")
            break

        except Exception as e:
            print(f"[ERROR] Sensor bridge error: {e}")
            print("[INFO] Reconnecting in 2 seconds...")

            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except Exception:
                pass

            time.sleep(2)


if __name__ == "__main__":
    main()