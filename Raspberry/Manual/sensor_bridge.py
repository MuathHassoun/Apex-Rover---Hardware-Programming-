
#!/usr/bin/env python3
"""
sensor_bridge.py - Apex Rover Sensor Bridge

Runs in MANUAL mode and also stays running in AUTO mode.

Responsibilities:
  1. Read SENSOR lines from Arduino Mega over USB Serial.
  2. Normalize sensor data:
       UF -> FRONT
       UR -> REAR
       ALERT -> BALANCE
  3. Save latest sensor data to:
       /tmp/apex_last_sensor.json
  4. Forward sensor data to ESP32 so the mobile app can display it.
  5. NEW:
       Read ACK / ERROR lines from Mega blocks and save them to:
       /tmp/apex_last_mega_ack.json

Important:
  Only this file opens the Mega USB Serial.
  auto_stair_climb.py must NOT open Serial directly.
"""

import json
import os
import sys
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

# NEW endpoint. If ESP32 has it, ACK will be forwarded.
# If ESP32 does not have it yet, this bridge will still work locally.
ESP32_BRIDGE_EVENT_URL = f"http://{ESP32_IP}/bridge_event"

LAST_SENSOR_FILE = "/tmp/apex_last_sensor.json"
LAST_SENSOR_TEXT_FILE = "/tmp/apex_last_sensor.txt"

# NEW: Mega ACK files used by Raspberry auto orchestrator.
LAST_MEGA_ACK_FILE = "/tmp/apex_last_mega_ack.json"
LAST_MEGA_ACK_TEXT_FILE = "/tmp/apex_last_mega_ack.txt"
LAST_MEGA_LINE_FILE = "/tmp/apex_last_mega_line.json"

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
        print(f"[PARSE ERROR] {e} | line={line}", flush=True)
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

def atomic_write_json(path, data):
    temp_file = path + ".tmp"

    with open(temp_file, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)

    os.replace(temp_file, path)


def save_latest_sensor(sensor):
    try:
        atomic_write_json(LAST_SENSOR_FILE, sensor)

        with open(LAST_SENSOR_TEXT_FILE, "w", encoding="utf-8") as f:
            f.write(build_mobile_sensor_message(sensor) + "\n")

    except Exception as e:
        print(f"[SAVE SENSOR ERROR] {e}", flush=True)


# ============================================================
# NEW: MEGA ACK / ERROR PARSING
# ============================================================

def parse_mega_ack_line(line):
    """
    Supported examples:
      ACK:MEGA:START:UP_STAIRS
      ACK:MEGA:STEP:UP_STAIRS:ALIGN
      ACK:MEGA:DONE:UP_STAIRS
      ACK:MEGA:DONE:TURN
      ERR:MEGA:UP_STAIRS:TILT_DANGER
      ERROR:MEGA:...
    """

    text = line.strip()
    upper = text.upper()

    is_ack = upper.startswith("ACK:")
    is_error = upper.startswith("ERR:") or upper.startswith("ERROR:")

    if not is_ack and not is_error:
        return None

    parts = text.split(":")
    parts_upper = [p.upper() for p in parts]

    source = parts[1] if len(parts) > 1 else "MEGA"
    event = parts[2] if len(parts) > 2 else ("ERROR" if is_error else "ACK")
    name = parts[3] if len(parts) > 3 else ""

    done = "DONE" in parts_upper
    started = "START" in parts_upper
    step = "STEP" in parts_upper

    ack = {
        "ok": not is_error,
        "type": "error" if is_error else "ack",
        "source": source,
        "event": event,
        "name": name,
        "done": done,
        "started": started,
        "step": step,
        "line": text,
        "parts": parts,
        "timestamp": time.time(),
    }

    return ack


def save_latest_mega_line(line):
    try:
        data = {
            "line": line.strip(),
            "timestamp": time.time(),
        }
        atomic_write_json(LAST_MEGA_LINE_FILE, data)
    except Exception as e:
        print(f"[SAVE MEGA LINE ERROR] {e}", flush=True)


def save_latest_mega_ack(ack):
    try:
        atomic_write_json(LAST_MEGA_ACK_FILE, ack)

        with open(LAST_MEGA_ACK_TEXT_FILE, "w", encoding="utf-8") as f:
            f.write(ack.get("line", "") + "\n")

    except Exception as e:
        print(f"[SAVE MEGA ACK ERROR] {e}", flush=True)


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

        print(f"[ESP32 WARN] /sensor_update HTTP {r.status_code}", flush=True)

    except Exception as e:
        print(f"[ESP32 WARN] /sensor_update failed: {e}", flush=True)

    # Fallback POST endpoint
    try:
        r = requests.post(
            ESP32_SENSOR_POST_URL,
            data={"data": mobile_message},
            timeout=FORWARD_TIMEOUT,
        )

        if r.status_code == 200:
            return True

        print(f"[ESP32 WARN] /sensor HTTP {r.status_code}", flush=True)

    except Exception as e:
        print(f"[ESP32 ERROR] /sensor failed: {e}", flush=True)

    return False


def forward_ack_to_esp32(ack):
    """
    Forwards Mega ACK to ESP32 if /bridge_event exists.
    If it fails, we do not stop anything because the local ACK file is enough
    for Raspberry auto logic.
    """

    try:
        r = requests.get(
            ESP32_BRIDGE_EVENT_URL,
            params={
                "type": ack.get("type", "ack"),
                "source": ack.get("source", "MEGA"),
                "event": ack.get("event", ""),
                "name": ack.get("name", ""),
                "done": "1" if ack.get("done") else "0",
                "ok": "1" if ack.get("ok") else "0",
                "line": ack.get("line", ""),
            },
            timeout=FORWARD_TIMEOUT,
        )

        if r.status_code == 200:
            return True

        print(f"[ESP32 WARN] /bridge_event HTTP {r.status_code}", flush=True)

    except Exception as e:
        print(f"[ESP32 WARN] /bridge_event failed: {e}", flush=True)

    return False


# ============================================================
# NON-SENSOR LINE HANDLER
# ============================================================

def handle_non_sensor_line(line):
    text = line.strip()

    if not text:
        return

    save_latest_mega_line(text)

    ack = parse_mega_ack_line(text)

    if ack is not None:
        save_latest_mega_ack(ack)
        forward_ack_to_esp32(ack)

        if ack.get("ok"):
            print(f"[MEGA ACK] {text}", flush=True)
        else:
            print(f"[MEGA ERROR] {text}", flush=True)

        return

    print(f"[MEGA] {text}", flush=True)


# ============================================================
# SERIAL LOOP
# ============================================================

def open_mega_serial():
    print(f"[SERIAL] Opening Mega on {MEGA_PORT} @ {BAUD_RATE}", flush=True)

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

    print("[SERIAL] Mega connected", flush=True)
    return ser


def main():
    print("==========================================", flush=True)
    print("Apex Rover Sensor Bridge", flush=True)
    print("Mode: Manual services / Auto support", flush=True)
    print("Reads Mega Serial once and shares MPU + ACK data", flush=True)
    print(f"Mega port: {MEGA_PORT}", flush=True)
    print(f"ESP32: {ESP32_IP}", flush=True)
    print(f"Latest sensor JSON: {LAST_SENSOR_FILE}", flush=True)
    print(f"Latest Mega ACK JSON: {LAST_MEGA_ACK_FILE}", flush=True)
    print("==========================================", flush=True)

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
                    handle_non_sensor_line(line)
                    continue

                save_latest_sensor(sensor)
                forward_to_esp32(sensor)

                print(
                    "[SENSOR] "
                    f"pitch={sensor['pitch']:.2f} "
                    f"roll={sensor['roll']:.2f} "
                    f"front={sensor['front']:.2f} "
                    f"rear={sensor['rear']:.2f} "
                    f"balance={sensor['balance']}",
                    flush=True,
                )

        except KeyboardInterrupt:
            print("\n[INFO] Sensor bridge stopped by user", flush=True)
            break

        except Exception as e:
            print(f"[ERROR] Sensor bridge error: {e}", flush=True)
            print("[INFO] Reconnecting in 2 seconds...", flush=True)

            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except Exception:
                pass

            time.sleep(2)


if __name__ == "__main__":
    main()