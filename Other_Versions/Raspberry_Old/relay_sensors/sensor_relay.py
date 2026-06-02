import time
import threading
from typing import Optional

import serial
import requests

from config import (
    MEGA_PORT,
    BAUD_RATE,
    SERIAL_TIMEOUT,
    SERIAL_WRITE_TIMEOUT,
    ESP32_SENSOR_UPDATE_URL,
    SENSOR_FORWARD_TIMEOUT,
    PITCH_WARNING_DEG,
    ROLL_WARNING_DEG,
    PITCH_DANGER_DEG,
    ROLL_DANGER_DEG,
)


class SensorRelay:
    """
    Reads SENSOR lines from Mega USB Serial and forwards them to ESP32.

    Mega sends:
      SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE

    ESP32 receives:
      /sensor_update?pitch=...&roll=...&front=...&rear=...&balance=...
    """

    def __init__(self):
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False

    def start(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=MEGA_PORT,
                baudrate=BAUD_RATE,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_WRITE_TIMEOUT,
            )

            # Arduino Mega may reset when serial opens
            time.sleep(2)

            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass

            self._stop_event.clear()

            self._thread = threading.Thread(
                target=self._relay_loop,
                name="SensorRelayThread",
                daemon=True,
            )
            self._thread.start()

            self._running = True

            print(f"[OK] Reading Mega sensors from: {MEGA_PORT}")
            print(f"[OK] Forwarding sensors to: {ESP32_SENSOR_UPDATE_URL}")

            return True

        except Exception as e:
            print(f"[ERROR] Cannot open Mega serial port {MEGA_PORT}: {e}")
            self._running = False
            return False

    def stop(self):
        self._stop_event.set()

        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass

        self._running = False
        print("[OK] SensorRelay stopped")

    def is_running(self) -> bool:
        return self._running

    def _relay_loop(self):
        while not self._stop_event.is_set():
            try:
                if not self._serial or not self._serial.is_open:
                    print("[ERROR] Mega serial is not open")
                    self._running = False
                    return

                raw = self._serial.readline()

                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()

                if not line:
                    continue

                self._handle_line(line)

            except Exception as e:
                print(f"[WARN] SensorRelay read error: {e}")
                time.sleep(0.1)

    def _handle_line(self, line: str):
        if not line.startswith("SENSOR:"):
            print(f"[Mega->Pi] {line}")
            return

        print(f"[Mega SENSOR] {line}")

        sensor_data = self._parse_sensor_line(line)

        if sensor_data is None:
            print("[WARN] Could not parse SENSOR line")
            return

        self._forward_to_esp32(sensor_data)

    def _parse_sensor_line(self, line: str):
        try:
            payload = line[len("SENSOR:"):]
            parts = payload.split(";")

            data = {}

            for part in parts:
                if "=" not in part:
                    continue

                key, value = part.split("=", 1)
                key = key.strip().upper()
                value = value.strip()

                data[key] = value

            pitch = float(data.get("PITCH", 0.0))
            roll = float(data.get("ROLL", 0.0))
            front = float(data.get("UF", data.get("FRONT", -1.0)))
            rear = float(data.get("UR", data.get("REAR", -1.0)))
            alert = data.get("ALERT", "NONE").upper()

            balance = self._compute_balance_status(
                pitch=pitch,
                roll=roll,
                alert=alert,
            )

            return {
                "pitch": pitch,
                "roll": roll,
                "front": front,
                "rear": rear,
                "balance": balance,
                "alert": alert,
            }

        except Exception as e:
            print(f"[ERROR] Parse failed: {e}")
            return None

    def _compute_balance_status(self, pitch: float, roll: float, alert: str) -> str:
        abs_pitch = abs(pitch)
        abs_roll = abs(roll)

        # Mega critical alerts
        if alert == "TILT_DANGER":
            return "DANGER"

        # Obstacle alerts are important, but not falling danger
        if alert in ("OBSTACLE_FRONT", "OBSTACLE_REAR"):
            return "WARNING"

        # Raspberry-side balance check
        if abs_pitch >= PITCH_DANGER_DEG or abs_roll >= ROLL_DANGER_DEG:
            return "DANGER"

        if abs_pitch >= PITCH_WARNING_DEG or abs_roll >= ROLL_WARNING_DEG:
            return "WARNING"

        return "STABLE"

    def _forward_to_esp32(self, sensor_data: dict):
        try:
            response = requests.get(
                ESP32_SENSOR_UPDATE_URL,
                params={
                    "pitch": f"{sensor_data['pitch']:.2f}",
                    "roll": f"{sensor_data['roll']:.2f}",
                    "front": f"{sensor_data['front']:.2f}",
                    "rear": f"{sensor_data['rear']:.2f}",
                    "balance": sensor_data["balance"],
                },
                timeout=SENSOR_FORWARD_TIMEOUT,
            )

            if response.status_code == 200:
                print(
                    "[OK] Forwarded to ESP32:",
                    f"PITCH={sensor_data['pitch']:.2f}",
                    f"ROLL={sensor_data['roll']:.2f}",
                    f"FRONT={sensor_data['front']:.2f}",
                    f"REAR={sensor_data['rear']:.2f}",
                    f"BALANCE={sensor_data['balance']}",
                )
            else:
                print(f"[WARN] ESP32 returned HTTP {response.status_code}: {response.text}")

        except requests.exceptions.Timeout:
            print("[WARN] ESP32 HTTP timeout while forwarding sensor data")

        except Exception as e:
            print(f"[ERROR] Failed to forward to ESP32: {e}")