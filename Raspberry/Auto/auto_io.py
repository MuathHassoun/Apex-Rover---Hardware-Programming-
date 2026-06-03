#!/usr/bin/env python3
"""
auto_io.py
I/O helpers for automatic mode:
- ESP32 HTTP commands
- local sensor file reader
- snapshot reader
- safety checks
"""

import json
import os
import time
from urllib.parse import quote

import cv2
import numpy as np
import requests

from auto_config import (
    COMMAND_GAP_SEC,
    ESP32_COMMAND_TIMEOUT,
    ESP32_HTTP_COMMAND_URL,
    FRONT_ULTRASONIC_HARD_STOP_CM,
    LAST_SENSOR_FILE,
    LOCAL_ARM_SNAPSHOT_URL,
    LOCAL_FRONT_SNAPSHOT_URL,
    PITCH_DANGER_DEG,
    ROLL_DANGER_DEG,
    SNAPSHOT_SETTLE_SEC,
)


class RoverIO:
    def __init__(self, status):
        self.status = status
        self.last_command_time = 0.0

    # ------------------------------------------------------------
    # ESP32 commands
    # ------------------------------------------------------------
    def command(self, cmd, stop_on_fail=False):
        now = time.time()
        dt = now - self.last_command_time
        if dt < COMMAND_GAP_SEC:
            time.sleep(COMMAND_GAP_SEC - dt)

        self.last_command_time = time.time()
        self.status.set_command(cmd)

        try:
            r = requests.get(
                ESP32_HTTP_COMMAND_URL,
                params={"cmd": cmd},
                timeout=ESP32_COMMAND_TIMEOUT,
            )
            ok = r.status_code == 200
            if not ok:
                self.status.event(decision=f"command failed HTTP {r.status_code}", command=cmd, command_response=r.text[:160])
            return ok
        except Exception as e:
            self.status.event(decision="command connection failed", command=cmd, command_error=str(e))
            if stop_on_fail:
                raise
            return False

    def stop_all(self):
        for cmd in ["STOP", "JACK:ALL:STOP", "ARM:STOP", "CAM:STOP"]:
            self.command(cmd)

    def pulse(self, direction, ms):
        direction = str(direction).upper().strip()
        ms = int(max(50, min(1500, ms)))
        return self.command(f"PULSE:{direction}:{ms}")

    # ------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------
    def read_sensor(self, max_age_sec=3.0):
        try:
            if not os.path.exists(LAST_SENSOR_FILE):
                return None
            with open(LAST_SENSOR_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            age = time.time() - float(data.get("timestamp", 0))
            data["age_sec"] = age
            if age > max_age_sec:
                self.status.event(decision="sensor data is stale", sensor_age=age)
            self.status.set_sensor(data)
            return data
        except Exception as e:
            self.status.event(decision="could not read sensor file", sensor_error=str(e))
            return None

    def safety_ok(self):
        sensor = self.read_sensor(max_age_sec=4.0)
        if sensor is None:
            # Do not immediately fail because camera may still work, but be cautious.
            return True

        pitch = abs(float(sensor.get("pitch", 0)))
        roll = abs(float(sensor.get("roll", 0)))
        front = float(sensor.get("front", -1))
        balance = str(sensor.get("balance", "")).upper()

        if pitch >= PITCH_DANGER_DEG or roll >= ROLL_DANGER_DEG or balance == "DANGER":
            self.status.fail("SAFETY", "Danger tilt detected", sensor=sensor)
            self.stop_all()
            return False

        if front > 0 and front <= FRONT_ULTRASONIC_HARD_STOP_CM:
            self.status.fail("SAFETY", "Hard front obstacle stop", sensor=sensor)
            self.stop_all()
            return False

        return True

    # ------------------------------------------------------------
    # Camera snapshots
    # ------------------------------------------------------------
    def snapshot(self, camera="front"):
        url = LOCAL_FRONT_SNAPSHOT_URL if camera == "front" else LOCAL_ARM_SNAPSHOT_URL
        try:
            time.sleep(SNAPSHOT_SETTLE_SEC)
            r = requests.get(url, timeout=2.2)
            if r.status_code != 200:
                self.status.event(decision=f"{camera} snapshot HTTP {r.status_code}", camera=camera)
                return None
            arr = np.frombuffer(r.content, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.status.event(decision=f"{camera} snapshot decode failed", camera=camera)
            return frame
        except Exception as e:
            self.status.event(decision=f"{camera} snapshot failed", camera=camera, snapshot_error=str(e))
            return None

    def wait(self, seconds):
        end = time.time() + float(seconds)
        while time.time() < end:
            if not self.safety_ok():
                return False
            time.sleep(0.1)
        return True
