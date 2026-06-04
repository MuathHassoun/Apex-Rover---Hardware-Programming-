
#!/usr/bin/env python3
"""
auto_io.py
I/O helpers for automatic mode:
- ESP32 HTTP commands
- local sensor file reader
- Mega ACK file reader
- snapshot reader
- safety checks
- LEGO helpers:
    run_mega_block()
    wait_for_mega_ack()
    run_uno_pose()

IMPORTANT:
  Ultrasonic FRONT/REAR values are read and stored for display only.
  They do NOT affect AUTO decisions or safety stop.
"""

import json
import os
import time

import cv2
import numpy as np
import requests

from auto_config import (
    COMMAND_GAP_SEC,
    ESP32_COMMAND_TIMEOUT,
    ESP32_HTTP_COMMAND_URL,
    LAST_MEGA_ACK_FILE,
    LAST_SENSOR_FILE,
    LOCAL_ARM_SNAPSHOT_URL,
    LOCAL_FRONT_SNAPSHOT_URL,
    MEGA_BLOCK_POLL_SEC,
    MEGA_BLOCK_TIMEOUT_SEC,
    PITCH_DANGER_DEG,
    ROLL_DANGER_DEG,
    SNAPSHOT_SETTLE_SEC,
    UNO_ARM_DROP_IN_CMD,
    UNO_ARM_DROP_OUT_CMD,
    UNO_ARM_HOME_CMD,
    UNO_ARM_READY_CMD,
    UNO_ARM_TAKE_OUT_CMD,
    UNO_POSE_WAIT_SEC,
    UNO_SHORT_WAIT_SEC,
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
                self.status.event(
                    decision=f"command failed HTTP {r.status_code}",
                    command=cmd,
                    command_response=r.text[:160],
                )

            return ok

        except Exception as e:
            self.status.event(
                decision="command connection failed",
                command=cmd,
                command_error=str(e),
            )

            if stop_on_fail:
                raise

            return False

    def stop_all(self):
        """
        Safe stop for all known devices.
        Extra commands are okay because unknown commands are ignored by targets.
        """

        for cmd in [
            "BLOCK:STOP",
            "AUTO:STOP",
            "STOP",
            "JACK:ALL:STOP",
            "JACK:FRONT:STOP",
            "JACK:REAR:STOP",
            "ARM:BASE:STOP",
            "ARM:STOP",
            "CAM:STOP",
            "CAM:SERVO:STOP",
        ]:
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
                self.status.event(
                    decision="sensor data is stale",
                    sensor_age=age,
                )

            self.status.set_sensor(data)
            return data

        except Exception as e:
            self.status.event(
                decision="could not read sensor file",
                sensor_error=str(e),
            )
            return None

    def safety_ok(self):
        sensor = self.read_sensor(max_age_sec=4.0)

        if sensor is None:
            # Camera may still work, so do not immediately fail.
            return True

        pitch = abs(float(sensor.get("pitch", 0)))
        roll = abs(float(sensor.get("roll", 0)))
        balance = str(sensor.get("balance", "")).upper()

        # IMPORTANT:
        # FRONT / REAR ultrasonic values are DISPLAY ONLY.
        # They must NOT stop AUTO or affect decisions.
        if pitch >= PITCH_DANGER_DEG or roll >= ROLL_DANGER_DEG or balance == "DANGER":
            self.status.fail(
                "SAFETY",
                "Danger tilt detected",
                sensor=sensor,
            )
            self.stop_all()
            return False

        return True

    # ------------------------------------------------------------
    # Mega ACK reader
    # ------------------------------------------------------------
    def read_mega_ack(self):
        try:
            if not os.path.exists(LAST_MEGA_ACK_FILE):
                return None

            with open(LAST_MEGA_ACK_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)

            if not isinstance(data, dict):
                return None

            return data

        except Exception as e:
            self.status.event(
                decision="could not read Mega ACK file",
                mega_ack_error=str(e),
            )
            return None

    def ack_matches(self, ack, expected_name=None, require_done=True):
        if not isinstance(ack, dict):
            return False

        line = str(ack.get("line", "")).upper()
        name = str(ack.get("name", "")).upper()
        event = str(ack.get("event", "")).upper()

        if ack.get("ok") is False:
            return False

        if require_done:
            if not ack.get("done") and "DONE" not in line and event != "DONE":
                return False

        if expected_name:
            expected = str(expected_name).upper()
            return expected in line or expected == name

        return True

    def ack_is_error(self, ack):
        if not isinstance(ack, dict):
            return False

        line = str(ack.get("line", "")).upper()

        return (
            ack.get("ok") is False
            or line.startswith("ERR:")
            or line.startswith("ERROR:")
            or ":ERROR:" in line
        )

    def wait_for_mega_ack(
        self,
        expected_name=None,
        after_timestamp=None,
        timeout_sec=MEGA_BLOCK_TIMEOUT_SEC,
        require_done=True,
    ):
        start = time.time()
        deadline = start + float(timeout_sec)
        last_log_time = 0.0
        last_seen_line = ""

        if after_timestamp is None:
            after_timestamp = start

        while time.time() < deadline:
            if not self.safety_ok():
                return False

            ack = self.read_mega_ack()

            if ack is not None:
                ack_time = float(ack.get("timestamp", 0))
                line = str(ack.get("line", ""))

                if ack_time >= after_timestamp:
                    if self.ack_is_error(ack):
                        self.status.fail(
                            "MEGA_BLOCK_ERROR",
                            "Mega returned error while waiting for block ACK",
                            expected_name=expected_name,
                            mega_ack=ack,
                        )
                        self.stop_all()
                        return False

                    if self.ack_matches(
                        ack,
                        expected_name=expected_name,
                        require_done=require_done,
                    ):
                        self.status.event(
                            phase="MEGA_ACK",
                            doing="Mega block finished",
                            decision="ACK received",
                            expected_name=expected_name,
                            mega_ack=ack,
                        )
                        return True

                    last_seen_line = line

            now = time.time()

            if now - last_log_time >= 1.5:
                self.status.event(
                    phase="WAIT_MEGA_ACK",
                    doing="Waiting for Mega block ACK",
                    decision="keep waiting",
                    expected_name=expected_name,
                    last_seen_ack=last_seen_line,
                    timeout_left=round(deadline - now, 1),
                )
                last_log_time = now

            time.sleep(MEGA_BLOCK_POLL_SEC)

        self.status.fail(
            "MEGA_ACK_TIMEOUT",
            "Timeout waiting for Mega block ACK",
            expected_name=expected_name,
            last_seen_ack=last_seen_line,
            timeout_sec=timeout_sec,
        )
        self.stop_all()
        return False

    def expected_name_from_command(self, cmd):
        text = str(cmd).strip().upper()
        parts = text.split(":")

        if text.startswith("AUTO:") and len(parts) >= 2:
            return parts[1]

        if text.startswith("BLOCK:") and len(parts) >= 2:
            return parts[1]

        return None

    def run_mega_block(
        self,
        cmd,
        expected_done=None,
        timeout_sec=MEGA_BLOCK_TIMEOUT_SEC,
    ):
        cmd = str(cmd).strip().upper()

        if expected_done is None:
            expected_done = self.expected_name_from_command(cmd)

        issued_at = time.time()

        self.status.event(
            phase="MEGA_BLOCK",
            doing=f"Starting Mega block: {cmd}",
            decision="send command and wait for DONE ACK",
            command=cmd,
            expected_done=expected_done,
        )

        ok = self.command(cmd)

        if not ok:
            self.status.fail(
                "MEGA_BLOCK",
                "Failed to send Mega block command",
                command=cmd,
                expected_done=expected_done,
            )
            return False

        if not expected_done:
            return True

        return self.wait_for_mega_ack(
            expected_name=expected_done,
            after_timestamp=issued_at,
            timeout_sec=timeout_sec,
            require_done=True,
        )

    # ------------------------------------------------------------
    # LEGO helper commands for Mega
    # ------------------------------------------------------------
    def block_turn(self, direction, degree, timeout_sec=MEGA_BLOCK_TIMEOUT_SEC):
        direction = str(direction).upper().strip()
        degree = int(max(0, min(360, int(degree))))
        return self.run_mega_block(
            f"BLOCK:TURN:{direction}:{degree}",
            expected_done="TURN",
            timeout_sec=timeout_sec,
        )

    def block_go(self, direction, amount, timeout_sec=MEGA_BLOCK_TIMEOUT_SEC):
        direction = str(direction).upper().strip()
        amount = int(max(1, int(amount)))
        return self.run_mega_block(
            f"BLOCK:GO:{direction}:{amount}",
            expected_done="GO",
            timeout_sec=timeout_sec,
        )

    def block_jack(self, which, action, amount, timeout_sec=MEGA_BLOCK_TIMEOUT_SEC):
        which = str(which).upper().strip()
        action = str(action).upper().strip()
        amount = int(max(1, int(amount)))
        return self.run_mega_block(
            f"BLOCK:JACK:{which}:{action}:{amount}",
            expected_done="JACK",
            timeout_sec=timeout_sec,
        )

    def auto_up_stairs(self, timeout_sec=MEGA_BLOCK_TIMEOUT_SEC):
        return self.run_mega_block(
            "AUTO:UP_STAIRS",
            expected_done="UP_STAIRS",
            timeout_sec=timeout_sec,
        )

    def auto_down_stairs(self, timeout_sec=MEGA_BLOCK_TIMEOUT_SEC):
        return self.run_mega_block(
            "AUTO:DOWN_STAIRS",
            expected_done="DOWN_STAIRS",
            timeout_sec=timeout_sec,
        )

    # ------------------------------------------------------------
    # UNO pose commands
    # ------------------------------------------------------------
    def run_uno_pose(self, cmd, wait_sec=UNO_POSE_WAIT_SEC):
        cmd = str(cmd).strip().upper()

        self.status.event(
            phase="UNO_ARM_POSE",
            doing=f"Moving UNO arm pose: {cmd}",
            decision=f"wait {wait_sec}s because UNO ACK is not used",
            command=cmd,
        )

        ok = self.command(cmd)

        if not ok:
            self.status.fail(
                "UNO_ARM_POSE",
                "Failed to send UNO pose command",
                command=cmd,
            )
            return False

        return self.wait(wait_sec)

    def arm_home(self):
        return self.run_uno_pose(UNO_ARM_HOME_CMD, wait_sec=UNO_POSE_WAIT_SEC)

    def arm_ready(self):
        return self.run_uno_pose(UNO_ARM_READY_CMD, wait_sec=UNO_POSE_WAIT_SEC)

    def arm_take_out(self):
        return self.run_uno_pose(UNO_ARM_TAKE_OUT_CMD, wait_sec=UNO_POSE_WAIT_SEC)

    def arm_drop_in(self):
        return self.run_uno_pose(UNO_ARM_DROP_IN_CMD, wait_sec=UNO_POSE_WAIT_SEC)

    def arm_drop_out(self):
        return self.run_uno_pose(UNO_ARM_DROP_OUT_CMD, wait_sec=UNO_POSE_WAIT_SEC)

    def arm_stop(self):
        self.command("ARM:STOP")
        return self.wait(UNO_SHORT_WAIT_SEC)

    # ------------------------------------------------------------
    # Camera snapshots
    # ------------------------------------------------------------
    def snapshot(self, camera="front"):
        url = LOCAL_FRONT_SNAPSHOT_URL if camera == "front" else LOCAL_ARM_SNAPSHOT_URL

        try:
            time.sleep(SNAPSHOT_SETTLE_SEC)

            r = requests.get(url, timeout=2.2)

            if r.status_code != 200:
                self.status.event(
                    decision=f"{camera} snapshot HTTP {r.status_code}",
                    camera=camera,
                )
                return None

            arr = np.frombuffer(r.content, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.status.event(
                    decision=f"{camera} snapshot decode failed",
                    camera=camera,
                )

            return frame

        except Exception as e:
            self.status.event(
                decision=f"{camera} snapshot failed",
                camera=camera,
                snapshot_error=str(e),
            )
            return None

    def wait(self, seconds):
        end = time.time() + float(seconds)

        while time.time() < end:
            if not self.safety_ok():
                return False

            time.sleep(0.1)

        return True