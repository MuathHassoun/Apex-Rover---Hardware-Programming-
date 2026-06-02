
#!/usr/bin/env python3
"""
auto_stair_climb.py - Apex Rover Automatic Stair Climb

Uses only:
  - Front camera snapshot
  - MPU pitch/roll from /tmp/apex_last_sensor.json

Does NOT open Mega Serial.
Does NOT use arm camera.
Does NOT use ultrasonic for decisions.

Command path:
  Raspberry Auto Brain -> ESP32 WebSocket -> Mega / UNO

Step logic:
  1. Follow yellow line forward.
  2. Cross the first wooden/ramp part and do not confuse it as the stair.
  3. Detect when the front part is on / contacting the stair.
  4. Extend rear jack.
  5. Wait until MPU shows the rear jack affected the robot.
  6. Move forward until the robot climbs the step.
  7. Retract rear jack enough so it does not hit the lower part of the stair.
  8. Recalibrate and repeat.
"""

import json
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import requests
import websocket


# ============================================================
# PATH FIX
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
PARENT_DIR = BASE_DIR.parent
sys.path.insert(0, str(PARENT_DIR))

try:
    from config import ESP32_WS_URL, FRONT_SNAPSHOT_URL
except Exception:
    ESP32_WS_URL = "ws://192.168.4.1:81"
    FRONT_SNAPSHOT_URL = "http://127.0.0.1:5000/front_snapshot"


# ============================================================
# FILES
# ============================================================

LAST_SENSOR_FILE = "/tmp/apex_last_sensor.json"
AUTO_STOP_FILE = "/tmp/apex_auto_stop"
DEBUG_IMAGE_FILE = "/tmp/apex_auto_front_debug.jpg"


# ============================================================
# AUTO SETTINGS
# ============================================================

MAX_STEPS_TO_CLIMB = 3

AUTO_SPEED_PERCENT = 55

# Forward is longer now because your robot needs more push with tracks.
APPROACH_FORWARD_MS = 420
CLIMB_FORWARD_MS = 560

TURN_PULSE_MS = 180
SMALL_FORWARD_MS = 300

LOOP_DELAY_SEC = 0.12

# Yellow path
YELLOW_MIN_AREA_RATIO = 0.0022
YELLOW_CENTER_DEADZONE = 0.17
YELLOW_LOST_LIMIT = 14

# Wood / first lifting board
WOOD_PITCH_DELTA_DEG = 2.0
WOOD_CLEAR_PITCH_DEG = 2.8
WOOD_MAX_CROSS_SEC = 9.0

# Front reaches stair
FRONT_ON_STEP_PITCH_DELTA = 4.2
STAIR_EDGE_TRIGGER = 0.60
STAIR_EDGE_REQUIRED_COUNT = 3

# Rear jack is slow, so timeout is longer.
REAR_JACK_MAX_EXTEND_SEC = 9.0
REAR_JACK_EFFECT_PITCH_DELTA = 1.5
REAR_JACK_EFFECT_ROLL_DELTA = 1.3
REAR_JACK_EXTRA_AFTER_EFFECT_SEC = 2.2

# Retract rear jack enough to clear lower stair part.
REAR_JACK_RETRACT_CLEAR_SEC = 3.0

# Safety
MAX_ABS_PITCH_DEG = 40.0
MAX_ABS_ROLL_DEG = 24.0
SENSOR_TIMEOUT_SEC = 1.2

# Success after climbing
CLIMB_STABLE_PITCH_DELTA = 3.2
CLIMB_MAX_SEC = 35.0

# Approach timeout
APPROACH_MAX_SEC = 45.0


# ============================================================
# DATA CLASSES
# ============================================================

@dataclass
class SensorState:
    pitch: float
    roll: float
    balance: str
    timestamp: float


@dataclass
class VisionState:
    ok: bool
    yellow_found: bool
    yellow_error: float
    yellow_confidence: float
    stair_edge_confidence: float


# ============================================================
# ESP32 COMMAND SENDER
# ============================================================

class ESP32Commander:
    def __init__(self, url: str):
        self.url = url
        self.ws = None

    def connect(self) -> bool:
        try:
            print(f"[WS] Connecting to {self.url}")
            self.ws = websocket.create_connection(self.url, timeout=4)
            print("[WS] Connected")
            return True
        except Exception as e:
            print(f"[WS ERROR] {e}")
            self.ws = None
            return False

    def send(self, command: str):
        command = command.strip()

        if not command:
            return

        try:
            if self.ws is None:
                if not self.connect():
                    print(f"[CMD FAIL] {command}")
                    return

            self.ws.send(command)
            print(f"[CMD] {command}")

        except Exception as e:
            print(f"[CMD ERROR] {command}: {e}")
            self.ws = None

    def stop_all(self):
        for cmd in [
            "STOP",
            "JACK:REAR:STOP",
            "JACK:FRONT:STOP",
            "CAM:STOP",
        ]:
            self.send(cmd)
            time.sleep(0.05)

    def close(self):
        try:
            if self.ws is not None:
                self.ws.close()
        except Exception:
            pass

        self.ws = None


# ============================================================
# SENSOR READER
# ============================================================

class SensorReader:
    def read(self) -> Optional[SensorState]:
        try:
            with open(LAST_SENSOR_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)

            return SensorState(
                pitch=float(data.get("pitch", 0.0)),
                roll=float(data.get("roll", 0.0)),
                balance=str(data.get("balance", "NO DATA")),
                timestamp=float(data.get("timestamp", 0.0)),
            )

        except Exception:
            return None

    def wait_for_sensor(self, timeout_sec: float = 8.0) -> bool:
        end = time.time() + timeout_sec

        while time.time() < end:
            sensor = self.read()

            if sensor is not None and time.time() - sensor.timestamp <= SENSOR_TIMEOUT_SEC:
                return True

            time.sleep(0.1)

        return False


# ============================================================
# FRONT CAMERA VISION
# ============================================================

class FrontVision:
    def __init__(self):
        self.session = requests.Session()

    def get_frame(self):
        try:
            r = self.session.get(FRONT_SNAPSHOT_URL, timeout=0.7)

            if r.status_code != 200:
                print(f"[VISION] Snapshot HTTP {r.status_code}")
                return None

            arr = np.frombuffer(r.content, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

            return frame

        except Exception as e:
            print(f"[VISION] Snapshot error: {e}")
            return None

    def analyze(self) -> VisionState:
        frame = self.get_frame()

        if frame is None:
            return VisionState(False, False, 0.0, 0.0, 0.0)

        h, w = frame.shape[:2]

        # Lower area for yellow path.
        y_start = int(h * 0.38)
        roi = frame[y_start:h, 0:w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow range. Adjust later if lighting changes.
        lower_yellow = np.array([18, 70, 70], dtype=np.uint8)
        upper_yellow = np.array([45, 255, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.medianBlur(mask, 5)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        yellow_found = False
        yellow_error = 0.0
        yellow_confidence = 0.0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            roi_area = roi.shape[0] * roi.shape[1]
            area_ratio = area / max(1, roi_area)

            if area_ratio >= YELLOW_MIN_AREA_RATIO:
                moment = cv2.moments(largest)

                if moment["m00"] != 0:
                    cx = int(moment["m10"] / moment["m00"])
                    yellow_error = (cx - (w / 2.0)) / (w / 2.0)
                    yellow_error = max(-1.0, min(1.0, yellow_error))
                    yellow_found = True
                    yellow_confidence = min(1.0, area_ratio * 45.0)

                    cv2.circle(
                        roi,
                        (cx, int(roi.shape[0] * 0.65)),
                        8,
                        (0, 0, 255),
                        -1,
                    )
                    cv2.line(
                        roi,
                        (w // 2, 0),
                        (w // 2, roi.shape[0]),
                        (255, 0, 0),
                        2,
                    )

        stair_edge_confidence = self.detect_stair_edges(frame)

        try:
            debug = frame.copy()
            cv2.putText(
                debug,
                f"yellow={yellow_found} err={yellow_error:.2f} "
                f"yc={yellow_confidence:.2f} edge={stair_edge_confidence:.2f}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
            cv2.imwrite(DEBUG_IMAGE_FILE, debug)
        except Exception:
            pass

        return VisionState(
            ok=True,
            yellow_found=yellow_found,
            yellow_error=yellow_error,
            yellow_confidence=yellow_confidence,
            stair_edge_confidence=stair_edge_confidence,
        )

    def detect_stair_edges(self, frame) -> float:
        h, w = frame.shape[:2]

        y1 = int(h * 0.24)
        y2 = int(h * 0.80)

        roi = frame[y1:y2, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(gray, 70, 160)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=55,
            minLineLength=int(w * 0.26),
            maxLineGap=20,
        )

        if lines is None:
            return 0.0

        horizontal_count = 0

        for line in lines[:25]:
            x1, y1_line, x2, y2_line = line[0]

            dx = x2 - x1
            dy = y2_line - y1_line

            if abs(dx) < 1:
                continue

            slope = abs(dy / dx)

            if slope < 0.18:
                horizontal_count += 1

        return min(1.0, horizontal_count / 4.0)


# ============================================================
# AUTO STAIR BRAIN
# ============================================================

class AutoStairBrain:
    def __init__(self):
        self.commander = ESP32Commander(ESP32_WS_URL)
        self.sensor_reader = SensorReader()
        self.vision = FrontVision()

        self.pitch_base = 0.0
        self.roll_base = 0.0

    # ----------------------------
    # Basic helpers
    # ----------------------------
    def sensor(self) -> Optional[SensorState]:
        return self.sensor_reader.read()

    def pitch_delta(self) -> float:
        s = self.sensor()

        if s is None:
            return 0.0

        return s.pitch - self.pitch_base

    def roll_delta(self) -> float:
        s = self.sensor()

        if s is None:
            return 0.0

        return s.roll - self.roll_base

    def stop_requested(self) -> bool:
        return os.path.exists(AUTO_STOP_FILE)

    def safety_ok(self) -> bool:
        if self.stop_requested():
            print("[SAFETY] Stop file detected")
            return False

        s = self.sensor()

        if s is None:
            print("[SAFETY] No sensor data")
            return False

        if time.time() - s.timestamp > SENSOR_TIMEOUT_SEC:
            print("[SAFETY] Sensor timeout")
            return False

        if abs(s.pitch) > MAX_ABS_PITCH_DEG:
            print(f"[SAFETY] Pitch danger: {s.pitch:.2f}")
            return False

        if abs(s.roll) > MAX_ABS_ROLL_DEG:
            print(f"[SAFETY] Roll danger: {s.roll:.2f}")
            return False

        return True

    def send(self, cmd: str):
        self.commander.send(cmd)

    def stop_all(self):
        self.commander.stop_all()

    def pulse(self, direction: str, duration_ms: int):
        if not self.safety_ok():
            self.stop_all()
            raise RuntimeError("Safety stop before pulse")

        duration_ms = int(max(80, min(1500, duration_ms)))
        self.send(f"PULSE:{direction}:{duration_ms}")
        time.sleep((duration_ms / 1000.0) + 0.12)

    # ----------------------------
    # Setup
    # ----------------------------
    def calibrate_mpu(self, label: str = "baseline") -> bool:
        print(f"[AUTO] Calibrating MPU: {label}")

        samples = []
        end = time.time() + 2.5

        while time.time() < end:
            s = self.sensor()

            if s is not None and time.time() - s.timestamp <= SENSOR_TIMEOUT_SEC:
                samples.append((s.pitch, s.roll))

            time.sleep(0.08)

        if len(samples) < 8:
            print("[AUTO] Not enough MPU samples")
            return False

        self.pitch_base = sum(p for p, _ in samples) / len(samples)
        self.roll_base = sum(r for _, r in samples) / len(samples)

        print(
            f"[AUTO] New baseline: "
            f"pitch={self.pitch_base:.2f}, roll={self.roll_base:.2f}"
        )

        return True

    def prepare_robot(self):
        print("[AUTO] Preparing robot")

        self.send("SYS:MODE:AUTO")
        time.sleep(0.1)

        self.send(f"SPEED:{AUTO_SPEED_PERCENT}")
        time.sleep(0.1)

        self.send("STOP")
        self.send("JACK:REAR:STOP")
        self.send("JACK:FRONT:STOP")
        time.sleep(0.2)

    # ----------------------------
    # Vision movement
    # ----------------------------
    def follow_yellow_once(self, forward_ms: int) -> bool:
        v = self.vision.analyze()

        if not v.ok:
            print("[VISION] Not OK")
            self.send("STOP")
            return False

        if not v.yellow_found:
            print("[LINE] Yellow not found")
            self.send("STOP")
            return False

        if v.yellow_error < -YELLOW_CENTER_DEADZONE:
            print(f"[LINE] Yellow left err={v.yellow_error:.2f} -> LEFT")
            self.pulse("LEFT", TURN_PULSE_MS)
            return True

        if v.yellow_error > YELLOW_CENTER_DEADZONE:
            print(f"[LINE] Yellow right err={v.yellow_error:.2f} -> RIGHT")
            self.pulse("RIGHT", TURN_PULSE_MS)
            return True

        print(f"[LINE] Yellow centered err={v.yellow_error:.2f} -> FORWARD")
        self.pulse("FORWARD", forward_ms)
        return True

    # ----------------------------
    # Step sequence
    # ----------------------------
    def approach_and_cross_wood(self) -> bool:
        print("[STATE] APPROACH_AND_CROSS_WOOD")

        start = time.time()
        yellow_lost = 0
        wood_seen = False
        wood_start_time = None

        while time.time() - start < APPROACH_MAX_SEC:
            if not self.safety_ok():
                return False

            s = self.sensor()
            v = self.vision.analyze()
            pd = self.pitch_delta()

            print(
                f"[OBS] wood_seen={wood_seen} "
                f"yellow={v.yellow_found} err={v.yellow_error:.2f} "
                f"edge={v.stair_edge_confidence:.2f} "
                f"pitch={s.pitch:.2f} roll={s.roll:.2f} pd={pd:.2f}"
            )

            if not v.yellow_found:
                yellow_lost += 1
                self.send("STOP")
                time.sleep(0.15)

                if yellow_lost >= YELLOW_LOST_LIMIT:
                    print("[FAIL] Yellow line lost too long")
                    return False

                continue

            yellow_lost = 0

            # Detect the wooden lifting part.
            if not wood_seen and abs(pd) >= WOOD_PITCH_DELTA_DEG:
                wood_seen = True
                wood_start_time = time.time()
                print("[WOOD] First lifting board detected, continue crossing it")

            if wood_seen:
                # Do not treat the first pitch bump as the real stair.
                # Continue forward until either pitch calms or enough time passes.
                if (
                    abs(pd) <= WOOD_CLEAR_PITCH_DEG
                    and wood_start_time is not None
                    and time.time() - wood_start_time > 1.2
                ):
                    print("[WOOD] Wood part cleared or stabilized")
                    return True

                if wood_start_time is not None and time.time() - wood_start_time > WOOD_MAX_CROSS_SEC:
                    print("[WOOD] Wood crossing max time reached, continue to stair detection")
                    return True

                self.follow_yellow_once(SMALL_FORWARD_MS)
                continue

            self.follow_yellow_once(APPROACH_FORWARD_MS)

        print("[FAIL] Approach wood timeout")
        return False

    def approach_until_front_on_step(self) -> bool:
        print("[STATE] APPROACH_UNTIL_FRONT_ON_STEP")

        start = time.time()
        edge_count = 0
        yellow_lost = 0

        while time.time() - start < APPROACH_MAX_SEC:
            if not self.safety_ok():
                return False

            s = self.sensor()
            v = self.vision.analyze()
            pd = self.pitch_delta()

            if v.stair_edge_confidence >= STAIR_EDGE_TRIGGER:
                edge_count += 1
            else:
                edge_count = 0

            print(
                f"[OBS] edge_count={edge_count} "
                f"yellow={v.yellow_found} err={v.yellow_error:.2f} "
                f"edge={v.stair_edge_confidence:.2f} "
                f"pitch={s.pitch:.2f} roll={s.roll:.2f} pd={pd:.2f}"
            )

            # Front is likely on / contacting step:
            # use MPU plus repeated stair edge confidence.
            if (
                abs(pd) >= FRONT_ON_STEP_PITCH_DELTA
                or edge_count >= STAIR_EDGE_REQUIRED_COUNT
            ):
                print("[STEP] Front part is on/contacting the step")
                self.send("STOP")
                time.sleep(0.25)
                return True

            if not v.yellow_found:
                yellow_lost += 1
                self.send("STOP")
                time.sleep(0.15)

                if yellow_lost >= YELLOW_LOST_LIMIT:
                    print("[FAIL] Yellow lost while approaching step")
                    return False

                continue

            yellow_lost = 0
            self.follow_yellow_once(APPROACH_FORWARD_MS)

        print("[FAIL] Front-on-step timeout")
        return False

    def rear_jack_lift(self) -> bool:
        print("[STATE] REAR_JACK_LIFT")

        before = self.sensor()

        if before is None:
            print("[FAIL] No sensor before rear jack")
            return False

        self.send("JACK:REAR:EXTEND")

        start = time.time()
        effect_started_at = None

        while time.time() - start < REAR_JACK_MAX_EXTEND_SEC:
            if not self.safety_ok():
                self.send("JACK:REAR:STOP")
                return False

            now = self.sensor()

            if now is None:
                time.sleep(0.08)
                continue

            pitch_effect = abs(now.pitch - before.pitch)
            roll_effect = abs(now.roll - before.roll)

            print(
                f"[JACK] pitch_effect={pitch_effect:.2f} "
                f"roll_effect={roll_effect:.2f}"
            )

            if (
                pitch_effect >= REAR_JACK_EFFECT_PITCH_DELTA
                or roll_effect >= REAR_JACK_EFFECT_ROLL_DELTA
            ):
                if effect_started_at is None:
                    effect_started_at = time.time()
                    print("[JACK] Rear jack affected the robot")

                if time.time() - effect_started_at >= REAR_JACK_EXTRA_AFTER_EFFECT_SEC:
                    break

            time.sleep(0.08)

        self.send("JACK:REAR:STOP")
        time.sleep(0.25)

        if effect_started_at is None:
            print("[FAIL] Rear jack did not affect MPU before timeout")
            return False

        print("[JACK] Rear jack lift finished")
        return True

    def climb_forward(self) -> bool:
        print("[STATE] CLIMB_FORWARD")

        start = time.time()
        high_pitch_seen = False

        while time.time() - start < CLIMB_MAX_SEC:
            if not self.safety_ok():
                return False

            s = self.sensor()
            pd = self.pitch_delta()
            v = self.vision.analyze()

            if abs(pd) >= FRONT_ON_STEP_PITCH_DELTA:
                high_pitch_seen = True

            print(
                f"[CLIMB] high_pitch_seen={high_pitch_seen} "
                f"yellow={v.yellow_found} err={v.yellow_error:.2f} "
                f"edge={v.stair_edge_confidence:.2f} "
                f"pitch={s.pitch:.2f} roll={s.roll:.2f} pd={pd:.2f}"
            )

            if v.yellow_found:
                self.follow_yellow_once(CLIMB_FORWARD_MS)
            else:
                # During climbing, yellow can disappear briefly.
                print("[CLIMB] Yellow missing, forward pulse by MPU state")
                self.pulse("FORWARD", CLIMB_FORWARD_MS)

            # Success: robot tilted during climb, then became near baseline.
            if high_pitch_seen and abs(pd) <= CLIMB_STABLE_PITCH_DELTA and time.time() - start > 3.0:
                print("[SUCCESS] Robot appears stable on top of step")
                self.send("STOP")
                return True

        print("[FAIL] Climb forward timeout")
        return False

    def retract_rear_jack_clear(self) -> bool:
        print("[STATE] RETRACT_REAR_JACK_CLEAR")

        self.send("JACK:REAR:RETRACT")
        time.sleep(REAR_JACK_RETRACT_CLEAR_SEC)
        self.send("JACK:REAR:STOP")

        print("[JACK] Rear jack retracted for clearance")
        return True

    def climb_one_step(self, step_number: int) -> bool:
        print("================================================")
        print(f"[STEP {step_number}] START")
        print("================================================")

        if not self.calibrate_mpu(label=f"step {step_number} start"):
            return False

        if not self.approach_and_cross_wood():
            return False

        # Recalibrate after crossing wood because robot attitude may change.
        self.calibrate_mpu(label=f"step {step_number} after wood")

        if not self.approach_until_front_on_step():
            return False

        if not self.rear_jack_lift():
            return False

        if not self.climb_forward():
            return False

        if not self.retract_rear_jack_clear():
            return False

        self.send("STOP")
        time.sleep(0.3)

        print(f"[STEP {step_number}] DONE")
        return True

    def run(self) -> int:
        print("================================================")
        print("Apex Rover Auto Stair Climb")
        print("Front camera + MPU only")
        print("Manual services remain running")
        print(f"Emergency stop: touch {AUTO_STOP_FILE}")
        print("================================================")

        if not self.commander.connect():
            print("[FAIL] Cannot connect to ESP32 WebSocket")
            return 1

        if not self.sensor_reader.wait_for_sensor(timeout_sec=8):
            print("[FAIL] No MPU sensor file data")
            print(f"Check sensor_bridge.py and {LAST_SENSOR_FILE}")
            return 2

        self.prepare_robot()

        for i in range(5, 0, -1):
            print(f"[AUTO] Starting in {i}...")
            time.sleep(1)

        try:
            for step in range(1, MAX_STEPS_TO_CLIMB + 1):
                if not self.climb_one_step(step):
                    print(f"[AUTO] Step {step} failed")
                    self.stop_all()
                    return 10 + step

            print("[AUTO] Requested steps completed")
            self.stop_all()
            return 0

        except KeyboardInterrupt:
            print("[AUTO] Keyboard interrupt")
            self.stop_all()
            return 90

        except RuntimeError as e:
            print(f"[AUTO] Runtime stop: {e}")
            self.stop_all()
            return 91

        finally:
            self.commander.close()


# ============================================================
# MAIN
# ============================================================

def cleanup_stop_file():
    try:
        if os.path.exists(AUTO_STOP_FILE):
            os.remove(AUTO_STOP_FILE)
    except Exception:
        pass


def main():
    cleanup_stop_file()

    brain = AutoStairBrain()

    def shutdown(sig, frame):
        print("\n[SHUTDOWN] Stopping robot...")
        brain.stop_all()
        sys.exit(1)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    return brain.run()


if __name__ == "__main__":
    sys.exit(main())