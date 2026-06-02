import cv2
import time
import serial
import threading
import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict


# ============================================================
# Apex Rover Integrated Brain
#
# Raspberry Pi 4 Model B
#
# Integrated Parts:
# 1) Control Brain:
#    - Mega movement
#    - Jacks
#    - MPU6500 pitch/roll
#    - Ultrasonic sensors
#    - Safety checks
#
# 2) Vision Brain:
#    - USB camera
#    - Color/object detection
#    - Stairs detection
#    - Camera servo commands to UNO
#    - Autonomous decisions to Mega
#
# Prepared for Apex Rover Graduation Project
# ============================================================


# ============================================================
# Serial Ports
# ============================================================

MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600


# ============================================================
# Camera Settings
# ============================================================

CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# ============================================================
# Modes
# ============================================================

MODE_IDLE = "IDLE"
MODE_MANUAL = "MANUAL"
MODE_OBJECT = "OBJECT"
MODE_STAIRS = "STAIRS"
MODE_CLIMB_ASSIST = "CLIMB_ASSIST"


# ============================================================
# Safety Limits
# ============================================================

MAX_SAFE_PITCH = 25.0
MAX_SAFE_ROLL = 25.0

DEFAULT_SPEED = 60
CLIMB_SPEED = 40
STAIRS_SPEED = 35

FRONT_JACK_TARGET_CM = 10.0
REAR_JACK_TARGET_CM = 10.0
JACK_TOLERANCE_CM = 0.5


# ============================================================
# Vision Settings
# ============================================================

CAPTURE_DELAY = 0.05

CENTER_TOLERANCE = 70
MIN_OBJECT_AREA = 800
OBJECT_REACHED_AREA = 5000

STAIR_MIN_LINES = 4
STAIR_CENTER_TOLERANCE = 90
STAIR_TOO_CLOSE_Y = 390
STAIR_MIN_LINE_LENGTH = 80
STAIR_MAX_LINE_GAP = 25


# ============================================================
# HSV Color Ranges
# ============================================================

COLOR_RANGES = {
    "red": [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (180, 255, 255))
    ],
    "green": [
        ((35, 70, 50), (85, 255, 255))
    ],
    "blue": [
        ((90, 70, 50), (130, 255, 255))
    ],
    "yellow": [
        ((20, 100, 100), (35, 255, 255))
    ],
    "orange": [
        ((10, 100, 100), (25, 255, 255))
    ],
    "pink": [
        ((140, 60, 80), (170, 255, 255))
    ],
    "purple": [
        ((125, 60, 50), (155, 255, 255))
    ],
    "black": [
        ((0, 0, 0), (180, 255, 60))
    ],
    "white": [
        ((0, 0, 180), (180, 60, 255))
    ]
}


# ============================================================
# Sensor Data
# ============================================================

@dataclass
class SensorData:
    pitch: float = 0.0
    roll: float = 0.0
    front_ultrasonic: float = 0.0
    rear_ultrasonic: float = 0.0

    def is_safe_angle(self) -> bool:
        return abs(self.pitch) <= MAX_SAFE_PITCH and abs(self.roll) <= MAX_SAFE_ROLL

    def is_front_close(self, limit_cm: float = 8.0) -> bool:
        return self.front_ultrasonic > 0 and self.front_ultrasonic <= limit_cm

    def is_rear_close(self, limit_cm: float = 8.0) -> bool:
        return self.rear_ultrasonic > 0 and self.rear_ultrasonic <= limit_cm


# ============================================================
# Serial Device
# ============================================================

class SerialDevice:
    def __init__(self, name: str, port: str, baud_rate: int = 9600):
        self.name = name
        self.port = port
        self.baud_rate = baud_rate
        self.device: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    def connect(self) -> bool:
        try:
            self.device = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.05,
                write_timeout=1
            )

            # Arduino resets when serial opens
            time.sleep(2)

            print(f"[OK] Connected to {self.name} on {self.port}")
            return True

        except Exception as e:
            print(f"[ERROR] Could not connect to {self.name} on {self.port}")
            print(e)
            self.device = None
            return False

    def is_connected(self) -> bool:
        return self.device is not None and self.device.is_open

    def send(self, command: str) -> bool:
        if not self.is_connected():
            print(f"[ERROR] {self.name} is not connected")
            return False

        try:
            with self.lock:
                self.device.write((command + "\n").encode())
                self.device.flush()

            print(f"[SEND TO {self.name}] {command}")
            return True

        except Exception as e:
            print(f"[ERROR] Failed to send to {self.name}: {e}")
            return False

    def read_line(self) -> Optional[str]:
        if not self.is_connected():
            return None

        try:
            with self.lock:
                line = self.device.readline().decode(errors="ignore").strip()

            if line:
                return line

            return None

        except Exception as e:
            print(f"[ERROR] Failed reading from {self.name}: {e}")
            return None

    def close(self):
        try:
            if self.device and self.device.is_open:
                self.device.close()
                print(f"[OK] Closed {self.name}")
        except Exception:
            pass


# ============================================================
# Integrated Apex Brain
# ============================================================

class ApexIntegratedBrain:
    def __init__(self):
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)

        self.cap = None

        self.mode = MODE_MANUAL
        self.target_color = "red"

        self.latest_sensor_data = SensorData()
        self.speed = DEFAULT_SPEED

        self.running = True

        self.last_move_command = None
        self.last_camera_command = None

        self.last_sensor_request_time = 0
        self.last_debug_save_time = 0

    # ========================================================
    # Setup
    # ========================================================

    def connect_all(self):
        print("Connecting Apex Rover integrated brain...")
        print("-----------------------------------------")

        mega_ok = self.mega.connect()
        uno_ok = self.uno.connect()

        print("-----------------------------------------")

        if mega_ok:
            print("[READY] Mega controller ready")
        else:
            print("[WARNING] Mega controller not ready")

        if uno_ok:
            print("[READY] UNO camera controller ready")
        else:
            print("[WARNING] UNO camera controller not ready")

        print()

    def open_camera(self):
        self.cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("[ERROR] Could not open USB camera")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        print(f"[OK] USB camera opened on {CAMERA_DEVICE}")
        return True

    # ========================================================
    # Low-Level Send Helpers
    # ========================================================

    def send_move_if_changed(self, command: str):
        if command != self.last_move_command:
            self.mega.send(command)
            self.last_move_command = command

    def send_camera_if_changed(self, command: str):
        if command != self.last_camera_command:
            self.uno.send(command)
            self.last_camera_command = command

    def set_speed(self, speed: int):
        speed = max(0, min(100, speed))
        self.speed = speed
        self.mega.send(f"SPEED:{speed}")

    # ========================================================
    # Movement Commands
    # ========================================================

    def forward(self):
        if self.check_safety_before_movement():
            self.mega.send("FORWARD")
            self.last_move_command = "FORWARD"

    def backward(self):
        if self.check_safety_before_movement():
            self.mega.send("BACKWARD")
            self.last_move_command = "BACKWARD"

    def left(self):
        if self.check_safety_before_movement():
            self.mega.send("LEFT")
            self.last_move_command = "LEFT"

    def right(self):
        if self.check_safety_before_movement():
            self.mega.send("RIGHT")
            self.last_move_command = "RIGHT"

    def stop(self):
        self.mega.send("STOP")
        self.last_move_command = "STOP"

    def apply_move_command(self, command: str):
        """
        Converts decision command to the actual Mega command.
        Accepts both:
            MOVE:FORWARD
            FORWARD
        """

        command = command.strip().upper()

        if command.startswith("MOVE:"):
            command = command.replace("MOVE:", "")

        if command == "FORWARD":
            self.forward()
        elif command == "BACKWARD":
            self.backward()
        elif command == "LEFT":
            self.left()
        elif command == "RIGHT":
            self.right()
        elif command == "SLOW":
            self.set_speed(STAIRS_SPEED)
            self.forward()
        elif command == "STOP":
            self.stop()
        else:
            print(f"[WARNING] Unknown move command: {command}")

    # ========================================================
    # Mega Modes
    # ========================================================

    def mode_normal(self):
        self.mega.send("MODE:NORMAL")

    def mode_climb(self):
        self.mega.send("MODE:CLIMB")

    # ========================================================
    # Jacks
    # ========================================================

    def front_jack_extend(self):
        self.mega.send("JACK:FRONT:EXTEND")

    def front_jack_retract(self):
        self.mega.send("JACK:FRONT:RETRACT")

    def front_jack_stop(self):
        self.mega.send("JACK:FRONT:STOP")

    def rear_jack_extend(self):
        self.mega.send("JACK:REAR:EXTEND")

    def rear_jack_retract(self):
        self.mega.send("JACK:REAR:RETRACT")

    def rear_jack_stop(self):
        self.mega.send("JACK:REAR:STOP")

    def stop_all_jacks(self):
        self.front_jack_stop()
        self.rear_jack_stop()

    # ========================================================
    # Camera Commands
    # ========================================================

    def camera_left(self):
        self.send_camera_if_changed("CAM:LEFT")

    def camera_right(self):
        self.send_camera_if_changed("CAM:RIGHT")

    def camera_stop(self):
        self.send_camera_if_changed("CAM:STOP")

    def camera_up(self):
        self.send_camera_if_changed("CAM:UP")

    def camera_down(self):
        self.send_camera_if_changed("CAM:DOWN")

    def camera_center(self):
        self.send_camera_if_changed("CAM:CENTER")

    # ========================================================
    # Sensor Reading
    # ========================================================

    def parse_sensor_data(self, response: str) -> Optional[SensorData]:
        """
        Expected Mega response:

        DATA:PITCH=5.2;ROLL=-1.4;UF=8.5;UR=9.1
        """

        if not response.startswith("DATA:"):
            return None

        try:
            response = response.replace("DATA:", "")
            parts = response.split(";")

            values: Dict[str, float] = {}

            for part in parts:
                if "=" not in part:
                    continue

                key, value = part.split("=")
                values[key.strip().upper()] = float(value.strip())

            return SensorData(
                pitch=values.get("PITCH", 0.0),
                roll=values.get("ROLL", 0.0),
                front_ultrasonic=values.get("UF", 0.0),
                rear_ultrasonic=values.get("UR", 0.0)
            )

        except Exception as e:
            print("[ERROR] Failed to parse sensor data")
            print(e)
            return None

    def request_sensors_non_blocking(self):
        """
        Sends request every 1 second.
        Incoming DATA response is parsed in handle_incoming_mega_line().
        """

        now = time.time()

        if now - self.last_sensor_request_time < 1.0:
            return

        self.last_sensor_request_time = now
        self.mega.send("GET:SENSORS")

    def check_safety_before_movement(self) -> bool:
        data = self.latest_sensor_data

        if not data.is_safe_angle():
            print("[SAFETY STOP] Unsafe pitch/roll detected")
            print(f"Pitch={data.pitch:.2f}, Roll={data.roll:.2f}")
            self.stop()
            self.stop_all_jacks()
            return False

        return True

    def print_sensor_data(self):
        data = self.latest_sensor_data

        print()
        print("========== Latest Sensor Data ==========")
        print(f"Pitch              : {data.pitch:.2f} deg")
        print(f"Roll               : {data.roll:.2f} deg")
        print(f"Front Ultrasonic   : {data.front_ultrasonic:.2f} cm")
        print(f"Rear Ultrasonic    : {data.rear_ultrasonic:.2f} cm")
        print(f"Safe Angle         : {data.is_safe_angle()}")
        print("========================================")
        print()

    # ========================================================
    # Incoming Commands From Mega / ESP32
    # ========================================================

    def handle_incoming_mega_line(self):
        """
        Reads one line from Mega.

        Mega may send:
            DATA:PITCH=...;ROLL=...;UF=...;UR=...
            MODE:OBJECT
            MODE:STAIRS
            MODE:MANUAL
            MODE:IDLE
            TARGET:red
            TARGET:blue
            CMD:STOP
        """

        line = self.mega.read_line()

        if not line:
            return

        print(f"[FROM MEGA/ESP32] {line}")

        # Sensor data
        if line.startswith("DATA:"):
            data = self.parse_sensor_data(line)

            if data:
                self.latest_sensor_data = data

            return

        # Mode control
        if line.startswith("MODE:"):
            new_mode = line.replace("MODE:", "").strip().upper()

            if new_mode in [MODE_IDLE, MODE_MANUAL, MODE_OBJECT, MODE_STAIRS, MODE_CLIMB_ASSIST]:
                self.change_mode(new_mode)
            else:
                print(f"[WARNING] Unknown mode: {new_mode}")

            return

        # Target object color
        if line.startswith("TARGET:"):
            new_color = line.replace("TARGET:", "").strip().lower()

            if new_color in COLOR_RANGES:
                self.target_color = new_color
                print(f"[BRAIN] Target color changed to: {self.target_color}")
            else:
                print(f"[WARNING] Unsupported target color: {new_color}")

            return

        # Emergency/manual command
        if line.startswith("CMD:"):
            cmd = line.replace("CMD:", "").strip().upper()

            if cmd == "STOP":
                self.emergency_stop()

            return

    def change_mode(self, new_mode: str):
        self.mode = new_mode
        print(f"[BRAIN] Mode changed to: {self.mode}")

        if self.mode == MODE_IDLE:
            self.stop()
            self.camera_stop()

        elif self.mode == MODE_MANUAL:
            self.camera_center()

        elif self.mode == MODE_OBJECT:
            self.set_speed(DEFAULT_SPEED)
            self.camera_center()

        elif self.mode == MODE_STAIRS:
            self.set_speed(STAIRS_SPEED)
            self.camera_down()

        elif self.mode == MODE_CLIMB_ASSIST:
            self.set_speed(CLIMB_SPEED)
            self.mode_climb()
            self.camera_down()

    # ========================================================
    # Vision: Object Detection
    # ========================================================

    def detect_color_object(self, frame, color_name):
        if color_name not in COLOR_RANGES:
            return None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = None

        for lower, upper in COLOR_RANGES[color_name]:
            lower_np = np.array(lower)
            upper_np = np.array(upper)

            current_mask = cv2.inRange(hsv, lower_np, upper_np)

            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(mask, current_mask)

        # Clean noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < MIN_OBJECT_AREA:
            return None, mask

        x, y, w, h = cv2.boundingRect(largest)

        detection = {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "center_x": x + w // 2,
            "center_y": y + h // 2,
            "area": area,
            "color": color_name
        }

        return detection, mask

    def decide_object_action(self, detection):
        if detection is None:
            return "STOP", "CAM:STOP", "OBJECT_NOT_FOUND"

        object_x = detection["center_x"]
        object_area = detection["area"]

        frame_center_x = FRAME_WIDTH // 2
        error_x = object_x - frame_center_x

        if error_x < -CENTER_TOLERANCE:
            return "LEFT", "CAM:LEFT", "OBJECT_LEFT"

        elif error_x > CENTER_TOLERANCE:
            return "RIGHT", "CAM:RIGHT", "OBJECT_RIGHT"

        else:
            if object_area < OBJECT_REACHED_AREA:
                return "FORWARD", "CAM:STOP", "OBJECT_CENTER_FAR"
            else:
                return "STOP", "CAM:STOP", "OBJECT_REACHED"

    # ========================================================
    # Vision: Stairs Detection
    # ========================================================

    def detect_stairs(self, frame):
        debug = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(blur, 60, 160)

        roi_y_start = FRAME_HEIGHT // 2
        roi = edges[roi_y_start:FRAME_HEIGHT, 0:FRAME_WIDTH]

        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi / 180,
            threshold=60,
            minLineLength=STAIR_MIN_LINE_LENGTH,
            maxLineGap=STAIR_MAX_LINE_GAP
        )

        stair_lines = []
        centers_x = []
        centers_y = []
        slopes = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                y1_full = y1 + roi_y_start
                y2_full = y2 + roi_y_start

                dx = x2 - x1
                dy = y2 - y1

                if dx == 0:
                    continue

                slope = dy / dx
                angle = abs(math.degrees(math.atan(slope)))
                length = math.sqrt(dx * dx + dy * dy)

                # Stairs appear mostly as horizontal lines in the bottom half.
                if angle < 25 and length >= STAIR_MIN_LINE_LENGTH:
                    stair_lines.append((x1, y1_full, x2, y2_full))
                    centers_x.append((x1 + x2) // 2)
                    centers_y.append((y1_full + y2_full) // 2)
                    slopes.append(slope)

                    cv2.line(debug, (x1, y1_full), (x2, y2_full), (0, 255, 0), 2)

        if len(stair_lines) < STAIR_MIN_LINES:
            result = {
                "stairs_found": False,
                "state": "NO_STAIRS",
                "line_count": len(stair_lines),
                "center_x": None,
                "center_y": None,
                "avg_slope": None
            }

            return result, debug, edges

        avg_center_x = int(sum(centers_x) / len(centers_x))
        avg_center_y = int(sum(centers_y) / len(centers_y))
        avg_slope = sum(slopes) / len(slopes)

        cv2.circle(debug, (avg_center_x, avg_center_y), 8, (0, 0, 255), -1)

        result = {
            "stairs_found": True,
            "state": "STAIRS_FOUND",
            "line_count": len(stair_lines),
            "center_x": avg_center_x,
            "center_y": avg_center_y,
            "avg_slope": avg_slope
        }

        return result, debug, edges

    def decide_stairs_action(self, stairs):
        """
        Uses camera + latest sensors.
        This is the functional integration point.
        """

        data = self.latest_sensor_data

        # First safety: angle
        if not data.is_safe_angle():
            return "STOP", "UNSAFE_ANGLE_STOP"

        # If front is very close, do not hit hard.
        if data.is_front_close(6.0):
            return "STOP", "FRONT_TOO_CLOSE_STOP"

        # If camera does not see stairs, stop.
        if not stairs["stairs_found"]:
            return "STOP", "NO_STAIRS_STOP"

        frame_center_x = FRAME_WIDTH // 2
        stairs_center_x = stairs["center_x"]
        stairs_center_y = stairs["center_y"]
        avg_slope = stairs["avg_slope"]

        error_x = stairs_center_x - frame_center_x

        # If stair lines are very low in frame, the robot is close.
        if stairs_center_y > STAIR_TOO_CLOSE_Y:
            return "SLOW", "STAIRS_TOO_CLOSE_GO_SLOW"

        if error_x < -STAIR_CENTER_TOLERANCE:
            return "LEFT", "ALIGN_LEFT_TO_STAIRS"

        if error_x > STAIR_CENTER_TOLERANCE:
            return "RIGHT", "ALIGN_RIGHT_TO_STAIRS"

        if avg_slope is not None:
            if avg_slope > 0.18:
                return "RIGHT", "CORRECT_TILT_RIGHT"
            elif avg_slope < -0.18:
                return "LEFT", "CORRECT_TILT_LEFT"

        return "FORWARD", "STAIRS_CENTERED_FORWARD"

    # ========================================================
    # Climb Assist
    # ========================================================

    def climb_assist_step(self, stairs):
        """
        Semi-autonomous climb helper.

        Logic:
        - Camera says if stairs are centered.
        - MPU/ultrasonic sensors protect from unsafe movement.
        - Jacks can be used only when needed.
        """

        data = self.latest_sensor_data

        if not data.is_safe_angle():
            self.stop()
            self.stop_all_jacks()
            return "UNSAFE_ANGLE_STOP"

        if not stairs["stairs_found"]:
            self.stop()
            return "NO_STAIRS_WAITING"

        move_cmd, state = self.decide_stairs_action(stairs)

        # If the robot is near a step and still safe, move slowly.
        if state == "STAIRS_TOO_CLOSE_GO_SLOW":
            self.set_speed(CLIMB_SPEED)
            self.apply_move_command("SLOW")

        # If pitch is increasing but still safe, use front jack assist.
        elif data.pitch > 12.0:
            self.stop()
            self.front_jack_extend()
            time.sleep(0.2)
            self.front_jack_stop()
            state = "FRONT_JACK_ASSIST"

        # If rear angle/position needs support, use rear jack assist.
        elif data.pitch < -8.0:
            self.stop()
            self.rear_jack_extend()
            time.sleep(0.2)
            self.rear_jack_stop()
            state = "REAR_JACK_ASSIST"

        else:
            self.apply_move_command(move_cmd)

        return state

    # ========================================================
    # Drawing Debug
    # ========================================================

    def draw_common_status(self, frame):
        cv2.putText(frame, f"MODE: {self.mode}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        cv2.putText(frame, f"SPEED: {self.speed}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        data = self.latest_sensor_data

        cv2.putText(frame, f"PITCH: {data.pitch:.1f}  ROLL: {data.roll:.1f}", (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        cv2.putText(frame, f"UF: {data.front_ultrasonic:.1f}cm  UR: {data.rear_ultrasonic:.1f}cm", (20, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    def draw_object_debug(self, frame, detection, move_cmd, cam_cmd, state):
        debug = frame.copy()
        self.draw_common_status(debug)

        cv2.putText(debug, f"TARGET: {self.target_color.upper()}", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        cv2.putText(debug, f"STATE: {state}", (20, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        cv2.putText(debug, f"MEGA MOVE: {move_cmd}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv2.putText(debug, f"UNO CAM: {cam_cmd}", (20, 275),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        center_x = FRAME_WIDTH // 2
        cv2.line(debug, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 2)

        if detection:
            x = detection["x"]
            y = detection["y"]
            w = detection["w"]
            h = detection["h"]
            cx = detection["center_x"]
            cy = detection["center_y"]

            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 5, (0, 0, 255), -1)

            cv2.putText(debug, f"AREA: {int(detection['area'])}", (20, 310),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        return debug

    def draw_stairs_debug(self, debug, stairs, move_cmd, state):
        self.draw_common_status(debug)

        cv2.putText(debug, f"STATE: {state}", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        cv2.putText(debug, f"MEGA MOVE: {move_cmd}", (20, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv2.putText(debug, f"LINES: {stairs['line_count']}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        center_x = FRAME_WIDTH // 2
        cv2.line(debug, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 2)

        return debug

    def draw_idle_debug(self, frame):
        debug = frame.copy()
        self.draw_common_status(debug)

        cv2.putText(debug, "Robot stopped / manual waiting", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        return debug

    # ========================================================
    # Emergency / Shutdown
    # ========================================================

    def emergency_stop(self):
        print("[EMERGENCY STOP] Stopping all systems")
        self.stop()
        self.stop_all_jacks()
        self.camera_stop()

    def shutdown(self):
        print("[SYSTEM] Safe shutdown")
        self.running = False

        try:
            self.emergency_stop()
        except Exception:
            pass

        time.sleep(0.3)

        if self.cap:
            self.cap.release()

        self.mega.close()
        self.uno.close()
        cv2.destroyAllWindows()

    # ========================================================
    # Main Loop
    # ========================================================

    def run(self):
        print("[SYSTEM] Apex Integrated Brain Started")
        print("[INFO] Keyboard:")
        print("  0 = IDLE")
        print("  m = MANUAL")
        print("  1 = OBJECT")
        print("  2 = STAIRS")
        print("  3 = CLIMB_ASSIST")
        print("  f/b/l/r/x = manual movement")
        print("  w/s/a/d/c = camera up/down/left/right/center")
        print("  p = print latest sensors")
        print("  q = quit")
        print()

        self.set_speed(DEFAULT_SPEED)
        self.camera_center()

        while self.running:
            self.handle_incoming_mega_line()
            self.request_sensors_non_blocking()

            ret, frame = self.cap.read()

            if not ret:
                print("[WARNING] Failed to capture frame")
                time.sleep(0.2)
                continue

            mask = None

            if self.mode == MODE_OBJECT:
                detection, mask = self.detect_color_object(frame, self.target_color)
                move_cmd, cam_cmd, state = self.decide_object_action(detection)

                self.apply_move_command(move_cmd)
                self.send_camera_if_changed(cam_cmd)

                debug_frame = self.draw_object_debug(frame, detection, move_cmd, cam_cmd, state)

            elif self.mode == MODE_STAIRS:
                stairs, stairs_debug, edges = self.detect_stairs(frame)
                move_cmd, state = self.decide_stairs_action(stairs)

                self.apply_move_command(move_cmd)
                self.camera_down()

                debug_frame = self.draw_stairs_debug(stairs_debug, stairs, move_cmd, state)
                mask = edges

            elif self.mode == MODE_CLIMB_ASSIST:
                stairs, stairs_debug, edges = self.detect_stairs(frame)
                state = self.climb_assist_step(stairs)

                self.camera_down()

                debug_frame = self.draw_stairs_debug(stairs_debug, stairs, "AUTO", state)
                mask = edges

            else:
                debug_frame = self.draw_idle_debug(frame)

            cv2.imshow("Apex Rover Integrated Brain", debug_frame)

            if mask is not None:
                cv2.imshow("Vision Mask / Edges", mask)

            key = cv2.waitKey(1) & 0xFF

            # -------------------------
            # Mode keys
            # -------------------------
            if key == ord("q"):
                break

            elif key == ord("0"):
                self.change_mode(MODE_IDLE)

            elif key == ord("m"):
                self.change_mode(MODE_MANUAL)

            elif key == ord("1"):
                self.change_mode(MODE_OBJECT)

            elif key == ord("2"):
                self.change_mode(MODE_STAIRS)

            elif key == ord("3"):
                self.change_mode(MODE_CLIMB_ASSIST)

            # -------------------------
            # Manual movement keys
            # -------------------------
            elif key == ord("f"):
                self.forward()

            elif key == ord("b"):
                self.backward()

            elif key == ord("l"):
                self.left()

            elif key == ord("r"):
                self.right()

            elif key == ord("x"):
                self.stop()

            # -------------------------
            # Camera keys
            # -------------------------
            elif key == ord("w"):
                self.camera_up()

            elif key == ord("s"):
                self.camera_down()

            elif key == ord("a"):
                self.camera_left()

            elif key == ord("d"):
                self.camera_right()

            elif key == ord("c"):
                self.camera_center()

            # -------------------------
            # Sensors
            # -------------------------
            elif key == ord("p"):
                self.print_sensor_data()

            time.sleep(CAPTURE_DELAY)

        self.shutdown()


# ============================================================
# Main
# ============================================================

def main():
    brain = ApexIntegratedBrain()

    try:
        brain.connect_all()

        if not brain.open_camera():
            brain.shutdown()
            return

        brain.run()

    except KeyboardInterrupt:
        print()
        brain.shutdown()

    except Exception as e:
        print("[ERROR] Fatal brain error")
        print(e)
        brain.shutdown()


if __name__ == "__main__":
    main()
