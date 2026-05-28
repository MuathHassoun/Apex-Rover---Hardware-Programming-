import cv2
import math
import numpy as np

from config import (
    CAMERA_DEVICE,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    COLOR_RANGES,
    CENTER_TOLERANCE,
    MIN_OBJECT_AREA,
    OBJECT_REACHED_AREA,
    STAIR_MIN_LINES,
    STAIR_CENTER_TOLERANCE,
    STAIR_TOO_CLOSE_Y,
    STAIR_MIN_LINE_LENGTH,
    STAIR_MAX_LINE_GAP
)


class VisionBrain:
    """
    This file does camera intelligence only:
    - open USB camera
    - detect color object
    - detect stairs
    - decide object movement suggestion
    - decide stairs movement suggestion

    It does NOT send serial commands directly.
    The main brain connects this file with ControlBrain.
    """

    def __init__(self):
        self.cap = None
        self.target_color = "red"

    # ========================================================
    # Camera
    # ========================================================

    def open_camera(self) -> bool:
        self.cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("[ERROR] Could not open USB camera")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        # Reduce camera latency if backend supports it
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        print(f"[OK] USB camera opened on {CAMERA_DEVICE}")
        return True

    def read_frame(self):
        if self.cap is None:
            return False, None

        return self.cap.read()

    def close(self):
        if self.cap:
            self.cap.release()

        cv2.destroyAllWindows()

    # ========================================================
    # Target color
    # ========================================================

    def set_target_color(self, color_name: str):
        color_name = color_name.strip().lower()

        if color_name in COLOR_RANGES:
            self.target_color = color_name
            print(f"[VISION] Target color changed to: {self.target_color}")
            return True

        print(f"[WARNING] Unsupported color: {color_name}")
        print("[INFO] Available colors:", ", ".join(COLOR_RANGES.keys()))
        return False

    # ========================================================
    # Object detection
    # ========================================================

    def detect_color_object(self, frame):
        color_name = self.target_color

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
        """
        Returns:
            move_command, camera_command, state
        """

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
    # Stairs detection
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

                # Stairs usually appear as horizontal or slightly tilted lines.
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

    def decide_stairs_action(self, stairs, sensor_data):
        """
        Functional integration point:
        Vision decision + sensor safety data.

        Returns:
            move_command, state
        """

        if not sensor_data.is_safe_angle():
            return "STOP", "UNSAFE_ANGLE_STOP"

        if sensor_data.is_front_close(6.0):
            return "STOP", "FRONT_TOO_CLOSE_STOP"

        if not stairs["stairs_found"]:
            return "STOP", "NO_STAIRS_STOP"

        frame_center_x = FRAME_WIDTH // 2
        stairs_center_x = stairs["center_x"]
        stairs_center_y = stairs["center_y"]
        avg_slope = stairs["avg_slope"]

        error_x = stairs_center_x - frame_center_x

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
    # Drawing
    # ========================================================

    def draw_common_status(self, frame, mode, speed, sensor_data):
        cv2.putText(frame, f"MODE: {mode}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        cv2.putText(frame, f"SPEED: {speed}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        cv2.putText(frame, f"PITCH: {sensor_data.pitch:.1f}  ROLL: {sensor_data.roll:.1f}", (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        cv2.putText(frame, f"UF: {sensor_data.front_ultrasonic:.1f}cm  UR: {sensor_data.rear_ultrasonic:.1f}cm", (20, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    def draw_object_debug(self, frame, detection, move_cmd, cam_cmd, state, mode, speed, sensor_data):
        debug = frame.copy()
        self.draw_common_status(debug, mode, speed, sensor_data)

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

    def draw_stairs_debug(self, debug, stairs, move_cmd, state, mode, speed, sensor_data):
        self.draw_common_status(debug, mode, speed, sensor_data)

        cv2.putText(debug, f"STATE: {state}", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        cv2.putText(debug, f"MEGA MOVE: {move_cmd}", (20, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv2.putText(debug, f"LINES: {stairs['line_count']}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

        center_x = FRAME_WIDTH // 2
        cv2.line(debug, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 255), 2)

        return debug

    def draw_idle_debug(self, frame, mode, speed, sensor_data):
        debug = frame.copy()
        self.draw_common_status(debug, mode, speed, sensor_data)

        cv2.putText(debug, "Robot stopped / manual waiting", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        return debug
