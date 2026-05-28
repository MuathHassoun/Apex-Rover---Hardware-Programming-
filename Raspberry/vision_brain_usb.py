import cv2
import time
import serial
import sys


# ==================================================
# Apex Rover Vision Brain - USB Camera Version
#
# Prepared for: Abdulhafiz - Apex Rover Graduation Project
#
# Raspberry Pi:
# - Opens USB camera
# - Processes frames
# - Detects target color by name
# - Decides camera command
# - Sends command to UNO
# - Requests sensors from Mega
#
# Example:
# python3 vision_brain_usb.py red
# python3 vision_brain_usb.py blue
# python3 vision_brain_usb.py green
# ==================================================


# -----------------------------
# Serial Ports
# -----------------------------

MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600


# -----------------------------
# Camera Settings
# -----------------------------

CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# -----------------------------
# Vision Decision Settings
# -----------------------------

CENTER_TOLERANCE = 70
MIN_OBJECT_AREA = 800
CAPTURE_DELAY = 0.05


# -----------------------------
# Target Color
# -----------------------------

TARGET_COLOR = "red"

if len(sys.argv) > 1:
    TARGET_COLOR = sys.argv[1].lower()


# -----------------------------
# HSV Color Ranges
# -----------------------------
#
# Colors supported:
# red, green, blue, yellow, orange, pink, purple, black, white
#
# HSV values may need small tuning depending on lighting.
# -----------------------------

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


# ==================================================
# Serial Device
# ==================================================

class SerialDevice:
    def __init__(self, name, port, baud_rate=9600):
        self.name = name
        self.port = port
        self.baud_rate = baud_rate
        self.device = None

    def connect(self):
        try:
            self.device = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)
            print(f"[OK] Connected to {self.name} on {self.port}")
            return True

        except Exception as e:
            print(f"[ERROR] Could not connect to {self.name} on {self.port}")
            print(e)
            return False

    def send(self, command):
        if self.device is None or not self.device.is_open:
            print(f"[ERROR] {self.name} not connected")
            return

        self.device.write((command + "\n").encode())
        self.device.flush()
        print(f"[SEND TO {self.name}] {command}")

    def read_line(self):
        if self.device is None or not self.device.is_open:
            return None

        try:
            line = self.device.readline().decode(errors="ignore").strip()
            return line if line else None
        except Exception:
            return None

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()


# ==================================================
# Vision Brain
# ==================================================

class ApexVisionBrainUSB:
    def __init__(self):
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)

        self.cap = None
        self.last_camera_command = "CAM:STOP"
        self.last_mega_sensor_time = 0

    # -----------------------------
    # Setup
    # -----------------------------

    def connect_controllers(self):
        self.mega.connect()
        self.uno.connect()

    def open_camera(self):
        self.cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("[ERROR] Could not open USB camera")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        print(f"[OK] USB camera opened on {CAMERA_DEVICE}")
        return True

    # -----------------------------
    # Image Processing
    # -----------------------------

    def detect_color_object(self, frame, color_name):
        """
        Detect object by color name.

        Supported colors:
        red, green, blue, yellow, orange, pink, purple, black, white
        """

        if color_name not in COLOR_RANGES:
            print(f"[ERROR] Unknown color: {color_name}")
            print("[INFO] Available colors:")
            for color in COLOR_RANGES.keys():
                print(f" - {color}")
            return None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = None

        for lower, upper in COLOR_RANGES[color_name]:
            current_mask = cv2.inRange(hsv, lower, upper)

            if mask is None:
                mask = current_mask
            else:
                mask = mask + current_mask

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

        center_x = x + w // 2
        center_y = y + h // 2

        detection = {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "center_x": center_x,
            "center_y": center_y,
            "area": area,
            "color": color_name
        }

        return detection, mask

    # -----------------------------
    # Decision Making
    # -----------------------------

    def decide_camera_command(self, detection):
        if detection is None:
            return "CAM:STOP"

        object_x = detection["center_x"]
        frame_center_x = FRAME_WIDTH // 2
        error_x = object_x - frame_center_x

        if error_x < -CENTER_TOLERANCE:
            return "CAM:LEFT"

        elif error_x > CENTER_TOLERANCE:
            return "CAM:RIGHT"

        else:
            return "CAM:STOP"

    def send_camera_command_if_changed(self, command):
        if command != self.last_camera_command:
            self.uno.send(command)
            self.last_camera_command = command

    def request_mega_sensors_every_second(self):
        now = time.time()

        if now - self.last_mega_sensor_time < 1.0:
            return

        self.last_mega_sensor_time = now

        self.mega.send("GET:SENSORS")
        time.sleep(0.15)

        response = self.mega.read_line()

        if response:
            print("[MEGA]", response)

    # -----------------------------
    # Drawing / Display
    # -----------------------------

    def draw_debug_frame(self, frame, detection, command):
        debug_frame = frame.copy()

        center_x = FRAME_WIDTH // 2

        cv2.line(
            debug_frame,
            (center_x, 0),
            (center_x, FRAME_HEIGHT),
            (255, 255, 255),
            2
        )

        cv2.putText(
            debug_frame,
            f"TARGET COLOR: {TARGET_COLOR.upper()}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2
        )

        if detection:
            x = detection["x"]
            y = detection["y"]
            w = detection["w"]
            h = detection["h"]
            cx = detection["center_x"]
            cy = detection["center_y"]
            area = detection["area"]

            cv2.rectangle(
                debug_frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            cv2.circle(
                debug_frame,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                debug_frame,
                f"{TARGET_COLOR.upper()} OBJECT area={int(area)}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug_frame,
                f"CMD: {command}",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2
            )

        else:
            cv2.putText(
                debug_frame,
                f"NO {TARGET_COLOR.upper()} OBJECT",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2
            )

            cv2.putText(
                debug_frame,
                "CMD: CAM:STOP",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2
            )

        return debug_frame

    # -----------------------------
    # Main Loop
    # -----------------------------

    def run(self):
        if TARGET_COLOR not in COLOR_RANGES:
            print(f"[ERROR] Unsupported color: {TARGET_COLOR}")
            print("[INFO] Run with one of these colors:")
            for color in COLOR_RANGES.keys():
                print(f"python3 vision_brain_usb.py {color}")
            return

        print("[SYSTEM] Apex Rover Vision Brain Started")
        print(f"[INFO] Target color: {TARGET_COLOR}")
        print("[INFO] Show the target color object to the camera")
        print("[INFO] Press s to save result image")
        print("[INFO] Press m to save mask image")
        print("[INFO] Press q to quit")

        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("[WARNING] Failed to capture frame")
                time.sleep(0.2)
                continue

            detection, mask = self.detect_color_object(frame, TARGET_COLOR)

            command = self.decide_camera_command(detection)

            self.send_camera_command_if_changed(command)

            self.request_mega_sensors_every_second()

            debug_frame = self.draw_debug_frame(frame, detection, command)

            cv2.imshow("Apex Rover Vision", debug_frame)
            cv2.imshow(f"{TARGET_COLOR.upper()} Detection Mask", mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                filename = f"apex_{TARGET_COLOR}_vision_result.jpg"
                cv2.imwrite(filename, debug_frame)
                print(f"[OK] Saved result image: {filename}")

            if key == ord("m"):
                filename = f"apex_{TARGET_COLOR}_detection_mask.jpg"
                cv2.imwrite(filename, mask)
                print(f"[OK] Saved mask image: {filename}")

            if key == ord("q"):
                break

            time.sleep(CAPTURE_DELAY)

    # -----------------------------
    # Shutdown
    # -----------------------------

    def shutdown(self):
        print("[SYSTEM] Shutdown")

        try:
            self.uno.send("CAM:STOP")
            self.mega.send("STOP")
        except Exception:
            pass

        if self.cap:
            self.cap.release()

        self.uno.close()
        self.mega.close()
        cv2.destroyAllWindows()


# ==================================================
# Main
# ==================================================

def main():
    brain = ApexVisionBrainUSB()

    try:
        brain.connect_controllers()

        if not brain.open_camera():
            return

        brain.run()

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Keyboard Interrupt")

    finally:
        brain.shutdown()


if __name__ == "__main__":
    main()