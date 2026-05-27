import time
import cv2
import serial
from picamera2 import Picamera2


# ==================================================
# Apex Rover Vision Brain
#
# Raspberry Pi = Brain
# Camera       = Eyes
# UNO          = Camera mount controller
# Mega         = Movement + jacks + sensors
#
# This version:
# - Opens Raspberry Pi camera
# - Captures frames
# - Finds a simple colored object
# - Decides LEFT / RIGHT / STOP
# - Sends camera commands to UNO
# - Keeps placeholders for Mega movement decisions
# ==================================================


# ==================================================
# Serial Ports
# ==================================================

MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600


# ==================================================
# Vision Settings
# ==================================================

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

CENTER_TOLERANCE = 70

MIN_OBJECT_AREA = 800

CAPTURE_DELAY = 0.05


# ==================================================
# Serial Device Class
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
            print(f"[ERROR] {self.name} is not connected")
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
        except:
            return None

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()


# ==================================================
# Apex Vision Brain Class
# ==================================================

class ApexVisionBrain:
    def __init__(self):
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)

        self.camera = None
        self.last_camera_command = "CAM:STOP"

    # ------------------------------
    # Setup
    # ------------------------------

    def connect_controllers(self):
        self.mega.connect()
        self.uno.connect()

    def start_camera(self):
        self.camera = Picamera2()

        config = self.camera.create_preview_configuration(
            main={
                "size": (FRAME_WIDTH, FRAME_HEIGHT),
                "format": "RGB888"
            }
        )

        self.camera.configure(config)
        self.camera.start()

        time.sleep(1)
        print("[OK] Camera started")

    # ------------------------------
    # Camera Processing
    # ------------------------------

    def get_frame(self):
        frame = self.camera.capture_array()
        return frame

    def detect_colored_object(self, frame):
        """
        First simple processing:
        Detect a red object.

        Later we can replace this with:
        - stair edge detection
        - obstacle detection
        - YOLO
        - line tracking
        - AI model
        """

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Red has two ranges in HSV
        lower_red_1 = (0, 120, 70)
        upper_red_1 = (10, 255, 255)

        lower_red_2 = (170, 120, 70)
        upper_red_2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        mask = mask1 + mask2

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

        result = {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "center_x": center_x,
            "center_y": center_y,
            "area": area
        }

        return result, mask

    # ------------------------------
    # Decision Logic
    # ------------------------------

    def decide_camera_command(self, detection):
        """
        Decide how to move camera based on object position.
        """

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
        """
        Avoid sending the same command too many times.
        """

        if command != self.last_camera_command:
            self.uno.send(command)
            self.last_camera_command = command

    def request_mega_sensors(self):
        """
        Optional: read MPU + ultrasonic from Mega.
        """

        self.mega.send("GET:SENSORS")
        time.sleep(0.2)

        response = self.mega.read_line()

        if response:
            print("[MEGA]", response)

        return response

    # ------------------------------
    # Main Vision Loop
    # ------------------------------

    def run(self):
        print("[SYSTEM] Apex Vision Brain started")
        print("[INFO] Press CTRL+C to stop")

        last_sensor_time = 0

        while True:
            frame = self.get_frame()

            detection, mask = self.detect_colored_object(frame)

            command = self.decide_camera_command(detection)
            self.send_camera_command_if_changed(command)

            # Optional sensor request every 1 second
            if time.time() - last_sensor_time > 1.0:
                last_sensor_time = time.time()
                self.request_mega_sensors()

            # Draw information on frame
            debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame_center_x = FRAME_WIDTH // 2

            cv2.line(
                debug_frame,
                (frame_center_x, 0),
                (frame_center_x, FRAME_HEIGHT),
                (255, 255, 255),
                2
            )

            if detection:
                x = detection["x"]
                y = detection["y"]
                w = detection["w"]
                h = detection["h"]
                cx = detection["center_x"]
                cy = detection["center_y"]

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
                    f"CMD: {command}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )
            else:
                cv2.putText(
                    debug_frame,
                    "NO OBJECT - CMD: CAM:STOP",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            cv2.imshow("Apex Rover Camera", debug_frame)
            cv2.imshow("Detection Mask", mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            time.sleep(CAPTURE_DELAY)

    # ------------------------------
    # Shutdown
    # ------------------------------

    def shutdown(self):
        print("[SYSTEM] Shutting down safely")

        try:
            self.uno.send("CAM:STOP")
            self.mega.send("STOP")
        except:
            pass

        if self.camera:
            self.camera.stop()

        self.uno.close()
        self.mega.close()

        cv2.destroyAllWindows()


# ==================================================
# Main
# ==================================================

def main():
    brain = ApexVisionBrain()

    try:
        brain.connect_controllers()
        brain.start_camera()
        brain.run()

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Keyboard interrupt")

    finally:
        brain.shutdown()


if __name__ == "__main__":
    main()