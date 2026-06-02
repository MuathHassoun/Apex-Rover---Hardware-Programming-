import cv2
import time
from flask import Flask, Response, jsonify

# ==================================================
# Apex Rover Dual Camera Server
#
# Raspberry Pi responsibility:
#   1) Front camera stream
#   2) Arm camera stream
#
# Raspberry Pi DOES NOT control robot movement.
#
# Mobile App URLs:
#   http://192.168.4.2:5000/front_camera
#   http://192.168.4.2:5000/arm_camera
#   http://192.168.4.2:5000/front_snapshot
#   http://192.168.4.2:5000/arm_snapshot
#   http://192.168.4.2:5000/status
# ==================================================

FRONT_CAMERA_DEVICE = "/dev/v4l/by-id/usb-GENERAL_2K_HD_Camera-video-index0"
ARM_CAMERA_DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_A59D7FAF-video-index0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_FPS = 20
JPEG_QUALITY = 70

app = Flask(__name__)


class CameraStream:
    def __init__(self, name, device):
        self.name = name
        self.device = device
        self.cap = None

    def open(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        print(f"[INFO] Opening {self.name}: {self.device}")

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)

        if self.cap.isOpened():
            print(f"[OK] {self.name} opened")
            return True

        print(f"[ERROR] Could not open {self.name}")
        return False

    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()

    def read_frame(self):
        if not self.is_opened():
            self.open()

        if not self.is_opened():
            return None

        ret, frame = self.cap.read()

        if not ret or frame is None:
            print(f"[WARN] Failed to read from {self.name}, reopening...")
            self.open()
            return None

        return frame

    def encode_jpeg(self, frame):
        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if not ok:
            return None

        return buffer.tobytes()

    def generate_mjpeg(self):
        while True:
            frame = self.read_frame()

            if frame is None:
                time.sleep(0.1)
                continue

            jpg = self.encode_jpeg(frame)

            if jpg is None:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            )

            time.sleep(0.03)

    def snapshot(self):
        frame = self.read_frame()

        if frame is None:
            return None

        return self.encode_jpeg(frame)


front_camera = CameraStream("Front Camera", FRONT_CAMERA_DEVICE)
arm_camera = CameraStream("Arm Camera", ARM_CAMERA_DEVICE)


@app.route("/")
def home():
    return """
Apex Rover Dual Camera Server Running

Front Camera:
  /front_camera
  /front_snapshot

Arm Camera:
  /arm_camera
  /arm_snapshot

Status:
  /status
"""


@app.route("/status")
def status():
    return jsonify({
        "ok": True,
        "front_camera_device": FRONT_CAMERA_DEVICE,
        "arm_camera_device": ARM_CAMERA_DEVICE,
        "front_opened": front_camera.is_opened(),
        "arm_opened": arm_camera.is_opened(),
        "front_stream": "/front_camera",
        "arm_stream": "/arm_camera",
        "front_snapshot": "/front_snapshot",
        "arm_snapshot": "/arm_snapshot",
    })


@app.route("/front_camera")
def front_camera_stream():
    return Response(
        front_camera.generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/arm_camera")
def arm_camera_stream():
    return Response(
        arm_camera.generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/front_snapshot")
def front_snapshot():
    jpg = front_camera.snapshot()

    if jpg is None:
        return "Front camera not available", 503

    return Response(jpg, mimetype="image/jpeg")


@app.route("/arm_snapshot")
def arm_snapshot():
    jpg = arm_camera.snapshot()

    if jpg is None:
        return "Arm camera not available", 503

    return Response(jpg, mimetype="image/jpeg")


if __name__ == "__main__":
    front_camera.open()
    arm_camera.open()

    print("====================================")
    print("Apex Rover Dual Camera Server")
    print("Front stream   : http://192.168.4.2:5000/front_camera")
    print("Arm stream     : http://192.168.4.2:5000/arm_camera")
    print("Front snapshot : http://192.168.4.2:5000/front_snapshot")
    print("Arm snapshot   : http://192.168.4.2:5000/arm_snapshot")
    print("Status         : http://192.168.4.2:5000/status")
    print("====================================")

    app.run(host="0.0.0.0", port=5000, threaded=True)
