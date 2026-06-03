#!/usr/bin/env python3
"""
front_camera_server.py - Apex Rover AUTO camera/status server

In AUTO mode this replaces manual camera_admin_server.py.
It still opens only ONE camera at a time to reduce Raspberry Pi load.

Endpoints:
  /front_snapshot       front robot camera snapshot
  /arm_snapshot         arm camera snapshot
  /front_camera         front robot camera MJPEG stream
  /arm_camera           arm camera MJPEG stream
  /camera/use/front
  /camera/use/arm
  /camera/release
  /auto_status          full auto tracking JSON
  /status               camera server status

Access example while Raspberry is connected to ESP32 AP:
  http://192.168.4.X:5000/auto_status
"""

import sys
import time
import threading
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify

BASE_DIR = Path(__file__).resolve().parent
PARENT_DIR = BASE_DIR.parent
sys.path.insert(0, str(PARENT_DIR))
sys.path.insert(0, str(BASE_DIR))

try:
    from config import (
        CAMERA_SERVER_HOST,
        CAMERA_SERVER_PORT,
        FRONT_CAMERA_DEVICE,
        ARM_CAMERA_DEVICE,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        JPEG_QUALITY,
    )
except Exception:
    from auto_config import (
        CAMERA_SERVER_HOST,
        CAMERA_SERVER_PORT,
        FRONT_CAMERA_DEVICE,
        ARM_CAMERA_DEVICE,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        JPEG_QUALITY,
    )

from auto_status import read_status_file

FRAME_FPS = 20
REOPEN_GUARD_SEC = 0.7

app = Flask(__name__)


class AutoCameraAdmin:
    def __init__(self):
        self.lock = threading.RLock()
        self.active_name = None
        self.active_device = None
        self.cap = None
        self.last_switch_time = 0.0
        self.last_error = ""
        self.devices = {
            "front": FRONT_CAMERA_DEVICE,
            "arm": ARM_CAMERA_DEVICE,
        }

    def release_current_locked(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.active_name is not None:
            print(f"[AUTO CAM] Released {self.active_name} camera", flush=True)
        self.cap = None
        self.active_name = None
        self.active_device = None

    def switch_to_locked(self, name):
        name = str(name).lower().strip()
        if name not in self.devices:
            self.last_error = f"Unknown camera: {name}"
            return False

        if self.active_name == name and self.cap is not None and self.cap.isOpened():
            return True

        now = time.time()
        wait_left = REOPEN_GUARD_SEC - (now - self.last_switch_time)
        if wait_left > 0:
            time.sleep(wait_left)
        self.last_switch_time = time.time()

        self.release_current_locked()

        device = self.devices[name]
        print(f"[AUTO CAM] Opening {name}: {device}", flush=True)
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)

        if not cap.isOpened():
            self.last_error = f"Could not open {name} camera: {device}"
            print(f"[AUTO CAM ERROR] {self.last_error}", flush=True)
            try:
                cap.release()
            except Exception:
                pass
            return False

        self.cap = cap
        self.active_name = name
        self.active_device = device
        self.last_error = ""
        print(f"[AUTO CAM OK] Active camera: {name}", flush=True)
        return True

    def use_camera(self, name):
        with self.lock:
            return self.switch_to_locked(name)

    def read_frame(self, name):
        with self.lock:
            if not self.switch_to_locked(name):
                return None
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return frame

            old_name = self.active_name
            self.release_current_locked()
            if old_name and self.switch_to_locked(old_name):
                ok, frame = self.cap.read()
                if ok and frame is not None:
                    return frame
            self.last_error = f"Read failed from {name}"
            return None

    def jpeg_snapshot(self, name):
        frame = self.read_frame(name)
        if frame is None:
            return None
        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            self.last_error = "JPEG encode failed"
            return None
        return buffer.tobytes()

    def mjpeg_stream(self, name):
        while True:
            jpg = self.jpeg_snapshot(name)
            if jpg is None:
                time.sleep(0.12)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            time.sleep(0.04)

    def status_dict(self):
        with self.lock:
            opened = self.cap is not None and self.cap.isOpened()
            return {
                "ok": True,
                "service": "Apex Rover AUTO Camera Server",
                "mode": "AUTO",
                "active_camera": self.active_name or "none",
                "active_device": self.active_device or "",
                "opened": opened,
                "front_camera_device": FRONT_CAMERA_DEVICE,
                "arm_camera_device": ARM_CAMERA_DEVICE,
                "front_snapshot": "/front_snapshot",
                "arm_snapshot": "/arm_snapshot",
                "front_stream": "/front_camera",
                "arm_stream": "/arm_camera",
                "auto_status": "/auto_status",
                "last_error": self.last_error,
                "important": "Only one camera is opened at a time in AUTO mode.",
            }

    def release(self):
        with self.lock:
            self.release_current_locked()


camera_admin = AutoCameraAdmin()


@app.route("/")
def home():
    return jsonify({
        "service": "Apex Rover AUTO Camera + Status Server",
        "status": "/status",
        "auto_status": "/auto_status",
        "front_snapshot": "/front_snapshot",
        "arm_snapshot": "/arm_snapshot",
        "front_camera": "/front_camera",
        "arm_camera": "/arm_camera",
    })


@app.route("/status")
def status():
    return jsonify(camera_admin.status_dict())


@app.route("/auto_status")
def auto_status():
    return jsonify(read_status_file())


@app.route("/camera/use/front", methods=["GET", "POST"])
def use_front():
    ok = camera_admin.use_camera("front")
    data = camera_admin.status_dict()
    data["ok"] = ok
    return jsonify(data), 200 if ok else 503


@app.route("/camera/use/arm", methods=["GET", "POST"])
def use_arm():
    ok = camera_admin.use_camera("arm")
    data = camera_admin.status_dict()
    data["ok"] = ok
    return jsonify(data), 200 if ok else 503


@app.route("/camera/release", methods=["GET", "POST"])
def release_camera():
    camera_admin.release()
    return jsonify({"ok": True, "message": "Camera released"})


@app.route("/snapshot")
def snapshot():
    active = camera_admin.active_name or "front"
    jpg = camera_admin.jpeg_snapshot(active)
    if jpg is None:
        return f"{active} camera not available", 503
    return Response(jpg, mimetype="image/jpeg")


@app.route("/front_snapshot")
def front_snapshot():
    jpg = camera_admin.jpeg_snapshot("front")
    if jpg is None:
        return "Front camera not available", 503
    return Response(jpg, mimetype="image/jpeg")


@app.route("/arm_snapshot")
def arm_snapshot():
    jpg = camera_admin.jpeg_snapshot("arm")
    if jpg is None:
        return "Arm camera not available", 503
    return Response(jpg, mimetype="image/jpeg")


@app.route("/front_camera")
def front_camera_stream():
    return Response(camera_admin.mjpeg_stream("front"), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/arm_camera")
def arm_camera_stream():
    return Response(camera_admin.mjpeg_stream("arm"), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    print("==========================================", flush=True)
    print("Apex Rover AUTO Camera + Status Server", flush=True)
    print("AUTO mode: ONE camera active at a time", flush=True)
    print(f"Front camera: {FRONT_CAMERA_DEVICE}", flush=True)
    print(f"Arm camera  : {ARM_CAMERA_DEVICE}", flush=True)
    print(f"Server      : http://0.0.0.0:{CAMERA_SERVER_PORT}", flush=True)
    print("Status      : /auto_status", flush=True)
    print("==========================================", flush=True)

    camera_admin.use_camera("front")
    try:
        app.run(host=CAMERA_SERVER_HOST, port=CAMERA_SERVER_PORT, threaded=True)
    finally:
        camera_admin.release()


if __name__ == "__main__":
    main()
