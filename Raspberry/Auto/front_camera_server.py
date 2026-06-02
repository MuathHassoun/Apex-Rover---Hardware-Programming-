#!/usr/bin/env python3
"""
front_camera_server.py - Apex Rover Auto Front Camera Server

This server is used in AUTO mode only.

It opens ONLY the front camera.
It does NOT open the arm camera.

Endpoint:
  /front_snapshot

Auto brain uses:
  http://127.0.0.1:5000/front_snapshot

Important:
  Manual dual_camera_server.py must be stopped before this starts,
  because both use port 5000.
"""

import sys
import time
import threading
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify


# ============================================================
# PATH FIX
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
PARENT_DIR = BASE_DIR.parent
sys.path.insert(0, str(PARENT_DIR))


# ============================================================
# CONFIG
# ============================================================

try:
    from config import (
        CAMERA_SERVER_HOST,
        CAMERA_SERVER_PORT,
        FRONT_CAMERA_DEVICE,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        JPEG_QUALITY,
    )
except Exception:
    CAMERA_SERVER_HOST = "0.0.0.0"
    CAMERA_SERVER_PORT = 5000

    FRONT_CAMERA_DEVICE = "/dev/v4l/by-id/usb-GENERAL_2K_HD_Camera-video-index0"

    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    JPEG_QUALITY = 70


app = Flask(__name__)


# ============================================================
# FRONT CAMERA CLASS
# ============================================================

class FrontCamera:
    def __init__(self, device):
        self.device = device
        self.cap = None
        self.lock = threading.Lock()
        self.last_open_time = 0

    def open(self):
        with self.lock:
            now = time.time()

            # Prevent too many fast reopen attempts.
            if now - self.last_open_time < 1.0:
                return False

            self.last_open_time = now

            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass

            print(f"[CAM] Opening front camera: {self.device}")

            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, 20)

            if self.cap.isOpened():
                print("[CAM] Front camera opened successfully")
                return True

            print("[CAM ERROR] Could not open front camera")
            return False

    def read_frame(self):
        with self.lock:
            if self.cap is None or not self.cap.isOpened():
                ok = self.open()

                if not ok:
                    return None

            ok, frame = self.cap.read()

            if not ok or frame is None:
                print("[CAM WARN] Front camera read failed, reopening...")
                try:
                    self.cap.release()
                except Exception:
                    pass

                self.cap = None
                return None

            return frame

    def get_jpeg(self):
        frame = self.read_frame()

        if frame is None:
            return None

        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if not ok:
            return None

        return buffer.tobytes()

    def release(self):
        with self.lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass

            self.cap = None
            print("[CAM] Front camera released")


front_camera = FrontCamera(FRONT_CAMERA_DEVICE)


# ============================================================
# ROUTES
# ============================================================

@app.route("/")
def home():
    return jsonify({
        "service": "Apex Rover Auto Front Camera Server",
        "mode": "AUTO",
        "camera": "front_only",
        "front_snapshot": "/front_snapshot",
    })


@app.route("/status")
def status():
    opened = front_camera.cap is not None and front_camera.cap.isOpened()

    return jsonify({
        "ok": True,
        "mode": "AUTO",
        "front_camera_device": FRONT_CAMERA_DEVICE,
        "front_camera_opened": opened,
        "frame_width": FRAME_WIDTH,
        "frame_height": FRAME_HEIGHT,
        "jpeg_quality": JPEG_QUALITY,
    })


@app.route("/front_snapshot")
def front_snapshot():
    jpg = front_camera.get_jpeg()

    if jpg is None:
        return "Front camera not available", 503

    return Response(jpg, mimetype="image/jpeg")


# ============================================================
# MAIN
# ============================================================

def main():
    print("==========================================")
    print("Apex Rover Auto Front Camera Server")
    print("AUTO mode: front camera only")
    print(f"Device: {FRONT_CAMERA_DEVICE}")
    print(f"URL: http://0.0.0.0:{CAMERA_SERVER_PORT}/front_snapshot")
    print("==========================================")

    front_camera.open()

    try:
        app.run(
            host=CAMERA_SERVER_HOST,
            port=CAMERA_SERVER_PORT,
            threaded=True,
        )
    finally:
        front_camera.release()


if __name__ == "__main__":
    main()