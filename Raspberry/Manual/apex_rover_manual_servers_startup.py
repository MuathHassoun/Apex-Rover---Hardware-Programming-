#!/usr/bin/env python3
"""
Apex Rover Manual Camera Startup - CLEAN VERSION

This file starts ONLY the manual camera admin server.

IMPORTANT CHANGE:
  The old sensor_bridge.py is no longer started from Raspberry.
  Mega sensor readings must go directly:

    Mega Serial1 TX -> ESP32 RX
    ESP32 WebSocket -> Mobile App

This avoids Raspberry USB serial ground/noise problems with the audio system.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from typing import Optional

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAMERA_FILE = os.path.join(BASE_DIR, "camera_admin_server.py")

process: Optional[subprocess.Popen] = None
running = True


def log(message: str) -> None:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def is_running() -> bool:
    return process is not None and process.poll() is None


def print_output(proc: subprocess.Popen) -> None:
    if proc.stdout is None:
        return

    while running and proc.poll() is None:
        line = proc.stdout.readline()
        if line:
            print(f"[camera_admin] {line}", end="", flush=True)
        else:
            time.sleep(0.05)


def start_camera() -> None:
    global process

    if is_running():
        return

    if not os.path.exists(CAMERA_FILE):
        log(f"[ERROR] camera_admin_server.py not found: {CAMERA_FILE}")
        return

    log(f"[START] camera_admin_server.py")

    process = subprocess.Popen(
        [sys.executable, "-u", CAMERA_FILE],
        cwd=BASE_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    threading.Thread(target=print_output, args=(process,), daemon=True).start()


def stop_camera() -> None:
    global process

    if process is None:
        return

    if process.poll() is None:
        log("[STOP] camera_admin_server.py")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

    process = None


def shutdown(signum=None, frame=None) -> None:
    global running
    running = False
    stop_camera()
    sys.exit(0)


def main() -> None:
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    log("==========================================")
    log("Apex Rover Manual Camera Startup")
    log("Sensor bridge: DISABLED")
    log("Only camera_admin_server.py will run")
    log("==========================================")

    start_camera()

    while running:
        if not is_running():
            log("[MONITOR] camera_admin_server.py stopped, restarting...")
            start_camera()
        time.sleep(2)


if __name__ == "__main__":
    main()
