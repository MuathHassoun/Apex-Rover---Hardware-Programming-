#!/usr/bin/env python3
"""
main_startup.py - Apex Rover Raspberry Mode Manager

This file is the main Raspberry controller.

It does NOT drive motors directly.

Manual mode:
  - Starts two-camera server.
  - Starts sensor bridge.
  - Mobile App controls robot through ESP32.

Auto mode:
  - Keeps camera/sensor services running.
  - Starts Auto stair climb brain.
  - Auto brain sends commands to ESP32.
  - ESP32 routes commands to Mega / UNO.

Important:
  ESP32 remains the main command gateway.
"""

import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path

from flask import Flask, jsonify, request

try:
    import websocket
except Exception:
    websocket = None


# ============================================================
# PATHS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

MANUAL_DIR = BASE_DIR / "Manual"
AUTO_DIR = BASE_DIR / "Auto"

MANUAL_CAMERA_FILE = MANUAL_DIR / "dual_camera_server.py"
MANUAL_SENSOR_FILE = MANUAL_DIR / "sensor_bridge.py"

AUTO_BRAIN_FILE = AUTO_DIR / "auto_stair_climb.py"

LOG_DIR = Path("/tmp/apex_rover_logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# NETWORK
# ============================================================

ESP32_WS_URL = "ws://192.168.4.1:81"

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5050


# ============================================================
# GLOBAL STATE
# ============================================================

app = Flask(__name__)

current_mode = "MANUAL"

manual_camera_process = None
manual_sensor_process = None
auto_process = None

process_lock = threading.Lock()


# ============================================================
# PROCESS HELPERS
# ============================================================

def is_process_running(process):
    return process is not None and process.poll() is None


def start_process(name, file_path, log_name):
    if not file_path.exists():
        print(f"[ERROR] {name} file not found: {file_path}")
        return None

    log_path = LOG_DIR / log_name

    log_file = open(log_path, "a", buffering=1)

    print(f"[START] {name}")
    print(f"[LOG] {log_path}")

    process = subprocess.Popen(
        [sys.executable, str(file_path)],
        cwd=str(file_path.parent),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
    )

    return process


def stop_process(name, process, timeout=4):
    if process is None:
        return None

    if process.poll() is not None:
        return None

    print(f"[STOP] {name}")

    try:
        process.terminate()

        end_time = time.time() + timeout

        while time.time() < end_time:
            if process.poll() is not None:
                print(f"[OK] {name} stopped")
                return None

            time.sleep(0.1)

        print(f"[WARN] {name} did not stop, killing...")
        process.kill()
        return None

    except Exception as e:
        print(f"[ERROR] stopping {name}: {e}")
        return None


# ============================================================
# ESP32 COMMANDS
# ============================================================

def send_esp32_command(command, wait_after=0.05):
    if websocket is None:
        print("[WARN] websocket-client not installed")
        return False

    try:
        ws = websocket.create_connection(ESP32_WS_URL, timeout=2)
        ws.send(command)
        ws.close()

        print(f"[ESP32 CMD] {command}")

        if wait_after > 0:
            time.sleep(wait_after)

        return True

    except Exception as e:
        print(f"[ESP32 CMD ERROR] {command}: {e}")
        return False


def safe_stop_robot():
    commands = [
        "STOP",
        "JACK:REAR:STOP",
        "JACK:FRONT:STOP",
        "CAM:STOP",
    ]

    for cmd in commands:
        send_esp32_command(cmd)


# ============================================================
# MODE MANAGEMENT
# ============================================================

def ensure_manual_services_running():
    global manual_camera_process
    global manual_sensor_process

    with process_lock:
        if not is_process_running(manual_camera_process):
            manual_camera_process = start_process(
                name="Manual Camera Server",
                file_path=MANUAL_CAMERA_FILE,
                log_name="manual_camera.log",
            )
            time.sleep(1)

        if not is_process_running(manual_sensor_process):
            manual_sensor_process = start_process(
                name="Manual Sensor Bridge",
                file_path=MANUAL_SENSOR_FILE,
                log_name="manual_sensor_bridge.log",
            )
            time.sleep(1)


def start_manual_mode():
    global current_mode
    global auto_process

    print("===================================")
    print("Switching to MANUAL mode")
    print("===================================")

    with process_lock:
        auto_process = stop_process("Auto Stair Climb", auto_process)

    safe_stop_robot()

    send_esp32_command("SYS:MODE:MANUAL")

    ensure_manual_services_running()

    current_mode = "MANUAL"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Manual mode active. Cameras and sensors are running.",
    }


def start_auto_mode():
    global current_mode
    global auto_process

    print("===================================")
    print("Switching to AUTO mode")
    print("===================================")

    ensure_manual_services_running()

    safe_stop_robot()

    send_esp32_command("SYS:MODE:AUTO")
    send_esp32_command("SPEED:45")

    with process_lock:
        if is_process_running(auto_process):
            return {
                "ok": True,
                "mode": "AUTO",
                "message": "Auto mode already running.",
            }

        auto_process = start_process(
            name="Auto Stair Climb",
            file_path=AUTO_BRAIN_FILE,
            log_name="auto_stair_climb.log",
        )

    current_mode = "AUTO"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto stair climb started.",
    }


def stop_auto_only():
    global current_mode
    global auto_process

    print("===================================")
    print("Stopping AUTO only")
    print("===================================")

    with process_lock:
        auto_process = stop_process("Auto Stair Climb", auto_process)

    safe_stop_robot()

    current_mode = "MANUAL"

    send_esp32_command("SYS:MODE:MANUAL")

    ensure_manual_services_running()

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto stopped. Manual services still running.",
    }


# ============================================================
# API ROUTES
# ============================================================

@app.route("/")
def home():
    return jsonify({
        "service": "Apex Rover Raspberry Mode Manager",
        "mode": current_mode,
        "manual_services": {
            "camera": is_process_running(manual_camera_process),
            "sensor_bridge": is_process_running(manual_sensor_process),
        },
        "auto_running": is_process_running(auto_process),
    })


@app.route("/status")
def status():
    return jsonify({
        "ok": True,
        "mode": current_mode,
        "manual_camera_running": is_process_running(manual_camera_process),
        "manual_sensor_running": is_process_running(manual_sensor_process),
        "auto_running": is_process_running(auto_process),
        "logs": {
            "manual_camera": str(LOG_DIR / "manual_camera.log"),
            "manual_sensor_bridge": str(LOG_DIR / "manual_sensor_bridge.log"),
            "auto_stair_climb": str(LOG_DIR / "auto_stair_climb.log"),
        },
    })


@app.route("/mode/manual", methods=["GET", "POST"])
def api_manual():
    result = start_manual_mode()
    return jsonify(result)


@app.route("/mode/auto", methods=["GET", "POST"])
def api_auto():
    result = start_auto_mode()
    return jsonify(result)


@app.route("/mode/stop", methods=["GET", "POST"])
def api_stop():
    result = stop_auto_only()
    return jsonify(result)


@app.route("/robot/stop", methods=["GET", "POST"])
def api_robot_stop():
    safe_stop_robot()

    return jsonify({
        "ok": True,
        "message": "STOP sent to robot.",
    })


# ============================================================
# STARTUP / SHUTDOWN
# ============================================================

def shutdown_handler(sig, frame):
    global manual_camera_process
    global manual_sensor_process
    global auto_process

    print("\n[SHUTDOWN] Stopping all processes...")

    safe_stop_robot()

    with process_lock:
        auto_process = stop_process("Auto Stair Climb", auto_process)
        manual_sensor_process = stop_process("Manual Sensor Bridge", manual_sensor_process)
        manual_camera_process = stop_process("Manual Camera Server", manual_camera_process)

    sys.exit(0)


def monitor_loop():
    while True:
        time.sleep(3)

        if current_mode in ["MANUAL", "AUTO"]:
            ensure_manual_services_running()


def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("===================================")
    print("Apex Rover Raspberry Mode Manager")
    print("Default mode: MANUAL")
    print("===================================")

    ensure_manual_services_running()

    threading.Thread(
        target=monitor_loop,
        daemon=True,
    ).start()

    app.run(
        host=SERVER_HOST,
        port=SERVER_PORT,
        threaded=True,
    )


if __name__ == "__main__":
    main()
