#!/usr/bin/env python3
"""
main_startup.py - Apex Rover Raspberry Mode Manager

This file controls which Raspberry mode is running.

Manual Mode:
  - Starts Manual/dual_camera_server.py
  - Starts Manual/sensor_bridge.py
  - Stops Auto/front_camera_server.py
  - Stops Auto/auto_stair_climb.py

Auto Mode:
  - Stops Manual/dual_camera_server.py
  - Starts Auto/front_camera_server.py
  - Keeps Manual/sensor_bridge.py running
  - Starts Auto/auto_stair_climb.py

Important:
  ESP32 remains the command gateway.
  Raspberry does not send commands directly to Mega or UNO.
"""

import sys
import time
import signal
import subprocess
import threading
from pathlib import Path

from flask import Flask, jsonify

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

MANUAL_DUAL_CAMERA_FILE = MANUAL_DIR / "dual_camera_server.py"
MANUAL_SENSOR_BRIDGE_FILE = MANUAL_DIR / "sensor_bridge.py"

AUTO_FRONT_CAMERA_FILE = AUTO_DIR / "front_camera_server.py"
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
# FLASK + STATE
# ============================================================

app = Flask(__name__)

current_mode = "MANUAL"

manual_camera_process = None
manual_sensor_process = None

auto_front_camera_process = None
auto_brain_process = None

process_lock = threading.Lock()


# ============================================================
# PROCESS HELPERS
# ============================================================

def is_running(process):
    return process is not None and process.poll() is None


def start_process(name, file_path, log_name):
    if not file_path.exists():
        print(f"[ERROR] {name} file not found: {file_path}")
        return None

    log_path = LOG_DIR / log_name
    log_file = open(log_path, "a", buffering=1)

    print(f"[START] {name}")
    print(f"[FILE] {file_path}")
    print(f"[LOG]  {log_path}")

    return subprocess.Popen(
        [sys.executable, str(file_path)],
        cwd=str(file_path.parent),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_process(name, process, timeout=5):
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
# ESP32 SAFETY COMMANDS
# ============================================================

def send_esp32_command(command):
    if websocket is None:
        print("[WARN] websocket-client not installed")
        return False

    try:
        ws = websocket.create_connection(ESP32_WS_URL, timeout=2)
        ws.send(command)
        ws.close()

        print(f"[ESP32 CMD] {command}")
        time.sleep(0.05)

        return True

    except Exception as e:
        print(f"[ESP32 ERROR] {command}: {e}")
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
# SERVICE STARTERS
# ============================================================

def ensure_sensor_bridge_running():
    global manual_sensor_process

    if not is_running(manual_sensor_process):
        manual_sensor_process = start_process(
            name="Sensor Bridge",
            file_path=MANUAL_SENSOR_BRIDGE_FILE,
            log_name="sensor_bridge.log",
        )
        time.sleep(1)


def start_manual_camera_server():
    global manual_camera_process

    if not is_running(manual_camera_process):
        manual_camera_process = start_process(
            name="Manual Dual Camera Server",
            file_path=MANUAL_DUAL_CAMERA_FILE,
            log_name="manual_dual_camera.log",
        )
        time.sleep(2)


def start_auto_front_camera_server():
    global auto_front_camera_process

    if not is_running(auto_front_camera_process):
        auto_front_camera_process = start_process(
            name="Auto Front Camera Server",
            file_path=AUTO_FRONT_CAMERA_FILE,
            log_name="auto_front_camera.log",
        )
        time.sleep(2)


def start_auto_brain():
    global auto_brain_process

    if not is_running(auto_brain_process):
        auto_brain_process = start_process(
            name="Auto Stair Climb Brain",
            file_path=AUTO_BRAIN_FILE,
            log_name="auto_stair_climb.log",
        )
        time.sleep(1)


# ============================================================
# MODE SWITCHING
# ============================================================

def switch_to_manual():
    global current_mode
    global manual_camera_process
    global auto_front_camera_process
    global auto_brain_process

    print("==========================================")
    print("SWITCH TO MANUAL MODE")
    print("==========================================")

    with process_lock:
        # Stop auto movement first.
        auto_brain_process = stop_process(
            "Auto Stair Climb Brain",
            auto_brain_process,
        )

        safe_stop_robot()

        # Stop auto front camera, because manual uses dual camera on same port.
        auto_front_camera_process = stop_process(
            "Auto Front Camera Server",
            auto_front_camera_process,
        )

        # Start manual services.
        ensure_sensor_bridge_running()
        start_manual_camera_server()

        send_esp32_command("SYS:MODE:MANUAL")

        current_mode = "MANUAL"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Manual mode active: dual cameras + sensors running.",
    }


def switch_to_auto():
    global current_mode
    global manual_camera_process
    global auto_front_camera_process
    global auto_brain_process

    print("==========================================")
    print("SWITCH TO AUTO MODE")
    print("==========================================")

    with process_lock:
        # Safety first.
        safe_stop_robot()

        # Keep sensor bridge running because Auto reads MPU from:
        # /tmp/apex_last_sensor.json
        ensure_sensor_bridge_running()

        # Stop manual dual camera to reduce Raspberry load.
        # Auto needs front camera only.
        manual_camera_process = stop_process(
            "Manual Dual Camera Server",
            manual_camera_process,
        )

        # Start front camera only.
        start_auto_front_camera_server()

        send_esp32_command("SYS:MODE:AUTO")
        send_esp32_command("SPEED:55")

        # Start auto brain.
        start_auto_brain()

        current_mode = "AUTO"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto mode active: front camera + MPU + auto brain.",
    }


def stop_auto_and_return_manual():
    global current_mode
    global auto_brain_process
    global auto_front_camera_process

    print("==========================================")
    print("STOP AUTO AND RETURN TO MANUAL")
    print("==========================================")

    with process_lock:
        auto_brain_process = stop_process(
            "Auto Stair Climb Brain",
            auto_brain_process,
        )

        safe_stop_robot()

        auto_front_camera_process = stop_process(
            "Auto Front Camera Server",
            auto_front_camera_process,
        )

        ensure_sensor_bridge_running()
        start_manual_camera_server()

        send_esp32_command("SYS:MODE:MANUAL")

        current_mode = "MANUAL"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto stopped. Manual mode restored.",
    }


# ============================================================
# API ROUTES
# ============================================================

@app.route("/")
def home():
    return jsonify({
        "service": "Apex Rover Raspberry Mode Manager",
        "mode": current_mode,
        "endpoints": [
            "/status",
            "/mode/manual",
            "/mode/auto",
            "/mode/stop",
            "/robot/stop",
        ],
    })


@app.route("/status")
def status():
    return jsonify({
        "ok": True,
        "mode": current_mode,
        "manual_dual_camera_running": is_running(manual_camera_process),
        "sensor_bridge_running": is_running(manual_sensor_process),
        "auto_front_camera_running": is_running(auto_front_camera_process),
        "auto_brain_running": is_running(auto_brain_process),
        "logs": {
            "manual_dual_camera": str(LOG_DIR / "manual_dual_camera.log"),
            "sensor_bridge": str(LOG_DIR / "sensor_bridge.log"),
            "auto_front_camera": str(LOG_DIR / "auto_front_camera.log"),
            "auto_stair_climb": str(LOG_DIR / "auto_stair_climb.log"),
        },
    })


@app.route("/mode/manual", methods=["GET", "POST"])
def api_manual():
    return jsonify(switch_to_manual())


@app.route("/mode/auto", methods=["GET", "POST"])
def api_auto():
    return jsonify(switch_to_auto())


@app.route("/mode/stop", methods=["GET", "POST"])
def api_stop():
    return jsonify(stop_auto_and_return_manual())


@app.route("/robot/stop", methods=["GET", "POST"])
def api_robot_stop():
    safe_stop_robot()

    return jsonify({
        "ok": True,
        "message": "Emergency STOP sent.",
    })


# ============================================================
# MONITOR LOOP
# ============================================================

def monitor_loop():
    while True:
        time.sleep(4)

        with process_lock:
            # Sensor bridge must always be running in both modes.
            ensure_sensor_bridge_running()

            if current_mode == "MANUAL":
                # Manual needs dual cameras.
                if not is_running(manual_camera_process):
                    print("[MONITOR] Manual camera server is down, restarting...")
                    start_manual_camera_server()

            elif current_mode == "AUTO":
                # Auto needs front camera only.
                if not is_running(auto_front_camera_process):
                    print("[MONITOR] Auto front camera is down, restarting...")
                    start_auto_front_camera_server()

                if not is_running(auto_brain_process):
                    print("[MONITOR] Auto brain is not running.")
                    # Do NOT auto-restart auto brain after failure.
                    # Better stay safe and return to manual.
                    safe_stop_robot()


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown_handler(sig, frame):
    global manual_camera_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    print("\n[SHUTDOWN] Stopping all Raspberry services...")

    safe_stop_robot()

    with process_lock:
        auto_brain_process = stop_process(
            "Auto Stair Climb Brain",
            auto_brain_process,
        )

        auto_front_camera_process = stop_process(
            "Auto Front Camera Server",
            auto_front_camera_process,
        )

        manual_camera_process = stop_process(
            "Manual Dual Camera Server",
            manual_camera_process,
        )

        manual_sensor_process = stop_process(
            "Sensor Bridge",
            manual_sensor_process,
        )

    sys.exit(0)


# ============================================================
# MAIN
# ============================================================

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("==========================================")
    print("Apex Rover Raspberry Mode Manager")
    print("Default mode: MANUAL")
    print("Manual: dual cameras + sensors")
    print("Auto  : front camera + MPU + auto brain")
    print("API port: 5050")
    print("==========================================")

    # Default boot mode = Manual.
    switch_to_manual()

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