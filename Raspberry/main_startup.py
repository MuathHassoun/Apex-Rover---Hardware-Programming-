#!/usr/bin/env python3
"""
main_startup.py - Apex Rover Raspberry Mode Manager

This manager switches Raspberry Pi between:

MANUAL:
  - Runs Manual/apex_rover_manual_servers_startup.py
  - That manual startup file starts:
      1) Manual/camera_admin_server.py
      2) Manual/sensor_bridge.py
  - Manual camera mode opens only one camera at a time:
      Front camera for Basic / Rear Jack / Front Jack
      Arm camera for Arm mode

AUTO:
  - Stops Manual/apex_rover_manual_servers_startup.py
  - Starts Manual/sensor_bridge.py only
  - Starts Auto/front_camera_server.py
  - Starts Auto/auto_stair_climb.py

Important:
  ESP32 stays the main command gateway.
  Raspberry sends commands to ESP32 in Auto mode.
  Mobile sends commands to ESP32 in Manual mode.
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

# IMPORTANT:
# Your manual startup file name is:
#   apex_rover_manual_servers_startup.py
MANUAL_STARTUP_FILE = MANUAL_DIR / "apex_rover_manual_servers_startup.py"
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
# APP STATE
# ============================================================

app = Flask(__name__)

current_mode = "MANUAL"

manual_startup_process = None
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
        print(f"[ERROR] {name} file not found: {file_path}", flush=True)
        return None

    log_path = LOG_DIR / log_name
    log_file = open(log_path, "a", buffering=1)

    print(f"[START] {name}", flush=True)
    print(f"[FILE]  {file_path}", flush=True)
    print(f"[LOG]   {log_path}", flush=True)

    return subprocess.Popen(
        [sys.executable, "-u", str(file_path)],
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

    print(f"[STOP] {name}", flush=True)

    try:
        process.terminate()

        end_time = time.time() + timeout

        while time.time() < end_time:
            if process.poll() is not None:
                print(f"[OK] {name} stopped", flush=True)
                return None

            time.sleep(0.1)

        print(f"[WARN] {name} did not stop, killing...", flush=True)
        process.kill()
        return None

    except Exception as e:
        print(f"[ERROR] stopping {name}: {e}", flush=True)
        return None


# ============================================================
# ESP32 COMMANDS
# ============================================================

def send_esp32_command(command):
    if websocket is None:
        print("[WARN] websocket-client not installed", flush=True)
        return False

    try:
        ws = websocket.create_connection(ESP32_WS_URL, timeout=2)
        ws.send(command)
        ws.close()

        print(f"[ESP32 CMD] {command}", flush=True)
        time.sleep(0.05)
        return True

    except Exception as e:
        print(f"[ESP32 ERROR] {command}: {e}", flush=True)
        return False


def safe_stop_robot():
    commands = [
        "STOP",
        "JACK:REAR:STOP",
        "JACK:FRONT:STOP",
        "ARM:STOP",
        "CAM:STOP",
    ]

    for cmd in commands:
        send_esp32_command(cmd)


# ============================================================
# SERVICE STARTERS
# ============================================================

def start_manual_startup():
    """
    Manual mode uses:
      Manual/apex_rover_manual_servers_startup.py

    That file should start:
      - camera_admin_server.py on port 5000
      - sensor_bridge.py
    """
    global manual_startup_process

    if not is_running(manual_startup_process):
        manual_startup_process = start_process(
            name="Manual Services Startup Manager",
            file_path=MANUAL_STARTUP_FILE,
            log_name="manual_startup.log",
        )

        time.sleep(2)


def start_manual_sensor_bridge_only():
    """
    Auto mode still needs sensor_bridge, but not the manual camera admin.
    So in Auto we run sensor_bridge.py alone.
    """
    global manual_sensor_process

    if not is_running(manual_sensor_process):
        manual_sensor_process = start_process(
            name="Sensor Bridge Only",
            file_path=MANUAL_SENSOR_BRIDGE_FILE,
            log_name="sensor_bridge.log",
        )

        time.sleep(1)


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
    global manual_startup_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    print("==========================================", flush=True)
    print("SWITCH TO MANUAL MODE", flush=True)
    print("==========================================", flush=True)

    with process_lock:
        auto_brain_process = stop_process(
            "Auto Stair Climb Brain",
            auto_brain_process,
        )

        safe_stop_robot()

        # Stop Auto front camera because Manual camera admin uses port 5000.
        auto_front_camera_process = stop_process(
            "Auto Front Camera Server",
            auto_front_camera_process,
        )

        # Stop sensor_bridge-only if it was started for Auto.
        # Manual startup will start its own sensor_bridge.py.
        manual_sensor_process = stop_process(
            "Sensor Bridge Only",
            manual_sensor_process,
        )

        start_manual_startup()

        send_esp32_command("SYS:MODE:MANUAL")

        current_mode = "MANUAL"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Manual mode active: camera admin + sensors.",
    }


def switch_to_auto():
    global current_mode
    global manual_startup_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    print("==========================================", flush=True)
    print("SWITCH TO AUTO MODE", flush=True)
    print("==========================================", flush=True)

    with process_lock:
        safe_stop_robot()

        # Stop Manual startup.
        # This stops camera_admin_server.py and its sensor_bridge.py child.
        manual_startup_process = stop_process(
            "Manual Services Startup Manager",
            manual_startup_process,
        )

        # Give port 5000 and Mega serial time to release.
        time.sleep(2)

        # Auto needs sensor data, but not manual cameras.
        start_manual_sensor_bridge_only()

        # Auto uses front camera only.
        start_auto_front_camera_server()

        send_esp32_command("SYS:MODE:AUTO")
        send_esp32_command("SPEED:55")

        start_auto_brain()

        current_mode = "AUTO"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto mode active: front camera + sensor bridge + auto brain.",
    }


def stop_auto_and_return_manual():
    global current_mode
    global auto_brain_process
    global auto_front_camera_process
    global manual_sensor_process

    print("==========================================", flush=True)
    print("STOP AUTO AND RETURN TO MANUAL", flush=True)
    print("==========================================", flush=True)

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

        manual_sensor_process = stop_process(
            "Sensor Bridge Only",
            manual_sensor_process,
        )

        start_manual_startup()

        send_esp32_command("SYS:MODE:MANUAL")

        current_mode = "MANUAL"

    return {
        "ok": True,
        "mode": current_mode,
        "message": "Auto stopped. Manual camera admin restored.",
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
    manual_running = is_running(manual_startup_process)
    sensor_bridge_running = manual_running or is_running(manual_sensor_process)

    return jsonify({
        "ok": True,
        "mode": current_mode,

        # Keep these names because the mobile app already expects them.
        # In the new manual system this means:
        # manual camera admin service is running, not two cameras opened together.
        "manual_dual_camera_running": manual_running,
        "sensor_bridge_running": sensor_bridge_running,
        "auto_front_camera_running": is_running(auto_front_camera_process),
        "auto_brain_running": is_running(auto_brain_process),

        # Extra detailed status
        "manual_camera_admin_running": manual_running,
        "manual_startup_running": manual_running,
        "sensor_bridge_only_running": is_running(manual_sensor_process),

        "manual_startup_file": str(MANUAL_STARTUP_FILE),

        "logs": {
            "manual_startup": str(LOG_DIR / "manual_startup.log"),
            "sensor_bridge_only": str(LOG_DIR / "sensor_bridge.log"),
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
        "mode": current_mode,
        "message": "Emergency STOP sent.",
    })


# ============================================================
# MONITOR LOOP
# ============================================================

def monitor_loop():
    global manual_startup_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    while True:
        time.sleep(4)

        with process_lock:
            if current_mode == "MANUAL":
                if not is_running(manual_startup_process):
                    print("[MONITOR] Manual startup is down, restarting...", flush=True)
                    start_manual_startup()

                # In Manual, do not run sensor_bridge_only.
                # Manual startup runs sensor_bridge.py.
                if is_running(manual_sensor_process):
                    manual_sensor_process = stop_process(
                        "Sensor Bridge Only",
                        manual_sensor_process,
                    )

            elif current_mode == "AUTO":
                # In Auto, manual startup must be off to avoid camera/port conflicts.
                if is_running(manual_startup_process):
                    manual_startup_process = stop_process(
                        "Manual Services Startup Manager",
                        manual_startup_process,
                    )

                if not is_running(manual_sensor_process):
                    print("[MONITOR] Sensor bridge only is down, restarting...", flush=True)
                    start_manual_sensor_bridge_only()

                if not is_running(auto_front_camera_process):
                    print("[MONITOR] Auto front camera is down, restarting...", flush=True)
                    start_auto_front_camera_server()

                if not is_running(auto_brain_process):
                    print("[MONITOR] Auto brain is not running. Sending safety stop.", flush=True)
                    safe_stop_robot()


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown_handler(sig, frame):
    global manual_startup_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    print("\n[SHUTDOWN] Stopping all Raspberry services...", flush=True)

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

        manual_sensor_process = stop_process(
            "Sensor Bridge Only",
            manual_sensor_process,
        )

        manual_startup_process = stop_process(
            "Manual Services Startup Manager",
            manual_startup_process,
        )

    sys.exit(0)


# ============================================================
# MAIN
# ============================================================

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("==========================================", flush=True)
    print("Apex Rover Raspberry Mode Manager", flush=True)
    print("Default mode: MANUAL", flush=True)
    print("Manual: Manual/apex_rover_manual_servers_startup.py", flush=True)
    print("Auto  : Sensor bridge only + front camera + auto brain", flush=True)
    print("API port: 5050", flush=True)
    print("==========================================", flush=True)

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
