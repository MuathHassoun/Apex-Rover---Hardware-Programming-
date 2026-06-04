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

New:
  - Adds /auto_status for the mobile Auto Status Track screen.
  - Normalizes Auto/auto_status.py format:
      phase   -> stage
      doing   -> action
      history -> track
"""

import sys
import time
import signal
import subprocess
import threading
import json
from pathlib import Path
from datetime import datetime

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

MANUAL_STARTUP_FILE = MANUAL_DIR / "apex_rover_manual_servers_startup.py"
MANUAL_SENSOR_BRIDGE_FILE = MANUAL_DIR / "sensor_bridge.py"

AUTO_FRONT_CAMERA_FILE = AUTO_DIR / "front_camera_server.py"
AUTO_BRAIN_FILE = AUTO_DIR / "auto_stair_climb.py"

LOG_DIR = Path("/tmp/apex_rover_logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

AUTO_STATUS_FILE = Path("/tmp/apex_auto_status.json")


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
auto_status_lock = threading.Lock()


# ============================================================
# AUTO STATUS TRACK
# ============================================================

auto_status_state = {
    "ok": True,
    "mode": "MANUAL",
    "stage": "Manual Mode",
    "action": "Manual mode is active",
    "decision": "Waiting for auto mode",
    "error": "",
    "time": "",
    "track": [],
}


def now_text():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def timestamp_to_text(value):
    try:
        if isinstance(value, (int, float)):
            return datetime.fromtimestamp(value).strftime("%Y-%m-%d %H:%M:%S")

        if isinstance(value, str) and value.strip():
            return value.strip()
    except Exception:
        pass

    return ""


def push_auto_track(track_type, stage, message, important=False):
    """
    Adds one message to the auto status track.
    This is used by main_startup.py itself.
    Later auto_stair_climb.py can also write /tmp/apex_auto_status.json.
    """
    global auto_status_state

    with auto_status_lock:
        item = {
            "type": track_type,
            "stage": stage,
            "message": message,
            "time": now_text(),
            "important": important,
        }

        auto_status_state["track"].append(item)

        # Keep last 80 messages only, to avoid very large JSON.
        if len(auto_status_state["track"]) > 80:
            auto_status_state["track"] = auto_status_state["track"][-80:]

        auto_status_state["time"] = item["time"]


def set_auto_status(stage=None, action=None, decision=None, error=None, mode=None):
    global auto_status_state

    with auto_status_lock:
        if mode is not None:
            auto_status_state["mode"] = mode

        if stage is not None:
            auto_status_state["stage"] = stage

        if action is not None:
            auto_status_state["action"] = action

        if decision is not None:
            auto_status_state["decision"] = decision

        if error is not None:
            auto_status_state["error"] = error

        auto_status_state["ok"] = True
        auto_status_state["time"] = now_text()


def write_auto_status_file():
    """
    Writes current status to /tmp/apex_auto_status.json.
    Not required for the mobile page, but useful for Auto brain integration later.
    """
    try:
        with auto_status_lock:
            data = dict(auto_status_state)

        AUTO_STATUS_FILE.write_text(
            json.dumps(data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception as e:
        print(f"[AUTO_STATUS WARN] Could not write status file: {e}", flush=True)


def read_external_auto_status():
    """
    If Auto/auto_stair_climb.py writes /tmp/apex_auto_status.json,
    we prefer its detailed state. If not available, we use the internal fallback.
    """
    try:
        if not AUTO_STATUS_FILE.exists():
            return None

        text = AUTO_STATUS_FILE.read_text(encoding="utf-8").strip()

        if not text:
            return None

        data = json.loads(text)

        if isinstance(data, dict):
            return data

    except Exception as e:
        print(f"[AUTO_STATUS WARN] Could not read external status: {e}", flush=True)

    return None


def normalize_external_auto_status(external):
    """
    Convert Auto/auto_status.py format to mobile AutoStatusScreen format.

    Auto brain may write:
      phase, doing, decision, history

    Mobile expects:
      stage, action, decision, track
    """

    if not isinstance(external, dict):
        return external

    # Basic aliases for mobile screen.
    if not external.get("stage"):
        external["stage"] = external.get("phase", "AUTO")

    if not external.get("action"):
        external["action"] = external.get("doing", "")

    if not external.get("decision"):
        external["decision"] = external.get("last_decision", "")

    if external.get("error") is None:
        external["error"] = ""

    if not external.get("time"):
        updated_at = external.get("updated_at")
        converted = timestamp_to_text(updated_at)
        external["time"] = converted if converted else now_text()

    # Convert history -> track.
    history = external.get("history", [])
    existing_track = external.get("track", [])

    if (not existing_track) and isinstance(history, list):
        track = []

        for item in history[-100:]:
            if not isinstance(item, dict):
                track.append({
                    "type": "info",
                    "stage": str(external.get("stage", "AUTO")),
                    "message": str(item),
                    "time": "",
                    "important": False,
                })
                continue

            phase = item.get("phase") or item.get("stage") or external.get("stage", "AUTO")
            doing = item.get("doing") or item.get("action") or ""
            decision = item.get("decision") or ""
            error = item.get("error") or ""

            raw_time = item.get("time", "")
            time_text = timestamp_to_text(raw_time)

            if error:
                track_type = "error"
                message = str(error)
                important = True

            elif decision and str(decision).upper() not in ["NONE", "NULL", ""]:
                track_type = "decision"

                if doing and str(doing).strip():
                    message = f"{doing} | Decision: {decision}"
                else:
                    message = str(decision)

                important = True

            elif doing and str(doing).strip():
                track_type = "action"
                message = str(doing)
                important = False

            else:
                track_type = "info"
                message = "Auto status update"
                important = False

            track.append({
                "type": track_type,
                "stage": str(phase),
                "message": message,
                "time": time_text,
                "important": important,
            })

        external["track"] = track

    if "track" not in external:
        external["track"] = []

    return external


def auto_status_snapshot():
    external = read_external_auto_status()

    if external is not None:
        external = normalize_external_auto_status(external)

        external.setdefault("ok", True)
        external.setdefault("mode", current_mode)
        external.setdefault("stage", "AUTO")
        external.setdefault("action", "")
        external.setdefault("decision", "")
        external.setdefault("error", "")
        external.setdefault("track", [])
        external.setdefault("time", now_text())

        external["mode_manager_mode"] = current_mode
        external["auto_front_camera_running"] = is_running(auto_front_camera_process)
        external["auto_brain_running"] = is_running(auto_brain_process)
        external["sensor_bridge_running"] = (
            is_running(manual_startup_process) or
            is_running(manual_sensor_process)
        )

        return external

    with auto_status_lock:
        data = dict(auto_status_state)
        data["track"] = list(auto_status_state.get("track", []))

    data["ok"] = True
    data["mode"] = current_mode
    data["auto_front_camera_running"] = is_running(auto_front_camera_process)
    data["auto_brain_running"] = is_running(auto_brain_process)
    data["sensor_bridge_running"] = (
        is_running(manual_startup_process) or
        is_running(manual_sensor_process)
    )

    return data


# ============================================================
# PROCESS HELPERS
# ============================================================

def is_running(process):
    return process is not None and process.poll() is None


def start_process(name, file_path, log_name):
    if not file_path.exists():
        print(f"[ERROR] {name} file not found: {file_path}", flush=True)

        set_auto_status(
            action=f"Failed to start {name}",
            decision="File missing",
            error=f"{file_path} not found",
        )

        push_auto_track(
            "error",
            "Process Start",
            f"{name} file not found: {file_path}",
            important=True,
        )

        write_auto_status_file()
        return None

    log_path = LOG_DIR / log_name
    log_file = open(log_path, "a", buffering=1)

    print(f"[START] {name}", flush=True)
    print(f"[FILE]  {file_path}", flush=True)
    print(f"[LOG]   {log_path}", flush=True)

    push_auto_track(
        "action",
        "Service Start",
        f"Starting {name}",
        important=False,
    )

    write_auto_status_file()

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

    push_auto_track(
        "action",
        "Service Stop",
        f"Stopping {name}",
        important=False,
    )

    try:
        process.terminate()

        end_time = time.time() + timeout

        while time.time() < end_time:
            if process.poll() is not None:
                print(f"[OK] {name} stopped", flush=True)

                push_auto_track(
                    "info",
                    "Service Stop",
                    f"{name} stopped",
                    important=False,
                )

                write_auto_status_file()
                return None

            time.sleep(0.1)

        print(f"[WARN] {name} did not stop, killing...", flush=True)

        push_auto_track(
            "warning",
            "Service Stop",
            f"{name} did not stop normally, killing process",
            important=True,
        )

        process.kill()
        write_auto_status_file()
        return None

    except Exception as e:
        print(f"[ERROR] stopping {name}: {e}", flush=True)

        set_auto_status(
            error=f"Error stopping {name}: {e}",
        )

        push_auto_track(
            "error",
            "Service Stop",
            f"Error stopping {name}: {e}",
            important=True,
        )

        write_auto_status_file()
        return None


# ============================================================
# ESP32 COMMANDS
# ============================================================

def send_esp32_command(command):
    if websocket is None:
        print("[WARN] websocket-client not installed", flush=True)

        push_auto_track(
            "warning",
            "ESP32 Command",
            f"websocket-client not installed, command not sent: {command}",
            important=True,
        )

        write_auto_status_file()
        return False

    try:
        ws = websocket.create_connection(ESP32_WS_URL, timeout=2)
        ws.send(command)
        ws.close()

        print(f"[ESP32 CMD] {command}", flush=True)

        push_auto_track(
            "action",
            "ESP32 Command",
            f"Sent command to ESP32: {command}",
            important=False,
        )

        time.sleep(0.05)
        write_auto_status_file()
        return True

    except Exception as e:
        print(f"[ESP32 ERROR] {command}: {e}", flush=True)

        set_auto_status(
            error=f"ESP32 command failed: {command} | {e}",
        )

        push_auto_track(
            "error",
            "ESP32 Command",
            f"Failed to send command to ESP32: {command} | {e}",
            important=True,
        )

        write_auto_status_file()
        return False


def safe_stop_robot():
    commands = [
        "STOP",
        "JACK:REAR:STOP",
        "JACK:FRONT:STOP",
        "ARM:STOP",
        "CAM:STOP",
    ]

    set_auto_status(
        action="Sending safety stop commands",
        decision="Stop robot before mode switching",
    )

    for cmd in commands:
        send_esp32_command(cmd)


# ============================================================
# SERVICE STARTERS
# ============================================================

def start_manual_startup():
    global manual_startup_process

    if not is_running(manual_startup_process):
        manual_startup_process = start_process(
            name="Manual Services Startup Manager",
            file_path=MANUAL_STARTUP_FILE,
            log_name="manual_startup.log",
        )

        time.sleep(2)


def start_manual_sensor_bridge_only():
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
        set_auto_status(
            mode="MANUAL",
            stage="Switching to Manual",
            action="Stopping auto services",
            decision="Manual mode selected",
            error="",
        )

        push_auto_track(
            "decision",
            "Mode Switching",
            "Switching to MANUAL mode",
            important=True,
        )

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

        set_auto_status(
            stage="Manual Mode",
            action="Starting manual camera admin and sensor bridge",
            decision="Manual services will run",
            error="",
        )

        start_manual_startup()

        send_esp32_command("SYS:MODE:MANUAL")

        current_mode = "MANUAL"

        set_auto_status(
            mode="MANUAL",
            stage="Manual Mode",
            action="Manual mode is active",
            decision="Robot is ready for mobile manual control",
            error="",
        )

        push_auto_track(
            "info",
            "Manual Mode",
            "Manual mode active: camera admin + sensors",
            important=True,
        )

        write_auto_status_file()

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
        set_auto_status(
            mode="AUTO",
            stage="Auto Startup",
            action="Preparing robot for autonomous mode",
            decision="Auto mode selected",
            error="",
        )

        push_auto_track(
            "decision",
            "Auto Startup",
            "Switching to AUTO mode",
            important=True,
        )

        safe_stop_robot()

        manual_startup_process = stop_process(
            "Manual Services Startup Manager",
            manual_startup_process,
        )

        time.sleep(2)

        set_auto_status(
            stage="Auto Startup",
            action="Starting sensor bridge for auto mode",
            decision="Sensors are required for auto decisions",
            error="",
        )

        start_manual_sensor_bridge_only()

        set_auto_status(
            stage="Auto Startup",
            action="Starting front camera server",
            decision="Auto uses robot front camera first",
            error="",
        )

        start_auto_front_camera_server()

        send_esp32_command("SYS:MODE:AUTO")
        send_esp32_command("SPEED:55")

        set_auto_status(
            stage="Auto Startup",
            action="Starting auto brain",
            decision="Auto brain will begin decision tracking",
            error="",
        )

        start_auto_brain()

        current_mode = "AUTO"

        set_auto_status(
            mode="AUTO",
            stage="Auto Running",
            action="Auto brain is running",
            decision="Robot is ready to execute autonomous sequence",
            error="",
        )

        push_auto_track(
            "info",
            "Auto Running",
            "Auto mode active: front camera + sensor bridge + auto brain",
            important=True,
        )

        write_auto_status_file()

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
        set_auto_status(
            stage="Stopping Auto",
            action="Stopping auto and restoring manual mode",
            decision="Return to manual mode",
            error="",
        )

        push_auto_track(
            "decision",
            "Stopping Auto",
            "Stop auto and return to manual mode",
            important=True,
        )

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

        set_auto_status(
            mode="MANUAL",
            stage="Manual Mode",
            action="Manual mode restored",
            decision="Robot is ready for manual control",
            error="",
        )

        push_auto_track(
            "info",
            "Manual Mode",
            "Auto stopped. Manual camera admin restored.",
            important=True,
        )

        write_auto_status_file()

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
            "/auto_status",
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

        "manual_dual_camera_running": manual_running,
        "sensor_bridge_running": sensor_bridge_running,
        "auto_front_camera_running": is_running(auto_front_camera_process),
        "auto_brain_running": is_running(auto_brain_process),

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


@app.route("/auto_status")
def auto_status():
    return jsonify(auto_status_snapshot())


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

    set_auto_status(
        action="Emergency stop sent",
        decision="Stop all robot movement immediately",
    )

    push_auto_track(
        "warning",
        "Emergency Stop",
        "Emergency STOP sent to robot",
        important=True,
    )

    write_auto_status_file()

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

                    push_auto_track(
                        "warning",
                        "Monitor",
                        "Manual startup is down, restarting",
                        important=True,
                    )

                    start_manual_startup()

                if is_running(manual_sensor_process):
                    manual_sensor_process = stop_process(
                        "Sensor Bridge Only",
                        manual_sensor_process,
                    )

            elif current_mode == "AUTO":
                if is_running(manual_startup_process):
                    manual_startup_process = stop_process(
                        "Manual Services Startup Manager",
                        manual_startup_process,
                    )

                if not is_running(manual_sensor_process):
                    print("[MONITOR] Sensor bridge only is down, restarting...", flush=True)

                    set_auto_status(
                        stage="Auto Recovery",
                        action="Restarting sensor bridge",
                        decision="Sensor bridge must stay alive during auto",
                    )

                    push_auto_track(
                        "warning",
                        "Auto Recovery",
                        "Sensor bridge only is down, restarting",
                        important=True,
                    )

                    start_manual_sensor_bridge_only()

                if not is_running(auto_front_camera_process):
                    print("[MONITOR] Auto front camera is down, restarting...", flush=True)

                    set_auto_status(
                        stage="Auto Recovery",
                        action="Restarting auto front camera server",
                        decision="Robot camera must stay alive during auto",
                    )

                    push_auto_track(
                        "warning",
                        "Auto Recovery",
                        "Auto front camera is down, restarting",
                        important=True,
                    )

                    start_auto_front_camera_server()

                if not is_running(auto_brain_process):
                    print("[MONITOR] Auto brain is not running. Sending safety stop.", flush=True)

                    set_auto_status(
                        stage="Auto Error",
                        action="Auto brain stopped",
                        decision="Safety stop required",
                        error="Auto brain is not running",
                    )

                    push_auto_track(
                        "error",
                        "Auto Error",
                        "Auto brain is not running. Safety stop sent.",
                        important=True,
                    )

                    safe_stop_robot()

                write_auto_status_file()


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown_handler(sig, frame):
    global manual_startup_process
    global manual_sensor_process
    global auto_front_camera_process
    global auto_brain_process

    print("\n[SHUTDOWN] Stopping all Raspberry services...", flush=True)

    set_auto_status(
        stage="Shutdown",
        action="Stopping all Raspberry services",
        decision="Shutdown signal received",
    )

    push_auto_track(
        "warning",
        "Shutdown",
        "Stopping all Raspberry services",
        important=True,
    )

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

    write_auto_status_file()
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
    print("Auto Status: /auto_status", flush=True)
    print("==========================================", flush=True)

    set_auto_status(
        mode="MANUAL",
        stage="Startup",
        action="Starting Raspberry mode manager",
        decision="Default mode is MANUAL",
        error="",
    )

    push_auto_track(
        "info",
        "Startup",
        "Raspberry mode manager started",
        important=True,
    )

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