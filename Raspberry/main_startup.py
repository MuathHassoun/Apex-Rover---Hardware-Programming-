#!/usr/bin/env python3
"""
Apex Rover Raspberry Main Startup - CLEAN VERSION

This version removes the old Raspberry <-> Mega USB sensor bridge.

New design:
  - Mega sends SENSOR lines to ESP32 using Serial1.
  - ESP32 broadcasts SENSOR lines to the mobile app over WebSocket.
  - Raspberry does NOT open Mega USB Serial anymore.

Raspberry responsibilities now:
  1) Start the voice/sound service automatically.
  2) Run camera services only.
  3) In AUTO mode, open the front camera service only.
  4) The voice service handles SYS:MODE:AUTO / SYS:MODE:MANUAL logic:
       - speaks the command,
       - starts/stops camera stand scan,
       - sends AUTO:FULL_SCENARIO to ESP32 when AUTO starts.

Run:
  python3 main_startup.py

Service API:
  http://<raspberry-ip>:5050/status
  http://<raspberry-ip>:5050/auto_status
  http://<raspberry-ip>:5050/mode/manual
  http://<raspberry-ip>:5050/mode/auto
  http://<raspberry-ip>:5050/mode/stop
  http://<raspberry-ip>:5050/robot/stop
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Optional

from flask import Flask, jsonify, request

try:
    import websocket  # pip install websocket-client
except Exception:
    websocket = None


# ============================================================
# Paths
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
MANUAL_DIR = BASE_DIR / "Manual"
AUTO_DIR = BASE_DIR / "Auto"

MANUAL_STARTUP_FILE = MANUAL_DIR / "apex_rover_manual_servers_startup.py"
AUTO_FRONT_CAMERA_FILE = AUTO_DIR / "front_camera_server.py"
VOICE_SERVICE_FILE = BASE_DIR / "raspberry_voice_auto_service.py"

LOG_DIR = Path("/tmp/apex_rover_logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)

AUTO_STATUS_FILE = Path("/tmp/apex_auto_status.json")


# ============================================================
# Network
# ============================================================

ESP32_IP = os.environ.get("APEX_ESP32_IP", "192.168.4.1")
ESP32_WS_URL = os.environ.get("APEX_ESP32_WS_URL", f"ws://{ESP32_IP}:81")
ESP32_HTTP_PORT = int(os.environ.get("APEX_ESP32_HTTP_PORT", "80"))

SERVER_HOST = "0.0.0.0"
SERVER_PORT = int(os.environ.get("APEX_RPI_MODE_PORT", "5050"))


# ============================================================
# Flask / state
# ============================================================

app = Flask(__name__)

process_lock = threading.RLock()
status_lock = threading.RLock()

current_mode = "MANUAL"

manual_camera_process: Optional[subprocess.Popen] = None
auto_front_camera_process: Optional[subprocess.Popen] = None
voice_process: Optional[subprocess.Popen] = None

_shutdown = threading.Event()

_auto_status = {
    "ok": True,
    "mode": "MANUAL",
    "stage": "Startup",
    "action": "Starting Raspberry mode manager",
    "decision": "Default mode is MANUAL",
    "error": "",
    "time": "",
    "track": [],
}


# ============================================================
# Utility
# ============================================================

def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def log(message: str) -> None:
    print(f"[{now_text()}] {message}", flush=True)


def is_running(process: Optional[subprocess.Popen]) -> bool:
    return process is not None and process.poll() is None


def set_status(stage=None, action=None, decision=None, error=None, mode=None) -> None:
    with status_lock:
        if mode is not None:
            _auto_status["mode"] = mode
        if stage is not None:
            _auto_status["stage"] = stage
        if action is not None:
            _auto_status["action"] = action
        if decision is not None:
            _auto_status["decision"] = decision
        if error is not None:
            _auto_status["error"] = error
        _auto_status["ok"] = True
        _auto_status["time"] = now_text()
        write_status_file_locked()


def push_track(track_type: str, stage: str, message: str, important: bool = False) -> None:
    with status_lock:
        item = {
            "type": track_type,
            "stage": stage,
            "message": message,
            "time": now_text(),
            "important": important,
        }
        _auto_status["track"].append(item)
        _auto_status["track"] = _auto_status["track"][-100:]
        _auto_status["time"] = item["time"]
        write_status_file_locked()


def write_status_file_locked() -> None:
    try:
        AUTO_STATUS_FILE.write_text(
            json.dumps(_auto_status, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception as exc:
        log(f"[STATUS WARN] Could not write {AUTO_STATUS_FILE}: {exc}")


def process_log_file(log_name: str):
    log_path = LOG_DIR / log_name
    return open(log_path, "a", buffering=1)


def start_process(name: str, file_path: Path, log_name: str, cwd: Optional[Path] = None) -> Optional[subprocess.Popen]:
    if not file_path.exists():
        log(f"[ERROR] {name} file not found: {file_path}")
        set_status(action=f"Failed to start {name}", decision="File missing", error=str(file_path))
        push_track("error", "Process Start", f"{name} file not found: {file_path}", True)
        return None

    log_path = LOG_DIR / log_name
    log(f"[START] {name}")
    log(f"[FILE]  {file_path}")
    log(f"[LOG]   {log_path}")

    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    env["PATH"] = env.get("PATH", "") + ":/usr/local/bin:/usr/bin:/bin"
    env.setdefault("APEX_ESP32_IP", ESP32_IP)
    env.setdefault("APEX_ESP32_HTTP_PORT", str(ESP32_HTTP_PORT))
    env.setdefault("APEX_IDLE_MUSIC", "ASSAULT.mp4")
    env.setdefault("APEX_RPI_MODE_URL", f"http://127.0.0.1:{SERVER_PORT}/internal/mode_from_voice")

    return subprocess.Popen(
        [sys.executable, "-u", str(file_path)],
        cwd=str(cwd or file_path.parent),
        stdout=process_log_file(log_name),
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )


def stop_process(name: str, process: Optional[subprocess.Popen], timeout: float = 5.0) -> None:
    if process is None:
        return

    if process.poll() is not None:
        return

    log(f"[STOP] {name}")
    try:
        process.terminate()
        process.wait(timeout=timeout)
        log(f"[OK] {name} stopped")
    except subprocess.TimeoutExpired:
        log(f"[KILL] {name}")
        process.kill()
    except Exception as exc:
        log(f"[ERROR] Could not stop {name}: {exc}")


# ============================================================
# ESP32 command helpers
# ============================================================

def send_esp32_command_http(command: str, timeout: float = 1.5) -> bool:
    encoded = urllib.parse.quote(command, safe="")
    url = f"http://{ESP32_IP}:{ESP32_HTTP_PORT}/command?cmd={encoded}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            body = response.read(120).decode("utf-8", errors="ignore")
            ok = 200 <= response.status < 300
            log(f"[ESP32 HTTP] {command} -> {response.status} {body[:80]}")
            return ok
    except Exception as exc:
        log(f"[ESP32 HTTP ERROR] {command}: {exc}")
        return False


def send_esp32_command_ws(command: str, timeout: float = 1.5) -> bool:
    if websocket is None:
        return False
    try:
        ws = websocket.create_connection(ESP32_WS_URL, timeout=timeout)
        ws.send(command)
        ws.close()
        log(f"[ESP32 WS] {command}")
        return True
    except Exception as exc:
        log(f"[ESP32 WS ERROR] {command}: {exc}")
        return False


def send_esp32_command(command: str) -> bool:
    # HTTP is usually easier for Raspberry scripts. WebSocket fallback is kept.
    if send_esp32_command_http(command):
        push_track("action", "ESP32 Command", f"Sent command to ESP32: {command}")
        return True

    if send_esp32_command_ws(command):
        push_track("action", "ESP32 Command", f"Sent command to ESP32: {command}")
        return True

    set_status(error=f"Failed to send ESP32 command: {command}")
    push_track("error", "ESP32 Command", f"Failed to send command to ESP32: {command}", True)
    return False


def safe_stop_robot() -> None:
    for cmd in [
        "BLOCK:STOP",
        "AUTO:FULL_STOP",
        "AUTO:STOP",
        "STOP",
        "JACK:ALL:STOP",
        "ARM:STOP",
        "CAM:STOP",
    ]:
        send_esp32_command(cmd)
        time.sleep(0.04)


# ============================================================
# Service control
# ============================================================

def start_voice_service() -> None:
    global voice_process
    if is_running(voice_process):
        return
    voice_process = start_process(
        "Raspberry Voice Auto Service",
        VOICE_SERVICE_FILE,
        "voice_auto_service.log",
        cwd=BASE_DIR,
    )


def stop_voice_service() -> None:
    global voice_process
    stop_process("Raspberry Voice Auto Service", voice_process)
    voice_process = None


def start_manual_camera() -> None:
    global manual_camera_process
    if is_running(manual_camera_process):
        return
    manual_camera_process = start_process(
        "Manual Camera Admin",
        MANUAL_STARTUP_FILE,
        "manual_camera.log",
        cwd=MANUAL_DIR,
    )
    time.sleep(1)


def stop_manual_camera() -> None:
    global manual_camera_process
    stop_process("Manual Camera Admin", manual_camera_process)
    manual_camera_process = None


def start_auto_front_camera() -> None:
    global auto_front_camera_process
    if is_running(auto_front_camera_process):
        return
    auto_front_camera_process = start_process(
        "Auto Front Camera Server",
        AUTO_FRONT_CAMERA_FILE,
        "auto_front_camera.log",
        cwd=AUTO_DIR,
    )
    time.sleep(1)


def stop_auto_front_camera() -> None:
    global auto_front_camera_process
    stop_process("Auto Front Camera Server", auto_front_camera_process)
    auto_front_camera_process = None


# ============================================================
# Local mode application
# ============================================================

def apply_local_manual(source: str = "api") -> dict:
    global current_mode
    with process_lock:
        log(f"[MODE] Apply local MANUAL from {source}")
        stop_auto_front_camera()
        start_manual_camera()
        current_mode = "MANUAL"
        set_status(
            mode="MANUAL",
            stage="Manual Mode",
            action="Manual camera service is active. Mega USB sensor bridge is disabled.",
            decision="Sensors are handled by Mega -> ESP32 -> Mobile only.",
            error="",
        )
        push_track("info", "Manual Mode", "Manual mode active: camera only, no Mega USB sensor bridge.", True)
        return {"ok": True, "mode": current_mode, "message": "Manual mode active. Sensor bridge disabled."}


def apply_local_auto(source: str = "api") -> dict:
    global current_mode
    with process_lock:
        log(f"[MODE] Apply local AUTO from {source}")
        stop_manual_camera()
        start_auto_front_camera()
        current_mode = "AUTO"
        set_status(
            mode="AUTO",
            stage="Auto Camera Scan",
            action="Auto front camera service is active. Camera stand scan is controlled by the voice service.",
            decision="ESP32 will execute AUTO:FULL_SCENARIO after SYS:MODE:AUTO.",
            error="",
        )
        push_track("info", "Auto Mode", "Auto mode active: front camera only; no Raspberry Mega sensor bridge; no Raspberry auto brain.", True)
        return {"ok": True, "mode": current_mode, "message": "Auto mode active. Front camera service only."}


def switch_to_manual() -> dict:
    # API action: tell ESP32 too. Voice service will also hear the mirror and stop scan.
    safe_stop_robot()
    send_esp32_command("SYS:MODE:MANUAL")
    return apply_local_manual("api")


def switch_to_auto() -> dict:
    # API action: tell ESP32. Voice service will hear SYS:MODE:AUTO and send AUTO:FULL_SCENARIO.
    safe_stop_robot()
    send_esp32_command("SYS:MODE:AUTO")
    return apply_local_auto("api")


def stop_auto_and_return_manual() -> dict:
    safe_stop_robot()
    send_esp32_command("SYS:MODE:MANUAL")
    return apply_local_manual("api_stop")


# ============================================================
# API routes
# ============================================================

@app.route("/")
def home():
    return jsonify({
        "service": "Apex Rover Raspberry Mode Manager - Clean No-Mega-USB Version",
        "mode": current_mode,
        "important": "Raspberry no longer opens Mega USB Serial. Sensors go Mega -> ESP32 -> Mobile.",
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
    return jsonify({
        "ok": True,
        "mode": current_mode,
        "mega_usb_sensor_bridge_enabled": False,
        "sensor_path": "Mega Serial1 -> ESP32 -> Mobile WebSocket only",
        "manual_camera_running": is_running(manual_camera_process),
        "auto_front_camera_running": is_running(auto_front_camera_process),
        "voice_service_running": is_running(voice_process),
        "auto_brain_running": False,
        "sensor_bridge_running": False,
        "esp32_ip": ESP32_IP,
        "esp32_ws_url": ESP32_WS_URL,
        "logs": {
            "voice": str(LOG_DIR / "voice_auto_service.log"),
            "manual_camera": str(LOG_DIR / "manual_camera.log"),
            "auto_front_camera": str(LOG_DIR / "auto_front_camera.log"),
        },
    })


@app.route("/auto_status")
def auto_status():
    with status_lock:
        data = dict(_auto_status)
        data["track"] = list(_auto_status.get("track", []))

    data.update({
        "ok": True,
        "mode": current_mode,
        "stage": data.get("stage", "AUTO"),
        "action": data.get("action", ""),
        "decision": data.get("decision", ""),
        "error": data.get("error", ""),
        "mega_usb_sensor_bridge_enabled": False,
        "sensor_bridge_running": False,
        "auto_brain_running": False,
        "auto_front_camera_running": is_running(auto_front_camera_process),
        "voice_service_running": is_running(voice_process),
    })
    return jsonify(data)


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
    set_status(action="Emergency stop sent", decision="Stop all robot movement immediately")
    push_track("warning", "Emergency Stop", "Emergency STOP sent to robot", True)
    return jsonify({"ok": True, "mode": current_mode, "message": "Emergency STOP sent."})


@app.route("/internal/mode_from_voice", methods=["GET", "POST"])
def internal_mode_from_voice():
    mode = (request.values.get("mode") or "").upper().strip()
    if mode == "AUTO":
        return jsonify(apply_local_auto("voice_service"))
    if mode == "MANUAL":
        return jsonify(apply_local_manual("voice_service"))
    return jsonify({"ok": False, "error": "mode must be AUTO or MANUAL"}), 400


# ============================================================
# Monitor / shutdown
# ============================================================

def monitor_loop() -> None:
    while not _shutdown.is_set():
        time.sleep(3)
        with process_lock:
            if not is_running(voice_process):
                log("[MONITOR] Voice service stopped, restarting...")
                start_voice_service()

            if current_mode == "MANUAL":
                if is_running(auto_front_camera_process):
                    stop_auto_front_camera()
                if not is_running(manual_camera_process):
                    log("[MONITOR] Manual camera service stopped, restarting...")
                    start_manual_camera()

            elif current_mode == "AUTO":
                if is_running(manual_camera_process):
                    stop_manual_camera()
                if not is_running(auto_front_camera_process):
                    log("[MONITOR] Auto front camera service stopped, restarting...")
                    start_auto_front_camera()


def shutdown_handler(sig=None, frame=None) -> None:
    _shutdown.set()
    log("[SHUTDOWN] Stopping Raspberry services...")
    try:
        safe_stop_robot()
    except Exception:
        pass

    with process_lock:
        stop_auto_front_camera()
        stop_manual_camera()
        stop_voice_service()

    log("[SHUTDOWN] Done")
    sys.exit(0)


def main() -> None:
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    log("==========================================")
    log("Apex Rover Raspberry Mode Manager")
    log("CLEAN VERSION: No Mega USB sensor bridge")
    log("Sensors: Mega -> ESP32 -> Mobile only")
    log("Default mode: MANUAL")
    log(f"API port: {SERVER_PORT}")
    log("==========================================")

    set_status(
        mode="MANUAL",
        stage="Startup",
        action="Starting Raspberry mode manager",
        decision="Default mode is MANUAL. Voice service starts automatically.",
        error="",
    )
    push_track("info", "Startup", "Raspberry clean mode manager started", True)

    start_voice_service()
    apply_local_manual("startup")

    threading.Thread(target=monitor_loop, daemon=True).start()

    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)


if __name__ == "__main__":
    main()
