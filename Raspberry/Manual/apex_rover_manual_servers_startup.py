#!/usr/bin/env python3
"""
apex_rover_manual_servers_startup.py

Apex Rover Raspberry Pi Manual Services Startup Manager

This file starts and monitors Raspberry Pi MANUAL mode microservices:

1. camera_admin_server.py
   - Smart manual camera admin
   - Opens ONLY ONE camera at a time
   - Front camera for Basic / Rear Jack / Front Jack modes
   - Arm camera for Arm mode
   - Flask server on port 5000
   - Keeps old URLs working:
       /front_snapshot
       /arm_snapshot
       /front_camera
       /arm_camera
       /status

2. sensor_bridge.py
   - Reads SENSOR lines from Arduino Mega over USB Serial
   - Saves latest MPU/sensor data to /tmp/apex_last_sensor.json
   - Sends sensor updates to ESP32 /sensor_update

Important:
Raspberry Pi does NOT control robot movement in Manual mode.
All manual movement commands come from:

Mobile App -> ESP32 -> Mega / UNO
"""

import os
import sys
import time
import signal
import subprocess
import threading
from datetime import datetime


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

SERVICES = [
    {
        "name": "camera_admin_service",
        "file": "camera_admin_server.py",
        "restart_delay": 3,
    },
    {
        "name": "sensor_bridge",
        "file": "sensor_bridge.py",
        "restart_delay": 3,
    },
]

processes = {}
running = True


def log(message):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def service_path(service_file):
    return os.path.join(BASE_DIR, service_file)


def start_service(service):
    name = service["name"]
    path = service_path(service["file"])

    if not os.path.exists(path):
        log(f"[ERROR] {name}: file not found: {path}")
        return None

    old_item = processes.get(name)
    if old_item:
        old_process = old_item.get("process")
        if old_process is not None and old_process.poll() is None:
            log(f"[INFO] {name} already running")
            return old_process

    log(f"[START] {name}: {sys.executable} -u {path}")

    process = subprocess.Popen(
        [sys.executable, "-u", path],
        cwd=BASE_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    processes[name] = {
        "process": process,
        "service": service,
        "last_restart": time.time(),
    }

    output_thread = threading.Thread(
        target=print_service_output,
        args=(name, process),
        daemon=True,
    )
    output_thread.start()

    return process


def stop_service(name):
    item = processes.get(name)
    if not item:
        return

    process = item["process"]

    if process.poll() is None:
        log(f"[STOP] {name}")
        process.terminate()

        try:
            process.wait(timeout=5)
            log(f"[OK] {name} stopped")
        except subprocess.TimeoutExpired:
            log(f"[KILL] {name}")
            process.kill()

    processes.pop(name, None)


def stop_all_services():
    log("[INFO] Stopping all manual services...")

    for name in list(processes.keys()):
        stop_service(name)

    log("[OK] All manual services stopped")


def handle_shutdown(signum, frame):
    global running

    log(f"[INFO] Shutdown signal received: {signum}")
    running = False
    stop_all_services()
    sys.exit(0)


def print_service_output(name, process):
    if process.stdout is None:
        return

    while running:
        if process.poll() is not None:
            break

        line = process.stdout.readline()

        if line:
            print(f"[{name}] {line}", end="", flush=True)
        else:
            time.sleep(0.05)


def monitor_services():
    global running

    log("=" * 60)
    log("Apex Rover Manual Services Startup Manager")
    log("Mode: MANUAL")
    log("Camera: Smart camera admin, one camera active at a time")
    log("Sensors: Mega sensor bridge")
    log("=" * 60)

    for service in SERVICES:
        start_service(service)

    log("[OK] Manual services manager is running")
    log("[INFO] Press Ctrl+C to stop")

    while running:
        for service in SERVICES:
            name = service["name"]

            if name not in processes:
                log(f"[WARN] {name} not running. Starting...")
                start_service(service)
                continue

            process = processes[name]["process"]
            exit_code = process.poll()

            if exit_code is not None:
                log(f"[ERROR] {name} stopped with exit code {exit_code}")
                processes.pop(name, None)

                delay = service.get("restart_delay", 3)
                log(f"[INFO] Restarting {name} in {delay} seconds...")
                time.sleep(delay)

                if running:
                    start_service(service)

        time.sleep(1)


def main():
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        monitor_services()
    except KeyboardInterrupt:
        handle_shutdown(signal.SIGINT, None)


if __name__ == "__main__":
    main()