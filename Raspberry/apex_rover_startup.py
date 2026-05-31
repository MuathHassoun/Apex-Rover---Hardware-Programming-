#!/usr/bin/env python3
"""
apex_rover_startup.py

Apex Rover Raspberry Pi Main Startup Manager

This file starts and monitors Raspberry Pi microservices:

1. dual_camera_server.py
   - Front camera stream
   - Arm camera stream
   - Flask server on port 5000

2. sensor_bridge.py
   - Reads SENSOR lines from Arduino Mega
   - Sends sensor updates to ESP32 /sensor_update

Important:
Raspberry Pi does NOT control robot movement.
All movement commands come from:
Mobile App -> ESP32 -> Mega / UNO
"""

import os
import sys
import time
import signal
import subprocess
from datetime import datetime


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

SERVICES = [
    {
        "name": "camera_service",
        "file": "dual_camera_server.py",
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

    log(f"[START] {name}: python3 {path}")

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
        except subprocess.TimeoutExpired:
            log(f"[KILL] {name}")
            process.kill()

    processes.pop(name, None)


def stop_all_services():
    log("[INFO] Stopping all services...")

    for name in list(processes.keys()):
        stop_service(name)

    log("[OK] All services stopped")


def handle_shutdown(signum, frame):
    global running
    log(f"[INFO] Shutdown signal received: {signum}")
    running = False
    stop_all_services()
    sys.exit(0)


def print_service_output(name, process):
    """
    Non-blocking output reader is not used here to keep code simple.
    We read output line-by-line using a small background process loop.
    """
    if process.stdout is None:
        return

    while running and process.poll() is None:
        line = process.stdout.readline()

        if line:
            print(f"[{name}] {line}", end="", flush=True)
        else:
            time.sleep(0.05)


def monitor_services():
    global running

    log("=" * 60)
    log("Apex Rover Startup Manager")
    log("Mode: Manual only")
    log("Starting Raspberry Pi microservices...")
    log("=" * 60)

    for service in SERVICES:
        start_service(service)

    log("[OK] Startup manager is running")
    log("[INFO] Press Ctrl+C to stop")

    # Simple monitor loop
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
