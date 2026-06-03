#!/usr/bin/env python3
"""
auto_status.py
Shared status tracker for Apex Rover AUTO mode.

Both auto_stair_climb.py and front_camera_server.py use this file.
Status is stored in /tmp/apex_auto_status.json and shown from /auto_status.
"""

import json
import os
import time
from collections import deque
from threading import RLock

try:
    from auto_config import STATUS_FILE, EVENTS_FILE
except Exception:
    STATUS_FILE = "/tmp/apex_auto_status.json"
    EVENTS_FILE = "/tmp/apex_auto_status_events.jsonl"


class AutoStatus:
    def __init__(self, max_history=100):
        self.lock = RLock()
        self.max_history = max_history
        self.history = deque(maxlen=max_history)
        self.data = {
            "ok": True,
            "mode": "AUTO",
            "running": False,
            "scenario": "pickup_climb_deliver",
            "phase": "BOOT",
            "doing": "Starting auto system",
            "decision": "none",
            "error": None,
            "last_command": None,
            "last_detection": None,
            "last_sensor": None,
            "objects_loaded": 0,
            "objects_delivered": 0,
            "started_at": time.time(),
            "updated_at": time.time(),
            "history": [],
        }
        self._load_existing_history()
        self.write()

    def _load_existing_history(self):
        try:
            if os.path.exists(STATUS_FILE):
                with open(STATUS_FILE, "r", encoding="utf-8") as f:
                    old = json.load(f)
                for item in old.get("history", [])[-self.max_history:]:
                    self.history.append(item)
        except Exception:
            pass

    def event(self, phase=None, doing=None, decision=None, error=None, **extra):
        with self.lock:
            now = time.time()
            entry = {
                "time": now,
                "phase": phase or self.data.get("phase"),
                "doing": doing if doing is not None else self.data.get("doing"),
                "decision": decision if decision is not None else self.data.get("decision"),
                "error": error,
            }
            for key, value in extra.items():
                entry[key] = value

            if phase is not None:
                self.data["phase"] = phase
            if doing is not None:
                self.data["doing"] = doing
            if decision is not None:
                self.data["decision"] = decision
            if error is not None:
                self.data["error"] = str(error)

            for key, value in extra.items():
                self.data[key] = value

            self.data["updated_at"] = now
            self.history.append(entry)
            self.data["history"] = list(self.history)
            self.write_locked()

            try:
                with open(EVENTS_FILE, "a", encoding="utf-8") as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            except Exception:
                pass

    def set_running(self, running):
        self.event(running=bool(running))

    def set_command(self, cmd):
        self.event(last_command=cmd, decision=f"sent command: {cmd}")

    def set_sensor(self, sensor):
        self.event(last_sensor=sensor)

    def set_detection(self, detection):
        self.event(last_detection=detection)

    def set_counts(self, loaded=None, delivered=None):
        extra = {}
        if loaded is not None:
            extra["objects_loaded"] = int(loaded)
        if delivered is not None:
            extra["objects_delivered"] = int(delivered)
        self.event(**extra)

    def fail(self, phase, message, **extra):
        self.event(phase=phase, doing="Stopped because of error", decision="STOP", error=message, **extra)

    def write(self):
        with self.lock:
            self.write_locked()

    def write_locked(self):
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, STATUS_FILE)


def read_status_file():
    try:
        if not os.path.exists(STATUS_FILE):
            return {
                "ok": False,
                "error": "auto status file not created yet",
                "status_file": STATUS_FILE,
            }
        with open(STATUS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        return {
            "ok": False,
            "error": str(e),
            "status_file": STATUS_FILE,
        }
