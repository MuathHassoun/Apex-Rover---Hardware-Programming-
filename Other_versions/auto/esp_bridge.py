import time
import threading
import requests
from typing import Optional
from collections import deque


class EspBridge:
    """
    Replaces SerialDevice for Mega and UNO.

    Instead of writing bytes over USB Serial, every command is sent
    as an HTTP POST to the ESP32 WiFi bridge:

        POST http://<ESP32_IP>/command
        Body (form): cmd=<command>

    The ESP32 then routes the command to the correct Arduino
    (Mega or UNO) over its own HardwareSerial lines.

    Sensor responses that the Mega sends back are received by the ESP32
    via the /get_sensors HTTP endpoint and returned here.

    Usage is identical to SerialDevice:
        device.send("FORWARD")
        line = device.read_line()
        device.connect()
        device.close()
    """

    def __init__(self, name: str, esp32_ip: str, timeout_sec: float = 0.4):
        self.name = name
        self.esp32_ip = esp32_ip.rstrip("/")
        self.timeout_sec = timeout_sec

        self._cmd_url = f"http://{self.esp32_ip}/command"
        self._sensor_url = f"http://{self.esp32_ip}/get_sensors"
        self._status_url = f"http://{self.esp32_ip}/get_status"

        # Lines pushed by send() that look like sensor data replies
        # are queued here so read_line() can return them.
        self._rx_queue: deque = deque(maxlen=32)
        self._lock = threading.Lock()
        self._connected = False

    # ------------------------------------------------------------------
    # Connection (no real socket — just a reachability check)
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            r = requests.get(self._status_url, timeout=2.0)
            if r.status_code == 200:
                self._connected = True
                print(f"[OK] {self.name} reachable via ESP32 at {self.esp32_ip}")
                return True
        except Exception as e:
            print(f"[ERROR] {self.name}: cannot reach ESP32 at {self.esp32_ip} — {e}")
        self._connected = False
        return False

    def is_connected(self) -> bool:
        return self._connected

    def close(self):
        self._connected = False
        print(f"[OK] {self.name} bridge closed")

    # ------------------------------------------------------------------
    # Send a command to the ESP32 (which routes to Mega or UNO)
    # ------------------------------------------------------------------

    def send(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return False

        print(f"[SEND VIA ESP32 -> {self.name}] {command}")

        try:
            r = requests.post(
                self._cmd_url,
                data={"cmd": command},
                timeout=self.timeout_sec
            )
            if r.status_code == 200:
                # If the server echoed anything useful, queue it.
                body = r.text.strip()
                if body:
                    with self._lock:
                        self._rx_queue.append(body)
                return True
            else:
                print(f"[WARN] {self.name} HTTP {r.status_code} for cmd={command}")
                return False
        except requests.exceptions.Timeout:
            print(f"[WARN] {self.name} HTTP timeout for cmd={command}")
            return False
        except Exception as e:
            print(f"[ERROR] {self.name} send failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Read one line (sensor data or ACK) from the receive queue.
    # If the queue is empty and this is the Mega bridge, fetch sensors.
    # ------------------------------------------------------------------

    def read_line(self) -> Optional[str]:
        with self._lock:
            if self._rx_queue:
                return self._rx_queue.popleft()
        return None

    # ------------------------------------------------------------------
    # Dedicated sensor fetch — called by control_brain every second.
    # Returns the raw DATA: line or None.
    # ------------------------------------------------------------------

    def fetch_sensors(self) -> Optional[str]:
        try:
            r = requests.get(self._sensor_url, timeout=self.timeout_sec)
            if r.status_code == 200:
                import json as _json
                body = r.json()
                if body.get("ok") and body.get("raw"):
                    raw = body["raw"].strip()
                    if raw:
                        with self._lock:
                            self._rx_queue.append(raw)
                        return raw
        except Exception as e:
            print(f"[WARN] fetch_sensors failed: {e}")
        return None
