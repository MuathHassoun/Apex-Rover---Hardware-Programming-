import time
import threading
import serial
import requests
from typing import Optional
from config import MEGA_PORT, BAUD_RATE, SERIAL_TIMEOUT, SERIAL_WRITE_TIMEOUT, ESP32_IP

# =================================================================
# sensor_relay.py
#
# Single responsibility: read sensor lines from the Mega over USB
# Serial and forward them to the ESP32 via HTTP POST /sensor.
#
# The Mega pushes data automatically:
#   - Every 10 seconds: routine report
#   - Immediately: critical alert (tilt danger, obstacle)
#
# Data format from Mega:
#   SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
#
# The ESP32 then broadcasts the data as JSON to the mobile app
# via WebSocket.
# =================================================================

ESP32_SENSOR_URL = f"http://{ESP32_IP}/sensor"
FORWARD_TIMEOUT  = 1.5   # seconds for HTTP POST to ESP32


class SensorRelay:
    """
    Reads from Mega USB Serial and forwards to ESP32 HTTP.

    Thread-safe. Call start() once; call stop() to shut down.
    """

    def __init__(self):
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False

    def start(self) -> bool:
        try:
            self._serial = serial.Serial(
                port          = MEGA_PORT,
                baudrate      = BAUD_RATE,
                timeout       = SERIAL_TIMEOUT,
                write_timeout = SERIAL_WRITE_TIMEOUT,
            )
            time.sleep(2)   # wait for Arduino reset after DTR toggle
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass

            self._stop_event.clear()
            self._thread = threading.Thread(
                target = self._relay_loop,
                name   = "SensorRelayThread",
                daemon = True,
            )
            self._thread.start()
            self._running = True
            print(f"[OK] SensorRelay: reading Mega on {MEGA_PORT}")
            print(f"[OK] SensorRelay: forwarding to {ESP32_SENSOR_URL}")
            return True

        except Exception as e:
            print(f"[ERROR] SensorRelay: cannot open {MEGA_PORT} — {e}")
            return False

    def stop(self):
        self._stop_event.set()
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        self._running = False
        print("[OK] SensorRelay stopped")

    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Background thread: read lines, filter sensor lines, forward them.
    # ------------------------------------------------------------------
    def _relay_loop(self):
        buffer = ""
        while not self._stop_event.is_set():
            try:
                if self._serial and self._serial.is_open and self._serial.in_waiting:
                    raw = self._serial.readline()
                    if raw:
                        line = raw.decode(errors="ignore").strip()
                        if line:
                            self._handle_line(line)
                else:
                    time.sleep(0.01)

            except Exception as e:
                print(f"[WARN] SensorRelay read error: {e}")
                time.sleep(0.1)

    def _handle_line(self, line: str):
        # Only forward lines that start with "SENSOR:" — ignore boot
        # messages, ACKs, etc.
        if not line.startswith("SENSOR:"):
            print(f"[Mega->Pi] {line}")
            return

        print(f"[Mega SENSOR] {line}")
        self._forward_to_esp32(line)

    def _forward_to_esp32(self, sensor_line: str):
        try:
            r = requests.post(
                ESP32_SENSOR_URL,
                data    = {"data": sensor_line},
                timeout = FORWARD_TIMEOUT,
            )
            if r.status_code == 200:
                print(f"[OK] Forwarded to ESP32: {sensor_line}")
            else:
                print(f"[WARN] ESP32 returned HTTP {r.status_code}")
        except requests.exceptions.Timeout:
            print("[WARN] ESP32 HTTP timeout when forwarding sensor data")
        except Exception as e:
            print(f"[ERROR] Failed to forward to ESP32: {e}")
