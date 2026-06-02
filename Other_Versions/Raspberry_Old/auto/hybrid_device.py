import threading
import serial
import requests
from typing import Optional
from collections import deque
from config import SERIAL_TIMEOUT, SERIAL_WRITE_TIMEOUT, ESP32_HTTP_TIMEOUT


class HybridDevice:
    """
    Hybrid communication device for Mega and UNO.

    SEND path  : Raspberry Pi  ->  HTTP POST /command  ->  ESP32  ->  Arduino
    READ path  : Raspberry Pi  <-  USB Serial direct   <-  Arduino

    This solves the one-way problem: the ESP32 forwards commands to the
    Arduinos but cannot relay their replies back to the Pi over HTTP in
    real-time. By keeping a direct USB Serial read channel we get full
    two-way communication without changing the Arduino firmware.

    The public API is identical to SerialDevice and EspBridge:
        device.send("FORWARD")
        line  = device.read_line()
        device.connect()
        device.close()
    """

    def __init__(
        self,
        name: str,
        esp32_ip: str,
        serial_port: str,
        baud_rate: int = 9600,
        http_timeout: float = ESP32_HTTP_TIMEOUT,
    ):
        self.name        = name
        self.esp32_ip    = esp32_ip.rstrip("/")
        self.serial_port = serial_port
        self.baud_rate   = baud_rate
        self.http_timeout = http_timeout

        self._cmd_url = f"http://{self.esp32_ip}/command"

        # USB Serial for reading replies from the Arduino.
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()

        # Small buffer so read_line() is non-blocking.
        self._rx_queue: deque = deque(maxlen=64)
        self._rx_lock   = threading.Lock()

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_reader   = threading.Event()
        self._connected     = False

    # ------------------------------------------------------------------
    # Connect: open the USB Serial read channel.
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port         = self.serial_port,
                baudrate     = self.baud_rate,
                timeout      = SERIAL_TIMEOUT,
                write_timeout= SERIAL_WRITE_TIMEOUT,
            )
            import time; time.sleep(2)   # wait for Arduino reset after DTR

            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass

            self._stop_reader.clear()
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                name=f"HybridReader-{self.name}",
                daemon=True,
            )
            self._reader_thread.start()

            self._connected = True
            print(f"[OK] {self.name}: USB read channel open on {self.serial_port}")
            print(f"[OK] {self.name}: HTTP send channel -> {self._cmd_url}")
            return True

        except Exception as e:
            print(f"[ERROR] {self.name}: cannot open {self.serial_port} — {e}")
            self._serial = None
            self._connected = False
            return False

    def is_connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    def close(self):
        self._stop_reader.set()
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        self._connected = False
        print(f"[OK] {self.name}: closed")

    # ------------------------------------------------------------------
    # Background thread: read lines from USB Serial and push to queue.
    # ------------------------------------------------------------------

    def _reader_loop(self):
        while not self._stop_reader.is_set():
            try:
                with self._serial_lock:
                    if self._serial and self._serial.in_waiting:
                        raw = self._serial.readline()
                    else:
                        raw = b""

                if raw:
                    line = raw.decode(errors="ignore").strip()
                    if line:
                        with self._rx_lock:
                            self._rx_queue.append(line)
            except Exception as e:
                print(f"[WARN] {self.name} reader error: {e}")

    # ------------------------------------------------------------------
    # SEND: go through ESP32 HTTP.
    # ------------------------------------------------------------------

    def send(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return False

        print(f"[SEND via ESP32 -> {self.name}] {command}")

        try:
            r = requests.post(
                self._cmd_url,
                data    = {"cmd": command},
                timeout = self.http_timeout,
            )
            if r.status_code == 200:
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
    # READ: pop one line from the queue (filled by the reader thread).
    # Returns None immediately if nothing is available.
    # ------------------------------------------------------------------

    def read_line(self) -> Optional[str]:
        with self._rx_lock:
            if self._rx_queue:
                return self._rx_queue.popleft()
        return None

    # ------------------------------------------------------------------
    # fetch_sensors: kept for compatibility with control_brain.
    # In hybrid mode the reader thread already captures all Arduino
    # output, so we just ask the Mega via the normal send channel and
    # the reply will appear in read_line() automatically.
    # ------------------------------------------------------------------

    def fetch_sensors(self) -> None:
        self.send("GET:SENSORS")
