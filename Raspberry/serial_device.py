import time
import threading
import serial
from typing import Optional
from config import SERIAL_TIMEOUT, SERIAL_WRITE_TIMEOUT


class SerialDevice:
    def __init__(self, name: str, port: str, baud_rate: int = 9600):
        self.name = name
        self.port = port
        self.baud_rate = baud_rate
        self.device: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    def connect(self) -> bool:
        try:
            self.device = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_WRITE_TIMEOUT
            )
            time.sleep(2)  # Arduino reset
            try:
                self.device.reset_input_buffer()
                self.device.reset_output_buffer()
            except Exception:
                pass
            print(f"[OK] Connected to {self.name} on {self.port}")
            return True
        except Exception as e:
            print(f"[ERROR] Could not connect to {self.name} on {self.port}")
            print(e)
            self.device = None
            return False

    def is_connected(self) -> bool:
        return self.device is not None and self.device.is_open

    def send(self, command: str) -> bool:
        if not self.is_connected():
            print(f"[ERROR] {self.name} is not connected")
            return False
        try:
            with self.lock:
                self.device.write((command + "\n").encode())
                self.device.flush()
            print(f"[SEND TO {self.name}] {command}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send to {self.name}: {e}")
            return False

    def read_line(self) -> Optional[str]:
        if not self.is_connected():
            return None
        try:
            with self.lock:
                line = self.device.readline().decode(errors="ignore").strip()
            return line if line else None
        except Exception as e:
            print(f"[ERROR] Failed reading from {self.name}: {e}")
            return None

    def close(self):
        try:
            if self.device and self.device.is_open:
                self.device.close()
                print(f"[OK] Closed {self.name}")
        except Exception:
            pass
