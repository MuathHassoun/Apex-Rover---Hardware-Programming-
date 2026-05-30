
import time
import requests

from sensors import SensorData, parse_sensor_data
from config import (
    DEFAULT_SPEED,
    STAIRS_SPEED,
    AUTO_COMMAND_INTERVAL,
    AUTO_CAMERA_COMMAND_INTERVAL,
    ESP32_BASE_URL,
    ESP32_HTTP_TIMEOUT,
)


class NetworkEndpoint:
    """
    Compatibility object so old main_brain.py can still call:
        self.control.mega.read_line()
        self.control.uno.read_line()

    In the new architecture Raspberry Pi does not read directly from
    Mega/UNO serial. It talks to ESP32 over Wi-Fi/HTTP.
    """

    def __init__(self, name: str, parent: "ControlBrain"):
        self.name = name
        self.parent = parent

    def connect(self) -> bool:
        return self.parent.check_esp32_connection()

    def send(self, command: str) -> bool:
        return self.parent.send_command(command)

    def read_line(self):
        return None

    def close(self):
        pass


class ControlBrain:
    """
    Apex Rover Control Brain - ESP32 WiFi Bridge Version

    New architecture:
        Raspberry Pi  -> Wi-Fi HTTP -> ESP32
        ESP32         -> Serial     -> Mega / UNO

    Raspberry Pi no longer needs direct USB serial to Mega/UNO.

    ESP32 endpoints expected:
        GET /get_status
            returns: {"mode":"MANUAL"} or {"mode":"AUTO"}

        GET /command?cmd=...
            sends command to ESP32, ESP32 routes it:
                CAM:* / ARM:* -> UNO
                FORWARD/SPEED/JACK/STOP/GET:SENSORS -> Mega

        GET /get_sensors
            returns: {"ok":true,"raw":"DATA:PITCH=...;ROLL=...;UF=...;UR=..."}
    """

    def __init__(self):
        self.latest_sensor_data = SensorData()
        self.speed = DEFAULT_SPEED

        self.last_sensor_request_time = 0.0

        self.last_auto_move_command = None
        self.last_auto_move_time = 0.0

        self.last_auto_camera_command = None
        self.last_auto_camera_time = 0.0
        self.auto_camera_interval = AUTO_CAMERA_COMMAND_INTERVAL

        self.last_known_system_mode = "MANUAL"

        # Compatibility handles used by main_brain.py
        self.mega = NetworkEndpoint("ESP32->Mega", self)
        self.uno = NetworkEndpoint("ESP32->UNO", self)

    # ========================================================
    # ESP32 HTTP helpers
    # ========================================================

    def check_esp32_connection(self) -> bool:
        try:
            r = requests.get(
                f"{ESP32_BASE_URL}/get_status",
                timeout=ESP32_HTTP_TIMEOUT,
            )
            if r.status_code == 200:
                data = r.json()
                self.last_known_system_mode = data.get("mode", "MANUAL")
                print(f"[OK] ESP32 bridge connected | mode={self.last_known_system_mode}")
                return True

            print(f"[WARNING] ESP32 status HTTP {r.status_code}: {r.text}")
            return False
        except Exception as e:
            print("[ERROR] Could not connect to ESP32 WiFi bridge")
            print(e)
            return False

    def get_system_mode(self) -> str:
        try:
            r = requests.get(
                f"{ESP32_BASE_URL}/get_status",
                timeout=ESP32_HTTP_TIMEOUT,
            )
            if r.status_code == 200:
                data = r.json()
                self.last_known_system_mode = data.get("mode", "MANUAL").upper()
                return self.last_known_system_mode
        except Exception:
            pass

        # Safe fallback: if Raspberry cannot reach ESP32, behave as MANUAL/idle.
        self.last_known_system_mode = "MANUAL"
        return "MANUAL"

    def is_auto_mode(self) -> bool:
        return self.get_system_mode() == "AUTO"

    def send_command(self, command: str, avoid_print: bool = False) -> bool:
        command = command.strip()
        if not command:
            return False

        try:
            r = requests.get(
                f"{ESP32_BASE_URL}/command",
                params={"cmd": command},
                timeout=ESP32_HTTP_TIMEOUT,
            )

            if r.status_code == 200:
                if not avoid_print:
                    print(f"[SEND VIA ESP32] {command}")
                return True

            if not avoid_print:
                print(f"[BLOCK/ERROR VIA ESP32] {command} -> {r.status_code}: {r.text}")
            return False

        except Exception as e:
            if not avoid_print:
                print(f"[ERROR] Failed to send via ESP32: {command}")
                print(e)
            return False

    # ========================================================
    # Connections
    # ========================================================

    def connect_all(self):
        print("Connecting Apex Rover through ESP32 WiFi bridge...")
        print("---------------------------------------------------")
        ok = self.check_esp32_connection()
        print("---------------------------------------------------")
        print("[READY] ESP32 bridge ready" if ok else "[WARNING] ESP32 bridge not ready")
        print("[INFO] Raspberry will control only when ESP32 mode is AUTO")
        print()

    # ========================================================
    # Sensors
    # ========================================================

    def update_sensor_data(self, data: SensorData):
        self.latest_sensor_data = data

    def request_sensors_every_second(self):
        now = time.time()

        if now - self.last_sensor_request_time < 1.0:
            return

        self.last_sensor_request_time = now

        try:
            r = requests.get(
                f"{ESP32_BASE_URL}/get_sensors",
                timeout=ESP32_HTTP_TIMEOUT,
            )
            if r.status_code != 200:
                print(f"[SENSOR ERROR] HTTP {r.status_code}: {r.text}")
                return

            raw = r.json().get("raw", "")
            if not raw:
                return

            print(f"[SENSORS RAW] {raw}")
            data = parse_sensor_data(raw)
            if data:
                self.update_sensor_data(data)

        except Exception as e:
            print("[SENSOR ERROR] Failed to get sensors from ESP32")
            print(e)

    def check_safety_before_movement(self) -> bool:
        data = self.latest_sensor_data

        if not data.is_safe_angle():
            print("[SAFETY STOP] Unsafe pitch/roll detected")
            print(f"Pitch={data.pitch:.2f}, Roll={data.roll:.2f}")
            self.stop()
            self.stop_all_jacks()
            return False

        return True

    def print_sensor_data(self):
        data = self.latest_sensor_data

        print()
        print("========== Latest Sensor Data ==========")
        print(f"Pitch              : {data.pitch:.2f} deg")
        print(f"Roll               : {data.roll:.2f} deg")
        print(f"Front Ultrasonic   : {data.front_ultrasonic:.2f} cm")
        print(f"Rear Ultrasonic    : {data.rear_ultrasonic:.2f} cm")
        print(f"Safe Angle         : {data.is_safe_angle()}")
        print(f"ESP32 System Mode  : {self.last_known_system_mode}")
        print("========================================")
        print()

    # ========================================================
    # Mega Movement
    # ========================================================

    def set_speed(self, speed: int):
        speed = max(0, min(100, int(speed)))
        self.speed = speed
        self.send_command(f"SPEED:{speed}")

    def forward(self):
        if self.check_safety_before_movement():
            self.send_command("FORWARD")

    def backward(self):
        if self.check_safety_before_movement():
            self.send_command("BACKWARD")

    def left(self):
        if self.check_safety_before_movement():
            self.send_command("LEFT")

    def right(self):
        if self.check_safety_before_movement():
            self.send_command("RIGHT")

    def stop(self):
        self.send_command("STOP")

    def apply_auto_move_command(self, command: str):
        command = command.strip().upper()

        if command.startswith("MOVE:"):
            command = command.replace("MOVE:", "")

        now = time.time()

        same_command = command == self.last_auto_move_command
        too_soon = (now - self.last_auto_move_time) < AUTO_COMMAND_INTERVAL

        if same_command and too_soon:
            return

        self.last_auto_move_command = command
        self.last_auto_move_time = now

        if command == "FORWARD":
            self.forward()
        elif command == "BACKWARD":
            self.backward()
        elif command == "LEFT":
            self.left()
        elif command == "RIGHT":
            self.right()
        elif command == "SLOW":
            self.set_speed(STAIRS_SPEED)
            self.forward()
        elif command == "STOP":
            self.stop()
        else:
            print(f"[WARNING] Unknown move command: {command}")

    # ========================================================
    # Mega Modes
    # ========================================================

    def mode_normal(self):
        self.send_command("MODE:NORMAL")

    def mode_climb(self):
        self.send_command("MODE:CLIMB")

    # ========================================================
    # Mega Jacks
    # ========================================================

    def front_jack_extend(self):
        self.send_command("JACK:FRONT:EXTEND")

    def front_jack_retract(self):
        self.send_command("JACK:FRONT:RETRACT")

    def front_jack_stop(self):
        self.send_command("JACK:FRONT:STOP")

    def rear_jack_extend(self):
        self.send_command("JACK:REAR:EXTEND")

    def rear_jack_retract(self):
        self.send_command("JACK:REAR:RETRACT")

    def rear_jack_stop(self):
        self.send_command("JACK:REAR:STOP")

    def stop_all_jacks(self):
        self.send_command("JACK:ALL:STOP")
        self.front_jack_stop()
        self.rear_jack_stop()

    # ========================================================
    # UNO Camera
    # ========================================================

    def send_uno_raw(self, command: str):
        command = command.strip()
        if not command:
            return
        self.send_command(command)
        print(f"[CAM RAW VIA ESP32] {command}")

    def send_camera_command(self, command: str):
        command = command.strip().upper()

        allowed_exact = [
            "CAM:LEFT",
            "CAM:RIGHT",
            "CAM:STOP",
            "CAM:UP",
            "CAM:DOWN",
            "CAM:CENTER",
            "CAM:ENABLE",
            "CAM:DISABLE",
            "CAM:STATUS",
            "CAM:START",
            "CAM:DIR:LEFT",
            "CAM:DIR:RIGHT",
        ]

        if command in allowed_exact:
            self.send_uno_raw(command)
            return

        if command.startswith("CAM:STEPS:"):
            self.send_uno_raw(command)
            return

        if command.startswith("CAM:SPEED:"):
            self.send_uno_raw(command)
            return

        print(f"[WARNING] Unknown camera command: {command}")

    def apply_auto_camera_command(self, command: str):
        command = command.strip().upper()
        now = time.time()

        same_command = command == self.last_auto_camera_command
        too_soon = (now - self.last_auto_camera_time) < self.auto_camera_interval

        if same_command and too_soon:
            return

        self.last_auto_camera_command = command
        self.last_auto_camera_time = now
        self.send_camera_command(command)

    def camera_left(self):
        self.send_uno_raw("CAM:LEFT")

    def camera_right(self):
        self.send_uno_raw("CAM:RIGHT")

    def camera_stop(self):
        self.send_uno_raw("CAM:STOP")

    def camera_up(self):
        self.send_uno_raw("CAM:UP")

    def camera_down(self):
        self.send_uno_raw("CAM:DOWN")

    def camera_center(self):
        self.send_uno_raw("CAM:CENTER")

    def camera_enable(self):
        self.send_uno_raw("CAM:ENABLE")

    def camera_disable(self):
        self.send_uno_raw("CAM:DISABLE")

    def camera_status(self):
        self.send_uno_raw("CAM:STATUS")

    def camera_speed(self, delay_micros: int):
        delay_micros = max(300, min(10000, int(delay_micros)))
        self.send_uno_raw(f"CAM:SPEED:{delay_micros}")

    def camera_steps(self, steps: int):
        steps = max(1, int(steps))
        self.send_uno_raw(f"CAM:STEPS:{steps}")

    def camera_dir_left(self):
        self.send_uno_raw("CAM:DIR:LEFT")

    def camera_dir_right(self):
        self.send_uno_raw("CAM:DIR:RIGHT")

    # ========================================================
    # Emergency / shutdown
    # ========================================================

    def emergency_stop(self):
        print("[EMERGENCY STOP] Stopping all systems")
        self.stop()
        self.stop_all_jacks()
        self.camera_stop()

    def shutdown(self):
        print("[CONTROL] Shutdown")
        try:
            self.emergency_stop()
        except Exception:
            pass
        time.sleep(0.3)

