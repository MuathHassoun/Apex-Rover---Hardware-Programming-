import time
from serial_device import SerialDevice
from sensors import SensorData
from config import (
    MEGA_PORT, UNO_PORT, BAUD_RATE, DEFAULT_SPEED, STAIRS_SPEED,
    INVERT_CAMERA_VERTICAL, AUTO_COMMAND_INTERVAL
)


class ControlBrain:
    """
    Fast control brain.

    V2 fixes:
    - Manual movement sends every time you press the key.
    - Camera movement sends every time you press the key.
    - UP/DOWN camera inversion is fixed here.
    - Autonomous repeated commands are only lightly throttled.
    """

    def __init__(self):
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)
        self.latest_sensor_data = SensorData()
        self.speed = DEFAULT_SPEED
        self.last_sensor_request_time = 0.0
        self.last_auto_move_command = None
        self.last_auto_move_time = 0.0

    def connect_all(self):
        print("Connecting Apex Rover controllers...")
        print("-----------------------------------")
        mega_ok = self.mega.connect()
        uno_ok = self.uno.connect()
        print("-----------------------------------")
        print("[READY] Mega controller ready" if mega_ok else "[WARNING] Mega controller not ready")
        print("[READY] UNO camera controller ready" if uno_ok else "[WARNING] UNO camera controller not ready")
        print()

    # -----------------------------
    # Sensors
    # -----------------------------

    def update_sensor_data(self, data: SensorData):
        self.latest_sensor_data = data

    def request_sensors_every_second(self):
        now = time.time()
        if now - self.last_sensor_request_time < 1.0:
            return
        self.last_sensor_request_time = now
        self.mega.send("GET:SENSORS")

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
        print("========================================")
        print()

    # -----------------------------
    # Movement: manual direct
    # -----------------------------

    def set_speed(self, speed: int):
        speed = max(0, min(100, speed))
        self.speed = speed
        self.mega.send(f"SPEED:{speed}")

    def forward(self):
        if self.check_safety_before_movement():
            self.mega.send("FORWARD")

    def backward(self):
        if self.check_safety_before_movement():
            self.mega.send("BACKWARD")

    def left(self):
        if self.check_safety_before_movement():
            self.mega.send("LEFT")

    def right(self):
        if self.check_safety_before_movement():
            self.mega.send("RIGHT")

    def stop(self):
        self.mega.send("STOP")

    def apply_auto_move_command(self, command: str):
        """Used by autonomous modes only."""
        command = command.strip().upper()
        if command.startswith("MOVE:"):
            command = command.replace("MOVE:", "")

        now = time.time()
        if command == self.last_auto_move_command and (now - self.last_auto_move_time) < AUTO_COMMAND_INTERVAL:
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

    # -----------------------------
    # Mega modes
    # -----------------------------

    def mode_normal(self):
        self.mega.send("MODE:NORMAL")

    def mode_climb(self):
        self.mega.send("MODE:CLIMB")

    # -----------------------------
    # Jacks: direct and repeatable
    # -----------------------------

    def front_jack_extend(self):
        self.mega.send("JACK:FRONT:EXTEND")

    def front_jack_retract(self):
        self.mega.send("JACK:FRONT:RETRACT")

    def front_jack_stop(self):
        self.mega.send("JACK:FRONT:STOP")

    def rear_jack_extend(self):
        self.mega.send("JACK:REAR:EXTEND")

    def rear_jack_retract(self):
        self.mega.send("JACK:REAR:RETRACT")

    def rear_jack_stop(self):
        self.mega.send("JACK:REAR:STOP")

    def stop_all_jacks(self):
        self.front_jack_stop()
        self.rear_jack_stop()

    # -----------------------------
    # Camera to UNO: direct and repeatable
    # -----------------------------

    def camera_left(self):
        self.uno.send("CAM:LEFT")

    def camera_right(self):
        self.uno.send("CAM:RIGHT")

    def camera_stop(self):
        self.uno.send("CAM:STOP")

    def camera_up(self):
        # Fix reversed physical movement
        self.uno.send("CAM:DOWN" if INVERT_CAMERA_VERTICAL else "CAM:UP")

    def camera_down(self):
        # Fix reversed physical movement
        self.uno.send("CAM:UP" if INVERT_CAMERA_VERTICAL else "CAM:DOWN")

    def camera_center(self):
        self.uno.send("CAM:CENTER")

    def send_camera_command(self, command: str):
        """For vision decisions. Supports CAM:LEFT/CAM:RIGHT/CAM:STOP/CAM:CENTER."""
        command = command.strip().upper()
        if command == "CAM:LEFT":
            self.camera_left()
        elif command == "CAM:RIGHT":
            self.camera_right()
        elif command == "CAM:UP":
            self.camera_up()
        elif command == "CAM:DOWN":
            self.camera_down()
        elif command == "CAM:CENTER":
            self.camera_center()
        elif command == "CAM:STOP":
            self.camera_stop()
        else:
            print(f"[WARNING] Unknown camera command: {command}")

    # -----------------------------
    # Emergency / shutdown
    # -----------------------------

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
        self.mega.close()
        self.uno.close()
