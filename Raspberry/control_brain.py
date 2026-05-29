
import time

from serial_device import SerialDevice
from sensors import SensorData
from config import (
    MEGA_PORT,
    UNO_PORT,
    BAUD_RATE,
    DEFAULT_SPEED,
    STAIRS_SPEED,
    AUTO_COMMAND_INTERVAL
)


class ControlBrain:
    """
    Apex Rover Control Brain V3

    This class talks to:
    - Arduino Mega: motors, jacks, sensors
    - Arduino UNO : camera stand stepper + servo

    Important V3 camera fix:
    - Python does NOT invert camera UP/DOWN anymore.
    - UNO V3 handles servo inversion using INVERT_SERVO_VERTICAL.
    - Python sends raw CAM commands exactly:
        CAM:LEFT
        CAM:RIGHT
        CAM:STOP
        CAM:UP
        CAM:DOWN
        CAM:CENTER
        CAM:ENABLE
        CAM:DISABLE
        CAM:STATUS
        CAM:DIR:LEFT
        CAM:DIR:RIGHT
        CAM:STEPS:200
        CAM:SPEED:1200
    """

    def __init__(self):
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)

        self.latest_sensor_data = SensorData()
        self.speed = DEFAULT_SPEED

        self.last_sensor_request_time = 0.0

        self.last_auto_move_command = None
        self.last_auto_move_time = 0.0

        self.last_auto_camera_command = None
        self.last_auto_camera_time = 0.0
        self.auto_camera_interval = 0.12

    # ========================================================
    # Connections
    # ========================================================

    def connect_all(self):
        print("Connecting Apex Rover controllers...")
        print("-----------------------------------")

        mega_ok = self.mega.connect()
        uno_ok = self.uno.connect()

        print("-----------------------------------")
        print("[READY] Mega controller ready" if mega_ok else "[WARNING] Mega controller not ready")
        print("[READY] UNO camera controller ready" if uno_ok else "[WARNING] UNO camera controller not ready")
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

    # ========================================================
    # Mega Movement - manual direct
    # ========================================================

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
        """
        Used by autonomous modes only.
        Manual keyboard commands do not use this throttle.
        """

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
        self.mega.send("MODE:NORMAL")

    def mode_climb(self):
        self.mega.send("MODE:CLIMB")

    # ========================================================
    # Mega Jacks - direct and repeatable
    # ========================================================

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

    # ========================================================
    # UNO Camera - raw exact commands
    # ========================================================

    def send_uno_raw(self, command: str):
        """
        Send exact raw command to UNO V3.
        Use this for all camera stand commands.
        """

        command = command.strip()

        if not command:
            return

        self.uno.send(command)
        print(f"[UNO RAW] {command}")

    def send_camera_command(self, command: str):
        """
        Camera command bridge for vision decisions.

        Only sends known CAM commands.
        Python does not invert up/down here.
        UNO V3 owns servo inversion.
        """

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
            "CAM:DIR:RIGHT"
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
        """
        For autonomous object/stairs vision only.
        Prevents sending CAM:LEFT/CAM:RIGHT/CAM:STOP hundreds of times per second.
        Manual keys do not use this throttle.
        """

        command = command.strip().upper()
        now = time.time()

        same_command = command == self.last_auto_camera_command
        too_soon = (now - self.last_auto_camera_time) < self.auto_camera_interval

        if same_command and too_soon:
            return

        self.last_auto_camera_command = command
        self.last_auto_camera_time = now

        self.send_camera_command(command)

    # Convenience wrappers
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

        self.mega.close()
        self.uno.close()
