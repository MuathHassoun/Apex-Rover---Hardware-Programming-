import serial
import time
import threading
from dataclasses import dataclass
from typing import Optional, Dict


# ============================================================
# Apex Rover Raspberry Pi Brain
# Raspberry Pi 4 Model B
#
# MEGA  -> Movement + Jacks + MPU6500 + Ultrasonics
# UNO   -> Camera Stepper + Servo
# ============================================================


# -----------------------------
# Serial Ports
# -----------------------------

MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"

BAUD_RATE = 9600


# -----------------------------
# Safety Limits
# -----------------------------

MAX_SAFE_PITCH = 25.0
MAX_SAFE_ROLL = 25.0

DEFAULT_SPEED = 60

FRONT_JACK_TARGET_CM = 10.0
REAR_JACK_TARGET_CM = 10.0

JACK_TOLERANCE_CM = 0.5


# -----------------------------
# Sensor Data Model
# -----------------------------

@dataclass
class SensorData:
    pitch: float = 0.0
    roll: float = 0.0
    front_ultrasonic: float = 0.0
    rear_ultrasonic: float = 0.0

    def is_safe_angle(self) -> bool:
        return abs(self.pitch) <= MAX_SAFE_PITCH and abs(self.roll) <= MAX_SAFE_ROLL


# -----------------------------
# Serial Device Class
# -----------------------------

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
                timeout=1,
                write_timeout=1
            )

            # Arduino resets when serial opens
            time.sleep(2)

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
            print(f"[ERROR] Failed to send to {self.name}")
            print(e)
            return False

    def read_line(self) -> Optional[str]:
        if not self.is_connected():
            return None

        try:
            with self.lock:
                line = self.device.readline().decode(errors="ignore").strip()

            if line:
                return line

            return None

        except Exception as e:
            print(f"[ERROR] Failed reading from {self.name}")
            print(e)
            return None

    def send_and_read(self, command: str, delay: float = 0.2) -> Optional[str]:
        self.send(command)
        time.sleep(delay)
        return self.read_line()

    def close(self):
        try:
            if self.device and self.device.is_open:
                self.device.close()
                print(f"[OK] Closed {self.name}")
        except Exception:
            pass


# -----------------------------
# Apex Rover Brain
# -----------------------------

class ApexRoverBrain:
    def __init__(self):
        self.mega = SerialDevice("Arduino Mega", MEGA_PORT, BAUD_RATE)
        self.uno = SerialDevice("Arduino UNO", UNO_PORT, BAUD_RATE)

        self.latest_sensor_data = SensorData()
        self.speed = DEFAULT_SPEED
        self.running = True

    # ========================================================
    # Connection
    # ========================================================

    def connect_all(self):
        print("Connecting Apex Rover controllers...")
        print("-----------------------------------")

        mega_ok = self.mega.connect()
        uno_ok = self.uno.connect()

        print("-----------------------------------")

        if mega_ok:
            print("[READY] Mega controller ready")
        else:
            print("[WARNING] Mega controller not ready")

        if uno_ok:
            print("[READY] UNO camera controller ready")
        else:
            print("[WARNING] UNO camera controller not ready")

        print()

    # ========================================================
    # Mega Commands: Movement
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

    # ========================================================
    # Mega Commands: Mode
    # ========================================================

    def mode_normal(self):
        self.mega.send("MODE:NORMAL")

    def mode_climb(self):
        self.mega.send("MODE:CLIMB")

    # ========================================================
    # Mega Commands: Jacks
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
    # UNO Commands: Camera
    # ========================================================

    def camera_left(self):
        self.uno.send("CAM:LEFT")

    def camera_right(self):
        self.uno.send("CAM:RIGHT")

    def camera_stop(self):
        self.uno.send("CAM:STOP")

    def camera_up(self):
        self.uno.send("CAM:UP")

    def camera_down(self):
        self.uno.send("CAM:DOWN")

    def camera_center(self):
        self.uno.send("CAM:CENTER")

    # ========================================================
    # Sensor Reading
    # ========================================================

    def request_sensors(self) -> Optional[SensorData]:
        """
        Expected Mega response format:

        DATA:PITCH=5.2;ROLL=-1.4;UF=8.5;UR=9.1

        PITCH = MPU6500 pitch
        ROLL  = MPU6500 roll
        UF    = Front ultrasonic distance in cm
        UR    = Rear ultrasonic distance in cm
        """

        response = self.mega.send_and_read("GET:SENSORS", delay=0.3)

        if response is None:
            print("[WARNING] No sensor response from Mega")
            return None

        print(f"[MEGA RESPONSE] {response}")

        data = self.parse_sensor_data(response)

        if data:
            self.latest_sensor_data = data

        return data

    def parse_sensor_data(self, response: str) -> Optional[SensorData]:
        if not response.startswith("DATA:"):
            print("[ERROR] Invalid sensor format")
            return None

        try:
            response = response.replace("DATA:", "")
            parts = response.split(";")

            values: Dict[str, float] = {}

            for part in parts:
                key, value = part.split("=")
                values[key.strip()] = float(value.strip())

            return SensorData(
                pitch=values.get("PITCH", 0.0),
                roll=values.get("ROLL", 0.0),
                front_ultrasonic=values.get("UF", 0.0),
                rear_ultrasonic=values.get("UR", 0.0)
            )

        except Exception as e:
            print("[ERROR] Failed to parse sensor data")
            print(e)
            return None

    def print_sensor_data(self):
        data = self.request_sensors()

        if data is None:
            return

        print()
        print("========== Sensor Data ==========")
        print(f"Pitch              : {data.pitch:.2f} deg")
        print(f"Roll               : {data.roll:.2f} deg")
        print(f"Front Ultrasonic   : {data.front_ultrasonic:.2f} cm")
        print(f"Rear Ultrasonic    : {data.rear_ultrasonic:.2f} cm")
        print(f"Safe Angle         : {data.is_safe_angle()}")
        print("=================================")
        print()

    # ========================================================
    # Safety
    # ========================================================

    def check_safety_before_movement(self) -> bool:
        data = self.request_sensors()

        if data is None:
            print("[WARNING] Cannot verify sensors. Movement command still sent manually.")
            return True

        if not data.is_safe_angle():
            print("[SAFETY STOP] Unsafe pitch/roll detected")
            print(f"Pitch={data.pitch:.2f}, Roll={data.roll:.2f}")
            self.stop()
            self.stop_all_jacks()
            return False

        return True

    def emergency_stop(self):
        print("[EMERGENCY STOP] Stopping all systems")

        self.stop()
        self.stop_all_jacks()
        self.camera_stop()

    # ========================================================
    # Semi-Automatic Jack Control
    # ========================================================

    def extend_front_jack_to_target(self, target_cm: float = FRONT_JACK_TARGET_CM):
        print(f"[AUTO] Extending front jack to {target_cm} cm")

        self.front_jack_extend()

        while True:
            data = self.request_sensors()

            if data is None:
                print("[ERROR] Sensor failed. Stopping front jack.")
                self.front_jack_stop()
                break

            distance = data.front_ultrasonic
            print(f"[FRONT JACK] Distance = {distance:.2f} cm")

            if not data.is_safe_angle():
                print("[SAFETY] Unsafe angle. Stopping front jack.")
                self.front_jack_stop()
                break

            if distance >= target_cm - JACK_TOLERANCE_CM:
                print("[AUTO] Front jack target reached")
                self.front_jack_stop()
                break

            time.sleep(0.2)

    def extend_rear_jack_to_target(self, target_cm: float = REAR_JACK_TARGET_CM):
        print(f"[AUTO] Extending rear jack to {target_cm} cm")

        self.rear_jack_extend()

        while True:
            data = self.request_sensors()

            if data is None:
                print("[ERROR] Sensor failed. Stopping rear jack.")
                self.rear_jack_stop()
                break

            distance = data.rear_ultrasonic
            print(f"[REAR JACK] Distance = {distance:.2f} cm")

            if not data.is_safe_angle():
                print("[SAFETY] Unsafe angle. Stopping rear jack.")
                self.rear_jack_stop()
                break

            if distance >= target_cm - JACK_TOLERANCE_CM:
                print("[AUTO] Rear jack target reached")
                self.rear_jack_stop()
                break

            time.sleep(0.2)

    # ========================================================
    # Future Autonomous Climb Placeholder
    # ========================================================

    def simple_climb_sequence(self):
        """
        Basic safe climb test sequence.

        This is NOT full autonomous stair climbing yet.
        It is only a structured test sequence.
        """

        print("[CLIMB] Starting simple climb sequence")
        self.mode_climb()
        self.set_speed(40)

        self.print_sensor_data()

        print("[CLIMB] Extending front jack")
        self.extend_front_jack_to_target(10.0)

        time.sleep(0.5)

        print("[CLIMB] Moving forward slowly")
        self.forward()
        time.sleep(1.0)
        self.stop()

        time.sleep(0.5)

        print("[CLIMB] Extending rear jack")
        self.extend_rear_jack_to_target(10.0)

        time.sleep(0.5)

        print("[CLIMB] Stopping climb sequence")
        self.stop()
        self.stop_all_jacks()
        self.mode_normal()

    # ========================================================
    # Shutdown
    # ========================================================

    def shutdown(self):
        print("[SYSTEM] Safe shutdown")

        self.running = False

        self.emergency_stop()

        time.sleep(0.3)

        self.mega.close()
        self.uno.close()


# -----------------------------
# Menu
# -----------------------------

def print_menu():
    print()
    print("=================================================")
    print("              APEX ROVER BRAIN")
    print("=================================================")
    print()
    print("Movement - Mega")
    print("  f              -> Forward")
    print("  b              -> Backward")
    print("  l              -> Left")
    print("  r              -> Right")
    print("  x              -> Stop")
    print("  speed 70       -> Set speed 0-100")
    print()
    print("Modes - Mega")
    print("  normal         -> Normal mode")
    print("  climb          -> Climb mode")
    print()
    print("Jacks - Mega")
    print("  fe             -> Front jack extend")
    print("  fr             -> Front jack retract")
    print("  fs             -> Front jack stop")
    print("  re             -> Rear jack extend")
    print("  rr             -> Rear jack retract")
    print("  rs             -> Rear jack stop")
    print("  jstop          -> Stop both jacks")
    print()
    print("Auto Jacks")
    print("  afe            -> Auto front jack extend to 10 cm")
    print("  are            -> Auto rear jack extend to 10 cm")
    print()
    print("Camera - UNO")
    print("  cl             -> Camera left")
    print("  cr             -> Camera right")
    print("  cs             -> Camera stop")
    print("  cu             -> Camera up")
    print("  cd             -> Camera down")
    print("  cc             -> Camera center")
    print()
    print("Sensors")
    print("  sensor         -> Read MPU6500 + Ultrasonics")
    print()
    print("Autonomous Test")
    print("  testclimb      -> Simple climb test sequence")
    print()
    print("Safety")
    print("  estop          -> Emergency stop")
    print()
    print("Other")
    print("  menu           -> Show menu")
    print("  q              -> Quit")
    print()
    print("=================================================")
    print()


# -----------------------------
# Main Program
# -----------------------------

def main():
    brain = ApexRoverBrain()
    brain.connect_all()

    brain.set_speed(DEFAULT_SPEED)

    print_menu()

    while brain.running:
        try:
            command = input("ApexRover> ").strip().lower()

            # -------------------------
            # Movement
            # -------------------------

            if command == "f":
                brain.forward()

            elif command == "b":
                brain.backward()

            elif command == "l":
                brain.left()

            elif command == "r":
                brain.right()

            elif command == "x":
                brain.stop()

            elif command.startswith("speed"):
                try:
                    speed = int(command.split()[1])
                    brain.set_speed(speed)
                except Exception:
                    print("[ERROR] Use: speed 70")

            # -------------------------
            # Modes
            # -------------------------

            elif command == "normal":
                brain.mode_normal()

            elif command == "climb":
                brain.mode_climb()

            # -------------------------
            # Jacks
            # -------------------------

            elif command == "fe":
                brain.front_jack_extend()

            elif command == "fr":
                brain.front_jack_retract()

            elif command == "fs":
                brain.front_jack_stop()

            elif command == "re":
                brain.rear_jack_extend()

            elif command == "rr":
                brain.rear_jack_retract()

            elif command == "rs":
                brain.rear_jack_stop()

            elif command == "jstop":
                brain.stop_all_jacks()

            # -------------------------
            # Auto Jacks
            # -------------------------

            elif command == "afe":
                brain.extend_front_jack_to_target(10.0)

            elif command == "are":
                brain.extend_rear_jack_to_target(10.0)

            # -------------------------
            # Camera
            # -------------------------

            elif command == "cl":
                brain.camera_left()

            elif command == "cr":
                brain.camera_right()

            elif command == "cs":
                brain.camera_stop()

            elif command == "cu":
                brain.camera_up()

            elif command == "cd":
                brain.camera_down()

            elif command == "cc":
                brain.camera_center()

            # -------------------------
            # Sensors
            # -------------------------

            elif command == "sensor":
                brain.print_sensor_data()

            # -------------------------
            # Autonomous Test
            # -------------------------

            elif command == "testclimb":
                brain.simple_climb_sequence()

            # -------------------------
            # Safety
            # -------------------------

            elif command == "estop":
                brain.emergency_stop()

            # -------------------------
            # Other
            # -------------------------

            elif command == "menu":
                print_menu()

            elif command == "q":
                brain.shutdown()
                break

            elif command == "":
                continue

            else:
                print("[ERROR] Unknown command. Type 'menu' to show commands.")

        except KeyboardInterrupt:
            print()
            brain.shutdown()
            break

        except Exception as e:
            print("[ERROR] Main loop error")
            print(e)
            brain.emergency_stop()


if __name__ == "__main__":
    main()
