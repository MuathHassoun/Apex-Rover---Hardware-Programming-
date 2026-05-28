import time
import cv2

from control_brain import ControlBrain
from vision_brain import VisionBrain
from sensors import parse_sensor_data
from config import (
    CAPTURE_DELAY,
    DEFAULT_SPEED,
    STAIRS_SPEED,
    CLIMB_SPEED,
    MODE_IDLE,
    MODE_MANUAL,
    MODE_OBJECT,
    MODE_STAIRS,
    MODE_CLIMB_ASSIST
)


class ApexMainBrain:
    """
    Apex Rover Main Brain V4

    This file connects:
    - ControlBrain: Mega + UNO serial commands
    - VisionBrain : camera processing
    - Sensors     : data from Mega

    V4 camera fix:
    - Camera commands are sent raw to UNO V3.
    - Python does NOT invert CAM:UP / CAM:DOWN.
    - UNO V3 owns servo inversion using INVERT_SERVO_VERTICAL.
    - Manual camera keys are direct and repeatable:
        a = CAM:LEFT
        d = CAM:RIGHT
        v = CAM:STOP
        w = CAM:UP
        s = CAM:DOWN
        c = CAM:CENTER
    """

    def __init__(self):
        self.control = ControlBrain()
        self.vision = VisionBrain()

        self.mode = MODE_MANUAL
        self.running = True

        self.last_climb_state = "NONE"
        self.last_debug_print_time = 0.0

    # ========================================================
    # Start / Info
    # ========================================================

    def start(self):
        self.control.connect_all()

        if not self.vision.open_camera():
            self.shutdown()
            return

        self.control.set_speed(DEFAULT_SPEED)
        self.control.camera_center()

        self.print_start_info()
        self.run_loop()

    def print_start_info(self):
        print()
        print("=================================================")
        print("       APEX ROVER MODULAR MAIN BRAIN V4")
        print("=================================================")
        print()
        print("Modes:")
        print("  0 = IDLE")
        print("  m = MANUAL")
        print("  1 = OBJECT")
        print("  2 = STAIRS")
        print("  3 = CLIMB_ASSIST")
        print()
        print("Movement:")
        print("  f = forward")
        print("  b = backward")
        print("  l = left")
        print("  r = right")
        print("  x = stop")
        print()
        print("Camera to UNO V3:")
        print("  a = CAM:LEFT")
        print("  d = CAM:RIGHT")
        print("  v = CAM:STOP")
        print("  w = CAM:UP")
        print("  s = CAM:DOWN")
        print("  c = CAM:CENTER")
        print("  e = CAM:ENABLE")
        print("  y = CAM:DISABLE")
        print("  t = CAM:STATUS")
        print("  [ = CAM:DIR:LEFT + CAM:STEPS:100")
        print("  ] = CAM:DIR:RIGHT + CAM:STEPS:100")
        print()
        print("Jacks:")
        print("  g = front extend")
        print("  h = front retract")
        print("  n = front stop")
        print("  u = rear extend")
        print("  j = rear retract")
        print("  k = rear stop")
        print("  z = stop all jacks")
        print()
        print("Other:")
        print("  p = print sensors")
        print("  q = quit")
        print()
        print("CLIMB_ASSIST uses camera + MPU6500 + ultrasonics + jacks.")
        print("=================================================")
        print()

    # ========================================================
    # Incoming Mega / ESP32
    # ========================================================

    def handle_incoming_mega_lines(self):
        """
        Drain available Mega lines so serial buffer does not lag.

        Mega may send:
            DATA:PITCH=5.2;ROLL=-1.4;UF=8.5;UR=9.1

        ESP32 forwarded through Mega may send:
            MODE:OBJECT
            MODE:STAIRS
            MODE:CLIMB_ASSIST
            MODE:MANUAL
            MODE:IDLE
            TARGET:red
            CMD:STOP
        """

        for _ in range(8):
            line = self.control.mega.read_line()

            if not line:
                break

            print(f"[FROM MEGA/ESP32] {line}")

            if line.startswith("DATA:"):
                data = parse_sensor_data(line)

                if data:
                    self.control.update_sensor_data(data)

                continue

            if line.startswith("MODE:"):
                new_mode = line.replace("MODE:", "").strip().upper()

                if new_mode in [MODE_IDLE, MODE_MANUAL, MODE_OBJECT, MODE_STAIRS, MODE_CLIMB_ASSIST]:
                    self.change_mode(new_mode)
                else:
                    print(f"[WARNING] Unknown mode: {new_mode}")

                continue

            if line.startswith("TARGET:"):
                new_color = line.replace("TARGET:", "").strip().lower()
                self.vision.set_target_color(new_color)
                continue

            if line.startswith("CMD:"):
                cmd = line.replace("CMD:", "").strip().upper()

                if cmd == "STOP":
                    self.control.emergency_stop()
                    self.change_mode(MODE_IDLE)

                continue

    def handle_incoming_uno_lines(self):
        """
        Read UNO ACK/status lines without blocking.
        This helps verify that camera commands reach UNO.
        """

        for _ in range(5):
            line = self.control.uno.read_line()

            if not line:
                break

            print(f"[FROM UNO] {line}")

    # ========================================================
    # Mode Control
    # ========================================================

    def change_mode(self, new_mode: str):
        self.mode = new_mode
        print(f"[BRAIN] Mode changed to: {self.mode}")

        if self.mode == MODE_IDLE:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()

        elif self.mode == MODE_MANUAL:
            self.control.stop()
            self.control.camera_center()

        elif self.mode == MODE_OBJECT:
            self.control.set_speed(DEFAULT_SPEED)
            self.control.camera_center()

        elif self.mode == MODE_STAIRS:
            self.control.set_speed(STAIRS_SPEED)
            # One camera down command when entering mode.
            self.control.camera_down()

        elif self.mode == MODE_CLIMB_ASSIST:
            self.control.set_speed(CLIMB_SPEED)
            self.control.mode_climb()
            # One camera down command when entering mode.
            self.control.camera_down()

    # ========================================================
    # Climb Assist - Autonomous stairs logic
    # ========================================================

    def climb_assist_step(self, stairs):
        """
        Auto up/down stairs decision using:
        - Camera stairs detection
        - MPU6500 pitch/roll
        - Ultrasonic front/rear
        - Motors
        - Front/rear jacks
        """

        data = self.control.latest_sensor_data

        pitch = data.pitch
        roll = data.roll
        front_distance = data.front_ultrasonic
        rear_distance = data.rear_ultrasonic

        # ==========================================
        # 1. Emergency balance safety
        # ==========================================

        if abs(roll) > 25:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            return "DANGER_ROLL_STOP"

        if abs(pitch) > 30:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            return "DANGER_PITCH_STOP"

        # ==========================================
        # 2. Camera does not see stairs
        # ==========================================

        if not stairs["stairs_found"]:
            self.control.stop()
            # Do not keep spinning stepper. Servo down is one small tilt command.
            self.control.apply_auto_camera_command("CAM:DOWN")
            return "NO_STAIRS_WAIT"

        # ==========================================
        # 3. Too close to obstacle / stair
        # ==========================================

        if front_distance > 0 and front_distance < 5:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            return "FRONT_TOO_CLOSE_STOP"

        if rear_distance > 0 and rear_distance < 4:
            self.control.stop()
            self.control.stop_all_jacks()
            return "REAR_TOO_CLOSE_STOP"

        # ==========================================
        # 4. Correct left/right alignment using camera
        # ==========================================

        move_cmd, vision_state = self.vision.decide_stairs_action(stairs, data)

        if vision_state == "ALIGN_LEFT_TO_STAIRS":
            self.control.set_speed(30)
            self.control.left()
            return "TURN_LEFT_ALIGN_STAIRS"

        if vision_state == "ALIGN_RIGHT_TO_STAIRS":
            self.control.set_speed(30)
            self.control.right()
            return "TURN_RIGHT_ALIGN_STAIRS"

        if vision_state == "CORRECT_TILT_LEFT":
            self.control.set_speed(25)
            self.control.left()
            return "CAMERA_TILT_LEFT_CORRECTION"

        if vision_state == "CORRECT_TILT_RIGHT":
            self.control.set_speed(25)
            self.control.right()
            return "CAMERA_TILT_RIGHT_CORRECTION"

        # ==========================================
        # 5. MPU6500 balance decisions
        # ==========================================

        if pitch > 15 and pitch <= 25:
            self.control.stop()

            self.control.front_jack_extend()
            time.sleep(0.25)
            self.control.front_jack_stop()

            return "FRONT_JACK_BALANCE_ASSIST"

        if pitch < -10 and pitch >= -25:
            self.control.stop()

            self.control.rear_jack_extend()
            time.sleep(0.25)
            self.control.rear_jack_stop()

            return "REAR_JACK_BALANCE_ASSIST"

        if roll > 12:
            self.control.stop()
            self.control.set_speed(25)
            self.control.left()
            time.sleep(0.2)
            self.control.stop()
            return "ROLL_RIGHT_CORRECTION"

        if roll < -12:
            self.control.stop()
            self.control.set_speed(25)
            self.control.right()
            time.sleep(0.2)
            self.control.stop()
            return "ROLL_LEFT_CORRECTION"

        # ==========================================
        # 6. Safe forward climb
        # ==========================================

        self.control.set_speed(35)
        self.control.forward()

        return "BALANCED_FORWARD_SLOW"

    # ========================================================
    # Debug
    # ========================================================

    def print_climb_debug_every_second(self, state):
        now = time.time()

        if now - self.last_debug_print_time < 1.0:
            return

        self.last_debug_print_time = now

        data = self.control.latest_sensor_data

        print(
            f"[CLIMB] state={state} "
            f"pitch={data.pitch:.2f} roll={data.roll:.2f} "
            f"UF={data.front_ultrasonic:.2f} UR={data.rear_ultrasonic:.2f}"
        )

    # ========================================================
    # Main Loop
    # ========================================================

    def run_loop(self):
        while self.running:
            self.handle_incoming_mega_lines()
            self.handle_incoming_uno_lines()
            self.control.request_sensors_every_second()

            ret, frame = self.vision.read_frame()

            if not ret:
                print("[WARNING] Failed to capture frame")
                self.handle_keyboard()
                time.sleep(0.05)
                continue

            mask = None
            data = self.control.latest_sensor_data

            # ====================================================
            # OBJECT MODE
            # ====================================================

            if self.mode == MODE_OBJECT:
                detection, mask = self.vision.detect_color_object(frame)
                move_cmd, cam_cmd, state = self.vision.decide_object_action(detection)

                self.control.apply_auto_move_command(move_cmd)
                self.control.apply_auto_camera_command(cam_cmd)

                debug_frame = self.vision.draw_object_debug(
                    frame, detection, move_cmd, cam_cmd, state,
                    self.mode, self.control.speed, data
                )

            # ====================================================
            # STAIRS MODE
            # ====================================================

            elif self.mode == MODE_STAIRS:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)
                move_cmd, state = self.vision.decide_stairs_action(stairs, data)

                self.control.apply_auto_move_command(move_cmd)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, move_cmd, state,
                    self.mode, self.control.speed, data
                )

                mask = edges

            # ====================================================
            # CLIMB ASSIST MODE
            # ====================================================

            elif self.mode == MODE_CLIMB_ASSIST:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)

                state = self.climb_assist_step(stairs)
                self.last_climb_state = state
                self.print_climb_debug_every_second(state)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, "AUTO", state,
                    self.mode, self.control.speed, data
                )

                mask = edges

            # ====================================================
            # IDLE / MANUAL
            # ====================================================

            else:
                debug_frame = self.vision.draw_idle_debug(
                    frame, self.mode, self.control.speed, data
                )

            cv2.imshow("Apex Rover Modular Brain V4", debug_frame)

            if mask is not None:
                cv2.imshow("Vision Mask / Edges", mask)

            self.handle_keyboard()
            time.sleep(CAPTURE_DELAY)

        self.shutdown()

    # ========================================================
    # Keyboard
    # ========================================================

    def handle_keyboard(self):
        key = cv2.waitKey(1) & 0xFF

        if key == 255:
            return

        # ----------------------------
        # Modes
        # ----------------------------

        if key == ord("q"):
            self.running = False

        elif key == ord("0"):
            self.change_mode(MODE_IDLE)

        elif key == ord("m"):
            self.change_mode(MODE_MANUAL)

        elif key == ord("1"):
            self.change_mode(MODE_OBJECT)

        elif key == ord("2"):
            self.change_mode(MODE_STAIRS)

        elif key == ord("3"):
            self.change_mode(MODE_CLIMB_ASSIST)

        # ----------------------------
        # Manual movement to Mega
        # ----------------------------

        elif key == ord("f"):
            self.control.forward()

        elif key == ord("b"):
            self.control.backward()

        elif key == ord("l"):
            self.control.left()

        elif key == ord("r"):
            self.control.right()

        elif key == ord("x"):
            self.control.stop()

        # ----------------------------
        # Camera to UNO V3 - direct raw
        # ----------------------------

        elif key == ord("w"):
            self.control.camera_up()

        elif key == ord("s"):
            self.control.camera_down()

        elif key == ord("a"):
            self.control.camera_left()

        elif key == ord("d"):
            self.control.camera_right()

        elif key == ord("v"):
            self.control.camera_stop()

        elif key == ord("c"):
            self.control.camera_center()

        elif key == ord("e"):
            self.control.camera_enable()

        elif key == ord("y"):
            self.control.camera_disable()

        elif key == ord("t"):
            self.control.camera_status()

        elif key == ord("["):
            self.control.camera_dir_left()
            time.sleep(0.02)
            self.control.camera_steps(100)

        elif key == ord("]"):
            self.control.camera_dir_right()
            time.sleep(0.02)
            self.control.camera_steps(100)

        # ----------------------------
        # Jacks to Mega
        # ----------------------------

        elif key == ord("g"):
            self.control.front_jack_extend()

        elif key == ord("h"):
            self.control.front_jack_retract()

        elif key == ord("n"):
            self.control.front_jack_stop()

        elif key == ord("u"):
            self.control.rear_jack_extend()

        elif key == ord("j"):
            self.control.rear_jack_retract()

        elif key == ord("k"):
            self.control.rear_jack_stop()

        elif key == ord("z"):
            self.control.stop_all_jacks()

        # ----------------------------
        # Sensors
        # ----------------------------

        elif key == ord("p"):
            self.control.print_sensor_data()

    # ========================================================
    # Shutdown
    # ========================================================

    def shutdown(self):
        print("[SYSTEM] Shutting down modular brain V4")

        try:
            self.control.shutdown()
        except Exception:
            pass

        try:
            self.vision.close()
        except Exception:
            pass


def main():
    brain = ApexMainBrain()

    try:
        brain.start()

    except KeyboardInterrupt:
        print()
        brain.shutdown()

    except Exception as e:
        print("[ERROR] Fatal main brain error")
        print(e)
        brain.shutdown()


if __name__ == "__main__":
    main()
