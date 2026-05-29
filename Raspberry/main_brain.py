
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
    VISION_LOST_FORWARD_SPEED,
    YELLOW_LOST_FORWARD_MAX_SEC,
    NO_STAIRS_FORWARD_MAX_SEC,
    MODE_IDLE,
    MODE_MANUAL,
    MODE_OBJECT,
    MODE_STAIRS,
    MODE_CLIMB_ASSIST,
    AUTO_START_CLIMB_UP,
    AUTO_ARM_DELAY_SEC,
    CLIMB_ROLL_DANGER,
    CLIMB_PITCH_DANGER,
    CLIMB_FRONT_ON_STEP_PITCH,
    AUTO_CLIMB_ALIGN_SPEED,
    AUTO_CLIMB_FORWARD_SPEED,
    AUTO_CLIMB_JACK_DRIVE_SPEED,
    REAR_JACK_USE_ULTRASONIC,
    REAR_JACK_EXTEND_STOP_WHEN_LESS_EQUAL,
    REAR_JACK_EXTEND_TARGET_CM,
    REAR_JACK_EXTEND_MAX_SEC,
    REAR_JACK_RETRACT_STOP_WHEN_GREATER_EQUAL,
    REAR_JACK_RETRACT_TARGET_CM,
    REAR_JACK_RETRACT_MAX_SEC,
    DRIVE_WITH_REAR_JACK_SEC,
    RECOVER_FORWARD_SEC,
    LEVEL_PITCH_ABS,
    LEVEL_ROLL_ABS,
    TOP_LEVEL_TIME_SEC,
    NO_STAIRS_TOP_TIME_SEC
)


class ApexMainBrain:
    """
    Apex Rover Main Brain - Auto Climb UP V1

    This version is for UP stairs only.

    Manual observation converted to auto logic:
    - During climbing up, we keep driving forward along the yellow path.
    - When the front part of the robot is already on the stair, pitch rises.
    - Then we use ONLY the rear linear actuator/jack.
    - Rear jack extends for enough time or until rear ultrasonic target is reached.
    - Robot drives forward while rear jack is still supporting/lifting.
    - Then rear jack retracts because it can hit the lower stair part.
    - Repeat the same process for the next stair.

    Important:
    - Front jack is NOT used for climbing up in this version.
    - Both jacks may be used later for DOWN stairs, but not now.
    """

    PHASE_ARMING = "ARMING"
    PHASE_SEARCH_ALIGN = "SEARCH_ALIGN_YELLOW"
    PHASE_FORWARD = "FORWARD_ON_YELLOW"
    PHASE_REAR_JACK_EXTEND = "REAR_JACK_EXTEND"
    PHASE_DRIVE_WITH_REAR_JACK = "DRIVE_WITH_REAR_JACK"
    PHASE_REAR_JACK_RETRACT = "REAR_JACK_RETRACT"
    PHASE_RECOVER_FORWARD = "RECOVER_FORWARD"
    PHASE_TOP_REACHED = "TOP_REACHED_STOP"
    PHASE_DANGER_STOP = "DANGER_STOP"

    def __init__(self):
        self.control = ControlBrain()
        self.vision = VisionBrain()

        self.mode = MODE_CLIMB_ASSIST if AUTO_START_CLIMB_UP else MODE_MANUAL
        self.running = True

        self.last_climb_state = "NONE"
        self.last_debug_print_time = 0.0

        # Auto climb state machine variables
        self.auto_arm_until = time.time() + AUTO_ARM_DELAY_SEC
        self.climb_phase = self.PHASE_ARMING
        self.phase_start_time = time.time()
        self.had_started_climb = False
        self.level_start_time = None
        self.no_stairs_since = None
        self.rear_jack_cycle_count = 0

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
        print("       APEX ROVER - AUTO CLIMB UP V1")
        print("=================================================")
        print()
        print("Startup mode:", self.mode)
        print("Auto start climb up:", AUTO_START_CLIMB_UP)
        print()
        print("Modes:")
        print("  0 = IDLE")
        print("  m = MANUAL")
        print("  1 = OBJECT")
        print("  2 = STAIRS")
        print("  3 = CLIMB_ASSIST / AUTO CLIMB UP")
        print()
        print("Movement:")
        print("  f = forward")
        print("  b = backward")
        print("  l = left")
        print("  r = right")
        print("  x = stop")
        print()
        print("Camera:")
        print("  a = CAM:LEFT")
        print("  d = CAM:RIGHT")
        print("  v = CAM:STOP")
        print("  w = CAM:UP")
        print("  s = CAM:DOWN")
        print("  c = CAM:CENTER")
        print("  t = CAM:STATUS")
        print()
        print("Jacks manual test:")
        print("  g = front extend")
        print("  h = front retract")
        print("  n = front stop")
        print("  u = rear extend")
        print("  j = rear retract")
        print("  k = rear stop")
        print("  z = stop all jacks")
        print()
        print("Auto UP logic:")
        print("  Follow yellow path -> forward -> rear jack extend -> drive -> rear jack retract -> repeat")
        print("=================================================")
        print()

    # ========================================================
    # Auto climb state helpers
    # ========================================================

    def reset_auto_climb(self):
        self.auto_arm_until = time.time() + AUTO_ARM_DELAY_SEC
        self.climb_phase = self.PHASE_ARMING
        self.phase_start_time = time.time()
        self.had_started_climb = False
        self.level_start_time = None
        self.no_stairs_since = None
        self.rear_jack_cycle_count = 0
        self.control.stop()
        self.control.stop_all_jacks()

    def set_climb_phase(self, new_phase):
        if self.climb_phase == new_phase:
            return

        print(f"[AUTO CLIMB] Phase: {self.climb_phase} -> {new_phase}")

        self.climb_phase = new_phase
        self.phase_start_time = time.time()

        if new_phase == self.PHASE_REAR_JACK_EXTEND:
            self.control.stop()
            self.control.rear_jack_extend()

        elif new_phase == self.PHASE_DRIVE_WITH_REAR_JACK:
            # Rear jack stays extended while robot moves forward a little.
            self.control.set_speed(AUTO_CLIMB_JACK_DRIVE_SPEED)
            self.control.forward()

        elif new_phase == self.PHASE_REAR_JACK_RETRACT:
            self.control.stop()
            self.control.rear_jack_retract()

        elif new_phase == self.PHASE_RECOVER_FORWARD:
            self.control.set_speed(AUTO_CLIMB_FORWARD_SPEED)
            self.control.forward()

        elif new_phase in [self.PHASE_TOP_REACHED, self.PHASE_DANGER_STOP]:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()

    def rear_jack_extend_finished(self, rear_ultrasonic, elapsed):
        """
        Rear jack is a linear actuator, so it needs time.
        We stop it using rear ultrasonic if valid, otherwise by max time fallback.
        """

        if elapsed >= REAR_JACK_EXTEND_MAX_SEC:
            return True

        if not REAR_JACK_USE_ULTRASONIC:
            return False

        if rear_ultrasonic <= 0:
            return False

        if REAR_JACK_EXTEND_STOP_WHEN_LESS_EQUAL:
            return rear_ultrasonic <= REAR_JACK_EXTEND_TARGET_CM

        return rear_ultrasonic >= REAR_JACK_EXTEND_TARGET_CM

    def rear_jack_retract_finished(self, rear_ultrasonic, elapsed):
        """
        Stop rear jack retract using rear ultrasonic if valid, otherwise by max time fallback.
        """

        if elapsed >= REAR_JACK_RETRACT_MAX_SEC:
            return True

        if not REAR_JACK_USE_ULTRASONIC:
            return False

        if rear_ultrasonic <= 0:
            return False

        if REAR_JACK_RETRACT_STOP_WHEN_GREATER_EQUAL:
            return rear_ultrasonic >= REAR_JACK_RETRACT_TARGET_CM

        return rear_ultrasonic <= REAR_JACK_RETRACT_TARGET_CM

    # ========================================================
    # Incoming Mega / ESP32 / UNO
    # ========================================================

    def handle_incoming_mega_lines(self):
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

            if line.startswith("CAM:"):
                self.control.send_camera_command(line)
                continue

    def handle_incoming_uno_lines(self):
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
            self.control.stop_all_jacks()
            self.control.camera_center()

        elif self.mode == MODE_OBJECT:
            self.control.set_speed(DEFAULT_SPEED)
            self.control.camera_center()

        elif self.mode == MODE_STAIRS:
            self.control.set_speed(STAIRS_SPEED)
            self.control.camera_down()

        elif self.mode == MODE_CLIMB_ASSIST:
            self.reset_auto_climb()
            self.control.set_speed(CLIMB_SPEED)
            self.control.mode_climb()
            self.control.camera_down()

    # ========================================================
    # Auto Climb UP only
    # ========================================================

    def auto_climb_up_step(self, stairs, yellow):
        """
        Auto climb UP only.

        Real process based on your manual mobile control:
        1. Keep pressing forward along the yellow path.
        2. When the robot front part gets on the stair, pitch rises.
        3. Use rear linear actuator only.
        4. Wait for rear jack using rear ultrasonic or time fallback.
        5. Move forward while rear jack is extended.
        6. Retract rear jack because it can hit the lower stair.
        7. Repeat for next stair.
        """

        data = self.control.latest_sensor_data
        pitch = data.pitch
        roll = data.roll
        rear_ultrasonic = data.rear_ultrasonic
        now = time.time()
        elapsed = now - self.phase_start_time

        # ==========================================
        # 1. Emergency safety
        # ==========================================

        if abs(roll) > CLIMB_ROLL_DANGER:
            self.set_climb_phase(self.PHASE_DANGER_STOP)
            return "DANGER_ROLL_STOP"

        if abs(pitch) > CLIMB_PITCH_DANGER:
            self.set_climb_phase(self.PHASE_DANGER_STOP)
            return "DANGER_PITCH_STOP"

        # ==========================================
        # 2. Arming wait
        # ==========================================

        if self.climb_phase == self.PHASE_ARMING:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_down()

            if now >= self.auto_arm_until:
                self.set_climb_phase(self.PHASE_SEARCH_ALIGN)

            return "AUTO_ARMING_WAIT"

        # ==========================================
        # 3. Top / landing detection
        # ==========================================

        if not stairs["stairs_found"]:
            if self.no_stairs_since is None:
                self.no_stairs_since = now
        else:
            self.no_stairs_since = None

        is_level = abs(pitch) <= LEVEL_PITCH_ABS and abs(roll) <= LEVEL_ROLL_ABS

        if self.rear_jack_cycle_count >= 1 and is_level:
            if self.level_start_time is None:
                self.level_start_time = now
        else:
            self.level_start_time = None

        if (
            self.rear_jack_cycle_count >= 1 and
            self.level_start_time is not None and
            self.no_stairs_since is not None and
            now - self.level_start_time >= TOP_LEVEL_TIME_SEC and
            now - self.no_stairs_since >= NO_STAIRS_TOP_TIME_SEC,
            VISION_LOST_FORWARD_SPEED,
            YELLOW_LOST_FORWARD_MAX_SEC,
            NO_STAIRS_FORWARD_MAX_SEC
        ):
            self.set_climb_phase(self.PHASE_TOP_REACHED)
            self.auto_climb_finished = True
            return "TOP_REACHED_STOP"

        # ==========================================
        # 4. Rear jack extend phase
        # ==========================================

        if self.climb_phase == self.PHASE_REAR_JACK_EXTEND:
            if self.rear_jack_extend_finished(rear_ultrasonic, elapsed):
                self.control.rear_jack_stop()
                self.set_climb_phase(self.PHASE_DRIVE_WITH_REAR_JACK)
                return "REAR_JACK_EXTENDED_DRIVE_NOW"

            return f"REAR_JACK_EXTENDING_UR={rear_ultrasonic:.1f}"

        # ==========================================
        # 5. Drive while rear jack is extended
        # ==========================================

        if self.climb_phase == self.PHASE_DRIVE_WITH_REAR_JACK:
            if elapsed >= DRIVE_WITH_REAR_JACK_SEC:
                self.set_climb_phase(self.PHASE_REAR_JACK_RETRACT)
                return "DRIVE_DONE_RETRACT_REAR_JACK"

            return "DRIVING_WITH_REAR_JACK_EXTENDED"

        # ==========================================
        # 6. Rear jack retract phase
        # ==========================================

        if self.climb_phase == self.PHASE_REAR_JACK_RETRACT:
            if self.rear_jack_retract_finished(rear_ultrasonic, elapsed):
                self.control.rear_jack_stop()
                self.rear_jack_cycle_count += 1
                self.set_climb_phase(self.PHASE_RECOVER_FORWARD)
                return "REAR_JACK_RETRACTED_RECOVER_FORWARD"

            return f"REAR_JACK_RETRACTING_UR={rear_ultrasonic:.1f}"

        # ==========================================
        # 7. Recover forward after retract
        # ==========================================

        if self.climb_phase == self.PHASE_RECOVER_FORWARD:
            if elapsed >= RECOVER_FORWARD_SEC:
                self.set_climb_phase(self.PHASE_SEARCH_ALIGN)
                return "RECOVER_DONE_SEARCH_NEXT_STEP"

            return "RECOVER_FORWARD_AFTER_RETRACT"

        # ==========================================
        # 8. Search / align on yellow path
        # ==========================================

        if not yellow["yellow_found"]:
            # Do not freeze here.
            # Sometimes the camera does not see yellow at the beginning.
            # So we move forward slowly for a limited time, like manual mobile control.
            # Also do not keep moving the camera every frame.
            if self.yellow_lost_since is None:
                self.yellow_lost_since = now

            lost_time = now - self.yellow_lost_since

            if lost_time <= YELLOW_LOST_FORWARD_MAX_SEC:
                self.control.set_speed(VISION_LOST_FORWARD_SPEED)
                self.control.forward()
                self.set_climb_phase(self.PHASE_FORWARD_ON_YELLOW)
                return "YELLOW_NOT_FOUND_FORWARD_SLOW"

            self.control.stop()
            self.set_climb_phase(self.PHASE_SEARCH_ALIGN)
            return "YELLOW_LOST_TOO_LONG_STOP"

        self.yellow_lost_since = None

        move_cmd, yellow_state = self.vision.decide_yellow_path_action(yellow)

        if yellow_state == "YELLOW_LEFT_ALIGN":
            self.control.set_speed(AUTO_CLIMB_ALIGN_SPEED)
            self.control.left()
            self.set_climb_phase(self.PHASE_SEARCH_ALIGN)
            return "ALIGN_LEFT_ON_YELLOW"

        if yellow_state == "YELLOW_RIGHT_ALIGN":
            self.control.set_speed(AUTO_CLIMB_ALIGN_SPEED)
            self.control.right()
            self.set_climb_phase(self.PHASE_SEARCH_ALIGN)
            return "ALIGN_RIGHT_ON_YELLOW"

        # If yellow is centered, drive forward.
        if self.climb_phase == self.PHASE_SEARCH_ALIGN:
            self.set_climb_phase(self.PHASE_FORWARD)

        # ==========================================
        # 9. Forward on yellow and trigger rear jack
        # ==========================================

        if self.climb_phase == self.PHASE_FORWARD:
            self.control.set_speed(AUTO_CLIMB_FORWARD_SPEED)
            self.control.forward()

            # Front of robot is on the stair: pitch goes up.
            # Then use rear jack only.
            if pitch >= CLIMB_FRONT_ON_STEP_PITCH:
                self.had_started_climb = True
                self.set_climb_phase(self.PHASE_REAR_JACK_EXTEND)
                return "FRONT_ON_STEP_START_REAR_JACK"

            return "FORWARD_ON_YELLOW_WAITING_FOR_STEP"

        return f"AUTO_CLIMB_PHASE_{self.climb_phase}"

    # ========================================================
    # Debug
    # ========================================================

    def print_climb_debug_every_second(self, state, yellow=None):
        now = time.time()

        if now - self.last_debug_print_time < 1.0:
            return

        self.last_debug_print_time = now
        data = self.control.latest_sensor_data

        yellow_text = ""
        if yellow is not None:
            yellow_text = f" yellow={yellow['state']} err={yellow['error_x']}"

        print(
            f"[CLIMB_UP] phase={self.climb_phase} state={state} "
            f"pitch={data.pitch:.2f} roll={data.roll:.2f} "
            f"UF={data.front_ultrasonic:.2f} UR={data.rear_ultrasonic:.2f} "
            f"cycles={self.rear_jack_cycle_count}{yellow_text}"
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

            if self.mode == MODE_OBJECT:
                detection, mask = self.vision.detect_color_object(frame)
                move_cmd, cam_cmd, state = self.vision.decide_object_action(detection)

                self.control.apply_auto_move_command(move_cmd)
                self.control.apply_auto_camera_command(cam_cmd)

                debug_frame = self.vision.draw_object_debug(
                    frame, detection, move_cmd, cam_cmd, state,
                    self.mode, self.control.speed, data
                )

            elif self.mode == MODE_STAIRS:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)
                move_cmd, state = self.vision.decide_stairs_action(stairs, data)

                self.control.apply_auto_move_command(move_cmd)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, move_cmd, state,
                    self.mode, self.control.speed, data
                )
                mask = edges

            elif self.mode == MODE_CLIMB_ASSIST:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)
                yellow, yellow_mask = self.vision.detect_yellow_path(frame)

                state = self.auto_climb_up_step(stairs, yellow)
                self.last_climb_state = state
                self.print_climb_debug_every_second(state, yellow)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, "AUTO_UP", state,
                    self.mode, self.control.speed, data
                )

                debug_frame = self.vision.draw_yellow_path_debug(debug_frame, yellow)
                mask = yellow_mask

            else:
                debug_frame = self.vision.draw_idle_debug(
                    frame, self.mode, self.control.speed, data
                )

            cv2.imshow("Apex Rover Auto Climb UP V1", debug_frame)

            if mask is not None:
                cv2.imshow("Yellow Path / Vision Mask", mask)

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

        elif key == ord("t"):
            self.control.camera_status()

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

        elif key == ord("p"):
            self.control.print_sensor_data()

    # ========================================================
    # Shutdown
    # ========================================================

    def shutdown(self):
        print("[SYSTEM] Shutting down Auto Climb UP V1")

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

