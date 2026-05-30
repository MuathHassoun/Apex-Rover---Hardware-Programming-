
import time
import cv2

from control_brain import ControlBrain
from vision_brain import VisionBrain
from sensors import parse_sensor_data
from smart_observer import SmartMotionObserver
from config import (
    CAPTURE_DELAY,
    DEFAULT_MODE,
    DEFAULT_SPEED,
    STAIRS_SPEED,
    CLIMB_SPEED,
    MODE_IDLE,
    MODE_MANUAL,
    MODE_OBJECT,
    MODE_STAIRS,
    MODE_CLIMB_ASSIST,
    STAIRS_ALIGN_SPEED,
    STAIRS_APPROACH_SPEED,
    STAIRS_CLIMB_SPEED,
    STAIRS_SHORT_PULSE_SPEED,
    CAMERA_STEPPER_SPEED,
    CAMERA_STEPPER_STEPS,
    AUTO_CAMERA_SCAN_INTERVAL,
    YELLOW_TRACK_CENTER_TOLERANCE,
    YELLOW_TRACK_LOST_TIMEOUT,
    STAIRS_APPROACH_DISTANCE_CM,
    STAIRS_FRONT_TOO_CLOSE_CM,
    STAIRS_REAR_TOO_CLOSE_CM,
    STAIRS_STOP_BEFORE_CLIMB_TIME,
    REAR_JACK_EXTEND_TIME,
    REAR_JACK_RETRACT_TIME,
    STAIRS_LONG_FORWARD_TIME,
    STAIRS_SHORT_FORWARD_TIME,
    STAIRS_REPEAT_PAUSE_TIME,
    FINISH_FLAT_PITCH_DEG,
    FINISH_FRONT_CLEAR_CM,
    CLIMB_DANGER_PITCH,
    CLIMB_DANGER_ROLL,
    SHOW_DEBUG_WINDOWS,
    CAMERA_FORWARD_DOWN_PULSES,
    CAMERA_GROUND_DOWN_PULSES,
    CAMERA_PULSE_DELAY_SEC,
    CAMERA_POSE_SETTLE_TIME,
    APPROACH_FORWARD_TIME_BEFORE_GROUND_CHECK,
    GROUND_VERIFY_TIMEOUT,
    GROUND_CONFIRM_FRAMES,
    GROUND_CONFIRM_MIN_CONFIDENCE,
    SMART_NO_PROGRESS_GRACE_SEC,
    SMART_MIN_FORWARD_STATE_TIME,
    SMART_MAX_FORWARD_STATE_TIME,
    SMART_JACK_MIN_EXTEND_TIME,
    SMART_JACK_CONTACT_TIMEOUT,
    RECOVERY_MAX_ATTEMPTS,
    RECOVERY_SETTLE_TIME,
    RECOVERY_BACKWARD_TIME,
    RECOVERY_BACKWARD_SPEED,
    RECOVERY_AFTER_BACKWARD_REALIGN_TIME,
    SMART_LOST_TRACK_RECOVERY_TIMEOUT,
    CAMERA_MOUNT_OFFSET_X_PIXELS,
    CAMERA_CLIMB_DOWN_PULSES,
    CAMERA_CLIMB_SETTLE_TIME,
    JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC,
    JACK_EFFECT_DETECT_TIMEOUT_SEC,
    JACK_ABSOLUTE_MAX_EXTEND_SEC
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

        # Default to autonomous stair climb. Manual mode still works from keyboard/mobile.
        self.mode = DEFAULT_MODE
        self.running = True

        # Stair climb state machine
        self.stair_state = "SEARCH_STAIRS"
        self.stair_action_started_at = 0.0
        self.last_stairs_seen_at = 0.0
        self.last_known_stairs = None
        self.camera_scan_direction = "RIGHT"
        self.last_camera_scan_time = 0.0
        self.climb_camera_prepared = False
        self.camera_pose = "UNKNOWN"
        self.ground_confirm_counter = 0
        self.close_confirmed_by_ground_camera = False
        self.approach_started_at = 0.0
        self.jack_effect_detected_at = None
        self.jack_extend_started_at = 0.0

        self.last_climb_state = "NONE"
        self.last_debug_print_time = 0.0

        # Smart adaptive observer / recovery layer.
        # The robot must not blindly memorize timing. It watches real feedback.
        self.observer = SmartMotionObserver()
        self.observer.reset()
        self.recovery_attempts = {}
        self.recovery_reason = "NONE"
        self.state_before_recovery = "SEARCH_STAIRS"

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

        if self.mode == MODE_CLIMB_ASSIST:
            self.change_mode(MODE_CLIMB_ASSIST)

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
        print("New network mode: Raspberry controls only when ESP32 reports AUTO.")
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
            self.prepare_auto_camera_pose()

        elif self.mode == MODE_CLIMB_ASSIST:
            self.control.set_speed(CLIMB_SPEED)
            self.control.mode_climb()
            self.reset_stair_state()
            self.prepare_auto_camera_pose()

    def reset_stair_state(self):
        self.stair_state = "SEARCH_STAIRS"
        self.stair_action_started_at = time.time()
        self.last_stairs_seen_at = 0.0
        self.last_known_stairs = None
        self.camera_scan_direction = "RIGHT"
        self.last_camera_scan_time = 0.0
        self.climb_camera_prepared = False
        self.camera_pose = "UNKNOWN"
        self.ground_confirm_counter = 0
        self.close_confirmed_by_ground_camera = False
        self.approach_started_at = 0.0
        self.jack_effect_detected_at = None
        self.jack_extend_started_at = 0.0
        self.observer.reset()
        self.recovery_attempts = {}
        self.recovery_reason = "NONE"
        self.state_before_recovery = "SEARCH_STAIRS"

    def set_stair_state(self, new_state: str):
        if self.stair_state != new_state:
            print(f"[STAIR STATE] {self.stair_state} -> {new_state}")
            self.stair_state = new_state
            self.stair_action_started_at = time.time()
            if new_state in ["REAR_JACK_EXTEND", "REAR_JACK_EXTEND_AGAIN"]:
                self.jack_effect_detected_at = None
                self.jack_extend_started_at = self.stair_action_started_at
            # Mark a fresh sensor/vision baseline for this new state.
            try:
                self.observer.mark_state_start(
                    new_state,
                    self.last_known_stairs,
                    self.control.latest_sensor_data
                )
            except Exception as e:
                print(f"[SMART] observer state-start warning: {e}")

    def state_elapsed(self) -> float:
        return time.time() - self.stair_action_started_at

    def robot_center_x_in_camera(self) -> int:
        """
        Camera is mounted on the RIGHT side of the robot, not centered.
        So the camera frame center is not the same as robot body center.
        Tune CAMERA_MOUNT_OFFSET_X_PIXELS in config.py on the real robot.
        """
        return int(320 + CAMERA_MOUNT_OFFSET_X_PIXELS)

    def yellow_track_error_x(self, stairs) -> float:
        path_center_x = stairs.get("path_center_x") or stairs.get("center_x")
        if path_center_x is None:
            return 0.0
        return float(path_center_x - self.robot_center_x_in_camera())

    def pulse_camera_down(self, pulses: int):
        """
        Current UNO command set is relative for vertical movement.
        So we only send a fixed number of CAM:DOWN pulses when changing pose,
        never continuously inside the loop.
        """
        for _ in range(max(0, int(pulses))):
            self.control.camera_down()
            time.sleep(CAMERA_PULSE_DELAY_SEC)
        self.control.camera_stop()

    def set_camera_forward_view(self):
        """
        Forward/down view used for searching, alignment, and approach.
        This is not a climb confirmation view; it only gives direction.
        """
        if self.camera_pose == "FORWARD":
            return

        self.control.camera_speed(CAMERA_STEPPER_SPEED)
        time.sleep(0.02)
        self.control.camera_steps(CAMERA_STEPPER_STEPS)
        time.sleep(0.02)
        self.control.camera_center()
        time.sleep(CAMERA_PULSE_DELAY_SEC)
        self.pulse_camera_down(CAMERA_FORWARD_DOWN_PULSES)
        self.camera_pose = "FORWARD"
        self.climb_camera_prepared = True
        print("[CAMERA POSE] FORWARD")

    def set_camera_ground_view(self):
        """
        Ground view used only to confirm the robot is very close to the stairs.
        Rule:
            If yellow stair tracks are visible while the camera is looking down,
            then the robot is close enough to start the rear-jack climbing sequence.
        """
        if self.camera_pose == "GROUND":
            return

        self.control.camera_speed(CAMERA_STEPPER_SPEED)
        time.sleep(0.02)
        self.control.camera_steps(CAMERA_STEPPER_STEPS)
        time.sleep(0.02)
        self.control.camera_center()
        time.sleep(CAMERA_PULSE_DELAY_SEC)
        self.pulse_camera_down(CAMERA_GROUND_DOWN_PULSES)
        self.camera_pose = "GROUND"
        print("[CAMERA POSE] GROUND")

    def set_camera_climb_view(self):
        """
        Climb view: camera must look DOWN while climbing stairs.
        This allows the robot to keep seeing the yellow tracks and stair surface.
        It is not allowed to stay looking upward during climb states.
        """
        if self.camera_pose == "CLIMB_DOWN":
            return

        self.control.camera_speed(CAMERA_STEPPER_SPEED)
        time.sleep(0.02)
        self.control.camera_steps(CAMERA_STEPPER_STEPS)
        time.sleep(0.02)
        self.control.camera_center()
        time.sleep(CAMERA_PULSE_DELAY_SEC)
        self.pulse_camera_down(CAMERA_CLIMB_DOWN_PULSES)
        self.camera_pose = "CLIMB_DOWN"
        print("[CAMERA POSE] CLIMB_DOWN")

    def prepare_auto_camera_pose(self):
        """
        Auto mode starts in forward view.
        The camera does NOT keep moving up/down repeatedly.
        """
        self.set_camera_forward_view()

    def slow_camera_track_scan(self):
        """
        Slow horizontal camera scan only. No repeated up/down spam.
        """
        now = time.time()
        if now - self.last_camera_scan_time < AUTO_CAMERA_SCAN_INTERVAL:
            return

        # Search must use forward view. Ground view is only for close confirmation.
        self.set_camera_forward_view()
        self.last_camera_scan_time = now

        if self.camera_scan_direction == "RIGHT":
            self.control.camera_dir_right()
            time.sleep(0.02)
            self.control.camera_steps(CAMERA_STEPPER_STEPS)
            self.camera_scan_direction = "LEFT"
        else:
            self.control.camera_dir_left()
            time.sleep(0.02)
            self.control.camera_steps(CAMERA_STEPPER_STEPS)
            self.camera_scan_direction = "RIGHT"


    # ========================================================
    # Smart adaptive monitor / recovery layer
    # ========================================================

    def is_forward_motion_state(self) -> bool:
        return self.stair_state in [
            "APPROACH_STAIRS",
            "LONG_FORWARD_PULSE",
            "SHORT_FORWARD_PULSE_1",
            "SHORT_FORWARD_PULSE_2",
        ]

    def is_jack_action_state(self) -> bool:
        return self.stair_state in [
            "REAR_JACK_EXTEND",
            "REAR_JACK_EXTEND_AGAIN",
        ]

    def enter_recovery(self, recovery_state: str, reason: str):
        """
        Stop the blind scenario and solve the current problem before continuing.
        """
        key = recovery_state + ":" + reason
        self.recovery_attempts[key] = self.recovery_attempts.get(key, 0) + 1
        attempts = self.recovery_attempts[key]

        print(
            f"[SMART RECOVERY] enter={recovery_state} reason={reason} "
            f"attempt={attempts} from={self.stair_state} {self.observer.summary()}"
        )

        self.control.stop()
        self.control.stop_all_jacks()
        self.recovery_reason = reason
        self.state_before_recovery = self.stair_state

        if attempts > RECOVERY_MAX_ATTEMPTS:
            self.set_stair_state("SAFE_STOP")
            return "RECOVERY_FAILED_SAFE_STOP"

        self.set_stair_state(recovery_state)
        return f"{recovery_state}_{reason}"

    def smart_problem_monitor(self, stairs):
        """
        Detect unexpected situations before the state machine continues.
        This is the main difference between memorized automation and smart automation.
        """
        # Do not interrupt while already recovering or intentionally stopped.
        if self.stair_state.startswith("RECOVERY") or self.stair_state in ["SAFE_STOP", "SEARCH_STAIRS", "VERIFY_CLOSE_WITH_CAMERA_DOWN"]:
            return None

        # Slip / bad drift: MPU6500 says the robot changed attitude suddenly.
        if self.observer.slip_or_drift_detected():
            return ("RECOVERY_SLIP", "SLIP_OR_BAD_DRIFT")

        # If the robot is driving forward, there must be real evidence of motion.
        if self.is_forward_motion_state():
            elapsed = self.state_elapsed()
            if elapsed >= SMART_NO_PROGRESS_GRACE_SEC and not self.observer.progress_detected():
                return ("RECOVERY_STUCK", "NO_PROGRESS_WHILE_MOTORS_FORWARD")

        # If the rear jack is extending, there must be evidence of contact/action.
        if self.is_jack_action_state():
            elapsed = self.state_elapsed()
            # Jack decisions are MPU-based. Timeout is only safety protection.
            if elapsed >= JACK_EFFECT_DETECT_TIMEOUT_SEC and not self.observer.mpu_jack_effect_detected():
                return ("RECOVERY_JACK_NO_CONTACT", "REAR_JACK_NO_MPU_EFFECT")

        # If the yellow tracks disappear while approaching, recover instead of blindly continuing.
        if self.stair_state in ["ALIGN_WITH_YELLOW_TRACKS", "APPROACH_STAIRS"]:
            if not stairs or not stairs.get("stairs_found"):
                if self.state_elapsed() >= SMART_LOST_TRACK_RECOVERY_TIMEOUT:
                    return ("RECOVERY_LOST_TRACK", "YELLOW_TRACKS_LOST")

        return None

    def handle_recovery(self, stairs):
        data = self.control.latest_sensor_data

        if self.stair_state == "SAFE_STOP":
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            return "SAFE_STOP_NEEDS_HUMAN_CHECK"

        if self.stair_state == "RECOVERY_STUCK":
            # Step 1: stop and let the robot settle.
            if self.state_elapsed() < RECOVERY_SETTLE_TIME:
                self.control.stop()
                self.control.stop_all_jacks()
                return "RECOVERY_STUCK_SETTLING"

            # Step 2: short reverse to unload the wheels/jack, not a memorized climb step.
            if self.state_elapsed() < RECOVERY_SETTLE_TIME + RECOVERY_BACKWARD_TIME:
                self.control.set_speed(RECOVERY_BACKWARD_SPEED)
                self.control.backward()
                return "RECOVERY_STUCK_SMALL_BACKWARD"

            # Step 3: stop, look forward, and re-enter perception-based alignment.
            self.control.stop()
            self.control.stop_all_jacks()
            self.close_confirmed_by_ground_camera = False
            self.set_camera_forward_view()
            if stairs and stairs.get("stairs_found"):
                self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
                return "RECOVERY_STUCK_DONE_REALIGN"
            self.set_stair_state("SEARCH_STAIRS")
            return "RECOVERY_STUCK_DONE_SEARCH"

        if self.stair_state == "RECOVERY_SLIP":
            self.control.stop()
            self.control.stop_all_jacks()
            self.close_confirmed_by_ground_camera = False

            if self.state_elapsed() < RECOVERY_SETTLE_TIME:
                return "RECOVERY_SLIP_SETTLING"

            self.set_camera_forward_view()
            if abs(data.roll) > CLIMB_DANGER_ROLL or abs(data.pitch) > CLIMB_DANGER_PITCH:
                self.set_stair_state("SAFE_STOP")
                return "RECOVERY_SLIP_DANGEROUS_SAFE_STOP"

            if stairs and stairs.get("stairs_found"):
                self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
                return "RECOVERY_SLIP_DONE_REALIGN"

            self.set_stair_state("SEARCH_STAIRS")
            return "RECOVERY_SLIP_DONE_SEARCH"

        if self.stair_state == "RECOVERY_LOST_TRACK":
            self.control.stop()
            self.close_confirmed_by_ground_camera = False
            self.set_camera_forward_view()
            self.slow_camera_track_scan()

            if stairs and stairs.get("stairs_found"):
                self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
                return "RECOVERY_LOST_TRACK_FOUND_REALIGN"

            return "RECOVERY_LOST_TRACK_SCANNING"

        if self.stair_state == "RECOVERY_JACK_NO_CONTACT":
            self.control.stop()
            self.control.rear_jack_stop()
            self.close_confirmed_by_ground_camera = False

            if self.state_elapsed() < RECOVERY_SETTLE_TIME:
                return "RECOVERY_JACK_NO_CONTACT_SETTLING"

            # Do not continue the climb sequence. Re-check closeness with camera-down.
            self.set_stair_state("VERIFY_CLOSE_WITH_CAMERA_DOWN")
            return "RECOVERY_JACK_NO_CONTACT_REVERIFY_CLOSE"

        self.control.stop()
        self.control.stop_all_jacks()
        self.set_stair_state("SEARCH_STAIRS")
        return "UNKNOWN_RECOVERY_RESET"

    # ========================================================
    # Climb Assist - Autonomous stairs logic
    # ========================================================

    def climb_assist_step(self, stairs):
        """
        Smart autonomous stair climb using camera-based close confirmation.

        Important rule for this robot:
        - Ultrasonic sensors are directed toward the ground, not forward.
        - Therefore, UF/UR are NOT used to decide distance from the stairs.
        - Camera FORWARD view is used for search/align/approach.
        - Camera GROUND view is used to confirm the robot is actually close.

        The robot must NOT start rear jack climbing just because it saw stairs far away.
        It starts climbing only after:
            yellow tracks found + aligned + approach + ground-view confirmation.
        """

        now = time.time()
        data = self.control.latest_sensor_data

        pitch = data.pitch
        roll = data.roll

        # ==========================================
        # 0. Emergency balance safety only
        # ==========================================
        if abs(roll) > CLIMB_DANGER_ROLL:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            self.close_confirmed_by_ground_camera = False
            self.set_stair_state("SEARCH_STAIRS")
            return "DANGER_ROLL_STOP"

        if abs(pitch) > CLIMB_DANGER_PITCH:
            self.control.stop()
            self.control.stop_all_jacks()
            self.control.camera_stop()
            self.close_confirmed_by_ground_camera = False
            self.set_stair_state("SEARCH_STAIRS")
            return "DANGER_PITCH_STOP"

        # Keep short memory so one bad camera frame does not stop the robot.
        if stairs and stairs.get("stairs_found"):
            self.last_stairs_seen_at = now
            self.last_known_stairs = stairs

        stairs_recently_seen = (now - self.last_stairs_seen_at) < YELLOW_TRACK_LOST_TIMEOUT
        active_stairs = stairs if stairs and stairs.get("stairs_found") else self.last_known_stairs

        # Update smart observer after choosing the active vision target.
        self.observer.update(active_stairs, data, self.stair_state)

        # Recovery states run before the normal scenario continues.
        if self.stair_state.startswith("RECOVERY") or self.stair_state == "SAFE_STOP":
            return self.handle_recovery(active_stairs)

        # Smart monitor can interrupt the scenario if the robot is stuck, slipping,
        # losing track, or the jack did not actually produce feedback.
        problem = self.smart_problem_monitor(active_stairs)
        if problem:
            recovery_state, reason = problem
            return self.enter_recovery(recovery_state, reason)

        # ==========================================
        # 1. Search yellow stair tracks from forward view
        # ==========================================
        if self.stair_state == "SEARCH_STAIRS":
            self.control.stop()
            self.close_confirmed_by_ground_camera = False
            self.ground_confirm_counter = 0
            self.set_camera_forward_view()

            if not stairs_recently_seen:
                self.slow_camera_track_scan()
                return "SEARCHING_YELLOW_TRACKS_FORWARD_VIEW"

            self.control.camera_stop()
            self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
            return "YELLOW_TRACKS_FOUND_FORWARD_VIEW"

        # ==========================================
        # 2. Align robot with the two yellow tracks
        # ==========================================
        if self.stair_state == "ALIGN_WITH_YELLOW_TRACKS":
            self.set_camera_forward_view()
            self.close_confirmed_by_ground_camera = False

            if not active_stairs:
                self.control.stop()
                self.set_stair_state("SEARCH_STAIRS")
                return "LOST_TRACKS_RETURN_SEARCH"

            path_center_x = active_stairs.get("path_center_x") or active_stairs.get("center_x")
            if path_center_x is None:
                self.control.stop()
                self.set_stair_state("SEARCH_STAIRS")
                return "NO_TRACK_CENTER_RETURN_SEARCH"

            error_x = self.yellow_track_error_x(active_stairs)

            self.control.set_speed(STAIRS_ALIGN_SPEED)

            if error_x < -YELLOW_TRACK_CENTER_TOLERANCE:
                self.control.left()
                return "ALIGN_LEFT_TO_YELLOW_TRACKS"

            if error_x > YELLOW_TRACK_CENTER_TOLERANCE:
                self.control.right()
                return "ALIGN_RIGHT_TO_YELLOW_TRACKS"

            self.control.stop()
            self.approach_started_at = now
            self.set_stair_state("APPROACH_STAIRS")
            return "ALIGNED_START_APPROACH"

        # ==========================================
        # 3. Approach using forward camera view only
        # ==========================================
        if self.stair_state == "APPROACH_STAIRS":
            self.set_camera_forward_view()
            self.close_confirmed_by_ground_camera = False
            self.ground_confirm_counter = 0

            if not stairs_recently_seen or not active_stairs:
                self.control.stop()
                self.set_stair_state("SEARCH_STAIRS")
                return "LOST_TRACKS_DURING_APPROACH"

            path_center_x = active_stairs.get("path_center_x") or active_stairs.get("center_x")
            if path_center_x is None:
                self.control.stop()
                self.set_stair_state("SEARCH_STAIRS")
                return "NO_TRACK_CENTER_DURING_APPROACH"

            error_x = self.yellow_track_error_x(active_stairs)

            # Do not continue forward if alignment became bad.
            if error_x < -YELLOW_TRACK_CENTER_TOLERANCE:
                self.control.stop()
                self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
                return "APPROACH_NEEDS_REALIGN_LEFT"

            if error_x > YELLOW_TRACK_CENTER_TOLERANCE:
                self.control.stop()
                self.set_stair_state("ALIGN_WITH_YELLOW_TRACKS")
                return "APPROACH_NEEDS_REALIGN_RIGHT"

            # Move forward for a short controlled time, then stop and verify close by camera-down view.
            if now - self.approach_started_at >= APPROACH_FORWARD_TIME_BEFORE_GROUND_CHECK:
                self.control.stop()
                self.set_stair_state("VERIFY_CLOSE_WITH_CAMERA_DOWN")
                return "APPROACH_SEGMENT_DONE_VERIFY_CLOSE"

            self.control.set_speed(STAIRS_APPROACH_SPEED)
            self.control.forward()
            return "APPROACHING_ON_YELLOW_TRACKS_FORWARD_VIEW"

        # ==========================================
        # 4. Confirm closeness by looking down
        # ==========================================
        if self.stair_state == "VERIFY_CLOSE_WITH_CAMERA_DOWN":
            self.control.stop()

            # First entry: move camera to ground view and wait for it to settle.
            if self.camera_pose != "GROUND":
                self.ground_confirm_counter = 0
                self.set_camera_ground_view()
                return "MOVING_CAMERA_TO_GROUND_VIEW"

            if self.state_elapsed() < CAMERA_POSE_SETTLE_TIME:
                return "WAITING_CAMERA_GROUND_VIEW_SETTLE"

            ground_tracks_visible = (
                stairs is not None
                and stairs.get("yellow_tracks_found", False)
                and stairs.get("confidence", 0.0) >= GROUND_CONFIRM_MIN_CONFIDENCE
            )

            if ground_tracks_visible:
                self.ground_confirm_counter += 1
                if self.ground_confirm_counter >= GROUND_CONFIRM_FRAMES:
                    self.close_confirmed_by_ground_camera = True
                    self.control.camera_stop()
                    self.set_stair_state("STOP_BEFORE_CLIMB")
                    return "CLOSE_CONFIRMED_BY_GROUND_CAMERA"
                return "GROUND_VIEW_YELLOW_CONFIRMING"

            # If ground view does not see yellow after timeout, robot is still not close enough.
            if self.state_elapsed() >= GROUND_VERIFY_TIMEOUT:
                self.close_confirmed_by_ground_camera = False
                self.ground_confirm_counter = 0
                self.set_camera_forward_view()
                self.approach_started_at = now
                self.set_stair_state("APPROACH_STAIRS")
                return "NOT_CLOSE_CONTINUE_APPROACH"

            return "GROUND_VIEW_SEARCHING_FOR_CLOSE_YELLOW"

        # ==========================================
        # 5. Stop before climb
        # ==========================================
        if self.stair_state == "STOP_BEFORE_CLIMB":
            self.control.stop()

            # Safety guard: never start the jack sequence without camera-down confirmation.
            if not self.close_confirmed_by_ground_camera:
                self.set_stair_state("VERIFY_CLOSE_WITH_CAMERA_DOWN")
                return "CLIMB_BLOCKED_NEED_GROUND_CONFIRM"

            # From here until finish/recovery, keep camera looking down at stairs/yellow path.
            self.set_camera_climb_view()
            if self.state_elapsed() < CAMERA_CLIMB_SETTLE_TIME:
                return "WAITING_CAMERA_CLIMB_DOWN_SETTLE"

            if self.state_elapsed() >= STAIRS_STOP_BEFORE_CLIMB_TIME:
                self.set_stair_state("REAR_JACK_EXTEND")
            return "PREPARE_REAR_JACK_AFTER_CLOSE_CONFIRM"

        # ==========================================
        # 6. Rear jack first: extend
        # ==========================================
        if self.stair_state == "REAR_JACK_EXTEND":
            if not self.close_confirmed_by_ground_camera:
                self.control.stop()
                self.control.stop_all_jacks()
                self.set_stair_state("VERIFY_CLOSE_WITH_CAMERA_DOWN")
                return "REAR_JACK_BLOCKED_NOT_CLOSE_CONFIRMED"

            self.set_camera_climb_view()
            self.control.stop()
            self.control.rear_jack_extend()

            # MPU rule:
            # Keep extending until MPU6500 shows the jack affected robot balance.
            # After MPU effect, keep extending 3 extra seconds, then stop.
            if self.jack_effect_detected_at is None and self.observer.mpu_jack_effect_detected():
                self.jack_effect_detected_at = now
                print(f"[JACK SMART] rear effect detected by MPU; extra {JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC:.1f}s")
                return "REAR_JACK_MPU_EFFECT_DETECTED_EXTRA_EXTEND"

            if self.jack_effect_detected_at is not None:
                if now - self.jack_effect_detected_at >= JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC:
                    self.control.rear_jack_stop()
                    self.jack_effect_detected_at = None
                    self.set_stair_state("LONG_FORWARD_PULSE")
                    return "REAR_JACK_STOP_AFTER_MPU_EFFECT_PLUS_3SEC"
                return "REAR_JACK_EXTRA_EXTENDING_AFTER_MPU_EFFECT"

            if self.state_elapsed() >= JACK_EFFECT_DETECT_TIMEOUT_SEC:
                return self.enter_recovery("RECOVERY_JACK_NO_CONTACT", "REAR_JACK_EXTEND_NO_MPU_EFFECT")

            if self.state_elapsed() >= JACK_ABSOLUTE_MAX_EXTEND_SEC:
                self.control.rear_jack_stop()
                self.set_stair_state("SAFE_STOP")
                return "REAR_JACK_ABSOLUTE_TIMEOUT_SAFE_STOP"

            return "REAR_JACK_EXTENDING_WAITING_FOR_MPU_EFFECT"

        # ==========================================
        # 7. Long forward pulse
        # ==========================================
        if self.stair_state == "LONG_FORWARD_PULSE":
            self.set_camera_climb_view()
            self.control.set_speed(STAIRS_CLIMB_SPEED)
            self.control.forward()

            # Sensor-driven transition: continue only after real progress is detected.
            if self.state_elapsed() >= SMART_MIN_FORWARD_STATE_TIME and self.observer.progress_detected():
                self.control.stop()
                self.set_stair_state("REAR_JACK_RETRACT")
                return "LONG_FORWARD_PROGRESS_CONFIRMED"

            # Max time is only a safety cap, not the reason to blindly continue.
            if self.state_elapsed() >= SMART_MAX_FORWARD_STATE_TIME:
                if self.observer.progress_detected():
                    self.control.stop()
                    self.set_stair_state("REAR_JACK_RETRACT")
                    return "LONG_FORWARD_MAX_TIME_WITH_PROGRESS"
                return self.enter_recovery("RECOVERY_STUCK", "LONG_FORWARD_NO_PROGRESS")

            return "LONG_FORWARD_WAITING_FOR_PROGRESS"

        # ==========================================
        # 8. Rear jack retract
        # ==========================================
        if self.stair_state == "REAR_JACK_RETRACT":
            self.control.stop()
            self.control.rear_jack_retract()
            if self.state_elapsed() >= REAR_JACK_RETRACT_TIME:
                self.control.rear_jack_stop()
                self.set_stair_state("SHORT_FORWARD_PULSE_1")
            return "REAR_JACK_RETRACTING"

        # ==========================================
        # 9. Short forward pulse 1
        # ==========================================
        if self.stair_state == "SHORT_FORWARD_PULSE_1":
            self.set_camera_climb_view()
            self.control.set_speed(STAIRS_SHORT_PULSE_SPEED)
            self.control.forward()

            if self.state_elapsed() >= SMART_MIN_FORWARD_STATE_TIME and self.observer.progress_detected():
                self.control.stop()
                self.set_stair_state("REAR_JACK_EXTEND_AGAIN")
                return "SHORT_FORWARD_1_PROGRESS_CONFIRMED"

            if self.state_elapsed() >= SMART_MAX_FORWARD_STATE_TIME:
                if self.observer.progress_detected():
                    self.control.stop()
                    self.set_stair_state("REAR_JACK_EXTEND_AGAIN")
                    return "SHORT_FORWARD_1_MAX_TIME_WITH_PROGRESS"
                return self.enter_recovery("RECOVERY_STUCK", "SHORT_FORWARD_1_NO_PROGRESS")

            return "SHORT_FORWARD_1_WAITING_FOR_PROGRESS"

        # ==========================================
        # 10. Rear jack extend again
        # ==========================================
        if self.stair_state == "REAR_JACK_EXTEND_AGAIN":
            self.set_camera_climb_view()
            self.control.stop()
            self.control.rear_jack_extend()

            if self.jack_effect_detected_at is None and self.observer.mpu_jack_effect_detected():
                self.jack_effect_detected_at = now
                print(f"[JACK SMART] rear effect detected again by MPU; extra {JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC:.1f}s")
                return "REAR_JACK_AGAIN_MPU_EFFECT_DETECTED_EXTRA_EXTEND"

            if self.jack_effect_detected_at is not None:
                if now - self.jack_effect_detected_at >= JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC:
                    self.control.rear_jack_stop()
                    self.jack_effect_detected_at = None
                    self.set_stair_state("SHORT_FORWARD_PULSE_2")
                    return "REAR_JACK_AGAIN_STOP_AFTER_MPU_EFFECT_PLUS_3SEC"
                return "REAR_JACK_AGAIN_EXTRA_EXTENDING_AFTER_MPU_EFFECT"

            if self.state_elapsed() >= JACK_EFFECT_DETECT_TIMEOUT_SEC:
                return self.enter_recovery("RECOVERY_JACK_NO_CONTACT", "REAR_JACK_EXTEND_AGAIN_NO_MPU_EFFECT")

            if self.state_elapsed() >= JACK_ABSOLUTE_MAX_EXTEND_SEC:
                self.control.rear_jack_stop()
                self.set_stair_state("SAFE_STOP")
                return "REAR_JACK_AGAIN_ABSOLUTE_TIMEOUT_SAFE_STOP"

            return "REAR_JACK_EXTEND_AGAIN_WAITING_FOR_MPU_EFFECT"

        # ==========================================
        # 11. Short forward pulse 2
        # ==========================================
        if self.stair_state == "SHORT_FORWARD_PULSE_2":
            self.set_camera_climb_view()
            self.control.set_speed(STAIRS_SHORT_PULSE_SPEED)
            self.control.forward()

            if self.state_elapsed() >= SMART_MIN_FORWARD_STATE_TIME and self.observer.progress_detected():
                self.control.stop()
                self.set_stair_state("REAR_JACK_RETRACT_AGAIN")
                return "SHORT_FORWARD_2_PROGRESS_CONFIRMED"

            if self.state_elapsed() >= SMART_MAX_FORWARD_STATE_TIME:
                if self.observer.progress_detected():
                    self.control.stop()
                    self.set_stair_state("REAR_JACK_RETRACT_AGAIN")
                    return "SHORT_FORWARD_2_MAX_TIME_WITH_PROGRESS"
                return self.enter_recovery("RECOVERY_STUCK", "SHORT_FORWARD_2_NO_PROGRESS")

            return "SHORT_FORWARD_2_WAITING_FOR_PROGRESS"

        # ==========================================
        # 12. Rear jack retract again
        # ==========================================
        if self.stair_state == "REAR_JACK_RETRACT_AGAIN":
            self.control.stop()
            self.control.rear_jack_retract()
            if self.state_elapsed() >= REAR_JACK_RETRACT_TIME:
                self.control.rear_jack_stop()
                self.set_stair_state("CHECK_CONTINUE_OR_FINISH")
            return "REAR_JACK_RETRACT_AGAIN"

        # ==========================================
        # 13. Repeat or finish
        # ==========================================
        if self.stair_state == "CHECK_CONTINUE_OR_FINISH":
            self.control.stop()

            flat_again = abs(pitch) < FINISH_FLAT_PITCH_DEG
            no_tracks_recently = not stairs_recently_seen

            if flat_again and no_tracks_recently:
                self.set_stair_state("FINISH_CLIMB")
                return "CLIMB_FINISH_CONDITION_MET"

            if self.state_elapsed() >= STAIRS_REPEAT_PAUSE_TIME:
                # Smart behavior: do NOT blindly repeat the same climb script.
                # Re-check with the camera-down pose before starting the next step.
                self.close_confirmed_by_ground_camera = False
                self.ground_confirm_counter = 0
                self.set_stair_state("VERIFY_CLOSE_WITH_CAMERA_DOWN")

            return "REVERIFY_BEFORE_NEXT_STAIR_STEP"

        # ==========================================
        # 14. Finish
        # ==========================================
        if self.stair_state == "FINISH_CLIMB":
            self.control.stop()
            self.control.stop_all_jacks()
            self.set_camera_forward_view()
            return "CLIMB_FINISHED"

        # Unknown state recovery
        self.control.stop()
        self.control.stop_all_jacks()
        self.close_confirmed_by_ground_camera = False
        self.set_stair_state("SEARCH_STAIRS")
        return "UNKNOWN_STATE_RESET"

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
            f"[CLIMB] state={state} machine={self.stair_state} "
            f"camera={self.camera_pose} close_ok={self.close_confirmed_by_ground_camera} "
            f"pitch={data.pitch:.2f} roll={data.roll:.2f} "
            f"UF_ground={data.front_ultrasonic:.2f} UR_ground={data.rear_ultrasonic:.2f} "
            f"robot_cx={self.robot_center_x_in_camera()} {self.observer.summary()}"
        )

    # ========================================================
    # Main Loop
    # ========================================================

    def run_loop(self):
        last_manual_idle_print = 0.0

        while self.running:
            # ====================================================
            # New ESP32 WiFi architecture:
            # Raspberry Pi is allowed to control only when ESP32 mode is AUTO.
            # In MANUAL, mobile app owns control through ESP32, so Raspberry stays idle.
            # ====================================================
            esp32_mode = self.control.get_system_mode()

            if esp32_mode != "AUTO":
                now = time.time()
                if now - last_manual_idle_print > 2.0:
                    last_manual_idle_print = now
                    print("[ESP32 MODE] MANUAL - Raspberry brain idle. Select Automatic from mobile Home to start auto stairs.")
                time.sleep(0.25)
                continue

            if self.mode != MODE_CLIMB_ASSIST:
                print("[ESP32 MODE] AUTO - Starting CLIMB_ASSIST brain")
                self.change_mode(MODE_CLIMB_ASSIST)

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

            if SHOW_DEBUG_WINDOWS:
                cv2.imshow("Apex Rover Modular Brain V6", debug_frame)

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
            self.control.camera_steps(CAMERA_STEPPER_STEPS)

        elif key == ord("]"):
            self.control.camera_dir_right()
            time.sleep(0.02)
            self.control.camera_steps(CAMERA_STEPPER_STEPS)

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
