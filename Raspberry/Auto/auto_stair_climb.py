#!/usr/bin/env python3
"""
auto_stair_climb.py - Apex Rover automatic scenario

Scenario:
  1) Search for the nearest valid source box.
  2) Approach it with robot left side close to the box because the arm is on the left.
  3) Use arm camera to move 3 objects one by one from source box to robot rear basket.
  4) Move arm HOME before climbing.
  5) Search for stairs and align robot body center, compensating for right-mounted front camera.
  6) Climb using vision + MPU feedback. Ultrasonic is only safety gating.
  7) At top, search destination box and move objects from robot basket to destination box.
  8) Track everything in /tmp/apex_auto_status.json and /auto_status.

Important architecture rule:
  This file does NOT open Mega serial directly.
  It reads sensors from /tmp/apex_last_sensor.json written by Manual/sensor_bridge.py.
  Commands go Raspberry -> ESP32 HTTP /command -> Mega/UNO.
"""

import math
import signal
import sys
import time
from pathlib import Path

# Allow importing from Auto and Raspberry root.
AUTO_DIR = Path(__file__).resolve().parent
RASPBERRY_DIR = AUTO_DIR.parent
sys.path.insert(0, str(AUTO_DIR))
sys.path.insert(0, str(RASPBERRY_DIR))

from auto_config import *  # noqa: F401,F403
from auto_io import RoverIO
from auto_status import AutoStatus
from auto_vision import TemplateLibrary, VisionBrain


class AutoScenario:
    def __init__(self):
        self.status = AutoStatus()
        self.io = RoverIO(self.status)
        self.vision = VisionBrain(self.status, TemplateLibrary())
        self.running = True
        self.objects_loaded = 0
        self.objects_delivered = 0
        self.last_progress_signature = None
        self.no_progress_count = 0

    # ============================================================
    # Lifecycle
    # ============================================================
    def stop_requested(self, signum=None, frame=None):
        self.running = False
        self.status.event(
            phase="SHUTDOWN",
            doing="Shutdown signal received",
            decision="safe stop all actuators",
            signal=signum,
        )
        self.io.stop_all()

    def run(self):
        self.status.set_running(True)
        self.status.event(
            phase="START",
            doing="Starting automatic pickup, climb, and delivery scenario",
            decision="initialize robot mode, speed, camera, arm",
        )

        try:
            self.initialize_robot()

            source_found = self.search_and_approach_box(source=True)
            if not source_found:
                self.safe_finish("NO_SOURCE_BOX", "No valid source box found")
                return

            self.transfer_source_box_to_robot_basket()

            self.status.event(
                phase="ARM_HOME_BEFORE_CLIMB",
                doing="Moving arm to HOME before stair climbing",
                decision="avoid shaking/interference during climb",
            )
            self.arm_home()

            stairs_found = self.search_align_and_climb_stairs()
            if not stairs_found:
                self.safe_finish("NO_STAIRS", "No stairs found or climb failed")
                return

            destination_found = self.search_and_approach_box(source=False)
            if not destination_found:
                self.safe_finish("NO_DESTINATION_BOX", "No destination box found on top area")
                return

            self.transfer_robot_basket_to_destination_box()

            self.safe_finish("DONE", "Scenario completed successfully")

        except KeyboardInterrupt:
            self.stop_requested(signal.SIGINT, None)
        except Exception as e:
            self.status.fail("AUTO_EXCEPTION", f"Unhandled auto error: {e}")
            self.io.stop_all()
        finally:
            self.status.set_running(False)
            self.status.write()

    def initialize_robot(self):
        self.io.command("SYS:MODE:AUTO")
        self.io.command(f"SPEED:{NORMAL_SPEED}")
        self.io.command("MODE:NORMAL")
        self.io.command("CAM:CENTER")
        self.io.command("ARM:HOME")
        self.io.command("JACK:ALL:STOP")
        self.io.wait(1.0)

    def safe_finish(self, phase, message):
        self.status.event(phase=phase, doing=message, decision="STOP and keep auto status available")
        self.io.stop_all()
        self.status.set_running(False)

    # ============================================================
    # Search helpers
    # ============================================================
    def scan_for_detection(self, category, camera="front", phase="SCAN", accept_fn=None):
        """
        Search by using front/arm camera and moving camera stand horizontally.
        The camera stepper is returned close to center at the end of each sweep.
        """
        scan_moves = [
            ("CAM:CENTER", "center"),
            ("CAM:STEP_LEFT:350", "left small"),
            ("CAM:STEP_LEFT:350", "left wide"),
            ("CAM:STEP_RIGHT:700", "right wide"),
            ("CAM:STEP_RIGHT:350", "right extra"),
            ("CAM:STEP_LEFT:350", "return center-ish"),
        ]

        best = None
        for sweep in range(MAX_SEARCH_SWEEPS):
            if not self.running or not self.io.safety_ok():
                return None
            self.status.event(
                phase=phase,
                doing=f"Searching for {category}",
                decision=f"camera scan sweep {sweep + 1}/{MAX_SEARCH_SWEEPS}",
            )

            for cmd, label in scan_moves:
                self.io.command(cmd)
                self.io.wait(0.35)
                frame = self.io.snapshot(camera)
                det = self.vision.detect(frame, category)
                det["scan_label"] = label
                det["scan_sweep"] = sweep + 1
                self.status.set_detection(det)

                if best is None or det.get("score", 0) > best.get("score", 0):
                    best = det

                if det.get("found"):
                    if accept_fn is None or accept_fn(det, frame):
                        self.status.event(
                            phase=phase,
                            doing=f"Found {category}",
                            decision="accept detection and continue",
                            detection=det,
                        )
                        return det
                    self.status.event(
                        phase=phase,
                        doing=f"Rejected {category} detection",
                        decision="detection does not pass geometry/safety constraints",
                        detection=det,
                    )

            # If not found, rotate the whole robot a little and try again.
            self.status.event(
                phase=phase,
                doing=f"{category} not found in camera sweep",
                decision="turn robot a little and rescan",
                best_detection=best,
            )
            self.io.pulse("LEFT", PULSE_TURN_MS)
            self.io.wait(0.3)

        self.status.event(
            phase=phase,
            doing=f"Finished search without valid {category}",
            decision="not found",
            best_detection=best,
        )
        return None

    # ============================================================
    # Source/destination boxes
    # ============================================================
    def search_and_approach_box(self, source=True):
        category = "source_box" if source else "destination_box"
        phase = "SEARCH_SOURCE_BOX" if source else "SEARCH_DESTINATION_BOX"
        close_ratio = BOX_CLOSE_BBOX_HEIGHT_RATIO if source else DESTINATION_BOX_CLOSE_BBOX_HEIGHT_RATIO

        def accept_box(det, frame):
            if frame is None:
                return False
            return self.vision.is_box_height_allowed(det, frame.shape)

        det = self.scan_for_detection(category, camera="front", phase=phase, accept_fn=accept_box)
        if det is None:
            return False

        self.status.event(
            phase="APPROACH_SOURCE_BOX" if source else "APPROACH_DESTINATION_BOX",
            doing="Approaching box with robot left side close to target",
            decision="align target to left-side arm approach point",
            detection=det,
        )

        return self.approach_detection(
            category=category,
            target_x_ratio=BOX_APPROACH_TARGET_X_RATIO,
            close_height_ratio=close_ratio,
            phase="APPROACH_SOURCE_BOX" if source else "APPROACH_DESTINATION_BOX",
            camera="front",
            max_steps=MAX_APPROACH_STEPS if source else MAX_DELIVERY_STEPS,
            require_box_height_rule=True,
        )

    def approach_detection(
        self,
        category,
        target_x_ratio,
        close_height_ratio,
        phase,
        camera="front",
        max_steps=MAX_APPROACH_STEPS,
        require_box_height_rule=False,
    ):
        for step in range(max_steps):
            if not self.running or not self.io.safety_ok():
                return False

            frame = self.io.snapshot(camera)
            det = self.vision.detect(frame, category)
            self.status.set_detection(det)

            if not det.get("found"):
                self.status.event(
                    phase=phase,
                    doing="Target temporarily lost while approaching",
                    decision="stop, rescan small left/right",
                    step=step,
                )
                self.io.command("STOP")
                self.io.pulse("LEFT", PULSE_TURN_SMALL_MS)
                self.io.wait(0.25)
                continue

            if require_box_height_rule and not self.vision.is_box_height_allowed(det, frame.shape):
                self.status.fail(phase, "Box detected but rejected because it is above allowed robot horizontal level", detection=det)
                self.io.stop_all()
                return False

            frame_w = det.get("frame_w") or FRAME_WIDTH
            bbox = det.get("bbox") or [0, 0, 0, 0]
            bbox_h_ratio = bbox[3] / max(1, det.get("frame_h") or FRAME_HEIGHT)
            target_x = int(frame_w * target_x_ratio)
            error_x = int(det["cx"] - target_x)

            self.status.event(
                phase=phase,
                doing="Approaching visual target",
                decision="align then forward" if abs(error_x) > ALIGN_TOLERANCE_PX else "forward pulse",
                step=step,
                error_x=error_x,
                target_x=target_x,
                bbox_h_ratio=bbox_h_ratio,
                detection=det,
            )

            if bbox_h_ratio >= close_height_ratio:
                self.io.command("STOP")
                self.status.event(
                    phase=phase,
                    doing="Reached target close zone",
                    decision="stop approach and start next phase",
                    bbox_h_ratio=bbox_h_ratio,
                )
                return True

            if abs(error_x) > ALIGN_TOLERANCE_PX:
                direction = "RIGHT" if error_x > 0 else "LEFT"
                self.io.pulse(direction, PULSE_TURN_SMALL_MS)
            else:
                sensor = self.io.read_sensor()
                front = float(sensor.get("front", -1)) if sensor else -1
                if 0 < front < FRONT_ULTRASONIC_CAUTION_CM:
                    self.io.pulse("FORWARD", PULSE_FORWARD_SLOW_MS)
                else:
                    self.io.pulse("FORWARD", PULSE_FORWARD_MS)
            self.io.wait(0.25)

        self.status.fail(phase, "Approach step limit reached without close detection")
        self.io.stop_all()
        return False

    # ============================================================
    # Arm control
    # ============================================================
    def arm_home(self):
        self.io.command("ARM:HOME")
        self.io.wait(1.6)

    def arm_ready(self):
        self.io.command("ARM:READY")
        self.io.wait(1.2)
        self.apply_arm_pose(ARM_POSE_READY, "ARM_READY_FINE")

    def apply_arm_pose(self, pose, label="ARM_POSE"):
        self.status.event(phase=label, doing="Applying calibrated arm pose", decision="send servo/base angle commands", pose=pose)
        if "base_deg" in pose:
            self.io.command(f"ARM:BASE:GOTO_DEG:{int(pose['base_deg'])}")
            self.io.wait(ARM_BASE_SETTLE_SEC)
        if "shoulder" in pose:
            self.io.command(f"ARM:SHOULDER:ANGLE:{int(pose['shoulder'])}")
        if "elbow" in pose:
            self.io.command(f"ARM:ELBOW:ANGLE:{int(pose['elbow'])}")
        if "wrist" in pose:
            self.io.command(f"ARM:WRIST:ANGLE:{int(pose['wrist'])}")
        if "aux" in pose:
            self.io.command(f"ARM:AUX:ANGLE:{int(pose['aux'])}")
        if "gripper" in pose:
            self.io.command(f"ARM:GRIPPER:ANGLE:{int(pose['gripper'])}")
        self.io.wait(ARM_SERVO_SETTLE_SEC)

    def visual_arm_align_to_object(self, phase):
        """
        Uses arm camera only to fine-align base toward detected object.
        This is intentionally simple because actual pickup depends on your calibrated pose.
        """
        for attempt in range(7):
            frame = self.io.snapshot("arm")
            det = self.vision.detect(frame, "object")
            self.status.event(
                phase=phase,
                doing="Arm camera checking object alignment",
                decision="fine align arm base" if det.get("found") else "object not visible",
                attempt=attempt + 1,
                detection=det,
            )
            if not det.get("found"):
                # Small base search motion.
                self.io.command("ARM:BASE:STEP_LEFT:80" if attempt % 2 == 0 else "ARM:BASE:STEP_RIGHT:160")
                self.io.wait(0.35)
                continue

            frame_w = det.get("frame_w") or FRAME_WIDTH
            error_x = int(det["cx"] - frame_w / 2)
            if abs(error_x) <= FINE_ALIGN_TOLERANCE_PX:
                return True
            if error_x > 0:
                self.io.command("ARM:BASE:STEP_RIGHT:70")
            else:
                self.io.command("ARM:BASE:STEP_LEFT:70")
            self.io.wait(0.35)
        return False

    def grab_from_source_and_drop_to_robot_basket(self, index):
        phase = f"PICK_OBJECT_{index}_FROM_SOURCE"
        self.status.event(phase=phase, doing="Picking one object from source box", decision="use arm camera then calibrated pick pose")

        self.arm_ready()
        self.apply_arm_pose(ARM_POSE_SOURCE_PICK_CENTER, phase + "_PRE_PICK")
        self.visual_arm_align_to_object(phase)

        # Open -> descend pose -> close -> lift.
        self.io.command("ARM:GRIPPER:OPEN")
        self.io.wait(GRIPPER_SETTLE_SEC)
        self.apply_arm_pose(ARM_POSE_SOURCE_PICK_CENTER, phase + "_PICK")
        self.io.command("ARM:GRIPPER:CLOSE")
        self.io.wait(GRIPPER_SETTLE_SEC)
        self.apply_arm_pose(ARM_POSE_SOURCE_LIFT, phase + "_LIFT")

        # Drop in robot rear basket.
        drop_phase = f"DROP_OBJECT_{index}_TO_ROBOT_BASKET"
        self.apply_arm_pose(ARM_POSE_ROBOT_BASKET_DROP, drop_phase)
        self.io.command("ARM:GRIPPER:OPEN")
        self.io.wait(GRIPPER_SETTLE_SEC)
        self.objects_loaded += 1
        self.status.set_counts(loaded=self.objects_loaded)
        self.status.event(phase=drop_phase, doing="Dropped object into robot basket", decision="continue if more objects exist")
        self.arm_ready()
        return True

    def grab_from_robot_basket_and_drop_to_destination(self, index):
        phase = f"PICK_OBJECT_{index}_FROM_ROBOT_BASKET"
        self.status.event(phase=phase, doing="Picking one object from robot rear basket", decision="use calibrated rear basket pickup pose")

        self.arm_ready()
        self.apply_arm_pose(ARM_POSE_ROBOT_BASKET_PICK, phase + "_PICK")
        self.io.command("ARM:GRIPPER:CLOSE")
        self.io.wait(GRIPPER_SETTLE_SEC)
        self.apply_arm_pose(ARM_POSE_SOURCE_LIFT, phase + "_LIFT")

        drop_phase = f"DROP_OBJECT_{index}_TO_DESTINATION_BOX"
        # Optional arm-camera check for destination/object area before drop.
        self.visual_arm_align_to_object(drop_phase)
        self.apply_arm_pose(ARM_POSE_DESTINATION_DROP, drop_phase)
        self.io.command("ARM:GRIPPER:OPEN")
        self.io.wait(GRIPPER_SETTLE_SEC)
        self.objects_delivered += 1
        self.status.set_counts(delivered=self.objects_delivered)
        self.status.event(phase=drop_phase, doing="Dropped object into destination box", decision="continue delivery")
        self.arm_ready()
        return True

    def transfer_source_box_to_robot_basket(self):
        self.status.event(
            phase="TRANSFER_TO_ROBOT_BASKET",
            doing="Moving objects one by one from source box to robot basket",
            decision="use front camera context then arm camera for manipulation",
        )
        self.io.command("SPEED:%d" % ARM_OPERATION_SPEED)
        self.io.command("CAM:CENTER")
        self.arm_ready()

        for i in range(1, MAX_OBJECTS_TO_TRANSFER + 1):
            if not self.running or not self.io.safety_ok():
                return False

            frame = self.io.snapshot("arm")
            det = self.vision.detect(frame, "object")
            self.status.event(
                phase="CHECK_SOURCE_OBJECTS",
                doing="Checking if another object remains in source box",
                decision="pick next object" if det.get("found") else "no object visible",
                object_index=i,
                detection=det,
            )

            if not det.get("found") and i > 1:
                break
            # If first object is not visually clear, still try because object templates may not be ready.
            self.grab_from_source_and_drop_to_robot_basket(i)

        self.arm_home()
        self.io.command(f"SPEED:{NORMAL_SPEED}")
        return True

    def transfer_robot_basket_to_destination_box(self):
        self.status.event(
            phase="TRANSFER_TO_DESTINATION_BOX",
            doing="Moving objects from robot basket to destination box",
            decision="deliver exactly the number of loaded objects",
            objects_loaded=self.objects_loaded,
        )
        self.io.command("SPEED:%d" % ARM_OPERATION_SPEED)
        self.arm_ready()

        count = max(0, self.objects_loaded)
        if count == 0:
            count = MAX_OBJECTS_TO_TRANSFER
            self.status.event(
                phase="TRANSFER_TO_DESTINATION_BOX",
                doing="Loaded count is zero, using fallback count",
                decision=f"fallback deliver {count} objects",
            )

        for i in range(1, count + 1):
            if not self.running or not self.io.safety_ok():
                return False
            self.grab_from_robot_basket_and_drop_to_destination(i)

        self.arm_home()
        self.io.command(f"SPEED:{NORMAL_SPEED}")
        return True

    # ============================================================
    # Stairs
    # ============================================================
    def search_align_and_climb_stairs(self):
        self.status.event(
            phase="SEARCH_STAIRS",
            doing="Searching for stairs similar to src stair templates",
            decision="camera/robot scan with right-camera offset compensation",
        )
        self.io.command("CAM:CENTER")
        self.io.command("CAM:ANGLE:105")  # look slightly down toward floor/stairs; tune if needed
        self.io.wait(0.6)

        det = self.scan_for_detection("stairs", camera="front", phase="SEARCH_STAIRS")
        if det is None:
            return False

        aligned = self.align_to_stairs()
        if not aligned:
            return False

        return self.climb_stairs()

    def align_to_stairs(self):
        self.status.event(
            phase="ALIGN_STAIRS",
            doing="Aligning robot body center with stairs center",
            decision="target x is left of frame center because camera is mounted on robot right side",
            target_x_ratio=STAIR_TARGET_X_RATIO,
        )

        for step in range(18):
            if not self.running or not self.io.safety_ok():
                return False
            frame = self.io.snapshot("front")
            det = self.vision.detect(frame, "stairs")
            if not det.get("found"):
                self.status.event(phase="ALIGN_STAIRS", doing="Stairs lost during alignment", decision="small scan turn", step=step)
                self.io.pulse("LEFT", PULSE_TURN_SMALL_MS)
                self.io.wait(0.25)
                continue

            target_x = int((det.get("frame_w") or FRAME_WIDTH) * STAIR_TARGET_X_RATIO)
            error_x = int(det["cx"] - target_x)
            self.status.event(
                phase="ALIGN_STAIRS",
                doing="Checking stair center alignment",
                decision="aligned" if abs(error_x) <= FINE_ALIGN_TOLERANCE_PX else "turn to align",
                error_x=error_x,
                target_x=target_x,
                detection=det,
            )

            if abs(error_x) <= FINE_ALIGN_TOLERANCE_PX:
                self.io.command("STOP")
                return True

            self.io.pulse("RIGHT" if error_x > 0 else "LEFT", PULSE_TURN_SMALL_MS)
            self.io.wait(0.25)

        self.status.fail("ALIGN_STAIRS", "Could not align stairs within step limit")
        self.io.stop_all()
        return False

    def climb_stairs(self):
        self.status.event(
            phase="CLIMB_STAIRS",
            doing="Starting stair climb",
            decision="use vision + MPU progress; ultrasonic only safety gate",
        )
        self.io.command("MODE:CLIMB")
        self.io.command(f"SPEED:{CLIMB_SPEED}")
        self.io.command("CAM:ANGLE:115")  # look downward during climb
        self.io.wait(0.5)

        base_sensor = self.io.read_sensor() or {}
        base_pitch = float(base_sensor.get("pitch", 0.0))
        self.no_progress_count = 0
        last_pitch = base_pitch
        last_area = 0.0
        flat_count = 0

        for step in range(MAX_CLIMB_STEPS):
            if not self.running or not self.io.safety_ok():
                return False

            frame = self.io.snapshot("front")
            det = self.vision.detect(frame, "stairs")
            sensor = self.io.read_sensor() or {}
            pitch = float(sensor.get("pitch", 0.0))
            roll = float(sensor.get("roll", 0.0))
            front = float(sensor.get("front", -1))
            area = float(det.get("area_ratio", 0.0)) if det else 0.0

            # Progress is not a fixed distance. It uses pitch/vision changes.
            pitch_change = abs(pitch - last_pitch)
            area_change = abs(area - last_area)
            progress = pitch_change > 0.7 or area_change > 0.012

            if progress:
                self.no_progress_count = 0
            else:
                self.no_progress_count += 1

            last_pitch = pitch
            last_area = area

            self.status.event(
                phase="CLIMB_STAIRS",
                doing="Climbing and monitoring progress",
                decision="forward" if self.no_progress_count < NO_PROGRESS_LIMIT else "rear jack assist needed",
                step=step,
                pitch=pitch,
                roll=roll,
                front_ultrasonic=front,
                progress=progress,
                no_progress_count=self.no_progress_count,
                detection=det,
            )

            if abs(roll) > ROLL_SAFE_FOR_CLIMB_DEG:
                self.status.fail("CLIMB_STAIRS", "Roll too high while climbing", sensor=sensor)
                self.io.stop_all()
                return False

            # Top flat area detection: pitch is stable/low and destination box may appear.
            if step > 8 and abs(pitch) <= PITCH_FLAT_TOP_DEG:
                flat_count += 1
                dest_frame = self.io.snapshot("front")
                dest = self.vision.detect(dest_frame, "destination_box")
                self.status.event(
                    phase="DETECT_TOP_AREA",
                    doing="Checking if robot reached flat top area",
                    decision="top likely reached" if flat_count >= 4 or dest.get("found") else "continue climb",
                    flat_count=flat_count,
                    destination_detection=dest,
                )
                if flat_count >= 4 or dest.get("found"):
                    self.io.command("STOP")
                    self.retract_rear_jack_until_safe()
                    self.io.command("MODE:NORMAL")
                    self.io.command(f"SPEED:{NORMAL_SPEED}")
                    self.status.event(phase="TOP_REACHED", doing="Reached flat top area", decision="continue to destination box search")
                    return True
            else:
                flat_count = 0

            if self.no_progress_count >= NO_PROGRESS_LIMIT:
                ok = self.rear_jack_assist()
                if not ok:
                    return False
                self.no_progress_count = 0
            else:
                # If front ultrasonic is close, use slow pulse but don't let it dominate decisions.
                if 0 < front < FRONT_ULTRASONIC_CAUTION_CM:
                    self.io.pulse("FORWARD", PULSE_FORWARD_SLOW_MS)
                else:
                    self.io.pulse("FORWARD", PULSE_FORWARD_MS)
                self.io.wait(0.25)

        self.status.fail("CLIMB_STAIRS", "Climb step limit reached before top detection")
        self.io.stop_all()
        return False

    def rear_jack_assist(self):
        self.status.event(
            phase="REAR_JACK_ASSIST",
            doing="Using rear jack because forward progress is weak/stuck",
            decision="extend until MPU indicates robot is affected, then stop",
        )
        self.io.command("STOP")
        before = self.io.read_sensor() or {}
        base_pitch = float(before.get("pitch", 0.0))
        base_roll = float(before.get("roll", 0.0))

        self.io.command("JACK:REAR:EXTEND")
        start = time.time()
        affected = False
        last_sensor = before

        while time.time() - start < JACK_MAX_EXTEND_SEC:
            if not self.running:
                break
            last_sensor = self.io.read_sensor() or last_sensor or {}
            pitch = float(last_sensor.get("pitch", base_pitch))
            roll = float(last_sensor.get("roll", base_roll))
            delta = max(abs(pitch - base_pitch), abs(roll - base_roll))
            self.status.event(
                phase="REAR_JACK_ASSIST",
                doing="Extending rear jack and watching MPU effect",
                decision="MPU affected" if delta >= JACK_EFFECT_DELTA_DEG else "keep extending with safety timeout",
                delta=delta,
                sensor=last_sensor,
            )
            if delta >= JACK_EFFECT_DELTA_DEG:
                affected = True
                break
            time.sleep(0.2)

        if affected:
            self.status.event(
                phase="REAR_JACK_ASSIST",
                doing="Rear jack affected robot balance",
                decision=f"continue {JACK_AFTER_EFFECT_SEC}s then stop jack",
            )
            self.io.wait(JACK_AFTER_EFFECT_SEC)
        else:
            self.status.event(
                phase="REAR_JACK_ASSIST",
                doing="Rear jack extension timeout",
                decision="stop jack for safety",
                sensor=last_sensor,
            )

        self.io.command("JACK:REAR:STOP")
        self.io.wait(0.3)

        # Move forward a little while supported.
        self.io.pulse("FORWARD", PULSE_FORWARD_SLOW_MS)
        self.io.wait(0.4)

        # Raise/retract rear jack back until stable/no strong change or timeout.
        return self.retract_rear_jack_until_safe()

    def retract_rear_jack_until_safe(self):
        self.status.event(
            phase="REAR_JACK_RETRACT",
            doing="Retracting/re-raising rear jack after assist/top detection",
            decision="retract with MPU safety timeout",
        )
        before = self.io.read_sensor() or {}
        base_pitch = float(before.get("pitch", 0.0))
        base_roll = float(before.get("roll", 0.0))
        self.io.command("JACK:REAR:RETRACT")
        start = time.time()
        stable_samples = 0
        last_sensor = before

        while time.time() - start < JACK_MAX_RETRACT_SEC:
            if not self.running:
                break
            last_sensor = self.io.read_sensor() or last_sensor or {}
            pitch = float(last_sensor.get("pitch", base_pitch))
            roll = float(last_sensor.get("roll", base_roll))
            delta = max(abs(pitch - base_pitch), abs(roll - base_roll))
            if delta < 0.8:
                stable_samples += 1
            else:
                stable_samples = 0

            self.status.event(
                phase="REAR_JACK_RETRACT",
                doing="Retracting rear jack and watching MPU stability",
                decision="stop retract" if stable_samples >= 5 else "continue retract",
                stable_samples=stable_samples,
                delta=delta,
                sensor=last_sensor,
            )
            if stable_samples >= 5:
                break
            time.sleep(0.2)

        self.io.command("JACK:REAR:STOP")
        self.io.wait(0.3)
        return True


def main():
    scenario = AutoScenario()
    signal.signal(signal.SIGINT, scenario.stop_requested)
    signal.signal(signal.SIGTERM, scenario.stop_requested)
    scenario.run()


if __name__ == "__main__":
    main()
