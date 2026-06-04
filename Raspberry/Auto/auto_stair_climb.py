
#!/usr/bin/env python3
"""
auto_stair_climb.py - Apex Rover AUTO Orchestrator

NEW ARCHITECTURE:
  Raspberry Pi is the orchestrator only.
  Mega executes heavy movement LEGO blocks.
  UNO executes saved arm poses.

Command path:
  Raspberry -> ESP32 HTTP /command -> ESP32 routes to Mega or UNO

Mega blocks:
  AUTO:UP_STAIRS
  AUTO:DOWN_STAIRS
  BLOCK:TURN:LEFT:90
  BLOCK:TURN:RIGHT:90
  BLOCK:GO:FORWARD:20
  BLOCK:GO:BACKWARD:20
  BLOCK:JACK:REAR:EXTEND:4
  BLOCK:JACK:REAR:RETRACT:4

UNO arm poses:
  ARM:READY
  ARM:TAKE_OUT
  ARM:DROP_IN
  ARM:DROP_OUT
  ARM:HOME

Mega ACK path:
  Mega USB Serial -> Manual/sensor_bridge.py -> /tmp/apex_last_mega_ack.json

This file does NOT open Mega serial directly.
"""

import signal
import sys
import time
from typing import Optional, Dict, Any

from auto_status import AutoStatus
from auto_io import RoverIO
from auto_vision import VisionBrain

from auto_config import (
    ALIGN_TOLERANCE_PX,
    AUTO_STEP_SETTLE_SEC,
    BLOCK_GO_APPROACH_AMOUNT,
    BLOCK_GO_SMALL_AMOUNT,
    BLOCK_TURN_DEGREE,
    BOX_APPROACH_TARGET_X_RATIO,
    BOX_CLOSE_BBOX_HEIGHT_RATIO,
    CLIMB_SPEED,
    DESTINATION_BOX_CLOSE_BBOX_HEIGHT_RATIO,
    FINE_ALIGN_TOLERANCE_PX,
    FRAME_WIDTH,
    MAX_APPROACH_STEPS,
    MAX_DELIVERY_STEPS,
    MAX_OBJECTS_TO_TRANSFER,
    MAX_SEARCH_SWEEPS,
    MEGA_BLOCK_TIMEOUT_SEC,
    NORMAL_SPEED,
    STAIR_NEAR_BBOX_HEIGHT_RATIO,
    STAIR_TARGET_X_RATIO,
)


class ApexAutoOrchestrator:
    def __init__(self):
        self.status = AutoStatus()
        self.io = RoverIO(self.status)
        self.vision = VisionBrain(self.status)

        self.stop_requested = False
        self.objects_loaded = 0
        self.objects_delivered = 0

    # ============================================================
    # Global stop / safety
    # ============================================================

    def request_stop(self, reason="Stop requested"):
        self.stop_requested = True
        self.status.event(
            phase="STOP_REQUESTED",
            doing="Stopping auto scenario",
            decision=reason,
        )
        self.io.stop_all()

    def should_continue(self):
        if self.stop_requested:
            self.status.fail(
                "STOP_REQUESTED",
                "Scenario stopped by stop request",
            )
            return False

        if not self.io.safety_ok():
            return False

        return True

    # ============================================================
    # Setup
    # ============================================================

    def setup_auto(self):
        self.status.set_running(True)

        self.status.event(
            phase="SETUP",
            doing="Switching robot to AUTO mode",
            decision="send SYS:MODE:AUTO",
        )

        self.io.command("SYS:MODE:AUTO")
        self.io.command(f"SPEED:{NORMAL_SPEED}")

        if not self.io.arm_ready():
            return False

        self.status.event(
            phase="SETUP",
            doing="Auto setup complete",
            decision="ready",
        )

        return True

    def finish_auto(self, success=True):
        self.status.event(
            phase="FINISH",
            doing="Finishing auto scenario",
            decision="success" if success else "failed",
            objects_loaded=self.objects_loaded,
            objects_delivered=self.objects_delivered,
        )

        self.io.arm_home()
        self.io.stop_all()
        self.status.set_running(False)

    # ============================================================
    # Vision helpers
    # ============================================================

    def detect_once(self, category: str, camera: str = "front") -> Dict[str, Any]:
        frame = self.io.snapshot(camera)

        if frame is None:
            self.status.event(
                phase="VISION",
                doing=f"Taking {camera} snapshot",
                decision="snapshot failed",
                category=category,
            )
            return {
                "found": False,
                "category": category,
                "reason": "snapshot failed",
                "frame_w": FRAME_WIDTH,
                "frame_h": 480,
            }

        detection = self.vision.detect(frame, category)

        self.status.event(
            phase="VISION",
            doing=f"Detecting {category}",
            decision="found" if detection.get("found") else "not found",
            detection=detection,
        )

        return detection

    def detection_height_ratio(self, detection: Dict[str, Any]) -> float:
        if not detection or not detection.get("found"):
            return 0.0

        bbox = detection.get("bbox")

        if not bbox or len(bbox) < 4:
            return 0.0

        frame_h = float(detection.get("frame_h") or 480)
        bbox_h = float(bbox[3])

        if frame_h <= 0:
            return 0.0

        return bbox_h / frame_h

    def is_close_enough(self, category: str, detection: Dict[str, Any]) -> bool:
        ratio = self.detection_height_ratio(detection)

        if category == "stairs":
            needed = STAIR_NEAR_BBOX_HEIGHT_RATIO
        elif category == "destination_box":
            needed = DESTINATION_BOX_CLOSE_BBOX_HEIGHT_RATIO
        else:
            needed = BOX_CLOSE_BBOX_HEIGHT_RATIO

        close = ratio >= needed

        self.status.event(
            phase="VISION_DISTANCE",
            doing=f"Checking if {category} is close enough",
            decision="close" if close else "not close yet",
            category=category,
            height_ratio=round(ratio, 3),
            needed_ratio=needed,
        )

        return close

    def align_to_detection(
        self,
        detection: Dict[str, Any],
        target_x_ratio: float,
        fine: bool = False,
    ) -> bool:
        """
        Returns True when aligned.
        If not aligned, sends a small Mega turn block and returns False.
        """

        if not detection or not detection.get("found"):
            return False

        cx = detection.get("cx")
        frame_w = detection.get("frame_w") or FRAME_WIDTH

        if cx is None:
            return False

        target_x = frame_w * target_x_ratio
        error_px = float(cx) - float(target_x)

        tolerance = FINE_ALIGN_TOLERANCE_PX if fine else ALIGN_TOLERANCE_PX

        if abs(error_px) <= tolerance:
            self.status.event(
                phase="ALIGN",
                doing="Target aligned",
                decision="no turn needed",
                error_px=round(error_px, 1),
                tolerance=tolerance,
            )
            return True

        direction = "RIGHT" if error_px > 0 else "LEFT"

        degree = 6 if abs(error_px) < 120 else 12

        self.status.event(
            phase="ALIGN",
            doing="Aligning robot with detected target",
            decision=f"turn {direction} {degree} degrees",
            error_px=round(error_px, 1),
            tolerance=tolerance,
        )

        return self.io.block_turn(
            direction,
            degree,
            timeout_sec=35,
        ) and False

    def scan_for_target(
        self,
        category: str,
        camera: str = "front",
        scan_direction: str = "LEFT",
        required: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """
        Try to find a target with camera.
        If not found, rotate slightly using Mega block and try again.
        """

        self.status.event(
            phase="SCAN",
            doing=f"Scanning for {category}",
            decision="start scan",
            category=category,
        )

        for sweep in range(MAX_SEARCH_SWEEPS):
            if not self.should_continue():
                return None

            detection = self.detect_once(category, camera=camera)

            if detection.get("found"):
                self.status.event(
                    phase="SCAN",
                    doing=f"{category} found",
                    decision="target found",
                    category=category,
                    sweep=sweep + 1,
                    detection=detection,
                )
                return detection

            self.status.event(
                phase="SCAN",
                doing=f"{category} not found",
                decision=f"small scan turn {scan_direction}",
                category=category,
                sweep=sweep + 1,
            )

            self.io.block_turn(
                scan_direction,
                12,
                timeout_sec=35,
            )

        if required:
            self.status.fail(
                "SCAN_FAILED",
                f"Could not find required target: {category}",
                category=category,
            )

        return None

    def approach_target(
        self,
        category: str,
        target_x_ratio: float,
        close_category: Optional[str] = None,
        max_steps: int = MAX_APPROACH_STEPS,
        go_amount: int = BLOCK_GO_SMALL_AMOUNT,
    ) -> bool:
        """
        Repeatedly:
          1. take snapshot
          2. detect target
          3. align
          4. go forward a little
          5. stop when visually close
        """

        if close_category is None:
            close_category = category

        self.status.event(
            phase="APPROACH",
            doing=f"Approaching {category}",
            decision="start visual approach",
            category=category,
            max_steps=max_steps,
        )

        for step in range(max_steps):
            if not self.should_continue():
                return False

            detection = self.detect_once(category, camera="front")

            if not detection.get("found"):
                self.status.event(
                    phase="APPROACH",
                    doing=f"{category} lost during approach",
                    decision="scan small left",
                    category=category,
                    step=step + 1,
                )

                self.io.block_turn("LEFT", 8, timeout_sec=30)
                continue

            if self.is_close_enough(close_category, detection):
                self.status.event(
                    phase="APPROACH",
                    doing=f"Reached {category}",
                    decision="target close enough",
                    category=category,
                    step=step + 1,
                )
                return True

            aligned = self.align_to_detection(
                detection,
                target_x_ratio=target_x_ratio,
                fine=True,
            )

            if not aligned:
                time.sleep(AUTO_STEP_SETTLE_SEC)
                continue

            self.status.event(
                phase="APPROACH",
                doing=f"Moving forward toward {category}",
                decision=f"BLOCK:GO:FORWARD:{go_amount}",
                category=category,
                step=step + 1,
            )

            if not self.io.block_go(
                "FORWARD",
                go_amount,
                timeout_sec=45,
            ):
                return False

            time.sleep(AUTO_STEP_SETTLE_SEC)

        self.status.fail(
            "APPROACH_FAILED",
            f"Could not approach target: {category}",
            category=category,
            max_steps=max_steps,
        )
        return False

    # ============================================================
    # Arm transfer helpers
    # ============================================================

    def transfer_source_objects_to_robot_basket(self):
        """
        For each object:
          ARM:READY
          ARM:TAKE_OUT
          ARM:DROP_IN
          ARM:READY
        """

        self.status.event(
            phase="TRANSFER_TO_ROBOT_BASKET",
            doing="Starting object transfer from source box to robot basket",
            decision="use UNO arm states",
            objects_to_transfer=MAX_OBJECTS_TO_TRANSFER,
        )

        for index in range(MAX_OBJECTS_TO_TRANSFER):
            if not self.should_continue():
                return False

            item_no = index + 1

            self.status.event(
                phase="TRANSFER_TO_ROBOT_BASKET",
                doing=f"Transferring object {item_no} to robot basket",
                decision="ARM:READY -> ARM:TAKE_OUT -> ARM:DROP_IN -> ARM:READY",
                object_index=item_no,
            )

            if not self.io.arm_ready():
                return False

            if not self.io.arm_take_out():
                return False

            if not self.io.arm_drop_in():
                return False

            if not self.io.arm_ready():
                return False

            self.objects_loaded += 1
            self.status.set_counts(loaded=self.objects_loaded)

        self.status.event(
            phase="TRANSFER_TO_ROBOT_BASKET",
            doing="All objects loaded into robot basket",
            decision="done",
            objects_loaded=self.objects_loaded,
        )

        return True

    def transfer_robot_basket_to_destination_box(self):
        """
        For each object:
          ARM:READY
          ARM:TAKE_OUT
          ARM:DROP_OUT
          ARM:READY
        """

        self.status.event(
            phase="TRANSFER_TO_DESTINATION",
            doing="Starting object transfer from robot basket to destination box",
            decision="use UNO arm states",
            objects_to_transfer=self.objects_loaded,
        )

        count = self.objects_loaded

        if count <= 0:
            self.status.event(
                phase="TRANSFER_TO_DESTINATION",
                doing="No loaded objects recorded",
                decision="skip delivery",
            )
            return True

        for index in range(count):
            if not self.should_continue():
                return False

            item_no = index + 1

            self.status.event(
                phase="TRANSFER_TO_DESTINATION",
                doing=f"Delivering object {item_no} to destination box",
                decision="ARM:READY -> ARM:TAKE_OUT -> ARM:DROP_OUT -> ARM:READY",
                object_index=item_no,
            )

            if not self.io.arm_ready():
                return False

            if not self.io.arm_take_out():
                return False

            if not self.io.arm_drop_out():
                return False

            if not self.io.arm_ready():
                return False

            self.objects_delivered += 1
            self.status.set_counts(
                loaded=self.objects_loaded,
                delivered=self.objects_delivered,
            )

        self.status.event(
            phase="TRANSFER_TO_DESTINATION",
            doing="All objects delivered to destination box",
            decision="done",
            objects_delivered=self.objects_delivered,
        )

        return True

    # ============================================================
    # Scenario parts
    # ============================================================

    def part_pickup(self):
        """
        Part 1:
          - Turn left 90 degrees
          - Find source box
          - Approach it
          - Transfer objects to robot basket
          - Turn right 90 degrees
        """

        self.status.event(
            phase="PART_1_PICKUP",
            doing="Starting pickup part",
            decision="turn left to source box",
        )

        if not self.io.block_turn("LEFT", BLOCK_TURN_DEGREE, timeout_sec=60):
            return False

        source_detection = self.scan_for_target(
            "source_box",
            camera="front",
            scan_direction="LEFT",
            required=True,
        )

        if source_detection is None:
            return False

        if not self.approach_target(
            category="source_box",
            target_x_ratio=BOX_APPROACH_TARGET_X_RATIO,
            close_category="source_box",
            max_steps=MAX_APPROACH_STEPS,
            go_amount=BLOCK_GO_APPROACH_AMOUNT,
        ):
            return False

        if not self.transfer_source_objects_to_robot_basket():
            return False

        self.status.event(
            phase="PART_1_PICKUP",
            doing="Returning robot direction after pickup",
            decision="turn right 90 degrees",
        )

        if not self.io.block_turn("RIGHT", BLOCK_TURN_DEGREE, timeout_sec=60):
            return False

        return True

    def part_climb_stairs(self):
        """
        Part 2:
          - Find stairs
          - Approach stairs
          - Ask Mega to run full AUTO:UP_STAIRS scenario
          - After reaching flat top, go forward a little
        """

        self.status.event(
            phase="PART_2_CLIMB",
            doing="Starting stairs climb part",
            decision="find stairs",
        )

        stairs_detection = self.scan_for_target(
            "stairs",
            camera="front",
            scan_direction="LEFT",
            required=True,
        )

        if stairs_detection is None:
            return False

        if not self.approach_target(
            category="stairs",
            target_x_ratio=STAIR_TARGET_X_RATIO,
            close_category="stairs",
            max_steps=MAX_APPROACH_STEPS,
            go_amount=BLOCK_GO_APPROACH_AMOUNT,
        ):
            return False

        self.status.event(
            phase="PART_2_CLIMB",
            doing="Starting Mega stair climbing block",
            decision="AUTO:UP_STAIRS",
        )

        self.io.command(f"SPEED:{int(CLIMB_SPEED)}")

        if not self.io.auto_up_stairs(timeout_sec=max(180, MEGA_BLOCK_TIMEOUT_SEC)):
            return False

        self.status.event(
            phase="PART_2_CLIMB",
            doing="Robot reached upper flat area",
            decision="move forward slightly",
        )

        self.io.command(f"SPEED:{int(NORMAL_SPEED)}")

        if not self.io.block_go(
            "FORWARD",
            BLOCK_GO_SMALL_AMOUNT,
            timeout_sec=45,
        ):
            return False

        return True

    def part_delivery(self):
        """
        Part 3:
          - Turn right 90
          - Find destination box
          - Approach it
          - Deliver objects from robot basket to destination box
          - Turn right 90
        """

        self.status.event(
            phase="PART_3_DELIVERY",
            doing="Starting delivery part",
            decision="turn right to destination area",
        )

        if not self.io.block_turn("RIGHT", BLOCK_TURN_DEGREE, timeout_sec=60):
            return False

        destination_detection = self.scan_for_target(
            "destination_box",
            camera="front",
            scan_direction="LEFT",
            required=True,
        )

        if destination_detection is None:
            return False

        if not self.approach_target(
            category="destination_box",
            target_x_ratio=BOX_APPROACH_TARGET_X_RATIO,
            close_category="destination_box",
            max_steps=MAX_DELIVERY_STEPS,
            go_amount=BLOCK_GO_SMALL_AMOUNT,
        ):
            return False

        if not self.transfer_robot_basket_to_destination_box():
            return False

        self.status.event(
            phase="PART_3_DELIVERY",
            doing="Final orientation turn",
            decision="turn right 90 degrees",
        )

        if not self.io.block_turn("RIGHT", BLOCK_TURN_DEGREE, timeout_sec=60):
            return False

        return True

    # ============================================================
    # Full scenario
    # ============================================================

    def run_full_scenario(self):
        success = False

        try:
            self.status.event(
                phase="AUTO_START",
                doing="Starting full pickup -> climb -> delivery scenario",
                decision="orchestrator mode",
            )

            if not self.setup_auto():
                return False

            if not self.part_pickup():
                return False

            if not self.part_climb_stairs():
                return False

            if not self.part_delivery():
                return False

            success = True

            self.status.event(
                phase="AUTO_DONE",
                doing="Full scenario completed successfully",
                decision="done",
                objects_loaded=self.objects_loaded,
                objects_delivered=self.objects_delivered,
            )

            return True

        except Exception as e:
            self.status.fail(
                "AUTO_EXCEPTION",
                "Unhandled exception in auto scenario",
                exception=str(e),
            )
            return False

        finally:
            self.finish_auto(success=success)

    # ============================================================
    # Test modes
    # These are useful when running directly from terminal.
    # ============================================================

    def run_test_mode(self, mode):
        mode = str(mode).lower().strip()

        self.status.set_running(True)

        try:
            self.status.event(
                phase="TEST_MODE",
                doing=f"Running test mode: {mode}",
                decision="direct block test",
            )

            self.io.command("SYS:MODE:AUTO")

            if mode in ["up", "up_stairs", "auto_up", "test_up"]:
                return self.io.auto_up_stairs(timeout_sec=max(180, MEGA_BLOCK_TIMEOUT_SEC))

            if mode in ["down", "down_stairs", "auto_down", "test_down"]:
                return self.io.auto_down_stairs(timeout_sec=max(180, MEGA_BLOCK_TIMEOUT_SEC))

            if mode in ["ready", "arm_ready"]:
                return self.io.arm_ready()

            if mode in ["home", "arm_home"]:
                return self.io.arm_home()

            if mode in ["take", "take_out", "arm_take_out"]:
                return self.io.arm_take_out()

            if mode in ["drop_in", "arm_drop_in"]:
                return self.io.arm_drop_in()

            if mode in ["drop_out", "arm_drop_out"]:
                return self.io.arm_drop_out()

            if mode == "turn_left":
                return self.io.block_turn("LEFT", BLOCK_TURN_DEGREE, timeout_sec=60)

            if mode == "turn_right":
                return self.io.block_turn("RIGHT", BLOCK_TURN_DEGREE, timeout_sec=60)

            if mode == "go_forward":
                return self.io.block_go("FORWARD", BLOCK_GO_SMALL_AMOUNT, timeout_sec=45)

            if mode == "go_backward":
                return self.io.block_go("BACKWARD", BLOCK_GO_SMALL_AMOUNT, timeout_sec=45)

            self.status.fail(
                "TEST_MODE",
                f"Unknown test mode: {mode}",
            )
            return False

        finally:
            self.status.set_running(False)


ACTIVE_SCENARIO: Optional[ApexAutoOrchestrator] = None


def handle_signal(signum, frame):
    global ACTIVE_SCENARIO

    if ACTIVE_SCENARIO is not None:
        ACTIVE_SCENARIO.request_stop(f"Signal received: {signum}")

    sys.exit(0)


def main():
    global ACTIVE_SCENARIO

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    scenario = ApexAutoOrchestrator()
    ACTIVE_SCENARIO = scenario

    if len(sys.argv) > 1:
        mode = sys.argv[1]
        ok = scenario.run_test_mode(mode)
    else:
        ok = scenario.run_full_scenario()

    if ok:
        print("[AUTO] Completed successfully", flush=True)
        sys.exit(0)

    print("[AUTO] Failed", flush=True)
    sys.exit(1)


if __name__ == "__main__":
    main()