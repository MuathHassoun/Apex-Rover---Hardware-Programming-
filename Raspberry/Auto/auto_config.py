#!/usr/bin/env python3
"""
auto_config.py
Configuration for Apex Rover automatic pickup -> climb stairs -> delivery scenario.

Put this file inside:
  Raspberry/Auto/auto_config.py

Images/templates should be placed in:
  Raspberry/src/

Recommended template file names:
  source_box_1.jpg, box_front.png
  stairs_1.jpg, stair_model.png
  destination_box_1.jpg, top_box.png
  object_1.jpg, object_2.jpg, object_3.jpg
  robot_basket.jpg
"""

from pathlib import Path

# ============================================================
# Paths
# ============================================================
AUTO_DIR = Path(__file__).resolve().parent
RASPBERRY_DIR = AUTO_DIR.parent
SRC_DIR = RASPBERRY_DIR / "src"

STATUS_FILE = "/tmp/apex_auto_status.json"
EVENTS_FILE = "/tmp/apex_auto_status_events.jsonl"
LAST_SENSOR_FILE = "/tmp/apex_last_sensor.json"

# ============================================================
# ESP32 bridge
# ============================================================
ESP32_IP = "192.168.4.1"
ESP32_HTTP_COMMAND_URL = f"http://{ESP32_IP}/command"
ESP32_COMMAND_TIMEOUT = 0.8

# ============================================================
# Camera devices / server
# These defaults match the manual camera_admin_server.py defaults.
# If your config.py has different values, front_camera_server.py imports them.
# ============================================================
CAMERA_SERVER_HOST = "0.0.0.0"
CAMERA_SERVER_PORT = 5000

FRONT_CAMERA_DEVICE = "/dev/v4l/by-id/usb-GENERAL_2K_HD_Camera-video-index0"
ARM_CAMERA_DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_A59D7FAF-video-index0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
JPEG_QUALITY = 70

LOCAL_FRONT_SNAPSHOT_URL = f"http://127.0.0.1:{CAMERA_SERVER_PORT}/front_snapshot"
LOCAL_ARM_SNAPSHOT_URL = f"http://127.0.0.1:{CAMERA_SERVER_PORT}/arm_snapshot"
LOCAL_AUTO_STATUS_URL = f"http://127.0.0.1:{CAMERA_SERVER_PORT}/auto_status"

# ============================================================
# Movement tuning
# ============================================================
NORMAL_SPEED = 55
CLIMB_SPEED = 60
ARM_OPERATION_SPEED = 35

# Small timed movements. Mega constrains PULSE duration between 50 and 1500 ms.
PULSE_FORWARD_MS = 420
PULSE_FORWARD_SLOW_MS = 250
PULSE_TURN_MS = 180
PULSE_TURN_SMALL_MS = 120
PULSE_BACKWARD_MS = 250

COMMAND_GAP_SEC = 0.08
SNAPSHOT_SETTLE_SEC = 0.18

# If the detected object center is away from target by more than this, align first.
ALIGN_TOLERANCE_PX = 55
FINE_ALIGN_TOLERANCE_PX = 32

# The robot front camera is on the right side, not robot center.
# For stair centering, the visual center of the stairs should be slightly LEFT
# of the image center so the robot body center, not the camera, is centered.
STAIR_TARGET_X_RATIO = 0.42

# For source/destination boxes, we want the robot left side close to the box
# because the arm is on the left side of the front upper body.
BOX_APPROACH_TARGET_X_RATIO = 0.38

# Basic distance/size heuristics. These are not hard-coded real distances;
# they are visual thresholds you can tune after testing.
BOX_CLOSE_BBOX_HEIGHT_RATIO = 0.36
STAIR_NEAR_BBOX_HEIGHT_RATIO = 0.42
DESTINATION_BOX_CLOSE_BBOX_HEIGHT_RATIO = 0.34

# Box height rule: reject a box that appears too high in the frame.
# This is a safety/heuristic interpretation of:
# "box allowed at robot horizontal level max; above that rejected".
# Tune using your real scene images.
MAX_ACCEPTED_BOX_TOP_Y_RATIO = 0.25

# Ultrasonic is only safety gating; auto decisions should mainly use vision + MPU.
FRONT_ULTRASONIC_HARD_STOP_CM = 9.0
FRONT_ULTRASONIC_CAUTION_CM = 16.0

# ============================================================
# MPU / climb tuning
# ============================================================
PITCH_DANGER_DEG = 30.0
ROLL_DANGER_DEG = 24.0
PITCH_CLIMB_START_DEG = 8.0
PITCH_FLAT_TOP_DEG = 6.0
ROLL_SAFE_FOR_CLIMB_DEG = 15.0

# Used to detect whether rear jack has affected robot balance.
JACK_EFFECT_DELTA_DEG = 2.0
JACK_MAX_EXTEND_SEC = 8.0
JACK_AFTER_EFFECT_SEC = 2.5
JACK_MAX_RETRACT_SEC = 8.0

# If repeated forward pulses do not change pitch/vision enough, use jack assist.
NO_PROGRESS_LIMIT = 4

# ============================================================
# Scenario limits
# ============================================================
MAX_OBJECTS_TO_TRANSFER = 3
MAX_SEARCH_SWEEPS = 3
MAX_APPROACH_STEPS = 28
MAX_CLIMB_STEPS = 80
MAX_DELIVERY_STEPS = 28

# ============================================================
# Template matching / vision
# ============================================================
TEMPLATE_MIN_SCORE = 0.48
TEMPLATE_GOOD_SCORE = 0.62
TEMPLATE_SCALES = [0.35, 0.45, 0.55, 0.70, 0.85, 1.00, 1.15, 1.30]

# Categories are inferred from file names in src/.
TEMPLATE_KEYWORDS = {
    "source_box": ["source_box", "start_box", "pickup_box", "box_source", "box"],
    "destination_box": ["destination_box", "dest_box", "target_box", "top_box", "delivery_box"],
    "stairs": ["stairs", "stair", "steps", "staircase", "daraj"],
    "object": ["object", "item", "thing", "cargo", "load"],
    "robot_basket": ["robot_basket", "basket_robot", "my_basket", "rear_basket"],
}

# ============================================================
# Arm calibrated poses
# ============================================================
# These poses are intentionally easy to tune. The code performs the logic,
# but real arm angles must be calibrated on your robot.
# Units:
#   base_deg: degrees used by ARM:BASE:GOTO_DEG:N
#   shoulder/elbow/wrist/gripper/aux: servo angles
# ============================================================
ARM_POSE_READY = {
    "base_deg": 0,
    "shoulder": 90,
    "elbow": 35,
    "wrist": 90,
    "aux": 120,
    "gripper": 180,
}

ARM_POSE_SOURCE_PICK_CENTER = {
    "base_deg": 0,
    "shoulder": 125,
    "elbow": 70,
    "wrist": 95,
    "aux": 125,
    "gripper": 180,
}

ARM_POSE_SOURCE_LIFT = {
    "base_deg": 0,
    "shoulder": 95,
    "elbow": 45,
    "wrist": 95,
    "aux": 125,
    "gripper": 120,
}

# Robot basket is behind the arm/camera, upper rear body.
ARM_POSE_ROBOT_BASKET_DROP = {
    "base_deg": 120,
    "shoulder": 110,
    "elbow": 55,
    "wrist": 85,
    "aux": 120,
    "gripper": 120,
}

ARM_POSE_ROBOT_BASKET_PICK = {
    "base_deg": 120,
    "shoulder": 125,
    "elbow": 70,
    "wrist": 90,
    "aux": 125,
    "gripper": 180,
}

ARM_POSE_DESTINATION_DROP = {
    "base_deg": 0,
    "shoulder": 120,
    "elbow": 65,
    "wrist": 90,
    "aux": 125,
    "gripper": 120,
}

ARM_SERVO_SETTLE_SEC = 0.45
ARM_BASE_SETTLE_SEC = 1.2
GRIPPER_SETTLE_SEC = 0.45
