# ============================================================
# Apex Rover Brain Configuration - V2 Fast Modular Version
# ============================================================

# ------------------------------------------------------------------
# ESP32 WiFi Bridge — Raspberry Pi connects to the ESP32 hotspot
# (SSID: Apex_Rover_Net, password: 12345678) and sends all commands
# through HTTP instead of USB Serial.
#
# The ESP32 default gateway on its softAP network is always 192.168.4.1.
# Change this only if you configured a custom IP on the ESP32.
# ------------------------------------------------------------------
ESP32_IP = "192.168.4.1"
ESP32_HTTP_TIMEOUT = 0.4   # seconds per HTTP request

# Legacy serial ports — kept for reference but no longer used when
# USE_ESP32_BRIDGE = True.
MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

# Set True  = route all commands through ESP32 WiFi (production)
# Set False = use USB Serial directly (debug / bench testing)
USE_ESP32_BRIDGE = True

# Faster serial reads. The old larger timeout makes the loop feel delayed.
SERIAL_TIMEOUT = 0.01
SERIAL_WRITE_TIMEOUT = 0.2

CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

MODE_IDLE = "IDLE"
MODE_MANUAL = "MANUAL"
MODE_OBJECT = "OBJECT"
MODE_STAIRS = "STAIRS"
MODE_CLIMB_ASSIST = "CLIMB_ASSIST"

DEFAULT_SPEED = 60
STAIRS_SPEED = 35
CLIMB_SPEED = 40

MAX_SAFE_PITCH = 25.0
MAX_SAFE_ROLL = 25.0

# Your camera vertical movement is reversed, so keep this True.
INVERT_CAMERA_VERTICAL = True

FRONT_JACK_TARGET_CM = 10.0
REAR_JACK_TARGET_CM = 10.0
JACK_TOLERANCE_CM = 0.5

CAPTURE_DELAY = 0.02
AUTO_COMMAND_INTERVAL = 0.12

CENTER_TOLERANCE = 70
MIN_OBJECT_AREA = 800
OBJECT_REACHED_AREA = 5000

STAIR_MIN_LINES = 4
STAIR_CENTER_TOLERANCE = 90
STAIR_TOO_CLOSE_Y = 390
STAIR_MIN_LINE_LENGTH = 80
STAIR_MAX_LINE_GAP = 25

COLOR_RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((35, 70, 50), (85, 255, 255))],
    "blue": [((90, 70, 50), (130, 255, 255))],
    "yellow": [((20, 100, 100), (35, 255, 255))],
    "orange": [((10, 100, 100), (25, 255, 255))],
    "pink": [((140, 60, 80), (170, 255, 255))],
    "purple": [((125, 60, 50), (155, 255, 255))],
    "black": [((0, 0, 0), (180, 255, 60))],
    "white": [((0, 0, 180), (180, 60, 255))]
}

# ============================================================
# Auto stair climb optimized settings - V5
# ============================================================
DEFAULT_MODE = MODE_CLIMB_ASSIST

STAIRS_ALIGN_SPEED = 28
STAIRS_APPROACH_SPEED = 38
STAIRS_CLIMB_SPEED = 50
STAIRS_SHORT_PULSE_SPEED = 45

# Camera movement must be slow in auto mode. UNO receives these once on start/mode entry.
CAMERA_STEPPER_SPEED = 450
CAMERA_STEPPER_STEPS = 25
AUTO_CAMERA_COMMAND_INTERVAL = 0.50
AUTO_CAMERA_SCAN_INTERVAL = 0.80

# Yellow track following. The robot drives on the two yellow side tracks.
YELLOW_TRACK_MIN_AREA = 1200
YELLOW_TRACK_MIN_WIDTH = 25
YELLOW_TRACK_CENTER_TOLERANCE = 55
YELLOW_TRACK_LOST_TIMEOUT = 1.00

# Approach / climb timing. Tune on the real robot.
STAIRS_APPROACH_DISTANCE_CM = 35.0
STAIRS_FRONT_TOO_CLOSE_CM = 5.0
STAIRS_REAR_TOO_CLOSE_CM = 4.0
STAIRS_STOP_BEFORE_CLIMB_TIME = 0.50
REAR_JACK_EXTEND_TIME = 1.00
REAR_JACK_RETRACT_TIME = 1.00
STAIRS_LONG_FORWARD_TIME = 1.40
STAIRS_SHORT_FORWARD_TIME = 0.60
STAIRS_REPEAT_PAUSE_TIME = 0.20

# Finish conditions.
FINISH_FLAT_PITCH_DEG = 8.0
FINISH_FRONT_CLEAR_CM = 45.0
CLIMB_DANGER_PITCH = 35.0
CLIMB_DANGER_ROLL = 25.0

# ============================================================
# Camera-based close verification - V6 smart approach
# ============================================================
# The ultrasonic sensors are pointing to the ground, so they are NOT used
# as front/rear distance-to-stair sensors.
USE_ULTRASONIC_FOR_STAIR_DISTANCE = False

# In service/boot mode it is safer to keep OpenCV windows disabled.
# Set True only when running manually from desktop with a screen.
SHOW_DEBUG_WINDOWS = False

# Camera poses are relative because the current UNO supports CAM:CENTER and CAM:DOWN.
# FORWARD view: look slightly down/forward to find and follow yellow tracks.
# GROUND view : look more down; if yellow tracks are still visible here, the robot is close.
CAMERA_FORWARD_DOWN_PULSES = 1
CAMERA_GROUND_DOWN_PULSES = 4
CAMERA_PULSE_DELAY_SEC = 0.10
CAMERA_POSE_SETTLE_TIME = 0.70

# Approach no longer depends on ultrasonic distance.
# The robot drives forward on the yellow tracks for a short time, then stops and
# verifies closeness by moving camera down to ground view.
APPROACH_FORWARD_TIME_BEFORE_GROUND_CHECK = 1.20
GROUND_VERIFY_TIMEOUT = 1.60
GROUND_CONFIRM_FRAMES = 2
GROUND_CONFIRM_MIN_CONFIDENCE = 0.60

# ============================================================
# Smart adaptive feedback layer - V7
# ============================================================
# These are NOT fixed memorized distances. They are thresholds used to decide
# whether the robot actually moved, slipped, touched ground, or got stuck.

SMART_NO_PROGRESS_GRACE_SEC = 0.90
SMART_MIN_FORWARD_STATE_TIME = 0.35
SMART_MAX_FORWARD_STATE_TIME = 2.80

SMART_PROGRESS_MIN_PITCH_DELTA = 0.9
SMART_PROGRESS_MIN_ROLL_DELTA = 1.2
SMART_PROGRESS_MIN_GROUND_DELTA = 0.7
SMART_PROGRESS_MIN_CENTER_DELTA = 8.0
SMART_PROGRESS_MIN_YELLOW_AREA_DELTA = 350.0

SMART_JACK_MIN_EXTEND_TIME = 0.45
SMART_JACK_CONTACT_TIMEOUT = 1.60
SMART_JACK_CONTACT_MIN_GROUND_DELTA = 0.6
SMART_JACK_CONTACT_MIN_PITCH_DELTA = 0.7

SMART_SLIP_ROLL_DELTA = 7.0
SMART_SLIP_PITCH_DROP = 5.0
SMART_LOST_TRACK_RECOVERY_TIMEOUT = 0.60

RECOVERY_MAX_ATTEMPTS = 3
RECOVERY_SETTLE_TIME = 0.35
RECOVERY_BACKWARD_TIME = 0.35
RECOVERY_BACKWARD_SPEED = 30
RECOVERY_TURN_SPEED = 24
RECOVERY_AFTER_BACKWARD_REALIGN_TIME = 0.60


# ============================================================
# Smart Adaptive V8 - MPU + Camera priority
# ============================================================
# The ultrasonic sensors are ground-facing, so they are low-priority clues only.
# Main decision sources are MPU6500 pitch/roll and camera vision.
ULTRASONIC_DECISION_WEIGHT = 0.15
MPU_DECISION_WEIGHT = 1.00
CAMERA_DECISION_WEIGHT = 0.85

# Camera is mounted on the RIGHT side of the robot, not in the center.
# Positive value means the robot center appears left/right shifted relative to camera center.
# Tune this on the real robot. Start around 50-80 pixels.
CAMERA_MOUNT_OFFSET_X_PIXELS = 65

# While climbing, camera must look DOWN to see stairs and yellow tracks.
# This pose is reused during jack/forward climb states.
CAMERA_CLIMB_DOWN_PULSES = 4
CAMERA_CLIMB_SETTLE_TIME = 0.50

# MPU-based jack logic:
# 1) extend jack until MPU detects body/balance effect.
# 2) after effect is detected, keep extending 3 sec.
# 3) stop jack and continue.
# Timeouts below are safety limits only, not the main logic.
JACK_EFFECT_MIN_PITCH_DELTA = 0.70
JACK_EFFECT_MIN_ROLL_DELTA = 0.70
JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC = 3.00
JACK_EFFECT_DETECT_TIMEOUT_SEC = 4.00
JACK_ABSOLUTE_MAX_EXTEND_SEC = 8.00

# Progress scoring in V8: MPU + camera dominate; ultrasonic alone should not pass.
SMART_PROGRESS_REQUIRED_SCORE = 2.20
SMART_JACK_EFFECT_REQUIRED_SCORE = 1.00


# ============================================================
# Smart Adaptive V9 - Ultrasonic helper with minimum delta/rate
# ============================================================
# Keep the ground-facing ultrasonic sensors, but do NOT let them decide alone.
# They are helper clues with a minimum accepted change/rate to reject noise.
USE_ULTRASONIC_AS_HELPER = True

# Ignore tiny ultrasonic changes smaller than this; they are usually noise.
ULTRASONIC_MIN_DELTA_CM = 0.80

# Ignore slow/tiny ultrasonic drift. Count it only if the change rate is high enough.
ULTRASONIC_MIN_RATE_CM_PER_SEC = 0.60

# Decision weights. MPU and camera dominate; ultrasonic is a small helper.
MPU_PROGRESS_WEIGHT = 3.00
CAMERA_PROGRESS_WEIGHT = 2.00
ULTRASONIC_PROGRESS_WEIGHT = 1.00

# Required score for real progress.
# MPU alone can pass. Camera + ultrasonic can pass. Ultrasonic alone cannot pass.
SMART_PROGRESS_SCORE_THRESHOLD = 3.00

# Jack effect decision: MPU is the main proof; ultrasonic only supports weak MPU evidence.
MPU_JACK_EFFECT_WEIGHT = 3.00
ULTRASONIC_JACK_EFFECT_WEIGHT = 1.00
JACK_EFFECT_SCORE_THRESHOLD = 3.00
WEAK_MPU_JACK_EFFECT_PITCH_DELTA = 0.35
WEAK_MPU_JACK_EFFECT_ROLL_DELTA = 0.35
