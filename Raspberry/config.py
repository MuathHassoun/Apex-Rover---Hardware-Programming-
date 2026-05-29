
# ============================================================
# Apex Rover Brain Configuration - V2 Fast Modular Version
# ============================================================

MEGA_PORT = "/dev/ttyUSB0"
UNO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

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
# AUTO CLIMB UP V1 SETTINGS
# ============================================================
# When True, main_brain.py starts directly in CLIMB_ASSIST after boot.
AUTO_START_CLIMB_UP = True
AUTO_ARM_DELAY_SEC = 3.0

# Yellow guide path tracking. The robot follows the yellow path while climbing.
YELLOW_PATH_MIN_AREA = 900
YELLOW_PATH_CENTER_TOLERANCE = 80
YELLOW_HSV_LOWER = (15, 60, 60)
YELLOW_HSV_UPPER = (45, 255, 255)

# Safety thresholds from MPU6500
CLIMB_ROLL_DANGER = 25.0
CLIMB_PITCH_DANGER = 32.0

# Start rear jack cycle when the front part is already on the stair.
# If your pitch sign is reversed, change this to a negative value and update condition in main if needed.
CLIMB_FRONT_ON_STEP_PITCH = 8.0

# Speeds during auto climb up
AUTO_CLIMB_ALIGN_SPEED = 45
AUTO_CLIMB_FORWARD_SPEED = 60
AUTO_CLIMB_JACK_DRIVE_SPEED = 60

# Rear jack only is used for climbing up.
# Linear actuator takes time, so we keep it running until ultrasonic target OR max time.
REAR_JACK_USE_ULTRASONIC = True

# If rear ultrasonic distance becomes smaller when jack goes down/touches ground, keep True.
# If your reading increases when jack extends, set this False.
REAR_JACK_EXTEND_STOP_WHEN_LESS_EQUAL = True
REAR_JACK_EXTEND_TARGET_CM = 6.0
REAR_JACK_EXTEND_MAX_SEC = 20.0

# If rear ultrasonic distance becomes larger when jack retracts, keep True.
# If your reading decreases when jack retracts, set this False.
REAR_JACK_RETRACT_STOP_WHEN_GREATER_EQUAL = True
REAR_JACK_RETRACT_TARGET_CM = 14.0
REAR_JACK_RETRACT_MAX_SEC = 20.0

# After rear jack lifts robot, drive forward while rear jack stays extended, then retract it.
DRIVE_WITH_REAR_JACK_SEC = 4.0
RECOVER_FORWARD_SEC = 3.0

# Detect top/landing: after at least one rear-jack cycle, pitch/roll become level and stairs disappear.
LEVEL_PITCH_ABS = 6.0
LEVEL_ROLL_ABS = 8.0
TOP_LEVEL_TIME_SEC = 2.0
NO_STAIRS_TOP_TIME_SEC = 1.5

FORWARD_ON_YELLOW_SPEED = 60

DRIVE_WITH_REAR_JACK_SPEED = 60

RECOVER_FORWARD_SPEED = 60

MIN_FORWARD_BEFORE_JACK_SEC = 1.5

VISION_LOST_FORWARD_SPEED = 60

YELLOW_LOST_FORWARD_MAX_SEC = 8.0

NO_STAIRS_FORWARD_MAX_SEC = 8.0


