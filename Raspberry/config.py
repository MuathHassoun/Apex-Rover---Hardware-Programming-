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
