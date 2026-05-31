# config.py
# ============================================================
# Apex Rover Raspberry Pi Configuration
# Manual-only system
# Raspberry Pi responsibilities:
# 1. Camera streaming
# 2. Sensor relay from Mega USB Serial to ESP32 HTTP
# No autonomous driving
# ============================================================

# ESP32 Access Point IP
ESP32_IP = "192.168.4.1"
ESP32_SENSOR_UPDATE_URL = f"http://{ESP32_IP}/sensor_update"
ESP32_HTTP_TIMEOUT = 0.5

# Mega USB Serial
# إذا طلع عندكم /dev/ttyACM1 بدل /dev/ttyACM0 عدّلها
MEGA_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.5
SERIAL_WRITE_TIMEOUT = 0.2

# Camera server
RASPBERRY_IP = "192.168.4.2"
CAMERA_SERVER_HOST = "0.0.0.0"
CAMERA_SERVER_PORT = 5000

FRONT_CAMERA_DEVICE = "/dev/v4l/by-id/usb-GENERAL_2K_HD_Camera-video-index0"
ARM_CAMERA_DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_A59D7FAF-video-index0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
JPEG_QUALITY = 70

# Sensor relay timing
SENSOR_FORWARD_TIMEOUT = 0.5

# Balance thresholds
PITCH_WARNING_DEG = 15.0
ROLL_WARNING_DEG = 12.0

PITCH_DANGER_DEG = 30.0
ROLL_DANGER_DEG = 25.0
