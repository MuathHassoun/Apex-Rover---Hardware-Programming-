Apex Rover Brain - Smart Adaptive V9 Ultrasonic Rate Helper
==========================================================

Changes from V8:

1. Ultrasonic sensors are NOT removed. They remain enabled as helper sensors.
2. Ultrasonic values no longer decide alone. They enter decisions through a score.
3. Ultrasonic changes count only after minimum delta/rate filtering:
   - ULTRASONIC_MIN_DELTA_CM
   - ULTRASONIC_MIN_RATE_CM_PER_SEC
4. MPU6500 and camera still dominate decisions:
   - MPU_PROGRESS_WEIGHT = 3.0
   - CAMERA_PROGRESS_WEIGHT = 2.0
   - ULTRASONIC_PROGRESS_WEIGHT = 1.0
5. Progress rule:
   - MPU alone can confirm progress.
   - Camera + ultrasonic can confirm progress.
   - Ultrasonic alone cannot confirm progress.
6. Jack effect rule:
   - MPU remains the main proof.
   - Ultrasonic can help only with weak MPU evidence.
   - Ultrasonic alone cannot confirm jack effect.

Important config values:

USE_ULTRASONIC_AS_HELPER = True
ULTRASONIC_MIN_DELTA_CM = 0.80
ULTRASONIC_MIN_RATE_CM_PER_SEC = 0.60
MPU_PROGRESS_WEIGHT = 3.00
CAMERA_PROGRESS_WEIGHT = 2.00
ULTRASONIC_PROGRESS_WEIGHT = 1.00
SMART_PROGRESS_SCORE_THRESHOLD = 3.00

Run:
cd ~/ApexRoverBrain
python3 -m py_compile *.py
python3 main_brain.py
