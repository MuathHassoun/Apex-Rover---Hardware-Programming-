Apex Rover Brain - Smart Adaptive V8 MPU + Camera Priority
==========================================================

Main idea
---------
This version reduces the effect of ultrasonic readings on decisions because
UF/UR are ground-facing sensors, not front distance sensors.

Main decision sources:
1) MPU6500 pitch/roll/balance changes
2) Camera vision/yellow tracks/stair view
3) Ground ultrasonic only as weak extra clue

What changed in V8
------------------
1. Ultrasonic values are low-priority clues only.
   They should not decide by themselves whether the robot progressed or the jack worked.

2. MPU6500 has high priority.
   Progress and jack effect are detected mainly from pitch/roll changes.

3. Camera has high priority.
   Yellow path, path center, yellow area, and visual changes are used with MPU.

4. Camera is treated as right-side mounted.
   The robot center in the image is compensated using:
   CAMERA_MOUNT_OFFSET_X_PIXELS

5. Camera must look down while climbing.
   During climb/jack/forward states the code uses CLIMB_DOWN pose to see stairs and yellow path.

6. Jack extension is MPU-based.
   Rear jack keeps extending until MPU detects that the robot balance/body was affected.
   After MPU effect is detected, the jack keeps extending for 3 extra seconds, then stops.

7. Time is used as safety only.
   Timers are now protection caps, not the main reason to continue.

Important config values
-----------------------
CAMERA_MOUNT_OFFSET_X_PIXELS = 65
Tune this because the camera is mounted on the right side.

CAMERA_CLIMB_DOWN_PULSES = 4
How far the camera looks down while climbing.

JACK_EFFECT_MIN_PITCH_DELTA = 0.70
JACK_EFFECT_MIN_ROLL_DELTA = 0.70
MPU change required to say the jack affected the robot.

JACK_EXTRA_EXTEND_AFTER_EFFECT_SEC = 3.00
After MPU detects jack effect, keep extending jack for this many seconds.

JACK_EFFECT_DETECT_TIMEOUT_SEC = 4.00
Safety only. If jack extends and MPU never changes, enter recovery.

SMART_PROGRESS_REQUIRED_SCORE = 2.20
Progress needs MPU/camera evidence. Ultrasonic alone cannot pass.

Expected logs
-------------
[JACK SMART] rear effect detected by MPU; extra 3.0s
REAR_JACK_MPU_EFFECT_DETECTED_EXTRA_EXTEND
REAR_JACK_STOP_AFTER_MPU_EFFECT_PLUS_3SEC
LONG_FORWARD_PROGRESS_CONFIRMED

Run
---
cd ~/ApexRoverBrain
python3 -m py_compile *.py
python3 main_brain.py

Service
-------
sudo systemctl daemon-reload
sudo systemctl restart apex-brain.service
journalctl -u apex-brain.service -f
