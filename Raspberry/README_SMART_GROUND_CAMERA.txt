Apex Rover Brain - V6 Smart Ground-Camera Confirmation
=====================================================

Main idea:
- Manual mode works, so hardware movement is OK.
- Ultrasonic sensors are pointing to the ground, so they are NOT used as front distance to the stairs.
- The robot uses camera FORWARD view to search, align, and approach yellow stair tracks.
- Before starting the rear jack climbing sequence, the robot stops and moves the camera to GROUND view.
- If the yellow tracks are still visible in GROUND view, the robot is very close to the stairs and starts climbing.
- If yellow tracks are not visible in GROUND view, the robot returns to FORWARD view and continues approaching.

Default mode:
- DEFAULT_MODE = CLIMB_ASSIST

New state flow:
SEARCH_STAIRS
ALIGN_WITH_YELLOW_TRACKS
APPROACH_STAIRS
VERIFY_CLOSE_WITH_CAMERA_DOWN
STOP_BEFORE_CLIMB
REAR_JACK_EXTEND
LONG_FORWARD_PULSE
REAR_JACK_RETRACT
SHORT_FORWARD_PULSE_1
REAR_JACK_EXTEND_AGAIN
SHORT_FORWARD_PULSE_2
REAR_JACK_RETRACT_AGAIN
CHECK_CONTINUE_OR_FINISH
FINISH_CLIMB

Important tuning in config.py:
CAMERA_FORWARD_DOWN_PULSES = 1
CAMERA_GROUND_DOWN_PULSES = 4
APPROACH_FORWARD_TIME_BEFORE_GROUND_CHECK = 1.20
GROUND_VERIFY_TIMEOUT = 1.60
GROUND_CONFIRM_FRAMES = 2

For boot service:
SHOW_DEBUG_WINDOWS = False

For manual desktop debugging with OpenCV windows and keyboard:
SHOW_DEBUG_WINDOWS = True
