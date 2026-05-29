Apex Rover Brain - Smart Adaptive V7
====================================

Goal
----
This version is NOT a blind memorized stair script.
The stair scenario is still organized, but every important transition now asks for real feedback:

- Did the robot actually move?
- Did the MPU6500 pitch/roll change as expected?
- Did ground ultrasonic values change?
- Did the camera/yellow-track scene change?
- Did the rear jack produce contact/action feedback?
- Did the robot slip, drift, or lose the yellow tracks?

New file
--------
smart_observer.py

This file watches sensor/vision changes and produces evidence for:
- progress_detected()
- jack_contact_detected()
- slip_or_drift_detected()
- progress_score()

Important idea
--------------
Timers are now safety caps and pacing aids, not the main reason to continue.
The robot should not continue from one climb stage to the next unless it has feedback.

Main smart recovery states
--------------------------
RECOVERY_STUCK
RECOVERY_SLIP
RECOVERY_LOST_TRACK
RECOVERY_JACK_NO_CONTACT
SAFE_STOP

Examples
--------
If motors are FORWARD but pitch/ground/camera do not change:
    RECOVERY_STUCK

If rear jack extends but ground ultrasonic/MPU do not change:
    RECOVERY_JACK_NO_CONTACT

If roll changes suddenly:
    RECOVERY_SLIP

If yellow tracks disappear while approaching:
    RECOVERY_LOST_TRACK

Config tuning
-------------
The new smart thresholds are at the bottom of config.py under:
Smart adaptive feedback layer - V7

Start tuning with these first:
- SMART_NO_PROGRESS_GRACE_SEC
- SMART_PROGRESS_MIN_PITCH_DELTA
- SMART_PROGRESS_MIN_GROUND_DELTA
- SMART_JACK_CONTACT_TIMEOUT
- SMART_JACK_CONTACT_MIN_GROUND_DELTA
- SMART_SLIP_ROLL_DELTA
- RECOVERY_MAX_ATTEMPTS

Boot service note
-----------------
For boot/systemd keep SHOW_DEBUG_WINDOWS = False.
If service starts too early, add ExecStartPre=/bin/sleep 15 in apex-brain.service.
