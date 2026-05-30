# Apex Rover Hardware Programming

This repository contains the hardware programming files for the Apex Rover robot.

## Folders

- `ESP32`: ESP32 WebSocket bridge. It receives commands from the mobile app and sends them to Mega or UNO.
- `MEGA`: Arduino Mega movement control code for BTS7960 motor drivers, IR sensors, MPU6500, Normal Mode, and Climb Mode.
- `UNO`: Arduino UNO arm control code for 1 stepper motor and 5 servo motors.
- `Raspberry`: Reserved for future Raspberry Pi camera and autonomous vision work.

## Communication Flow

### Movement System

Mobile App → ESP32 → Arduino Mega → BTS7960 → DC Motors

### Arm System

Mobile App → ESP32 → Arduino UNO → Stepper + Servo Motors

## Main Pins

### ESP32

- G17 → Mega RX1 Pin 19
- G4 → UNO Pin 2
- GND → Mega GND and UNO GND

### Arduino Mega

- Right BTS7960 RPWM → Pin 9
- Right BTS7960 LPWM → Pin 11
- Left BTS7960 RPWM → Pin 5
- Left BTS7960 LPWM → Pin 7
- IR Front Left → Pin 22
- IR Front Right → Pin 23
- IR Rear Left → Pin 24
- IR Rear Right → Pin 25
- MPU6500 SDA → Pin 20
- MPU6500 SCL → Pin 21

### Arduino UNO

- RX from ESP32 G4 → Pin 2
- Stepper EN → Pin 4
- Stepper STEP → Pin 7
- Stepper DIR → Pin 8
- Shoulder Servo → Pin 5
- Elbow Servo → Pin 6
- Wrist Servo → Pin 9
- Gripper Servo → Pin 10
- Aux Servo → Pin 11

## Modes

- `MODE:NORMAL`: normal driving with IR safety.
- `MODE:CLIMB`: stairs mode, IR safety is relaxed so stairs are not treated as normal obstacles.

## Arm Commands

Examples:

- `ARM:BASE:LEFT`
- `ARM:BASE:RIGHT`
- `ARM:BASE:STOP`
- `ARM:UP`
- `ARM:DOWN`
- `ARM:FORWARD`
- `ARM:BACK`
- `ARM:WRIST:UP`
- `ARM:WRIST:DOWN`
- `ARM:GRIPPER:OPEN`
- `ARM:GRIPPER:CLOSE`
- `ARM:HOME`
- `ARM:PICK`
- `ARM:CARRY`
- `ARM:DROP`