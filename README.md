# Apex Rover Hardware Programming

This repository contains the hardware programming files for the Apex Rover robot.

## Folders

- `ESP32`: Central WiFi bridge. Receives commands from the mobile app via WebSocket and from the Raspberry Pi via HTTP, then routes them to the Mega or UNO over Serial.
- `Mega_Movement_Control`: Arduino Mega code for BTS7960 motor drivers, L298N linear actuator jacks, MPU6500 IMU, and ultrasonic sensors. Receives commands from ESP32 only. Pushes sensor data to the Raspberry Pi over USB Serial automatically.
- `Uno_Arm_Control`: Arduino UNO code for the camera stand (A4988 stepper + servo). Receive-only — no data is sent back.
- `Raspberry`: Raspberry Pi sensor relay. Reads sensor data pushed by the Mega over USB Serial and forwards it to the ESP32, which broadcasts it to the mobile app via WebSocket.

---

## Communication Flow

### Command Flow (Mobile → Robot)
```
Mobile App
    │
    │  WebSocket (port 81)
    ▼
  ESP32
    ├──── Serial2 (TX GPIO17) ──────► Arduino Mega   (motors, jacks)
    └──── Serial1 (TX GPIO4)  ──────► Arduino UNO    (camera stand)
```

### Sensor Flow (Robot → Mobile)
```
Arduino Mega
    │
    │  USB Serial (automatic push every 10s, or instantly on critical alert)
    ▼
Raspberry Pi
    │
    │  HTTP POST /sensor  (WiFi — Apex_Rover_Net)
    ▼
  ESP32
    │
    │  WebSocket broadcast
    ▼
Mobile App
```

### Sensor Alert Types
| Alert | Trigger |
|---|---|
| `NONE` | Normal periodic report |
| `TILT_DANGER` | Pitch > 30° or Roll > 25° |
| `OBSTACLE_FRONT` | Front ultrasonic < 10 cm |
| `OBSTACLE_REAR` | Rear ultrasonic < 8 cm |

Sensor data format from Mega:
```
SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
```

---

## Modes

| Mode | Description |
|---|---|
| `MANUAL` | Default. Mobile app controls the robot. Raspberry Pi commands blocked. |
| `AUTO` | Reserved for future use. |

Mode is switched by sending `SYS:MODE:MANUAL` or `SYS:MODE:AUTO` from the mobile app.

---

## Main Pins

### ESP32
| Pin | Connection |
|---|---|
| GPIO17 (TX) | Mega RX1 — Pin 19 |
| GPIO4  (TX) | UNO D2 (SoftwareSerial RX) |
| GPIO16 (RX) | Mega TX1 — Pin 18 (not currently used) |
| GPIO15 (RX) | UNO D3 (not currently wired) |
| GND | Mega GND and UNO GND |

### Arduino Mega
| Pin | Connection |
|---|---|
| RX1 — Pin 19 | ESP32 GPIO17 TX (receives commands) |
| USB | Raspberry Pi (sends sensor data) |
| Pin 9  | Right BTS7960 RPWM |
| Pin 11 | Right BTS7960 LPWM |
| Pin 5  | Left BTS7960 RPWM |
| Pin 7  | Left BTS7960 LPWM |
| A8 | Front Ultrasonic TRIG |
| A9 | Front Ultrasonic ECHO |
| A11 | Rear Ultrasonic TRIG |
| A12 | Rear Ultrasonic ECHO |
| Pin 20 (SDA) | MPU6500 SDA |
| Pin 21 (SCL) | MPU6500 SCL |
| A0 | Rear Jack L298N IN1 |
| A1 | Rear Jack L298N IN2 |
| A3 | Front Jack L298N IN3 |
| A4 | Front Jack L298N IN4 |

### Arduino UNO
| Pin | Connection |
|---|---|
| D2 | ESP32 GPIO4 TX (SoftwareSerial RX — receives commands) |
| A0 | Servo signal (camera tilt) |
| A1 | A4988 EN |
| A2 | A4988 DIR |
| D12 | A4988 STEP |

---

## Movement Commands (Mega via ESP32)

| Command | Action |
|---|---|
| `FORWARD` | Move forward |
| `BACKWARD` | Move backward |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `STOP` | Stop motors and jacks |
| `SPEED:N` | Set speed (0–100%) |
| `JACK:FRONT:EXTEND` | Extend front jack |
| `JACK:FRONT:RETRACT` | Retract front jack |
| `JACK:FRONT:STOP` | Stop front jack |
| `JACK:REAR:EXTEND` | Extend rear jack |
| `JACK:REAR:RETRACT` | Retract rear jack |
| `JACK:REAR:STOP` | Stop rear jack |
| `JACK:ALL:STOP` | Stop all jacks |
| `MODE:NORMAL` | Normal driving mode |
| `MODE:CLIMB` | Stair climb mode |
| `SYS:MODE:MANUAL` | Switch system to MANUAL |
| `SYS:MODE:AUTO` | Switch system to AUTO |

---

## Camera Stand Commands (UNO via ESP32)

| Command | Action |
|---|---|
| `CAM:LEFT` | Rotate stepper left continuously |
| `CAM:RIGHT` | Rotate stepper right continuously |
| `CAM:STOP` | Stop stepper |
| `CAM:STEP_LEFT:N` | Move N steps left then stop |
| `CAM:STEP_RIGHT:N` | Move N steps right then stop |
| `CAM:UP` | Tilt servo up |
| `CAM:DOWN` | Tilt servo down |
| `CAM:CENTER` | Center servo and stop stepper |
| `CAM:ZERO` | Reset step position counter to 0 |
| `CAM:SPEED:N` | Set step interval in microseconds (300–5000) |

---

## Always-Allowed Commands

These commands are executed immediately regardless of mode or source:

`SYS:MODE:MANUAL` · `SYS:MODE:AUTO` · `STOP` · `ESTOP` · `CMD:STOP` ·
`JACK:FRONT:STOP` · `JACK:REAR:STOP` · `JACK:ALL:STOP` · `JACK:STOP` ·
`CAM:STOP` · `ARM:BASE:STOP`

---

## WiFi

| Setting | Value |
|---|---|
| SSID | `Apex_Rover_Net` |
| Password | `12345678` |
| ESP32 AP IP | `192.168.4.1` |
| WebSocket | `ws://192.168.4.1:81` |
| HTTP | `http://192.168.4.1` |

### ESP32 HTTP Endpoints

| Method | Endpoint | Description |
|---|---|---|
| GET | `/` | Status check |
| GET | `/get_status` | Returns current mode as JSON |
| POST | `/command?cmd=...` | Send always-allowed commands from Pi |
| POST | `/sensor?data=...` | Pi forwards Mega sensor data |
| GET | `/get_last_sensor` | Poll latest sensor reading |
