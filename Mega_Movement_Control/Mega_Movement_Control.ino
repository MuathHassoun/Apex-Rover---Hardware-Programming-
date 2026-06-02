#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Motors + Jacks + Sensors
//
// Manual path:
//   Mobile App -> ESP32 -> Mega / UNO
//
// Auto path:
//   Raspberry Auto Brain -> ESP32 -> Mega / UNO
//
// Mega responsibilities:
//   1. Receive movement/jack commands from ESP32 on Serial1.
//   2. Drive motors and front/rear linear actuators.
//   3. Read MPU6500 fast.
//   4. Send SENSOR lines to Raspberry Pi over USB Serial.
//
// Sensor reporting:
//   MANUAL mode: 1 report every 5 seconds.
//   AUTO mode:   2 reports every second.
//   ALERT:       urgent report without waiting for normal interval.
//
// Notes:
//   - Raspberry does NOT send commands directly to Mega.
//   - Mega receives commands from ESP32 only.
//   - Ultrasonic values are still reported, but auto climb should not depend on them.
// ==================================================


// ==================================================
// MOTOR DRIVER PINS (BTS7960)
// ==================================================
#define RIGHT_RPWM  9
#define RIGHT_LPWM 11
#define LEFT_RPWM   5
#define LEFT_LPWM   7


// ==================================================
// ULTRASONIC PINS
// ==================================================
#define FRONT_US_TRIG A8
#define FRONT_US_ECHO A9
#define REAR_US_TRIG  A11
#define REAR_US_ECHO  A12


// ==================================================
// MPU6500 I2C
// Mega SDA = 20
// Mega SCL = 21
// ==================================================
#define MPU_ADDR 0x68

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float pitch = 0.0;
float roll  = 0.0;


// ==================================================
// JACK PINS (L298N)
// Rear Jack:  IN1=A0 IN2=A1
// Front Jack: IN3=A3 IN4=A4
// ==================================================
#define REAR_JACK_IN1  A0
#define REAR_JACK_IN2  A1
#define FRONT_JACK_IN3 A3
#define FRONT_JACK_IN4 A4


// ==================================================
// ROBOT STATE
// ==================================================
int motorSpeed = 150;
int motorSpeedPercent = 60;

String currentMode = "NORMAL";
String systemMode = "MANUAL";
String lastMovement = "STOP";

float frontUltrasonicCM = -1.0;
float rearUltrasonicCM  = -1.0;


// ==================================================
// SENSOR TIMING
// ==================================================
const unsigned long MANUAL_SENSOR_REPORT_INTERVAL = 5000;
const unsigned long AUTO_SENSOR_REPORT_INTERVAL   = 500;

const unsigned long MPU_READ_INTERVAL             = 50;
const unsigned long US_READ_INTERVAL              = 700;
const unsigned long ALERT_COOLDOWN_MS             = 700;

unsigned long lastPeriodicReport = 0;
unsigned long lastAlertSent      = 0;
unsigned long lastMPURead        = 0;
unsigned long lastUSRead         = 0;


// ==================================================
// TIMED MOVEMENT PULSE SUPPORT
//
// Supported commands from ESP32:
//   PULSE:FORWARD:560
//   PULSE:BACKWARD:300
//   PULSE:LEFT:180
//   PULSE:RIGHT:180
//
// Mega automatically stops motors when pulse time ends.
// ==================================================
bool timedMoveActive = false;
unsigned long timedMoveEndAt = 0;


// ==================================================
// ALERT THRESHOLDS
// ==================================================
const float PITCH_DANGER_DEG  = 30.0;
const float ROLL_DANGER_DEG   = 25.0;
const float OBSTACLE_FRONT_CM = 10.0;
const float OBSTACLE_REAR_CM  = 8.0;


// ==================================================
// SETUP
// ==================================================
void setup() {
  // USB Serial -> Raspberry Pi
  Serial.begin(9600);
  Serial.setTimeout(10);

  // Serial1 <- ESP32
  Serial1.begin(9600);
  Serial1.setTimeout(10);

  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM,  OUTPUT);
  pinMode(LEFT_LPWM,  OUTPUT);

  pinMode(FRONT_US_TRIG, OUTPUT);
  pinMode(FRONT_US_ECHO, INPUT);
  pinMode(REAR_US_TRIG,  OUTPUT);
  pinMode(REAR_US_ECHO,  INPUT);

  digitalWrite(FRONT_US_TRIG, LOW);
  digitalWrite(REAR_US_TRIG, LOW);

  pinMode(REAR_JACK_IN1, OUTPUT);
  pinMode(REAR_JACK_IN2, OUTPUT);
  pinMode(FRONT_JACK_IN3, OUTPUT);
  pinMode(FRONT_JACK_IN4, OUTPUT);

  stopMotors();
  stopAllJacks();

  Wire.begin();
  initMPU();

  delay(500);

  Serial.println("MEGA:READY");
  Serial.println("MEGA:SENSOR_MODE_DEPENDENT_STREAM_ENABLED");
  Serial.println("MEGA:MANUAL_SENSOR_INTERVAL_5000MS");
  Serial.println("MEGA:AUTO_SENSOR_INTERVAL_500MS");
  Serial.println("MEGA:ALERT_URGENT_REPORT_ENABLED");
  Serial.println("MEGA:PULSE_COMMANDS_ENABLED");
  Serial.println("MEGA:COMMANDS_FROM_ESP32_ONLY");

  Serial1.println("MEGA:READY");
}


// ==================================================
// LOOP
// ==================================================
void loop() {
  readESP32Command();
  updateTimedMovement();
  updateAndReportSensors();
}


// ==================================================
// READ COMMANDS FROM ESP32 ONLY
// ==================================================
void readESP32Command() {
  if (Serial1.available() <= 0) {
    return;
  }

  String command = Serial1.readStringUntil('\n');
  command.trim();

  if (command.length() > 0) {
    handleCommand(command);
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================
void handleCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.startsWith("MOVE:")) {
    command = command.substring(5);
    command.trim();
  }

  // ---- System mode ----
  if (command == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    timedMoveActive = false;
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:MANUAL");
    sendStatusToPI();
    return;
  }

  if (command == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    timedMoveActive = false;
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:AUTO");
    sendStatusToPI();
    return;
  }

  // ---- Robot mode ----
  if (command == "MODE:NORMAL") {
    currentMode = "NORMAL";
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:MODE:NORMAL");
    sendStatusToPI();
    return;
  }

  if (command == "MODE:CLIMB") {
    currentMode = "CLIMB";
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:MODE:CLIMB");
    sendStatusToPI();
    return;
  }

  // ---- Rear jack ----
  if (command == "JACK:REAR:EXTEND") {
    rearJackExtend();
    Serial1.println("ACK:JACK:REAR:EXTEND");
    return;
  }

  if (command == "JACK:REAR:RETRACT") {
    rearJackRetract();
    Serial1.println("ACK:JACK:REAR:RETRACT");
    return;
  }

  if (command == "JACK:REAR:STOP") {
    rearJackStop();
    Serial1.println("ACK:JACK:REAR:STOP");
    return;
  }

  // ---- Front jack ----
  if (command == "JACK:FRONT:EXTEND") {
    frontJackExtend();
    Serial1.println("ACK:JACK:FRONT:EXTEND");
    return;
  }

  if (command == "JACK:FRONT:RETRACT") {
    frontJackRetract();
    Serial1.println("ACK:JACK:FRONT:RETRACT");
    return;
  }

  if (command == "JACK:FRONT:STOP") {
    frontJackStop();
    Serial1.println("ACK:JACK:FRONT:STOP");
    return;
  }

  if (command == "JACK:ALL:STOP" || command == "JACK:STOP" || command == "JSTOP") {
    stopAllJacks();
    Serial1.println("ACK:JACK:ALL:STOP");
    return;
  }

  // ---- Timed pulse movement ----
  if (command.startsWith("PULSE:")) {
    handlePulseCommand(command);
    return;
  }

  // ---- Normal movement ----
  if (command == "FORWARD") {
    timedMoveActive = false;
    lastMovement = "FORWARD";
    moveForward();
    Serial1.println("ACK:FORWARD");
    return;
  }

  if (command == "BACKWARD") {
    timedMoveActive = false;
    lastMovement = "BACKWARD";
    moveBackward();
    Serial1.println("ACK:BACKWARD");
    return;
  }

  if (command == "LEFT") {
    timedMoveActive = false;
    lastMovement = "LEFT";
    turnLeft();
    Serial1.println("ACK:LEFT");
    return;
  }

  if (command == "RIGHT") {
    timedMoveActive = false;
    lastMovement = "RIGHT";
    turnRight();
    Serial1.println("ACK:RIGHT");
    return;
  }

  if (command == "STOP" || command == "CMD:STOP" || command == "ESTOP") {
    timedMoveActive = false;
    lastMovement = "STOP";
    stopMotors();
    stopAllJacks();
    Serial1.println("ACK:STOP");
    return;
  }

  // ---- Speed ----
  if (command.startsWith("SPEED:")) {
    int spd = command.substring(6).toInt();
    spd = constrain(spd, 0, 100);

    motorSpeedPercent = spd;
    motorSpeed = map(spd, 0, 100, 0, 255);

    Serial1.print("ACK:SPEED:");
    Serial1.println(spd);

    applyLastMovement();
    return;
  }

  // ---- Status ----
  if (command == "STATUS") {
    sendStatusToPI();
    Serial1.println("ACK:STATUS:SENT_TO_PI");
    return;
  }

  Serial1.print("ERROR:UNKNOWN_COMMAND:");
  Serial1.println(command);
}


// ==================================================
// PULSE COMMAND HANDLER
// ==================================================
void handlePulseCommand(String command) {
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);

  if (firstColon < 0 || secondColon < 0) {
    Serial1.print("ERROR:BAD_PULSE:");
    Serial1.println(command);
    return;
  }

  String dir = command.substring(firstColon + 1, secondColon);
  String msText = command.substring(secondColon + 1);

  dir.trim();
  dir.toUpperCase();
  msText.trim();

  int durationMs = msText.toInt();
  durationMs = constrain(durationMs, 50, 1500);

  timedMoveActive = true;
  timedMoveEndAt = millis() + durationMs;

  if (dir == "FORWARD") {
    lastMovement = "FORWARD";
    moveForward();
    Serial1.print("ACK:PULSE:FORWARD:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "BACKWARD") {
    lastMovement = "BACKWARD";
    moveBackward();
    Serial1.print("ACK:PULSE:BACKWARD:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "LEFT") {
    lastMovement = "LEFT";
    turnLeft();
    Serial1.print("ACK:PULSE:LEFT:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "RIGHT") {
    lastMovement = "RIGHT";
    turnRight();
    Serial1.print("ACK:PULSE:RIGHT:");
    Serial1.println(durationMs);
    return;
  }

  timedMoveActive = false;
  stopMotors();
  lastMovement = "STOP";

  Serial1.print("ERROR:UNKNOWN_PULSE_DIR:");
  Serial1.println(dir);
}


// ==================================================
// TIMED MOVEMENT UPDATE
// ==================================================
void updateTimedMovement() {
  if (!timedMoveActive) {
    return;
  }

  if ((long)(millis() - timedMoveEndAt) >= 0) {
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:PULSE:DONE");
  }
}


// ==================================================
// SENSOR REPORTING
// Manual: 1 report every 5 seconds
// Auto:   2 reports every second
// Alert:  urgent report without waiting for normal interval
// ==================================================
void updateAndReportSensors() {
  unsigned long now = millis();

  if (now - lastMPURead >= MPU_READ_INTERVAL) {
    lastMPURead = now;
    readMPU();
  }

  if (now - lastUSRead >= US_READ_INTERVAL) {
    lastUSRead = now;
    frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
    rearUltrasonicCM  = readUltrasonicCM(REAR_US_TRIG,  REAR_US_ECHO);
  }

  String alert = detectAlert();
  bool alertActive = (alert != "NONE");

  if (alertActive && (now - lastAlertSent >= ALERT_COOLDOWN_MS)) {
    lastAlertSent = now;
    lastPeriodicReport = now;
    sendSensorData(alert);
    return;
  }

  unsigned long reportInterval;

  if (systemMode == "AUTO") {
    reportInterval = AUTO_SENSOR_REPORT_INTERVAL;
  } else {
    reportInterval = MANUAL_SENSOR_REPORT_INTERVAL;
  }

  if (now - lastPeriodicReport >= reportInterval) {
    lastPeriodicReport = now;
    sendSensorData("NONE");
  }
}


// ==================================================
// ALERT DETECTION
// ==================================================
String detectAlert() {
  if (fabs(pitch) > PITCH_DANGER_DEG) {
    return "TILT_DANGER";
  }

  if (fabs(roll) > ROLL_DANGER_DEG) {
    return "TILT_DANGER";
  }

  if (frontUltrasonicCM > 0 && frontUltrasonicCM < OBSTACLE_FRONT_CM) {
    return "OBSTACLE_FRONT";
  }

  if (rearUltrasonicCM > 0 && rearUltrasonicCM < OBSTACLE_REAR_CM) {
    return "OBSTACLE_REAR";
  }

  return "NONE";
}


// ==================================================
// SENSOR OUTPUT TO RASPBERRY PI
// ==================================================
void sendSensorData(String alertType) {
  Serial.print("SENSOR:");
  Serial.print("SYS=");
  Serial.print(systemMode);

  Serial.print(";MODE=");
  Serial.print(currentMode);

  Serial.print(";MOVE=");
  Serial.print(lastMovement);

  Serial.print(";SPEED=");
  Serial.print(motorSpeedPercent);

  Serial.print(";PITCH=");
  Serial.print(pitch, 2);

  Serial.print(";ROLL=");
  Serial.print(roll, 2);

  Serial.print(";UF=");
  Serial.print(frontUltrasonicCM, 2);

  Serial.print(";UR=");
  Serial.print(rearUltrasonicCM, 2);

  Serial.print(";ALERT=");
  Serial.println(alertType);
}

void sendStatusToPI() {
  Serial.print("STATUS:");
  Serial.print("SYS=");
  Serial.print(systemMode);

  Serial.print(";MODE=");
  Serial.print(currentMode);

  Serial.print(";MOVE=");
  Serial.print(lastMovement);

  Serial.print(";SPEED=");
  Serial.print(motorSpeedPercent);

  Serial.print(";PITCH=");
  Serial.print(pitch, 2);

  Serial.print(";ROLL=");
  Serial.print(roll, 2);

  Serial.print(";UF=");
  Serial.print(frontUltrasonicCM, 2);

  Serial.print(";UR=");
  Serial.println(rearUltrasonicCM, 2);
}


// ==================================================
// MOTOR FUNCTIONS
// ==================================================
void moveForward() {
  analogWrite(RIGHT_RPWM, motorSpeed);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, motorSpeed);
  analogWrite(LEFT_LPWM, 0);
}

void moveBackward() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, motorSpeed);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, motorSpeed);
}

void turnLeft() {
  analogWrite(RIGHT_RPWM, motorSpeed);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, motorSpeed);
}

void turnRight() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, motorSpeed);

  analogWrite(LEFT_RPWM, motorSpeed);
  analogWrite(LEFT_LPWM, 0);
}

void stopMotors() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);
}

void applyLastMovement() {
  if (lastMovement == "FORWARD") {
    moveForward();
  } else if (lastMovement == "BACKWARD") {
    moveBackward();
  } else if (lastMovement == "LEFT") {
    turnLeft();
  } else if (lastMovement == "RIGHT") {
    turnRight();
  } else {
    stopMotors();
  }
}


// ==================================================
// JACK FUNCTIONS
// ==================================================
void rearJackExtend() {
  digitalWrite(REAR_JACK_IN1, HIGH);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void rearJackRetract() {
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, HIGH);
}

void rearJackStop() {
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void frontJackExtend() {
  digitalWrite(FRONT_JACK_IN3, HIGH);
  digitalWrite(FRONT_JACK_IN4, LOW);
}

void frontJackRetract() {
  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, HIGH);
}

void frontJackStop() {
  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, LOW);
}

void stopAllJacks() {
  rearJackStop();
  frontJackStop();
}


// ==================================================
// ULTRASONIC
// ==================================================
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}


// ==================================================
// MPU6500
// ==================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  byte err = Wire.endTransmission();

  if (err == 0) {
    Serial.println("MPU:OK");
  } else {
    Serial.println("MPU:ERROR");
  }
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() < 6) {
    return;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  accelX = rawAx / 16384.0;
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  roll  = atan2(-accelX, accelZ) * 180.0 / PI;
}