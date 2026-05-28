#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Main Control V2
//
// Supports:
// 1. Raspberry Pi -> Mega using USB Serial
// 2. ESP32 -> Mega using Serial1
// 3. Robot movement using 2 BTS7960 motor drivers
// 4. Normal Mode / Climb Mode
// 5. MPU6500 / MPU6050 pitch and roll
// 6. Two ultrasonic sensors for jack height / distance
// 7. Two linear actuators using L298N
//
// V2 Updates for Raspberry Pi Brain V3:
// - Fast Serial timeout to reduce lag
// - Reduced debug spam on USB Serial
// - GET:SENSORS response stays clean:
//     DATA:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20
// - Supports both command styles:
//     FORWARD
//     MOVE:FORWARD
// - Supports:
//     STOP
//     CMD:STOP
//     JACK:ALL:STOP
// - ESP32 can forward brain commands to Raspberry Pi:
//     MODE:OBJECT
//     MODE:STAIRS
//     MODE:CLIMB_ASSIST
//     MODE:MANUAL
//     MODE:IDLE
//     TARGET:red
//     CMD:STOP
// ==================================================


// ==================================================
// 1) SERIAL CONNECTIONS
// ==================================================
//
// Raspberry Pi USB ---> Arduino Mega USB
// Uses Serial
//
// ESP32 G17 TX ---> Mega Pin 19 RX1
// ESP32 GND    ---> Mega GND
// Uses Serial1
// ==================================================


// ==================================================
// 2) BTS7960 MOTOR DRIVER PINS
// ==================================================
//
// Right Side BTS7960:
// RPWM ---> Mega Pin 9
// LPWM ---> Mega Pin 11
//
// Left Side BTS7960:
// RPWM ---> Mega Pin 5
// LPWM ---> Mega Pin 7
// ==================================================

#define RIGHT_RPWM 9
#define RIGHT_LPWM 11

#define LEFT_RPWM 5
#define LEFT_LPWM 7


// ==================================================
// 3) ULTRASONIC SENSOR PINS
// ==================================================
//
// Front Ultrasonic:
// TRIG ---> Mega Pin A8
// ECHO ---> Mega Pin A9
//
// Rear / Back Ultrasonic:
// TRIG ---> Mega Pin A11
// ECHO ---> Mega Pin A12
// ==================================================

#define FRONT_US_TRIG A8
#define FRONT_US_ECHO A9

#define REAR_US_TRIG A11
#define REAR_US_ECHO A12


// ==================================================
// 4) MPU6500 / MPU6050 CONNECTION
// ==================================================
//
// MPU SDA ---> Mega Pin 20 SDA
// MPU SCL ---> Mega Pin 21 SCL
// MPU VCC ---> 3.3V or 5V depending on your module
// MPU GND ---> GND
// ==================================================

#define MPU_ADDR 0x68

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float pitch = 0;
float roll = 0;


// ==================================================
// 5) L298N LINEAR ACTUATOR PINS
// ==================================================
//
// Rear Jack:
// IN1 ---> Mega Pin A0
// IN2 ---> Mega Pin A1
//
// Front Jack:
// IN3 ---> Mega Pin A3
// IN4 ---> Mega Pin A4
//
// EN pins are already enabled by hardware.
// ==================================================

#define REAR_JACK_IN1 A0
#define REAR_JACK_IN2 A1

#define FRONT_JACK_IN3 A3
#define FRONT_JACK_IN4 A4


// ==================================================
// 6) ROBOT STATE
// ==================================================

int motorSpeed = 150;           // PWM value from 0 to 255
int motorSpeedPercent = 60;     // 0 to 100

String currentMode = "NORMAL";  // NORMAL or CLIMB
String lastMovement = "STOP";   // FORWARD, BACKWARD, LEFT, RIGHT, STOP

float frontUltrasonicCM = -1.0;
float rearUltrasonicCM = -1.0;

unsigned long lastSensorUpdate = 0;
unsigned long sensorUpdateInterval = 120;

unsigned long lastDebugPrint = 0;
unsigned long debugPrintInterval = 2000;

// Keep false during Raspberry Pi autonomous tests to avoid polluting USB Serial.
bool debugToUSB = false;

// Send debug to ESP32 Serial1 instead of USB if needed.
bool debugToESP32 = false;


// ==================================================
// SETUP
// ==================================================

void setup() {
  // USB Serial for Raspberry Pi and Serial Monitor
  Serial.begin(9600);
  Serial.setTimeout(10);

  // Serial1 for ESP32
  Serial1.begin(9600);
  Serial1.setTimeout(10);

  // BTS7960 motor pins
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  // Ultrasonic pins
  pinMode(FRONT_US_TRIG, OUTPUT);
  pinMode(FRONT_US_ECHO, INPUT);

  pinMode(REAR_US_TRIG, OUTPUT);
  pinMode(REAR_US_ECHO, INPUT);

  digitalWrite(FRONT_US_TRIG, LOW);
  digitalWrite(REAR_US_TRIG, LOW);

  // L298N jack control pins
  pinMode(REAR_JACK_IN1, OUTPUT);
  pinMode(REAR_JACK_IN2, OUTPUT);

  pinMode(FRONT_JACK_IN3, OUTPUT);
  pinMode(FRONT_JACK_IN4, OUTPUT);

  // Stop everything at startup
  stopMotors();
  stopAllJacks();

  // MPU setup
  Wire.begin();
  initMPU();

  delay(500);

  Serial.println("MEGA:READY");
  Serial.println("MEGA:VERSION:APEX_ROVER_MEGA_V2");
  Serial.println("MEGA:USB_FOR_RASPBERRY_PI");
  Serial.println("MEGA:ESP32_ON_SERIAL1");

  Serial1.println("MEGA:READY");
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  readRaspberryCommand();
  readESP32Command();

  updateSensorsPeriodically();

  if (debugToUSB || debugToESP32) {
    if (millis() - lastDebugPrint >= debugPrintInterval) {
      lastDebugPrint = millis();
      printDebugStatus();
    }
  }
}


// ==================================================
// READ COMMANDS
// ==================================================

void readRaspberryCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial, "RASPBERRY_PI");
    }
  }
}


void readESP32Command() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial1, "ESP32");
    }
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================

void handleCommand(String command, Stream &replyPort, String sourceName) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  // If ESP32 sends brain-level commands, forward them to Raspberry Pi.
  // Raspberry Pi main_brain.py reads MODE/TARGET/CMD from Mega USB Serial.
  if (sourceName == "ESP32") {
    if (
      command.startsWith("MODE:") ||
      command.startsWith("TARGET:") ||
      command.startsWith("CMD:")
    ) {
      Serial.println(command);
      replyPort.print("ACK:FORWARDED_TO_PI:");
      replyPort.println(command);
      return;
    }
  }

  // Accept MOVE:FORWARD style from Raspberry Pi or tests.
  if (command.startsWith("MOVE:")) {
    command = command.substring(5);
    command.trim();
  }

  // --------------------------
  // SENSOR REQUEST
  // --------------------------

  if (command == "GET:SENSORS") {
    sendSensorData(replyPort);
    return;
  }

  // --------------------------
  // DEBUG COMMANDS
  // --------------------------

  if (command == "DEBUG:USB:ON") {
    debugToUSB = true;
    replyPort.println("ACK:DEBUG:USB:ON");
    return;
  }

  if (command == "DEBUG:USB:OFF") {
    debugToUSB = false;
    replyPort.println("ACK:DEBUG:USB:OFF");
    return;
  }

  if (command == "DEBUG:ESP32:ON") {
    debugToESP32 = true;
    replyPort.println("ACK:DEBUG:ESP32:ON");
    return;
  }

  if (command == "DEBUG:ESP32:OFF") {
    debugToESP32 = false;
    replyPort.println("ACK:DEBUG:ESP32:OFF");
    return;
  }

  // --------------------------
  // MODE COMMANDS
  // --------------------------

  if (command == "MODE:NORMAL") {
    currentMode = "NORMAL";
    stopMotors();
    lastMovement = "STOP";

    replyPort.println("ACK:MODE:NORMAL");
    return;
  }

  if (command == "MODE:CLIMB") {
    currentMode = "CLIMB";
    stopMotors();
    lastMovement = "STOP";

    replyPort.println("ACK:MODE:CLIMB");
    return;
  }

  // --------------------------
  // JACK COMMANDS
  // --------------------------

  if (command == "JACK:FRONT:EXTEND") {
    frontJackExtend();
    replyPort.println("ACK:JACK:FRONT:EXTEND");
    return;
  }

  if (command == "JACK:FRONT:RETRACT") {
    frontJackRetract();
    replyPort.println("ACK:JACK:FRONT:RETRACT");
    return;
  }

  if (command == "JACK:FRONT:STOP") {
    frontJackStop();
    replyPort.println("ACK:JACK:FRONT:STOP");
    return;
  }

  if (command == "JACK:REAR:EXTEND") {
    rearJackExtend();
    replyPort.println("ACK:JACK:REAR:EXTEND");
    return;
  }

  if (command == "JACK:REAR:RETRACT") {
    rearJackRetract();
    replyPort.println("ACK:JACK:REAR:RETRACT");
    return;
  }

  if (command == "JACK:REAR:STOP") {
    rearJackStop();
    replyPort.println("ACK:JACK:REAR:STOP");
    return;
  }

  if (command == "JACK:ALL:STOP" || command == "JACK:STOP" || command == "JSTOP") {
    stopAllJacks();
    replyPort.println("ACK:JACK:ALL:STOP");
    return;
  }

  // --------------------------
  // MOVEMENT COMMANDS
  // --------------------------

  if (command == "FORWARD") {
    lastMovement = "FORWARD";
    moveForward();
    replyPort.println("ACK:FORWARD");
    return;
  }

  if (command == "BACKWARD") {
    lastMovement = "BACKWARD";
    moveBackward();
    replyPort.println("ACK:BACKWARD");
    return;
  }

  if (command == "LEFT") {
    lastMovement = "LEFT";
    turnLeft();
    replyPort.println("ACK:LEFT");
    return;
  }

  if (command == "RIGHT") {
    lastMovement = "RIGHT";
    turnRight();
    replyPort.println("ACK:RIGHT");
    return;
  }

  if (command == "STOP" || command == "CMD:STOP" || command == "ESTOP") {
    lastMovement = "STOP";
    stopMotors();
    stopAllJacks();

    replyPort.println("ACK:STOP");
    return;
  }

  // --------------------------
  // SPEED COMMAND
  // --------------------------

  if (command.startsWith("SPEED:")) {
    int speedPercent = command.substring(6).toInt();
    speedPercent = constrain(speedPercent, 0, 100);

    motorSpeedPercent = speedPercent;
    motorSpeed = map(speedPercent, 0, 100, 0, 255);

    replyPort.print("ACK:SPEED:");
    replyPort.println(speedPercent);

    applyLastMovement();
    return;
  }

  // --------------------------
  // STATUS COMMAND
  // --------------------------

  if (command == "STATUS") {
    sendStatus(replyPort);
    return;
  }

  // --------------------------
  // UNKNOWN COMMAND
  // --------------------------

  replyPort.print("ERROR:UNKNOWN_COMMAND:");
  replyPort.println(command);
}


// ==================================================
// SENSOR UPDATE / SENSOR DATA RESPONSE
// ==================================================

void updateSensorsPeriodically() {
  if (millis() - lastSensorUpdate < sensorUpdateInterval) {
    return;
  }

  lastSensorUpdate = millis();

  readMPU();

  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);
}


void sendSensorData(Stream &replyPort) {
  readMPU();

  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);

  replyPort.print("DATA:");
  replyPort.print("PITCH=");
  replyPort.print(pitch, 2);

  replyPort.print(";ROLL=");
  replyPort.print(roll, 2);

  replyPort.print(";UF=");
  replyPort.print(frontUltrasonicCM, 2);

  replyPort.print(";UR=");
  replyPort.println(rearUltrasonicCM, 2);
}


void sendStatus(Stream &replyPort) {
  replyPort.print("DATA:STATUS:");
  replyPort.print("MODE=");
  replyPort.print(currentMode);

  replyPort.print(";MOVE=");
  replyPort.print(lastMovement);

  replyPort.print(";SPEED=");
  replyPort.print(motorSpeedPercent);

  replyPort.print(";PITCH=");
  replyPort.print(pitch, 2);

  replyPort.print(";ROLL=");
  replyPort.print(roll, 2);

  replyPort.print(";UF=");
  replyPort.print(frontUltrasonicCM, 2);

  replyPort.print(";UR=");
  replyPort.println(rearUltrasonicCM, 2);
}


// ==================================================
// ROBOT MOVEMENT FUNCTIONS
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
  }

  else if (lastMovement == "BACKWARD") {
    moveBackward();
  }

  else if (lastMovement == "LEFT") {
    turnLeft();
  }

  else if (lastMovement == "RIGHT") {
    turnRight();
  }

  else {
    stopMotors();
  }
}


// ==================================================
// LINEAR ACTUATOR FUNCTIONS - L298N
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
// ULTRASONIC SENSOR FUNCTIONS
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

  float distanceCM = duration * 0.0343 / 2.0;
  return distanceCM;
}


// ==================================================
// MPU6500 / MPU6050 BASIC READING
// ==================================================

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1 register
  Wire.write(0x00);   // Wake up MPU
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("MPU:OK");
  } else {
    Serial.println("MPU:ERROR");
  }
}


void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H register
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() >= 6) {
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();

    accelX = rawAx / 16384.0;
    accelY = rawAy / 16384.0;
    accelZ = rawAz / 16384.0;

    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    roll = atan2(-accelX, accelZ) * 180.0 / PI;
  }
}


// ==================================================
// DEBUG PRINTING
// ==================================================

void printDebugStatus() {
  Stream *out = nullptr;

  if (debugToUSB) {
    out = &Serial;
  }

  else if (debugToESP32) {
    out = &Serial1;
  }

  else {
    return;
  }

  out->println("----------- MEGA STATUS -----------");

  out->print("Mode: ");
  out->println(currentMode);

  out->print("Last Movement: ");
  out->println(lastMovement);

  out->print("Speed Percent: ");
  out->println(motorSpeedPercent);

  out->print("Speed PWM: ");
  out->println(motorSpeed);

  out->print("Pitch: ");
  out->println(pitch);

  out->print("Roll: ");
  out->println(roll);

  out->print("Front Ultrasonic cm: ");
  out->println(frontUltrasonicCM);

  out->print("Rear Ultrasonic cm: ");
  out->println(rearUltrasonicCM);

  out->println("-----------------------------------");
}
