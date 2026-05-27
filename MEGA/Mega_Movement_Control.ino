#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Main Control
//
// Supports:
// 1. Raspberry Pi -> Mega using USB Serial
// 2. ESP32 -> Mega using Serial1
// 3. Robot movement using 2 BTS7960 motor drivers
// 4. Normal Mode / Climb Mode
// 5. MPU6500 / MPU6050 pitch and roll
// 6. Two ultrasonic sensors for jack height
// 7. Two linear actuators using L298N
//
// Raspberry Pi command example:
// GET:SENSORS
//
// Mega response example:
// DATA:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20
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
// Front ultrasonic:
// TRIG ---> Mega Pin 22
// ECHO ---> Mega Pin 23
//
// Rear ultrasonic:
// TRIG ---> Mega Pin 24
// ECHO ---> Mega Pin 25
// ==================================================

#define FRONT_US_TRIG 22
#define FRONT_US_ECHO 23

#define REAR_US_TRIG 24
#define REAR_US_ECHO 25


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
// ENA ---> Mega Pin 32
// IN1 ---> Mega Pin 30
// IN2 ---> Mega Pin 31
//
// Front Jack:
// ENB ---> Mega Pin 35
// IN3 ---> Mega Pin 33
// IN4 ---> Mega Pin 34
// ==================================================

#define REAR_JACK_EN  32
#define REAR_JACK_IN1 30
#define REAR_JACK_IN2 31

#define FRONT_JACK_EN  35
#define FRONT_JACK_IN3 33
#define FRONT_JACK_IN4 34


// ==================================================
// 6) ROBOT STATE
// ==================================================

int motorSpeed = 150;           // PWM value 0 to 255
String currentMode = "NORMAL";  // NORMAL or CLIMB
String lastMovement = "STOP";   // FORWARD, BACKWARD, LEFT, RIGHT, STOP

float frontUltrasonicCM = 0;
float rearUltrasonicCM = 0;

unsigned long lastDebugPrint = 0;
unsigned long debugPrintInterval = 1500;


// ==================================================
// SETUP
// ==================================================

void setup() {
  // USB Serial for Raspberry Pi and Serial Monitor
  Serial.begin(9600);

  // Serial1 for ESP32
  Serial1.begin(9600);

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

  // L298N jack control pins
  pinMode(REAR_JACK_EN, OUTPUT);
  pinMode(REAR_JACK_IN1, OUTPUT);
  pinMode(REAR_JACK_IN2, OUTPUT);

  pinMode(FRONT_JACK_EN, OUTPUT);
  pinMode(FRONT_JACK_IN3, OUTPUT);
  pinMode(FRONT_JACK_IN4, OUTPUT);

  // Enable L298N channels
  digitalWrite(REAR_JACK_EN, HIGH);
  digitalWrite(FRONT_JACK_EN, HIGH);

  // Stop everything at startup
  stopMotors();
  stopAllJacks();

  // MPU setup
  Wire.begin();
  initMPU();

  delay(500);

  Serial.println("====================================");
  Serial.println("Arduino Mega Ready - Apex Rover");
  Serial.println("USB Serial: Raspberry Pi");
  Serial.println("Serial1: ESP32");
  Serial.println("IR removed");
  Serial.println("Ultrasonic enabled for jack height");
  Serial.println("====================================");

  Serial1.println("MEGA:READY");
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  // Commands from Raspberry Pi through USB Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial, "RASPBERRY_PI");
    }
  }

  // Commands from ESP32 through Serial1
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial1, "ESP32");
    }
  }

  // Keep sensor values updated
  readMPU();
  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);

  // Optional debug printing
  if (millis() - lastDebugPrint >= debugPrintInterval) {
    lastDebugPrint = millis();
    printDebugStatus();
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================

void handleCommand(String command, Stream &replyPort, String sourceName) {
  Serial.print("Received from ");
  Serial.print(sourceName);
  Serial.print(": [");
  Serial.print(command);
  Serial.println("]");

  // --------------------------
  // SENSOR REQUEST
  // --------------------------

  if (command == "GET:SENSORS") {
    sendSensorData(replyPort);
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

  if (command == "STOP") {
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

    motorSpeed = map(speedPercent, 0, 100, 0, 255);

    replyPort.print("ACK:SPEED:");
    replyPort.println(speedPercent);

    applyLastMovement();
    return;
  }

  // --------------------------
  // UNKNOWN COMMAND
  // --------------------------

  replyPort.print("ERROR:UNKNOWN_COMMAND:");
  replyPort.println(command);
}


// ==================================================
// SENSOR DATA RESPONSE
// ==================================================

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
  digitalWrite(REAR_JACK_EN, HIGH);
  digitalWrite(REAR_JACK_IN1, HIGH);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void rearJackRetract() {
  digitalWrite(REAR_JACK_EN, HIGH);
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, HIGH);
}

void rearJackStop() {
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void frontJackExtend() {
  digitalWrite(FRONT_JACK_EN, HIGH);
  digitalWrite(FRONT_JACK_IN3, HIGH);
  digitalWrite(FRONT_JACK_IN4, LOW);
}

void frontJackRetract() {
  digitalWrite(FRONT_JACK_EN, HIGH);
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

  long duration = pulseIn(echoPin, HIGH, 30000);

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
  Wire.write(0x6B);
  Wire.write(0x00);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("MPU detected and initialized");
  } else {
    Serial.println("MPU not detected. Check SDA/SCL/VCC/GND");
  }
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
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
  Serial.println("----------- MEGA STATUS -----------");

  Serial.print("Mode: ");
  Serial.println(currentMode);

  Serial.print("Last Movement: ");
  Serial.println(lastMovement);

  Serial.print("Speed PWM: ");
  Serial.println(motorSpeed);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("Roll: ");
  Serial.println(roll);

  Serial.print("Front Ultrasonic cm: ");
  Serial.println(frontUltrasonicCM);

  Serial.print("Rear Ultrasonic cm: ");
  Serial.println(rearUltrasonicCM);

  Serial.println("-----------------------------------");
}