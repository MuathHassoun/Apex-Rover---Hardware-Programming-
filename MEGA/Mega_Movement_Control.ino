#include <Wire.h>

// ==================================================
// Arduino Mega - Apex Rover Main Control
// Mobile App -> ESP32 -> Arduino Mega
//
// This code controls:
// 1. Robot movement using 2 BTS7960 motor drivers
// 2. Normal Mode / Climb Mode
// 3. 4 IR sensors
// 4. MPU6500 / MPU6050 basic reading
// 5. Two Linear Actuators using L298N driver
//
// IMPORTANT:
// Ultrasonic sensor was removed from the code.
// ==================================================


// ==================================================
// 1) ESP32 TO MEGA CONNECTION
// ==================================================
//
// ESP32 G17 TX  --->  Mega Pin 19 RX1
// ESP32 GND     --->  Mega GND
//
// Mega receives commands from ESP32 using Serial1.
// Serial1 RX on Arduino Mega is Pin 19.
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
//
// RPWM and LPWM are PWM control pins.
// analogWrite is used to control motor speed.
// ==================================================

#define RIGHT_RPWM 9
#define RIGHT_LPWM 11

#define LEFT_RPWM 5
#define LEFT_LPWM 7


// ==================================================
// 3) IR SENSOR PINS
// ==================================================
//
// Each IR sensor has 3 pins:
//
// VCC ---> 5V
// GND ---> GND
// OUT ---> Arduino Mega input pin
//
// Front Left IR  OUT ---> Mega Pin 22
// Front Right IR OUT ---> Mega Pin 23
// Rear Left IR   OUT ---> Mega Pin 24
// Rear Right IR  OUT ---> Mega Pin 25
//
// OUT may also be written as:
// DO / DOUT / S / SIG / Signal
// ==================================================

#define IR_FRONT_LEFT   22
#define IR_FRONT_RIGHT  23
#define IR_REAR_LEFT    24
#define IR_REAR_RIGHT   25

// Most IR modules:
// LOW  = obstacle detected
// HIGH = no obstacle
//
// If your IR sensor works opposite,
// change LOW to HIGH.
#define IR_OBSTACLE_STATE LOW


// ==================================================
// 4) MPU6500 / MPU6050 CONNECTION
// ==================================================
//
// MPU SDA ---> Mega Pin 20 SDA
// MPU SCL ---> Mega Pin 21 SCL
// MPU VCC ---> 3.3V or 5V depending on your module
// MPU GND ---> GND
//
// The MPU is used to read pitch and roll.
// This can help later with stairs/climb angle.
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
// L298N is used to control two linear actuators.
//
// Rear Linear Actuator:
// L298N OUT1 / OUT2 ---> Rear Jack motor wires
//
// Front Linear Actuator:
// L298N OUT3 / OUT4 ---> Front Jack motor wires
//
// L298N control pins:
//
// Rear Jack side:
// ENA ---> Mega Pin 32
// IN1 ---> Mega Pin 30
// IN2 ---> Mega Pin 31
//
// Front Jack side:
// ENB ---> Mega Pin 35
// IN3 ---> Mega Pin 33
// IN4 ---> Mega Pin 34
//
// If jack moves opposite direction,
// swap OUT wires or swap HIGH/LOW in the function.
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

int motorSpeed = 150;           // PWM value from 0 to 255
String currentMode = "NORMAL";  // NORMAL or CLIMB
String lastMovement = "STOP";   // FORWARD, BACKWARD, LEFT, RIGHT, STOP

unsigned long lastSensorPrint = 0;
unsigned long sensorPrintInterval = 1000;


// ==================================================
// SETUP
// ==================================================

void setup() {
  // Serial Monitor for debugging
  Serial.begin(9600);

  // Serial1 receives commands from ESP32
  // ESP32 G17 TX ---> Mega Pin 19 RX1
  Serial1.begin(9600);

  // BTS7960 motor pins
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  // IR sensor pins
  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_REAR_LEFT, INPUT);
  pinMode(IR_REAR_RIGHT, INPUT);

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

  // Stop motors and jacks at startup
  stopMotors();
  stopAllJacks();

  // MPU setup
  Wire.begin();
  initMPU();

  Serial.println("====================================");
  Serial.println("Arduino Mega Ready");
  Serial.println("Waiting for commands from ESP32...");
  Serial.println("Default Mode: NORMAL");
  Serial.println("Ultrasonic removed - IR only safety");
  Serial.println("Linear actuators enabled on L298N");
  Serial.println("====================================");
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  // Read commands from ESP32
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    Serial.print("Received from ESP32: [");
    Serial.print(command);
    Serial.println("]");

    handleCommand(command);
  }

  // Safety checking for movement in NORMAL mode
  safetyMonitor();

  // Print sensor status every second
  if (millis() - lastSensorPrint >= sensorPrintInterval) {
    lastSensorPrint = millis();
    readMPU();
    printSensorStatus();
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================

void handleCommand(String command) {
  // ==================================================
  // MODE COMMANDS
  // ==================================================

  if (command == "MODE:NORMAL") {
    currentMode = "NORMAL";
    Serial.println("Mode changed to NORMAL");

    stopMotors();
    lastMovement = "STOP";
    return;
  }

  if (command == "MODE:CLIMB") {
    currentMode = "CLIMB";
    Serial.println("Mode changed to CLIMB");
    Serial.println("Climb Mode: IR obstacle safety is relaxed for stairs.");

    stopMotors();
    lastMovement = "STOP";
    return;
  }


  // ==================================================
  // JACK COMMANDS FROM APP DRIVE MODE
  // ==================================================
  //
  // Front Jack buttons send:
  // JACK:FRONT:EXTEND
  // JACK:FRONT:RETRACT
  // JACK:FRONT:STOP
  //
  // Rear Jack buttons send:
  // JACK:REAR:EXTEND
  // JACK:REAR:RETRACT
  // JACK:REAR:STOP
  // ==================================================

  if (command == "JACK:FRONT:EXTEND") {
    frontJackExtend();
    return;
  }

  if (command == "JACK:FRONT:RETRACT") {
    frontJackRetract();
    return;
  }

  if (command == "JACK:FRONT:STOP") {
    frontJackStop();
    return;
  }

  if (command == "JACK:REAR:EXTEND") {
    rearJackExtend();
    return;
  }

  if (command == "JACK:REAR:RETRACT") {
    rearJackRetract();
    return;
  }

  if (command == "JACK:REAR:STOP") {
    rearJackStop();
    return;
  }


  // ==================================================
  // MOVEMENT COMMANDS
  // ==================================================

  if (command == "FORWARD") {
    lastMovement = "FORWARD";
    Serial.println("Action: Move Forward");

    if (canMoveForward()) {
      moveForward();
    } else {
      Serial.println("Blocked: Front IR obstacle detected in NORMAL mode");
      stopMotors();
      lastMovement = "STOP";
    }
  }

  else if (command == "BACKWARD") {
    lastMovement = "BACKWARD";
    Serial.println("Action: Move Backward");

    if (canMoveBackward()) {
      moveBackward();
    } else {
      Serial.println("Blocked: Rear IR obstacle detected in NORMAL mode");
      stopMotors();
      lastMovement = "STOP";
    }
  }

  else if (command == "LEFT") {
    lastMovement = "LEFT";
    Serial.println("Action: Turn Left");
    turnLeft();
  }

  else if (command == "RIGHT") {
    lastMovement = "RIGHT";
    Serial.println("Action: Turn Right");
    turnRight();
  }

  else if (command == "STOP") {
    lastMovement = "STOP";
    Serial.println("Action: Stop");

    // BRAKE / STOP from app stops robot movement
    // and also stops both jacks for safety.
    stopMotors();
    stopAllJacks();
  }


  // ==================================================
  // SPEED COMMAND
  // ==================================================

  else if (command.startsWith("SPEED:")) {
    int speedPercent = command.substring(6).toInt();
    speedPercent = constrain(speedPercent, 0, 100);

    motorSpeed = map(speedPercent, 0, 100, 0, 255);

    Serial.print("Speed Percent: ");
    Serial.println(speedPercent);
    Serial.print("PWM Speed: ");
    Serial.println(motorSpeed);

    applyLastMovement();
  }

  else {
    Serial.println("Unknown command");
  }
}


// ==================================================
// SAFETY LOGIC
// ==================================================

bool canMoveForward() {
  // In CLIMB mode, the robot should not stop just
  // because IR detects stairs as an obstacle.
  if (currentMode == "CLIMB") {
    return true;
  }

  return !frontObstacleDetected();
}

bool canMoveBackward() {
  // In CLIMB mode, rear IR safety is relaxed too.
  if (currentMode == "CLIMB") {
    return true;
  }

  return !rearObstacleDetected();
}

void safetyMonitor() {
  // Safety only works in NORMAL mode.
  // In CLIMB mode, stairs should not be treated as a normal obstacle.

  if (currentMode != "NORMAL") {
    return;
  }

  if (lastMovement == "FORWARD" && frontObstacleDetected()) {
    Serial.println("SAFETY STOP: Front IR obstacle detected");
    stopMotors();
    lastMovement = "STOP";
  }

  if (lastMovement == "BACKWARD" && rearObstacleDetected()) {
    Serial.println("SAFETY STOP: Rear IR obstacle detected");
    stopMotors();
    lastMovement = "STOP";
  }
}

bool frontObstacleDetected() {
  bool irFrontLeft = digitalRead(IR_FRONT_LEFT) == IR_OBSTACLE_STATE;
  bool irFrontRight = digitalRead(IR_FRONT_RIGHT) == IR_OBSTACLE_STATE;

  return irFrontLeft || irFrontRight;
}

bool rearObstacleDetected() {
  bool irRearLeft = digitalRead(IR_REAR_LEFT) == IR_OBSTACLE_STATE;
  bool irRearRight = digitalRead(IR_REAR_RIGHT) == IR_OBSTACLE_STATE;

  return irRearLeft || irRearRight;
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
    if (canMoveForward()) {
      moveForward();
    } else {
      stopMotors();
      lastMovement = "STOP";
    }
  }

  else if (lastMovement == "BACKWARD") {
    if (canMoveBackward()) {
      moveBackward();
    } else {
      stopMotors();
      lastMovement = "STOP";
    }
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
//
// If an actuator moves opposite to what you expect,
// you have two options:
//
// 1. Swap the two wires on OUT1/OUT2 or OUT3/OUT4
// 2. Swap HIGH and LOW in extend/retract function
//
// Rear Jack:
// OUT1 / OUT2 on L298N
//
// Front Jack:
// OUT3 / OUT4 on L298N
// ==================================================

void rearJackExtend() {
  Serial.println("Rear Jack: EXTEND");

  digitalWrite(REAR_JACK_EN, HIGH);
  digitalWrite(REAR_JACK_IN1, HIGH);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void rearJackRetract() {
  Serial.println("Rear Jack: RETRACT");

  digitalWrite(REAR_JACK_EN, HIGH);
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, HIGH);
}

void rearJackStop() {
  Serial.println("Rear Jack: STOP");

  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, LOW);
}

void frontJackExtend() {
  Serial.println("Front Jack: EXTEND");

  digitalWrite(FRONT_JACK_EN, HIGH);
  digitalWrite(FRONT_JACK_IN3, HIGH);
  digitalWrite(FRONT_JACK_IN4, LOW);
}

void frontJackRetract() {
  Serial.println("Front Jack: RETRACT");

  digitalWrite(FRONT_JACK_EN, HIGH);
  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, HIGH);
}

void frontJackStop() {
  Serial.println("Front Jack: STOP");

  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, LOW);
}

void stopAllJacks() {
  rearJackStop();
  frontJackStop();
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
    Serial.println("MPU detected and initialized");
  } else {
    Serial.println("MPU not detected. Check SDA/SCL/VCC/GND");
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

void printSensorStatus() {
  Serial.println("----------- SENSOR STATUS -----------");

  Serial.print("Mode: ");
  Serial.println(currentMode);

  Serial.print("Last Movement: ");
  Serial.println(lastMovement);

  Serial.print("Speed PWM: ");
  Serial.println(motorSpeed);

  Serial.print("IR Front Left: ");
  Serial.println(digitalRead(IR_FRONT_LEFT));

  Serial.print("IR Front Right: ");
  Serial.println(digitalRead(IR_FRONT_RIGHT));

  Serial.print("IR Rear Left: ");
  Serial.println(digitalRead(IR_REAR_LEFT));

  Serial.print("IR Rear Right: ");
  Serial.println(digitalRead(IR_REAR_RIGHT));

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("Roll: ");
  Serial.println(roll);

  Serial.println("-------------------------------------");
}
