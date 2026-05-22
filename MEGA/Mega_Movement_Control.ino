
@'
#include <Wire.h>

// ==================================================
// Arduino Mega - Apex Rover Main Control
// Mobile App -> ESP32 -> Arduino Mega
//
// Features:
// - 2 BTS7960 motor drivers
// - Normal / Climb Mode
// - 4 IR sensors
// - MPU6500 / MPU6050 basic reading
// - Ultrasonic removed
// ==================================================

// ================= Motor Driver Pins =================

// Right side BTS7960
#define RIGHT_RPWM 9
#define RIGHT_LPWM 11

// Left side BTS7960
#define LEFT_RPWM 5
#define LEFT_LPWM 7

// ================= IR Sensor Pins =================

#define IR_FRONT_LEFT   22
#define IR_FRONT_RIGHT  23
#define IR_REAR_LEFT    24
#define IR_REAR_RIGHT   25

// ================= IR Logic =================
// Most IR modules:
// LOW = obstacle detected
// HIGH = no obstacle
// If your IR works opposite, change LOW to HIGH.
#define IR_OBSTACLE_STATE LOW

// ================= MPU6500 Settings =================

#define MPU_ADDR 0x68

float accelX = 0;
float accelY = 0;
float accelZ = 0;
float pitch = 0;
float roll = 0;

// ================= Robot State =================

int motorSpeed = 150;           // 0 - 255
String currentMode = "NORMAL";  // NORMAL or CLIMB
String lastMovement = "STOP";   // FORWARD, BACKWARD, LEFT, RIGHT, STOP

unsigned long lastSensorPrint = 0;
unsigned long sensorPrintInterval = 1000;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);   // ESP32 G17 -> Mega RX1 Pin 19

  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_REAR_LEFT, INPUT);
  pinMode(IR_REAR_RIGHT, INPUT);

  Wire.begin();
  initMPU();

  stopMotors();

  Serial.println("====================================");
  Serial.println("Arduino Mega Ready");
  Serial.println("Waiting for commands from ESP32...");
  Serial.println("Default Mode: NORMAL");
  Serial.println("Ultrasonic removed - IR only safety");
  Serial.println("====================================");
}

void loop() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    Serial.print("Received from ESP32: [");
    Serial.print(command);
    Serial.println("]");

    handleCommand(command);
  }

  safetyMonitor();

  if (millis() - lastSensorPrint >= sensorPrintInterval) {
    lastSensorPrint = millis();
    readMPU();
    printSensorStatus();
  }
}

void handleCommand(String command) {
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
    stopMotors();
  }

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

bool canMoveForward() {
  if (currentMode == "CLIMB") {
    return true;
  }

  return !frontObstacleDetected();
}

bool canMoveBackward() {
  if (currentMode == "CLIMB") {
    return true;
  }

  return !rearObstacleDetected();
}

void safetyMonitor() {
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
'@ | Set-Content -Encoding UTF8 MEGA\Mega_Movement_Control.ino
