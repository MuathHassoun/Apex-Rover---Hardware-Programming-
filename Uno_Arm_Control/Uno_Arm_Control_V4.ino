#include <Servo.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Mount + ARM Placeholder
//
// SAFE VERSION
// Based on the original working behavior.
// Only small additions:
// - CAM:ENABLE
// - CAM:DISABLE
// - CAM:STATUS
//
// No stepperMode.
// No finite steps.
// No complicated start/stop logic.
// ==================================================


// ==================================================
// A4988 Stepper Driver Pins
// ==================================================
//
// Your current hardware:
// EN   -> A1
// STEP -> D12
// DIR  -> A2
//
// If your DIR is still on D13, change DIR_PIN back to 13.
// ==================================================

#define EN_PIN   A1
#define STEP_PIN 12
#define DIR_PIN  A2


// ==================================================
// Camera Servo Pin
// ==================================================

#define CAMERA_SERVO_PIN A0


// ==================================================
// Servo Settings
// ==================================================

Servo cameraServo;

int cameraAngle = 90;

const int SERVO_MIN  = 30;
const int SERVO_MAX  = 150;
const int SERVO_STEP = 5;

// If UP/DOWN is reversed, change this.
// true  = CAM:UP decreases angle, CAM:DOWN increases angle
// false = CAM:UP increases angle, CAM:DOWN decreases angle
const bool INVERT_SERVO_VERTICAL = true;


// ==================================================
// Stepper Settings
// ==================================================
//
// cameraDirection:
// -1 = rotate left
//  0 = stop
//  1 = rotate right
// ==================================================

int cameraDirection = 0;

unsigned long lastStepTime = 0;

// Smaller = faster
// Bigger  = slower
const unsigned long STEP_INTERVAL_MICROS = 1200;

// A4988 pulse width
const unsigned int STEP_PULSE_MICROS = 3;


// ==================================================
// ARM Placeholder Settings
// ==================================================

bool armEnabled = false;
String lastArmCommand = "NONE";


// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Enable A4988
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(cameraAngle);

  Serial.println("====================================");
  Serial.println("UNO Ready - Apex Rover Camera Mount SAFE");
  Serial.println("Stepper EN   : A1");
  Serial.println("Stepper STEP : D12");
  Serial.println("Stepper DIR  : A2");
  Serial.println("Servo Pin    : A0");
  Serial.println("ARM placeholder enabled");
  Serial.println("====================================");

  Serial.println("Available CAM commands:");
  Serial.println("CAM:LEFT");
  Serial.println("CAM:RIGHT");
  Serial.println("CAM:STOP");
  Serial.println("CAM:UP");
  Serial.println("CAM:DOWN");
  Serial.println("CAM:CENTER");
  Serial.println("CAM:ENABLE");
  Serial.println("CAM:DISABLE");
  Serial.println("CAM:STATUS");
}


// ==================================================
// LOOP
// ==================================================

void loop() {
  readRaspberryCommands();
  runCameraStepper();
  runArmPlaceholder();
}


// ==================================================
// Read Commands From Raspberry Pi
// ==================================================

void readRaspberryCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) {
      return;
    }

    Serial.print("RX:");
    Serial.println(command);

    if (command.startsWith("CAM:")) {
      handleCameraCommand(command);
    }

    else if (command.startsWith("ARM:")) {
      handleArmCommand(command);
    }

    else {
      Serial.print("ERROR:UNKNOWN_COMMAND:");
      Serial.println(command);
    }
  }
}


// ==================================================
// Handle Camera Commands
// ==================================================

void handleCameraCommand(String command) {

  if (command == "CAM:LEFT") {
    cameraDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:LEFT");
  }

  else if (command == "CAM:RIGHT") {
    cameraDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:RIGHT");
  }

  else if (command == "CAM:STOP") {
    cameraDirection = 0;
    digitalWrite(STEP_PIN, LOW);
    Serial.println("ACK:CAM:STOP");
  }

  else if (command == "CAM:ENABLE") {
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:ENABLE");
  }

  else if (command == "CAM:DISABLE") {
    cameraDirection = 0;
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(EN_PIN, HIGH);
    Serial.println("ACK:CAM:DISABLE");
  }

  else if (command == "CAM:UP") {
    if (INVERT_SERVO_VERTICAL) {
      cameraAngle -= SERVO_STEP;
    } else {
      cameraAngle += SERVO_STEP;
    }

    updateCameraServo();
    Serial.println("ACK:CAM:UP");
  }

  else if (command == "CAM:DOWN") {
    if (INVERT_SERVO_VERTICAL) {
      cameraAngle += SERVO_STEP;
    } else {
      cameraAngle -= SERVO_STEP;
    }

    updateCameraServo();
    Serial.println("ACK:CAM:DOWN");
  }

  else if (command == "CAM:CENTER") {
    cameraDirection = 0;
    digitalWrite(STEP_PIN, LOW);

    cameraAngle = 90;
    updateCameraServo();

    Serial.println("ACK:CAM:CENTER");
  }

  else if (command == "CAM:STATUS") {
    sendCameraStatus();
  }

  else {
    Serial.print("ERROR:UNKNOWN_CAM_COMMAND:");
    Serial.println(command);
  }
}


// ==================================================
// Run Stepper Motor Continuously
// ==================================================

void runCameraStepper() {
  if (cameraDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - lastStepTime >= STEP_INTERVAL_MICROS) {
    lastStepTime = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_MICROS);
    digitalWrite(STEP_PIN, LOW);
  }
}


// ==================================================
// Update Servo Angle
// ==================================================

void updateCameraServo() {
  cameraAngle = constrain(cameraAngle, SERVO_MIN, SERVO_MAX);
  cameraServo.write(cameraAngle);

  Serial.print("CAM:ANGLE:");
  Serial.println(cameraAngle);
}


// ==================================================
// Send Camera Status
// ==================================================

void sendCameraStatus() {
  Serial.print("DATA:CAMERA:");
  Serial.print("ANGLE=");
  Serial.print(cameraAngle);

  Serial.print(";DIR=");
  Serial.print(cameraDirection);

  Serial.print(";EN=");
  Serial.println(digitalRead(EN_PIN) == LOW ? "1" : "0");
}


// ==================================================
// ARM Command Handler - Placeholder
// ==================================================

void handleArmCommand(String command) {
  lastArmCommand = command;

  if (command == "ARM:STOP") {
    armEnabled = false;
    Serial.println("ACK:ARM:STOP");
  }

  else if (command == "ARM:HOME") {
    armEnabled = true;
    Serial.println("ACK:ARM:HOME:PLACEHOLDER");
  }

  else if (command == "ARM:STATUS") {
    sendArmStatus();
  }

  else if (command == "ARM:BASE:LEFT") {
    armEnabled = true;
    Serial.println("ACK:ARM:BASE:LEFT:PLACEHOLDER");
  }

  else if (command == "ARM:BASE:RIGHT") {
    armEnabled = true;
    Serial.println("ACK:ARM:BASE:RIGHT:PLACEHOLDER");
  }

  else if (command == "ARM:SHOULDER:UP") {
    armEnabled = true;
    Serial.println("ACK:ARM:SHOULDER:UP:PLACEHOLDER");
  }

  else if (command == "ARM:SHOULDER:DOWN") {
    armEnabled = true;
    Serial.println("ACK:ARM:SHOULDER:DOWN:PLACEHOLDER");
  }

  else if (command == "ARM:ELBOW:UP") {
    armEnabled = true;
    Serial.println("ACK:ARM:ELBOW:UP:PLACEHOLDER");
  }

  else if (command == "ARM:ELBOW:DOWN") {
    armEnabled = true;
    Serial.println("ACK:ARM:ELBOW:DOWN:PLACEHOLDER");
  }

  else if (command == "ARM:GRIPPER:OPEN") {
    armEnabled = true;
    Serial.println("ACK:ARM:GRIPPER:OPEN:PLACEHOLDER");
  }

  else if (command == "ARM:GRIPPER:CLOSE") {
    armEnabled = true;
    Serial.println("ACK:ARM:GRIPPER:CLOSE:PLACEHOLDER");
  }

  else {
    Serial.print("ERROR:UNKNOWN_ARM_COMMAND:");
    Serial.println(command);
  }
}


// ==================================================
// ARM Runtime Placeholder
// ==================================================

void runArmPlaceholder() {
  // Empty for now.
}


// ==================================================
// Send ARM Status
// ==================================================

void sendArmStatus() {
  Serial.print("DATA:ARM:");

  Serial.print("ENABLED=");
  Serial.print(armEnabled ? "1" : "0");

  Serial.print(";LAST=");
  Serial.println(lastArmCommand);
}