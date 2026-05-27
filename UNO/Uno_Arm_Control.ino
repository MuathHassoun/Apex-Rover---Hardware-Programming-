#include <Servo.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Mount + ARM Placeholder
//
// Raspberry Pi sends commands to Arduino UNO through USB Serial.
//
// System:
// Raspberry Pi -> USB Cable -> Arduino UNO
// Arduino UNO -> A4988 Stepper Driver + Camera Servo
//
// Camera movement:
// Stepper Motor -> rotate camera LEFT / RIGHT
// Servo Motor   -> tilt camera UP / DOWN
//
// ARM commands:
// ARM control is kept as a placeholder for advanced control later.
// Do NOT remove ARM section.
// ==================================================


// ==================================================
// A4988 Stepper Driver Pins - UPDATED
// ==================================================
//
// A4988 EN   -> Arduino UNO Pin A1
// A4988 STEP -> Arduino UNO Pin D13
// A4988 DIR  -> Arduino UNO Pin D12
//
// Important for A4988:
// RESET and SLEEP should be connected together,
// then connected to 5V.
//
// VMOT should be connected to motor power supply.
// VMOT GND should be common with Arduino GND.
// Arduino GND and motor power GND must be common.
// ==================================================

#define EN_PIN   A1
#define STEP_PIN 13
#define DIR_PIN  12


// ==================================================
// Camera Servo Pin
// ==================================================
//
// Servo Signal -> Arduino UNO Pin 9
// Servo VCC    -> External 5V / 6V supply
// Servo GND    -> External power GND
// UNO GND      -> External power GND
// ==================================================

#define CAMERA_SERVO_PIN A0


// ==================================================
// Servo Settings
// ==================================================

Servo cameraServo;

int cameraAngle = 90;          // Start center position

const int SERVO_MIN = 30;      // Minimum safe angle
const int SERVO_MAX = 150;     // Maximum safe angle
const int SERVO_STEP = 5;      // Angle change per command


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

// Smaller value = faster stepper
// Larger value = slower stepper
const unsigned long STEP_INTERVAL_MICROS = 1200;

// STEP pulse width for A4988
const unsigned int STEP_PULSE_MICROS = 3;


// ==================================================
// ARM Placeholder Settings
// ==================================================
//
// Keep this section for future advanced arm control.
// Later we can add servo motors, stepper motors,
// inverse kinematics, gripper, presets, etc.
//
// Current supported placeholder commands:
// ARM:STOP
// ARM:HOME
// ARM:STATUS
// ARM:BASE:LEFT
// ARM:BASE:RIGHT
// ARM:SHOULDER:UP
// ARM:SHOULDER:DOWN
// ARM:ELBOW:UP
// ARM:ELBOW:DOWN
// ARM:GRIPPER:OPEN
// ARM:GRIPPER:CLOSE
// ==================================================

bool armEnabled = false;
String lastArmCommand = "NONE";


// ==================================================
// SETUP
// ==================================================

void setup() {
  // USB Serial with Raspberry Pi
  Serial.begin(9600);

  // Stepper driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // A4988 enable:
  // LOW  = driver enabled
  // HIGH = driver disabled
  digitalWrite(EN_PIN, LOW);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // Servo setup
  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(cameraAngle);

  Serial.println("====================================");
  Serial.println("UNO Ready - Apex Rover Camera Mount");
  Serial.println("Stepper EN   : A1");
  Serial.println("Stepper STEP : D13");
  Serial.println("Stepper DIR  : D12");
  Serial.println("Servo Pin    : D9");
  Serial.println("ARM placeholder enabled");
  Serial.println("====================================");

  Serial.println("Available CAM commands:");
  Serial.println("CAM:LEFT");
  Serial.println("CAM:RIGHT");
  Serial.println("CAM:STOP");
  Serial.println("CAM:UP");
  Serial.println("CAM:DOWN");
  Serial.println("CAM:CENTER");
  Serial.println("CAM:STATUS");

  Serial.println("Available ARM placeholder commands:");
  Serial.println("ARM:STOP");
  Serial.println("ARM:HOME");
  Serial.println("ARM:STATUS");
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
    Serial.println("ACK:CAM:STOP");
  }

  else if (command == "CAM:UP") {
    cameraAngle += SERVO_STEP;
    updateCameraServo();
    Serial.println("ACK:CAM:UP");
  }

  else if (command == "CAM:DOWN") {
    cameraAngle -= SERVO_STEP;
    updateCameraServo();
    Serial.println("ACK:CAM:DOWN");
  }

  else if (command == "CAM:CENTER") {
    cameraDirection = 0;
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
//
// When cameraDirection is LEFT or RIGHT,
// this function keeps sending STEP pulses.
// When CAM:STOP is received, cameraDirection becomes 0.
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
  Serial.println(cameraDirection);
}


// ==================================================
// ARM Command Handler - Placeholder
// ==================================================
//
// This part is intentionally kept for future advanced arm control.
// For now it only receives commands and sends ACK.
// Later we will add real arm motor/servo pins here.
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
//
// Later, continuous arm movement or smooth servo control
// can be handled here.
// ==================================================

void runArmPlaceholder() {
  // Empty for now.
  // Future:
  // - smooth arm servo updates
  // - stepper movement
  // - gripper control
  // - safety limits
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
