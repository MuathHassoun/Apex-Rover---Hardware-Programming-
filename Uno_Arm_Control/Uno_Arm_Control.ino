#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand + 6-Axis Arm
//
// IMPORTANT:
// Camera stand pins are kept the same because it was working.
//
// Command path:
//   Mobile App -> WebSocket -> ESP32 -> UNO D2
//
// ESP32:
//   ESP32 GPIO4 TX -> UNO D2 SoftwareSerial RX
//   Common GND between ESP32, UNO, drivers, servo power
//
// Camera Stand:
//   Camera A4988 EN       -> UNO A1
//   Camera A4988 STEP     -> UNO D12
//   Camera A4988 DIR      -> UNO A2
//   Camera Servo Signal   -> UNO A0
//
// Arm Base Stepper:
//   Arm A4988 EN          -> UNO D4
//   Arm A4988 STEP        -> UNO D8
//   Arm A4988 DIR         -> UNO D7
//
// Arm Servos:
//   Shoulder Servo        -> UNO D5
//   Elbow Servo           -> UNO D6
//   Wrist Servo           -> UNO D9
//   Gripper Servo         -> UNO D10
//   Aux / Axis Servo      -> UNO D11
//
// Supported CAM commands:
//   CAM:LEFT
//   CAM:RIGHT
//   CAM:STOP
//   CAM:STEP_LEFT:N
//   CAM:STEP_RIGHT:N
//   CAM:UP
//   CAM:DOWN
//   CAM:CENTER
//   CAM:ZERO
//   CAM:SPEED:N
//   CAM:ANGLE:N
//
// Supported ARM commands:
//   ARM:BASE:LEFT
//   ARM:BASE:RIGHT
//   ARM:BASE:STOP
//   ARM:BASE:STEP_LEFT:N
//   ARM:BASE:STEP_RIGHT:N
//   ARM:BASE:SPEED:N
//
//   ARM:SHOULDER:UP
//   ARM:SHOULDER:DOWN
//   ARM:SHOULDER:ANGLE:N
//   ARM:ELBOW:UP
//   ARM:ELBOW:DOWN
//   ARM:ELBOW:ANGLE:N
//   ARM:WRIST:UP
//   ARM:WRIST:DOWN
//   ARM:WRIST:ANGLE:N
//   ARM:GRIPPER:OPEN
//   ARM:GRIPPER:CLOSE
//   ARM:GRIPPER:ANGLE:N
//   ARM:AUX:UP
//   ARM:AUX:DOWN
//   ARM:AUX:ANGLE:N
//   ARM:HOME
//   ARM:STOP
//
// Global commands:
//   STOP
//   ESTOP
//   SYS:MODE:MANUAL
//   SYS:MODE:AUTO
// ==================================================


// ==================================================
// ESP32 -> UNO Serial
// ==================================================
// ESP32 GPIO4 TX -> UNO D2 RX
// D3 TX is not wired, but SoftwareSerial needs a TX pin.
// ==================================================
SoftwareSerial espSerial(2, 3);


// ==================================================
// Camera Stand Pins - DO NOT CHANGE
// ==================================================
#define CAM_EN_PIN       A1
#define CAM_STEP_PIN     12
#define CAM_DIR_PIN      A2
#define CAM_SERVO_PIN    A0


// ==================================================
// Arm Base Stepper Pins
// EN = 4, STEP = 8, DIR = 7
// ==================================================
#define ARM_EN_PIN       4
#define ARM_STEP_PIN     8
#define ARM_DIR_PIN      7


// ==================================================
// Arm Servo Pins
// Shoulder 5, Elbow 6, Wrist 9, Gripper 10, Aux/Axis 11
// ==================================================
#define SHOULDER_PIN     5
#define ELBOW_PIN        6
#define WRIST_PIN        9
#define GRIPPER_PIN      10
#define AUX_PIN          11


// ==================================================
// Servo Objects
// ==================================================
Servo cameraServo;

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;
Servo auxServo;


// ==================================================
// Debug Settings
// ==================================================
const bool DEBUG_SERIAL = false;


// ==================================================
// Stepper Settings
// ==================================================
const unsigned int STEP_PULSE_MICROS = 3;


// ==================================================
// Camera Servo Range
// ==================================================
const int CAM_SERVO_MIN = 30;
const int CAM_SERVO_MAX = 180;
const int CAM_SERVO_STEP = 5;
const int CAM_SERVO_HOME = 90;

const bool INVERT_CAMERA_VERTICAL = true;


// ==================================================
// Arm Servo Safe Ranges
// ==================================================
const int SHOULDER_MIN = 60;
const int SHOULDER_MAX = 180;

const int ELBOW_MIN = 10;
const int ELBOW_MAX = 150;

const int WRIST_MIN = 30;
const int WRIST_MAX = 150;

const int AUX_MIN = 90;
const int AUX_MAX = 170;

const int GRIPPER_MIN = 120;
const int GRIPPER_MAX = 180;

const int ARM_SERVO_STEP = 5;


// ==================================================
// Temporary Home Angles
// Adjust these values after mechanical testing.
// ==================================================
const int HOME_SHOULDER = 60;
const int HOME_ELBOW = 10;
const int HOME_WRIST = 90;
const int HOME_AUX = 90;
const int HOME_GRIPPER = 180;


// ==================================================
// Gripper Angles
// Change OPEN/CLOSE if gripper direction is reversed.
// ==================================================
const int GRIPPER_OPEN_ANGLE = 120;
const int GRIPPER_CLOSE_ANGLE = 180;


// ==================================================
// Arm Holding Settings
// While the arm is not home, servos keep receiving
// their last saved angles to reduce drifting.
// ==================================================
const unsigned long ARM_HOLD_REFRESH_MS = 25;
const unsigned long HOME_DETACH_DELAY_MS = 1200;

const bool GRIPPER_RELAX_WHEN_HOME = true;


// ==================================================
// Camera Servo State
// ==================================================
int cameraServoAngle = CAM_SERVO_HOME;
bool cameraServoAttached = false;


// ==================================================
// Arm Servo State
// Last angle of each servo is always saved here.
// ==================================================
int shoulderAngle = HOME_SHOULDER;
int elbowAngle = HOME_ELBOW;
int wristAngle = HOME_WRIST;
int gripperAngle = HOME_GRIPPER;
int auxAngle = HOME_AUX;

bool shoulderAttached = false;
bool elbowAttached = false;
bool wristAttached = false;
bool gripperAttached = false;
bool auxAttached = false;

bool armAtHome = true;
bool pendingHomeDetach = false;

unsigned long lastArmHoldRefresh = 0;
unsigned long homeDetachStartMs = 0;


// ==================================================
// Camera Stepper State
// -1 = left, 0 = stop, 1 = right
// ==================================================
int cameraStepperDirection = 0;

long cameraFiniteStepsRemaining = 0;
long cameraStepPosition = 0;

unsigned long cameraLastStepTime = 0;
unsigned long cameraStepIntervalMicros = 700;


// ==================================================
// Arm Base Stepper State
// -1 = left, 0 = stop, 1 = right
// ==================================================
int armBaseStepperDirection = 0;

long armBaseFiniteStepsRemaining = 0;
long armBaseStepPosition = 0;

unsigned long armBaseLastStepTime = 0;
unsigned long armBaseStepIntervalMicros = 700;


// ==================================================
// Non-Blocking Command Buffer
// ==================================================
char cmdBuffer[80];
byte cmdIndex = 0;


// ==================================================
// Basic Helpers
// ==================================================
int clampAngle(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

long parseLastNumber(const char *cmd) {
  const char *lastColon = strrchr(cmd, ':');

  if (lastColon == NULL) {
    return 0;
  }

  long value = atol(lastColon + 1);

  if (value < 0) {
    value = -value;
  }

  if (value < 1) {
    value = 1;
  }

  if (value > 50000) {
    value = 50000;
  }

  return value;
}

bool equalsCmd(const char *cmd, const char *target) {
  return strcmp(cmd, target) == 0;
}

bool startsWithCmd(const char *cmd, const char *prefix) {
  return strncmp(cmd, prefix, strlen(prefix)) == 0;
}

void enableDriver(int enPin) {
  digitalWrite(enPin, LOW);
}

void disableDriver(int enPin) {
  digitalWrite(enPin, HIGH);
}


// ==================================================
// Servo Attach Helpers
// ==================================================
void attachCameraServoIfNeeded() {
  if (!cameraServoAttached) {
    cameraServo.attach(CAM_SERVO_PIN);
    cameraServoAttached = true;
  }
}

void attachShoulderIfNeeded() {
  if (!shoulderAttached) {
    shoulderServo.attach(SHOULDER_PIN);
    shoulderAttached = true;
  }
}

void attachElbowIfNeeded() {
  if (!elbowAttached) {
    elbowServo.attach(ELBOW_PIN);
    elbowAttached = true;
  }
}

void attachWristIfNeeded() {
  if (!wristAttached) {
    wristServo.attach(WRIST_PIN);
    wristAttached = true;
  }
}

void attachGripperIfNeeded() {
  if (!gripperAttached) {
    gripperServo.attach(GRIPPER_PIN);
    gripperAttached = true;
  }
}

void attachAuxIfNeeded() {
  if (!auxAttached) {
    auxServo.attach(AUX_PIN);
    auxAttached = true;
  }
}

void attachArmServosIfNeeded() {
  attachShoulderIfNeeded();
  attachElbowIfNeeded();
  attachWristIfNeeded();
  attachGripperIfNeeded();
  attachAuxIfNeeded();
}

void detachArmServos() {
  if (shoulderAttached) {
    shoulderServo.detach();
    shoulderAttached = false;
  }

  if (elbowAttached) {
    elbowServo.detach();
    elbowAttached = false;
  }

  if (wristAttached) {
    wristServo.detach();
    wristAttached = false;
  }

  if (gripperAttached) {
    gripperServo.detach();
    gripperAttached = false;
  }

  if (auxAttached) {
    auxServo.detach();
    auxAttached = false;
  }
}


// ==================================================
// Arm Servo Output
// ==================================================
void writeArmServos() {
  shoulderAngle = clampAngle(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
  elbowAngle = clampAngle(elbowAngle, ELBOW_MIN, ELBOW_MAX);
  wristAngle = clampAngle(wristAngle, WRIST_MIN, WRIST_MAX);
  gripperAngle = clampAngle(gripperAngle, GRIPPER_MIN, GRIPPER_MAX);
  auxAngle = clampAngle(auxAngle, AUX_MIN, AUX_MAX);

  attachArmServosIfNeeded();

  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);
  auxServo.write(auxAngle);
}

void markArmActive() {
  armAtHome = false;
  pendingHomeDetach = false;
  writeArmServos();
}


// ==================================================
// Camera Motion Control
// ==================================================
void stopCameraMotion() {
  cameraStepperDirection = 0;
  cameraFiniteStepsRemaining = 0;

  digitalWrite(CAM_STEP_PIN, LOW);
  disableDriver(CAM_EN_PIN);
}

void setCameraAngle(int angle) {
  cameraServoAngle = clampAngle(angle, CAM_SERVO_MIN, CAM_SERVO_MAX);

  attachCameraServoIfNeeded();
  cameraServo.write(cameraServoAngle);
}


// ==================================================
// Arm Motion Control
// ==================================================
void stopArmBaseMotion() {
  armBaseStepperDirection = 0;
  armBaseFiniteStepsRemaining = 0;

  digitalWrite(ARM_STEP_PIN, LOW);
  disableDriver(ARM_EN_PIN);
}

void stopAllArmMotion() {
  stopArmBaseMotion();

  if (!armAtHome) {
    writeArmServos();
  }
}

void armHome() {
  stopArmBaseMotion();

  shoulderAngle = HOME_SHOULDER;
  elbowAngle = HOME_ELBOW;
  wristAngle = HOME_WRIST;
  gripperAngle = HOME_GRIPPER;
  auxAngle = HOME_AUX;

  writeArmServos();

  armAtHome = true;
  pendingHomeDetach = true;
  homeDetachStartMs = millis();
}


// ==================================================
// Arm Holding Logic
// ==================================================
void refreshArmHold() {
  if (armAtHome) {
    return;
  }

  unsigned long now = millis();

  if (now - lastArmHoldRefresh < ARM_HOLD_REFRESH_MS) {
    return;
  }

  lastArmHoldRefresh = now;
  writeArmServos();
}

void handleHomeDetach() {
  if (!pendingHomeDetach) {
    return;
  }

  if (millis() - homeDetachStartMs < HOME_DETACH_DELAY_MS) {
    return;
  }

  pendingHomeDetach = false;

  if (GRIPPER_RELAX_WHEN_HOME) {
    detachArmServos();
    return;
  }

  if (shoulderAttached) {
    shoulderServo.detach();
    shoulderAttached = false;
  }

  if (elbowAttached) {
    elbowServo.detach();
    elbowAttached = false;
  }

  if (wristAttached) {
    wristServo.detach();
    wristAttached = false;
  }

  if (auxAttached) {
    auxServo.detach();
    auxAttached = false;
  }
}


// ==================================================
// Arm Servo Setters
// ==================================================
void setShoulderAngle(int angle) {
  markArmActive();
  shoulderAngle = clampAngle(angle, SHOULDER_MIN, SHOULDER_MAX);
  writeArmServos();
}

void setElbowAngle(int angle) {
  markArmActive();
  elbowAngle = clampAngle(angle, ELBOW_MIN, ELBOW_MAX);
  writeArmServos();
}

void setWristAngle(int angle) {
  markArmActive();
  wristAngle = clampAngle(angle, WRIST_MIN, WRIST_MAX);
  writeArmServos();
}

void setGripperAngle(int angle) {
  markArmActive();
  gripperAngle = clampAngle(angle, GRIPPER_MIN, GRIPPER_MAX);
  writeArmServos();
}

void setAuxAngle(int angle) {
  markArmActive();
  auxAngle = clampAngle(angle, AUX_MIN, AUX_MAX);
  writeArmServos();
}


// ==================================================
// Camera Command Handler
// ==================================================
void handleCameraCommand(const char *cmd) {
  if (equalsCmd(cmd, "CAM:STOP")) {
    stopCameraMotion();
    return;
  }

  if (equalsCmd(cmd, "CAM:CENTER")) {
    stopCameraMotion();
    setCameraAngle(CAM_SERVO_HOME);
    return;
  }

  if (equalsCmd(cmd, "CAM:LEFT")) {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = -1;

    digitalWrite(CAM_DIR_PIN, LOW);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "CAM:RIGHT")) {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = 1;

    digitalWrite(CAM_DIR_PIN, HIGH);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "CAM:STEP_LEFT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd);
    cameraStepperDirection = -1;

    digitalWrite(CAM_DIR_PIN, LOW);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd);
    cameraStepperDirection = 1;

    digitalWrite(CAM_DIR_PIN, HIGH);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "CAM:UP")) {
    int delta = INVERT_CAMERA_VERTICAL ? -CAM_SERVO_STEP : CAM_SERVO_STEP;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (equalsCmd(cmd, "CAM:DOWN")) {
    int delta = INVERT_CAMERA_VERTICAL ? CAM_SERVO_STEP : -CAM_SERVO_STEP;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (equalsCmd(cmd, "CAM:ZERO")) {
    cameraStepPosition = 0;
    return;
  }

  if (startsWithCmd(cmd, "CAM:SPEED:")) {
    long interval = parseLastNumber(cmd);
    cameraStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  if (startsWithCmd(cmd, "CAM:ANGLE:")) {
    setCameraAngle((int)parseLastNumber(cmd));
    return;
  }
}


// ==================================================
// Arm Command Handler
// ==================================================
void handleArmCommand(const char *cmd) {
  if (equalsCmd(cmd, "ARM:STOP")) {
    stopAllArmMotion();
    return;
  }

  if (equalsCmd(cmd, "ARM:HOME")) {
    armHome();
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:LEFT")) {
    markArmActive();

    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = -1;

    digitalWrite(ARM_DIR_PIN, LOW);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:RIGHT")) {
    markArmActive();

    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = 1;

    digitalWrite(ARM_DIR_PIN, HIGH);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:STOP")) {
    stopArmBaseMotion();

    if (!armAtHome) {
      writeArmServos();
    }

    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:STEP_LEFT:")) {
    markArmActive();

    armBaseFiniteStepsRemaining = parseLastNumber(cmd);
    armBaseStepperDirection = -1;

    digitalWrite(ARM_DIR_PIN, LOW);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:STEP_RIGHT:")) {
    markArmActive();

    armBaseFiniteStepsRemaining = parseLastNumber(cmd);
    armBaseStepperDirection = 1;

    digitalWrite(ARM_DIR_PIN, HIGH);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:SPEED:")) {
    long interval = parseLastNumber(cmd);
    armBaseStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  if (equalsCmd(cmd, "ARM:SHOULDER:UP")) {
    setShoulderAngle(shoulderAngle + ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:SHOULDER:DOWN")) {
    setShoulderAngle(shoulderAngle - ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:SHOULDER:ANGLE:")) {
    setShoulderAngle((int)parseLastNumber(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:UP")) {
    setElbowAngle(elbowAngle + ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:DOWN")) {
    setElbowAngle(elbowAngle - ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:ELBOW:ANGLE:")) {
    setElbowAngle((int)parseLastNumber(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:UP")) {
    setWristAngle(wristAngle + ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:DOWN")) {
    setWristAngle(wristAngle - ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:WRIST:ANGLE:")) {
    setWristAngle((int)parseLastNumber(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:GRIPPER:OPEN")) {
    setGripperAngle(GRIPPER_OPEN_ANGLE);
    return;
  }

  if (equalsCmd(cmd, "ARM:GRIPPER:CLOSE")) {
    setGripperAngle(GRIPPER_CLOSE_ANGLE);
    return;
  }

  if (startsWithCmd(cmd, "ARM:GRIPPER:ANGLE:")) {
    setGripperAngle((int)parseLastNumber(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:AUX:UP")) {
    setAuxAngle(auxAngle + ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:AUX:DOWN")) {
    setAuxAngle(auxAngle - ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:AUX:ANGLE:")) {
    setAuxAngle((int)parseLastNumber(cmd));
    return;
  }
}


// ==================================================
// System Command Handler
// ==================================================
void handleSystemCommand(const char *cmd) {
  if (equalsCmd(cmd, "STOP") || equalsCmd(cmd, "ESTOP")) {
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }

  if (equalsCmd(cmd, "SYS:MODE:MANUAL")) {
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }

  if (equalsCmd(cmd, "SYS:MODE:AUTO")) {
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }
}


// ==================================================
// Main Command Router
// Camera commands never control arm servos.
// Arm commands never control camera servo.
// ==================================================
void handleCommand(const char *cmd) {
  if (startsWithCmd(cmd, "CAM:")) {
    handleCameraCommand(cmd);
    return;
  }

  if (startsWithCmd(cmd, "ARM:")) {
    handleArmCommand(cmd);
    return;
  }

  handleSystemCommand(cmd);
}


// ==================================================
// Non-Blocking Serial Reader
// ==================================================
void readSerialStream(Stream &port) {
  while (port.available() > 0) {
    char c = port.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';

      if (cmdIndex > 0) {
        if (DEBUG_SERIAL) {
          Serial.print("[UNO RX] ");
          Serial.println(cmdBuffer);
        }

        handleCommand(cmdBuffer);
      }

      cmdIndex = 0;
      return;
    }

    if (cmdIndex < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIndex++] = c;
    } else {
      cmdIndex = 0;
    }
  }
}


// ==================================================
// Camera Stepper Runner - Non Blocking
// ==================================================
void runCameraStepper() {
  if (cameraStepperDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - cameraLastStepTime < cameraStepIntervalMicros) {
    return;
  }

  cameraLastStepTime = now;

  digitalWrite(CAM_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(CAM_STEP_PIN, LOW);

  cameraStepPosition += cameraStepperDirection;

  if (cameraFiniteStepsRemaining > 0) {
    cameraFiniteStepsRemaining--;

    if (cameraFiniteStepsRemaining == 0) {
      stopCameraMotion();
    }
  }
}


// ==================================================
// Arm Base Stepper Runner - Non Blocking
// ==================================================
void runArmBaseStepper() {
  if (armBaseStepperDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - armBaseLastStepTime < armBaseStepIntervalMicros) {
    return;
  }

  armBaseLastStepTime = now;

  digitalWrite(ARM_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(ARM_STEP_PIN, LOW);

  armBaseStepPosition += armBaseStepperDirection;

  if (armBaseFiniteStepsRemaining > 0) {
    armBaseFiniteStepsRemaining--;

    if (armBaseFiniteStepsRemaining == 0) {
      stopArmBaseMotion();
    }
  }
}


// ==================================================
// Setup
// ==================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(CAM_EN_PIN, OUTPUT);
  pinMode(CAM_STEP_PIN, OUTPUT);
  pinMode(CAM_DIR_PIN, OUTPUT);

  pinMode(ARM_EN_PIN, OUTPUT);
  pinMode(ARM_STEP_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);

  digitalWrite(CAM_STEP_PIN, LOW);
  digitalWrite(CAM_DIR_PIN, LOW);
  disableDriver(CAM_EN_PIN);

  digitalWrite(ARM_STEP_PIN, LOW);
  digitalWrite(ARM_DIR_PIN, LOW);
  disableDriver(ARM_EN_PIN);

  attachCameraServoIfNeeded();
  cameraServo.write(cameraServoAngle);

  armHome();

  Serial.println("UNO_READY");
}


// ==================================================
// Main Loop
// ==================================================
void loop() {
  readSerialStream(espSerial);

  if (Serial.available() > 0) {
    readSerialStream(Serial);
  }

  runCameraStepper();
  runArmBaseStepper();

  refreshArmHold();
  handleHomeDetach();
}