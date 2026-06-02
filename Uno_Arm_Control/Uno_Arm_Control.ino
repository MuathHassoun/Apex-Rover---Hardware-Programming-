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
//   ARM:READY
//   ARM:HOME
//   ARM:STOP
//
//   ARM:BASE:LEFT
//   ARM:BASE:RIGHT
//   ARM:BASE:STOP
//   ARM:BASE:STEP_LEFT:N
//   ARM:BASE:STEP_RIGHT:N
//   ARM:BASE:SPEED:N
//   ARM:BASE:ZERO
//   ARM:BASE:SET_HOME
//   ARM:BASE:GOTO_ZERO
//   ARM:BASE:GOTO_HOME
//   ARM:BASE:GOTO_DEG:N
//   ARM:BASE:GOTO_STEPS:N
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

const unsigned long CAM_SERVO_DETACH_DELAY_MS = 450;
const unsigned long HOME_DETACH_DELAY_MS = 1200;
const unsigned long ARM_HOLD_REFRESH_MS = 35;


// ==================================================
// Camera Servo Range
// ==================================================
const int CAM_SERVO_MIN = 30;
const int CAM_SERVO_MAX = 150;
const int CAM_SERVO_STEP = 5;
const int CAM_SERVO_CENTER = 90;

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
// Arm READY Pose
// READY means the arm is active and facing the front.
// Adjust these values after testing.
// ==================================================
const int READY_SHOULDER = 90;
const int READY_ELBOW = 30;
const int READY_WRIST = 90;
const int READY_AUX = 120;
const int READY_GRIPPER = 180;


// ==================================================
// Arm HOME Pose
// HOME means the arm is resting on the rear stand.
// Adjust these values after testing.
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
const int GRIPPER_OPEN_ANGLE = 180;
const int GRIPPER_CLOSE_ANGLE = 120;


// ==================================================
// Arm Base Stepper Position Settings
//
// Position is software-based.
// The robot must start from a known position.
// In this code, setup assumes the arm starts at HOME position.
// ==================================================
const long ARM_BASE_STEPS_PER_DEG = 10;

const long ARM_BASE_ZERO_DEG = 0;
const long ARM_BASE_HOME_DEG = 150;
const long ARM_BASE_READY_DEG = 0;

const long ARM_BASE_ZERO_POSITION = ARM_BASE_ZERO_DEG * ARM_BASE_STEPS_PER_DEG;
const long ARM_BASE_HOME_POSITION = ARM_BASE_HOME_DEG * ARM_BASE_STEPS_PER_DEG;
const long ARM_BASE_READY_POSITION = ARM_BASE_READY_DEG * ARM_BASE_STEPS_PER_DEG;


// ==================================================
// System State
// ==================================================
String systemMode = "MANUAL";

enum ArmPoseState {
  ARM_POSE_HOME,
  ARM_POSE_READY,
  ARM_POSE_ACTIVE,
  ARM_POSE_MOVING_HOME,
  ARM_POSE_MOVING_READY
};

ArmPoseState armPoseState = ARM_POSE_HOME;


// ==================================================
// Camera Servo State
// ==================================================
int cameraServoAngle = CAM_SERVO_CENTER;
bool cameraServoAttached = false;
bool pendingCameraDetach = false;
unsigned long cameraDetachStartMs = 0;


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
long armBaseStepPosition = ARM_BASE_HOME_POSITION;

bool armBaseTargetMode = false;
long armBaseTargetPosition = ARM_BASE_HOME_POSITION;

unsigned long armBaseLastStepTime = 0;
unsigned long armBaseStepIntervalMicros = 700;


// ==================================================
// Non-Blocking Command Buffers
// ==================================================
char espCmdBuffer[80];
byte espCmdIndex = 0;

char usbCmdBuffer[80];
byte usbCmdIndex = 0;


// ==================================================
// Basic Helpers
// ==================================================
int clampAngle(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return value > maxValue ? maxValue : value;
  return value;
}

long parseLastLong(const char *cmd) {
  const char *lastColon = strrchr(cmd, ':');
  if (lastColon == NULL) return 0;
  return atol(lastColon + 1);
}

long parsePositiveSteps(const char *cmd) {
  long value = parseLastLong(cmd);
  if (value < 0) value = -value;
  if (value < 1) value = 1;
  if (value > 50000) value = 50000;
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

long degToArmBaseSteps(long deg) {
  return deg * ARM_BASE_STEPS_PER_DEG;
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

void detachCameraServo() {
  if (cameraServoAttached) {
    cameraServo.detach();
    cameraServoAttached = false;
  }

  pendingCameraDetach = false;
}

void scheduleCameraDetach() {
  pendingCameraDetach = true;
  cameraDetachStartMs = millis();
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

void setArmAngles(int sh, int el, int wr, int gr, int ax) {
  shoulderAngle = clampAngle(sh, SHOULDER_MIN, SHOULDER_MAX);
  elbowAngle = clampAngle(el, ELBOW_MIN, ELBOW_MAX);
  wristAngle = clampAngle(wr, WRIST_MIN, WRIST_MAX);
  gripperAngle = clampAngle(gr, GRIPPER_MIN, GRIPPER_MAX);
  auxAngle = clampAngle(ax, AUX_MIN, AUX_MAX);

  writeArmServos();
}

void markArmActive() {
  if (armPoseState == ARM_POSE_HOME) {
    armPoseState = ARM_POSE_ACTIVE;
  }

  pendingHomeDetach = false;
  attachArmServosIfNeeded();
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

  scheduleCameraDetach();
}


// ==================================================
// Arm Base Stepper Target Control
// ==================================================
void onArmBaseTargetReached() {
  if (armPoseState == ARM_POSE_MOVING_HOME) {
    armPoseState = ARM_POSE_HOME;
    pendingHomeDetach = true;
    homeDetachStartMs = millis();
    return;
  }

  if (armPoseState == ARM_POSE_MOVING_READY) {
    armPoseState = ARM_POSE_READY;
    pendingHomeDetach = false;
    writeArmServos();
    return;
  }
}

void stopArmBaseMotion() {
  armBaseStepperDirection = 0;
  armBaseFiniteStepsRemaining = 0;
  armBaseTargetMode = false;

  digitalWrite(ARM_STEP_PIN, LOW);
  disableDriver(ARM_EN_PIN);
}

void finishArmBaseTarget() {
  armBaseStepperDirection = 0;
  armBaseFiniteStepsRemaining = 0;
  armBaseTargetMode = false;

  digitalWrite(ARM_STEP_PIN, LOW);
  disableDriver(ARM_EN_PIN);

  onArmBaseTargetReached();
}

void setArmBaseTargetSteps(long targetSteps) {
  armBaseTargetPosition = targetSteps;
  armBaseTargetMode = true;
  armBaseFiniteStepsRemaining = 0;

  if (armBaseStepPosition == armBaseTargetPosition) {
    finishArmBaseTarget();
    return;
  }

  if (armBaseStepPosition < armBaseTargetPosition) {
    armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH);
  } else {
    armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW);
  }

  enableDriver(ARM_EN_PIN);
}

void setArmBaseTargetDeg(long deg) {
  setArmBaseTargetSteps(degToArmBaseSteps(deg));
}


// ==================================================
// Arm Pose Control
// ==================================================
void requestArmHome() {
  stopCameraMotion();

  pendingHomeDetach = false;
  armPoseState = ARM_POSE_MOVING_HOME;

  setArmAngles(HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, HOME_GRIPPER, HOME_AUX);
  setArmBaseTargetSteps(ARM_BASE_HOME_POSITION);
}

void requestArmReady() {
  pendingHomeDetach = false;
  armPoseState = ARM_POSE_MOVING_READY;

  setArmAngles(READY_SHOULDER, READY_ELBOW, READY_WRIST, READY_GRIPPER, READY_AUX);
  setArmBaseTargetSteps(ARM_BASE_READY_POSITION);
}

void stopAllArmMotion() {
  stopArmBaseMotion();

  if (armPoseState != ARM_POSE_HOME) {
    writeArmServos();
  }
}


// ==================================================
// Arm Holding Logic
// ==================================================
void refreshArmHold() {
  if (armPoseState == ARM_POSE_HOME) {
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

  if (armPoseState == ARM_POSE_HOME) {
    detachArmServos();
    stopArmBaseMotion();
  }
}

void handleCameraDetach() {
  if (!pendingCameraDetach) {
    return;
  }

  if (millis() - cameraDetachStartMs < CAM_SERVO_DETACH_DELAY_MS) {
    return;
  }

  detachCameraServo();
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
    setCameraAngle(CAM_SERVO_CENTER);
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
    cameraFiniteStepsRemaining = parsePositiveSteps(cmd);
    cameraStepperDirection = -1;

    digitalWrite(CAM_DIR_PIN, LOW);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = parsePositiveSteps(cmd);
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
    long interval = parsePositiveSteps(cmd);
    cameraStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  if (startsWithCmd(cmd, "CAM:ANGLE:")) {
    setCameraAngle((int)parseLastLong(cmd));
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
    requestArmHome();
    return;
  }

  if (equalsCmd(cmd, "ARM:READY")) {
    requestArmReady();
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:LEFT")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = -1;

    digitalWrite(ARM_DIR_PIN, LOW);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:RIGHT")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = 1;

    digitalWrite(ARM_DIR_PIN, HIGH);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:STOP")) {
    stopArmBaseMotion();

    if (armPoseState != ARM_POSE_HOME) {
      writeArmServos();
    }

    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:STEP_LEFT:")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = parsePositiveSteps(cmd);
    armBaseStepperDirection = -1;

    digitalWrite(ARM_DIR_PIN, LOW);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:STEP_RIGHT:")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = parsePositiveSteps(cmd);
    armBaseStepperDirection = 1;

    digitalWrite(ARM_DIR_PIN, HIGH);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:SPEED:")) {
    long interval = parsePositiveSteps(cmd);
    armBaseStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:ZERO")) {
    armBaseStepPosition = ARM_BASE_ZERO_POSITION;
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:SET_HOME")) {
    armBaseStepPosition = ARM_BASE_HOME_POSITION;
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:GOTO_ZERO")) {
    markArmActive();
    setArmBaseTargetSteps(ARM_BASE_ZERO_POSITION);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:GOTO_HOME")) {
    markArmActive();
    setArmBaseTargetSteps(ARM_BASE_HOME_POSITION);
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:GOTO_DEG:")) {
    markArmActive();
    setArmBaseTargetDeg(parseLastLong(cmd));
    return;
  }

  if (startsWithCmd(cmd, "ARM:BASE:GOTO_STEPS:")) {
    markArmActive();
    setArmBaseTargetSteps(parseLastLong(cmd));
    return;
  }

  // ==================================================
  // IMPORTANT:
  // UP / DOWN are intentionally inverted here
  // to match the Remote Control arm buttons.
  // Do not change camera, base, or gripper logic.
  // ==================================================

  if (equalsCmd(cmd, "ARM:SHOULDER:UP")) {
    setShoulderAngle(shoulderAngle - ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:SHOULDER:DOWN")) {
    setShoulderAngle(shoulderAngle + ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:SHOULDER:ANGLE:")) {
    setShoulderAngle((int)parseLastLong(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:UP")) {
    setElbowAngle(elbowAngle - ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:DOWN")) {
    setElbowAngle(elbowAngle + ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:ELBOW:ANGLE:")) {
    setElbowAngle((int)parseLastLong(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:UP")) {
    setWristAngle(wristAngle - ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:DOWN")) {
    setWristAngle(wristAngle + ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:WRIST:ANGLE:")) {
    setWristAngle((int)parseLastLong(cmd));
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
    setGripperAngle((int)parseLastLong(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:AUX:UP")) {
    setAuxAngle(auxAngle - ARM_SERVO_STEP);
    return;
  }

  if (equalsCmd(cmd, "ARM:AUX:DOWN")) {
    setAuxAngle(auxAngle + ARM_SERVO_STEP);
    return;
  }

  if (startsWithCmd(cmd, "ARM:AUX:ANGLE:")) {
    setAuxAngle((int)parseLastLong(cmd));
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
    systemMode = "MANUAL";
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }

  if (equalsCmd(cmd, "SYS:MODE:AUTO")) {
    systemMode = "AUTO";
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
void readSerialStream(Stream &port, char *buffer, byte &index) {
  while (port.available() > 0) {
    char c = port.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      buffer[index] = '\0';

      if (index > 0) {
        if (DEBUG_SERIAL) {
          Serial.print("[UNO RX] ");
          Serial.println(buffer);
        }

        handleCommand(buffer);
      }

      index = 0;
      return;
    }

    if (index < 79) {
      buffer[index++] = c;
    } else {
      index = 0;
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

  if (armBaseTargetMode) {
    if (armBaseStepperDirection > 0 && armBaseStepPosition >= armBaseTargetPosition) {
      armBaseStepPosition = armBaseTargetPosition;
      finishArmBaseTarget();
      return;
    }

    if (armBaseStepperDirection < 0 && armBaseStepPosition <= armBaseTargetPosition) {
      armBaseStepPosition = armBaseTargetPosition;
      finishArmBaseTarget();
      return;
    }
  }

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

  setCameraAngle(CAM_SERVO_CENTER);

  armBaseStepPosition = ARM_BASE_HOME_POSITION;
  requestArmReady();

  Serial.println("UNO_READY");
  Serial.println("UNO_ARM_READY_ON_STARTUP");
}


// ==================================================
// Main Loop
// ==================================================
void loop() {
  readSerialStream(espSerial, espCmdBuffer, espCmdIndex);

  if (Serial.available() > 0) {
    readSerialStream(Serial, usbCmdBuffer, usbCmdIndex);
  }

  runCameraStepper();
  runArmBaseStepper();

  refreshArmHold();
  handleHomeDetach();
  handleCameraDetach();
}