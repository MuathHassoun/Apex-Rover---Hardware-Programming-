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
// New config commands from mobile:
//   ARM:CONFIG:STEPPER_STEPS:N
//   ARM:CONFIG:SERVO_STEP:N
//   CAM:CONFIG:STEPPER_STEPS:N
//   CAM:CONFIG:SERVO_STEP:N
//
// Supported CAM commands:
//   CAM:LEFT
//   CAM:RIGHT
//   CAM:STOP
//   CAM:STEP_LEFT
//   CAM:STEP_RIGHT
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
//   ARM:READY   -> staged: base stepper, shoulder, aux, gripper, wrist, elbow
//   ARM:HOME    -> staged: base stepper, shoulder, aux, gripper, wrist, elbow
//   ARM:STOP
//
//   ARM:BASE:LEFT
//   ARM:BASE:RIGHT
//   ARM:BASE:STOP
//   ARM:BASE:STEP_LEFT
//   ARM:BASE:STEP_RIGHT
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
// ==================================================
#define ARM_EN_PIN       4
#define ARM_STEP_PIN     8
#define ARM_DIR_PIN      7


// ==================================================
// Arm Servo Pins
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
// Staged HOME / READY Motion Settings
//
// HOME and READY are NOT applied all at once.
// Sequence is:
//   1) Arm base stepper
//   2) Shoulder
//   3) Aux
//   4) Gripper
//   5) Wrist
//   6) Elbow
// ==================================================
const unsigned long ARM_POSE_SERVO_INTERVAL_MS = 25;
const int ARM_POSE_SERVO_STEP_DEG = 2;


// ==================================================
// Configurable Motion Settings
//
// These are changed by mobile commands:
//   ARM:CONFIG:STEPPER_STEPS:N
//   ARM:CONFIG:SERVO_STEP:N
//
// They are shared by camera stand and arm.
// ==================================================
const long DEFAULT_STEPPER_STEPS = 100;
const int DEFAULT_SERVO_STEP = 5;

long configuredStepperSteps = DEFAULT_STEPPER_STEPS;
int configuredServoStep = DEFAULT_SERVO_STEP;

const long MIN_STEPPER_STEPS = 1;
const long MAX_STEPPER_STEPS = 50000;

const int MIN_SERVO_STEP = 1;
const int MAX_SERVO_STEP = 30;


// ==================================================
// Camera Servo Range
// ==================================================
const int CAM_SERVO_MIN = 30;
const int CAM_SERVO_MAX = 150;
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


// ==================================================
// Arm READY Pose
// ==================================================
const int READY_SHOULDER = 60;
const int READY_ELBOW = 80;
const int READY_WRIST = 150;
const int READY_AUX = 170;
const int READY_GRIPPER = 120;


// ==================================================
// Arm HOME Pose
// ==================================================
const int HOME_SHOULDER = 120;
const int HOME_ELBOW = 90;
const int HOME_WRIST = 110;
const int HOME_AUX = 170;
const int HOME_GRIPPER = 180;


// ==================================================
// Gripper Angles
// ==================================================
const int GRIPPER_OPEN_ANGLE = 120;
const int GRIPPER_CLOSE_ANGLE = 180;


// ==================================================
// Arm Base Stepper Position Settings
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

enum ArmPoseStage {
  ARM_STAGE_IDLE,
  ARM_STAGE_BASE_STEPPER,
  ARM_STAGE_SHOULDER,
  ARM_STAGE_AUX,
  ARM_STAGE_GRIPPER,
  ARM_STAGE_WRIST,
  ARM_STAGE_ELBOW
};

ArmPoseStage armPoseStage = ARM_STAGE_IDLE;


// ==================================================
// Camera Servo State
// ==================================================
int cameraServoAngle = CAM_SERVO_CENTER;
bool cameraServoAttached = false;
bool pendingCameraDetach = false;
unsigned long cameraDetachStartMs = 0;


// ==================================================
// Arm Servo State
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
// Staged HOME / READY Target State
// ==================================================
int targetShoulderAngle = HOME_SHOULDER;
int targetElbowAngle = HOME_ELBOW;
int targetWristAngle = HOME_WRIST;
int targetGripperAngle = HOME_GRIPPER;
int targetAuxAngle = HOME_AUX;
long targetArmBasePosition = ARM_BASE_HOME_POSITION;

unsigned long lastPoseServoMoveMs = 0;


// ==================================================
// Camera Stepper State
// ==================================================
int cameraStepperDirection = 0;

long cameraFiniteStepsRemaining = 0;
long cameraStepPosition = 0;

unsigned long cameraLastStepTime = 0;
unsigned long cameraStepIntervalMicros = 700;


// ==================================================
// Arm Base Stepper State
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
  if (value > maxValue) return maxValue;
  return value;
}

long clampLong(long value, long minValue, long maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
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
  return clampLong(value, MIN_STEPPER_STEPS, MAX_STEPPER_STEPS);
}

long parseStepsOrDefault(const char *cmd) {
  const char *lastColon = strrchr(cmd, ':');

  if (lastColon == NULL) {
    return configuredStepperSteps;
  }

  const char *afterColon = lastColon + 1;

  if (afterColon == NULL || afterColon[0] == '\0') {
    return configuredStepperSteps;
  }

  bool hasDigit = false;

  for (int i = 0; afterColon[i] != '\0'; i++) {
    if (afterColon[i] >= '0' && afterColon[i] <= '9') {
      hasDigit = true;
      break;
    }
  }

  if (!hasDigit) {
    return configuredStepperSteps;
  }

  return parsePositiveSteps(cmd);
}

int parseServoStepOrDefault(const char *cmd) {
  long value = parseLastLong(cmd);

  if (value <= 0) {
    return configuredServoStep;
  }

  return (int)clampLong(value, MIN_SERVO_STEP, MAX_SERVO_STEP);
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

void sendAck(const char *msg) {
  Serial.print("ACK:");
  Serial.println(msg);
}


// ==================================================
// Config Command Handler
// ==================================================
bool handleConfigCommand(const char *cmd) {
  if (startsWithCmd(cmd, "ARM:CONFIG:STEPPER_STEPS:") ||
      startsWithCmd(cmd, "CAM:CONFIG:STEPPER_STEPS:")) {
    long value = parseLastLong(cmd);
    configuredStepperSteps = clampLong(value, MIN_STEPPER_STEPS, MAX_STEPPER_STEPS);

    Serial.print("ACK:CONFIG:STEPPER_STEPS:");
    Serial.println(configuredStepperSteps);
    return true;
  }

  if (startsWithCmd(cmd, "ARM:CONFIG:SERVO_STEP:") ||
      startsWithCmd(cmd, "CAM:CONFIG:SERVO_STEP:")) {
    long value = parseLastLong(cmd);
    configuredServoStep = (int)clampLong(value, MIN_SERVO_STEP, MAX_SERVO_STEP);

    Serial.print("ACK:CONFIG:SERVO_STEP:");
    Serial.println(configuredServoStep);
    return true;
  }

  if (equalsCmd(cmd, "ARM:CONFIG:RESET") ||
      equalsCmd(cmd, "CAM:CONFIG:RESET")) {
    configuredStepperSteps = DEFAULT_STEPPER_STEPS;
    configuredServoStep = DEFAULT_SERVO_STEP;

    Serial.println("ACK:CONFIG:RESET");
    return true;
  }

  if (equalsCmd(cmd, "ARM:CONFIG:STATUS") ||
      equalsCmd(cmd, "CAM:CONFIG:STATUS")) {
    Serial.print("CONFIG:STEPPER_STEPS=");
    Serial.print(configuredStepperSteps);
    Serial.print(";SERVO_STEP=");
    Serial.println(configuredServoStep);
    return true;
  }

  return false;
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

void cancelArmPoseSequenceForManualControl() {
  if (armPoseState == ARM_POSE_MOVING_HOME || armPoseState == ARM_POSE_MOVING_READY) {
    armPoseStage = ARM_STAGE_IDLE;
    armPoseState = ARM_POSE_ACTIVE;

    armBaseStepperDirection = 0;
    armBaseFiniteStepsRemaining = 0;
    armBaseTargetMode = false;
    digitalWrite(ARM_STEP_PIN, LOW);
    disableDriver(ARM_EN_PIN);
  }
}

void markArmActive() {
  cancelArmPoseSequenceForManualControl();

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
void beginNextPoseStageAfterBase() {
  armPoseStage = ARM_STAGE_SHOULDER;
  lastPoseServoMoveMs = 0;

  attachArmServosIfNeeded();

  if (armPoseState == ARM_POSE_MOVING_HOME) {
    Serial.println("ACK:ARM:HOME:STAGE:SHOULDER");
  } else if (armPoseState == ARM_POSE_MOVING_READY) {
    Serial.println("ACK:ARM:READY:STAGE:SHOULDER");
  }
}

void onArmBaseTargetReached() {
  if ((armPoseState == ARM_POSE_MOVING_HOME || armPoseState == ARM_POSE_MOVING_READY) &&
      armPoseStage == ARM_STAGE_BASE_STEPPER) {
    beginNextPoseStageAfterBase();
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
// Arm Pose Control - Staged HOME / READY
// ==================================================
void setArmPoseTargets(int sh, int el, int wr, int gr, int ax, long basePosition) {
  targetShoulderAngle = clampAngle(sh, SHOULDER_MIN, SHOULDER_MAX);
  targetElbowAngle = clampAngle(el, ELBOW_MIN, ELBOW_MAX);
  targetWristAngle = clampAngle(wr, WRIST_MIN, WRIST_MAX);
  targetGripperAngle = clampAngle(gr, GRIPPER_MIN, GRIPPER_MAX);
  targetAuxAngle = clampAngle(ax, AUX_MIN, AUX_MAX);
  targetArmBasePosition = basePosition;
}

void beginArmPoseSequence(ArmPoseState movingState) {
  stopArmBaseMotion();

  pendingHomeDetach = false;
  armPoseState = movingState;
  armPoseStage = ARM_STAGE_BASE_STEPPER;
  lastPoseServoMoveMs = 0;

  Serial.println(movingState == ARM_POSE_MOVING_HOME ?
                 "ACK:ARM:HOME:STAGE:BASE_STEPPER" :
                 "ACK:ARM:READY:STAGE:BASE_STEPPER");

  setArmBaseTargetSteps(targetArmBasePosition);
}

void requestArmHome() {
  stopCameraMotion();

  setArmPoseTargets(HOME_SHOULDER,
                    HOME_ELBOW,
                    HOME_WRIST,
                    HOME_GRIPPER,
                    HOME_AUX,
                    ARM_BASE_HOME_POSITION);

  beginArmPoseSequence(ARM_POSE_MOVING_HOME);
}

void requestArmReady() {
  setArmPoseTargets(READY_SHOULDER,
                    READY_ELBOW,
                    READY_WRIST,
                    READY_GRIPPER,
                    READY_AUX,
                    ARM_BASE_READY_POSITION);

  beginArmPoseSequence(ARM_POSE_MOVING_READY);
}

void stopAllArmMotion() {
  stopArmBaseMotion();
  armPoseStage = ARM_STAGE_IDLE;

  if (armPoseState == ARM_POSE_MOVING_HOME || armPoseState == ARM_POSE_MOVING_READY) {
    armPoseState = ARM_POSE_ACTIVE;
  }

  if (armPoseState != ARM_POSE_HOME) {
    writeArmServos();
  }
}


// ==================================================
// Staged HOME / READY Runner
// ==================================================
bool moveAngleTowardTarget(int &currentAngle, int targetAngle, int minAngle, int maxAngle) {
  currentAngle = clampAngle(currentAngle, minAngle, maxAngle);
  targetAngle = clampAngle(targetAngle, minAngle, maxAngle);

  if (currentAngle == targetAngle) {
    return true;
  }

  int diff = targetAngle - currentAngle;
  int stepSize = ARM_POSE_SERVO_STEP_DEG;

  if (diff > 0) {
    if (diff < stepSize) stepSize = diff;
    currentAngle += stepSize;
  } else {
    if (-diff < stepSize) stepSize = -diff;
    currentAngle -= stepSize;
  }

  return currentAngle == targetAngle;
}

void printPoseStageAck(const char *stageName) {
  if (armPoseState == ARM_POSE_MOVING_HOME) {
    Serial.print("ACK:ARM:HOME:STAGE:");
    Serial.println(stageName);
  } else if (armPoseState == ARM_POSE_MOVING_READY) {
    Serial.print("ACK:ARM:READY:STAGE:");
    Serial.println(stageName);
  }
}

void finishArmPoseSequence() {
  armPoseStage = ARM_STAGE_IDLE;

  if (armPoseState == ARM_POSE_MOVING_HOME) {
    armPoseState = ARM_POSE_HOME;
    pendingHomeDetach = true;
    homeDetachStartMs = millis();
    Serial.println("ACK:ARM:HOME:DONE");
    return;
  }

  if (armPoseState == ARM_POSE_MOVING_READY) {
    armPoseState = ARM_POSE_READY;
    pendingHomeDetach = false;
    writeArmServos();
    Serial.println("ACK:ARM:READY:DONE");
    return;
  }
}

void advanceArmPoseStage() {
  if (armPoseStage == ARM_STAGE_SHOULDER) {
    armPoseStage = ARM_STAGE_AUX;
    printPoseStageAck("AUX");
    return;
  }

  if (armPoseStage == ARM_STAGE_AUX) {
    armPoseStage = ARM_STAGE_GRIPPER;
    printPoseStageAck("GRIPPER");
    return;
  }

  if (armPoseStage == ARM_STAGE_GRIPPER) {
    armPoseStage = ARM_STAGE_WRIST;
    printPoseStageAck("WRIST");
    return;
  }

  if (armPoseStage == ARM_STAGE_WRIST) {
    armPoseStage = ARM_STAGE_ELBOW;
    printPoseStageAck("ELBOW");
    return;
  }

  if (armPoseStage == ARM_STAGE_ELBOW) {
    finishArmPoseSequence();
    return;
  }
}

void runArmPoseSequence() {
  if (!(armPoseState == ARM_POSE_MOVING_HOME || armPoseState == ARM_POSE_MOVING_READY)) {
    return;
  }

  if (armPoseStage == ARM_STAGE_IDLE || armPoseStage == ARM_STAGE_BASE_STEPPER) {
    return;
  }

  unsigned long now = millis();

  if (now - lastPoseServoMoveMs < ARM_POSE_SERVO_INTERVAL_MS) {
    return;
  }

  lastPoseServoMoveMs = now;

  bool stageDone = false;

  if (armPoseStage == ARM_STAGE_SHOULDER) {
    stageDone = moveAngleTowardTarget(shoulderAngle, targetShoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
  } else if (armPoseStage == ARM_STAGE_AUX) {
    stageDone = moveAngleTowardTarget(auxAngle, targetAuxAngle, AUX_MIN, AUX_MAX);
  } else if (armPoseStage == ARM_STAGE_GRIPPER) {
    stageDone = moveAngleTowardTarget(gripperAngle, targetGripperAngle, GRIPPER_MIN, GRIPPER_MAX);
  } else if (armPoseStage == ARM_STAGE_WRIST) {
    stageDone = moveAngleTowardTarget(wristAngle, targetWristAngle, WRIST_MIN, WRIST_MAX);
  } else if (armPoseStage == ARM_STAGE_ELBOW) {
    stageDone = moveAngleTowardTarget(elbowAngle, targetElbowAngle, ELBOW_MIN, ELBOW_MAX);
  }

  writeArmServos();

  if (stageDone) {
    advanceArmPoseStage();
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
  if (handleConfigCommand(cmd)) {
    return;
  }

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

  if (equalsCmd(cmd, "CAM:STEP_LEFT") || startsWithCmd(cmd, "CAM:STEP_LEFT:")) {
    cameraFiniteStepsRemaining = parseStepsOrDefault(cmd);
    cameraStepperDirection = -1;

    digitalWrite(CAM_DIR_PIN, LOW);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "CAM:STEP_RIGHT") || startsWithCmd(cmd, "CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = parseStepsOrDefault(cmd);
    cameraStepperDirection = 1;

    digitalWrite(CAM_DIR_PIN, HIGH);
    enableDriver(CAM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "CAM:UP")) {
    int delta = INVERT_CAMERA_VERTICAL ? -configuredServoStep : configuredServoStep;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (equalsCmd(cmd, "CAM:DOWN")) {
    int delta = INVERT_CAMERA_VERTICAL ? configuredServoStep : -configuredServoStep;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (startsWithCmd(cmd, "CAM:UP:")) {
    int step = parseServoStepOrDefault(cmd);
    int delta = INVERT_CAMERA_VERTICAL ? -step : step;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (startsWithCmd(cmd, "CAM:DOWN:")) {
    int step = parseServoStepOrDefault(cmd);
    int delta = INVERT_CAMERA_VERTICAL ? step : -step;
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
  if (handleConfigCommand(cmd)) {
    return;
  }

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

  if (equalsCmd(cmd, "ARM:BASE:STEP_LEFT") ||
      startsWithCmd(cmd, "ARM:BASE:STEP_LEFT:")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = parseStepsOrDefault(cmd);
    armBaseStepperDirection = -1;

    digitalWrite(ARM_DIR_PIN, LOW);
    enableDriver(ARM_EN_PIN);
    return;
  }

  if (equalsCmd(cmd, "ARM:BASE:STEP_RIGHT") ||
      startsWithCmd(cmd, "ARM:BASE:STEP_RIGHT:")) {
    markArmActive();

    armBaseTargetMode = false;
    armBaseFiniteStepsRemaining = parseStepsOrDefault(cmd);
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
  // Servo step is configurable from mobile.
  // ==================================================

  if (equalsCmd(cmd, "ARM:SHOULDER:UP")) {
    setShoulderAngle(shoulderAngle - configuredServoStep);
    return;
  }

  if (equalsCmd(cmd, "ARM:SHOULDER:DOWN")) {
    setShoulderAngle(shoulderAngle + configuredServoStep);
    return;
  }

  if (startsWithCmd(cmd, "ARM:SHOULDER:UP:")) {
    int step = parseServoStepOrDefault(cmd);
    setShoulderAngle(shoulderAngle - step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:SHOULDER:DOWN:")) {
    int step = parseServoStepOrDefault(cmd);
    setShoulderAngle(shoulderAngle + step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:SHOULDER:ANGLE:")) {
    setShoulderAngle((int)parseLastLong(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:UP")) {
    setElbowAngle(elbowAngle - configuredServoStep);
    return;
  }

  if (equalsCmd(cmd, "ARM:ELBOW:DOWN")) {
    setElbowAngle(elbowAngle + configuredServoStep);
    return;
  }

  if (startsWithCmd(cmd, "ARM:ELBOW:UP:")) {
    int step = parseServoStepOrDefault(cmd);
    setElbowAngle(elbowAngle - step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:ELBOW:DOWN:")) {
    int step = parseServoStepOrDefault(cmd);
    setElbowAngle(elbowAngle + step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:ELBOW:ANGLE:")) {
    setElbowAngle((int)parseLastLong(cmd));
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:UP")) {
    setWristAngle(wristAngle - configuredServoStep);
    return;
  }

  if (equalsCmd(cmd, "ARM:WRIST:DOWN")) {
    setWristAngle(wristAngle + configuredServoStep);
    return;
  }

  if (startsWithCmd(cmd, "ARM:WRIST:UP:")) {
    int step = parseServoStepOrDefault(cmd);
    setWristAngle(wristAngle - step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:WRIST:DOWN:")) {
    int step = parseServoStepOrDefault(cmd);
    setWristAngle(wristAngle + step);
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
    setAuxAngle(auxAngle - configuredServoStep);
    return;
  }

  if (equalsCmd(cmd, "ARM:AUX:DOWN")) {
    setAuxAngle(auxAngle + configuredServoStep);
    return;
  }

  if (startsWithCmd(cmd, "ARM:AUX:UP:")) {
    int step = parseServoStepOrDefault(cmd);
    setAuxAngle(auxAngle - step);
    return;
  }

  if (startsWithCmd(cmd, "ARM:AUX:DOWN:")) {
    int step = parseServoStepOrDefault(cmd);
    setAuxAngle(auxAngle + step);
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
  Serial.print("UNO_CONFIG:STEPPER_STEPS=");
  Serial.print(configuredStepperSteps);
  Serial.print(";SERVO_STEP=");
  Serial.println(configuredServoStep);
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
  runArmPoseSequence();

  refreshArmHold();
  handleHomeDetach();
  handleCameraDetach();
}