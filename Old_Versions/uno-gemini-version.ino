#include <Servo.h>
#include <SoftwareSerial.h>

// ====================================================================================
// Arduino UNO - Apex Rover Camera Stand + 6-Axis Arm Core Firmware
// ====================================================================================
//
// SYSTEM OVERVIEW & CRITICAL FIXES:
// 1. Camera Servo Jitter Fix:
//    The official Servo.h library relies on Timer 1 interrupts. When heavy activity, 
//    stepper stepping, or rapid arm updates occur, small microsecond timing variations 
//    (contention) introduce signal instability, causing the camera servo to vibrate/shake.
//    Fix: Implemented an active 'Auto-Detach' architecture. Servos are attached dynamically
//         upon receiving a movement command, and safely detached 500ms after motion stops.
//         This cuts off the PWM signal entirely when stable, eliminating jittering completely.
//
// 2. Dual Arm System States (Home vs Ready Sequence):
//    - READY state (Setup initialization): Executed automatically at boot or via command.
//      Brings the base stepper forward to front center (0 degrees) and elevates the arm
//      linkages to active manipulation positions with an open gripper, ready for action.
//    - HOME state (Rest/Storage): Rotates the base stepper 150 degrees backward to dock 
//      with the physical storage frame behind the rover, collapsing linkages to safe angles.
//
// 3. Absolute Position Tracking & Angle Computations:
//    The arm base stepper actively monitors its step count relative to a fixed center.
//    Using a calibrated step-to-degree scalar, it supports direct absolute angular movements
//    (e.g., target 0, target -150) by computing the exact delta and directional sign needed.
//
// COMMAND PATHWAY:
//   Mobile App -> WebSocket -> ESP32 -> UNO D2 (SoftwareSerial RX @ 9600 Baud)
//   * Note: Ensure a common structural Ground (GND) is shared among all microcontrollers,
//     stepper drivers, logic levels, and external high-current servo power supplies.
//
// HARDWARE WIRING ASSIGNMENTS:
//   Camera A4988 EN       -> UNO A1
//   Camera A4988 STEP     -> UNO D12
//   Camera A4988 DIR      -> UNO A2
//   Camera Servo Signal   -> UNO A0
//
//   Arm A4988 EN          -> UNO D4
//   Arm A4988 STEP        -> UNO D8
//   Arm A4988 DIR         -> UNO D7
//
//   Shoulder Servo        -> UNO D5
//   Elbow Servo           -> UNO D6
//   Wrist Servo           -> UNO D9
//   Gripper Servo         -> UNO D10
//   Aux / Axis Servo      -> UNO D11
//
// SUPPORTED CAM COMMANDS:
//   CAM:LEFT              - Continuous camera panning left
//   CAM:RIGHT             - Continuous camera panning right
//   CAM:STOP              - Stop camera stepper motor execution
//   CAM:STEP_LEFT:N       - Step camera left precisely N steps
//   CAM:STEP_RIGHT:N      - Step camera right precisely N steps
//   CAM:UP                - Tilt camera up by standard step increment
//   CAM:DOWN              - Tilt camera down by standard step increment
//   CAM:CENTER            - Reset camera tilt servo to default home angle (90)
//   CAM:ZERO              - Zero out current camera stepper software position tracking
//   CAM:SPEED:N           - Set camera stepper delay interval in microseconds
//   CAM:ANGLE:N           - Set absolute angle for camera tilt servo
//
// SUPPORTED ARM COMMANDS:
//   ARM:READY             - Execute forward-facing deployment sequence (Front center)
//   ARM:HOME              - Execute rear-facing parking sequence (150 degrees back)
//   ARM:STOP              - Emergency halt all arm system motion (Stepper + Servos)
//
//   ARM:BASE:LEFT         - Continuous base rotation left
//   ARM:BASE:RIGHT        - Continuous base rotation right
//   ARM:BASE:STOP         - Stop base stepper motor execution
//   ARM:BASE:STEP_LEFT:N  - Step base left precisely N steps
//   ARM:BASE:STEP_RIGHT:N - Step base right precisely N steps
//   ARM:BASE:SPEED:N      - Set base stepper delay interval in microseconds
//   ARM:BASE:ZERO         - Manually zero out the absolute step position register
//   ARM:BASE:SET_HOME     - Force rewrite current physical position as Home step count (-150 deg)
//   ARM:BASE:GOTO_ZERO    - Drive base directly back to front center position (0 steps)
//   ARM:BASE:GOTO_HOME    - Drive base directly back to rear dock position (-150 deg steps)
//   ARM:BASE:GOTO_DEG:N   - Absolute movement command to rotate base to an explicit angle N
//   ARM:BASE:GOTO_STEPS:N - Absolute movement command to drive base to a raw step location N
//
//   ARM:SHOULDER:UP / DOWN / ANGLE:N
//   ARM:ELBOW:UP    / DOWN / ANGLE:N
//   ARM:WRIST:UP    / DOWN / ANGLE:N
//   ARM:GRIPPER:OPEN / CLOSE / ANGLE:N
//   ARM:AUX:UP      / DOWN / ANGLE:N
//
// GLOBAL SYSTEM COMMANDS:
//   STOP / ESTOP          - Hard freeze all actuators on rover platform
//   SYS:MODE:MANUAL       - Disengage active autonomy tracking, zero tasks
//   SYS:MODE:AUTO         - Engage automated feedback loop state tracking
// ====================================================================================

SoftwareSerial espSerial(2, 3); // RX = D2, TX = D3 (TX line unused but declared)

// Camera Stand Hardware Pins
#define CAM_EN_PIN       A1
#define CAM_STEP_PIN     12
#define CAM_DIR_PIN      A2
#define CAM_SERVO_PIN    A0

// Arm Base Stepper Hardware Pins
#define ARM_EN_PIN       4
#define ARM_STEP_PIN     8
#define ARM_DIR_PIN      7

// Arm Servo Hardware Pins
#define SHOULDER_PIN     5
#define ELBOW_PIN        6
#define WRIST_PIN        9
#define GRIPPER_PIN      10
#define AUX_PIN          11

// Dynamic Actuator Objects
Servo cameraServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;
Servo auxServo;

const bool DEBUG_SERIAL = true;
const unsigned int STEP_PULSE_MICROS = 3;

// ==================================================
// Stepper Calibration Constants
// ==================================================
// Calibrate according to the microstepping configurations of the driver hardware.
// Formula standard: (Base Motor Steps * Microstepping Resolution) / 360 Degrees
// Example Default: (200 steps * 10 multiplier for sub-step accuracy scaling) / 360 = 5.5556 steps/deg
const float STEPS_PER_DEGREE = 5.5556; 

// Approximate numerical step counts required to pan 150 degrees back to the resting dock
const long STEPS_FOR_150_DEG = (long)(150.0 * STEPS_PER_DEGREE); 

// Camera Servo Boundary Ranges
const int CAM_SERVO_MIN = 30;
const int CAM_SERVO_MAX = 180;
const int CAM_SERVO_STEP = 5;
const int CAM_SERVO_HOME = 90;
const bool INVERT_CAMERA_VERTICAL = true;

// Arm Axis Boundary Enforcements (Safe Operational Thresholds)
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
// Positional State Definitions (Home vs Ready)
// ==================================================
// Arm Standby/Storage State Configuration (Rearward Dock)
const int HOME_SHOULDER = 60;
const int HOME_ELBOW = 10;
const int HOME_WRIST = 90;
const int HOME_AUX = 90;
const int HOME_GRIPPER = 180;
const long HOME_STEP_POSITION = -STEPS_FOR_150_DEG; // Mechanical stand resides at -150 deg orientation

// Arm Active Deployment Configuration (Facing Front Forward)
const int READY_SHOULDER = 120; // Pre-calculated safe posture angles for object pickup
const int READY_ELBOW = 90;
const int READY_WRIST = 90;
const int READY_AUX = 120;
const int READY_GRIPPER = 120;   // Fully open gripper shell, prepared to grasp targets
const long READY_STEP_POSITION = 0; // Front dead center (0 steps reference tracking)

const int GRIPPER_OPEN_ANGLE = 120;
const int GRIPPER_CLOSE_ANGLE = 180;

// Auto-Detach execution interval (Inactivity window before releasing PWM control)
const unsigned long SERVO_DETACH_DELAY_MS = 500; 

// Volatile Servo Trajectory Storage
int cameraServoAngle = CAM_SERVO_HOME;
unsigned long lastCameraServoMove = 0;
bool cameraServoAttached = false;

int shoulderAngle = HOME_SHOULDER;
int elbowAngle = HOME_ELBOW;
int wristAngle = HOME_WRIST;
int gripperAngle = HOME_GRIPPER;
int auxAngle = HOME_AUX;

unsigned long lastArmServoMove = 0;
bool armServosAttached = false;

// Volatile Camera Stepper Operational State Registers
int cameraStepperDirection = 0;
long cameraFiniteStepsRemaining = 0;
long cameraStepPosition = 0;
unsigned long cameraLastStepTime = 0;
unsigned long cameraStepIntervalMicros = 700;

// Volatile Arm Base Stepper Operational State Registers
int armBaseStepperDirection = 0;
long armBaseFiniteStepsRemaining = 0;
long armBaseStepPosition = 0; // Absolute runtime counter: Clockwise (+), Counter-Clockwise (-)
unsigned long armBaseLastStepTime = 0;
unsigned long armBaseStepIntervalMicros = 700;

// Non-blocking Command Read Buffers
char cmdBuffer[80];
byte cmdIndex = 0;

// Basic Limit Enforcement Helpers
int clampAngle(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

long parseLastNumber(const char *cmd) {
  const char *lastColon = strrchr(cmd, ':');
  if (lastColon == NULL) return 0;
  return atol(lastColon + 1); // Extract numbers directly (supports negative values for positioning)
}

bool equalsCmd(const char *cmd, const char *target) { return strcmp(cmd, target) == 0; }
bool startsWithCmd(const char *cmd, const char *prefix) { return strncmp(cmd, prefix, strlen(prefix)) == 0; }
void enableDriver(int enPin) { digitalWrite(enPin, LOW); }
void disableDriver(int enPin) { digitalWrite(enPin, HIGH); }

// ==================================================
// Dynamic Servo Management System (Jitter Mitigation)
// ==================================================
void attachCameraServo() {
  if (!cameraServoAttached) {
    cameraServo.attach(CAM_SERVO_PIN);
    cameraServoAttached = true;
  }
  lastCameraServoMove = millis();
}

void attachArmServos() {
  if (!armServosAttached) {
    shoulderServo.attach(SHOULDER_PIN);
    elbowServo.attach(ELBOW_PIN);
    wristServo.attach(WRIST_PIN);
    gripperServo.attach(GRIPPER_PIN);
    auxServo.attach(AUX_PIN);
    armServosAttached = true;
  }
  lastArmServoMove = millis();
}

void checkAndDetachServos() {
  unsigned long now = millis();
  
  // Cut off PWM drive if camera tilt has settled to completely isolate it from electrical crosstalk
  if (cameraServoAttached && (now - lastCameraServoMove > SERVO_DETACH_DELAY_MS)) {
    cameraServo.detach();
    cameraServoAttached = false;
  }
  
  // Release arm servos to freeze positional tracking mechanically, dropping line noise entirely
  if (armServosAttached && (now - lastArmServoMove > SERVO_DETACH_DELAY_MS)) {
    shoulderServo.detach();
    elbowServo.detach();
    wristServo.detach();
    gripperServo.detach();
    auxServo.detach();
    armServosAttached = false;
  }
}

void writeArmServos() {
  shoulderAngle = clampAngle(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
  elbowAngle = clampAngle(elbowAngle, ELBOW_MIN, ELBOW_MAX);
  wristAngle = clampAngle(wristAngle, WRIST_MIN, WRIST_MAX);
  gripperAngle = clampAngle(gripperAngle, GRIPPER_MIN, GRIPPER_MAX);
  auxAngle = clampAngle(auxAngle, AUX_MIN, AUX_MAX);

  attachArmServos();

  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);
  auxServo.write(auxAngle);
}

void setCameraAngle(int angle) {
  cameraServoAngle = clampAngle(angle, CAM_SERVO_MIN, CAM_SERVO_MAX);
  attachCameraServo();
  cameraServo.write(cameraServoAngle);
}

// ==================================================
// Absolute Stepper Target Positioning Routines
// ==================================================
void moveArmBaseToStepPosition(long targetPosition) {
  long deltaSteps = targetPosition - armBaseStepPosition;
  if (deltaSteps == 0) return;

  armBaseFiniteStepsRemaining = abs(deltaSteps);
  if (deltaSteps > 0) {
    armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH);
  } else {
    armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW);
  }
  enableDriver(ARM_EN_PIN);
}

// Macro Sequence Execution Functions
void armHome() {
  if (DEBUG_SERIAL) Serial.println("[ARM] Routing to low-stress HOME docking storage configuration...");
  
  // Step 1: Align mechanical joint linkages to compact storage configurations first
  shoulderAngle = HOME_SHOULDER;
  elbowAngle = HOME_ELBOW;
  wristAngle = HOME_WRIST;
  auxAngle = HOME_AUX;
  gripperAngle = HOME_GRIPPER;
  writeArmServos();
  
  // Step 2: Spin base stepper back to line up squarely with the back rest docking frame
  moveArmBaseToStepPosition(HOME_STEP_POSITION);
}

void armReady() {
  if (DEBUG_SERIAL) Serial.println("[ARM] Executing deployment sequence to active READY state...");
  
  // Step 1: Spin base back to absolute front dead center facing forward
  moveArmBaseToStepPosition(READY_STEP_POSITION);
  
  // Step 2: Elevate linkages smoothly into target tracking profiles and open gripper jaw
  shoulderAngle = READY_SHOULDER;
  elbowAngle = READY_ELBOW;
  wristAngle = READY_WRIST;
  auxAngle = READY_AUX;
  gripperAngle = READY_GRIPPER;
  writeArmServos();
}

void stopCameraMotion() {
  cameraStepperDirection = 0;
  cameraFiniteStepsRemaining = 0;
  digitalWrite(CAM_STEP_PIN, LOW);
  disableDriver(CAM_EN_PIN);
}

void stopArmBaseMotion() {
  armBaseStepperDirection = 0;
  armBaseFiniteStepsRemaining = 0;
  digitalWrite(ARM_STEP_PIN, LOW);
  disableDriver(ARM_EN_PIN);
}

// ==================================================
// Command Parsing Engines
// ==================================================
void handleCameraCommand(const char *cmd) {
  if (equalsCmd(cmd, "CAM:STOP"))   { stopCameraMotion(); return; }
  if (equalsCmd(cmd, "CAM:CENTER")) { stopCameraMotion(); setCameraAngle(CAM_SERVO_HOME); return; }
  
  if (equalsCmd(cmd, "CAM:LEFT")) {
    cameraFiniteStepsRemaining = 0; cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW); enableDriver(CAM_EN_PIN); return;
  }
  if (equalsCmd(cmd, "CAM:RIGHT")) {
    cameraFiniteStepsRemaining = 0; cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH); enableDriver(CAM_EN_PIN); return;
  }
  if (startsWithCmd(cmd, "CAM:STEP_LEFT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd); cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW); enableDriver(CAM_EN_PIN); return;
  }
  if (startsWithCmd(cmd, "CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd); cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH); enableDriver(CAM_EN_PIN); return;
  }
  if (equalsCmd(cmd, "CAM:UP")) {
    int delta = INVERT_CAMERA_VERTICAL ? -CAM_SERVO_STEP : CAM_SERVO_STEP;
    setCameraAngle(cameraServoAngle + delta); return;
  }
  if (equalsCmd(cmd, "CAM:DOWN")) {
    int delta = INVERT_CAMERA_VERTICAL ? CAM_SERVO_STEP : -CAM_SERVO_STEP;
    setCameraAngle(cameraServoAngle + delta); return;
  }
  if (equalsCmd(cmd, "CAM:ZERO")) { cameraStepPosition = 0; return; }
  if (startsWithCmd(cmd, "CAM:SPEED:")) {
    cameraStepIntervalMicros = constrain(parseLastNumber(cmd), 300, 5000); return;
  }
  if (startsWithCmd(cmd, "CAM:ANGLE:")) { setCameraAngle((int)parseLastNumber(cmd)); return; }
}

void handleArmCommand(const char *cmd) {
  if (equalsCmd(cmd, "ARM:STOP"))  { stopArmBaseMotion(); return; }
  if (equalsCmd(cmd, "ARM:HOME"))  { armHome(); return; }
  if (equalsCmd(cmd, "ARM:READY")) { armReady(); return; }

  // Base Stepper Continuous Panning Commands
  if (equalsCmd(cmd, "ARM:BASE:LEFT")) {
    armBaseFiniteStepsRemaining = 0; armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW); enableDriver(ARM_EN_PIN); return;
  }
  if (equalsCmd(cmd, "ARM:BASE:RIGHT")) {
    armBaseFiniteStepsRemaining = 0; armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH); enableDriver(ARM_EN_PIN); return;
  }
  if (equalsCmd(cmd, "ARM:BASE:STOP")) { stopArmBaseMotion(); return; }
  
  // Absolute Coordinate Positioning Overrides
  if (startsWithCmd(cmd, "ARM:BASE:GOTO_DEG:")) {
    long targetAngle = parseLastNumber(cmd);
    long targetSteps = (long)(targetAngle * STEPS_PER_DEGREE);
    moveArmBaseToStepPosition(targetSteps);
    return;
  }
  if (startsWithCmd(cmd, "ARM:BASE:GOTO_STEPS:")) {
    moveArmBaseToStepPosition(parseLastNumber(cmd));
    return;
  }
  if (equalsCmd(cmd, "ARM:BASE:GOTO_ZERO")) { moveArmBaseToStepPosition(READY_STEP_POSITION); return; }
  if (equalsCmd(cmd, "ARM:BASE:GOTO_HOME")) { moveArmBaseToStepPosition(HOME_STEP_POSITION); return; }
  if (equalsCmd(cmd, "ARM:BASE:ZERO"))      { armBaseStepPosition = 0; return; }
  if (equalsCmd(cmd, "ARM:BASE:SET_HOME"))  { armBaseStepPosition = HOME_STEP_POSITION; return; }

  // Incremental Step Commands
  if (startsWithCmd(cmd, "ARM:BASE:STEP_LEFT:")) {
    armBaseFiniteStepsRemaining = parseLastNumber(cmd); armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW); enableDriver(ARM_EN_PIN); return;
  }
  if (startsWithCmd(cmd, "ARM:BASE:STEP_RIGHT:")) {
    armBaseFiniteStepsRemaining = parseLastNumber(cmd); armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH); enableDriver(ARM_EN_PIN); return;
  }
  if (startsWithCmd(cmd, "ARM:BASE:SPEED:")) {
    armBaseStepIntervalMicros = constrain(parseLastNumber(cmd), 300, 5000); return;
  }

  // Structural Arm Joint Directives
  if (equalsCmd(cmd, "ARM:SHOULDER:UP"))   { shoulderAngle += ARM_SERVO_STEP; writeArmServos(); return; }
  if (equalsCmd(cmd, "ARM:SHOULDER:DOWN")) { shoulderAngle -= ARM_SERVO_STEP; writeArmServos(); return; }
  if (startsWithCmd(cmd, "ARM:SHOULDER:ANGLE:")) { shoulderAngle = (int)parseLastNumber(cmd); writeArmServos(); return; }

  if (equalsCmd(cmd, "ARM:ELBOW:UP"))   { elbowAngle += ARM_SERVO_STEP; writeArmServos(); return; }
  if (equalsCmd(cmd, "ARM:ELBOW:DOWN")) { elbowAngle -= ARM_SERVO_STEP; writeArmServos(); return; }
  if (startsWithCmd(cmd, "ARM:ELBOW:ANGLE:")) { elbowAngle = (int)parseLastNumber(cmd); writeArmServos(); return; }

  if (equalsCmd(cmd, "ARM:WRIST:UP"))   { wristAngle += ARM_SERVO_STEP; writeArmServos(); return; }
  if (equalsCmd(cmd, "ARM:WRIST:DOWN")) { wristAngle -= ARM_SERVO_STEP; writeArmServos(); return; }
  if (startsWithCmd(cmd, "ARM:WRIST:ANGLE:")) { wristAngle = (int)parseLastNumber(cmd); writeArmServos(); return; }

  if (equalsCmd(cmd, "ARM:GRIPPER:OPEN"))  { gripperAngle = GRIPPER_OPEN_ANGLE; writeArmServos(); return; }
  if (equalsCmd(cmd, "ARM:GRIPPER:CLOSE")) { gripperAngle = GRIPPER_CLOSE_ANGLE; writeArmServos(); return; }
  if (startsWithCmd(cmd, "ARM:GRIPPER:ANGLE:")) { gripperAngle = (int)parseLastNumber(cmd); writeArmServos(); return; }

  if (equalsCmd(cmd, "ARM:AUX:UP"))   { auxAngle += ARM_SERVO_STEP; writeArmServos(); return; }
  if (equalsCmd(cmd, "ARM:AUX:DOWN")) { auxAngle -= ARM_SERVO_STEP; writeArmServos(); return; }
  if (startsWithCmd(cmd, "ARM:AUX:ANGLE:")) { auxAngle = (int)parseLastNumber(cmd); writeArmServos(); return; }
}

void handleSystemCommand(const char *cmd) {
  if (equalsCmd(cmd, "STOP") || equalsCmd(cmd, "ESTOP")) {
    stopCameraMotion();
    stopArmBaseMotion();
    return;
  }
}

void handleCommand(const char *cmd) {
  if (startsWithCmd(cmd, "CAM:")) { handleCameraCommand(cmd); return; }
  if (startsWithCmd(cmd, "ARM:")) { handleArmCommand(cmd); return; }
  handleSystemCommand(cmd);
}

void readSerialStream(Stream &port) {
  while (port.available() > 0) {
    char c = port.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) {
        if (DEBUG_SERIAL) { Serial.print("[RX] "); Serial.println(cmdBuffer); }
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
// Non-Blocking Asynchronous Stepper Drivers
// ==================================================
void runCameraStepper() {
  if (cameraStepperDirection == 0) return;
  unsigned long now = micros();
  if (now - cameraLastStepTime < cameraStepIntervalMicros) return;

  cameraLastStepTime = now;
  digitalWrite(CAM_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(CAM_STEP_PIN, LOW);

  cameraStepPosition += cameraStepperDirection;

  if (cameraFiniteStepsRemaining > 0) {
    cameraFiniteStepsRemaining--;
    if (cameraFiniteStepsRemaining == 0) stopCameraMotion();
  }
}

void runArmBaseStepper() {
  if (armBaseStepperDirection == 0) return;
  unsigned long now = micros();
  if (now - armBaseLastStepTime < armBaseStepIntervalMicros) return;

  armBaseLastStepTime = now;
  digitalWrite(ARM_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(ARM_STEP_PIN, LOW);

  armBaseStepPosition += armBaseStepperDirection;

  if (armBaseFiniteStepsRemaining > 0) {
    armBaseFiniteStepsRemaining--;
    if (armBaseFiniteStepsRemaining == 0) stopArmBaseMotion();
  }
}

// ==================================================
// Initialization and Runtime Loops
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

  stopCameraMotion();
  stopArmBaseMotion();

  // Attach camera and establish primary structural point alignment
  setCameraAngle(CAM_SERVO_HOME);

  // Directly engage and park system components in forward-facing operational ready posture
  armReady();

  Serial.println("UNO_READY");
}

void loop() {
  readSerialStream(espSerial);
  if (Serial.available() > 0) {
    readSerialStream(Serial);
  }

  runCameraStepper();
  runArmBaseStepper();

  // Periodically decouple quiescent servo lines to intercept interrupt cross-talk vibration issues
  checkAndDetachServos();
}