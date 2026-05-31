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
//   ARM:ELBOW:UP
//   ARM:ELBOW:DOWN
//   ARM:WRIST:UP
//   ARM:WRIST:DOWN
//   ARM:GRIPPER:OPEN
//   ARM:GRIPPER:CLOSE
//   ARM:AUX:UP
//   ARM:AUX:DOWN
//   ARM:HOME
//   ARM:STOP
// ==================================================


// ==================================================
// ESP32 -> UNO Serial
// ==================================================
// ESP32 GPIO4 TX -> UNO D2 RX
// D3 TX is not wired, but SoftwareSerial needs a TX pin.
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
// From your note:
// EN = 4, STEP = 8, DIR = 7
// ==================================================
#define ARM_EN_PIN       4
#define ARM_STEP_PIN     8
#define ARM_DIR_PIN      7


// ==================================================
// Arm Servo Pins
// From your note:
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
// System State
// ==================================================
String systemMode = "MANUAL";


// ==================================================
// Camera Servo State
// ==================================================
int cameraServoAngle = 90;

const int CAM_SERVO_MIN = 30;
const int CAM_SERVO_MAX = 150;
const int CAM_SERVO_STEP = 5;
const bool INVERT_CAMERA_VERTICAL = true;


// ==================================================
// Camera Stepper State
// -1 = left, 0 = stop, 1 = right
// ==================================================
int cameraStepperDirection = 0;

long cameraFiniteStepsRemaining = 0;
long cameraStepPosition = 0;

unsigned long cameraLastStepTime = 0;
unsigned long cameraStepIntervalMicros = 700;

const unsigned int STEP_PULSE_MICROS = 3;


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
// Arm Servo Angles
// Adjust these limits after testing the real arm.
// Start safe.
// ==================================================
int shoulderAngle = 90;
int elbowAngle = 90;
int wristAngle = 90;
int gripperAngle = 90;
int auxAngle = 90;

const int ARM_SERVO_STEP = 5;

const int SHOULDER_MIN = 20;
const int SHOULDER_MAX = 160;

const int ELBOW_MIN = 20;
const int ELBOW_MAX = 160;

const int WRIST_MIN = 20;
const int WRIST_MAX = 160;

const int AUX_MIN = 20;
const int AUX_MAX = 160;

// عدّلهم حسب الجريبر عندكم إذا الفتح/الإغلاق بالعكس
const int GRIPPER_OPEN_ANGLE = 35;
const int GRIPPER_CLOSE_ANGLE = 100;


// ==================================================
// Setup
// ==================================================
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  espSerial.begin(9600);
  espSerial.setTimeout(50);

  // Camera stepper
  pinMode(CAM_EN_PIN, OUTPUT);
  pinMode(CAM_STEP_PIN, OUTPUT);
  pinMode(CAM_DIR_PIN, OUTPUT);

  digitalWrite(CAM_EN_PIN, LOW);
  digitalWrite(CAM_STEP_PIN, LOW);
  digitalWrite(CAM_DIR_PIN, LOW);

  // Arm base stepper
  pinMode(ARM_EN_PIN, OUTPUT);
  pinMode(ARM_STEP_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);

  digitalWrite(ARM_EN_PIN, LOW);
  digitalWrite(ARM_STEP_PIN, LOW);
  digitalWrite(ARM_DIR_PIN, LOW);

  // Camera servo
  cameraServo.attach(CAM_SERVO_PIN);
  cameraServo.write(cameraServoAngle);

  // Arm servos
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  gripperServo.attach(GRIPPER_PIN);
  auxServo.attach(AUX_PIN);

  armHome();

  Serial.println("UNO_READY:CAMERA_STAND_AND_ARM");
  Serial.println("UNO_RECEIVE_ONLY_MODE");
}


// ==================================================
// Loop
// ==================================================
void loop() {
  readCommand();

  runCameraStepper();
  runArmBaseStepper();
}


// ==================================================
// Read command from ESP32
// ==================================================
void readCommand() {
  if (espSerial.available() == 0) return;

  String cmd = espSerial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  Serial.print("[UNO RX] ");
  Serial.println(cmd);

  handleCommand(cmd);
}


// ==================================================
// Helpers
// ==================================================
long parseLastNumber(String cmd) {
  int lastColon = cmd.lastIndexOf(':');

  if (lastColon < 0 || lastColon >= (int)cmd.length() - 1) {
    return 0;
  }

  long value = cmd.substring(lastColon + 1).toInt();

  if (value < 0) value = -value;

  return constrain(value, 1, 50000);
}

void stopCameraMotion() {
  cameraStepperDirection = 0;
  cameraFiniteStepsRemaining = 0;
  digitalWrite(CAM_STEP_PIN, LOW);
}

void stopArmBaseMotion() {
  armBaseStepperDirection = 0;
  armBaseFiniteStepsRemaining = 0;
  digitalWrite(ARM_STEP_PIN, LOW);
}

void stopAllArmMotion() {
  stopArmBaseMotion();

  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);
  auxServo.write(auxAngle);
}

int safeAngle(int value, int minAngle, int maxAngle) {
  return constrain(value, minAngle, maxAngle);
}

void applyArmServos() {
  shoulderAngle = safeAngle(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
  elbowAngle = safeAngle(elbowAngle, ELBOW_MIN, ELBOW_MAX);
  wristAngle = safeAngle(wristAngle, WRIST_MIN, WRIST_MAX);
  auxAngle = safeAngle(auxAngle, AUX_MIN, AUX_MAX);

  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);
  auxServo.write(auxAngle);
}

void armHome() {
  stopArmBaseMotion();

  shoulderAngle = 90;
  elbowAngle = 90;
  wristAngle = 90;
  auxAngle = 90;
  gripperAngle = GRIPPER_OPEN_ANGLE;

  applyArmServos();
}


// ==================================================
// Command Handler
// ==================================================
void handleCommand(String cmd) {

  // ==================================================
  // System mode
  // ==================================================
  if (cmd == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }

  if (cmd == "SYS:MODE:AUTO") {
    // AUTO is not used now, but kept for compatibility.
    systemMode = "AUTO";
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }


  // ==================================================
  // Global Stop
  // ==================================================
  if (cmd == "STOP" || cmd == "ESTOP") {
    stopCameraMotion();
    stopAllArmMotion();
    return;
  }


  // ==================================================
  // Camera Stand Commands - kept same behavior
  // ==================================================
  if (cmd == "CAM:STOP") {
    stopCameraMotion();
    return;
  }

  if (cmd == "CAM:CENTER") {
    stopCameraMotion();
    cameraServoAngle = 90;
    cameraServo.write(cameraServoAngle);
    return;
  }

  if (cmd == "CAM:LEFT") {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW);
    digitalWrite(CAM_EN_PIN, LOW);
    return;
  }

  if (cmd == "CAM:RIGHT") {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH);
    digitalWrite(CAM_EN_PIN, LOW);
    return;
  }

  if (cmd.startsWith("CAM:STEP_LEFT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd);
    cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW);
    digitalWrite(CAM_EN_PIN, LOW);
    return;
  }

  if (cmd.startsWith("CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = parseLastNumber(cmd);
    cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH);
    digitalWrite(CAM_EN_PIN, LOW);
    return;
  }

  if (cmd == "CAM:UP") {
    cameraServoAngle += INVERT_CAMERA_VERTICAL ? -CAM_SERVO_STEP : CAM_SERVO_STEP;
    cameraServoAngle = constrain(cameraServoAngle, CAM_SERVO_MIN, CAM_SERVO_MAX);
    cameraServo.write(cameraServoAngle);
    return;
  }

  if (cmd == "CAM:DOWN") {
    cameraServoAngle += INVERT_CAMERA_VERTICAL ? CAM_SERVO_STEP : -CAM_SERVO_STEP;
    cameraServoAngle = constrain(cameraServoAngle, CAM_SERVO_MIN, CAM_SERVO_MAX);
    cameraServo.write(cameraServoAngle);
    return;
  }

  if (cmd.startsWith("CAM:SPEED:")) {
    long interval = parseLastNumber(cmd);
    cameraStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  if (cmd == "CAM:ZERO") {
    cameraStepPosition = 0;
    return;
  }


  // ==================================================
  // Arm Base Stepper Commands
  // ==================================================
  if (cmd == "ARM:BASE:LEFT") {
    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW);
    digitalWrite(ARM_EN_PIN, LOW);
    return;
  }

  if (cmd == "ARM:BASE:RIGHT") {
    armBaseFiniteStepsRemaining = 0;
    armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH);
    digitalWrite(ARM_EN_PIN, LOW);
    return;
  }

  if (cmd == "ARM:BASE:STOP") {
    stopArmBaseMotion();
    return;
  }

  if (cmd.startsWith("ARM:BASE:STEP_LEFT:")) {
    armBaseFiniteStepsRemaining = parseLastNumber(cmd);
    armBaseStepperDirection = -1;
    digitalWrite(ARM_DIR_PIN, LOW);
    digitalWrite(ARM_EN_PIN, LOW);
    return;
  }

  if (cmd.startsWith("ARM:BASE:STEP_RIGHT:")) {
    armBaseFiniteStepsRemaining = parseLastNumber(cmd);
    armBaseStepperDirection = 1;
    digitalWrite(ARM_DIR_PIN, HIGH);
    digitalWrite(ARM_EN_PIN, LOW);
    return;
  }

  if (cmd.startsWith("ARM:BASE:SPEED:")) {
    long interval = parseLastNumber(cmd);
    armBaseStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }


  // ==================================================
  // Arm General Commands
  // ==================================================
  if (cmd == "ARM:STOP") {
    stopAllArmMotion();
    return;
  }

  if (cmd == "ARM:HOME") {
    armHome();
    return;
  }


  // ==================================================
  // Shoulder Servo
  // ==================================================
  if (cmd == "ARM:SHOULDER:UP") {
    shoulderAngle += ARM_SERVO_STEP;
    shoulderAngle = safeAngle(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
    shoulderServo.write(shoulderAngle);
    return;
  }

  if (cmd == "ARM:SHOULDER:DOWN") {
    shoulderAngle -= ARM_SERVO_STEP;
    shoulderAngle = safeAngle(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
    shoulderServo.write(shoulderAngle);
    return;
  }


  // ==================================================
  // Elbow Servo
  // ==================================================
  if (cmd == "ARM:ELBOW:UP") {
    elbowAngle += ARM_SERVO_STEP;
    elbowAngle = safeAngle(elbowAngle, ELBOW_MIN, ELBOW_MAX);
    elbowServo.write(elbowAngle);
    return;
  }

  if (cmd == "ARM:ELBOW:DOWN") {
    elbowAngle -= ARM_SERVO_STEP;
    elbowAngle = safeAngle(elbowAngle, ELBOW_MIN, ELBOW_MAX);
    elbowServo.write(elbowAngle);
    return;
  }


  // ==================================================
  // Wrist Servo
  // ==================================================
  if (cmd == "ARM:WRIST:UP") {
    wristAngle += ARM_SERVO_STEP;
    wristAngle = safeAngle(wristAngle, WRIST_MIN, WRIST_MAX);
    wristServo.write(wristAngle);
    return;
  }

  if (cmd == "ARM:WRIST:DOWN") {
    wristAngle -= ARM_SERVO_STEP;
    wristAngle = safeAngle(wristAngle, WRIST_MIN, WRIST_MAX);
    wristServo.write(wristAngle);
    return;
  }


  // ==================================================
  // Gripper Servo
  // ==================================================
  if (cmd == "ARM:GRIPPER:OPEN") {
    gripperAngle = GRIPPER_OPEN_ANGLE;
    gripperServo.write(gripperAngle);
    return;
  }

  if (cmd == "ARM:GRIPPER:CLOSE") {
    gripperAngle = GRIPPER_CLOSE_ANGLE;
    gripperServo.write(gripperAngle);
    return;
  }


  // ==================================================
  // Aux / Sixth Axis Servo
  // These commands are optional if mobile app adds buttons later.
  // ==================================================
  if (cmd == "ARM:AUX:UP") {
    auxAngle += ARM_SERVO_STEP;
    auxAngle = safeAngle(auxAngle, AUX_MIN, AUX_MAX);
    auxServo.write(auxAngle);
    return;
  }

  if (cmd == "ARM:AUX:DOWN") {
    auxAngle -= ARM_SERVO_STEP;
    auxAngle = safeAngle(auxAngle, AUX_MIN, AUX_MAX);
    auxServo.write(auxAngle);
    return;
  }
}


// ==================================================
// Camera Stepper Runner - Non Blocking
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

    if (cameraFiniteStepsRemaining == 0) {
      stopCameraMotion();
    }
  }
}


// ==================================================
// Arm Base Stepper Runner - Non Blocking
// ==================================================
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

    if (armBaseFiniteStepsRemaining == 0) {
      stopArmBaseMotion();
    }
  }
}