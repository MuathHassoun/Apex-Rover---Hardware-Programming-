#include <Servo.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Mount + ARM Placeholder V2
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
//
// V2 Updates:
// - Full stepper control
// - Continuous left/right movement
// - Stop command
// - Enable / disable A4988
// - Set direction only
// - Move exact number of steps
// - Change speed from Raspberry Pi
// - Status command
// ==================================================


// ==================================================
// A4988 Stepper Driver Pins
// ==================================================
//
// A4988 EN   -> Arduino UNO Pin A1
// A4988 STEP -> Arduino UNO Pin D12
// A4988 DIR  -> Arduino UNO Pin D13
//
// A4988:
// EN LOW  = enabled
// EN HIGH = disabled
//
// Important:
// RESET and SLEEP should be connected together,
// then connected to 5V.
// VMOT should be connected to motor power.
// VMOT GND and Arduino GND must be common.
// ==================================================

#define EN_PIN   A1
#define STEP_PIN 12
#define DIR_PIN  13


// ==================================================
// Camera Servo Pin
// ==================================================
//
// Servo Signal -> Arduino UNO Pin A0
// Servo VCC    -> External 5V / 6V supply
// Servo GND    -> External power GND
// UNO GND      -> External power GND
// ==================================================

#define CAMERA_SERVO_PIN A0


// ==================================================
// Servo Settings
// ==================================================

Servo cameraServo;

int cameraAngle = 90;

const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_STEP = 5;

// If UP/DOWN is reversed, keep this true.
// If UP/DOWN becomes correct without it, change to false.
const bool INVERT_SERVO_VERTICAL = true;


// ==================================================
// Stepper Settings
// ==================================================
//
// cameraDirection:
// -1 = left
//  0 = stop
//  1 = right
//
// stepperMode:
// 0 = stopped
// 1 = continuous movement
// 2 = finite steps movement
// ==================================================

int cameraDirection = 0;
int stepperMode = 0;

bool stepperEnabled = false;

long stepsRemaining = 0;

unsigned long lastStepTime = 0;

// Smaller value = faster stepper
// Larger value  = slower stepper
unsigned long stepIntervalMicros = 1200;

// A4988 needs a short HIGH pulse on STEP
const unsigned int STEP_PULSE_MICROS = 3;

// false = motor holds position when stopped
// true  = motor becomes free/silent when stopped
const bool DISABLE_STEPPER_WHEN_STOPPED = false;


// ==================================================
// ARM Placeholder Settings
// ==================================================
//
// Keep this section for future advanced arm control.
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
  Serial.begin(9600);
  Serial.setTimeout(10);

  // Stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // Enable A4988 at startup
  enableStepper();

  // Servo setup
  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(cameraAngle);

  Serial.println("====================================");
  Serial.println("UNO Ready - Apex Rover Camera Mount V2");
  Serial.println("Stepper EN   : A1");
  Serial.println("Stepper STEP : D12");
  Serial.println("Stepper DIR  : D13");
  Serial.println("Servo Pin    : A0");
  Serial.println("ARM placeholder enabled");
  Serial.println("====================================");

  Serial.println("Available CAM commands:");
  Serial.println("CAM:LEFT          -> continuous left");
  Serial.println("CAM:RIGHT         -> continuous right");
  Serial.println("CAM:START         -> start continuous in last direction");
  Serial.println("CAM:STOP          -> stop stepper");
  Serial.println("CAM:ENABLE        -> enable A4988");
  Serial.println("CAM:DISABLE       -> disable A4988");
  Serial.println("CAM:DIR:LEFT      -> set direction left only");
  Serial.println("CAM:DIR:RIGHT     -> set direction right only");
  Serial.println("CAM:STEPS:200     -> move exact number of steps");
  Serial.println("CAM:SPEED:1200    -> set step interval in microseconds");
  Serial.println("CAM:UP            -> servo up");
  Serial.println("CAM:DOWN          -> servo down");
  Serial.println("CAM:CENTER        -> center servo and stop stepper");
  Serial.println("CAM:STATUS        -> camera status");

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
// Stepper Low-Level Control
// ==================================================

void enableStepper() {
  digitalWrite(EN_PIN, LOW);     // A4988 enabled
  stepperEnabled = true;
}

void disableStepper() {
  stopStepper(false);
  digitalWrite(EN_PIN, HIGH);    // A4988 disabled
  stepperEnabled = false;
}

void setStepperDirection(int dir) {
  if (dir < 0) {
    cameraDirection = -1;
    digitalWrite(DIR_PIN, LOW);
  }

  else if (dir > 0) {
    cameraDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
  }

  else {
    cameraDirection = 0;
  }
}

void startStepperContinuous() {
  // If no direction selected, default right
  if (cameraDirection == 0) {
    setStepperDirection(1);
  }

  enableStepper();

  stepperMode = 1;
  stepsRemaining = 0;

  Serial.println("ACK:CAM:START");
}

void startStepperSteps(long stepCount) {
  if (stepCount <= 0) {
    Serial.println("ERROR:CAM:STEPS_INVALID");
    return;
  }

  // If no direction selected, default right
  if (cameraDirection == 0) {
    setStepperDirection(1);
  }

  enableStepper();

  stepperMode = 2;
  stepsRemaining = stepCount;

  Serial.print("ACK:CAM:STEPS:");
  Serial.println(stepsRemaining);
}

void stopStepper(bool sendAck = true) {
  stepperMode = 0;
  cameraDirection = 0;
  stepsRemaining = 0;

  digitalWrite(STEP_PIN, LOW);

  if (DISABLE_STEPPER_WHEN_STOPPED) {
    digitalWrite(EN_PIN, HIGH);
    stepperEnabled = false;
  }

  if (sendAck) {
    Serial.println("ACK:CAM:STOP");
  }
}

void makeOneStepPulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(STEP_PIN, LOW);
}


// ==================================================
// Run Stepper Motor Non-Blocking
// ==================================================
//
// Continuous mode:
// stepperMode = 1
//
// Finite steps mode:
// stepperMode = 2
// stepsRemaining decreases every step
//
// Stop:
// stepperMode = 0
// ==================================================

void runCameraStepper() {
  if (stepperMode == 0) {
    return;
  }

  if (!stepperEnabled) {
    return;
  }

  if (cameraDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - lastStepTime >= stepIntervalMicros) {
    lastStepTime = now;

    makeOneStepPulse();

    if (stepperMode == 2) {
      stepsRemaining--;

      if (stepsRemaining <= 0) {
        stopStepper(false);
        Serial.println("ACK:CAM:STEPS:DONE");
      }
    }
  }
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

  // ------------------------------
  // Continuous stepper movement
  // ------------------------------

  if (command == "CAM:LEFT") {
    setStepperDirection(-1);
    startStepperContinuous();
    Serial.println("ACK:CAM:LEFT");
  }

  else if (command == "CAM:RIGHT") {
    setStepperDirection(1);
    startStepperContinuous();
    Serial.println("ACK:CAM:RIGHT");
  }

  else if (command == "CAM:START") {
    startStepperContinuous();
  }

  else if (command == "CAM:STOP") {
    stopStepper(true);
  }

  // ------------------------------
  // Enable / disable A4988
  // ------------------------------

  else if (command == "CAM:ENABLE") {
    enableStepper();
    Serial.println("ACK:CAM:ENABLE");
  }

  else if (command == "CAM:DISABLE") {
    disableStepper();
    Serial.println("ACK:CAM:DISABLE");
  }

  // ------------------------------
  // Direction only, without moving
  // ------------------------------

  else if (command == "CAM:DIR:LEFT") {
    setStepperDirection(-1);
    Serial.println("ACK:CAM:DIR:LEFT");
  }

  else if (command == "CAM:DIR:RIGHT") {
    setStepperDirection(1);
    Serial.println("ACK:CAM:DIR:RIGHT");
  }

  // ------------------------------
  // Move exact number of steps
  // Example:
  // CAM:DIR:LEFT
  // CAM:STEPS:200
  // ------------------------------

  else if (command.startsWith("CAM:STEPS:")) {
    long stepCount = command.substring(10).toInt();
    startStepperSteps(stepCount);
  }

  // ------------------------------
  // Speed
  // Example:
  // CAM:SPEED:1200
  //
  // Smaller = faster
  // Bigger  = slower
  // ------------------------------

  else if (command.startsWith("CAM:SPEED:")) {
    unsigned long value = command.substring(10).toInt();

    if (value < 300) {
      value = 300;
    }

    if (value > 10000) {
      value = 10000;
    }

    stepIntervalMicros = value;

    Serial.print("ACK:CAM:SPEED:");
    Serial.println(stepIntervalMicros);
  }

  // ------------------------------
  // Servo UP / DOWN
  // ------------------------------

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
    stopStepper(false);
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

  Serial.print(";MODE=");
  Serial.print(stepperMode);

  Serial.print(";EN=");
  Serial.print(stepperEnabled ? "1" : "0");

  Serial.print(";STEPS=");
  Serial.print(stepsRemaining);

  Serial.print(";SPEED=");
  Serial.println(stepIntervalMicros);
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
