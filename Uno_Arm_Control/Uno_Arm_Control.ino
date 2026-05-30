


#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand SAFE COMMAND VERSION
//
// Supports two control modes:
// MANUAL  = accept CAM commands from ESP32 / Mobile App
// AUTO    = accept CAM commands from Raspberry Pi through USB Serial
//
// Default mode: MANUAL
//
// Hardware:
// A4988 EN   -> A1
// A4988 STEP -> D12
// A4988 DIR  -> A2
// Servo      -> A0
// ESP32 TX   -> D2  (SoftwareSerial RX)
// Raspberry  -> UNO USB Serial
// ==================================================

// ==================================================
// Camera commands:
// CAM:LEFT         stepper continuous left
// CAM:RIGHT        stepper continuous right
// CAM:STOP         stop stepper
// CAM:UP           servo up
// CAM:DOWN         servo down
// CAM:CENTER       servo center + stop stepper + return to zero position
// CAM:STATUS       print status
// CAM:DIR:LEFT     set stepper direction left (for step-based moves)
// CAM:DIR:RIGHT    set stepper direction right
// CAM:STEPS:N      move N steps in current direction then stop
// CAM:SPEED:N      set step interval in microseconds
// CAM:ENABLE       enable stepper driver
// CAM:DISABLE      disable stepper driver
// CAM:HOME         save current position as zero (home)
// CAM:GOTO:HOME    return to saved home/zero position
//
// System mode commands:
// SYS:MODE:MANUAL
// SYS:MODE:AUTO
// ==================================================

#define EN_PIN     A1
#define STEP_PIN   12
#define DIR_PIN    A2
#define SERVO_PIN  A0

// ESP32 serial: RX=2, TX=3 (TX not used but SoftwareSerial requires it)
SoftwareSerial espSerial(2, 3);

Servo cameraServo;

String systemMode = "MANUAL";  // Default mode

int servoAngle = 90;
int stepperDirection = 0;

unsigned long lastStepTime = 0;
unsigned long stepIntervalMicros = 700;
const unsigned int STEP_PULSE_MICROS = 3;

const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_STEP = 5;
const bool INVERT_SERVO_VERTICAL = true;

// ==================================================
// Stepper position tracking
// stepPosition  = current absolute position (in steps from power-on)
// homePosition  = the saved zero / reference point
// stepsToGo     = steps remaining in a CAM:STEPS:N command
// returnToHome  = true while executing CAM:GOTO:HOME
// ==================================================
long stepPosition   = 0;
long homePosition   = 0;
long stepsToGo      = 0;
bool returnToHome   = false;


// ==================================================
// Setup
// ==================================================

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  espSerial.begin(9600);
  espSerial.setTimeout(50);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);

  Serial.println("====================================");
  Serial.println("UNO Camera Stand SAFE Command Code");
  Serial.println("Default System Mode: MANUAL");
  Serial.println("MANUAL: commands from ESP32/Mobile");
  Serial.println("AUTO: commands from Raspberry Pi USB");
  Serial.println("EN   = A1");
  Serial.println("STEP = D12");
  Serial.println("DIR  = A2");
  Serial.println("SERVO= A0");
  Serial.println("ESP32= D2 (SoftwareSerial RX)");
  Serial.println("Stepper position tracking enabled.");
  Serial.println("====================================");
}


// ==================================================
// Main loop
// ==================================================

void loop() {
  readCommands();
  runStepper();
}


// ==================================================
// Read command from BOTH Raspberry USB Serial and ESP32 SoftwareSerial
// ==================================================

void readCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      handleCommand(cmd, "RASPBERRY_PI");
    }
  }

  if (espSerial.available() > 0) {
    String cmd = espSerial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      handleCommand(cmd, "ESP32");
    }
  }
}


// ==================================================
// Command handler with MANUAL / AUTO gate
// ==================================================

void handleCommand(String cmd, String sourceName) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("RX:");
  Serial.print(sourceName);
  Serial.print(":");
  Serial.println(cmd);

  // --------------------------
  // System mode commands
  // --------------------------

  if (cmd == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    stopCameraMotion();
    Serial.println("ACK:SYS:MODE:MANUAL");
    return;
  }

  if (cmd == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    stopCameraMotion();
    Serial.println("ACK:SYS:MODE:AUTO");
    return;
  }

  // --------------------------
  // Always allowed safety/status commands
  // --------------------------

  if (cmd == "CAM:STOP") {
    stepperDirection = 0;
    stepsToGo = 0;
    returnToHome = false;
    digitalWrite(STEP_PIN, LOW);
    Serial.println("ACK:CAM:STOP");
    return;
  }

  if (cmd == "CAM:STATUS") {
    sendStatus();
    return;
  }

  // --------------------------
  // Source gate
  // --------------------------
  //
  // MANUAL: only ESP32/Mobile CAM commands are accepted.
  // AUTO: only Raspberry Pi CAM commands are accepted.
  // --------------------------

  if (systemMode == "MANUAL" && sourceName != "ESP32") {
    Serial.print("IGNORED:");
    Serial.print(cmd);
    Serial.println(":SYSTEM_MODE_MANUAL_ONLY_ESP32_ALLOWED");
    return;
  }

  if (systemMode == "AUTO" && sourceName != "RASPBERRY_PI") {
    Serial.print("IGNORED:");
    Serial.print(cmd);
    Serial.println(":SYSTEM_MODE_AUTO_ONLY_RASPBERRY_ALLOWED");
    return;
  }

  // --------------------------
  // Camera movement commands
  // --------------------------

  if (cmd == "CAM:LEFT") {
    returnToHome = false;
    stepsToGo = 0;
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:LEFT");
  }

  else if (cmd == "CAM:RIGHT") {
    returnToHome = false;
    stepsToGo = 0;
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:RIGHT");
  }

  else if (cmd == "CAM:UP") {
    if (INVERT_SERVO_VERTICAL) {
      servoAngle -= SERVO_STEP;
    } else {
      servoAngle += SERVO_STEP;
    }

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    Serial.print("ACK:CAM:UP ANGLE=");
    Serial.println(servoAngle);
  }

  else if (cmd == "CAM:DOWN") {
    if (INVERT_SERVO_VERTICAL) {
      servoAngle += SERVO_STEP;
    } else {
      servoAngle -= SERVO_STEP;
    }

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    Serial.print("ACK:CAM:DOWN ANGLE=");
    Serial.println(servoAngle);
  }

  else if (cmd == "CAM:CENTER") {
    // Stop stepper, center servo, and return stepper to home/zero position.
    servoAngle = 90;
    cameraServo.write(servoAngle);
    goToHome();
    Serial.println("ACK:CAM:CENTER");
  }

  else if (cmd == "CAM:ENABLE") {
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:ENABLE");
  }

  else if (cmd == "CAM:DISABLE") {
    stepperDirection = 0;
    stepsToGo = 0;
    returnToHome = false;
    digitalWrite(EN_PIN, HIGH);
    Serial.println("ACK:CAM:DISABLE");
  }

  else if (cmd == "CAM:DIR:LEFT") {
    returnToHome = false;
    stepperDirection = 0;
    digitalWrite(DIR_PIN, LOW);
    Serial.println("ACK:CAM:DIR:LEFT");
  }

  else if (cmd == "CAM:DIR:RIGHT") {
    returnToHome = false;
    stepperDirection = 0;
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("ACK:CAM:DIR:RIGHT");
  }

  else if (cmd.startsWith("CAM:STEPS:")) {
    // Move N steps in the currently set direction, then stop automatically.
    String numStr = cmd.substring(10);
    numStr.trim();
    long n = numStr.toInt();
    if (n > 0) {
      returnToHome = false;
      stepsToGo = n;
      // stepperDirection must already be set by a prior CAM:DIR command.
      // If it is 0 (no direction set), default to RIGHT.
      if (stepperDirection == 0) {
        stepperDirection = 1;
        digitalWrite(DIR_PIN, HIGH);
      }
      digitalWrite(EN_PIN, LOW);
      Serial.print("ACK:CAM:STEPS:");
      Serial.println(n);
    } else {
      Serial.println("ERROR:CAM:STEPS:INVALID_NUMBER");
    }
  }

  else if (cmd.startsWith("CAM:SPEED:")) {
    String numStr = cmd.substring(10);
    numStr.trim();
    long v = numStr.toInt();
    if (v >= 300 && v <= 10000) {
      stepIntervalMicros = (unsigned long)v;
      Serial.print("ACK:CAM:SPEED:");
      Serial.println(v);
    } else {
      Serial.println("ERROR:CAM:SPEED:OUT_OF_RANGE_300_10000");
    }
  }

  // CAM:HOME  -- save current position as the zero reference
  else if (cmd == "CAM:HOME") {
    homePosition = stepPosition;
    Serial.print("ACK:CAM:HOME SAVED_AT=");
    Serial.println(homePosition);
  }

  // CAM:GOTO:HOME  -- return stepper to saved zero position
  else if (cmd == "CAM:GOTO:HOME") {
    goToHome();
    Serial.println("ACK:CAM:GOTO:HOME");
  }

  else {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(cmd);
  }
}


// ==================================================
// Helpers
// ==================================================

void stopCameraMotion() {
  stepperDirection = 0;
  stepsToGo = 0;
  returnToHome = false;
  digitalWrite(STEP_PIN, LOW);
}

// Start a move back to homePosition.
void goToHome() {
  long diff = stepPosition - homePosition;
  if (diff == 0) {
    stepperDirection = 0;
    stepsToGo = 0;
    returnToHome = false;
    Serial.println("INFO:ALREADY_AT_HOME");
    return;
  }

  returnToHome = true;
  stepsToGo = abs(diff);

  if (diff > 0) {
    // Need to move left (negative direction) to get back to home
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
  } else {
    // Need to move right (positive direction)
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
  }

  digitalWrite(EN_PIN, LOW);
}

void runStepper() {
  if (stepperDirection == 0) return;

  unsigned long now = micros();

  if (now - lastStepTime >= stepIntervalMicros) {
    lastStepTime = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_MICROS);
    digitalWrite(STEP_PIN, LOW);

    // Track absolute position
    stepPosition += stepperDirection;

    // If running a counted move (CAM:STEPS or goToHome), decrement counter
    if (stepsToGo > 0) {
      stepsToGo--;
      if (stepsToGo == 0) {
        stepperDirection = 0;
        returnToHome = false;
        Serial.println("INFO:STEPS_DONE");
      }
    }
  }
}

void sendStatus() {
  Serial.print("DATA:CAMERA:");
  Serial.print("MODE=");
  Serial.print(systemMode);
  Serial.print(";ANGLE=");
  Serial.print(servoAngle);
  Serial.print(";DIR=");
  Serial.print(stepperDirection);
  Serial.print(";EN=");
  Serial.print(digitalRead(EN_PIN) == LOW ? "1" : "0");
  Serial.print(";POS=");
  Serial.print(stepPosition);
  Serial.print(";HOME=");
  Serial.print(homePosition);
  Serial.print(";STEPS_LEFT=");
  Serial.println(stepsToGo);
}
