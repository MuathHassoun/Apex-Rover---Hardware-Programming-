
@'
#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Arm Control
// ESP32 G4 TX -> UNO Pin 2 RX
//
// Stepper Driver: A4988 / DRV8825 / TB6600 style
// STEP = 7
// DIR  = 8
// EN   = 4
//
// Servos:
// Shoulder = 5
// Elbow    = 6
// Wrist    = 9
// Gripper  = 10
// Aux      = 11
// ==================================================

SoftwareSerial espSerial(2, 3); // RX = 2, TX = 3 unused

#define STEP_PIN 7
#define DIR_PIN 8
#define EN_PIN 4

#define SHOULDER_PIN 5
#define ELBOW_PIN 6
#define WRIST_PIN 9
#define GRIPPER_PIN 10
#define AUX_PIN 11

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;
Servo auxServo;

int shoulderPos = 90;
int elbowPos = 90;
int wristPos = 90;
int gripperPos = 90;
int auxPos = 90;

int baseDirection = 0; // -1 left, 0 stop, 1 right
unsigned long lastStepTime = 0;
const unsigned long stepIntervalMicros = 1200;

const int servoStep = 8;

const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Most A4988/DRV8825/TB6600 drivers: LOW = enabled
  digitalWrite(EN_PIN, LOW);

  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  gripperServo.attach(GRIPPER_PIN);
  auxServo.attach(AUX_PIN);

  goHome();

  Serial.println("Arduino UNO Arm Ready");
  Serial.println("Waiting for ARM commands from ESP32...");
}

void loop() {
  readEspCommands();
  runBaseStepper();
}

void readEspCommands() {
  if (espSerial.available() > 0) {
    String command = espSerial.readStringUntil('\n');
    command.trim();

    Serial.print("Received from ESP32: [");
    Serial.print(command);
    Serial.println("]");

    handleArmCommand(command);
  }
}

void handleArmCommand(String command) {
  if (!command.startsWith("ARM:")) {
    Serial.println("Ignored: not ARM command");
    return;
  }

  if (command == "ARM:BASE:LEFT") {
    baseDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    Serial.println("Base rotating LEFT");
  }

  else if (command == "ARM:BASE:RIGHT") {
    baseDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("Base rotating RIGHT");
  }

  else if (command == "ARM:BASE:STOP") {
    baseDirection = 0;
    Serial.println("Base STOP");
  }

  else if (command == "ARM:UP") {
    shoulderPos += servoStep;
    elbowPos += servoStep / 2;
    updateArmServos();
    Serial.println("Arm UP");
  }

  else if (command == "ARM:DOWN") {
    shoulderPos -= servoStep;
    elbowPos -= servoStep / 2;
    updateArmServos();
    Serial.println("Arm DOWN");
  }

  else if (command == "ARM:FORWARD") {
    elbowPos += servoStep;
    shoulderPos -= servoStep / 2;
    updateArmServos();
    Serial.println("Arm FORWARD");
  }

  else if (command == "ARM:BACK") {
    elbowPos -= servoStep;
    shoulderPos += servoStep / 2;
    updateArmServos();
    Serial.println("Arm BACK");
  }

  else if (command == "ARM:WRIST:UP") {
    wristPos += servoStep;
    updateArmServos();
    Serial.println("Wrist UP");
  }

  else if (command == "ARM:WRIST:DOWN") {
    wristPos -= servoStep;
    updateArmServos();
    Serial.println("Wrist DOWN");
  }

  else if (command == "ARM:GRIPPER:OPEN") {
    gripperPos = 35;
    updateArmServos();
    Serial.println("Gripper OPEN");
  }

  else if (command == "ARM:GRIPPER:CLOSE") {
    gripperPos = 110;
    updateArmServos();
    Serial.println("Gripper CLOSE");
  }

  else if (command == "ARM:HOME") {
    goHome();
    Serial.println("Arm HOME");
  }

  else if (command == "ARM:PICK") {
    pickPosition();
    Serial.println("Arm PICK position");
  }

  else if (command == "ARM:CARRY") {
    carryPosition();
    Serial.println("Arm CARRY position");
  }

  else if (command == "ARM:DROP") {
    dropPosition();
    Serial.println("Arm DROP BOX position");
  }

  else {
    Serial.println("Unknown ARM command");
  }
}

void runBaseStepper() {
  if (baseDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - lastStepTime >= stepIntervalMicros) {
    lastStepTime = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(STEP_PIN, LOW);
  }
}

void updateArmServos() {
  shoulderPos = constrain(shoulderPos, SERVO_MIN, SERVO_MAX);
  elbowPos = constrain(elbowPos, SERVO_MIN, SERVO_MAX);
  wristPos = constrain(wristPos, SERVO_MIN, SERVO_MAX);
  gripperPos = constrain(gripperPos, SERVO_MIN, SERVO_MAX);
  auxPos = constrain(auxPos, SERVO_MIN, SERVO_MAX);

  shoulderServo.write(shoulderPos);
  elbowServo.write(elbowPos);
  wristServo.write(wristPos);
  gripperServo.write(gripperPos);
  auxServo.write(auxPos);

  printPositions();
}

void goHome() {
  baseDirection = 0;

  shoulderPos = 90;
  elbowPos = 90;
  wristPos = 90;
  gripperPos = 90;
  auxPos = 90;

  updateArmServos();
}

void pickPosition() {
  baseDirection = 0;

  shoulderPos = 60;
  elbowPos = 120;
  wristPos = 85;
  gripperPos = 35; // open
  auxPos = 90;

  updateArmServos();
}

void carryPosition() {
  baseDirection = 0;

  shoulderPos = 110;
  elbowPos = 80;
  wristPos = 90;
  gripperPos = 110; // close
  auxPos = 90;

  updateArmServos();
}

void dropPosition() {
  baseDirection = 0;

  shoulderPos = 95;
  elbowPos = 105;
  wristPos = 100;
  gripperPos = 35; // open to drop
  auxPos = 90;

  updateArmServos();
}

void printPositions() {
  Serial.print("Shoulder: ");
  Serial.print(shoulderPos);

  Serial.print(" | Elbow: ");
  Serial.print(elbowPos);

  Serial.print(" | Wrist: ");
  Serial.print(wristPos);

  Serial.print(" | Gripper: ");
  Serial.print(gripperPos);

  Serial.print(" | Aux: ");
  Serial.println(auxPos);
}
'@ | Set-Content -Encoding UTF8 UNO\Uno_Arm_Control.ino
