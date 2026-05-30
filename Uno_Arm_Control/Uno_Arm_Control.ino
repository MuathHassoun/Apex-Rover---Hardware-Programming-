
#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand
// New WiFi architecture:
// Mobile    -> ESP32 -> UNO
// Raspberry -> ESP32 -> UNO
//
// UNO does not decide Manual/Auto source anymore.
// ESP32 is the central gate.
// UNO only executes CAM commands routed by ESP32.
// ==================================================

#define EN_PIN     A1
#define STEP_PIN   12
#define DIR_PIN    A2
#define SERVO_PIN  A0

// ESP32 GPIO4 TX -> UNO D2 RX
SoftwareSerial espSerial(2, 3); // RX=2, TX=3 unused

Servo cameraServo;

String systemMode = "MANUAL";

int servoAngle = 90;
int stepperDirection = 0;

unsigned long lastStepTime = 0;
unsigned long stepIntervalMicros = 700;
const unsigned int STEP_PULSE_MICROS = 3;

const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_STEP = 5;
const bool INVERT_SERVO_VERTICAL = true;

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

  Serial.println("UNO:CAMERA_STAND_READY");
  Serial.println("UNO:DEFAULT_MODE:MANUAL");
}

void loop() {
  readCommand();
  runStepper();
}

void readCommand() {
  String cmd = "";

  // USB Serial is only for testing now.
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n');
  }
  else if (espSerial.available() > 0) {
    cmd = espSerial.readStringUntil('\n');
  }

  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("RX:");
  Serial.println(cmd);

  handleCommand(cmd);
}

void handleCommand(String cmd) {
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

  if (cmd == "CAM:LEFT") {
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:LEFT");
  }

  else if (cmd == "CAM:RIGHT") {
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);
    Serial.println("ACK:CAM:RIGHT");
  }

  else if (cmd == "CAM:STOP") {
    stopCameraMotion();
    Serial.println("ACK:CAM:STOP");
  }

  else if (cmd == "CAM:UP") {
    if (INVERT_SERVO_VERTICAL) servoAngle -= SERVO_STEP;
    else servoAngle += SERVO_STEP;

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    Serial.print("ACK:CAM:UP ANGLE=");
    Serial.println(servoAngle);
  }

  else if (cmd == "CAM:DOWN") {
    if (INVERT_SERVO_VERTICAL) servoAngle += SERVO_STEP;
    else servoAngle -= SERVO_STEP;

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    Serial.print("ACK:CAM:DOWN ANGLE=");
    Serial.println(servoAngle);
  }

  else if (cmd == "CAM:CENTER") {
    stepperDirection = 0;
    digitalWrite(STEP_PIN, LOW);
    servoAngle = 90;
    cameraServo.write(servoAngle);
    Serial.println("ACK:CAM:CENTER");
  }

  else if (cmd == "CAM:STATUS" || cmd == "STATUS") {
    sendStatus();
  }

  else {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(cmd);
  }
}

void stopCameraMotion() {
  stepperDirection = 0;
  digitalWrite(STEP_PIN, LOW);
}

void runStepper() {
  if (stepperDirection == 0) return;

  unsigned long now = micros();
  if (now - lastStepTime >= stepIntervalMicros) {
    lastStepTime = now;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_MICROS);
    digitalWrite(STEP_PIN, LOW);
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
  Serial.println(digitalRead(EN_PIN) == LOW ? "1" : "0");
}
