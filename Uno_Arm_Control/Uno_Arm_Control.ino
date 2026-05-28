#include <Servo.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand SAFE COMMAND VERSION
//
// Based on the direct hardware test that worked successfully.
//
// Hardware:
// A4988 EN   -> A1
// A4988 STEP -> D12
// A4988 DIR  -> A2
// Servo      -> A0
//
// Commands:
// CAM:LEFT      stepper continuous left
// CAM:RIGHT     stepper continuous right
// CAM:STOP      stop stepper
// CAM:UP        servo up
// CAM:DOWN      servo down
// CAM:CENTER    servo center + stop stepper
// CAM:STATUS    print status
// ==================================================

#include <Servo.h>

#define EN_PIN     A1
#define STEP_PIN   12
#define DIR_PIN    A2
#define SERVO_PIN  A0

Servo cameraServo;

int servoAngle = 90;

// Stepper state
int stepperDirection = 0;
// -1 = left
//  0 = stop
//  1 = right

unsigned long lastStepTime = 0;

// This speed worked in your direct test
unsigned long stepIntervalMicros = 700;

const unsigned int STEP_PULSE_MICROS = 3;

// Servo limits
const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_STEP = 5;

// Your direct test:
// UP moved angle from 90 down to 30
// DOWN moved angle from 30 up to 150
const bool INVERT_SERVO_VERTICAL = true;


// ==================================================
// Setup
// ==================================================

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);       // Enable A4988
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);

  Serial.println("====================================");
  Serial.println("UNO Camera Stand SAFE Command Code");
  Serial.println("EN   = A1");
  Serial.println("STEP = D12");
  Serial.println("DIR  = A2");
  Serial.println("SERVO= A0");
  Serial.println("====================================");
  Serial.println("Commands:");
  Serial.println("CAM:LEFT");
  Serial.println("CAM:RIGHT");
  Serial.println("CAM:STOP");
  Serial.println("CAM:UP");
  Serial.println("CAM:DOWN");
  Serial.println("CAM:CENTER");
  Serial.println("CAM:STATUS");
}


// ==================================================
// Main loop
// ==================================================

void loop() {
  readCommand();
  runStepper();
}


// ==================================================
// Read command
// ==================================================

void readCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    Serial.print("RX:");
    Serial.println(cmd);

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
      stepperDirection = 0;
      digitalWrite(STEP_PIN, LOW);
      Serial.println("ACK:CAM:STOP");
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
      stepperDirection = 0;
      digitalWrite(STEP_PIN, LOW);

      servoAngle = 90;
      cameraServo.write(servoAngle);

      Serial.println("ACK:CAM:CENTER");
    }

    else if (cmd == "CAM:STATUS") {
      sendStatus();
    }

    else {
      Serial.print("ERROR:UNKNOWN_COMMAND:");
      Serial.println(cmd);
    }
  }
}


// ==================================================
// Run stepper continuously
// ==================================================

void runStepper() {
  if (stepperDirection == 0) {
    return;
  }

  unsigned long now = micros();

  if (now - lastStepTime >= stepIntervalMicros) {
    lastStepTime = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_MICROS);
    digitalWrite(STEP_PIN, LOW);
  }
}


// ==================================================
// Status
// ==================================================

void sendStatus() {
  Serial.print("DATA:CAMERA:");
  Serial.print("ANGLE=");
  Serial.print(servoAngle);

  Serial.print(";DIR=");
  Serial.print(stepperDirection);

  Serial.print(";EN=");
  Serial.println(digitalRead(EN_PIN) == LOW ? "1" : "0");
}