#include <Servo.h>

// ==================================================
// Arduino UNO - Raspberry Pi Camera Mount Control
//
// Raspberry Pi sends commands to Arduino UNO through USB Serial.
//
// System:
// Raspberry Pi  → USB Cable → Arduino UNO
// Arduino UNO   → A4988 Stepper Driver + Servo Motor
//
// Camera movement:
// Stepper Motor → rotate camera LEFT / RIGHT
// Servo Motor   → tilt camera UP / DOWN
// ==================================================


// ==================================================
// A4988 Stepper Driver Pins
// ==================================================
//
// A4988 EN   → Arduino UNO Pin 4
// A4988 STEP → Arduino UNO Pin 7
// A4988 DIR  → Arduino UNO Pin 8
// A4988 GND  → Arduino UNO GND
//
// Important for A4988:
// RESET and SLEEP should be connected together,
// then connected to 5V.
// VMOT should be connected to motor power supply.
// VMOT GND should be common with Arduino GND.
// ==================================================

#define STEP_PIN 7
#define DIR_PIN 8
#define EN_PIN 4


// ==================================================
// Camera Servo Pin
// ==================================================
//
// Servo Signal → Arduino UNO Pin 9
// Servo VCC    → External 5V / 6V supply
// Servo GND    → External power GND
// UNO GND      → External power GND
// ==================================================

#define CAMERA_SERVO_PIN 9


// ==================================================
// Servo Settings
// ==================================================

Servo cameraServo;

int cameraAngle = 90;       // Start angle: center position

const int SERVO_MIN = 30;   // Minimum safe angle
const int SERVO_MAX = 150;  // Maximum safe angle
const int SERVO_STEP = 5;   // Angle change per command


// ==================================================
// Stepper Settings
// ==================================================
//
// cameraDirection:
// -1 = rotate left
//  0 = stop
//  1 = rotate right
// ==================================================

int cameraDirection = 0;

unsigned long lastStepTime = 0;

// Smaller value = faster stepper
// Larger value = slower stepper
const unsigned long stepIntervalMicros = 1200;


// ==================================================
// SETUP
// ==================================================

void setup() {
  // USB Serial with Raspberry Pi
  Serial.begin(9600);

  // Stepper driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // A4988 enable:
  // LOW  = driver enabled
  // HIGH = driver disabled
  digitalWrite(EN_PIN, LOW);

  // Servo setup
  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(cameraAngle);

  Serial.println("UNO Camera Mount Ready");
  Serial.println("Waiting for Raspberry Pi commands...");
  Serial.println("Available commands:");
  Serial.println("CAM:LEFT");
  Serial.println("CAM:RIGHT");
  Serial.println("CAM:STOP");
  Serial.println("CAM:UP");
  Serial.println("CAM:DOWN");
  Serial.println("CAM:CENTER");
}


// ==================================================
// LOOP
// ==================================================

void loop() {
  readRaspberryCommands();
  runCameraStepper();
}


// ==================================================
// Read Commands From Raspberry Pi
// ==================================================

void readRaspberryCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("Received from Raspberry Pi: [");
    Serial.print(command);
    Serial.println("]");

    handleCameraCommand(command);
  }
}


// ==================================================
// Handle Camera Commands
// ==================================================

void handleCameraCommand(String command) {

  // Rotate camera left using stepper motor
  if (command == "CAM:LEFT") {
    cameraDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    Serial.println("Camera rotating LEFT");
  }

  // Rotate camera right using stepper motor
  else if (command == "CAM:RIGHT") {
    cameraDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("Camera rotating RIGHT");
  }

  // Stop stepper rotation
  else if (command == "CAM:STOP") {
    cameraDirection = 0;
    Serial.println("Camera rotation STOP");
  }

  // Tilt camera up using servo
  else if (command == "CAM:UP") {
    cameraAngle += SERVO_STEP;
    updateCameraServo();
    Serial.println("Camera tilt UP");
  }

  // Tilt camera down using servo
  else if (command == "CAM:DOWN") {
    cameraAngle -= SERVO_STEP;
    updateCameraServo();
    Serial.println("Camera tilt DOWN");
  }

  // Center camera and stop stepper
  else if (command == "CAM:CENTER") {
    cameraDirection = 0;
    cameraAngle = 90;
    updateCameraServo();
    Serial.println("Camera CENTER");
  }

  else {
    Serial.println("Unknown camera command");
  }
}


// ==================================================
// Run Stepper Motor Continuously
// ==================================================
//
// When cameraDirection is LEFT or RIGHT,
// this function keeps sending STEP pulses.
// When CAM:STOP is received, cameraDirection becomes 0.
// ==================================================

void runCameraStepper() {
  if (cameraDirection == 0) {
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


// ==================================================
// Update Servo Angle
// ==================================================

void updateCameraServo() {
  cameraAngle = constrain(cameraAngle, SERVO_MIN, SERVO_MAX);
  cameraServo.write(cameraAngle);

  Serial.print("Camera Servo Angle: ");
  Serial.println(cameraAngle);
}