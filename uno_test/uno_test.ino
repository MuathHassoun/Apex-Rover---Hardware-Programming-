#include <Servo.h>

#define EN_PIN     A1
#define STEP_PIN   12
#define DIR_PIN    A2
#define SERVO_PIN  A0

Servo cameraServo;

int servoAngle = 90;

void setup() {
  Serial.begin(9600);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);   // Enable A4988
  digitalWrite(STEP_PIN, LOW);

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);

  Serial.println("Direct Motor Test Started");
}

void loop() {

  // =========================================
  // TEST 1: Stepper rotate RIGHT
  // =========================================
  Serial.println("Stepper RIGHT");

  digitalWrite(DIR_PIN, HIGH);

  for (int i = 0; i < 1000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(700);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(700);
  }

  delay(1000);

  // =========================================
  // TEST 2: Stepper rotate LEFT
  // =========================================
  Serial.println("Stepper LEFT");

  digitalWrite(DIR_PIN, LOW);

  for (int i = 0; i < 1000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(700);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(700);
  }

  delay(1000);

  // =========================================
  // TEST 3: Servo UP
  // =========================================
  Serial.println("Servo UP");

  for (servoAngle = 90; servoAngle >= 30; servoAngle -= 2) {
    cameraServo.write(servoAngle);
    delay(20);
  }

  delay(1000);

  // =========================================
  // TEST 4: Servo DOWN
  // =========================================
  Serial.println("Servo DOWN");

  for (servoAngle = 30; servoAngle <= 150; servoAngle += 2) {
    cameraServo.write(servoAngle);
    delay(20);
  }

  delay(1000);

  // =========================================
  // TEST 5: Servo CENTER
  // =========================================
  Serial.println("Servo CENTER");

  cameraServo.write(90);

  delay(2000);
}