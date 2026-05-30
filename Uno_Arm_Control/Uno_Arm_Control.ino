#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand Direct Status To Pi
//
// Request path:
//   Raspberry Pi -> WiFi -> ESP32 -> UNO D2
//
// Response path:
//   UNO USB Serial -> Raspberry Pi
//
// Hardware:
//   ESP32 GPIO4 TX -> UNO D2 / Pin 2 SoftwareSerial RX
//   A4988 EN       -> UNO A1
//   A4988 STEP     -> UNO D12
//   A4988 DIR      -> UNO A2
//   Servo Signal   -> UNO A0
//   Common GND between ESP32, UNO, A4988, servo power
//
// Commands:
//   CAM:LEFT
//   CAM:RIGHT
//   CAM:STOP
//   CAM:STEP_LEFT:200
//   CAM:STEP_RIGHT:200
//   CAM:UP
//   CAM:DOWN
//   CAM:CENTER
//   CAM:STATUS
//   GET:CAMERA
//   CAM:SPEED:700
// ==================================================


#define EN_PIN     A1
#define STEP_PIN   12
#define DIR_PIN    A2
#define SERVO_PIN  A0

SoftwareSerial espSerial(2, 3); // RX=D2 from ESP32, TX=D3 unused

Servo cameraServo;

String systemMode = "MANUAL";

int servoAngle = 90;

const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_STEP = 5;
const bool INVERT_SERVO_VERTICAL = true;

// -1 = left, 0 = stop, 1 = right
int stepperDirection = 0;

long finiteStepsRemaining = 0;

// Relative stepper position from startup.
// Important: this is not encoder feedback.
// If power resets or motor slips, it resets / becomes inaccurate.
long stepPosition = 0;

unsigned long lastStepTime = 0;
unsigned long stepIntervalMicros = 700;
const unsigned int STEP_PULSE_MICROS = 3;

// Keep false to avoid polluting Raspberry USB serial.
bool debugUSB = false;


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

  // Minimal startup lines for Raspberry identification.
  Serial.println("UNO_CAMERA:READY");
  Serial.println("UNO_CAMERA:DIRECT_STATUS_TO_PI_ENABLED");
}


void loop() {
  readCommand();
  runStepper();
}


void readCommand() {
  String cmd = "";
  String source = "";

  // USB Serial from Raspberry or Serial Monitor.
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n');
    source = "USB";
  }

  // ESP32 command from SoftwareSerial D2.
  else if (espSerial.available() > 0) {
    cmd = espSerial.readStringUntil('\n');
    source = "ESP32";
  }

  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  handleCommand(cmd, source);
}


long parseLastNumber(String cmd) {
  int lastColon = cmd.lastIndexOf(':');

  if (lastColon < 0 || lastColon >= cmd.length() - 1) {
    return 0;
  }

  long value = cmd.substring(lastColon + 1).toInt();

  if (value < 0) {
    value = -value;
  }

  return constrain(value, 1, 50000);
}


void handleCommand(String cmd, String source) {
  // =========================
  // SYSTEM MODE
  // =========================

  if (cmd == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    stopCameraMotion();

    if (source == "ESP32") {
      espSerial.println("ACK:SYS:MODE:MANUAL");
    } else {
      Serial.println("ACK:SYS:MODE:MANUAL");
    }
    return;
  }

  if (cmd == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    stopCameraMotion();

    if (source == "ESP32") {
      espSerial.println("ACK:SYS:MODE:AUTO");
    } else {
      Serial.println("ACK:SYS:MODE:AUTO");
    }
    return;
  }

  // =========================
  // STATUS REQUEST
  // =========================
  //
  // If ESP32 asks CAM:STATUS for Raspberry,
  // response goes to USB Serial, not back to ESP32.
  // =========================

  if (cmd == "CAM:STATUS" || cmd == "GET:CAMERA" || cmd == "STATUS") {
    sendStatusToRaspberryUSB();

    if (source == "ESP32") {
      espSerial.println("ACK:CAM:STATUS:SENT_TO_PI_USB");
    }
    return;
  }

  // =========================
  // CONTINUOUS STEPPER
  // =========================

  if (cmd == "CAM:LEFT") {
    finiteStepsRemaining = 0;
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, LOW);

    ack(source, "ACK:CAM:LEFT");
    return;
  }

  if (cmd == "CAM:RIGHT") {
    finiteStepsRemaining = 0;
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);

    ack(source, "ACK:CAM:RIGHT");
    return;
  }

  // =========================
  // FINITE STEPS
  // =========================

  if (cmd.startsWith("CAM:STEP_LEFT:")) {
    finiteStepsRemaining = parseLastNumber(cmd);
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, LOW);

    ack(source, "ACK:CAM:STEP_LEFT");
    return;
  }

  if (cmd.startsWith("CAM:STEP_RIGHT:")) {
    finiteStepsRemaining = parseLastNumber(cmd);
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);

    ack(source, "ACK:CAM:STEP_RIGHT");
    return;
  }

  // =========================
  // STOP / CENTER
  // =========================

  if (cmd == "CAM:STOP") {
    stopCameraMotion();

    ack(source, "ACK:CAM:STOP");
    return;
  }

  if (cmd == "CAM:CENTER") {
    stopCameraMotion();
    servoAngle = 90;
    cameraServo.write(servoAngle);

    ack(source, "ACK:CAM:CENTER");
    return;
  }

  // =========================
  // SERVO TILT
  // =========================

  if (cmd == "CAM:UP") {
    if (INVERT_SERVO_VERTICAL) {
      servoAngle -= SERVO_STEP;
    } else {
      servoAngle += SERVO_STEP;
    }

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    ack(source, "ACK:CAM:UP");
    return;
  }

  if (cmd == "CAM:DOWN") {
    if (INVERT_SERVO_VERTICAL) {
      servoAngle += SERVO_STEP;
    } else {
      servoAngle -= SERVO_STEP;
    }

    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);

    ack(source, "ACK:CAM:DOWN");
    return;
  }

  // =========================
  // STEPPER SPEED
  // =========================

  if (cmd.startsWith("CAM:SPEED:")) {
    long interval = parseLastNumber(cmd);
    stepIntervalMicros = constrain(interval, 300, 5000);

    ack(source, "ACK:CAM:SPEED");
    return;
  }

  // =========================
  // RESET POSITION COUNTER
  // =========================

  if (cmd == "CAM:ZERO") {
    stepPosition = 0;
    ack(source, "ACK:CAM:ZERO");
    return;
  }

  if (debugUSB) {
    Serial.print("ERROR:UNKNOWN_COMMAND:");
    Serial.println(cmd);
  }

  if (source == "ESP32") {
    espSerial.println("ERROR:UNKNOWN_COMMAND");
  }
}


void ack(String source, String message) {
  if (source == "ESP32") {
    espSerial.println(message);
  } else {
    Serial.println(message);
  }
}


void stopCameraMotion() {
  stepperDirection = 0;
  finiteStepsRemaining = 0;
  digitalWrite(STEP_PIN, LOW);
}


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

    if (stepperDirection > 0) {
      stepPosition++;
    } else {
      stepPosition--;
    }

    if (finiteStepsRemaining > 0) {
      finiteStepsRemaining--;

      if (finiteStepsRemaining <= 0) {
        stopCameraMotion();

        if (debugUSB) {
          Serial.print("ACK:CAM:STEP_DONE POS=");
          Serial.println(stepPosition);
        }
      }
    }
  }
}


void sendStatusToRaspberryUSB() {
  Serial.print("DATA:CAMERA:");
  Serial.print("SYSTEM=");
  Serial.print(systemMode);

  Serial.print(";ANGLE=");
  Serial.print(servoAngle);

  Serial.print(";DIR=");
  Serial.print(stepperDirection);

  Serial.print(";POS=");
  Serial.print(stepPosition);

  Serial.print(";REMAIN=");
  Serial.print(finiteStepsRemaining);

  Serial.print(";INTERVAL_US=");
  Serial.println(stepIntervalMicros);
}