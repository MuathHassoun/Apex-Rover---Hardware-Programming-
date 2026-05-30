#include <Servo.h>
#include <SoftwareSerial.h>

// ==================================================
// Arduino UNO - Apex Rover Camera Stand
//
// This device RECEIVES commands only — it does NOT
// send anything back to the Raspberry Pi.
//
// Command path:
//   Mobile App -> WebSocket -> ESP32 -> UNO D2
//
// Hardware:
//   ESP32 GPIO4 TX -> UNO D2 SoftwareSerial RX
//   A4988 EN       -> UNO A1
//   A4988 STEP     -> UNO D12
//   A4988 DIR      -> UNO A2
//   Servo Signal   -> UNO A0
//   Common GND between ESP32, UNO, A4988, servo power
//
// Supported commands:
//   CAM:LEFT            continuous stepper left
//   CAM:RIGHT           continuous stepper right
//   CAM:STOP            stop stepper
//   CAM:STEP_LEFT:N     move N steps left then stop
//   CAM:STEP_RIGHT:N    move N steps right then stop
//   CAM:UP              tilt servo up
//   CAM:DOWN            tilt servo down
//   CAM:CENTER          center servo and stop stepper
//   CAM:ZERO            reset step position counter to 0
//   CAM:SPEED:N         set step interval in microseconds (300-5000)
//   SYS:MODE:MANUAL     set system mode (stored locally)
//   SYS:MODE:AUTO       set system mode (stored locally)
// ==================================================


#define EN_PIN    A1
#define STEP_PIN  12
#define DIR_PIN   A2
#define SERVO_PIN A0

// ESP32 GPIO4 TX -> UNO D2 RX (D3 TX not wired)
SoftwareSerial espSerial(2, 3);

Servo cameraServo;

String systemMode = "MANUAL";

int servoAngle = 90;

const int  SERVO_MIN              = 30;
const int  SERVO_MAX              = 150;
const int  SERVO_STEP             = 5;
const bool INVERT_SERVO_VERTICAL  = true;

// -1 = left, 0 = stop, 1 = right
int  stepperDirection     = 0;
long finiteStepsRemaining = 0;
long stepPosition         = 0;

unsigned long lastStepTime       = 0;
unsigned long stepIntervalMicros = 700;
const unsigned int STEP_PULSE_MICROS = 3;


// ==================================================
// SETUP
// ==================================================
void setup() {
  // USB Serial is available for debug monitor only.
  // The UNO does NOT send data to the Raspberry Pi.
  Serial.begin(9600);
  Serial.setTimeout(50);

  espSerial.begin(9600);
  espSerial.setTimeout(50);

  pinMode(EN_PIN,   OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  digitalWrite(EN_PIN,   LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);

  Serial.println("UNO_CAMERA:READY");
  Serial.println("UNO_CAMERA:RECEIVE_ONLY_MODE");
}


// ==================================================
// LOOP
// ==================================================
void loop() {
  readCommand();
  runStepper();
}


// ==================================================
// READ COMMAND FROM ESP32 ONLY
// ==================================================
void readCommand() {
  if (espSerial.available() == 0) return;

  String cmd = espSerial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  handleCommand(cmd);
}


// ==================================================
// HELPERS
// ==================================================
long parseLastNumber(String cmd) {
  int lastColon = cmd.lastIndexOf(':');
  if (lastColon < 0 || lastColon >= (int)cmd.length() - 1) return 0;
  long value = cmd.substring(lastColon + 1).toInt();
  if (value < 0) value = -value;
  return constrain(value, 1, 50000);
}

void stopCameraMotion() {
  stepperDirection     = 0;
  finiteStepsRemaining = 0;
  digitalWrite(STEP_PIN, LOW);
}


// ==================================================
// COMMAND HANDLER
// No ACK is sent back — receive-only design.
// ==================================================
void handleCommand(String cmd) {

  // ---- System mode ----
  if (cmd == "SYS:MODE:MANUAL") { systemMode = "MANUAL"; stopCameraMotion(); return; }
  if (cmd == "SYS:MODE:AUTO")   { systemMode = "AUTO";   stopCameraMotion(); return; }

  // ---- Stop / Center ----
  if (cmd == "CAM:STOP")   { stopCameraMotion(); return; }
  if (cmd == "CAM:CENTER") { stopCameraMotion(); servoAngle = 90; cameraServo.write(servoAngle); return; }

  // ---- Continuous stepper ----
  if (cmd == "CAM:LEFT") {
    finiteStepsRemaining = 0;
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN,  LOW);
    return;
  }
  if (cmd == "CAM:RIGHT") {
    finiteStepsRemaining = 0;
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN,  LOW);
    return;
  }

  // ---- Finite steps ----
  if (cmd.startsWith("CAM:STEP_LEFT:")) {
    finiteStepsRemaining = parseLastNumber(cmd);
    stepperDirection = -1;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN,  LOW);
    return;
  }
  if (cmd.startsWith("CAM:STEP_RIGHT:")) {
    finiteStepsRemaining = parseLastNumber(cmd);
    stepperDirection = 1;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN,  LOW);
    return;
  }

  // ---- Servo tilt ----
  if (cmd == "CAM:UP") {
    servoAngle += INVERT_SERVO_VERTICAL ? -SERVO_STEP : SERVO_STEP;
    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);
    return;
  }
  if (cmd == "CAM:DOWN") {
    servoAngle += INVERT_SERVO_VERTICAL ? SERVO_STEP : -SERVO_STEP;
    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    cameraServo.write(servoAngle);
    return;
  }

  // ---- Stepper speed ----
  if (cmd.startsWith("CAM:SPEED:")) {
    long interval = parseLastNumber(cmd);
    stepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }

  // ---- Reset position counter ----
  if (cmd == "CAM:ZERO") { stepPosition = 0; return; }
}


// ==================================================
// STEPPER RUNNER (non-blocking)
// ==================================================
void runStepper() {
  if (stepperDirection == 0) return;

  unsigned long now = micros();
  if (now - lastStepTime < stepIntervalMicros) return;
  lastStepTime = now;

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_MICROS);
  digitalWrite(STEP_PIN, LOW);

  stepPosition += stepperDirection;

  if (finiteStepsRemaining > 0) {
    finiteStepsRemaining--;
    if (finiteStepsRemaining == 0) stopCameraMotion();
  }
}
