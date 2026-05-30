#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Motors + Jacks + Sensors
//
// Communication:
//   ESP32  GPIO17 TX  ->  Mega RX1 pin 19  (receives commands)
//   Mega   USB Serial ->  Raspberry Pi     (sends sensor data)
//
// The Mega does NOT receive commands from the Raspberry Pi anymore.
// All commands come from the ESP32 (which gets them from the mobile app).
//
// Sensor reporting (automatic, no request needed):
//   - Every 10 seconds: MPU + Ultrasonic data sent to Raspberry Pi USB
//   - Immediately on critical values (tilt danger, obstacle close):
//     interrupt-style push to Raspberry Pi USB
//
// Data format sent to Raspberry Pi:
//   SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
//   SENSOR:PITCH=32.0;ROLL=5.10;UF=3.20;UR=9.20;ALERT=TILT_DANGER
//   SENSOR:PITCH=2.30;ROLL=-1.10;UF=2.10;UR=9.20;ALERT=OBSTACLE_FRONT
// ==================================================


// ==================================================
// MOTOR DRIVER PINS (BTS7960)
// ==================================================
#define RIGHT_RPWM  9
#define RIGHT_LPWM 11
#define LEFT_RPWM   5
#define LEFT_LPWM   7


// ==================================================
// ULTRASONIC PINS
//   Front: TRIG=A8  ECHO=A9
//   Rear:  TRIG=A11 ECHO=A12
// ==================================================
#define FRONT_US_TRIG A8
#define FRONT_US_ECHO A9
#define REAR_US_TRIG  A11
#define REAR_US_ECHO  A12


// ==================================================
// MPU6500 (I2C: SDA=Mega20, SCL=Mega21)
// ==================================================
#define MPU_ADDR 0x68

float accelX = 0, accelY = 0, accelZ = 0;
float pitch   = 0, roll   = 0;


// ==================================================
// JACK PINS (L298N)
//   Rear:  IN1=A0  IN2=A1
//   Front: IN3=A3  IN4=A4
// ==================================================
#define REAR_JACK_IN1  A0
#define REAR_JACK_IN2  A1
#define FRONT_JACK_IN3 A3
#define FRONT_JACK_IN4 A4


// ==================================================
// ROBOT STATE
// ==================================================
int    motorSpeed        = 150;
int    motorSpeedPercent = 60;
String currentMode       = "NORMAL";
String systemMode        = "MANUAL";
String lastMovement      = "STOP";

float frontUltrasonicCM = -1.0;
float rearUltrasonicCM  = -1.0;


// ==================================================
// SENSOR REPORTING TIMERS
//
// Periodic: send sensor data every SENSOR_REPORT_INTERVAL ms.
// Critical: send immediately when thresholds are crossed,
//           but no more than once every ALERT_COOLDOWN_MS.
// ==================================================
const unsigned long SENSOR_REPORT_INTERVAL = 10000; // 10 seconds
const unsigned long ALERT_COOLDOWN_MS      = 2000;  // min gap between alerts

unsigned long lastPeriodicReport = 0;
unsigned long lastAlertSent      = 0;

// Critical thresholds
const float PITCH_DANGER_DEG     = 30.0;
const float ROLL_DANGER_DEG      = 25.0;
const float OBSTACLE_FRONT_CM    = 10.0;
const float OBSTACLE_REAR_CM     = 8.0;


// ==================================================
// SETUP
// ==================================================
void setup() {
  // USB Serial -> Raspberry Pi (read only by Pi)
  Serial.begin(9600);
  Serial.setTimeout(10);

  // Serial1 <- ESP32 (commands from mobile app via ESP32)
  Serial1.begin(9600);
  Serial1.setTimeout(10);

  pinMode(RIGHT_RPWM, OUTPUT);  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM,  OUTPUT);  pinMode(LEFT_LPWM,  OUTPUT);

  pinMode(FRONT_US_TRIG, OUTPUT); pinMode(FRONT_US_ECHO, INPUT);
  pinMode(REAR_US_TRIG,  OUTPUT); pinMode(REAR_US_ECHO,  INPUT);
  digitalWrite(FRONT_US_TRIG, LOW);
  digitalWrite(REAR_US_TRIG,  LOW);

  pinMode(REAR_JACK_IN1,  OUTPUT); pinMode(REAR_JACK_IN2,  OUTPUT);
  pinMode(FRONT_JACK_IN3, OUTPUT); pinMode(FRONT_JACK_IN4, OUTPUT);

  stopMotors();
  stopAllJacks();

  Wire.begin();
  initMPU();
  delay(500);

  // Notify Raspberry Pi that Mega is ready.
  Serial.println("MEGA:READY");
  Serial.println("MEGA:SENSOR_PUSH_TO_PI_ENABLED");
  Serial.println("MEGA:COMMANDS_FROM_ESP32_ONLY");

  // Notify ESP32.
  Serial1.println("MEGA:READY");
}


// ==================================================
// LOOP
// ==================================================
void loop() {
  readESP32Command();
  updateAndReportSensors();
}


// ==================================================
// READ COMMANDS FROM ESP32 ONLY
// The Mega no longer reads from USB Serial (Raspberry Pi).
// All commands arrive through Serial1 from the ESP32.
// ==================================================
void readESP32Command() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      handleCommand(command);
    }
  }
}


// ==================================================
// COMMAND HANDLER
// All ACKs go back to ESP32 via Serial1.
// ==================================================
void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Strip optional "MOVE:" prefix for backward compat.
  if (command.startsWith("MOVE:")) {
    command = command.substring(5);
    command.trim();
  }

  // ---- System mode ----
  if (command == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    stopMotors(); stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:MANUAL");
    return;
  }
  if (command == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    stopMotors(); stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:AUTO");
    return;
  }

  // ---- Robot mode ----
  if (command == "MODE:NORMAL") {
    currentMode = "NORMAL";
    stopMotors(); lastMovement = "STOP";
    Serial1.println("ACK:MODE:NORMAL");
    return;
  }
  if (command == "MODE:CLIMB") {
    currentMode = "CLIMB";
    stopMotors(); lastMovement = "STOP";
    Serial1.println("ACK:MODE:CLIMB");
    return;
  }

  // ---- Jacks ----
  if (command == "JACK:FRONT:EXTEND")  { frontJackExtend();  Serial1.println("ACK:JACK:FRONT:EXTEND");  return; }
  if (command == "JACK:FRONT:RETRACT") { frontJackRetract(); Serial1.println("ACK:JACK:FRONT:RETRACT"); return; }
  if (command == "JACK:FRONT:STOP")    { frontJackStop();    Serial1.println("ACK:JACK:FRONT:STOP");    return; }
  if (command == "JACK:REAR:EXTEND")   { rearJackExtend();   Serial1.println("ACK:JACK:REAR:EXTEND");   return; }
  if (command == "JACK:REAR:RETRACT")  { rearJackRetract();  Serial1.println("ACK:JACK:REAR:RETRACT");  return; }
  if (command == "JACK:REAR:STOP")     { rearJackStop();     Serial1.println("ACK:JACK:REAR:STOP");     return; }

  if (command == "JACK:ALL:STOP" || command == "JACK:STOP" || command == "JSTOP") {
    stopAllJacks();
    Serial1.println("ACK:JACK:ALL:STOP");
    return;
  }

  // ---- Movement ----
  if (command == "FORWARD")  { lastMovement = "FORWARD";  moveForward();  Serial1.println("ACK:FORWARD");  return; }
  if (command == "BACKWARD") { lastMovement = "BACKWARD"; moveBackward(); Serial1.println("ACK:BACKWARD"); return; }
  if (command == "LEFT")     { lastMovement = "LEFT";     turnLeft();     Serial1.println("ACK:LEFT");     return; }
  if (command == "RIGHT")    { lastMovement = "RIGHT";    turnRight();    Serial1.println("ACK:RIGHT");    return; }

  if (command == "STOP" || command == "CMD:STOP" || command == "ESTOP") {
    lastMovement = "STOP";
    stopMotors(); stopAllJacks();
    Serial1.println("ACK:STOP");
    return;
  }

  // ---- Speed ----
  if (command.startsWith("SPEED:")) {
    int spd = constrain(command.substring(6).toInt(), 0, 100);
    motorSpeedPercent = spd;
    motorSpeed = map(spd, 0, 100, 0, 255);
    Serial1.print("ACK:SPEED:"); Serial1.println(spd);
    applyLastMovement();
    return;
  }

  // ---- Status (send to Raspberry Pi USB) ----
  if (command == "STATUS") {
    sendStatusToPI();
    Serial1.println("ACK:STATUS:SENT_TO_PI");
    return;
  }

  Serial1.print("ERROR:UNKNOWN_COMMAND:"); Serial1.println(command);
}


// ==================================================
// SENSOR REPORTING
//
// Called every loop iteration.
// 1) Periodic report every 10 seconds.
// 2) Critical alert as soon as threshold is crossed
//    (with a 2-second cooldown to avoid spam).
// ==================================================
void updateAndReportSensors() {
  readMPU();
  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM  = readUltrasonicCM(REAR_US_TRIG,  REAR_US_ECHO);

  unsigned long now = millis();

  // Check for critical values first (interrupt-style push).
  String alert = detectAlert();
  bool alertActive = (alert != "NONE");

  if (alertActive && (now - lastAlertSent >= ALERT_COOLDOWN_MS)) {
    lastAlertSent = now;
    sendSensorData(alert);
    // Reset periodic timer so we don't double-send immediately after.
    lastPeriodicReport = now;
    return;
  }

  // Periodic report every 10 seconds.
  if (now - lastPeriodicReport >= SENSOR_REPORT_INTERVAL) {
    lastPeriodicReport = now;
    sendSensorData("NONE");
  }
}


// Returns the alert name if any critical threshold is crossed, else "NONE".
String detectAlert() {
  if (abs(pitch) > PITCH_DANGER_DEG) return "TILT_DANGER";
  if (abs(roll)  > ROLL_DANGER_DEG)  return "TILT_DANGER";

  if (frontUltrasonicCM > 0 && frontUltrasonicCM < OBSTACLE_FRONT_CM)
    return "OBSTACLE_FRONT";

  if (rearUltrasonicCM > 0 && rearUltrasonicCM < OBSTACLE_REAR_CM)
    return "OBSTACLE_REAR";

  return "NONE";
}


// Send sensor data line to Raspberry Pi over USB Serial.
// Format: SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
void sendSensorData(String alertType) {
  Serial.print("SENSOR:");
  Serial.print("PITCH="); Serial.print(pitch, 2);
  Serial.print(";ROLL="); Serial.print(roll, 2);
  Serial.print(";UF=");   Serial.print(frontUltrasonicCM, 2);
  Serial.print(";UR=");   Serial.print(rearUltrasonicCM, 2);
  Serial.print(";ALERT="); Serial.println(alertType);
}


// Full status line (sent on STATUS command).
void sendStatusToPI() {
  Serial.print("STATUS:");
  Serial.print("SYS=");    Serial.print(systemMode);
  Serial.print(";MODE=");  Serial.print(currentMode);
  Serial.print(";MOVE=");  Serial.print(lastMovement);
  Serial.print(";SPEED="); Serial.print(motorSpeedPercent);
  Serial.print(";PITCH="); Serial.print(pitch, 2);
  Serial.print(";ROLL=");  Serial.print(roll, 2);
  Serial.print(";UF=");    Serial.print(frontUltrasonicCM, 2);
  Serial.print(";UR=");    Serial.println(rearUltrasonicCM, 2);
}


// ==================================================
// MOTOR FUNCTIONS
// ==================================================
void moveForward()  { analogWrite(RIGHT_RPWM, motorSpeed); analogWrite(RIGHT_LPWM, 0); analogWrite(LEFT_RPWM, motorSpeed); analogWrite(LEFT_LPWM, 0); }
void moveBackward() { analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, motorSpeed); analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, motorSpeed); }
void turnLeft()     { analogWrite(RIGHT_RPWM, motorSpeed); analogWrite(RIGHT_LPWM, 0); analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, motorSpeed); }
void turnRight()    { analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, motorSpeed); analogWrite(LEFT_RPWM, motorSpeed); analogWrite(LEFT_LPWM, 0); }
void stopMotors()   { analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0); analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, 0); }

void applyLastMovement() {
  if      (lastMovement == "FORWARD")  moveForward();
  else if (lastMovement == "BACKWARD") moveBackward();
  else if (lastMovement == "LEFT")     turnLeft();
  else if (lastMovement == "RIGHT")    turnRight();
  else                                  stopMotors();
}


// ==================================================
// JACK FUNCTIONS
// ==================================================
void rearJackExtend()  { digitalWrite(REAR_JACK_IN1, HIGH); digitalWrite(REAR_JACK_IN2, LOW);  }
void rearJackRetract() { digitalWrite(REAR_JACK_IN1, LOW);  digitalWrite(REAR_JACK_IN2, HIGH); }
void rearJackStop()    { digitalWrite(REAR_JACK_IN1, LOW);  digitalWrite(REAR_JACK_IN2, LOW);  }
void frontJackExtend() { digitalWrite(FRONT_JACK_IN3, HIGH); digitalWrite(FRONT_JACK_IN4, LOW);  }
void frontJackRetract(){ digitalWrite(FRONT_JACK_IN3, LOW);  digitalWrite(FRONT_JACK_IN4, HIGH); }
void frontJackStop()   { digitalWrite(FRONT_JACK_IN3, LOW);  digitalWrite(FRONT_JACK_IN4, LOW);  }
void stopAllJacks()    { rearJackStop(); frontJackStop(); }


// ==================================================
// ULTRASONIC
// ==================================================
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  return (duration == 0) ? -1.0 : duration * 0.0343 / 2.0;
}


// ==================================================
// MPU6500
// ==================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  byte err = Wire.endTransmission();
  Serial.println(err == 0 ? "MPU:OK" : "MPU:ERROR");
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) return;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  accelX = rawAx / 16384.0;
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  roll  = atan2(-accelX, accelZ) * 180.0 / PI;
}
