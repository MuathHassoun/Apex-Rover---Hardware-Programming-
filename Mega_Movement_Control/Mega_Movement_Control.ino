#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Sensors + Motors + Jacks
//
// Communication:
// Raspberry Pi USB ---> Mega USB Serial
// ESP32 G17 TX      ---> Mega RX1 Pin 19 Serial1
//
// New behavior:
// If Raspberry asks directly over USB:
//   GET:SENSORS
// Mega replies to Raspberry USB.
//
// If Raspberry asks ESP32 over WiFi, ESP32 sends to Mega:
//   GET:SENSORS
// Mega sends sensor data directly to Raspberry USB Serial,
// and only sends ACK to ESP32.
//
// Sensor data format sent to Raspberry:
//   DATA:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20
// ==================================================


// ==================================================
// 1) BTS7960 MOTOR DRIVER PINS
// ==================================================

#define RIGHT_RPWM 9
#define RIGHT_LPWM 11

#define LEFT_RPWM 5
#define LEFT_LPWM 7


// ==================================================
// 2) ULTRASONIC SENSOR PINS
// ==================================================
//
// Front Ultrasonic:
// TRIG ---> Mega A8
// ECHO ---> Mega A9
//
// Rear Ultrasonic:
// TRIG ---> Mega A11
// ECHO ---> Mega A12
// ==================================================

#define FRONT_US_TRIG A8
#define FRONT_US_ECHO A9

#define REAR_US_TRIG A11
#define REAR_US_ECHO A12


// ==================================================
// 3) MPU6500 / MPU6050
// ==================================================
//
// SDA ---> Mega 20
// SCL ---> Mega 21
// ==================================================

#define MPU_ADDR 0x68

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float pitch = 0;
float roll = 0;


// ==================================================
// 4) L298N LINEAR ACTUATORS
// ==================================================
//
// Rear Jack:
// IN1 ---> Mega A0
// IN2 ---> Mega A1
//
// Front Jack:
// IN3 ---> Mega A3
// IN4 ---> Mega A4
// ==================================================

#define REAR_JACK_IN1 A0
#define REAR_JACK_IN2 A1

#define FRONT_JACK_IN3 A3
#define FRONT_JACK_IN4 A4


// ==================================================
// 5) ROBOT STATE
// ==================================================

int motorSpeed = 150;
int motorSpeedPercent = 60;

String currentMode = "NORMAL";
String systemMode = "MANUAL";
String lastMovement = "STOP";

float frontUltrasonicCM = -1.0;
float rearUltrasonicCM = -1.0;

unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 120;

bool debugToUSB = false;
bool debugToESP32 = false;


// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  Serial1.begin(9600);
  Serial1.setTimeout(10);

  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  pinMode(FRONT_US_TRIG, OUTPUT);
  pinMode(FRONT_US_ECHO, INPUT);

  pinMode(REAR_US_TRIG, OUTPUT);
  pinMode(REAR_US_ECHO, INPUT);

  digitalWrite(FRONT_US_TRIG, LOW);
  digitalWrite(REAR_US_TRIG, LOW);

  pinMode(REAR_JACK_IN1, OUTPUT);
  pinMode(REAR_JACK_IN2, OUTPUT);

  pinMode(FRONT_JACK_IN3, OUTPUT);
  pinMode(FRONT_JACK_IN4, OUTPUT);

  stopMotors();
  stopAllJacks();

  Wire.begin();
  initMPU();

  delay(500);

  // USB Serial to Raspberry
  Serial.println("MEGA:READY");
  Serial.println("MEGA:USB_FOR_RASPBERRY_PI");
  Serial.println("MEGA:ESP32_ON_SERIAL1");
  Serial.println("MEGA:SENSOR_DIRECT_TO_PI_ENABLED");

  // Serial1 ACK to ESP32
  Serial1.println("MEGA:READY");
}


// ==================================================
// LOOP
// ==================================================

void loop() {
  readRaspberryCommand();
  readESP32Command();
  updateSensorsPeriodically();
}


// ==================================================
// READ COMMANDS
// ==================================================

void readRaspberryCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial, "RASPBERRY_PI");
    }
  }
}


void readESP32Command() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command, Serial1, "ESP32");
    }
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================

void handleCommand(String command, Stream &replyPort, String sourceName) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.startsWith("MOVE:")) {
    command = command.substring(5);
    command.trim();
  }

  // =========================
  // SYSTEM MODE
  // =========================

  if (command == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";

    replyPort.println("ACK:SYS:MODE:MANUAL");
    return;
  }

  if (command == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";

    replyPort.println("ACK:SYS:MODE:AUTO");
    return;
  }

  // =========================
  // SENSOR REQUEST
  // =========================
  //
  // Important:
  // If command came from ESP32, the sensor DATA goes to Raspberry via USB Serial.
  // ESP32 only receives an ACK.
  //
  // If command came from Raspberry USB directly, Raspberry gets DATA directly.
  // =========================

  if (command == "GET:SENSORS") {
    if (sourceName == "ESP32") {
      sendSensorDataToRaspberryUSB();
      replyPort.println("ACK:GET:SENSORS:SENT_TO_PI_USB");
    } else {
      sendSensorData(replyPort);
    }
    return;
  }

  // =========================
  // DEBUG
  // =========================

  if (command == "DEBUG:USB:ON") {
    debugToUSB = true;
    replyPort.println("ACK:DEBUG:USB:ON");
    return;
  }

  if (command == "DEBUG:USB:OFF") {
    debugToUSB = false;
    replyPort.println("ACK:DEBUG:USB:OFF");
    return;
  }

  if (command == "DEBUG:ESP32:ON") {
    debugToESP32 = true;
    replyPort.println("ACK:DEBUG:ESP32:ON");
    return;
  }

  if (command == "DEBUG:ESP32:OFF") {
    debugToESP32 = false;
    replyPort.println("ACK:DEBUG:ESP32:OFF");
    return;
  }

  // =========================
  // OLD MODE COMMANDS
  // =========================

  if (command == "MODE:NORMAL") {
    currentMode = "NORMAL";
    stopMotors();
    lastMovement = "STOP";

    replyPort.println("ACK:MODE:NORMAL");
    return;
  }

  if (command == "MODE:CLIMB") {
    currentMode = "CLIMB";
    stopMotors();
    lastMovement = "STOP";

    replyPort.println("ACK:MODE:CLIMB");
    return;
  }

  // =========================
  // JACK COMMANDS
  // =========================

  if (command == "JACK:FRONT:EXTEND") {
    frontJackExtend();
    replyPort.println("ACK:JACK:FRONT:EXTEND");
    return;
  }

  if (command == "JACK:FRONT:RETRACT") {
    frontJackRetract();
    replyPort.println("ACK:JACK:FRONT:RETRACT");
    return;
  }

  if (command == "JACK:FRONT:STOP") {
    frontJackStop();
    replyPort.println("ACK:JACK:FRONT:STOP");
    return;
  }

  if (command == "JACK:REAR:EXTEND") {
    rearJackExtend();
    replyPort.println("ACK:JACK:REAR:EXTEND");
    return;
  }

  if (command == "JACK:REAR:RETRACT") {
    rearJackRetract();
    replyPort.println("ACK:JACK:REAR:RETRACT");
    return;
  }

  if (command == "JACK:REAR:STOP") {
    rearJackStop();
    replyPort.println("ACK:JACK:REAR:STOP");
    return;
  }

  if (command == "JACK:ALL:STOP" || command == "JACK:STOP" || command == "JSTOP") {
    stopAllJacks();
    replyPort.println("ACK:JACK:ALL:STOP");
    return;
  }

  // =========================
  // MOVEMENT COMMANDS
  // =========================

  if (command == "FORWARD") {
    lastMovement = "FORWARD";
    moveForward();
    replyPort.println("ACK:FORWARD");
    return;
  }

  if (command == "BACKWARD") {
    lastMovement = "BACKWARD";
    moveBackward();
    replyPort.println("ACK:BACKWARD");
    return;
  }

  if (command == "LEFT") {
    lastMovement = "LEFT";
    turnLeft();
    replyPort.println("ACK:LEFT");
    return;
  }

  if (command == "RIGHT") {
    lastMovement = "RIGHT";
    turnRight();
    replyPort.println("ACK:RIGHT");
    return;
  }

  if (command == "STOP" || command == "CMD:STOP" || command == "ESTOP") {
    lastMovement = "STOP";
    stopMotors();
    stopAllJacks();

    replyPort.println("ACK:STOP");
    return;
  }

  // =========================
  // SPEED
  // =========================

  if (command.startsWith("SPEED:")) {
    int speedPercent = command.substring(6).toInt();
    speedPercent = constrain(speedPercent, 0, 100);

    motorSpeedPercent = speedPercent;
    motorSpeed = map(speedPercent, 0, 100, 0, 255);

    replyPort.print("ACK:SPEED:");
    replyPort.println(speedPercent);

    applyLastMovement();
    return;
  }

  // =========================
  // STATUS
  // =========================

  if (command == "STATUS") {
    sendStatus(replyPort);
    return;
  }

  replyPort.print("ERROR:UNKNOWN_COMMAND:");
  replyPort.println(command);
}


// ==================================================
// SENSOR UPDATE
// ==================================================

void updateSensorsPeriodically() {
  if (millis() - lastSensorUpdate < sensorUpdateInterval) {
    return;
  }

  lastSensorUpdate = millis();

  readMPU();

  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);
}


// This is used when Raspberry asks directly.
void sendSensorData(Stream &replyPort) {
  readMPU();

  frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
  rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);

  replyPort.print("DATA:");
  replyPort.print("PITCH=");
  replyPort.print(pitch, 2);

  replyPort.print(";ROLL=");
  replyPort.print(roll, 2);

  replyPort.print(";UF=");
  replyPort.print(frontUltrasonicCM, 2);

  replyPort.print(";UR=");
  replyPort.println(rearUltrasonicCM, 2);
}


// This is used when ESP32 requested sensors for Raspberry.
// The response must go to USB Serial, not back to ESP32.
void sendSensorDataToRaspberryUSB() {
  sendSensorData(Serial);
}


void sendStatus(Stream &replyPort) {
  replyPort.print("DATA:STATUS:");
  replyPort.print("SYSTEM=");
  replyPort.print(systemMode);

  replyPort.print(";MODE=");
  replyPort.print(currentMode);

  replyPort.print(";MOVE=");
  replyPort.print(lastMovement);

  replyPort.print(";SPEED=");
  replyPort.print(motorSpeedPercent);

  replyPort.print(";PITCH=");
  replyPort.print(pitch, 2);

  replyPort.print(";ROLL=");
  replyPort.print(roll, 2);

  replyPort.print(";UF=");
  replyPort.print(frontUltrasonicCM, 2);

  replyPort.print(";UR=");
  replyPort.println(rearUltrasonicCM, 2);
}


// ==================================================
// MOTOR FUNCTIONS
// ==================================================

void moveForward() {
  analogWrite(RIGHT_RPWM, motorSpeed);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, motorSpeed);
  analogWrite(LEFT_LPWM, 0);
}


void moveBackward() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, motorSpeed);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, motorSpeed);
}


void turnLeft() {
  analogWrite(RIGHT_RPWM, motorSpeed);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, motorSpeed);
}


void turnRight() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, motorSpeed);

  analogWrite(LEFT_RPWM, motorSpeed);
  analogWrite(LEFT_LPWM, 0);
}


void stopMotors() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);

  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);
}


void applyLastMovement() {
  if (lastMovement == "FORWARD") {
    moveForward();
  } else if (lastMovement == "BACKWARD") {
    moveBackward();
  } else if (lastMovement == "LEFT") {
    turnLeft();
  } else if (lastMovement == "RIGHT") {
    turnRight();
  } else {
    stopMotors();
  }
}


// ==================================================
// JACK FUNCTIONS
// ==================================================

void rearJackExtend() {
  digitalWrite(REAR_JACK_IN1, HIGH);
  digitalWrite(REAR_JACK_IN2, LOW);
}


void rearJackRetract() {
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, HIGH);
}


void rearJackStop() {
  digitalWrite(REAR_JACK_IN1, LOW);
  digitalWrite(REAR_JACK_IN2, LOW);
}


void frontJackExtend() {
  digitalWrite(FRONT_JACK_IN3, HIGH);
  digitalWrite(FRONT_JACK_IN4, LOW);
}


void frontJackRetract() {
  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, HIGH);
}


void frontJackStop() {
  digitalWrite(FRONT_JACK_IN3, LOW);
  digitalWrite(FRONT_JACK_IN4, LOW);
}


void stopAllJacks() {
  rearJackStop();
  frontJackStop();
}


// ==================================================
// ULTRASONIC
// ==================================================

float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}


// ==================================================
// MPU
// ==================================================

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("MPU:OK");
  } else {
    Serial.println("MPU:ERROR");
  }
}


void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() >= 6) {
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();

    accelX = rawAx / 16384.0;
    accelY = rawAy / 16384.0;
    accelZ = rawAz / 16384.0;

    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    roll = atan2(-accelX, accelZ) * 180.0 / PI;
  }
}