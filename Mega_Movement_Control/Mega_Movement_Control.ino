#include <Wire.h>
#include <math.h>

// ==================================================
// Arduino Mega - Apex Rover Motors + Jacks + Sensors
//
// Mega responsibilities:
// 1. Movement motors
// 2. Front / rear jacks
// 3. MPU6500 pitch/roll/yaw
// 4. Ultrasonic display only
// 5. SENSOR messages to Raspberry over USB Serial
// 6. LEGO movement blocks:
//
//    AUTO:UP_STAIRS
//    AUTO:DOWN_STAIRS
//    BLOCK:TURN:LEFT:90
//    BLOCK:TURN:RIGHT:90
//    BLOCK:GO:FORWARD:20
//    BLOCK:GO:BACKWARD:20
//    BLOCK:JACK:REAR:EXTEND:4
//    BLOCK:JACK:REAR:RETRACT:4
//    BLOCK:JACK:FRONT:EXTEND:4
//    BLOCK:JACK:FRONT:RETRACT:4
//    BLOCK:STOP
//
// NEW UP_STAIRS LOGIC:
// Forward speed 40
// If pitch >= 3.5 deg -> rear jack extend immediately
// Rear jack extends until MPU detects effect or timeout
// Then robot moves forward slowly speed 30
// Then motors stop and rear jack retracts for long time
// Then repeat for next stair
//
// STOP / BLOCK:STOP / AUTO:STOP always works.
// Ultrasonic is display only.
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
// Display only
// ==================================================
#define FRONT_US_TRIG A8
#define FRONT_US_ECHO A9
#define REAR_US_TRIG  A11
#define REAR_US_ECHO  A12

const bool ULTRASONIC_ENABLED = true;
const unsigned long ULTRASONIC_READ_INTERVAL_MS = 250;
unsigned long lastUltrasonicRead = 0;


// ==================================================
// MPU6500 I2C
// Mega SDA = 20
// Mega SCL = 21
// ==================================================
#define MPU_ADDR 0x68

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float gyroZBias = 0.0;
float yawDeg = 0.0;

float pitch = 0.0;
float roll  = 0.0;

unsigned long lastMPUIntegrationMicros = 0;


// ==================================================
// JACK PINS (L298N)
// Rear Jack:  IN1=A0 IN2=A1
// Front Jack: IN3=A3 IN4=A4
// ==================================================
#define REAR_JACK_IN1  A0
#define REAR_JACK_IN2  A1
#define FRONT_JACK_IN3 A3
#define FRONT_JACK_IN4 A4


// ==================================================
// ROBOT STATE
// ==================================================
int motorSpeed = 150;
int motorSpeedPercent = 60;

String currentMode = "NORMAL";
String systemMode = "MANUAL";
String lastMovement = "STOP";

float frontUltrasonicCM = -1.0;
float rearUltrasonicCM  = -1.0;


// ==================================================
// SENSOR TIMING
// ==================================================
const unsigned long MANUAL_SENSOR_REPORT_INTERVAL = 5000;
const unsigned long AUTO_SENSOR_REPORT_INTERVAL   = 500;

const unsigned long MPU_READ_INTERVAL             = 50;
const unsigned long ALERT_COOLDOWN_MS             = 700;

unsigned long lastPeriodicReport = 0;
unsigned long lastAlertSent      = 0;
unsigned long lastMPURead        = 0;


// ==================================================
// TIMED MOVEMENT PULSE SUPPORT
// ==================================================
bool timedMoveActive = false;
unsigned long timedMoveEndAt = 0;


// ==================================================
// HARD SAFETY THRESHOLDS
// Relaxed for testing, but not fully removed.
// ==================================================
const float PITCH_DANGER_DEG  = 42.0;
const float ROLL_DANGER_DEG   = 40.0;


// ==================================================
// LEGO BLOCK SETTINGS
// ==================================================
const unsigned long GO_MS_PER_UNIT = 80;
const unsigned long GO_MIN_MS = 150;
const unsigned long GO_MAX_MS = 9000;

const unsigned long JACK_MS_PER_UNIT = 500;
const unsigned long JACK_MIN_MS = 200;
const unsigned long JACK_MAX_MS = 12000;

const int TURN_SPEED_PERCENT = 35;
const float TURN_TOLERANCE_DEG = 4.0;
const unsigned long TURN_MIN_TIMEOUT_MS = 2500;
const unsigned long TURN_MS_PER_DEG = 45;
const unsigned long TURN_MAX_TIMEOUT_MS = 25000;


// ==================================================
// UP STAIRS SETTINGS - NEW LOGIC
// ==================================================

// Main approach/climb speed
const int UP_STAIRS_SPEED_PERCENT = 40;

// After rear jack lifts the robot, move slower to reduce deviation
const int UP_AFTER_JACK_SPEED_PERCENT = 30;

// Max full block time
const unsigned long UP_STAIRS_MAX_TOTAL_MS = 180000;

// Easier stair detection
const float UP_PITCH_CLIMB_START_DEG = 3.5;

// Rear jack extend logic
const unsigned long UP_JACK_MAX_EXTEND_MS = 8500;
const float UP_JACK_EFFECT_DELTA_DEG = 1.0;

// Forward after jack lift
const unsigned long UP_JACK_AFTER_EFFECT_FORWARD_MS = 2300;

// Important: long retract time so rear jack does not hit stair edge
const unsigned long UP_JACK_RETRACT_MS = 7500;

// Limit jack cycles
const int UP_MAX_JACK_USES = 20;

// Roll warning only
const float UP_ROLL_WARNING_DEG = 28.0;


// ==================================================
// LEGO BLOCK STATE MACHINE
// ==================================================
enum MegaBlockType {
  BLOCK_NONE,
  BLOCK_GO,
  BLOCK_TURN,
  BLOCK_JACK,
  BLOCK_UP_STAIRS,
  BLOCK_DOWN_STAIRS_PLACEHOLDER
};

enum MegaBlockStep {
  STEP_IDLE,

  STEP_GO_RUNNING,

  STEP_TURN_RUNNING,

  STEP_JACK_RUNNING,

  STEP_UP_INIT,
  STEP_UP_FORWARD_CLIMB,
  STEP_UP_REAR_JACK_EXTEND,
  STEP_UP_AFTER_JACK_FORWARD,
  STEP_UP_REAR_JACK_RETRACT,

  STEP_DOWN_PLACEHOLDER
};

MegaBlockType activeBlock = BLOCK_NONE;
MegaBlockStep blockStep = STEP_IDLE;

String activeBlockName = "NONE";

unsigned long blockStartMs = 0;
unsigned long blockStepStartMs = 0;
unsigned long blockEndAtMs = 0;

int savedMotorSpeedPercent = 60;

// GO block
String blockGoDirection = "FORWARD";

// TURN block
String blockTurnDirection = "LEFT";
float blockTurnTargetDeg = 90.0;
float blockTurnStartYaw = 0.0;

// JACK block
String blockJackWhich = "REAR";
String blockJackAction = "EXTEND";

// UP STAIRS
float upJackStartPitch = 0.0;
int upJackUseCount = 0;
int upDetectedStepCount = 0;


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
  pinMode(LEFT_RPWM,  OUTPUT);
  pinMode(LEFT_LPWM,  OUTPUT);

  pinMode(FRONT_US_TRIG, OUTPUT);
  pinMode(FRONT_US_ECHO, INPUT);
  pinMode(REAR_US_TRIG,  OUTPUT);
  pinMode(REAR_US_ECHO,  INPUT);

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
  calibrateGyroZ();

  delay(500);

  Serial.println("MEGA:READY");
  Serial.println("MEGA:SENSOR_MODE_DEPENDENT_STREAM_ENABLED");
  Serial.println("MEGA:MANUAL_SENSOR_INTERVAL_5000MS");
  Serial.println("MEGA:AUTO_SENSOR_INTERVAL_500MS");
  Serial.println("MEGA:ULTRASONIC_DISPLAY_ONLY_ENABLED");
  Serial.println("MEGA:ULTRASONIC_DECISION_DISABLED");
  Serial.println("MEGA:LEGO_BLOCKS_ENABLED");
  Serial.println("MEGA:AUTO_UP_STAIRS_DIRECT_REAR_JACK_ENABLED");
  Serial.println("MEGA:UP_STAIRS_SPEED_40_AFTER_JACK_30");
  Serial.println("MEGA:HARD_TILT_ONLY_ENABLED");

  Serial1.println("MEGA:READY");
  Serial1.println("MEGA:LEGO_BLOCKS_ENABLED");
  Serial1.println("MEGA:AUTO_UP_STAIRS_DIRECT_REAR_JACK_ENABLED");
  Serial1.println("MEGA:UP_STAIRS_SPEED_40_AFTER_JACK_30");
}


// ==================================================
// LOOP
// ==================================================
void loop() {
  readESP32Command();

  updateTimedMovement();

  runMegaBlockStateMachine();

  updateAndReportSensors();
}


// ==================================================
// READ COMMANDS FROM ESP32 ONLY
// ==================================================
void readESP32Command() {
  if (Serial1.available() <= 0) {
    return;
  }

  String command = Serial1.readStringUntil('\n');
  command.trim();

  if (command.length() > 0) {
    handleCommand(command);
  }
}


// ==================================================
// COMMAND HANDLER
// ==================================================
void handleCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.startsWith("MOVE:")) {
    command = command.substring(5);
    command.trim();
  }

  command.toUpperCase();

  if (command.startsWith("AUTO:") || command.startsWith("BLOCK:")) {
    handleMegaBlockCommand(command);
    return;
  }

  if (command == "SYS:MODE:MANUAL") {
    stopActiveBlock("SYS_MODE_MANUAL");
    systemMode = "MANUAL";
    timedMoveActive = false;
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:MANUAL");
    sendStatusToPI();
    return;
  }

  if (command == "SYS:MODE:AUTO") {
    stopActiveBlock("SYS_MODE_AUTO");
    systemMode = "AUTO";
    timedMoveActive = false;
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";
    Serial1.println("ACK:SYS:MODE:AUTO");
    sendStatusToPI();
    return;
  }

  if (command == "MODE:NORMAL") {
    stopActiveBlock("MODE_NORMAL");
    currentMode = "NORMAL";
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:MODE:NORMAL");
    sendStatusToPI();
    return;
  }

  if (command == "MODE:CLIMB") {
    stopActiveBlock("MODE_CLIMB");
    currentMode = "CLIMB";
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:MODE:CLIMB");
    sendStatusToPI();
    return;
  }

  if (command == "JACK:REAR:EXTEND") {
    stopActiveBlock("MANUAL_REAR_JACK_EXTEND");
    rearJackExtend();
    Serial1.println("ACK:JACK:REAR:EXTEND");
    return;
  }

  if (command == "JACK:REAR:RETRACT") {
    stopActiveBlock("MANUAL_REAR_JACK_RETRACT");
    rearJackRetract();
    Serial1.println("ACK:JACK:REAR:RETRACT");
    return;
  }

  if (command == "JACK:REAR:STOP") {
    rearJackStop();
    Serial1.println("ACK:JACK:REAR:STOP");
    return;
  }

  if (command == "JACK:FRONT:EXTEND") {
    stopActiveBlock("MANUAL_FRONT_JACK_EXTEND");
    frontJackExtend();
    Serial1.println("ACK:JACK:FRONT:EXTEND");
    return;
  }

  if (command == "JACK:FRONT:RETRACT") {
    stopActiveBlock("MANUAL_FRONT_JACK_RETRACT");
    frontJackRetract();
    Serial1.println("ACK:JACK:FRONT:RETRACT");
    return;
  }

  if (command == "JACK:FRONT:STOP") {
    frontJackStop();
    Serial1.println("ACK:JACK:FRONT:STOP");
    return;
  }

  if (command == "JACK:ALL:STOP" || command == "JACK:STOP" || command == "JSTOP") {
    stopAllJacks();
    Serial1.println("ACK:JACK:ALL:STOP");
    return;
  }

  if (command.startsWith("PULSE:")) {
    stopActiveBlock("PULSE_OVERRIDE");
    handlePulseCommand(command);
    return;
  }

  if (command == "FORWARD") {
    stopActiveBlock("MANUAL_FORWARD");
    timedMoveActive = false;
    lastMovement = "FORWARD";
    moveForward();
    Serial1.println("ACK:FORWARD");
    return;
  }

  if (command == "BACKWARD") {
    stopActiveBlock("MANUAL_BACKWARD");
    timedMoveActive = false;
    lastMovement = "BACKWARD";
    moveBackward();
    Serial1.println("ACK:BACKWARD");
    return;
  }

  if (command == "LEFT") {
    stopActiveBlock("MANUAL_LEFT");
    timedMoveActive = false;
    lastMovement = "LEFT";
    turnLeft();
    Serial1.println("ACK:LEFT");
    return;
  }

  if (command == "RIGHT") {
    stopActiveBlock("MANUAL_RIGHT");
    timedMoveActive = false;
    lastMovement = "RIGHT";
    turnRight();
    Serial1.println("ACK:RIGHT");
    return;
  }

  if (command == "STOP" || command == "CMD:STOP" || command == "ESTOP") {
    stopActiveBlock("STOP_COMMAND");
    timedMoveActive = false;
    lastMovement = "STOP";
    stopMotors();
    stopAllJacks();
    Serial1.println("ACK:STOP");
    return;
  }

  if (command.startsWith("SPEED:")) {
    int spd = command.substring(6).toInt();
    setSpeedPercent(spd, true);

    Serial1.print("ACK:SPEED:");
    Serial1.println(motorSpeedPercent);
    return;
  }

  if (command == "STATUS") {
    sendStatusToPI();
    Serial1.println("ACK:STATUS:SENT_TO_PI");
    return;
  }

  Serial1.print("ERROR:UNKNOWN_COMMAND:");
  Serial1.println(command);
}


// ==================================================
// LEGO BLOCK COMMAND HANDLER
// ==================================================
void handleMegaBlockCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "BLOCK:STOP" || command == "AUTO:STOP") {
    stopActiveBlock("BLOCK_STOP_COMMAND");
    stopMotors();
    stopAllJacks();
    lastMovement = "STOP";
    blockAck("DONE:BLOCK_STOP");
    return;
  }

  if (command == "AUTO:UP_STAIRS") {
    startUpStairsBlock();
    return;
  }

  if (command == "AUTO:DOWN_STAIRS") {
    startDownStairsPlaceholder();
    return;
  }

  if (command.startsWith("BLOCK:TURN:")) {
    startTurnBlock(command);
    return;
  }

  if (command.startsWith("BLOCK:GO:")) {
    startGoBlock(command);
    return;
  }

  if (command.startsWith("BLOCK:JACK:")) {
    startJackBlock(command);
    return;
  }

  blockError("UNKNOWN_BLOCK_COMMAND", command);
}


// ==================================================
// BLOCK PARSING HELPERS
// ==================================================
String getToken(String text, int wantedIndex) {
  int currentIndex = 0;
  int start = 0;

  for (int i = 0; i <= text.length(); i++) {
    if (i == text.length() || text.charAt(i) == ':') {
      if (currentIndex == wantedIndex) {
        return text.substring(start, i);
      }

      currentIndex++;
      start = i + 1;
    }
  }

  return "";
}

int getTokenInt(String text, int wantedIndex, int defaultValue) {
  String token = getToken(text, wantedIndex);
  token.trim();

  if (token.length() == 0) {
    return defaultValue;
  }

  return token.toInt();
}


// ==================================================
// BLOCK ACK / ERROR HELPERS
// ==================================================
void sendBoth(String message) {
  Serial1.println(message);
  Serial.println(message);
}

void blockAck(String message) {
  sendBoth("ACK:MEGA:" + message);
}

void blockError(String code, String details) {
  stopMotors();
  stopAllJacks();
  restoreSavedSpeed();

  activeBlock = BLOCK_NONE;
  blockStep = STEP_IDLE;
  activeBlockName = "NONE";

  lastMovement = "STOP";

  sendBoth("ERR:MEGA:" + code + ":" + details);
}

void stopActiveBlock(String reason) {
  if (activeBlock == BLOCK_NONE) {
    return;
  }

  stopMotors();
  stopAllJacks();
  restoreSavedSpeed();

  blockAck("CANCELLED:" + activeBlockName + ":" + reason);

  activeBlock = BLOCK_NONE;
  blockStep = STEP_IDLE;
  activeBlockName = "NONE";
  lastMovement = "STOP";
}

void finishActiveBlock(String doneName) {
  stopMotors();
  stopAllJacks();
  restoreSavedSpeed();

  activeBlock = BLOCK_NONE;
  blockStep = STEP_IDLE;
  activeBlockName = "NONE";
  lastMovement = "STOP";

  blockAck("DONE:" + doneName);
}

void saveCurrentSpeed() {
  savedMotorSpeedPercent = motorSpeedPercent;
}

void restoreSavedSpeed() {
  setSpeedPercent(savedMotorSpeedPercent, false);
}


// ==================================================
// BLOCK SAFETY
// ==================================================
bool blockTiltSafetyOK(String blockName) {
  if (fabs(pitch) >= PITCH_DANGER_DEG || fabs(roll) >= ROLL_DANGER_DEG) {
    blockError(blockName + ":HARD_TILT_DANGER", "PITCH=" + String(pitch, 2) + ";ROLL=" + String(roll, 2));
    return false;
  }

  return true;
}

bool frontHardObstacle() {
  return false;
}

bool rearHardObstacle() {
  return false;
}


// ==================================================
// BLOCK: GO
// ==================================================
void startGoBlock(String command) {
  stopActiveBlock("NEW_GO_BLOCK");

  String dir = getToken(command, 2);
  int amount = getTokenInt(command, 3, 20);

  dir.trim();
  dir.toUpperCase();

  if (dir != "FORWARD" && dir != "BACKWARD") {
    blockError("BAD_GO_DIRECTION", command);
    return;
  }

  amount = constrain(amount, 1, 200);

  unsigned long durationMs = (unsigned long)amount * GO_MS_PER_UNIT;
  durationMs = constrain(durationMs, GO_MIN_MS, GO_MAX_MS);

  saveCurrentSpeed();

  activeBlock = BLOCK_GO;
  blockStep = STEP_GO_RUNNING;
  activeBlockName = "GO_" + dir;

  blockGoDirection = dir;
  blockStartMs = millis();
  blockStepStartMs = blockStartMs;
  blockEndAtMs = blockStartMs + durationMs;

  timedMoveActive = false;

  if (dir == "FORWARD") {
    lastMovement = "FORWARD";
    moveForward();
  } else {
    lastMovement = "BACKWARD";
    moveBackward();
  }

  blockAck("START:GO:" + dir + ":" + String(amount) + ":DURATION_MS=" + String(durationMs));
}


// ==================================================
// BLOCK: TURN
// ==================================================
void startTurnBlock(String command) {
  stopActiveBlock("NEW_TURN_BLOCK");

  String dir = getToken(command, 2);
  int deg = getTokenInt(command, 3, 90);

  dir.trim();
  dir.toUpperCase();

  if (dir != "LEFT" && dir != "RIGHT") {
    blockError("BAD_TURN_DIRECTION", command);
    return;
  }

  deg = constrain(deg, 0, 360);

  if (deg == 0) {
    blockAck("DONE:TURN:" + dir + ":0");
    return;
  }

  saveCurrentSpeed();
  setSpeedPercent(TURN_SPEED_PERCENT, false);

  activeBlock = BLOCK_TURN;
  blockStep = STEP_TURN_RUNNING;
  activeBlockName = "TURN_" + dir;

  blockTurnDirection = dir;
  blockTurnTargetDeg = (float)deg;
  blockTurnStartYaw = yawDeg;

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;

  unsigned long timeoutMs = TURN_MIN_TIMEOUT_MS + ((unsigned long)deg * TURN_MS_PER_DEG);
  timeoutMs = constrain(timeoutMs, TURN_MIN_TIMEOUT_MS, TURN_MAX_TIMEOUT_MS);
  blockEndAtMs = blockStartMs + timeoutMs;

  timedMoveActive = false;

  if (dir == "LEFT") {
    lastMovement = "LEFT";
    turnLeft();
  } else {
    lastMovement = "RIGHT";
    turnRight();
  }

  blockAck("START:TURN:" + dir + ":" + String(deg));
}


// ==================================================
// BLOCK: JACK
// ==================================================
void startJackBlock(String command) {
  stopActiveBlock("NEW_JACK_BLOCK");

  String which = getToken(command, 2);
  String action = getToken(command, 3);
  int amount = getTokenInt(command, 4, 4);

  which.trim();
  which.toUpperCase();

  action.trim();
  action.toUpperCase();

  if (which != "REAR" && which != "FRONT") {
    blockError("BAD_JACK_WHICH", command);
    return;
  }

  if (action != "EXTEND" && action != "RETRACT") {
    blockError("BAD_JACK_ACTION", command);
    return;
  }

  amount = constrain(amount, 1, 20);

  unsigned long durationMs = (unsigned long)amount * JACK_MS_PER_UNIT;
  durationMs = constrain(durationMs, JACK_MIN_MS, JACK_MAX_MS);

  saveCurrentSpeed();

  activeBlock = BLOCK_JACK;
  blockStep = STEP_JACK_RUNNING;
  activeBlockName = "JACK_" + which + "_" + action;

  blockJackWhich = which;
  blockJackAction = action;

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;
  blockEndAtMs = blockStartMs + durationMs;

  timedMoveActive = false;

  if (which == "REAR" && action == "EXTEND") {
    rearJackExtend();
  } else if (which == "REAR" && action == "RETRACT") {
    rearJackRetract();
  } else if (which == "FRONT" && action == "EXTEND") {
    frontJackExtend();
  } else if (which == "FRONT" && action == "RETRACT") {
    frontJackRetract();
  }

  blockAck("START:JACK:" + which + ":" + action + ":" + String(amount) + ":DURATION_MS=" + String(durationMs));
}


// ==================================================
// AUTO BLOCK: UP STAIRS
// ==================================================
void startUpStairsBlock() {
  stopActiveBlock("NEW_UP_STAIRS");

  saveCurrentSpeed();

  activeBlock = BLOCK_UP_STAIRS;
  blockStep = STEP_UP_INIT;
  activeBlockName = "UP_STAIRS";

  systemMode = "AUTO";
  currentMode = "CLIMB";

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;

  upJackStartPitch = pitch;
  upJackUseCount = 0;
  upDetectedStepCount = 0;

  timedMoveActive = false;

  setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);

  blockAck("START:UP_STAIRS:DIRECT_REAR_JACK:SPEED_40");
}


// ==================================================
// AUTO BLOCK: DOWN STAIRS PLACEHOLDER
// ==================================================
void startDownStairsPlaceholder() {
  stopActiveBlock("NEW_DOWN_STAIRS");

  activeBlock = BLOCK_DOWN_STAIRS_PLACEHOLDER;
  blockStep = STEP_DOWN_PLACEHOLDER;
  activeBlockName = "DOWN_STAIRS_PLACEHOLDER";

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;

  blockAck("START:DOWN_STAIRS");
  blockAck("STEP:DOWN_STAIRS:PLACEHOLDER_READY_NO_MOVEMENT");

  finishActiveBlock("DOWN_STAIRS_PLACEHOLDER");
}


// ==================================================
// BLOCK STATE MACHINE RUNNER
// ==================================================
void runMegaBlockStateMachine() {
  if (activeBlock == BLOCK_NONE) {
    return;
  }

  unsigned long now = millis();

  if (activeBlock == BLOCK_GO) {
    runGoBlock(now);
    return;
  }

  if (activeBlock == BLOCK_TURN) {
    runTurnBlock(now);
    return;
  }

  if (activeBlock == BLOCK_JACK) {
    runJackBlock(now);
    return;
  }

  if (activeBlock == BLOCK_UP_STAIRS) {
    runUpStairsBlock(now);
    return;
  }
}


// ==================================================
// RUN BLOCK: GO
// ==================================================
void runGoBlock(unsigned long now) {
  if (!blockTiltSafetyOK("GO")) {
    return;
  }

  if ((long)(now - blockEndAtMs) >= 0) {
    finishActiveBlock("GO:" + blockGoDirection);
  }
}


// ==================================================
// RUN BLOCK: TURN
// ==================================================
void runTurnBlock(unsigned long now) {
  if (!blockTiltSafetyOK("TURN")) {
    return;
  }

  float turned = fabs(yawDeg - blockTurnStartYaw);

  if (turned >= (blockTurnTargetDeg - TURN_TOLERANCE_DEG)) {
    finishActiveBlock("TURN:" + blockTurnDirection + ":" + String(blockTurnTargetDeg, 0));
    return;
  }

  if ((long)(now - blockEndAtMs) >= 0) {
    blockError("TURN:TIMEOUT", "TARGET=" + String(blockTurnTargetDeg, 0) + ";TURNED=" + String(turned, 1));
    return;
  }
}


// ==================================================
// RUN BLOCK: JACK
// ==================================================
void runJackBlock(unsigned long now) {
  if (!blockTiltSafetyOK("JACK")) {
    return;
  }

  if ((long)(now - blockEndAtMs) >= 0) {
    finishActiveBlock("JACK:" + blockJackWhich + ":" + blockJackAction);
  }
}


// ==================================================
// RUN AUTO BLOCK: UP STAIRS - NEW DIRECT JACK LOGIC
// ==================================================
void runUpStairsBlock(unsigned long now) {
  // Hard safety only
  if (fabs(roll) >= ROLL_DANGER_DEG || fabs(pitch) >= PITCH_DANGER_DEG) {
    blockError(
      "UP_STAIRS:HARD_TILT_DANGER",
      "PITCH=" + String(pitch, 2) + ";ROLL=" + String(roll, 2)
    );
    return;
  }

  // Roll warning only, no stop
  if (fabs(roll) >= UP_ROLL_WARNING_DEG) {
    static unsigned long lastRollWarnMs = 0;
    if (now - lastRollWarnMs > 1000) {
      lastRollWarnMs = now;
      blockAck("WARN:UP_STAIRS:ROLL_HIGH_CONTINUING:ROLL=" + String(roll, 2));
    }
  }

  // Total timeout
  if (now - blockStartMs >= UP_STAIRS_MAX_TOTAL_MS) {
    blockError("UP_STAIRS:TIMEOUT", "MAX_TOTAL_MS");
    return;
  }

  // ==================================================
  // STEP 1:
  // Start moving forward with speed 40.
  // ==================================================
  if (blockStep == STEP_UP_INIT) {
    blockAck("STEP:UP_STAIRS:FORWARD_SPEED_40_WAIT_PITCH");

    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    blockStep = STEP_UP_FORWARD_CLIMB;
    blockStepStartMs = now;
    return;
  }

  // ==================================================
  // STEP 2:
  // Keep moving forward.
  // When pitch >= 3.5 deg, use rear jack immediately.
  // No no-progress condition anymore.
  // ==================================================
  if (blockStep == STEP_UP_FORWARD_CLIMB) {
    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (fabs(pitch) >= UP_PITCH_CLIMB_START_DEG) {
      if (upJackUseCount >= UP_MAX_JACK_USES) {
        blockError("UP_STAIRS:MAX_JACK_CYCLES", "COUNT=" + String(upJackUseCount));
        return;
      }

      upDetectedStepCount++;
      upJackUseCount++;
      upJackStartPitch = pitch;

      blockAck(
        "STEP:UP_STAIRS:STAIR_DETECTED_REAR_JACK_EXTEND:"
        "STEP_COUNT=" + String(upDetectedStepCount) +
        ":JACK_COUNT=" + String(upJackUseCount) +
        ":PITCH=" + String(pitch, 2)
      );

      rearJackExtend();

      blockStep = STEP_UP_REAR_JACK_EXTEND;
      blockStepStartMs = now;
      return;
    }

    return;
  }

  // ==================================================
  // STEP 3:
  // Extend rear jack until MPU detects effect
  // or until timeout.
  // ==================================================
  if (blockStep == STEP_UP_REAR_JACK_EXTEND) {
    float jackEffect = fabs(pitch - upJackStartPitch);

    if (jackEffect >= UP_JACK_EFFECT_DELTA_DEG) {
      rearJackStop();

      blockAck(
        "STEP:UP_STAIRS:JACK_EFFECT_DETECTED:"
        "DELTA=" + String(jackEffect, 2)
      );

      setSpeedPercent(UP_AFTER_JACK_SPEED_PERCENT, false);
      lastMovement = "FORWARD";
      moveForward();

      blockStep = STEP_UP_AFTER_JACK_FORWARD;
      blockStepStartMs = now;
      return;
    }

    if (now - blockStepStartMs >= UP_JACK_MAX_EXTEND_MS) {
      rearJackStop();

      blockAck("STEP:UP_STAIRS:JACK_EXTEND_TIMEOUT_FORWARD");

      setSpeedPercent(UP_AFTER_JACK_SPEED_PERCENT, false);
      lastMovement = "FORWARD";
      moveForward();

      blockStep = STEP_UP_AFTER_JACK_FORWARD;
      blockStepStartMs = now;
      return;
    }

    rearJackExtend();
    return;
  }

  // ==================================================
  // STEP 4:
  // Move forward slowly after jack lift.
  // Goal: let the robot body pass the stair edge.
  // ==================================================
  if (blockStep == STEP_UP_AFTER_JACK_FORWARD) {
    setSpeedPercent(UP_AFTER_JACK_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (now - blockStepStartMs >= UP_JACK_AFTER_EFFECT_FORWARD_MS) {
      blockAck("STEP:UP_STAIRS:STOP_MOTORS_RETRACT_REAR_JACK");

      stopMotors();
      lastMovement = "STOP";

      rearJackRetract();

      blockStep = STEP_UP_REAR_JACK_RETRACT;
      blockStepStartMs = now;
      return;
    }

    return;
  }

  // ==================================================
  // STEP 5:
  // Retract rear jack for long time.
  // Motors are stopped here so the jack does not hit stair edge.
  // ==================================================
  if (blockStep == STEP_UP_REAR_JACK_RETRACT) {
    stopMotors();
    lastMovement = "STOP";

    if (now - blockStepStartMs >= UP_JACK_RETRACT_MS) {
      rearJackStop();

      blockAck("STEP:UP_STAIRS:REAR_JACK_RETRACT_DONE_REPEAT");

      setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
      lastMovement = "FORWARD";
      moveForward();

      blockStep = STEP_UP_FORWARD_CLIMB;
      blockStepStartMs = now;
      return;
    }

    rearJackRetract();
    return;
  }
}


// ==================================================
// PULSE COMMAND HANDLER
// ==================================================
void handlePulseCommand(String command) {
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);

  if (firstColon < 0 || secondColon < 0) {
    Serial1.print("ERROR:BAD_PULSE:");
    Serial1.println(command);
    return;
  }

  String dir = command.substring(firstColon + 1, secondColon);
  String msText = command.substring(secondColon + 1);

  dir.trim();
  dir.toUpperCase();
  msText.trim();

  int durationMs = msText.toInt();
  durationMs = constrain(durationMs, 50, 1500);

  timedMoveActive = true;
  timedMoveEndAt = millis() + durationMs;

  if (dir == "FORWARD") {
    lastMovement = "FORWARD";
    moveForward();
    Serial1.print("ACK:PULSE:FORWARD:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "BACKWARD") {
    lastMovement = "BACKWARD";
    moveBackward();
    Serial1.print("ACK:PULSE:BACKWARD:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "LEFT") {
    lastMovement = "LEFT";
    turnLeft();
    Serial1.print("ACK:PULSE:LEFT:");
    Serial1.println(durationMs);
    return;
  }

  if (dir == "RIGHT") {
    lastMovement = "RIGHT";
    turnRight();
    Serial1.print("ACK:PULSE:RIGHT:");
    Serial1.println(durationMs);
    return;
  }

  timedMoveActive = false;
  stopMotors();
  lastMovement = "STOP";

  Serial1.print("ERROR:UNKNOWN_PULSE_DIR:");
  Serial1.println(dir);
}


// ==================================================
// TIMED MOVEMENT UPDATE
// ==================================================
void updateTimedMovement() {
  if (!timedMoveActive) {
    return;
  }

  if ((long)(millis() - timedMoveEndAt) >= 0) {
    timedMoveActive = false;
    stopMotors();
    lastMovement = "STOP";
    Serial1.println("ACK:PULSE:DONE");
  }
}


// ==================================================
// SENSOR REPORTING
// ==================================================
void updateAndReportSensors() {
  unsigned long now = millis();

  if (now - lastMPURead >= MPU_READ_INTERVAL) {
    lastMPURead = now;
    readMPU();
  }

  if (now - lastUltrasonicRead >= ULTRASONIC_READ_INTERVAL_MS) {
    lastUltrasonicRead = now;

    frontUltrasonicCM = readUltrasonicCM(FRONT_US_TRIG, FRONT_US_ECHO);
    rearUltrasonicCM = readUltrasonicCM(REAR_US_TRIG, REAR_US_ECHO);
  }

  String alert = detectAlert();
  bool alertActive = (alert != "NONE");

  if (alertActive && (now - lastAlertSent >= ALERT_COOLDOWN_MS)) {
    lastAlertSent = now;
    lastPeriodicReport = now;
    sendSensorData(alert);
    return;
  }

  unsigned long reportInterval;

  if (systemMode == "AUTO") {
    reportInterval = AUTO_SENSOR_REPORT_INTERVAL;
  } else {
    reportInterval = MANUAL_SENSOR_REPORT_INTERVAL;
  }

  if (now - lastPeriodicReport >= reportInterval) {
    lastPeriodicReport = now;
    sendSensorData("NONE");
  }
}


// ==================================================
// ALERT DETECTION
// ==================================================
String detectAlert() {
  if (fabs(pitch) > PITCH_DANGER_DEG) {
    return "TILT_DANGER";
  }

  if (fabs(roll) > ROLL_DANGER_DEG) {
    return "TILT_DANGER";
  }

  return "NONE";
}


// ==================================================
// SENSOR OUTPUT TO RASPBERRY PI
// ==================================================
void sendSensorData(String alertType) {
  Serial.print("SENSOR:");
  Serial.print("SYS=");
  Serial.print(systemMode);

  Serial.print(";MODE=");
  Serial.print(currentMode);

  Serial.print(";MOVE=");
  Serial.print(lastMovement);

  Serial.print(";SPEED=");
  Serial.print(motorSpeedPercent);

  Serial.print(";PITCH=");
  Serial.print(pitch, 2);

  Serial.print(";ROLL=");
  Serial.print(roll, 2);

  Serial.print(";YAW=");
  Serial.print(yawDeg, 2);

  Serial.print(";UF=");
  Serial.print(frontUltrasonicCM, 2);

  Serial.print(";UR=");
  Serial.print(rearUltrasonicCM, 2);

  Serial.print(";BLOCK=");
  Serial.print(activeBlockName);

  Serial.print(";ALERT=");
  Serial.println(alertType);
}

void sendStatusToPI() {
  Serial.print("STATUS:");
  Serial.print("SYS=");
  Serial.print(systemMode);

  Serial.print(";MODE=");
  Serial.print(currentMode);

  Serial.print(";MOVE=");
  Serial.print(lastMovement);

  Serial.print(";SPEED=");
  Serial.print(motorSpeedPercent);

  Serial.print(";PITCH=");
  Serial.print(pitch, 2);

  Serial.print(";ROLL=");
  Serial.print(roll, 2);

  Serial.print(";YAW=");
  Serial.print(yawDeg, 2);

  Serial.print(";UF=");
  Serial.print(frontUltrasonicCM, 2);

  Serial.print(";UR=");
  Serial.print(rearUltrasonicCM, 2);

  Serial.print(";BLOCK=");
  Serial.println(activeBlockName);
}


// ==================================================
// MOTOR FUNCTIONS
// ==================================================
void setSpeedPercent(int spd, bool applyCurrentMovement) {
  spd = constrain(spd, 0, 100);

  motorSpeedPercent = spd;
  motorSpeed = map(spd, 0, 100, 0, 255);

  if (applyCurrentMovement) {
    applyLastMovement();
  }
}

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
// Display only
// ==================================================
float readUltrasonicCM(int trigPin, int echoPin) {
  if (!ULTRASONIC_ENABLED) {
    return -1.0;
  }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 12000);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}


// ==================================================
// MPU6500
// ==================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  byte err = Wire.endTransmission();

  if (err == 0) {
    Serial.println("MPU:OK");
  } else {
    Serial.println("MPU:ERROR");
  }
}

bool readRawGyroZ(int16_t &rawGz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, 2, true);

  if (Wire.available() < 2) {
    return false;
  }

  rawGz = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrateGyroZ() {
  long sum = 0;
  int count = 0;

  Serial.println("MPU:GYRO_Z_CALIBRATING_KEEP_ROBOT_STILL");

  for (int i = 0; i < 250; i++) {
    int16_t rawGz = 0;

    if (readRawGyroZ(rawGz)) {
      sum += rawGz;
      count++;
    }

    delay(3);
  }

  if (count > 0) {
    gyroZBias = (sum / (float)count) / 131.0;
  } else {
    gyroZBias = 0.0;
  }

  yawDeg = 0.0;
  lastMPUIntegrationMicros = micros();

  Serial.print("MPU:GYRO_Z_BIAS=");
  Serial.println(gyroZBias, 4);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() < 14) {
    return;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  accelX = rawAx / 16384.0;
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  gyroX = rawGx / 131.0;
  gyroY = rawGy / 131.0;
  gyroZ = (rawGz / 131.0) - gyroZBias;

  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  roll  = atan2(-accelX, accelZ) * 180.0 / PI;

  unsigned long nowMicros = micros();

  if (lastMPUIntegrationMicros == 0) {
    lastMPUIntegrationMicros = nowMicros;
    return;
  }

  float dt = (nowMicros - lastMPUIntegrationMicros) / 1000000.0;
  lastMPUIntegrationMicros = nowMicros;

  if (fabs(gyroZ) > 0.45 && dt > 0 && dt < 0.2) {
    yawDeg += gyroZ * dt;
  }
}