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
// 3-stair scenario based on the measured real setup.
// Before first stair, distance is variable, so move forward until MPU
// detects the tested first-stair pitch angle.
// IMPORTANT: first stair values are user-tested and must not be changed.
// Stair 1: trigger at tested angle, rear jack down 14000 ms, forward 3500 ms, jack up.
// Stair 2: use a higher dedicated MPU angle, rear jack down 12000 ms, forward 2000 ms, jack up.
// Stair 3: use a higher dedicated MPU angle, rear jack down 20000 ms, forward 3000 ms, jack up.
// After last stair: forward 2s, turn 90 degrees, stop.
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
// SMART MPU SAFETY FILTER
// ==================================================
// Old behavior: one bad MPU sample could immediately stop the active block.
// New behavior: send warning first, then stop only if danger remains continuous.
// Very extreme tilt still stops immediately.
const unsigned long MPU_TILT_STOP_HOLD_MS       = 1500;
const unsigned long MPU_TILT_WARN_REPEAT_MS     = 700;
const float MPU_INSTANT_STOP_MARGIN_DEG         = 18.0;

// Ignore impossible single-sample MPU spikes.
const float MPU_MAX_VALID_ANGLE_DEG             = 85.0;
const float MPU_MAX_SINGLE_SAMPLE_JUMP_DEG      = 35.0;
const unsigned long MPU_SPIKE_WARN_REPEAT_MS    = 1000;

bool mpuDangerCandidateActive = false;
unsigned long mpuDangerCandidateStartMs = 0;
unsigned long mpuLastDangerWarnMs = 0;
String mpuDangerContext = "NONE";

bool mpuHaveValidAngles = false;
unsigned long lastMpuSpikeWarnMs = 0;


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
// UP STAIRS SETTINGS - 3 STAIRS SCENARIO
// ==================================================
// Stair geometry given by user:
// - 3 stairs
// - One tread depth = 18 cm
// - One riser height = 8 cm
// - Robot length = 44.5 cm
// - Robot height = 17 cm
//
// Jack timing measured by user:
// - Full jack down/extend travel is about 20 seconds.
//
// Important naming:
// - rearJackExtend()  = jack goes DOWN / extends
// - rearJackRetract() = jack goes UP / retracts

const int UP_STAIRS_SPEED_PERCENT = 40;
const int UP_AFTER_JACK_SPEED_PERCENT = 40;
const int UP_FINAL_TURN_SPEED_PERCENT = 35;

const unsigned long UP_STAIRS_MAX_TOTAL_MS = 180000;

// The distance before the first stair is variable.
// So the robot keeps moving until MPU detects the tested first-stair angle.
// IMPORTANT: these first-stair values came from real testing and are kept exactly.
const float UP_FIRST_STAIR_TRIGGER_ABS_DEG = 30.0;
const float UP_FIRST_STAIR_TRIGGER_DELTA_DEG = 35.0;

// Dedicated MPU triggers for the next stairs.
// The first stair is already good, so only stair 2 and stair 3 are made higher.
// Tune only these two pairs if future tests show early/late jack timing.
const float UP_SECOND_STAIR_TRIGGER_ABS_DEG = 34.0;
const float UP_SECOND_STAIR_TRIGGER_DELTA_DEG = 38.0;

const float UP_THIRD_STAIR_TRIGGER_ABS_DEG = 38.0;
const float UP_THIRD_STAIR_TRIGGER_DELTA_DEG = 42.0;

// After the first stair support cycle, the robot should keep moving until
// it becomes stable on stair 1 and stair 2, then do the second jack cycle.
const float UP_STABLE_PITCH_DEG = 10.0;
const float UP_STABLE_ROLL_DEG  = 12.0;
const unsigned long UP_STABLE_HOLD_MS = 700;
const unsigned long UP_MIN_MOVE_BEFORE_STABLE_MS = 900;

// Timeouts keep the robot from getting stuck forever if the MPU condition
// is not detected perfectly during testing.
const unsigned long UP_APPROACH_FIRST_TIMEOUT_MS = 60000;
const unsigned long UP_WAIT_STABLE_TIMEOUT_MS    = 14000;
const unsigned long UP_WAIT_FINAL_TILT_TIMEOUT_MS = 16000;

// Jack travel schedule for the 3 stairs.
const unsigned long UP_JACK_FULL_TRAVEL_MS = 20000;
const unsigned long UP_STEP1_JACK_EXTEND_MS = 14000;  // about 15 s
const unsigned long UP_STEP2_JACK_EXTEND_MS = 12000;  // about 13.3 s
const unsigned long UP_STEP3_JACK_EXTEND_MS = 20000;  // about 20 s

// Retract duration is matched to how far the jack was extended, with a
// small safety margin to make sure it is high enough before driving again.
const unsigned long UP_JACK_RETRACT_EXTRA_MS = 700;

// Forward after jack support.
const unsigned long UP_STEP1_FORWARD_AFTER_JACK_MS = 3500;
const unsigned long UP_STEP2_FORWARD_AFTER_JACK_MS = 2500;
const unsigned long UP_STEP3_FORWARD_AFTER_JACK_MS = 700;

// After the last stair: move forward 2 seconds, turn 90 degrees, then stop.
const unsigned long UP_FINAL_FORWARD_MS = 2000;
const float UP_FINAL_TURN_DEG = 90.0;
const unsigned long UP_FINAL_TURN_TIMEOUT_MS = 9000;
const char UP_FINAL_TURN_DIRECTION = 'R';  // Change to 'L' if you want left turn.

// During stair climbing pitch can be high, so hard danger must be higher
// than the normal manual-mode safety threshold.
const float UP_PITCH_HARD_DANGER_DEG = 50.0;
const float UP_ROLL_HARD_DANGER_DEG  = 50.0;

// Roll warning only, no stop.
const float UP_ROLL_WARNING_DEG = 35.0;


// ==================================================
// DOWN STAIRS SETTINGS - FRONT JACK + REAR SUPPORT DESCENT LOGIC
// ==================================================
// IMPORTANT:
// UP_STAIRS logic is not changed.
// This DOWN_STAIRS block mainly uses the FRONT jack,
// with one added rear-jack support cycle after the 1.5s forward step.
// The previous descent angle constants are kept unchanged.
// The new sequence is a more accurate timed descent sequence,
// with one MPU level check near the end.

const bool DOWN_MOVE_FORWARD_DURING_DESCENT = true;

const int DOWN_STAIRS_SPEED_PERCENT = 35;
const int DOWN_AFTER_JACK_SPEED_PERCENT = 35;

const unsigned long DOWN_STAIRS_MAX_TOTAL_MS = 180000;

// Descent angles: kept exactly as the previous configured values.
// They remain available for testing/logging; this new sequence mainly uses
// the final level check between 0 and 10 degrees.
const float DOWN_FIRST_STAIR_TRIGGER_ABS_DEG = 34.0;
const float DOWN_FIRST_STAIR_TRIGGER_DELTA_DEG = 38.0;

const float DOWN_SECOND_STAIR_TRIGGER_ABS_DEG = 38.0;
const float DOWN_SECOND_STAIR_TRIGGER_DELTA_DEG = 42.0;

const float DOWN_THIRD_STAIR_TRIGGER_ABS_DEG = 42.0;
const float DOWN_THIRD_STAIR_TRIGGER_DELTA_DEG = 46.0;

// Precise descent sequence requested by user.
// Naming:
// - frontJackExtend()  = front jack goes DOWN / extends
// - frontJackRetract() = front jack goes UP / retracts
const unsigned long DOWN_INITIAL_FORWARD_MS = 2000;

const unsigned long DOWN_FRONT_EXTEND_12S_MS = 12000;
const unsigned long DOWN_FORWARD_AFTER_EXTEND_12S_MS = 1000;

const unsigned long DOWN_FRONT_EXTEND_3S_MS = 8000;
const unsigned long DOWN_FORWARD_AFTER_EXTEND_3S_MS = 500;

// Added rear-jack support cycle after the 1.5s forward step.
const unsigned long DOWN_REAR_EXTEND_AFTER_FRONT_3S_MS = 20000;
const unsigned long DOWN_FORWARD_AFTER_REAR_EXTEND_MS = 800;
const unsigned long DOWN_REAR_RETRACT_AFTER_FORWARD_MS = 20000;

const unsigned long DOWN_FRONT_RETRACT_5S_A_MS = 5000;
const unsigned long DOWN_FORWARD_AFTER_RETRACT_5S_A_MS = 1000;

const unsigned long DOWN_FRONT_RETRACT_10S_B_MS = 10000;
const unsigned long DOWN_FORWARD_AFTER_RETRACT_10S_B_MS = 1000;

const unsigned long DOWN_FRONT_EXTEND_5S_A_MS = 0;
const unsigned long DOWN_FORWARD_AFTER_EXTEND_5S_A_MS = 2000;

const unsigned long DOWN_FRONT_RETRACT_2S_A_MS = 4000;
const unsigned long DOWN_FORWARD_AFTER_RETRACT_2S_A_MS = 2000;

const unsigned long DOWN_FRONT_RETRACT_3S_A_MS = 8000;

// After this forward, check that the pitch is approximately level.
// Because descent pitch may be positive or negative depending on MPU mounting,
// we use abs(pitch) <= 10.
const unsigned long DOWN_FORWARD_BEFORE_LEVEL_CHECK_MS = 5000;
const float DOWN_LEVEL_PITCH_MAX_DEG = 20.0;

// If pitch is not within 0..10 degrees, keep moving forward 6 seconds
// and check again until the robot becomes approximately level or total timeout.
const unsigned long DOWN_LEVEL_RETRY_FORWARD_MS = 6000;

// Final descent-support sequence after the level check succeeds.
const unsigned long DOWN_FRONT_EXTEND_5S_B_MS = 5000;
const unsigned long DOWN_FORWARD_AFTER_EXTEND_5S_B_MS = 2000;

const unsigned long DOWN_FRONT_RETRACT_2S_B_MS = 2000;
const unsigned long DOWN_FORWARD_AFTER_RETRACT_2S_B_MS = 2000;

const unsigned long DOWN_FRONT_RETRACT_3S_B_MS = 3000;

const float DOWN_PITCH_HARD_DANGER_DEG = 55.0;
const float DOWN_ROLL_HARD_DANGER_DEG  = 50.0;
const float DOWN_ROLL_WARNING_DEG = 35.0;


// ==================================================
// LEGO BLOCK STATE MACHINE
// ==================================================
enum MegaBlockType {
  BLOCK_NONE,
  BLOCK_GO,
  BLOCK_TURN,
  BLOCK_JACK,
  BLOCK_UP_STAIRS,
  BLOCK_DOWN_STAIRS
};

enum MegaBlockStep {
  STEP_IDLE,

  STEP_GO_RUNNING,

  STEP_TURN_RUNNING,

  STEP_JACK_RUNNING,

  STEP_UP_INIT,
  STEP_UP_APPROACH_FIRST_TILT,
  STEP_UP_WAIT_SECOND_TILT,
  STEP_UP_WAIT_THIRD_TILT,
  STEP_UP_REAR_JACK_EXTEND,
  STEP_UP_AFTER_JACK_FORWARD,
  STEP_UP_REAR_JACK_RETRACT,
  STEP_UP_FINAL_FORWARD,
  STEP_UP_FINAL_TURN,

  STEP_DOWN_INIT,
  STEP_DOWN_INITIAL_FORWARD,

  STEP_DOWN_FRONT_EXTEND_12S,
  STEP_DOWN_FORWARD_AFTER_EXTEND_12S,

  STEP_DOWN_FRONT_EXTEND_3S,
  STEP_DOWN_FORWARD_AFTER_EXTEND_3S,

  STEP_DOWN_REAR_EXTEND_20S,
  STEP_DOWN_FORWARD_AFTER_REAR_EXTEND_1S,
  STEP_DOWN_REAR_RETRACT_20S,

  STEP_DOWN_FRONT_RETRACT_5S_A,
  STEP_DOWN_FORWARD_AFTER_RETRACT_5S_A,

  STEP_DOWN_FRONT_RETRACT_5S_B,
  STEP_DOWN_FORWARD_AFTER_RETRACT_5S_B,

  STEP_DOWN_FRONT_EXTEND_5S_A,
  STEP_DOWN_FORWARD_AFTER_EXTEND_5S_A,

  STEP_DOWN_FRONT_RETRACT_2S_A,
  STEP_DOWN_FORWARD_AFTER_RETRACT_2S_A,

  STEP_DOWN_FRONT_RETRACT_3S_A,

  STEP_DOWN_FORWARD_BEFORE_LEVEL_CHECK,
  STEP_DOWN_LEVEL_CHECK,
  STEP_DOWN_LEVEL_RETRY_FORWARD,

  STEP_DOWN_FRONT_EXTEND_5S_B,
  STEP_DOWN_FORWARD_AFTER_EXTEND_5S_B,

  STEP_DOWN_FRONT_RETRACT_2S_B,
  STEP_DOWN_FORWARD_AFTER_RETRACT_2S_B,

  STEP_DOWN_FRONT_RETRACT_3S_B
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
float upStartPitch = 0.0;
float upStepStartPitch = 0.0;
float upTurnStartYaw = 0.0;
unsigned long upStableStartMs = 0;
unsigned long upCurrentJackExtendMs = 0;
unsigned long upCurrentJackRetractMs = 0;
unsigned long upCurrentForwardAfterJackMs = 0;
int upCurrentStair = 0;  // 1, 2, 3

// DOWN STAIRS
float downStartPitch = 0.0;
float downStepStartPitch = 0.0;


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
  Serial.println("MEGA:AUTO_UP_STAIRS_3_STAIRS_SCENARIO_ENABLED");
  Serial.println("MEGA:UP_STAIRS_WAIT_FIRST_MPU_TILT_THEN_3_4_2_3_FULL_JACK");
  Serial.println("MEGA:UP_STAIRS_PER_STAIR_MPU_TRIGGERS_FIRST_30_SECOND_34_THIRD_38");
  Serial.println("MEGA:UP_STAIRS_FINAL_FORWARD_2S_TURN_90_STOP");
  Serial.println("MEGA:AUTO_DOWN_STAIRS_FRONT_JACK_WITH_REAR_SUPPORT_SEQUENCE_ENABLED");
  Serial.println("MEGA:DOWN_STAIRS_FRONT_JACK_REAR_SUPPORT_LEVEL_CHECK_0_TO_10_DEG");
  Serial.println("MEGA:HARD_TILT_ONLY_ENABLED");

  Serial1.println("MEGA:READY");
  Serial1.println("MEGA:LEGO_BLOCKS_ENABLED");
  Serial1.println("MEGA:AUTO_UP_STAIRS_3_STAIRS_SCENARIO_ENABLED");
  Serial1.println("MEGA:UP_STAIRS_WAIT_FIRST_MPU_TILT_THEN_3_4_2_3_FULL_JACK");
  Serial1.println("MEGA:UP_STAIRS_PER_STAIR_MPU_TRIGGERS_FIRST_30_SECOND_34_THIRD_38");
  Serial1.println("MEGA:AUTO_DOWN_STAIRS_FRONT_JACK_WITH_REAR_SUPPORT_SEQUENCE_ENABLED");
  Serial1.println("MEGA:DOWN_STAIRS_FRONT_JACK_REAR_SUPPORT_LEVEL_CHECK_0_TO_10_DEG");
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
    startDownStairsBlock();
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

void resetMpuDangerConfirmation() {
  mpuDangerCandidateActive = false;
  mpuDangerCandidateStartMs = 0;
  mpuLastDangerWarnMs = 0;
  mpuDangerContext = "NONE";
}

void blockError(String code, String details) {
  resetMpuDangerConfirmation();

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

  resetMpuDangerConfirmation();

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
  resetMpuDangerConfirmation();

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
bool smartTiltSafetyOK(String blockName, float pitchLimit, float rollLimit, bool allowInstantStop) {
  unsigned long now = millis();

  bool pitchDanger = fabs(pitch) >= pitchLimit;
  bool rollDanger  = fabs(roll)  >= rollLimit;

  if (!pitchDanger && !rollDanger) {
    if (mpuDangerCandidateActive) {
      blockAck("WARN:" + blockName + ":TILT_BACK_TO_SAFE:PITCH=" + String(pitch, 2) + ":ROLL=" + String(roll, 2));
    }

    resetMpuDangerConfirmation();
    return true;
  }

  float pitchOver = fabs(pitch) - pitchLimit;
  float rollOver  = fabs(roll)  - rollLimit;
  float worstOver = max(pitchOver, rollOver);

  // Very extreme tilt is treated as a true emergency, not just an MPU glitch.
  if (allowInstantStop && worstOver >= MPU_INSTANT_STOP_MARGIN_DEG) {
    blockError(
      blockName + ":INSTANT_EXTREME_TILT_DANGER",
      "PITCH=" + String(pitch, 2) +
      ";ROLL=" + String(roll, 2) +
      ";PITCH_LIMIT=" + String(pitchLimit, 1) +
      ";ROLL_LIMIT=" + String(rollLimit, 1)
    );
    return false;
  }

  if (!mpuDangerCandidateActive || mpuDangerContext != blockName) {
    mpuDangerCandidateActive = true;
    mpuDangerCandidateStartMs = now;
    mpuLastDangerWarnMs = now;
    mpuDangerContext = blockName;

    blockAck(
      "WARN:" + blockName +
      ":TILT_DANGER_CANDIDATE_CONTINUING:"
      "PITCH=" + String(pitch, 2) +
      ":ROLL=" + String(roll, 2) +
      ":STOP_AFTER_MS=" + String(MPU_TILT_STOP_HOLD_MS)
    );

    return true;
  }

  unsigned long dangerAgeMs = now - mpuDangerCandidateStartMs;

  if (dangerAgeMs >= MPU_TILT_STOP_HOLD_MS) {
    blockError(
      blockName + ":CONFIRMED_TILT_DANGER",
      "PITCH=" + String(pitch, 2) +
      ";ROLL=" + String(roll, 2) +
      ";DURATION_MS=" + String(dangerAgeMs) +
      ";PITCH_LIMIT=" + String(pitchLimit, 1) +
      ";ROLL_LIMIT=" + String(rollLimit, 1)
    );
    return false;
  }

  if (now - mpuLastDangerWarnMs >= MPU_TILT_WARN_REPEAT_MS) {
    mpuLastDangerWarnMs = now;
    blockAck(
      "WARN:" + blockName +
      ":TILT_DANGER_STILL_CONFIRMING:"
      "PITCH=" + String(pitch, 2) +
      ":ROLL=" + String(roll, 2) +
      ":AGE_MS=" + String(dangerAgeMs) +
      ":STOP_AFTER_MS=" + String(MPU_TILT_STOP_HOLD_MS)
    );
  }

  // Warning only for now; continue. If the danger persists, the next calls stop it.
  return true;
}

bool blockTiltSafetyOK(String blockName) {
  return smartTiltSafetyOK(blockName, PITCH_DANGER_DEG, ROLL_DANGER_DEG, true);
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
  resetMpuDangerConfirmation();

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
  resetMpuDangerConfirmation();

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
  resetMpuDangerConfirmation();

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
  resetMpuDangerConfirmation();

  saveCurrentSpeed();

  activeBlock = BLOCK_UP_STAIRS;
  blockStep = STEP_UP_INIT;
  activeBlockName = "UP_STAIRS";

  systemMode = "AUTO";
  currentMode = "CLIMB";

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;

  upStartPitch = pitch;
  upStepStartPitch = pitch;
  upTurnStartYaw = yawDeg;
  upStableStartMs = 0;
  upCurrentJackExtendMs = 0;
  upCurrentJackRetractMs = 0;
  upCurrentForwardAfterJackMs = 0;
  upCurrentStair = 0;

  timedMoveActive = false;

  setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);

  blockAck("START:UP_STAIRS:3_STAIRS:WAIT_FIRST_MPU_TILT:SPEED_40");
}


// ==================================================
// AUTO BLOCK: DOWN STAIRS
// ==================================================
void startDownStairsBlock() {
  stopActiveBlock("NEW_DOWN_STAIRS");
  resetMpuDangerConfirmation();

  saveCurrentSpeed();

  activeBlock = BLOCK_DOWN_STAIRS;
  blockStep = STEP_DOWN_INIT;
  activeBlockName = "DOWN_STAIRS";

  systemMode = "AUTO";
  currentMode = "CLIMB";

  blockStartMs = millis();
  blockStepStartMs = blockStartMs;

  downStartPitch = pitch;
  downStepStartPitch = pitch;

  timedMoveActive = false;

  setSpeedPercent(DOWN_STAIRS_SPEED_PERCENT, false);

  blockAck("START:DOWN_STAIRS:FRONT_JACK_WITH_REAR_SUPPORT:INITIAL_FORWARD_2S:MPU_ANGLES_UNCHANGED");
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

  if (activeBlock == BLOCK_DOWN_STAIRS) {
    runDownStairsBlock(now);
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
// RUN AUTO BLOCK: UP STAIRS - 3 STAIRS SCENARIO
// ==================================================
unsigned long getUpJackExtendMs(int stairNumber) {
  if (stairNumber == 1) {
    return UP_STEP1_JACK_EXTEND_MS;
  }

  if (stairNumber == 2) {
    return UP_STEP2_JACK_EXTEND_MS;
  }

  return UP_STEP3_JACK_EXTEND_MS;
}

unsigned long getUpForwardAfterJackMs(int stairNumber) {
  if (stairNumber == 1) {
    return UP_STEP1_FORWARD_AFTER_JACK_MS;
  }

  if (stairNumber == 2) {
    return UP_STEP2_FORWARD_AFTER_JACK_MS;
  }

  return UP_STEP3_FORWARD_AFTER_JACK_MS;
}

bool upTiltDetected(float absTriggerDeg, float deltaTriggerDeg, float referencePitchDeg) {
  float absPitch = fabs(pitch);
  float deltaPitch = fabs(pitch - referencePitchDeg);

  return (absPitch >= absTriggerDeg || deltaPitch >= deltaTriggerDeg);
}

bool upFirstTiltDetected() {
  return upTiltDetected(
    UP_FIRST_STAIR_TRIGGER_ABS_DEG,
    UP_FIRST_STAIR_TRIGGER_DELTA_DEG,
    upStartPitch
  );
}

bool upSecondTiltDetected() {
  return upTiltDetected(
    UP_SECOND_STAIR_TRIGGER_ABS_DEG,
    UP_SECOND_STAIR_TRIGGER_DELTA_DEG,
    upStepStartPitch
  );
}

bool upThirdTiltDetected() {
  return upTiltDetected(
    UP_THIRD_STAIR_TRIGGER_ABS_DEG,
    UP_THIRD_STAIR_TRIGGER_DELTA_DEG,
    upStepStartPitch
  );
}

bool upRobotStable() {
  return (fabs(pitch) <= UP_STABLE_PITCH_DEG && fabs(roll) <= UP_STABLE_ROLL_DEG);
}

void startUpJackCycle(int stairNumber, unsigned long now, String reason) {
  upCurrentStair = constrain(stairNumber, 1, 3);
  upCurrentJackExtendMs = getUpJackExtendMs(upCurrentStair);
  upCurrentJackRetractMs = upCurrentJackExtendMs + UP_JACK_RETRACT_EXTRA_MS;
  upCurrentForwardAfterJackMs = getUpForwardAfterJackMs(upCurrentStair);
  upStepStartPitch = pitch;

  stopMotors();
  lastMovement = "STOP";

  rearJackExtend();

  blockStep = STEP_UP_REAR_JACK_EXTEND;
  blockStepStartMs = now;

  blockAck(
    "STEP:UP_STAIRS:STAIR_" + String(upCurrentStair) +
    ":REAR_JACK_DOWN_START:" + reason +
    ":EXTEND_MS=" + String(upCurrentJackExtendMs) +
    ":PITCH=" + String(pitch, 2) +
    ":ROLL=" + String(roll, 2)
  );
}

void runUpStairsBlock(unsigned long now) {
  // Stair climbing can naturally create a high pitch, so use climbing-specific
  // hard limits. The smart checker warns first and stops only if danger persists.
  if (!smartTiltSafetyOK("UP_STAIRS", UP_PITCH_HARD_DANGER_DEG, UP_ROLL_HARD_DANGER_DEG, true)) {
    return;
  }

  // Roll warning only, no stop.
  if (fabs(roll) >= UP_ROLL_WARNING_DEG) {
    static unsigned long lastRollWarnMs = 0;
    if (now - lastRollWarnMs > 1000) {
      lastRollWarnMs = now;
      blockAck("WARN:UP_STAIRS:ROLL_HIGH_CONTINUING:ROLL=" + String(roll, 2));
    }
  }

  if (now - blockStartMs >= UP_STAIRS_MAX_TOTAL_MS) {
    blockError("UP_STAIRS:TIMEOUT", "MAX_TOTAL_MS");
    return;
  }

  // ==================================================
  // STEP 0:
  // Start approach. Distance before first stair is variable.
  // The robot moves until MPU detects strong pitch change.
  // ==================================================
  if (blockStep == STEP_UP_INIT) {
    blockAck(
      "STEP:UP_STAIRS:APPROACH_FIRST_STAIR_WAIT_MPU_TILT:"
      "STEP1_ABS=" + String(UP_FIRST_STAIR_TRIGGER_ABS_DEG, 1) +
      ":STEP1_DELTA=" + String(UP_FIRST_STAIR_TRIGGER_DELTA_DEG, 1) +
      ":STEP2_ABS=" + String(UP_SECOND_STAIR_TRIGGER_ABS_DEG, 1) +
      ":STEP3_ABS=" + String(UP_THIRD_STAIR_TRIGGER_ABS_DEG, 1) +
      ":START_PITCH=" + String(upStartPitch, 2)
    );

    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    blockStep = STEP_UP_APPROACH_FIRST_TILT;
    blockStepStartMs = now;
    return;
  }

  // ==================================================
  // STEP 1:
  // Keep moving until the real stair-climb angle is detected.
  // This means the robot is not only touching the first stair edge;
  // it is already climbing enough to be supported between stair levels.
  // ==================================================
  if (blockStep == STEP_UP_APPROACH_FIRST_TILT) {
    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (upFirstTiltDetected()) {
      startUpJackCycle(1, now, "FIRST_MPU_TILT_DETECTED");
      return;
    }

    if (now - blockStepStartMs >= UP_APPROACH_FIRST_TIMEOUT_MS) {
      blockError(
        "UP_STAIRS:FIRST_TILT_NOT_DETECTED",
        "PITCH=" + String(pitch, 2) + ";ROLL=" + String(roll, 2)
      );
      return;
    }

    return;
  }

  // ==================================================
  // STEP 2:
  // After stair 1 cycle, move forward until the dedicated
  // second-stair MPU angle is detected.
  // Then do stair 2 jack cycle using the user's tested times.
  // ==================================================
  if (blockStep == STEP_UP_WAIT_SECOND_TILT) {
    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (upSecondTiltDetected()) {
      startUpJackCycle(2, now, "SECOND_MPU_TILT_DETECTED");
      return;
    }

    // Fallback: if the robot does not show the exact second angle, continue
    // with the known 3-stair scenario instead of getting stuck forever.
    if (now - blockStepStartMs >= UP_WAIT_STABLE_TIMEOUT_MS) {
      blockAck(
        "WARN:UP_STAIRS:SECOND_TILT_TIMEOUT_STARTING_STAIR_2:"
        "PITCH=" + String(pitch, 2) +
        ":ROLL=" + String(roll, 2) +
        ":STEP2_ABS_TRIGGER=" + String(UP_SECOND_STAIR_TRIGGER_ABS_DEG, 1) +
        ":STEP2_DELTA_TRIGGER=" + String(UP_SECOND_STAIR_TRIGGER_DELTA_DEG, 1)
      );
      startUpJackCycle(2, now, "SECOND_TILT_TIMEOUT_FALLBACK");
      return;
    }

    return;
  }

  // ==================================================
  // STEP 3:
  // After stair 2 cycle, move forward until the dedicated
  // third-stair MPU angle is detected.
  // Then do stair 3 jack cycle using the user's tested times.
  // ==================================================
  if (blockStep == STEP_UP_WAIT_THIRD_TILT) {
    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (upThirdTiltDetected()) {
      startUpJackCycle(3, now, "THIRD_MPU_TILT_DETECTED");
      return;
    }

    if (now - blockStepStartMs >= UP_WAIT_FINAL_TILT_TIMEOUT_MS) {
      blockAck(
        "WARN:UP_STAIRS:THIRD_TILT_TIMEOUT_STARTING_STAIR_3:"
        "PITCH=" + String(pitch, 2) +
        ":ROLL=" + String(roll, 2) +
        ":STEP3_ABS_TRIGGER=" + String(UP_THIRD_STAIR_TRIGGER_ABS_DEG, 1) +
        ":STEP3_DELTA_TRIGGER=" + String(UP_THIRD_STAIR_TRIGGER_DELTA_DEG, 1)
      );
      startUpJackCycle(3, now, "THIRD_TILT_TIMEOUT_FALLBACK");
      return;
    }

    return;
  }

  // ==================================================
  // JACK DOWN / EXTEND:
  // Uses the user's tested values:
  // Stair 1 = 14000 ms, Stair 2 = 12000 ms, Stair 3 = 20000 ms.
  // Motors are stopped while the jack is moving down.
  // ==================================================
  if (blockStep == STEP_UP_REAR_JACK_EXTEND) {
    stopMotors();
    lastMovement = "STOP";

    if (now - blockStepStartMs >= upCurrentJackExtendMs) {
      rearJackStop();

      blockAck(
        "STEP:UP_STAIRS:STAIR_" + String(upCurrentStair) +
        ":REAR_JACK_DOWN_DONE:FORWARD_AFTER_JACK_MS=" +
        String(upCurrentForwardAfterJackMs)
      );

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
  // FORWARD AFTER JACK:
  // Uses the user's tested values:
  // Stair 1 = 3500 ms, Stair 2 = 2000 ms, Stair 3 = 3000 ms.
  // ==================================================
  if (blockStep == STEP_UP_AFTER_JACK_FORWARD) {
    setSpeedPercent(UP_AFTER_JACK_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (now - blockStepStartMs >= upCurrentForwardAfterJackMs) {
      blockAck(
        "STEP:UP_STAIRS:STAIR_" + String(upCurrentStair) +
        ":FORWARD_AFTER_JACK_DONE:RETRACT_MS=" +
        String(upCurrentJackRetractMs)
      );

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
  // JACK UP / RETRACT:
  // Jack must go up after forward time so the robot can move correctly.
  // ==================================================
  if (blockStep == STEP_UP_REAR_JACK_RETRACT) {
    stopMotors();
    lastMovement = "STOP";

    if (now - blockStepStartMs >= upCurrentJackRetractMs) {
      rearJackStop();

      blockAck(
        "STEP:UP_STAIRS:STAIR_" + String(upCurrentStair) +
        ":REAR_JACK_UP_DONE"
      );

      if (upCurrentStair == 1) {
        upStableStartMs = 0;
        upStepStartPitch = pitch;
        blockStep = STEP_UP_WAIT_SECOND_TILT;
        blockStepStartMs = now;
        blockAck(
          "STEP:UP_STAIRS:MOVE_UNTIL_SECOND_STAIR_MPU_TILT:"
          "STEP2_ABS_TRIGGER=" + String(UP_SECOND_STAIR_TRIGGER_ABS_DEG, 1) +
          ":STEP2_DELTA_TRIGGER=" + String(UP_SECOND_STAIR_TRIGGER_DELTA_DEG, 1) +
          ":REFERENCE_PITCH=" + String(upStepStartPitch, 2)
        );

        setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
        lastMovement = "FORWARD";
        moveForward();
        return;
      }

      if (upCurrentStair == 2) {
        upStepStartPitch = pitch;
        blockStep = STEP_UP_WAIT_THIRD_TILT;
        blockStepStartMs = now;
        blockAck(
          "STEP:UP_STAIRS:MOVE_UNTIL_THIRD_STAIR_MPU_TILT:"
          "STEP3_ABS_TRIGGER=" + String(UP_THIRD_STAIR_TRIGGER_ABS_DEG, 1) +
          ":STEP3_DELTA_TRIGGER=" + String(UP_THIRD_STAIR_TRIGGER_DELTA_DEG, 1) +
          ":REFERENCE_PITCH=" + String(upStepStartPitch, 2)
        );

        setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
        lastMovement = "FORWARD";
        moveForward();
        return;
      }

      // Stair 3 finished. Move forward 2 seconds before turning.
      blockStep = STEP_UP_FINAL_FORWARD;
      blockStepStartMs = now;
      blockAck("STEP:UP_STAIRS:FINAL_STAIR_DONE_FORWARD_2S");

      setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
      lastMovement = "FORWARD";
      moveForward();
      return;
    }

    rearJackRetract();
    return;
  }

  // ==================================================
  // FINAL FORWARD 2 SECONDS
  // ==================================================
  if (blockStep == STEP_UP_FINAL_FORWARD) {
    setSpeedPercent(UP_STAIRS_SPEED_PERCENT, false);
    lastMovement = "FORWARD";
    moveForward();

    if (now - blockStepStartMs >= UP_FINAL_FORWARD_MS) {
      stopMotors();
      lastMovement = "STOP";

      upTurnStartYaw = yawDeg;
      setSpeedPercent(UP_FINAL_TURN_SPEED_PERCENT, false);

      if (UP_FINAL_TURN_DIRECTION == 'L') {
        lastMovement = "LEFT";
        turnLeft();
        blockAck("STEP:UP_STAIRS:FINAL_TURN_LEFT_90_START");
      } else {
        lastMovement = "RIGHT";
        turnRight();
        blockAck("STEP:UP_STAIRS:FINAL_TURN_RIGHT_90_START");
      }

      blockStep = STEP_UP_FINAL_TURN;
      blockStepStartMs = now;
      return;
    }

    return;
  }

  // ==================================================
  // FINAL TURN 90 DEGREES AND STOP
  // ==================================================
  if (blockStep == STEP_UP_FINAL_TURN) {
    float turned = fabs(yawDeg - upTurnStartYaw);

    if (turned >= (UP_FINAL_TURN_DEG - TURN_TOLERANCE_DEG)) {
      finishActiveBlock("UP_STAIRS_3_STAIRS_DONE_FINAL_TURN_90");
      return;
    }

    if (now - blockStepStartMs >= UP_FINAL_TURN_TIMEOUT_MS) {
      stopMotors();
      lastMovement = "STOP";
      blockAck(
        "WARN:UP_STAIRS:FINAL_TURN_TIMEOUT_STOPPING:"
        "TURNED=" + String(turned, 1)
      );
      finishActiveBlock("UP_STAIRS_3_STAIRS_DONE_TURN_TIMEOUT");
      return;
    }

    setSpeedPercent(UP_FINAL_TURN_SPEED_PERCENT, false);

    if (UP_FINAL_TURN_DIRECTION == 'L') {
      lastMovement = "LEFT";
      turnLeft();
    } else {
      lastMovement = "RIGHT";
      turnRight();
    }

    return;
  }
}


// ==================================================
// RUN AUTO BLOCK: DOWN STAIRS - FRONT JACK WITH REAR SUPPORT
// ==================================================
bool downPitchLevelOK() {
  // Descent pitch can be positive or negative depending on MPU mounting.
  // User requested approximately 0 to 10 degrees, so we check absolute pitch.
  return (fabs(pitch) <= DOWN_LEVEL_PITCH_MAX_DEG);
}

void downDriveStep() {
  if (DOWN_MOVE_FORWARD_DURING_DESCENT) {
    lastMovement = "FORWARD";
    moveForward();
  } else {
    lastMovement = "BACKWARD";
    moveBackward();
  }
}

String downDriveName() {
  if (DOWN_MOVE_FORWARD_DURING_DESCENT) {
    return "FORWARD";
  }

  return "BACKWARD";
}

void startDownTimedForward(MegaBlockStep nextStep, unsigned long now, unsigned long durationMs, String ackText) {
  setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
  downDriveStep();

  blockStep = nextStep;
  blockStepStartMs = now;

  blockAck(ackText + ":FORWARD_MS=" + String(durationMs));
}

void startDownFrontExtend(MegaBlockStep nextStep, unsigned long now, unsigned long durationMs, String ackText) {
  stopMotors();
  lastMovement = "STOP";

  frontJackExtend();

  blockStep = nextStep;
  blockStepStartMs = now;

  blockAck(ackText + ":FRONT_JACK_DOWN_MS=" + String(durationMs));
}

void startDownFrontRetract(MegaBlockStep nextStep, unsigned long now, unsigned long durationMs, String ackText) {
  stopMotors();
  lastMovement = "STOP";

  frontJackRetract();

  blockStep = nextStep;
  blockStepStartMs = now;

  blockAck(ackText + ":FRONT_JACK_UP_MS=" + String(durationMs));
}

void runDownStairsBlock(unsigned long now) {
  // DOWN_STAIRS emergency tilt stop is intentionally disabled by user request.
  // The block will continue even if pitch/roll becomes high.
  // Manual STOP / BLOCK:STOP / AUTO:STOP still works normally.
  if (fabs(roll) >= DOWN_ROLL_HARD_DANGER_DEG || fabs(pitch) >= DOWN_PITCH_HARD_DANGER_DEG) {
    static unsigned long lastDownHardTiltWarnMs = 0;
    if (now - lastDownHardTiltWarnMs > 1000) {
      lastDownHardTiltWarnMs = now;
      blockAck(
        "WARN:DOWN_STAIRS:HARD_TILT_IGNORED_CONTINUING:"
        "PITCH=" + String(pitch, 2) +
        ":ROLL=" + String(roll, 2)
      );
    }
  }

  // Roll warning only, no stop.
  if (fabs(roll) >= DOWN_ROLL_WARNING_DEG) {
    static unsigned long lastDownRollWarnMs = 0;
    if (now - lastDownRollWarnMs > 1000) {
      lastDownRollWarnMs = now;
      blockAck("WARN:DOWN_STAIRS:ROLL_HIGH_CONTINUING:ROLL=" + String(roll, 2));
    }
  }

  if (now - blockStartMs >= DOWN_STAIRS_MAX_TOTAL_MS) {
    blockError("DOWN_STAIRS:TIMEOUT", "MAX_TOTAL_MS");
    return;
  }

  // ==================================================
  // STEP 0:
  // Start the precise descent sequence with front jack and added rear support.
  // ==================================================
  if (blockStep == STEP_DOWN_INIT) {
    downStartPitch = pitch;
    downStepStartPitch = pitch;

    blockAck(
      "STEP:DOWN_STAIRS:PRECISE_FRONT_JACK_WITH_REAR_SUPPORT_START:"
      "DRIVE=" + downDriveName() +
      ":START_PITCH=" + String(downStartPitch, 2) +
      ":LEVEL_TARGET_ABS_PITCH_0_TO_" + String(DOWN_LEVEL_PITCH_MAX_DEG, 1)
    );

    setSpeedPercent(DOWN_STAIRS_SPEED_PERCENT, false);
    downDriveStep();

    blockStep = STEP_DOWN_INITIAL_FORWARD;
    blockStepStartMs = now;
    return;
  }

  // 1) Forward 2 seconds.
  if (blockStep == STEP_DOWN_INITIAL_FORWARD) {
    setSpeedPercent(DOWN_STAIRS_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_INITIAL_FORWARD_MS) {
      startDownFrontExtend(
        STEP_DOWN_FRONT_EXTEND_12S,
        now,
        DOWN_FRONT_EXTEND_12S_MS,
        "STEP:DOWN_STAIRS:FORWARD_2S_DONE"
      );
      return;
    }

    return;
  }

  // 2) Front jack down 12 seconds.
  if (blockStep == STEP_DOWN_FRONT_EXTEND_12S) {
    stopMotors();
    lastMovement = "STOP";
    frontJackExtend();

    if (now - blockStepStartMs >= DOWN_FRONT_EXTEND_12S_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_EXTEND_12S;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_DOWN_12S_DONE:FORWARD_1S_START");
      return;
    }

    return;
  }

  // 3) Forward 1 second.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_EXTEND_12S) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_EXTEND_12S_MS) {
      startDownFrontExtend(
        STEP_DOWN_FRONT_EXTEND_3S,
        now,
        DOWN_FRONT_EXTEND_3S_MS,
        "STEP:DOWN_STAIRS:FORWARD_1S_DONE"
      );
      return;
    }

    return;
  }

  // 4) Front jack down 3 seconds.
  if (blockStep == STEP_DOWN_FRONT_EXTEND_3S) {
    stopMotors();
    lastMovement = "STOP";
    frontJackExtend();

    if (now - blockStepStartMs >= DOWN_FRONT_EXTEND_3S_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_EXTEND_3S;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_DOWN_3S_DONE:FORWARD_1_5S_START");
      return;
    }

    return;
  }

  // 5) Forward 1.5 seconds.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_EXTEND_3S) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_EXTEND_3S_MS) {
      stopMotors();
      lastMovement = "STOP";

      rearJackExtend();

      blockStep = STEP_DOWN_REAR_EXTEND_20S;
      blockStepStartMs = now;

      blockAck(
        "STEP:DOWN_STAIRS:FORWARD_1_5S_DONE:REAR_JACK_DOWN_20S_START:"
        "REAR_JACK_DOWN_MS=" + String(DOWN_REAR_EXTEND_AFTER_FRONT_3S_MS)
      );
      return;
    }

    return;
  }

  // 6) Rear jack down 20 seconds.
  if (blockStep == STEP_DOWN_REAR_EXTEND_20S) {
    stopMotors();
    lastMovement = "STOP";
    rearJackExtend();

    if (now - blockStepStartMs >= DOWN_REAR_EXTEND_AFTER_FRONT_3S_MS) {
      rearJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_REAR_EXTEND_1S;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:REAR_JACK_DOWN_20S_DONE:FORWARD_1S_START");
      return;
    }

    return;
  }

  // 7) Forward 1 second after rear jack down.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_REAR_EXTEND_1S) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_REAR_EXTEND_MS) {
      stopMotors();
      lastMovement = "STOP";

      rearJackRetract();

      blockStep = STEP_DOWN_REAR_RETRACT_20S;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FORWARD_1S_AFTER_REAR_DOWN_DONE:REAR_JACK_UP_20S_START");
      return;
    }

    return;
  }

  // 8) Rear jack up 20 seconds.
  if (blockStep == STEP_DOWN_REAR_RETRACT_20S) {
    stopMotors();
    lastMovement = "STOP";
    rearJackRetract();

    if (now - blockStepStartMs >= DOWN_REAR_RETRACT_AFTER_FORWARD_MS) {
      rearJackStop();

      frontJackRetract();

      blockStep = STEP_DOWN_FRONT_RETRACT_5S_A;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:REAR_JACK_UP_20S_DONE:FRONT_JACK_UP_5S_A_START");
      return;
    }

    return;
  }

  // 9) Front jack up 5 seconds.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_5S_A) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_5S_A_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_RETRACT_5S_A;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_UP_5S_A_DONE:FORWARD_1S_START");
      return;
    }

    return;
  }

  // 10) Forward 1 second.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_RETRACT_5S_A) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_RETRACT_5S_A_MS) {
      startDownFrontRetract(
        STEP_DOWN_FRONT_RETRACT_5S_B,
        now,
        DOWN_FRONT_RETRACT_10S_B_MS,
        "STEP:DOWN_STAIRS:FORWARD_1S_DONE"
      );
      return;
    }

    return;
  }

  // 11) Front jack up another 5 seconds.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_5S_B) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_10S_B_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_RETRACT_5S_B;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_UP_5S_B_DONE:FORWARD_1S_START");
      return;
    }

    return;
  }

  // 12) Forward 1 second.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_RETRACT_5S_B) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_RETRACT_10S_B_MS) {
      startDownFrontExtend(
        STEP_DOWN_FRONT_EXTEND_5S_A,
        now,
        DOWN_FRONT_EXTEND_5S_A_MS,
        "STEP:DOWN_STAIRS:FORWARD_1S_DONE"
      );
      return;
    }

    return;
  }

  // 13) Front jack down 5 seconds.
  if (blockStep == STEP_DOWN_FRONT_EXTEND_5S_A) {
    stopMotors();
    lastMovement = "STOP";
    frontJackExtend();

    if (now - blockStepStartMs >= DOWN_FRONT_EXTEND_5S_A_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_EXTEND_5S_A;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_DOWN_5S_A_DONE:FORWARD_2S_START");
      return;
    }

    return;
  }

  // 14) Forward 2 seconds.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_EXTEND_5S_A) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_EXTEND_5S_A_MS) {
      startDownFrontRetract(
        STEP_DOWN_FRONT_RETRACT_2S_A,
        now,
        DOWN_FRONT_RETRACT_2S_A_MS,
        "STEP:DOWN_STAIRS:FORWARD_2S_DONE"
      );
      return;
    }

    return;
  }

  // 15) Front jack up 2 seconds.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_2S_A) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_2S_A_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_RETRACT_2S_A;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_UP_2S_A_DONE:FORWARD_2S_START");
      return;
    }

    return;
  }

  // 16) Forward 2 seconds.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_RETRACT_2S_A) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_RETRACT_2S_A_MS) {
      startDownFrontRetract(
        STEP_DOWN_FRONT_RETRACT_3S_A,
        now,
        DOWN_FRONT_RETRACT_3S_A_MS,
        "STEP:DOWN_STAIRS:FORWARD_2S_DONE"
      );
      return;
    }

    return;
  }

  // 17) Front jack up 3 seconds.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_3S_A) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_3S_A_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_BEFORE_LEVEL_CHECK;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_UP_3S_A_DONE:FORWARD_5S_BEFORE_LEVEL_CHECK_START");
      return;
    }

    return;
  }

  // 18) Forward 5 seconds, then check pitch level.
  if (blockStep == STEP_DOWN_FORWARD_BEFORE_LEVEL_CHECK) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_BEFORE_LEVEL_CHECK_MS) {
      stopMotors();
      lastMovement = "STOP";

      blockStep = STEP_DOWN_LEVEL_CHECK;
      blockStepStartMs = now;

      blockAck(
        "STEP:DOWN_STAIRS:FORWARD_5S_DONE:LEVEL_CHECK:"
        "ABS_PITCH=" + String(fabs(pitch), 2) +
        ":TARGET_MAX=" + String(DOWN_LEVEL_PITCH_MAX_DEG, 1)
      );
      return;
    }

    return;
  }

  // 19) If abs(pitch) is 0..10, continue.
  // If not, forward 6 seconds and check again.
  if (blockStep == STEP_DOWN_LEVEL_CHECK) {
    stopMotors();
    lastMovement = "STOP";

    if (downPitchLevelOK()) {
      blockAck(
        "STEP:DOWN_STAIRS:LEVEL_OK_CONTINUE:"
        "PITCH=" + String(pitch, 2) +
        ":ROLL=" + String(roll, 2)
      );

      startDownFrontExtend(
        STEP_DOWN_FRONT_EXTEND_5S_B,
        now,
        DOWN_FRONT_EXTEND_5S_B_MS,
        "STEP:DOWN_STAIRS:LEVEL_OK"
      );
      return;
    }

    blockAck(
      "WARN:DOWN_STAIRS:LEVEL_NOT_OK_FORWARD_6S_AND_RECHECK:"
      "PITCH=" + String(pitch, 2) +
      ":ROLL=" + String(roll, 2) +
      ":ABS_PITCH=" + String(fabs(pitch), 2)
    );

    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    blockStep = STEP_DOWN_LEVEL_RETRY_FORWARD;
    blockStepStartMs = now;
    return;
  }

  // 20) Forward 6 seconds, then check pitch again. Repeat until level or total timeout.
  if (blockStep == STEP_DOWN_LEVEL_RETRY_FORWARD) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_LEVEL_RETRY_FORWARD_MS) {
      stopMotors();
      lastMovement = "STOP";

      blockStep = STEP_DOWN_LEVEL_CHECK;
      blockStepStartMs = now;

      blockAck(
        "STEP:DOWN_STAIRS:FORWARD_6S_RETRY_DONE:RECHECK_LEVEL:"
        "ABS_PITCH=" + String(fabs(pitch), 2) +
        ":TARGET_MAX=" + String(DOWN_LEVEL_PITCH_MAX_DEG, 1)
      );
      return;
    }

    return;
  }

  // 21) Front jack down 5 seconds.
  if (blockStep == STEP_DOWN_FRONT_EXTEND_5S_B) {
    stopMotors();
    lastMovement = "STOP";
    frontJackExtend();

    if (now - blockStepStartMs >= DOWN_FRONT_EXTEND_5S_B_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_EXTEND_5S_B;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_DOWN_5S_B_DONE:FORWARD_2S_START");
      return;
    }

    return;
  }

  // 22) Forward 2 seconds.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_EXTEND_5S_B) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_EXTEND_5S_B_MS) {
      startDownFrontRetract(
        STEP_DOWN_FRONT_RETRACT_2S_B,
        now,
        DOWN_FRONT_RETRACT_2S_B_MS,
        "STEP:DOWN_STAIRS:FORWARD_2S_DONE"
      );
      return;
    }

    return;
  }

  // 23) Front jack up 2 seconds.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_2S_B) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_2S_B_MS) {
      frontJackStop();

      setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
      downDriveStep();

      blockStep = STEP_DOWN_FORWARD_AFTER_RETRACT_2S_B;
      blockStepStartMs = now;

      blockAck("STEP:DOWN_STAIRS:FRONT_JACK_UP_2S_B_DONE:FORWARD_2S_START");
      return;
    }

    return;
  }

  // 24) Forward 2 seconds.
  if (blockStep == STEP_DOWN_FORWARD_AFTER_RETRACT_2S_B) {
    setSpeedPercent(DOWN_AFTER_JACK_SPEED_PERCENT, false);
    downDriveStep();

    if (now - blockStepStartMs >= DOWN_FORWARD_AFTER_RETRACT_2S_B_MS) {
      startDownFrontRetract(
        STEP_DOWN_FRONT_RETRACT_3S_B,
        now,
        DOWN_FRONT_RETRACT_3S_B_MS,
        "STEP:DOWN_STAIRS:FORWARD_2S_DONE"
      );
      return;
    }

    return;
  }

  // 25) Front jack up 3 seconds, then finish.
  if (blockStep == STEP_DOWN_FRONT_RETRACT_3S_B) {
    stopMotors();
    lastMovement = "STOP";
    frontJackRetract();

    if (now - blockStepStartMs >= DOWN_FRONT_RETRACT_3S_B_MS) {
      frontJackStop();
      finishActiveBlock("DOWN_STAIRS_FRONT_JACK_REAR_SUPPORT_PRECISE_SEQUENCE_DONE");
      return;
    }

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
  float pitchLimit = PITCH_DANGER_DEG;
  float rollLimit = ROLL_DANGER_DEG;

  if (activeBlock == BLOCK_UP_STAIRS) {
    pitchLimit = UP_PITCH_HARD_DANGER_DEG;
    rollLimit = UP_ROLL_HARD_DANGER_DEG;
  } else if (activeBlock == BLOCK_DOWN_STAIRS) {
    pitchLimit = DOWN_PITCH_HARD_DANGER_DEG;
    rollLimit = DOWN_ROLL_HARD_DANGER_DEG;
  }

  if (fabs(pitch) > pitchLimit) {
    return "TILT_DANGER";
  }

  if (fabs(roll) > rollLimit) {
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

  float newPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float newRoll  = atan2(-accelX, accelZ) * 180.0 / PI;

  bool invalidAngles = isnan(newPitch) || isnan(newRoll) ||
                       fabs(newPitch) > MPU_MAX_VALID_ANGLE_DEG || fabs(newRoll) > MPU_MAX_VALID_ANGLE_DEG;

  if (invalidAngles) {
    unsigned long nowMs = millis();
    if (nowMs - lastMpuSpikeWarnMs >= MPU_SPIKE_WARN_REPEAT_MS) {
      lastMpuSpikeWarnMs = nowMs;
      sendBoth("WARN:MEGA:MPU_INVALID_READING_IGNORED:PITCH=" + String(newPitch, 2) + ":ROLL=" + String(newRoll, 2));
    }
    return;
  }

  if (mpuHaveValidAngles) {
    float pitchJump = fabs(newPitch - pitch);
    float rollJump  = fabs(newRoll - roll);

    if (pitchJump > MPU_MAX_SINGLE_SAMPLE_JUMP_DEG || rollJump > MPU_MAX_SINGLE_SAMPLE_JUMP_DEG) {
      unsigned long nowMs = millis();
      if (nowMs - lastMpuSpikeWarnMs >= MPU_SPIKE_WARN_REPEAT_MS) {
        lastMpuSpikeWarnMs = nowMs;
        sendBoth(
          "WARN:MEGA:MPU_SPIKE_IGNORED:"
          "OLD_PITCH=" + String(pitch, 2) +
          ":NEW_PITCH=" + String(newPitch, 2) +
          ":OLD_ROLL=" + String(roll, 2) +
          ":NEW_ROLL=" + String(newRoll, 2)
        );
      }
      return;
    }
  }

  pitch = newPitch;
  roll  = newRoll;
  mpuHaveValidAngles = true;

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