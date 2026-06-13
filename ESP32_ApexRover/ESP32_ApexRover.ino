#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge + Camera Stand Controller
//
// Manual path:
//   Mobile App -> WebSocket 81 -> ESP32 -> Mega / UNO
//
// Auto path:
//   Raspberry Auto -> HTTP /command or WebSocket 81 -> ESP32 -> Mega / UNO
//
// Sensor path - CLEAN VERSION:
//   Mega Serial1 TX -> ESP32 RX GPIO16 -> ESP32 WebSocket -> Mobile App
//   Raspberry does NOT read Mega USB Serial anymore.
//
// Mega ACK path:
//   Mega Serial1 TX -> ESP32 RX GPIO16 -> ESP32 WebSocket -> Mobile App
//
// Raspberry command mirror:
//   Every command received by ESP32 and every routed command sent by ESP32
//   is mirrored to Raspberry over UDP so Raspberry always knows the
//   command currently being applied by the system.
//
// Routing:
//   CAM:*        -> handled locally on ESP32 (camera stand)
//   ARM:*        -> UNO
//   AUTO:FULL_SCENARIO -> handled by ESP32 state machine
//   AUTO:* / BLOCK:* / movement / jack / speed -> Mega
//
// Camera Stand pins (ESP32):
//   Camera A4988 EN       -> ESP32 GPIO25
//   Camera A4988 STEP     -> ESP32 GPIO26
//   Camera A4988 DIR      -> ESP32 GPIO27
//   Camera Servo Signal   -> ESP32 GPIO14
//
// Serial pin mapping:
//   Mega bidirectional Serial1:
//     ESP32 GPIO17 TX -> Mega RX1 Pin 19
//     Mega TX1 Pin 18 -> ESP32 GPIO16 RX  (USE VOLTAGE DIVIDER 5V -> 3.3V)
//     ESP32 GND       -> Mega GND
//   UNO command direction:
//     ESP32 GPIO4 TX  -> UNO D2 SoftwareSerial RX
//     ESP32 GND       -> UNO GND
// =================================================================


// -----------------------------------------------------------------
// WiFi Access Point
// -----------------------------------------------------------------
const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";


// -----------------------------------------------------------------
// Serial pins
// -----------------------------------------------------------------
#define MEGA_RX_PIN 16
#define MEGA_TX_PIN 17
#define UNO_RX_PIN  15
#define UNO_TX_PIN  4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

WebSocketsServer webSocket = WebSocketsServer(81);
WebServer httpServer(80);
WiFiUDP raspberryCommandUdp;


// =================================================================
// Camera Stand Pins
// =================================================================
#define CAM_EN_PIN     25
#define CAM_STEP_PIN   26
#define CAM_DIR_PIN    27
#define CAM_SERVO_PIN  14


// =================================================================
// Camera Stand Constants
// =================================================================
const unsigned int CAM_STEP_PULSE_MICROS   = 3;
const unsigned long CAM_SERVO_DETACH_DELAY_MS = 450;

const int CAM_SERVO_MIN    = 0;
const int CAM_SERVO_MAX    = 90;
const int CAM_SERVO_CENTER = 45;
const bool INVERT_CAMERA_VERTICAL = true;

const long CAM_MIN_STEPPER_STEPS = 1;
const long CAM_MAX_STEPPER_STEPS = 50000;
const int  CAM_MIN_SERVO_STEP    = 1;
const int  CAM_MAX_SERVO_STEP    = 30;

const long CAM_DEFAULT_STEPPER_STEPS = 100;
const int  CAM_DEFAULT_SERVO_STEP    = 5;

const unsigned long CAM_CONTINUOUS_SERVO_INTERVAL_MS = 35;


// =================================================================
// Camera Stand State
// =================================================================
Servo cameraServo;

long  camConfiguredStepperSteps = CAM_DEFAULT_STEPPER_STEPS;
int   camConfiguredServoStep    = CAM_DEFAULT_SERVO_STEP;

int   cameraServoAngle    = CAM_SERVO_CENTER;
bool  cameraServoAttached = false;
bool  pendingCameraDetach = false;
unsigned long cameraDetachStartMs = 0;

int   cameraServoMoveDir  = 0;   // -1 up, 0 stop, 1 down (after invert applied)
unsigned long lastCamContinuousMoveMs = 0;

int  cameraStepperDirection      = 0;
long cameraFiniteStepsRemaining  = 0;
long cameraStepPosition          = 0;
unsigned long cameraLastStepTime = 0;
unsigned long cameraStepIntervalMicros = 700;


// =================================================================
// Bridge / sensor state
// =================================================================
String currentMode = "MANUAL";

String lastSensorMessage =
  "SENSOR:PITCH=0;ROLL=0;FRONT=-1;REAR=-1;BALANCE=NO DATA";

unsigned long lastCommandAt = 0;
String lastCommand = "NONE";

String lastBridgeEvent  = "NONE";
String lastMegaAck      = "NONE";
String lastMegaError    = "NONE";
String lastBridgeSource = "NONE";
unsigned long lastBridgeEventAt = 0;


// =================================================================
// Raspberry notification settings
// =================================================================
// Raspberry Pi should be connected to the ESP32 AP.
// Change this IP/port if your Raspberry gets another address.
const bool RASPBERRY_NOTIFY_ENABLED = true;
const char* RASPBERRY_HOST = "192.168.4.2";
const uint16_t RASPBERRY_PORT = 5050;
const char* RASPBERRY_AUTO_STATUS_PATH = "/auto_status";

// Every command that reaches the ESP32, and every command routed by the ESP32,
// is mirrored to the Raspberry Pi using UDP. UDP is used so manual control stays
// fast even if the Raspberry Pi is temporarily offline.
const bool RASPBERRY_COMMAND_MIRROR_ENABLED = true;
IPAddress RASPBERRY_COMMAND_IP(192, 168, 4, 2);
const uint16_t RASPBERRY_COMMAND_UDP_PORT = 5055;
const uint16_t ESP32_COMMAND_UDP_LOCAL_PORT = 5056;

String lastRaspberryMirrorPayload = "NONE";
String lastRaspberryMirrorTarget  = "NONE";
String lastRaspberryMirrorCommand = "NONE";
bool lastRaspberryMirrorOk = false;
unsigned long lastRaspberryMirrorAt = 0;
unsigned long raspberryMirrorCount = 0;


// =================================================================
// Full automatic scenario managed by ESP32
// Mobile/Raspberry sends one command:
//   AUTO:FULL_SCENARIO
// ESP32 then sends these commands to Mega one by one:
//   1) AUTO:UP_STAIRS
//   2) BLOCK:GO:FORWARD:4
//   3) BLOCK:TURN:RIGHT:90
//   4) BLOCK:GO:FORWARD:1
//   5) AUTO:DOWN_STAIRS
// ESP32 waits for Mega DONE ACK before sending the next command.
// ACK can arrive directly on MegaSerial RX or through Raspberry /bridge_event.
// =================================================================
enum FullAutoStep {
  FULL_AUTO_IDLE,
  FULL_AUTO_UP_STAIRS,
  FULL_AUTO_GO_FORWARD_4,
  FULL_AUTO_TURN_RIGHT_90,
  FULL_AUTO_GO_FORWARD_1,
  FULL_AUTO_DOWN_STAIRS,
  FULL_AUTO_DONE,
  FULL_AUTO_ERROR
};

bool fullAutoActive = false;
FullAutoStep fullAutoStep = FULL_AUTO_IDLE;
unsigned long fullAutoStartedAt = 0;
unsigned long fullAutoStepStartedAt = 0;
unsigned long fullAutoStepTimeoutMs = 0;
int fullAutoStepNumber = 0;

String fullAutoState = "IDLE";
String fullAutoCurrentCommand = "NONE";
String fullAutoExpectedDonePrefix = "NONE";
String fullAutoLastAck = "NONE";
String fullAutoLastError = "NONE";
String fullAutoLastStage = "IDLE";


// =================================================================
// Function prototypes used before their definitions
// =================================================================
void mirrorCommandToRaspberry(const String& direction, const String& target, const String& cmd);
void stopCameraMotion();
void broadcastMegaSensorToMobile(String line);


// =================================================================
// Serial helpers
// =================================================================
void sendToMega(const String& cmd) {
  MegaSerial.println(cmd);
  mirrorCommandToRaspberry("OUT", "MEGA", cmd);
  Serial.print("[-> MEGA] ");
  Serial.println(cmd);
}

void sendToUno(const String& cmd) {
  UnoSerial.println(cmd);
  mirrorCommandToRaspberry("OUT", "UNO", cmd);
  Serial.print("[-> UNO] ");
  Serial.println(cmd);
}


// =================================================================
// General helpers
// =================================================================
String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\n", "\\n");
  s.replace("\r", "");
  return s;
}

String urlDecode(String s) {
  s.replace("%3A", ":");
  s.replace("%3a", ":");
  s.replace("%3B", ";");
  s.replace("%3b", ";");
  s.replace("%3D", "=");
  s.replace("%3d", "=");
  s.replace("%2F", "/");
  s.replace("%2f", "/");
  s.replace("%20", " ");
  s.replace("+", " ");
  s.replace("%25", "%");
  return s;
}

String urlEncode(String s) {
  String out = "";

  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);

    if (isAlphaNumeric(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else if (c == ' ') {
      out += "%20";
    } else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", (uint8_t)c);
      out += buf;
    }
  }

  return out;
}

void mirrorCommandToRaspberry(const String& direction, const String& target, const String& cmd) {
  if (!RASPBERRY_COMMAND_MIRROR_ENABLED) return;
  if (cmd.length() == 0) return;

  String payload = "{";
  payload += "\"event\":\"esp32_command\",";
  payload += "\"direction\":\"" + jsonEscape(direction) + "\",";
  payload += "\"target\":\"" + jsonEscape(target) + "\",";
  payload += "\"mode\":\"" + jsonEscape(currentMode) + "\",";
  payload += "\"command\":\"" + jsonEscape(cmd) + "\",";
  payload += "\"full_auto_active\":" + String(fullAutoActive ? "true" : "false") + ",";
  payload += "\"full_auto_stage\":\"" + jsonEscape(fullAutoLastStage) + "\",";
  payload += "\"millis\":" + String(millis());
  payload += "}";

  raspberryCommandUdp.beginPacket(RASPBERRY_COMMAND_IP, RASPBERRY_COMMAND_UDP_PORT);
  raspberryCommandUdp.print(payload);
  int ok = raspberryCommandUdp.endPacket();

  lastRaspberryMirrorPayload = payload;
  lastRaspberryMirrorTarget = target;
  lastRaspberryMirrorCommand = cmd;
  lastRaspberryMirrorOk = (ok == 1);
  lastRaspberryMirrorAt = millis();
  raspberryMirrorCount++;

  Serial.print("[RPI MIRROR ");
  Serial.print(direction);
  Serial.print(" -> ");
  Serial.print(target);
  Serial.print("] ");
  Serial.println(cmd);
}

long camClampLong(long v, long lo, long hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

int camClampInt(int v, int lo, int hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

void broadcastStatusEvent(const String& eventName) {
  String msg = "{";
  msg += "\"event\":\"" + eventName + "\",";
  msg += "\"mode\":\"" + currentMode + "\",";
  msg += "\"last_command\":\"" + jsonEscape(lastCommand) + "\"";
  msg += "}";

  webSocket.broadcastTXT(msg);
}

void broadcastBridgeLine(const String& source, const String& line) {

  // WebSockets library wants String&, not const String&
  String rawPayload = line;
  webSocket.broadcastTXT(rawPayload);

  String json = "{";
  json += "\"event\":\"bridge_event\",";
  json += "\"source\":\"" + jsonEscape(source) + "\",";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"line\":\"" + jsonEscape(line) + "\",";
  json += "\"last_command\":\"" + jsonEscape(lastCommand) + "\"";
  json += "}";

  String jsonPayload = json;
  webSocket.broadcastTXT(jsonPayload);
}

String fullAutoStepName(FullAutoStep step) {
  switch (step) {
    case FULL_AUTO_UP_STAIRS:      return "UP_STAIRS";
    case FULL_AUTO_GO_FORWARD_4:   return "GO_FORWARD_4_UNITS";
    case FULL_AUTO_TURN_RIGHT_90:  return "TURN_RIGHT_90";
    case FULL_AUTO_GO_FORWARD_1:   return "GO_FORWARD_1_UNIT";
    case FULL_AUTO_DOWN_STAIRS:    return "DOWN_STAIRS";
    case FULL_AUTO_DONE:           return "DONE";
    case FULL_AUTO_ERROR:          return "ERROR";
    default:                       return "IDLE";
  }
}

void notifyRaspberryAutoStatus(const String& phase, const String& detail) {
  if (!RASPBERRY_NOTIFY_ENABLED) return;
  if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) return;

  HTTPClient http;
  String url = "http://" + String(RASPBERRY_HOST) + ":" + String(RASPBERRY_PORT) + String(RASPBERRY_AUTO_STATUS_PATH);
  url += "?source=ESP32";
  url += "&phase=" + urlEncode(phase);
  url += "&step=" + String(fullAutoStepNumber);
  url += "&stage=" + urlEncode(fullAutoLastStage);
  url += "&command=" + urlEncode(fullAutoCurrentCommand);
  url += "&detail=" + urlEncode(detail);

  http.begin(url);
  http.setConnectTimeout(700);
  http.setTimeout(700);
  int code = http.GET();
  Serial.print("[RASPBERRY AUTO STATUS] HTTP ");
  Serial.println(code);
  http.end();
}

void broadcastFullAutoEvent(const String& eventName, const String& detail) {
  String json = "{";
  json += "\"event\":\"" + jsonEscape(eventName) + "\",";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"auto_active\":" + String(fullAutoActive ? "true" : "false") + ",";
  json += "\"auto_state\":\"" + jsonEscape(fullAutoState) + "\",";
  json += "\"auto_step\":" + String(fullAutoStepNumber) + ",";
  json += "\"auto_stage\":\"" + jsonEscape(fullAutoLastStage) + "\",";
  json += "\"auto_command\":\"" + jsonEscape(fullAutoCurrentCommand) + "\",";
  json += "\"auto_expected_done\":\"" + jsonEscape(fullAutoExpectedDonePrefix) + "\",";
  json += "\"auto_last_ack\":\"" + jsonEscape(fullAutoLastAck) + "\",";
  json += "\"auto_last_error\":\"" + jsonEscape(fullAutoLastError) + "\",";
  json += "\"detail\":\"" + jsonEscape(detail) + "\"";
  json += "}";

  webSocket.broadcastTXT(json);
  notifyRaspberryAutoStatus(eventName, detail);
}

void fullAutoFail(const String& reason) {
  fullAutoActive = false;
  fullAutoStep = FULL_AUTO_ERROR;
  fullAutoState = "ERROR";
  fullAutoLastStage = "ERROR";
  fullAutoLastError = reason;

  stopCameraMotion();
  sendToMega("AUTO:STOP");
  sendToUno("ARM:STOP");

  Serial.print("[FULL AUTO ERROR] ");
  Serial.println(reason);
  broadcastFullAutoEvent("full_auto_error", reason);
}

void fullAutoSendStep(FullAutoStep step) {
  fullAutoStep = step;
  fullAutoStepNumber++;
  fullAutoStepStartedAt = millis();
  fullAutoLastStage = fullAutoStepName(step);
  fullAutoState = "RUNNING";

  if (step == FULL_AUTO_UP_STAIRS) {
    fullAutoCurrentCommand = "AUTO:UP_STAIRS";
    fullAutoExpectedDonePrefix = "ACK:MEGA:DONE:UP_STAIRS";
    fullAutoStepTimeoutMs = 210000UL;
  } else if (step == FULL_AUTO_GO_FORWARD_4) {
    fullAutoCurrentCommand = "BLOCK:GO:FORWARD:4";
    fullAutoExpectedDonePrefix = "ACK:MEGA:DONE:GO:FORWARD";
    fullAutoStepTimeoutMs = 15000UL;
  } else if (step == FULL_AUTO_TURN_RIGHT_90) {
    fullAutoCurrentCommand = "BLOCK:TURN:RIGHT:90";
    fullAutoExpectedDonePrefix = "ACK:MEGA:DONE:TURN:RIGHT";
    fullAutoStepTimeoutMs = 30000UL;
  } else if (step == FULL_AUTO_GO_FORWARD_1) {
    fullAutoCurrentCommand = "BLOCK:GO:FORWARD:1";
    fullAutoExpectedDonePrefix = "ACK:MEGA:DONE:GO:FORWARD";
    fullAutoStepTimeoutMs = 15000UL;
  } else if (step == FULL_AUTO_DOWN_STAIRS) {
    fullAutoCurrentCommand = "AUTO:DOWN_STAIRS";
    fullAutoExpectedDonePrefix = "ACK:MEGA:DONE:DOWN_STAIRS";
    fullAutoStepTimeoutMs = 230000UL;
  } else {
    return;
  }

  sendToMega(fullAutoCurrentCommand);

  Serial.print("[FULL AUTO STEP ");
  Serial.print(fullAutoStepNumber);
  Serial.print("] ");
  Serial.println(fullAutoCurrentCommand);

  broadcastFullAutoEvent("full_auto_step_started", fullAutoLastStage);
}

void fullAutoAdvance() {
  if (fullAutoStep == FULL_AUTO_UP_STAIRS) {
    fullAutoSendStep(FULL_AUTO_GO_FORWARD_4);
    return;
  }

  if (fullAutoStep == FULL_AUTO_GO_FORWARD_4) {
    fullAutoSendStep(FULL_AUTO_TURN_RIGHT_90);
    return;
  }

  if (fullAutoStep == FULL_AUTO_TURN_RIGHT_90) {
    fullAutoSendStep(FULL_AUTO_GO_FORWARD_1);
    return;
  }

  if (fullAutoStep == FULL_AUTO_GO_FORWARD_1) {
    fullAutoSendStep(FULL_AUTO_DOWN_STAIRS);
    return;
  }

  if (fullAutoStep == FULL_AUTO_DOWN_STAIRS) {
    fullAutoActive = false;
    fullAutoStep = FULL_AUTO_DONE;
    fullAutoState = "DONE";
    fullAutoLastStage = "DONE";
    fullAutoCurrentCommand = "NONE";
    fullAutoExpectedDonePrefix = "NONE";

    sendToMega("SYS:MODE:MANUAL");
    sendToUno("SYS:MODE:MANUAL");
    currentMode = "MANUAL";

    broadcastFullAutoEvent("full_auto_done", "FULL_SCENARIO_FINISHED");
    return;
  }
}

void startFullAutoScenario() {
  if (fullAutoActive) {
    broadcastFullAutoEvent("full_auto_busy", "Scenario already running");
    return;
  }

  stopCameraMotion();

  currentMode = "AUTO";
  sendToMega("SYS:MODE:AUTO");
  sendToUno("SYS:MODE:AUTO");

  fullAutoActive = true;
  fullAutoStep = FULL_AUTO_IDLE;
  fullAutoStartedAt = millis();
  fullAutoStepStartedAt = fullAutoStartedAt;
  fullAutoStepNumber = 0;
  fullAutoState = "RUNNING";
  fullAutoCurrentCommand = "NONE";
  fullAutoExpectedDonePrefix = "NONE";
  fullAutoLastAck = "NONE";
  fullAutoLastError = "NONE";
  fullAutoLastStage = "START";

  broadcastStatusEvent("mode_changed");
  broadcastFullAutoEvent("full_auto_started", "UP_STAIRS_GO4_TURN_RIGHT90_GO1_DOWN_STAIRS");

  fullAutoSendStep(FULL_AUTO_UP_STAIRS);
}

void cancelFullAutoScenario(const String& reason) {
  if (!fullAutoActive && fullAutoState != "RUNNING") {
    return;
  }

  fullAutoActive = false;
  fullAutoState = "CANCELLED";
  fullAutoLastStage = "CANCELLED";
  fullAutoLastError = reason;

  sendToMega("AUTO:STOP");
  sendToUno("ARM:STOP");
  stopCameraMotion();

  broadcastFullAutoEvent("full_auto_cancelled", reason);
}

void handleFullAutoFeedbackLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("ACK:MEGA:")) {
    fullAutoLastAck = line;
  }

  if (line.startsWith("ERR:MEGA:") || line.startsWith("ERROR:MEGA:")) {
    fullAutoFail(line);
    return;
  }

  if (!fullAutoActive) return;

  if (fullAutoExpectedDonePrefix != "NONE" && line.startsWith(fullAutoExpectedDonePrefix)) {
    broadcastFullAutoEvent("full_auto_step_done", line);
    fullAutoAdvance();
  }
}

void runFullAutoScenario() {
  if (!fullAutoActive) return;

  unsigned long now = millis();
  if (fullAutoStepTimeoutMs > 0 && (now - fullAutoStepStartedAt >= fullAutoStepTimeoutMs)) {
    fullAutoFail(
      String("TIMEOUT:") + fullAutoLastStage +
      ":WAITING_FOR=" + fullAutoExpectedDonePrefix +
      ":LAST_ACK=" + fullAutoLastAck
    );
  }
}


// =================================================================
// Mega direct sensor helpers
// =================================================================
String getFieldFromMegaLine(const String& payload, const String& key) {
  String search = key + "=";
  int idx = payload.indexOf(search);
  if (idx < 0) return "";

  int start = idx + search.length();
  int end = payload.indexOf(';', start);
  if (end < 0) end = payload.length();

  String value = payload.substring(start, end);
  value.trim();
  return value;
}

String normalizeMegaSensorForMobile(String line) {
  line.trim();

  String payload = line;
  if (payload.startsWith("SENSOR:")) {
    payload = payload.substring(7);
  } else if (payload.startsWith("STATUS:")) {
    payload = payload.substring(7);
  }

  String pitch = getFieldFromMegaLine(payload, "PITCH");
  String roll  = getFieldFromMegaLine(payload, "ROLL");
  String front = getFieldFromMegaLine(payload, "FRONT");
  String rear  = getFieldFromMegaLine(payload, "REAR");

  if (front.length() == 0) front = getFieldFromMegaLine(payload, "UF");
  if (rear.length()  == 0) rear  = getFieldFromMegaLine(payload, "UR");

  String balance = getFieldFromMegaLine(payload, "BALANCE");
  String alert   = getFieldFromMegaLine(payload, "ALERT");

  if (pitch.length() == 0) pitch = "0";
  if (roll.length()  == 0) roll  = "0";
  if (front.length() == 0) front = "-1";
  if (rear.length()  == 0) rear  = "-1";

  alert.toUpperCase();
  balance.toUpperCase();

  if (balance.length() == 0) {
    float p = pitch.toFloat();
    float r = roll.toFloat();

    if (alert == "TILT_DANGER") {
      balance = "DANGER";
    } else if (fabs(p) >= 30.0 || fabs(r) >= 25.0) {
      balance = "DANGER";
    } else if (fabs(p) >= 15.0 || fabs(r) >= 12.0) {
      balance = "WARNING";
    } else {
      balance = "STABLE";
    }
  }

  String sensorMessage = "SENSOR:";
  sensorMessage += "PITCH=" + pitch;
  sensorMessage += ";ROLL=" + roll;
  sensorMessage += ";FRONT=" + front;
  sensorMessage += ";REAR=" + rear;
  sensorMessage += ";BALANCE=" + balance;

  if (alert.length() > 0) {
    sensorMessage += ";ALERT=" + alert;
  }

  return sensorMessage;
}

void broadcastMegaSensorToMobile(String line) {
  String sensorMessage = normalizeMegaSensorForMobile(line);
  lastSensorMessage = sensorMessage;
  webSocket.broadcastTXT(sensorMessage);

  Serial.print("[MEGA SENSOR -> APP] ");
  Serial.println(sensorMessage);
}

void readMegaDirectFeedback() {
  while (MegaSerial.available() > 0) {
    String line = MegaSerial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    // CLEAN SENSOR PATH:
    // Mega sends SENSOR/STATUS to ESP32 directly.
    // ESP32 broadcasts normalized SENSOR messages to the mobile app.
    // Raspberry is not involved in sensor reading anymore.
    if (line.startsWith("SENSOR:") || line.startsWith("STATUS:")) {
      broadcastMegaSensorToMobile(line);
      continue;
    }

    lastBridgeSource = "MEGA_DIRECT";
    lastBridgeEvent = line;
    lastBridgeEventAt = millis();

    if (line.startsWith("ACK:")) lastMegaAck = line;
    if (line.startsWith("ERR:") || line.startsWith("ERROR:")) lastMegaError = line;

    Serial.print("[<- MEGA DIRECT] ");
    Serial.println(line);

    broadcastBridgeLine("MEGA_DIRECT", line);
    handleFullAutoFeedbackLine(line);
  }
}

// =================================================================
// Camera stand - stepper helpers
// =================================================================
void camEnableDriver() {
  digitalWrite(CAM_EN_PIN, LOW);
}

void camDisableDriver() {
  digitalWrite(CAM_EN_PIN, HIGH);
}


// =================================================================
// Camera stand - servo helpers
// =================================================================
void attachCameraServoIfNeeded() {
  if (!cameraServoAttached) {
    cameraServo.attach(CAM_SERVO_PIN);
    cameraServoAttached = true;
  }
}

void detachCameraServo() {
  if (cameraServoAttached) {
    cameraServo.detach();
    cameraServoAttached = false;
  }
  pendingCameraDetach = false;
}

void scheduleCameraDetach() {
  pendingCameraDetach = true;
  cameraDetachStartMs = millis();
}

void setCameraAngle(int angle) {
  cameraServoAngle = camClampInt(angle, CAM_SERVO_MIN, CAM_SERVO_MAX);
  attachCameraServoIfNeeded();
  cameraServo.write(cameraServoAngle);
  scheduleCameraDetach();
}

void stopCameraMotion() {
  cameraStepperDirection = 0;
  cameraFiniteStepsRemaining = 0;
  cameraServoMoveDir = 0;

  digitalWrite(CAM_STEP_PIN, LOW);
  camDisableDriver();

  scheduleCameraDetach();
}


// =================================================================
// Camera stand - command parsing helpers
// =================================================================
long camParseLastLong(const String& cmd) {
  int idx = cmd.lastIndexOf(':');
  if (idx < 0) return 0;
  return cmd.substring(idx + 1).toInt();
}

long camParsePositiveSteps(const String& cmd) {
  long v = camParseLastLong(cmd);
  if (v < 0) v = -v;
  return camClampLong(v, CAM_MIN_STEPPER_STEPS, CAM_MAX_STEPPER_STEPS);
}

long camParseStepsOrDefault(const String& cmd) {
  int idx = cmd.lastIndexOf(':');
  if (idx < 0) return camConfiguredStepperSteps;

  String afterColon = cmd.substring(idx + 1);
  afterColon.trim();

  bool hasDigit = false;
  for (unsigned int i = 0; i < afterColon.length(); i++) {
    if (isDigit(afterColon[i])) { hasDigit = true; break; }
  }

  if (!hasDigit) return camConfiguredStepperSteps;
  return camParsePositiveSteps(cmd);
}

int camParseServoStepOrDefault(const String& cmd) {
  long v = camParseLastLong(cmd);
  if (v <= 0) return camConfiguredServoStep;
  return (int)camClampLong(v, CAM_MIN_SERVO_STEP, CAM_MAX_SERVO_STEP);
}


// =================================================================
// Camera stand - config command handler
// Returns true if command was consumed.
// =================================================================
bool handleCamConfigCommand(const String& cmd) {
  if (cmd.startsWith("CAM:CONFIG:STEPPER_STEPS:")) {
    long v = camParseLastLong(cmd);
    camConfiguredStepperSteps = camClampLong(v, CAM_MIN_STEPPER_STEPS, CAM_MAX_STEPPER_STEPS);

    String ack = "ACK:CONFIG:STEPPER_STEPS:" + String(camConfiguredStepperSteps);
    webSocket.broadcastTXT(ack);
    Serial.println("[CAM CFG] " + ack);
    return true;
  }

  if (cmd.startsWith("CAM:CONFIG:SERVO_STEP:")) {
    long v = camParseLastLong(cmd);
    camConfiguredServoStep = (int)camClampLong(v, CAM_MIN_SERVO_STEP, CAM_MAX_SERVO_STEP);

    String ack = "ACK:CONFIG:SERVO_STEP:" + String(camConfiguredServoStep);
    webSocket.broadcastTXT(ack);
    Serial.println("[CAM CFG] " + ack);
    return true;
  }

  if (cmd == "CAM:CONFIG:RESET") {
    camConfiguredStepperSteps = CAM_DEFAULT_STEPPER_STEPS;
    camConfiguredServoStep    = CAM_DEFAULT_SERVO_STEP;
    webSocket.broadcastTXT("ACK:CONFIG:RESET");
    return true;
  }

  if (cmd == "CAM:CONFIG:STATUS") {
    String status = "CONFIG:STEPPER_STEPS=" + String(camConfiguredStepperSteps)
                    + ";SERVO_STEP=" + String(camConfiguredServoStep);
    webSocket.broadcastTXT(status);
    return true;
  }

  return false;
}


// =================================================================
// Camera stand - main command handler
// =================================================================
void handleCameraCommand(const String& cmd) {
  if (handleCamConfigCommand(cmd)) return;

  if (cmd == "CAM:STOP") {
    stopCameraMotion();
    return;
  }

  // --- Continuous servo ---
  if (cmd == "CAM:SERVO:STOP") {
    cameraServoMoveDir = 0;
    scheduleCameraDetach();
    return;
  }

  if (cmd == "CAM:SERVO:MOVE_UP") {
    cameraServoMoveDir = INVERT_CAMERA_VERTICAL ? -1 : 1;
    attachCameraServoIfNeeded();
    pendingCameraDetach = false;
    return;
  }

  if (cmd == "CAM:SERVO:MOVE_DOWN") {
    cameraServoMoveDir = INVERT_CAMERA_VERTICAL ? 1 : -1;
    attachCameraServoIfNeeded();
    pendingCameraDetach = false;
    return;
  }

  // --- Servo center / step / absolute ---
  if (cmd == "CAM:CENTER") {
    stopCameraMotion();
    setCameraAngle(CAM_SERVO_CENTER);
    return;
  }

  if (cmd == "CAM:UP") {
    int delta = INVERT_CAMERA_VERTICAL ? -camConfiguredServoStep : camConfiguredServoStep;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (cmd == "CAM:DOWN") {
    int delta = INVERT_CAMERA_VERTICAL ? camConfiguredServoStep : -camConfiguredServoStep;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (cmd.startsWith("CAM:UP:")) {
    int step = camParseServoStepOrDefault(cmd);
    int delta = INVERT_CAMERA_VERTICAL ? -step : step;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (cmd.startsWith("CAM:DOWN:")) {
    int step = camParseServoStepOrDefault(cmd);
    int delta = INVERT_CAMERA_VERTICAL ? step : -step;
    setCameraAngle(cameraServoAngle + delta);
    return;
  }

  if (cmd.startsWith("CAM:ANGLE:")) {
    setCameraAngle((int)camParseLastLong(cmd));
    return;
  }

  // --- Pan stepper (continuous) ---
  if (cmd == "CAM:LEFT") {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW);
    camEnableDriver();
    return;
  }

  if (cmd == "CAM:RIGHT") {
    cameraFiniteStepsRemaining = 0;
    cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH);
    camEnableDriver();
    return;
  }

  // --- Pan stepper (finite steps) ---
  if (cmd == "CAM:STEP_LEFT" || cmd.startsWith("CAM:STEP_LEFT:")) {
    cameraFiniteStepsRemaining = camParseStepsOrDefault(cmd);
    cameraStepperDirection = -1;
    digitalWrite(CAM_DIR_PIN, LOW);
    camEnableDriver();
    return;
  }

  if (cmd == "CAM:STEP_RIGHT" || cmd.startsWith("CAM:STEP_RIGHT:")) {
    cameraFiniteStepsRemaining = camParseStepsOrDefault(cmd);
    cameraStepperDirection = 1;
    digitalWrite(CAM_DIR_PIN, HIGH);
    camEnableDriver();
    return;
  }

  // --- Zero / speed ---
  if (cmd == "CAM:ZERO") {
    cameraStepPosition = 0;
    return;
  }

  if (cmd.startsWith("CAM:SPEED:")) {
    long interval = camParseLastLong(cmd);
    if (interval < 0) interval = -interval;
    cameraStepIntervalMicros = constrain(interval, 300, 5000);
    return;
  }
}


// =================================================================
// Camera stand - non-blocking runners (called every loop)
// =================================================================
void runCameraStepper() {
  if (cameraStepperDirection == 0) return;

  unsigned long now = micros();
  if (now - cameraLastStepTime < cameraStepIntervalMicros) return;
  cameraLastStepTime = now;

  digitalWrite(CAM_STEP_PIN, HIGH);
  delayMicroseconds(CAM_STEP_PULSE_MICROS);
  digitalWrite(CAM_STEP_PIN, LOW);

  cameraStepPosition += cameraStepperDirection;

  if (cameraFiniteStepsRemaining > 0) {
    cameraFiniteStepsRemaining--;
    if (cameraFiniteStepsRemaining == 0) {
      stopCameraMotion();
    }
  }
}

void runCameraServo() {
  if (cameraServoMoveDir == 0) return;

  unsigned long now = millis();
  if (now - lastCamContinuousMoveMs < CAM_CONTINUOUS_SERVO_INTERVAL_MS) return;
  lastCamContinuousMoveMs = now;

  int stepSize = camClampInt(camConfiguredServoStep, CAM_MIN_SERVO_STEP, CAM_MAX_SERVO_STEP);
  int nextAngle = camClampInt(cameraServoAngle + (cameraServoMoveDir * stepSize),
                               CAM_SERVO_MIN, CAM_SERVO_MAX);
  setCameraAngle(nextAngle);

  if (nextAngle == CAM_SERVO_MIN || nextAngle == CAM_SERVO_MAX) {
    cameraServoMoveDir = 0;
    scheduleCameraDetach();
  }
}

void handleCameraDetach() {
  if (!pendingCameraDetach) return;
  if (cameraServoMoveDir != 0) return;
  if (millis() - cameraDetachStartMs < CAM_SERVO_DETACH_DELAY_MS) return;
  detachCameraServo();
}


// =================================================================
// Command routing
// =================================================================
void routeCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  lastCommand = cmd;
  lastCommandAt = millis();

  // Mirror every command that reaches the ESP32 to Raspberry, before routing.
  mirrorCommandToRaspberry("IN", "ESP32_ROUTER", cmd);

  // One-command automatic scenario from mobile dashboard / Raspberry.
  if (cmd == "AUTO:FULL_SCENARIO" || cmd == "AUTO:START_FULL_SCENARIO" || cmd == "FULL_AUTO:START") {
    startFullAutoScenario();
    return;
  }

  if (cmd == "AUTO:FULL_STATUS" || cmd == "FULL_AUTO:STATUS") {
    broadcastFullAutoEvent("full_auto_status", fullAutoState);
    return;
  }

  if (cmd == "AUTO:FULL_STOP" || cmd == "FULL_AUTO:STOP") {
    cancelFullAutoScenario("USER_FULL_AUTO_STOP");
    return;
  }

  // System mode commands go to both Mega and UNO.
  if (cmd == "SYS:MODE:MANUAL") {
    cancelFullAutoScenario("SYS_MODE_MANUAL");
    currentMode = "MANUAL";
    sendToMega(cmd);
    sendToUno(cmd);
    stopCameraMotion();

    Serial.println("[MODE] MANUAL");
    broadcastStatusEvent("mode_changed");
    return;
  }

  if (cmd == "SYS:MODE:AUTO") {
    currentMode = "AUTO";
    sendToMega(cmd);
    sendToUno(cmd);
    stopCameraMotion();

    Serial.println("[MODE] AUTO");
    broadcastStatusEvent("mode_changed");
    return;
  }

  // STOP / ESTOP: stop camera locally, forward to both boards.
  if (cmd == "STOP" || cmd == "ESTOP") {
    cancelFullAutoScenario("STOP_OR_ESTOP");
    stopCameraMotion();
    sendToMega(cmd);
    sendToUno(cmd);
    return;
  }

  // Camera commands are handled entirely on ESP32.
  if (cmd.startsWith("CAM:")) {
    mirrorCommandToRaspberry("LOCAL", "CAMERA_STAND", cmd);
    Serial.print("[CAM LOCAL] ");
    Serial.println(cmd);
    handleCameraCommand(cmd);
    return;
  }

  // Arm commands go to UNO.
  if (cmd.startsWith("ARM:")) {
    sendToUno(cmd);
    return;
  }

  // Mega LEGO blocks and auto blocks.
  if (cmd.startsWith("AUTO:") || cmd.startsWith("BLOCK:")) {
    sendToMega(cmd);
    return;
  }

  // Everything else (motors, speed, jacks, PULSE, status) goes to Mega.
  sendToMega(cmd);
}


// =================================================================
// HTTP handlers
// =================================================================
void handleRoot() {
  String text = "";

  text += "Apex Rover ESP32 Bridge + Camera Stand Running\n";
  text += "Mode: " + currentMode + "\n";
  text += "WebSocket: ws://192.168.4.1:81\n";
  text += "HTTP: http://192.168.4.1\n";
  text += "\nEndpoints:\n";
  text += "/get_status\n";
  text += "/auto_status\n";
  text += "Raspberry UDP mirror: 192.168.4.2:5055\n";
  text += "Sensor path: Mega Serial1 -> ESP32 -> Mobile WebSocket\n";
  text += "/sensor_update?pitch=2.4&roll=-1.1&front=35.6&rear=18.2&balance=STABLE (debug only)\n";
  text += "/bridge_event?source=MEGA&line=ACK:MEGA:DONE:UP_STAIRS (debug only)\n";
  text += "/command?cmd=STOP\n";
  text += "/command?cmd=CAM:CENTER\n";
  text += "/command?cmd=CAM:LEFT\n";
  text += "/command?cmd=CAM:RIGHT\n";
  text += "/command?cmd=CAM:UP\n";
  text += "/command?cmd=CAM:DOWN\n";
  text += "/command?cmd=CAM:SERVO:MOVE_UP\n";
  text += "/command?cmd=CAM:SERVO:MOVE_DOWN\n";
  text += "/command?cmd=CAM:SERVO:STOP\n";
  text += "/command?cmd=CAM:STEP_LEFT:200\n";
  text += "/command?cmd=CAM:STEP_RIGHT:200\n";
  text += "/command?cmd=CAM:ANGLE:90\n";
  text += "/command?cmd=CAM:SPEED:500\n";
  text += "/command?cmd=CAM:CONFIG:STEPPER_STEPS:200\n";
  text += "/command?cmd=CAM:CONFIG:SERVO_STEP:5\n";
  text += "/command?cmd=SYS:MODE:AUTO\n";
  text += "/command?cmd=PULSE:FORWARD:560\n";
  text += "/command?cmd=BLOCK:TURN:LEFT:90\n";
  text += "/command?cmd=AUTO:UP_STAIRS\n";
  text += "/command?cmd=AUTO:FULL_SCENARIO\n";
  text += "Full scenario: UP_STAIRS -> GO_FORWARD_4 -> TURN_RIGHT_90 -> GO_FORWARD_1 -> DOWN_STAIRS\n";

  httpServer.send(200, "text/plain", text);
}

void handleGetStatus() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"cam_servo_angle\":" + String(cameraServoAngle) + ",";
  json += "\"cam_stepper_pos\":" + String(cameraStepPosition) + ",";
  json += "\"cam_stepper_dir\":" + String(cameraStepperDirection) + ",";
  json += "\"cam_servo_move_dir\":" + String(cameraServoMoveDir) + ",";
  json += "\"full_auto_active\":" + String(fullAutoActive ? "true" : "false") + ",";
  json += "\"full_auto_state\":\"" + jsonEscape(fullAutoState) + "\",";
  json += "\"full_auto_step\":" + String(fullAutoStepNumber) + ",";
  json += "\"full_auto_stage\":\"" + jsonEscape(fullAutoLastStage) + "\",";
  json += "\"full_auto_command\":\"" + jsonEscape(fullAutoCurrentCommand) + "\",";
  json += "\"full_auto_expected_done\":\"" + jsonEscape(fullAutoExpectedDonePrefix) + "\",";
  json += "\"full_auto_last_ack\":\"" + jsonEscape(fullAutoLastAck) + "\",";
  json += "\"full_auto_last_error\":\"" + jsonEscape(fullAutoLastError) + "\",";
  json += "\"raspberry_mirror_enabled\":" + String(RASPBERRY_COMMAND_MIRROR_ENABLED ? "true" : "false") + ",";
  json += "\"raspberry_mirror_count\":" + String(raspberryMirrorCount) + ",";
  json += "\"raspberry_mirror_last_ok\":" + String(lastRaspberryMirrorOk ? "true" : "false") + ",";
  json += "\"raspberry_mirror_last_target\":\"" + jsonEscape(lastRaspberryMirrorTarget) + "\",";
  json += "\"raspberry_mirror_last_command\":\"" + jsonEscape(lastRaspberryMirrorCommand) + "\",";
  json += "\"raspberry_mirror_last_ms_ago\":" + String(millis() - lastRaspberryMirrorAt) + ",";
  json += "\"last_command\":\"" + jsonEscape(lastCommand) + "\",";
  json += "\"last_command_ms_ago\":" + String(millis() - lastCommandAt) + ",";
  json += "\"last_sensor\":\"" + jsonEscape(lastSensorMessage) + "\",";
  json += "\"last_bridge_source\":\"" + jsonEscape(lastBridgeSource) + "\",";
  json += "\"last_bridge_event\":\"" + jsonEscape(lastBridgeEvent) + "\",";
  json += "\"last_bridge_event_ms_ago\":" + String(millis() - lastBridgeEventAt) + ",";
  json += "\"last_mega_ack\":\"" + jsonEscape(lastMegaAck) + "\",";
  json += "\"last_mega_error\":\"" + jsonEscape(lastMegaError) + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}

void handleAutoStatus() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"active\":" + String(fullAutoActive ? "true" : "false") + ",";
  json += "\"state\":\"" + jsonEscape(fullAutoState) + "\",";
  json += "\"step\":" + String(fullAutoStepNumber) + ",";
  json += "\"stage\":\"" + jsonEscape(fullAutoLastStage) + "\",";
  json += "\"command\":\"" + jsonEscape(fullAutoCurrentCommand) + "\",";
  json += "\"expected_done\":\"" + jsonEscape(fullAutoExpectedDonePrefix) + "\",";
  json += "\"last_ack\":\"" + jsonEscape(fullAutoLastAck) + "\",";
  json += "\"last_error\":\"" + jsonEscape(fullAutoLastError) + "\",";
  json += "\"last_sensor\":\"" + jsonEscape(lastSensorMessage) + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}

void handleSensorUpdate() {
  String pitch   = httpServer.hasArg("pitch")   ? httpServer.arg("pitch")   : "0";
  String roll    = httpServer.hasArg("roll")    ? httpServer.arg("roll")    : "0";
  String front   = httpServer.hasArg("front")   ? httpServer.arg("front")   : "-1";
  String rear    = httpServer.hasArg("rear")    ? httpServer.arg("rear")    : "-1";
  String balance = httpServer.hasArg("balance") ? httpServer.arg("balance") : "NO DATA";

  balance.toUpperCase();

  String sensorMessage = "SENSOR:";
  sensorMessage += "PITCH=" + pitch;
  sensorMessage += ";ROLL=" + roll;
  sensorMessage += ";FRONT=" + front;
  sensorMessage += ";REAR=" + rear;
  sensorMessage += ";BALANCE=" + balance;

  lastSensorMessage = sensorMessage;
  webSocket.broadcastTXT(sensorMessage);

  Serial.print("[SENSOR -> APP] ");
  Serial.println(sensorMessage);

  String json = "{\"ok\":true,\"broadcast\":\"" + jsonEscape(sensorMessage) + "\"}";
  httpServer.send(200, "application/json", json);
}

void handleBridgeEvent() {
  String source = httpServer.hasArg("source") ? httpServer.arg("source") : "UNKNOWN";
  String line   = httpServer.hasArg("line")   ? httpServer.arg("line")   : "";

  source = urlDecode(source);
  line   = urlDecode(line);
  source.trim();
  line.trim();

  if (line.length() == 0) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"empty line\"}");
    return;
  }

  lastBridgeSource = source;
  lastBridgeEvent  = line;
  lastBridgeEventAt = millis();

  if (source == "MEGA" && line.startsWith("ACK:"))   lastMegaAck   = line;
  if (source == "MEGA" && (line.startsWith("ERR:") || line.startsWith("ERROR:"))) lastMegaError = line;

  if (source == "MEGA" || source == "MEGA_DIRECT") {
    handleFullAutoFeedbackLine(line);
  }

  Serial.print("[BRIDGE EVENT ");
  Serial.print(source);
  Serial.print("] ");
  Serial.println(line);

  broadcastBridgeLine(source, line);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"source\":\"" + jsonEscape(source) + "\",";
  json += "\"line\":\"" + jsonEscape(line) + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}

void handleCommandHttp() {
  if (!httpServer.hasArg("cmd")) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd parameter\"}");
    return;
  }

  String command = urlDecode(httpServer.arg("cmd"));
  command.trim();

  if (command.length() == 0) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"empty command\"}");
    return;
  }

  Serial.print("[HTTP CMD] ");
  Serial.println(command);

  routeCommand(command);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"command\":\"" + jsonEscape(command) + "\",";
  json += "\"note\":\"cam_handled_locally;arm_and_mega_forwarded\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}


// =================================================================
// WebSocket handler
// =================================================================
void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(clientId);

      Serial.print("[WS] Connected client=");
      Serial.print(clientId);
      Serial.print(" IP=");
      Serial.println(ip);

      String msg = "{";
      msg += "\"event\":\"connected\",";
      msg += "\"mode\":\"" + currentMode + "\",";
      msg += "\"manual_only\":false";
      msg += "}";

      webSocket.sendTXT(clientId, msg);
      webSocket.sendTXT(clientId, lastSensorMessage);

      if (lastBridgeEvent != "NONE") {
        webSocket.sendTXT(clientId, lastBridgeEvent);
      }

      break;
    }

    case WStype_DISCONNECTED:
      Serial.print("[WS] Disconnected client=");
      Serial.println(clientId);

      // Safety stop on disconnect.
      routeCommand("STOP");
      routeCommand("JACK:ALL:STOP");
      routeCommand("CAM:STOP");       // handled locally
      routeCommand("ARM:BASE:STOP");
      routeCommand("ARM:STOP");
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();

      if (command.length() == 0) return;

      Serial.print("[WS CMD] ");
      Serial.println(command);

      routeCommand(command);
      break;
    }

    default:
      break;
  }
}


// =================================================================
// Setup
// =================================================================
void setup() {
  Serial.begin(115200);

  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  UnoSerial.begin(9600, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

  // Camera stepper pins
  pinMode(CAM_EN_PIN,   OUTPUT);
  pinMode(CAM_STEP_PIN, OUTPUT);
  pinMode(CAM_DIR_PIN,  OUTPUT);

  digitalWrite(CAM_STEP_PIN, LOW);
  digitalWrite(CAM_DIR_PIN,  LOW);
  camDisableDriver();

  // Camera servo - centre on boot
  setCameraAngle(CAM_SERVO_CENTER);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  raspberryCommandUdp.begin(ESP32_COMMAND_UDP_LOCAL_PORT);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/",              handleRoot);
  httpServer.on("/get_status",    handleGetStatus);
  httpServer.on("/auto_status",   handleAutoStatus);
  httpServer.on("/sensor_update", handleSensorUpdate);
  httpServer.on("/bridge_event",  handleBridgeEvent);
  httpServer.on("/command",       handleCommandHttp);
  httpServer.begin();

  currentMode = "MANUAL";
  sendToMega("SYS:MODE:MANUAL");
  sendToUno("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Bridge + Camera Stand");
  Serial.println("Mode     : MANUAL");
  Serial.println("CAM      : handled locally on ESP32");
  Serial.println("ARM      : forwarded to UNO");
  Serial.println("MEGA cmds: forwarded to Mega");
  Serial.print("SSID     : "); Serial.println(WIFI_SSID);
  Serial.print("AP IP    : "); Serial.println(WiFi.softAPIP());
  Serial.print("RPI UDP  : "); Serial.print(RASPBERRY_COMMAND_IP); Serial.print(":"); Serial.println(RASPBERRY_COMMAND_UDP_PORT);
  Serial.println("WebSocket: ws://192.168.4.1:81");
  Serial.println("HTTP     : http://192.168.4.1");
  Serial.println("=========================================");
}


// =================================================================
// Loop
// =================================================================
void loop() {
  webSocket.loop();
  httpServer.handleClient();

  readMegaDirectFeedback();
  runFullAutoScenario();

  // Camera stand runners - non-blocking
  runCameraStepper();
  runCameraServo();
  handleCameraDetach();
}