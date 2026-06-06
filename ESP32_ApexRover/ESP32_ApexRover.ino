#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
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
// Sensor path:
//   Mega -> Raspberry sensor_bridge -> ESP32 /sensor_update -> Mobile App
//
// Mega ACK path WITHOUT return wire:
//   Mega USB Serial -> Raspberry sensor_bridge -> ESP32 /bridge_event -> Mobile App
//
// Routing:
//   CAM:*        -> handled locally on ESP32 (camera stand)
//   ARM:*        -> UNO
//   AUTO:* / BLOCK:* / movement / jack / speed -> Mega
//
// Camera Stand pins (ESP32):
//   Camera A4988 EN       -> ESP32 GPIO25
//   Camera A4988 STEP     -> ESP32 GPIO26
//   Camera A4988 DIR      -> ESP32 GPIO27
//   Camera Servo Signal   -> ESP32 GPIO14
//
// Serial pin mapping:
//   Mega command direction only:
//     ESP32 GPIO17 TX -> Mega RX1 Pin 19
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

const int CAM_SERVO_MIN    = 30;
const int CAM_SERVO_MAX    = 150;
const int CAM_SERVO_CENTER = 90;
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
// Serial helpers
// =================================================================
void sendToMega(const String& cmd) {
  MegaSerial.println(cmd);
  Serial.print("[-> MEGA] ");
  Serial.println(cmd);
}

void sendToUno(const String& cmd) {
  UnoSerial.println(cmd);
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

  // System mode commands go to both Mega and UNO.
  if (cmd == "SYS:MODE:MANUAL") {
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
    stopCameraMotion();
    sendToMega(cmd);
    sendToUno(cmd);
    return;
  }

  // Camera commands are handled entirely on ESP32.
  if (cmd.startsWith("CAM:")) {
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
  text += "/sensor_update?pitch=2.4&roll=-1.1&front=35.6&rear=18.2&balance=STABLE\n";
  text += "/bridge_event?source=MEGA&line=ACK:MEGA:DONE:UP_STAIRS\n";
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

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/",              handleRoot);
  httpServer.on("/get_status",    handleGetStatus);
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

  // Camera stand runners - non-blocking
  runCameraStepper();
  runCameraServo();
  handleCameraDetach();
}
