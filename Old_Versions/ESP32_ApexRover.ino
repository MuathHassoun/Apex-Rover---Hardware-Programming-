#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge
//
// Architecture:
//   Mobile App   ->  WebSocket port 81  ->  ESP32  ->  Mega / UNO
//   Raspberry Pi ->  HTTP port 80       ->  ESP32  (sensor relay only)
//
// Modes:
//   MANUAL (default): Mobile app controls the robot.
//   AUTO  : kept for compatibility but no Raspberry auto-control.
//
// Command routing:
//   CAM:* / ARM:*   ->  UNO   (stepper + servo)
//   Everything else ->  Mega  (motors, jacks)
//
// Sensor data flow:
//   Mega  ->  USB Serial  ->  Raspberry Pi
//   Raspberry Pi  ->  POST /sensor  ->  ESP32
//   ESP32  ->  WebSocket broadcast  ->  Mobile App
//
// Always-allowed commands (any source, any mode):
//   SYS:MODE:MANUAL / SYS:MODE:AUTO
//   STOP / ESTOP / CMD:STOP
//   JACK:*:STOP / JACK:ALL:STOP / JACK:STOP
//   CAM:STOP / ARM:BASE:STOP
// =================================================================

const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";

// Mega: ESP32 GPIO17 TX -> Mega RX1 pin 19
#define MEGA_RX_PIN 16
#define MEGA_TX_PIN 17

// UNO: ESP32 GPIO4 TX -> UNO D2 SoftwareSerial RX
#define UNO_RX_PIN  15   // not wired — kept for HardwareSerial init
#define UNO_TX_PIN   4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

WebSocketsServer webSocket   = WebSocketsServer(81);
WebServer        httpServer(80);

String currentMode = "MANUAL";

// Last sensor JSON received from Raspberry Pi (broadcast to app on arrival).
String lastSensorJson = "{}";


// =================================================================
// Serial helpers
// =================================================================
void sendToMega(const String& cmd) {
  MegaSerial.println(cmd);
  Serial.print("[-> MEGA] "); Serial.println(cmd);
}

void sendToUno(const String& cmd) {
  UnoSerial.println(cmd);
  Serial.print("[-> UNO] "); Serial.println(cmd);
}


// =================================================================
// Command helpers
// =================================================================
bool isAlwaysAllowed(const String& cmd) {
  return cmd == "SYS:MODE:MANUAL" ||
         cmd == "SYS:MODE:AUTO"   ||
         cmd == "STOP"            ||
         cmd == "ESTOP"           ||
         cmd == "CMD:STOP"        ||
         cmd == "JACK:FRONT:STOP" ||
         cmd == "JACK:REAR:STOP"  ||
         cmd == "JACK:ALL:STOP"   ||
         cmd == "JACK:STOP"       ||
         cmd == "CAM:STOP"        ||
         cmd == "ARM:BASE:STOP";
}

void routeCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "SYS:MODE:MANUAL") { currentMode = "MANUAL"; sendToMega(cmd); sendToUno(cmd); return; }
  if (cmd == "SYS:MODE:AUTO")   { currentMode = "AUTO";   sendToMega(cmd); sendToUno(cmd); return; }

  if (cmd.startsWith("CAM:") || cmd.startsWith("ARM:")) { sendToUno(cmd);  return; }
  sendToMega(cmd);
}

String urlDecode(String s) {
  s.replace("%3A", ":"); s.replace("%3a", ":");
  s.replace("%2F", "/"); s.replace("%2f", "/");
  s.replace("%20", " "); s.replace("+",   " ");
  return s;
}


// =================================================================
// HTTP handlers — Raspberry Pi interface
// =================================================================

// GET /
void handleRoot() {
  httpServer.send(200, "text/plain", "Apex Rover ESP32 | mode=" + currentMode);
}

// GET /get_status
void handleGetStatus() {
  String json = "{\"ok\":true,\"mode\":\"" + currentMode + "\"}";
  httpServer.send(200, "application/json", json);
}

// POST /command?cmd=...
// Mobile app commands forwarded by the Pi are NOT expected here anymore.
// This endpoint is kept for emergencies / mode switches from the Pi.
void handleCommandHttp() {
  if (!httpServer.hasArg("cmd")) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }

  String command = urlDecode(httpServer.arg("cmd"));
  command.trim();

  Serial.print("[HTTP from Pi] "); Serial.println(command);

  if (isAlwaysAllowed(command)) {
    routeCommand(command);
    httpServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  // All other Pi commands are blocked — manual mode is the only mode.
  httpServer.send(403, "application/json",
    "{\"ok\":false,\"error\":\"only always-allowed commands accepted from Pi\"}");
}

// POST /sensor
// Raspberry Pi forwards sensor data received from the Mega.
// ESP32 broadcasts it to all connected WebSocket clients (mobile app).
//
// Expected body param: data=SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
void handleSensorHttp() {
  if (!httpServer.hasArg("data")) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing data\"}");
    return;
  }

  String raw = urlDecode(httpServer.arg("data"));
  raw.trim();

  Serial.print("[SENSOR from Pi] "); Serial.println(raw);

  // Build a JSON object from the raw SENSOR: line for the mobile app.
  lastSensorJson = buildSensorJson(raw);

  // Broadcast to all connected WebSocket clients.
  webSocket.broadcastTXT(lastSensorJson);

  httpServer.send(200, "application/json", "{\"ok\":true}");
}

// GET /get_last_sensor
// Mobile app can poll this if WebSocket is not available.
void handleGetLastSensor() {
  httpServer.send(200, "application/json", lastSensorJson);
}


// =================================================================
// Sensor JSON builder
// Converts: SENSOR:PITCH=2.30;ROLL=-1.10;UF=8.50;UR=9.20;ALERT=NONE
// Into:     {"type":"sensor","pitch":2.30,"roll":-1.10,"uf":8.50,"ur":9.20,"alert":"NONE"}
// =================================================================
String buildSensorJson(const String& raw) {
  // Strip the "SENSOR:" prefix if present.
  String s = raw;
  if (s.startsWith("SENSOR:")) s = s.substring(7);

  String pitch = "0", roll = "0", uf = "-1", ur = "-1", alert = "NONE";

  // Parse key=value pairs separated by ';'
  int start = 0;
  while (start < (int)s.length()) {
    int end = s.indexOf(';', start);
    if (end < 0) end = s.length();

    String pair = s.substring(start, end);
    int eq = pair.indexOf('=');
    if (eq > 0) {
      String key = pair.substring(0, eq);
      String val = pair.substring(eq + 1);
      key.toUpperCase();
      if      (key == "PITCH") pitch = val;
      else if (key == "ROLL")  roll  = val;
      else if (key == "UF")    uf    = val;
      else if (key == "UR")    ur    = val;
      else if (key == "ALERT") alert = val;
    }

    start = end + 1;
  }

  String json = "{\"type\":\"sensor\"";
  json += ",\"pitch\":"  + pitch;
  json += ",\"roll\":"   + roll;
  json += ",\"uf\":"     + uf;
  json += ",\"ur\":"     + ur;
  json += ",\"alert\":\"" + alert + "\"}";
  return json;
}


// =================================================================
// WebSocket handler — Mobile App
// =================================================================
void webSocketEvent(uint8_t clientId, WStype_t type,
                    uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.print("[WS] App connected, client="); Serial.println(clientId);
      // Send current mode and last known sensor data on connect.
      webSocket.sendTXT(clientId,
        "{\"event\":\"connected\",\"mode\":\"" + currentMode + "\"}");
      if (lastSensorJson != "{}") {
        webSocket.sendTXT(clientId, lastSensorJson);
      }
      break;

    case WStype_DISCONNECTED:
      Serial.print("[WS] App disconnected, client="); Serial.println(clientId);
      // Safety stop when the controlling app disconnects in MANUAL mode.
      if (currentMode == "MANUAL") {
        routeCommand("STOP");
        routeCommand("JACK:ALL:STOP");
        routeCommand("CAM:STOP");
      }
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();
      if (command.length() == 0) return;

      Serial.print("[WS from App] "); Serial.println(command);

      // Always-allowed commands bypass mode check.
      if (isAlwaysAllowed(command)) {
        routeCommand(command);
        return;
      }

      // In AUTO mode only always-allowed commands are accepted from the app.
      // (AUTO mode is reserved for future use; manual is the only active mode.)
      if (currentMode == "AUTO") {
        webSocket.sendTXT(clientId,
          "{\"ok\":false,\"error\":\"system is AUTO; command blocked\"}");
        return;
      }

      // MANUAL mode: route command normally.
      routeCommand(command);
      break;
    }

    default:
      break;
  }
}


// =================================================================
// Setup & Loop
// =================================================================
void setup() {
  Serial.begin(115200);

  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  UnoSerial.begin(9600,  SERIAL_8N1, UNO_RX_PIN,  UNO_TX_PIN);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/",                handleRoot);
  httpServer.on("/get_status",      handleGetStatus);
  httpServer.on("/command",         handleCommandHttp);
  httpServer.on("/sensor",          handleSensorHttp);       // NEW: receive from Pi
  httpServer.on("/get_last_sensor", handleGetLastSensor);    // NEW: poll from app
  httpServer.begin();

  // Boot in MANUAL mode.
  routeCommand("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Bridge Ready");
  Serial.print("SSID  : "); Serial.println(WIFI_SSID);
  Serial.print("AP IP : "); Serial.println(WiFi.softAPIP());
  Serial.println("WS    : ws://192.168.4.1:81");
  Serial.println("HTTP  : http://192.168.4.1");
  Serial.println("NEW   : POST /sensor  -> broadcast to app");
  Serial.println("=========================================");
}

void loop() {
  webSocket.loop();
  httpServer.handleClient();
}
