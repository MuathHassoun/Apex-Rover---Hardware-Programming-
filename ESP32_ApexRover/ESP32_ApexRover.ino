#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// ==================================================
// ESP32 - Apex Rover Central WiFi Bridge
//
// Architecture:
// Mobile App  -> WebSocket -> ESP32
// Raspberry Pi -> HTTP     -> ESP32
// ESP32 -> Mega using Serial2
// ESP32 -> UNO  using Serial1
//
// MANUAL mode:
// - Mobile commands are accepted.
// - Raspberry commands are rejected except status/sensor reads.
//
// AUTO mode:
// - Raspberry commands are accepted.
// - Mobile drive/camera/jack commands are rejected for safety.
// - Mobile can still send SYS:MODE:MANUAL and STOP.
//
// Default mode = MANUAL
// ==================================================

const char* ssid = "Apex_Rover_Net";
const char* password = "12345678";

WebSocketsServer webSocket = WebSocketsServer(81);
WebServer server(80);

String currentMode = "MANUAL";

// ==================================================
// Serial pins
// ==================================================
// Mega:
// ESP32 GPIO17 TX -> Mega RX1 Pin 19
// ESP32 GND       -> Mega GND
#define MEGA_RX 16
#define MEGA_TX 17

// UNO:
// ESP32 GPIO4 TX -> UNO D2 SoftwareSerial RX
// ESP32 GND      -> UNO GND
#define UNO_RX 15   // not used now
#define UNO_TX 4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

// ==================================================
// Helpers
// ==================================================
String urlDecode(String input) {
  input.replace("%3A", ":");
  input.replace("%2F", "/");
  input.replace("%20", " ");
  return input;
}

bool isSystemModeCommand(const String &cmd) {
  return cmd == "SYS:MODE:MANUAL" || cmd == "SYS:MODE:AUTO";
}

bool isEmergencyCommand(const String &cmd) {
  return cmd == "STOP" || cmd == "CMD:STOP" || cmd == "ESTOP" ||
         cmd == "JACK:ALL:STOP" || cmd == "JACK:STOP" ||
         cmd == "JACK:FRONT:STOP" || cmd == "JACK:REAR:STOP" ||
         cmd == "CAM:STOP" || cmd == "ARM:BASE:STOP";
}

void sendToMega(String command) {
  command.trim();
  if (command.length() == 0) return;
  MegaSerial.println(command);
  Serial.print("[TO MEGA] ");
  Serial.println(command);
}

void sendToUno(String command) {
  command.trim();
  if (command.length() == 0) return;
  UnoSerial.println(command);
  Serial.print("[TO UNO] ");
  Serial.println(command);
}

void routeCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  if (isSystemModeCommand(command)) {
    if (command == "SYS:MODE:MANUAL") currentMode = "MANUAL";
    if (command == "SYS:MODE:AUTO") currentMode = "AUTO";

    // Tell both controllers the active mode.
    sendToMega(command);
    sendToUno(command);
    return;
  }

  // Camera and old arm commands go to UNO.
  if (command.startsWith("CAM:") || command.startsWith("ARM:")) {
    sendToUno(command);
    return;
  }

  // Movement, speed, jacks, sensor requests go to Mega.
  sendToMega(command);
}

String readMegaLine(unsigned long timeoutMs) {
  unsigned long start = millis();
  String line = "";
  while (millis() - start < timeoutMs) {
    if (MegaSerial.available() > 0) {
      line = MegaSerial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) return line;
    }
    delay(2);
  }
  return "";
}

// ==================================================
// WebSocket commands from Mobile App
// ==================================================
void handleMobileCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  Serial.print("[MOBILE WS] ");
  Serial.println(command);

  // Mode change is always allowed from mobile.
  if (isSystemModeCommand(command)) {
    routeCommand(command);
    return;
  }

  // STOP is always allowed for safety.
  if (isEmergencyCommand(command)) {
    routeCommand(command);
    return;
  }

  // In AUTO mode, block manual mobile commands.
  if (currentMode == "AUTO") {
    Serial.print("[BLOCKED MOBILE IN AUTO] ");
    Serial.println(command);
    return;
  }

  routeCommand(command);
}

void webSocketEvent(uint8_t clientNumber, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.print("Mobile connected, client: ");
      Serial.println(clientNumber);
      webSocket.sendTXT(clientNumber, "ESP32 Connected | Mode=" + currentMode);
      break;

    case WStype_DISCONNECTED:
      Serial.print("Mobile disconnected, client: ");
      Serial.println(clientNumber);
      // Safety stop on disconnect.
      routeCommand("STOP");
      routeCommand("JACK:ALL:STOP");
      routeCommand("CAM:STOP");
      break;

    case WStype_TEXT:
      handleMobileCommand(String((char*)payload));
      break;

    default:
      break;
  }
}

// ==================================================
// HTTP endpoints for Raspberry Pi
// ==================================================
void handleRoot() {
  server.send(200, "text/plain", "Apex Rover ESP32 Bridge Running");
}

void handleGetStatus() {
  String json = "{";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleCommandHttp() {
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }

  String command = urlDecode(server.arg("cmd"));
  command.trim();

  Serial.print("[RASPBERRY HTTP] ");
  Serial.println(command);

  // Mode command through HTTP is allowed, but usually mobile sets mode.
  if (isSystemModeCommand(command)) {
    routeCommand(command);
    server.send(200, "application/json", "{\"ok\":true,\"source\":\"http\",\"type\":\"mode\"}");
    return;
  }

  // Safety stop is always allowed.
  if (isEmergencyCommand(command)) {
    routeCommand(command);
    server.send(200, "application/json", "{\"ok\":true,\"source\":\"http\",\"type\":\"emergency\"}");
    return;
  }

  // Raspberry controls only in AUTO.
  if (currentMode != "AUTO") {
    server.send(423, "application/json", "{\"ok\":false,\"error\":\"system is MANUAL; raspberry command blocked\"}");
    Serial.println("[BLOCKED RASPBERRY IN MANUAL]");
    return;
  }

  routeCommand(command);
  server.send(200, "application/json", "{\"ok\":true,\"source\":\"http\"}");
}

void handleGetSensors() {
  // Sensor request can be allowed in both modes because it is read-only.
  sendToMega("GET:SENSORS");
  String line = readMegaLine(300);

  String json = "{";
  json += "\"ok\":";
  json += (line.length() > 0 ? "true" : "false");
  json += ",\"raw\":\"" + line + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void setupHttpServer() {
  server.on("/", handleRoot);
  server.on("/get_status", handleGetStatus);
  server.on("/command", handleCommandHttp);
  server.on("/get_sensors", handleGetSensors);
  server.begin();
}

// ==================================================
// Setup / Loop
// ==================================================
void setup() {
  Serial.begin(115200);

  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX, MEGA_TX);
  UnoSerial.begin(9600, SERIAL_8N1, UNO_RX, UNO_TX);

  WiFi.softAP(ssid, password);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  setupHttpServer();

  Serial.println("====================================");
  Serial.println("ESP32 Apex Rover Central Bridge Ready");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("Password: "); Serial.println(password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket: port 81");
  Serial.println("HTTP: port 80");
  Serial.println("Default mode: MANUAL");
  Serial.println("====================================");

  routeCommand("SYS:MODE:MANUAL");
}

void loop() {
  webSocket.loop();
  server.handleClient();
}
