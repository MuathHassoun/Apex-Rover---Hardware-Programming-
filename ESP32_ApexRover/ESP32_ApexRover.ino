#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge
//
// Architecture:
//   Mobile App   ->  WebSocket (port 81)  ->  ESP32
//   Raspberry Pi ->  HTTP      (port 80)  ->  ESP32
//   ESP32        ->  Serial2              ->  Arduino Mega
//   ESP32        ->  Serial1              ->  Arduino UNO
//
// Modes:
//   MANUAL : Mobile app controls the robot. Raspberry commands blocked.
//   AUTO   : Raspberry Pi controls the robot. Mobile drive commands blocked.
//
// Routing rules:
//   CAM:*          -> UNO  (stepper + servo)
//   ARM:*          -> UNO
//   Everything else -> Mega (motors, jacks, sensors, mode)
//
// Commands always allowed from any source and any mode:
//   SYS:MODE:MANUAL
//   SYS:MODE:AUTO
//   STOP  /  ESTOP  /  CMD:STOP
//   JACK:FRONT:STOP  /  JACK:REAR:STOP  /  JACK:ALL:STOP
//   CAM:STOP
// =================================================================

// -----------------------------------------------------------------
// WiFi credentials
// -----------------------------------------------------------------
const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";

// -----------------------------------------------------------------
// Serial pin mapping
//   Serial2: ESP32 GPIO17 TX  ->  Mega  RX1 (pin 19)
//   Serial1: ESP32 GPIO4  TX  ->  UNO   D2  (SoftwareSerial RX)
// -----------------------------------------------------------------
#define MEGA_RX_PIN  16
#define MEGA_TX_PIN  17
#define UNO_RX_PIN   15   // not wired; kept so HardwareSerial init works
#define UNO_TX_PIN    4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

// -----------------------------------------------------------------
// Server objects
// -----------------------------------------------------------------
WebSocketsServer webSocket = WebSocketsServer(81);
WebServer        httpServer(80);

// -----------------------------------------------------------------
// System state
// -----------------------------------------------------------------
String currentMode = "MANUAL";   // default on power-up

// =================================================================
// Low-level send helpers
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

// Wait up to timeoutMs for one complete line from the Mega.
// Returns an empty string if nothing arrives in time.
String readMegaLine(unsigned int timeoutMs) {
  unsigned long start = millis();
  String buffer = "";

  while (millis() - start < timeoutMs) {
    while (MegaSerial.available()) {
      char c = MegaSerial.read();
      if (c == '\n') {
        buffer.trim();
        return buffer;
      }
      buffer += c;
    }
    delay(1);
  }

  buffer.trim();
  return buffer;
}

// =================================================================
// Command classification helpers
// =================================================================

// Returns true for the two system-mode switch commands.
bool isSystemModeCommand(const String& cmd) {
  return cmd == "SYS:MODE:MANUAL" || cmd == "SYS:MODE:AUTO";
}

// Returns true for any command that must execute regardless of mode
// (hard stops and mode switches).
bool isAlwaysAllowed(const String& cmd) {
  return isSystemModeCommand(cmd)     ||
         cmd == "STOP"                ||
         cmd == "ESTOP"               ||
         cmd == "CMD:STOP"            ||
         cmd == "JACK:FRONT:STOP"     ||
         cmd == "JACK:REAR:STOP"      ||
         cmd == "JACK:ALL:STOP"       ||
         cmd == "JACK:STOP"           ||
         cmd == "CAM:STOP"            ||
         cmd == "ARM:BASE:STOP";
}

// =================================================================
// Central router
// Decides which serial bus receives the command and applies the
// mode switch if the command is SYS:MODE:*.
// =================================================================
void routeCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  // Apply mode change locally first.
  if (cmd == "SYS:MODE:MANUAL") {
    currentMode = "MANUAL";
    sendToMega(cmd);
    sendToUno(cmd);
    Serial.println("[MODE] Switched to MANUAL");
    return;
  }
  if (cmd == "SYS:MODE:AUTO") {
    currentMode = "AUTO";
    sendToMega(cmd);
    sendToUno(cmd);
    Serial.println("[MODE] Switched to AUTO");
    return;
  }

  // Camera stand and arm commands go to the UNO.
  if (cmd.startsWith("CAM:") || cmd.startsWith("ARM:")) {
    sendToUno(cmd);
    return;
  }

  // Everything else (movement, speed, jacks, sensors, robot modes) goes to the Mega.
  sendToMega(cmd);
}

// =================================================================
// HTTP URL helper — decode the most common percent-encoded chars
// =================================================================
String urlDecode(String s) {
  s.replace("%3A", ":");
  s.replace("%3a", ":");
  s.replace("%2F", "/");
  s.replace("%2f", "/");
  s.replace("%20", " ");
  s.replace("+",   " ");
  return s;
}

// =================================================================
// HTTP handlers  (Raspberry Pi interface)
// =================================================================

// GET /
void handleRoot() {
  server.send(200, "text/plain",
    "Apex Rover ESP32 Bridge | mode=" + currentMode);
}

// GET /get_status
// Returns current mode as JSON.
void handleGetStatus() {
  String json = "{\"ok\":true,\"mode\":\"" + currentMode + "\"}";
  httpServer.send(200, "application/json", json);
}

// GET /get_sensors
// Always allowed (read-only). Asks the Mega for a fresh sensor line
// and returns it as JSON: {"ok":true,"raw":"DATA:PITCH=..."}
void handleGetSensors() {
  sendToMega("GET:SENSORS");
  String line = readMegaLine(300);

  String json = "{\"ok\":";
  json += (line.length() > 0) ? "true" : "false";
  json += ",\"raw\":\"" + line + "\"}";
  httpServer.send(200, "application/json", json);
}

// POST /command  (param: cmd=<command>)
// Main entry point for all Raspberry Pi commands.
void handleCommandHttp() {
  if (!httpServer.hasArg("cmd")) {
    httpServer.send(400, "application/json",
      "{\"ok\":false,\"error\":\"missing cmd parameter\"}");
    return;
  }

  String command = urlDecode(httpServer.arg("cmd"));
  command.trim();

  if (command.length() == 0) {
    httpServer.send(400, "application/json",
      "{\"ok\":false,\"error\":\"empty command\"}");
    return;
  }

  Serial.print("[HTTP from Pi] ");
  Serial.println(command);

  // Hard-stop and mode-switch commands are always executed.
  if (isAlwaysAllowed(command)) {
    routeCommand(command);
    httpServer.send(200, "application/json",
      "{\"ok\":true,\"type\":\"always_allowed\"}");
    return;
  }

  // In MANUAL mode the Raspberry Pi is not the active controller;
  // block its movement/camera/jack commands.
  if (currentMode == "MANUAL") {
    httpServer.send(403, "application/json",
      "{\"ok\":false,\"error\":\"system is MANUAL; command blocked\"}");
    Serial.println("[BLOCKED Pi] system is MANUAL");
    return;
  }

  // AUTO mode: route the command normally.
  routeCommand(command);
  httpServer.send(200, "application/json", "{\"ok\":true}");
}

// =================================================================
// WebSocket handler  (Mobile App interface)
// =================================================================
void webSocketEvent(uint8_t clientId, WStype_t type,
                    uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.print("[WS] App connected, client=");
      Serial.println(clientId);
      webSocket.sendTXT(clientId,
        "{\"event\":\"connected\",\"mode\":\"" + currentMode + "\"}");
      break;

    case WStype_DISCONNECTED:
      Serial.print("[WS] App disconnected, client=");
      Serial.println(clientId);
      // Safety stop when the mobile app drops its connection.
      routeCommand("STOP");
      routeCommand("JACK:ALL:STOP");
      routeCommand("CAM:STOP");
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();

      Serial.print("[WS from App] ");
      Serial.println(command);

      // Hard-stop and mode-switch always go through.
      if (isAlwaysAllowed(command)) {
        routeCommand(command);
        return;
      }

      // In AUTO mode the robot is driven by the Raspberry Pi;
      // block app drive/camera/jack commands to avoid conflicts.
      if (currentMode == "AUTO") {
        webSocket.sendTXT(clientId,
          "{\"ok\":false,\"error\":\"system is AUTO; command blocked\"}");
        Serial.println("[BLOCKED App] system is AUTO");
        return;
      }

      // MANUAL mode: route the app command normally.
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
  UnoSerial.begin(9600, SERIAL_8N1, UNO_RX_PIN,  UNO_TX_PIN);

  // Start WiFi access point.
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  // Start WebSocket server for the mobile app.
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Register HTTP endpoints for the Raspberry Pi.
  httpServer.on("/",            handleRoot);
  httpServer.on("/get_status",  handleGetStatus);
  httpServer.on("/get_sensors", handleGetSensors);
  httpServer.on("/command",     handleCommandHttp);
  httpServer.begin();

  // Ensure both Arduinos start in MANUAL mode.
  routeCommand("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Bridge Ready");
  Serial.print  ("SSID     : "); Serial.println(WIFI_SSID);
  Serial.print  ("AP IP    : "); Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket: port 81");
  Serial.println("HTTP     : port 80");
  Serial.println("Default  : MANUAL");
  Serial.println("=========================================");
}

// =================================================================
// Loop
// =================================================================
void loop() {
  webSocket.loop();
  httpServer.handleClient();
}
