#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge (Centralized Routing V3)
//
// Architecture:
//   Mobile App   ->  WebSocket (port 81)  ->  ESP32
//   Raspberry Pi ->  HTTP      (port 80)  ->  ESP32
//   ESP32        ->  Serial2              ->  Arduino Mega
//   ESP32        ->  Serial1              ->  Arduino UNO
//
// Modes:
//   MANUAL : Mobile app controls the robot. Raspberry commands & sensors BLOCKED.
//   AUTO   : Raspberry Pi controls the robot. Mobile drive commands blocked.
//
// Routing rules:
//   CAM:* -> UNO  (stepper + servo camera stand)
//   ARM:* -> UNO
//   Everything else  -> Mega (motors, jacks, sensors, mode)
// =================================================================

// -----------------------------------------------------------------
// WiFi credentials
// -----------------------------------------------------------------
const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";

// -----------------------------------------------------------------
// Serial pin mapping
// -----------------------------------------------------------------
#define MEGA_RX_PIN  16
#define MEGA_TX_PIN  17

#define UNO_RX_PIN   15   // Not wired now, only needed for HardwareSerial init
#define UNO_TX_PIN    4   // ESP32 GPIO4 / G4 -> UNO D2 / Pin 2

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

// -----------------------------------------------------------------
// Server objects
// -----------------------------------------------------------------
WebSocketsServer webSocket = WebSocketsServer(81);
WebServer httpServer(80);

// -----------------------------------------------------------------
// System state
// -----------------------------------------------------------------
String currentMode = "MANUAL";   // Default on power-up

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
bool isSystemModeCommand(const String& cmd) {
  return cmd == "SYS:MODE:MANUAL" || cmd == "SYS:MODE:AUTO";
}

bool isAlwaysAllowed(const String& cmd) {
  return isSystemModeCommand(cmd) ||
         cmd == "STOP" ||
         cmd == "ESTOP" ||
         cmd == "CMD:STOP" ||
         cmd == "JACK:FRONT:STOP" ||
         cmd == "JACK:REAR:STOP" ||
         cmd == "JACK:ALL:STOP" ||
         cmd == "JACK:STOP" ||
         cmd == "CAM:STOP" ||
         cmd == "ARM:BASE:STOP";
}

// =================================================================
// Central router
// =================================================================
void routeCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  // Mode switch commands go to both Mega and UNO.
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

  // Camera stand and arm commands go to UNO.
  if (cmd.startsWith("CAM:") || cmd.startsWith("ARM:")) {
    sendToUno(cmd);
    return;
  }

  // Movement, speed, jacks, sensors, and robot commands go to Mega.
  sendToMega(cmd);
}

// =================================================================
// HTTP URL helper
// =================================================================
String urlDecode(String s) {
  s.replace("%3A", ":");
  s.replace("%3a", ":");
  s.replace("%2F", "/");
  s.replace("%2f", "/");
  s.replace("%20", " ");
  s.replace("+", " ");
  return s;
}

// =================================================================
// HTTP handlers - Raspberry Pi interface
// =================================================================

// GET /
void handleRoot() {
  httpServer.send(200, "text/plain", "Apex Rover ESP32 Bridge | mode=" + currentMode);
}

// GET /get_status
void handleGetStatus() {
  String json = "{\"ok\":true,\"mode\":\"" + currentMode + "\"}";
  httpServer.send(200, "application/json", json);
}

// GET /get_sensors
void handleGetSensors() {
  // CRITICAL PROTECTION: Block Raspberry Pi from reading sensors entirely if system is in MANUAL mode
  if (currentMode == "MANUAL") {
    httpServer.send(
      403, 
      "application/json", 
      "{\"ok\":false,\"error\":\"Blocked: Raspberry Pi cannot request sensors in MANUAL mode\"}"
    );
    Serial.println("[BLOCKED Pi Sensors] system is MANUAL");
    return;
  }

  // AUTO Mode: Request real-time sensor data from Arduino Mega normally
  sendToMega("GET:SENSORS");
  String line = readMegaLine(300);

  String json = "{\"ok\":";
  json += (line.length() > 0) ? "true" : "false";
  json += ",\"raw\":\"" + line + "\"}";

  httpServer.send(200, "application/json", json);
}

// GET or POST /command?cmd=...
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

  Serial.print("[HTTP from Pi] ");
  Serial.println(command);

  // Safety exceptions: Always allow critical overrides (mode switching or emergency stops)
  if (isAlwaysAllowed(command)) {
    routeCommand(command);
    httpServer.send(200, "application/json", "{\"ok\":true,\"type\":\"always_allowed\"}");
    return;
  }

  // GATEKEEPER: Drop any directional/kinematic command from Raspberry Pi if system is in MANUAL mode
  if (currentMode == "MANUAL") {
    httpServer.send(
      403,
      "application/json",
      "{\"ok\":false,\"error\":\"system is MANUAL; command blocked\"}"
    );
    Serial.println("[BLOCKED Pi Command] system is MANUAL");
    return;
  }

  // AUTO Mode: Route autonomous processing pipeline commands normally
  routeCommand(command);
  httpServer.send(200, "application/json", "{\"ok\":true}");
}

// =================================================================
// WebSocket handler - Mobile App interface
// =================================================================
void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.print("[WS] App connected, client=");
      Serial.println(clientId);
      webSocket.sendTXT(clientId, "{\"event\":\"connected\",\"mode\":\"" + currentMode + "\"}");
      break;

    case WStype_DISCONNECTED:
      Serial.print("[WS] App disconnected, client=");
      Serial.println(clientId);
      if (currentMode == "MANUAL") {
        routeCommand("STOP");
        routeCommand("JACK:ALL:STOP");
        routeCommand("CAM:STOP");
      } else {
        Serial.println("[WS] App disconnected in AUTO mode; Raspberry remains active");
      }
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();

      if (command.length() == 0) return;

      Serial.print("[WS from App] ");
      Serial.println(command);

      if (isAlwaysAllowed(command)) {
        routeCommand(command);
        return;
      }

      // AUTO Mode Protection: Reject drive/motion controls coming from app to prevent autonomy drift
      if (currentMode == "AUTO") {
        webSocket.sendTXT(clientId, "{\"ok\":false,\"error\":\"system is AUTO; command blocked\"}");
        Serial.println("[BLOCKED App] system is AUTO");
        return;
      }

      // MANUAL Mode: Route mobile telemetry and mechanical inputs safely
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
  UnoSerial.begin(9600, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/", handleRoot);
  httpServer.on("/get_status", handleGetStatus);
  httpServer.on("/get_sensors", handleGetSensors);
  httpServer.on("/command", handleCommandHttp);
  httpServer.begin();

  // Inject system state defaults to downstream hardware registers immediately
  routeCommand("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Central Router v3 Ready");
  Serial.print("SSID     : "); Serial.println(WIFI_SSID);
  Serial.print("AP IP    : "); Serial.println(WiFi.softAPIP());
  Serial.println("=========================================");
}

void loop() {
  webSocket.loop();
  httpServer.handleClient();
}