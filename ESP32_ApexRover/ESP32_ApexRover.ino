#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge
//
// Current Architecture:
//
// Mobile App   -> WebSocket port 81 -> ESP32
// ESP32        -> Serial2           -> Arduino Mega
// ESP32        -> Serial1           -> Arduino UNO
//
// Raspberry Pi -> HTTP port 80      -> ESP32
// Raspberry is used for:
//   1. Camera streaming separately on port 5000
//   2. Sending sensor updates to ESP32 using /sensor_update
//
// Manual control only:
//   Mobile App controls the robot.
//   No automatic driving from Raspberry.
//
// Sensor Flow:
//   Mega -> Raspberry Pi -> ESP32 /sensor_update -> Mobile App WebSocket
//
// Sensor message broadcast to app:
//   SENSOR:PITCH=2.4;ROLL=-1.1;FRONT=35.6;REAR=18.2;BALANCE=STABLE
//
// Routing:
//   CAM:* / ARM:* -> UNO
//   Everything else -> Mega
// =================================================================


// -----------------------------------------------------------------
// WiFi Access Point
// -----------------------------------------------------------------

const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";


// -----------------------------------------------------------------
// Serial pin mapping
//
// Mega:
//   ESP32 GPIO17 TX -> Mega RX1 Pin 19
//   ESP32 GPIO16 RX -> Mega TX1 Pin 18 if needed
//   ESP32 GND       -> Mega GND
//
// UNO:
//   ESP32 GPIO4 TX  -> UNO D2 / Pin 2 SoftwareSerial RX
//   ESP32 GND       -> UNO GND
// -----------------------------------------------------------------

#define MEGA_RX_PIN 16
#define MEGA_TX_PIN 17

#define UNO_RX_PIN  15
#define UNO_TX_PIN  4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);


// -----------------------------------------------------------------
// Servers
// -----------------------------------------------------------------

WebSocketsServer webSocket = WebSocketsServer(81);
WebServer httpServer(80);


// -----------------------------------------------------------------
// State
// -----------------------------------------------------------------

String currentMode = "MANUAL";
String lastSensorMessage = "SENSOR:PITCH=0;ROLL=0;FRONT=-1;REAR=-1;BALANCE=NO DATA";


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
// Command helpers
// =================================================================

bool isStopCommand(const String& cmd) {
  return cmd == "STOP" ||
         cmd == "ESTOP" ||
         cmd == "CMD:STOP" ||
         cmd == "JACK:FRONT:STOP" ||
         cmd == "JACK:REAR:STOP" ||
         cmd == "JACK:ALL:STOP" ||
         cmd == "JACK:STOP" ||
         cmd == "CAM:STOP" ||
         cmd == "ARM:BASE:STOP";
}

void routeCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  // We keep this for compatibility only.
  // The system is manual-only now.
  if (cmd == "SYS:MODE:MANUAL") {
    currentMode = "MANUAL";
    sendToMega(cmd);
    sendToUno(cmd);
    Serial.println("[MODE] MANUAL");
    return;
  }

  // Automatic is disabled.
  if (cmd == "SYS:MODE:AUTO") {
    currentMode = "MANUAL";
    Serial.println("[IGNORED] AUTO mode is disabled. Keeping MANUAL.");
    return;
  }

  // Camera stand and arm go to UNO
  if (cmd.startsWith("CAM:") || cmd.startsWith("ARM:")) {
    sendToUno(cmd);
    return;
  }

  // Motors, speed, jacks, sensors go to Mega
  sendToMega(cmd);
}

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
// HTTP handlers
// =================================================================

void handleRoot() {
  String text = "";
  text += "Apex Rover ESP32 Bridge Running\n";
  text += "Mode: MANUAL ONLY\n";
  text += "WebSocket: ws://192.168.4.1:81\n";
  text += "HTTP: http://192.168.4.1\n";
  text += "\nEndpoints:\n";
  text += "/get_status\n";
  text += "/sensor_update?pitch=2.4&roll=-1.1&front=35.6&rear=18.2&balance=STABLE\n";
  text += "/command?cmd=STOP\n";

  httpServer.send(200, "text/plain", text);
}

void handleGetStatus() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"MANUAL\",";
  json += "\"manual_only\":true,";
  json += "\"last_sensor\":\"" + lastSensorMessage + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}


// -----------------------------------------------------------------
// IMPORTANT SENSOR ENDPOINT
//
// Raspberry Pi calls this endpoint after reading sensors from Mega.
//
// Example:
// http://192.168.4.1/sensor_update?pitch=2.4&roll=-1.1&front=35.6&rear=18.2&balance=STABLE
//
// ESP32 broadcasts this to all mobile app WebSocket clients:
//
// SENSOR:PITCH=2.4;ROLL=-1.1;FRONT=35.6;REAR=18.2;BALANCE=STABLE
// -----------------------------------------------------------------

void handleSensorUpdate() {
  String pitch = httpServer.hasArg("pitch") ? httpServer.arg("pitch") : "0";
  String roll = httpServer.hasArg("roll") ? httpServer.arg("roll") : "0";
  String front = httpServer.hasArg("front") ? httpServer.arg("front") : "-1";
  String rear = httpServer.hasArg("rear") ? httpServer.arg("rear") : "-1";
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

  String json = "{";
  json += "\"ok\":true,";
  json += "\"broadcast\":\"" + sensorMessage + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}


// Optional command endpoint.
// Useful for testing from Raspberry, but normal control comes from Mobile WebSocket.
void handleCommandHttp() {
  if (!httpServer.hasArg("cmd")) {
    httpServer.send(
      400,
      "application/json",
      "{\"ok\":false,\"error\":\"missing cmd parameter\"}"
    );
    return;
  }

  String command = urlDecode(httpServer.arg("cmd"));
  command.trim();

  if (command.length() == 0) {
    httpServer.send(
      400,
      "application/json",
      "{\"ok\":false,\"error\":\"empty command\"}"
    );
    return;
  }

  Serial.print("[HTTP CMD] ");
  Serial.println(command);

  // Since system is manual-only, Raspberry should not drive the robot.
  // But STOP commands are allowed for safety.
  if (isStopCommand(command) || command == "SYS:MODE:MANUAL") {
    routeCommand(command);
    httpServer.send(200, "application/json", "{\"ok\":true,\"type\":\"safe_command\"}");
    return;
  }

  httpServer.send(
    403,
    "application/json",
    "{\"ok\":false,\"error\":\"manual-only system; use mobile app for control\"}"
  );
}


// =================================================================
// WebSocket handler - Mobile App control
// =================================================================

void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.print("[WS] App connected, client=");
      Serial.println(clientId);

      webSocket.sendTXT(
        clientId,
        "{\"event\":\"connected\",\"mode\":\"MANUAL\",\"manual_only\":true}"
      );

      // Send last sensor state immediately when app connects
      webSocket.sendTXT(clientId, lastSensorMessage);
      break;


    case WStype_DISCONNECTED:
      Serial.print("[WS] App disconnected, client=");
      Serial.println(clientId);

      // Mobile is the controller. If it disconnects, stop safely.
      routeCommand("STOP");
      routeCommand("JACK:ALL:STOP");
      routeCommand("CAM:STOP");
      break;


    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();

      if (command.length() == 0) {
        return;
      }

      Serial.print("[WS from App] ");
      Serial.println(command);

      // App controls everything manually
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

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/", handleRoot);
  httpServer.on("/get_status", handleGetStatus);
  httpServer.on("/sensor_update", handleSensorUpdate);
  httpServer.on("/command", handleCommandHttp);
  httpServer.begin();

  // Default manual-only mode
  currentMode = "MANUAL";
  routeCommand("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Bridge Ready");
  Serial.println("Mode     : MANUAL ONLY");
  Serial.print("SSID     : ");
  Serial.println(WIFI_SSID);
  Serial.print("Password : ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("AP IP    : ");
  Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket: ws://192.168.4.1:81");
  Serial.println("HTTP     : http://192.168.4.1");
  Serial.println("Sensor   : http://192.168.4.1/sensor_update");
  Serial.println("=========================================");
}


// =================================================================
// Loop
// =================================================================

void loop() {
  webSocket.loop();
  httpServer.handleClient();
}