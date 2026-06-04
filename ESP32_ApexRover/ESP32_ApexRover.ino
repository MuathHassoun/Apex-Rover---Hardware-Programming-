#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

// =================================================================
// ESP32 - Apex Rover Central WiFi Bridge
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
//   CAM:* / ARM:* -> UNO
//   AUTO:* / BLOCK:* / movement / jack / speed -> Mega
// =================================================================


// -----------------------------------------------------------------
// WiFi Access Point
// -----------------------------------------------------------------
const char* WIFI_SSID     = "Apex_Rover_Net";
const char* WIFI_PASSWORD = "12345678";


// -----------------------------------------------------------------
// Serial pin mapping
//
// Mega command direction only:
//   ESP32 GPIO17 TX -> Mega RX1 Pin 19
//   ESP32 GND       -> Mega GND
//
// IMPORTANT:
//   Mega TX1 Pin 18 -> ESP32 GPIO16 RX is NOT required now.
//   ACK comes from Mega USB Serial to Raspberry.
//
// UNO command direction:
//   ESP32 GPIO4 TX  -> UNO D2 SoftwareSerial RX
//   ESP32 GND       -> UNO GND
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
// State
// =================================================================
String currentMode = "MANUAL";

String lastSensorMessage =
  "SENSOR:PITCH=0;ROLL=0;FRONT=-1;REAR=-1;BALANCE=NO DATA";

unsigned long lastCommandAt = 0;
String lastCommand = "NONE";

String lastBridgeEvent = "NONE";
String lastMegaAck = "NONE";
String lastMegaError = "NONE";
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
// Helpers
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

void broadcastStatusEvent(const String& eventName) {
  String msg = "{";
  msg += "\"event\":\"" + eventName + "\",";
  msg += "\"mode\":\"" + currentMode + "\",";
  msg += "\"last_command\":\"" + jsonEscape(lastCommand) + "\"";
  msg += "}";

  webSocket.broadcastTXT(msg);
}

void broadcastBridgeLine(const String& source, const String& line) {
  // Send raw line first.
  // This makes it easy for the mobile app to show ACK text directly.
  webSocket.broadcastTXT(line);

  // Also send JSON event for future Test screen.
  String json = "{";
  json += "\"event\":\"bridge_event\",";
  json += "\"source\":\"" + jsonEscape(source) + "\",";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"line\":\"" + jsonEscape(line) + "\",";
  json += "\"last_command\":\"" + jsonEscape(lastCommand) + "\"";
  json += "}";

  webSocket.broadcastTXT(json);
}


// =================================================================
// Command routing
// =================================================================
void routeCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  lastCommand = cmd;
  lastCommandAt = millis();

  // System mode commands go to both Mega and UNO.
  if (cmd == "SYS:MODE:MANUAL") {
    currentMode = "MANUAL";
    sendToMega(cmd);
    sendToUno(cmd);

    Serial.println("[MODE] MANUAL");
    broadcastStatusEvent("mode_changed");
    return;
  }

  if (cmd == "SYS:MODE:AUTO") {
    currentMode = "AUTO";
    sendToMega(cmd);
    sendToUno(cmd);

    Serial.println("[MODE] AUTO");
    broadcastStatusEvent("mode_changed");
    return;
  }

  // Camera stand and arm go to UNO.
  if (cmd.startsWith("CAM:") || cmd.startsWith("ARM:")) {
    sendToUno(cmd);
    return;
  }

  // Mega LEGO blocks and auto blocks.
  if (cmd.startsWith("AUTO:") || cmd.startsWith("BLOCK:")) {
    sendToMega(cmd);
    return;
  }

  // Motors, speed, jacks, PULSE, status go to Mega.
  sendToMega(cmd);
}


// =================================================================
// HTTP handlers
// =================================================================
void handleRoot() {
  String text = "";

  text += "Apex Rover ESP32 Bridge Running\n";
  text += "Mode: " + currentMode + "\n";
  text += "WebSocket: ws://192.168.4.1:81\n";
  text += "HTTP: http://192.168.4.1\n";
  text += "\nEndpoints:\n";
  text += "/get_status\n";
  text += "/sensor_update?pitch=2.4&roll=-1.1&front=35.6&rear=18.2&balance=STABLE\n";
  text += "/bridge_event?source=MEGA&line=ACK:MEGA:DONE:UP_STAIRS\n";
  text += "/command?cmd=STOP\n";
  text += "/command?cmd=SYS:MODE:AUTO\n";
  text += "/command?cmd=PULSE:FORWARD:560\n";
  text += "/command?cmd=BLOCK:TURN:LEFT:90\n";
  text += "/command?cmd=BLOCK:GO:FORWARD:20\n";
  text += "/command?cmd=BLOCK:JACK:REAR:EXTEND:4\n";
  text += "/command?cmd=AUTO:UP_STAIRS\n";
  text += "/command?cmd=AUTO:DOWN_STAIRS\n";

  httpServer.send(200, "text/plain", text);
}

void handleGetStatus() {
  String json = "{";

  json += "\"ok\":true,";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"manual_only\":false,";

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
  json += "\"broadcast\":\"" + jsonEscape(sensorMessage) + "\"";
  json += "}";

  httpServer.send(200, "application/json", json);
}

// Called by Raspberry sensor_bridge.py when it reads ACK/ERR from Mega USB Serial.
//
// Example:
//   /bridge_event?source=MEGA&line=ACK:MEGA:DONE:UP_STAIRS
void handleBridgeEvent() {
  String source = httpServer.hasArg("source") ? httpServer.arg("source") : "UNKNOWN";
  String line = httpServer.hasArg("line") ? httpServer.arg("line") : "";

  source = urlDecode(source);
  line = urlDecode(line);

  source.trim();
  line.trim();

  if (line.length() == 0) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"empty line\"}");
    return;
  }

  lastBridgeSource = source;
  lastBridgeEvent = line;
  lastBridgeEventAt = millis();

  if (source == "MEGA" && line.startsWith("ACK:")) {
    lastMegaAck = line;
  }

  if (source == "MEGA" && (line.startsWith("ERR:") || line.startsWith("ERROR:"))) {
    lastMegaError = line;
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

// Optional HTTP command endpoint.
// Useful for Raspberry tests.
//
// Important:
//   This endpoint sends the command immediately.
//   Long blocks like AUTO:UP_STAIRS finish later.
//   Their ACK is received by Raspberry through USB Serial,
//   then Raspberry sends it back here using /bridge_event.
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

  routeCommand(command);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"command\":\"" + jsonEscape(command) + "\",";
  json += "\"note\":\"command_sent_ack_returns_via_raspberry_sensor_bridge\"";
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
      routeCommand("CAM:STOP");
      routeCommand("ARM:BASE:STOP");
      routeCommand("ARM:STOP");
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();

      if (command.length() == 0) {
        return;
      }

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

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/", handleRoot);
  httpServer.on("/get_status", handleGetStatus);
  httpServer.on("/sensor_update", handleSensorUpdate);
  httpServer.on("/bridge_event", handleBridgeEvent);
  httpServer.on("/command", handleCommandHttp);
  httpServer.begin();

  currentMode = "MANUAL";
  routeCommand("SYS:MODE:MANUAL");

  Serial.println("=========================================");
  Serial.println("ESP32 Apex Rover Bridge Ready");
  Serial.println("Mode     : MANUAL + AUTO ROUTING ENABLED");
  Serial.println("ACK path : Mega USB -> Raspberry sensor_bridge -> /bridge_event");
  Serial.print("SSID     : ");
  Serial.println(WIFI_SSID);
  Serial.print("Password : ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("AP IP    : ");
  Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket: ws://192.168.4.1:81");
  Serial.println("HTTP     : http://192.168.4.1");
  Serial.println("Sensor   : http://192.168.4.1/sensor_update");
  Serial.println("Bridge   : http://192.168.4.1/bridge_event");
  Serial.println("Command  : http://192.168.4.1/command?cmd=STOP");
  Serial.println("Blocks   : http://192.168.4.1/command?cmd=BLOCK:TURN:LEFT:90");
  Serial.println("Auto     : http://192.168.4.1/command?cmd=AUTO:UP_STAIRS");
  Serial.println("=========================================");
}


// =================================================================
// Loop
// =================================================================
void loop() {
  webSocket.loop();
  httpServer.handleClient();
}