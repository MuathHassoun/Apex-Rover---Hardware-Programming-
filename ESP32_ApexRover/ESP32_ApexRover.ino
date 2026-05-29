
#include <WiFi.h>
#include <WebSocketsServer.h>

// ==================================================
// ESP32 - Apex Rover WebSocket Bridge
//
// Mobile App -> ESP32
//
// Routing:
// - SYS:MODE:MANUAL / SYS:MODE:AUTO -> Mega + UNO
// - ARM:* / CAM:*                       -> UNO
// - Movement / SPEED / JACK             -> Mega
//
// Default system mode is MANUAL.
// Mega and UNO also enforce their own MANUAL/AUTO gate.
// ==================================================

const char* ssid = "ApexRover";
const char* password = "12345678";

WebSocketsServer webSocket = WebSocketsServer(81);

// Mega Serial2:
// ESP32 GPIO17 TX -> Mega RX1 Pin 19
// ESP32 GND       -> Mega GND
#define MEGA_RX 16
#define MEGA_TX 17

// UNO Serial1:
// ESP32 GPIO4 TX -> UNO D2 SoftwareSerial RX
// ESP32 GND      -> UNO GND
#define UNO_RX 15   // Not used now
#define UNO_TX 4    // ESP32 GPIO4 TX -> UNO Pin D2 RX

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

String systemMode = "MANUAL";

void sendToMega(String command) {
  command.trim();
  MegaSerial.println(command);
  Serial.print("Sent to Mega: ");
  Serial.println(command);
}

void sendToUno(String command) {
  command.trim();
  UnoSerial.println(command);
  Serial.print("Sent to UNO: ");
  Serial.println(command);
}

void handleCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  Serial.print("Command received from mobile: ");
  Serial.println(command);

  // System mode commands must reach both controllers.
  if (command == "SYS:MODE:MANUAL") {
    systemMode = "MANUAL";
    sendToMega(command);
    sendToUno(command);
    return;
  }

  if (command == "SYS:MODE:AUTO") {
    systemMode = "AUTO";
    sendToMega(command);
    sendToUno(command);
    return;
  }

  // Arm and camera stand commands go to UNO.
  if (command.startsWith("ARM:") || command.startsWith("CAM:")) {
    sendToUno(command);
    return;
  }

  // Movement, speed, and jack commands go to Mega.
  sendToMega(command);
}

void webSocketEvent(uint8_t clientNumber, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.print("Mobile connected, client number: ");
      Serial.println(clientNumber);
      webSocket.sendTXT(clientNumber, "ESP32 WebSocket Connected");
      break;

    case WStype_DISCONNECTED:
      Serial.print("Mobile disconnected, client number: ");
      Serial.println(clientNumber);

      // Safety stop. Mega/UNO will accept STOP even in AUTO.
      sendToMega("STOP");
      sendToMega("JACK:FRONT:STOP");
      sendToMega("JACK:REAR:STOP");
      sendToUno("ARM:BASE:STOP");
      sendToUno("CAM:STOP");
      break;

    case WStype_TEXT:
      handleCommand(String((char*)payload));
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX, MEGA_TX);
  UnoSerial.begin(9600, SERIAL_8N1, UNO_RX, UNO_TX);

  WiFi.softAP(ssid, password);

  Serial.println("====================================");
  Serial.println("ESP32 Access Point Started");
  Serial.print("WiFi Name: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket server started on port 81");
  Serial.println("ESP32 Ready");
  Serial.println("Default Mode: MANUAL");
  Serial.println("SYS mode commands -> Mega + UNO");
  Serial.println("Movement + Jack commands -> Mega");
  Serial.println("ARM + CAM commands -> UNO");
  Serial.println("====================================");
}

void loop() {
  webSocket.loop();
}
