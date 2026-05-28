#include <WiFi.h>
#include <WebSocketsServer.h>

// ==================================================
// ESP32 - Apex Rover WebSocket Bridge
//
// Mobile App -> ESP32
//
// ESP32 sends:
// - Movement commands to Arduino Mega
// - Jack commands to Arduino Mega
// - Camera commands to Arduino UNO
// - Arm commands to Arduino UNO
//
// Mega:
// ESP32 G17 TX -> Mega Pin 19 RX1
//
// UNO:
// ESP32 G4 TX -> UNO Pin 2 RX SoftwareSerial
// ==================================================


// ================= WiFi Access Point =================

const char* ssid = "ApexRover";
const char* password = "12345678";

WebSocketsServer webSocket = WebSocketsServer(81);


// ==================================================
// Mega Serial Connection
// ==================================================
//
// ESP32 G17 TX  ---> Arduino Mega Pin 19 RX1
// ESP32 GND     ---> Arduino Mega GND
//
// Commands sent to Mega:
// FORWARD
// BACKWARD
// LEFT
// RIGHT
// STOP
// SPEED:50
// JACK:FRONT:EXTEND
// JACK:FRONT:RETRACT
// JACK:FRONT:STOP
// JACK:REAR:EXTEND
// JACK:REAR:RETRACT
// JACK:REAR:STOP
// ==================================================

#define MEGA_RX 16
#define MEGA_TX 17


// ==================================================
// UNO Serial Connection
// ==================================================
//
// ESP32 G4 TX ---> Arduino UNO Pin 2 RX SoftwareSerial
// ESP32 GND   ---> Arduino UNO GND
//
// Commands sent to UNO:
//
// ARM commands:
// ARM:BASE:LEFT
// ARM:BASE:RIGHT
// ARM:BASE:STOP
// ARM:UP
// ARM:DOWN
// ARM:FORWARD
// ARM:BACK
// ARM:WRIST:UP
// ARM:WRIST:DOWN
// ARM:GRIPPER:OPEN
// ARM:GRIPPER:CLOSE
// ARM:HOME
// ARM:PICK
// ARM:CARRY
// ARM:DROP
//
// Camera commands:
// CAM:LEFT
// CAM:RIGHT
// CAM:STOP
// CAM:UP
// CAM:DOWN
// CAM:CENTER
// ==================================================

#define UNO_RX 15   // Not used now
#define UNO_TX 4    // ESP32 G4 -> UNO Pin 2

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);


// ==================================================
// Send Commands
// ==================================================

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


// ==================================================
// Command Router
// ==================================================
//
// ARM and CAM commands go to UNO.
// Everything else goes to Mega.
// ==================================================

void handleCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  Serial.print("Command received from mobile: ");
  Serial.println(command);

  if (command.startsWith("ARM:") || command.startsWith("CAM:")) {
    sendToUno(command);
  } else {
    sendToMega(command);
  }
}


// ==================================================
// WebSocket Events
// ==================================================

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

      // Safety stop for robot movement and jacks
      sendToMega("STOP");
      sendToMega("JACK:FRONT:STOP");
      sendToMega("JACK:REAR:STOP");

      // Safety stop for UNO systems
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


// ==================================================
// Setup
// ==================================================

void setup() {
  Serial.begin(115200);

  // Serial to Mega
  // ESP32 G17 TX -> Mega Pin 19 RX1
  MegaSerial.begin(9600, SERIAL_8N1, MEGA_RX, MEGA_TX);

  // Serial to UNO
  // ESP32 G4 TX -> UNO Pin 2 RX
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
  Serial.println("Movement + Jack commands -> Mega");
  Serial.println("ARM + CAM commands -> UNO");
  Serial.println("====================================");
}


// ==================================================
// Loop
// ==================================================

void loop() {
  webSocket.loop();
}
