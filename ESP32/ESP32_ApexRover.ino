@'
#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "ApexRover";
const char* password = "12345678";

WebSocketsServer webSocket = WebSocketsServer(81);

// ================= Mega Serial =================
// ESP32 G17 TX -> Mega Pin 19 RX1
#define MEGA_RX 16
#define MEGA_TX 17

// ================= UNO Arm Serial =================
// ESP32 G4 TX -> UNO Pin 2 RX SoftwareSerial
#define UNO_RX 15   // not used now
#define UNO_TX 4

HardwareSerial MegaSerial(2);
HardwareSerial UnoSerial(1);

void sendToMega(String command) {
  command.trim();
  MegaSerial.println(command);

  Serial.print("Sent to Mega: ");
  Serial.println(command);
}

void sendToUno(String command) {
  command.trim();
  UnoSerial.println(command);

  Serial.print("Sent to UNO Arm: ");
  Serial.println(command);
}

void handleCommand(String command) {
  command.trim();

  Serial.print("Command received from mobile: ");
  Serial.println(command);

  if (command.startsWith("ARM:")) {
    sendToUno(command);
  } else {
    sendToMega(command);
  }
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

      sendToMega("STOP");
      sendToUno("ARM:BASE:STOP");
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
  Serial.println("ESP32 ready:");
  Serial.println("- Movement commands go to Mega");
  Serial.println("- ARM commands go to UNO");
}

void loop() {
  webSocket.loop();
}
'@ | Set-Content -Encoding UTF8 ESP32\ESP32_ApexRover.ino

