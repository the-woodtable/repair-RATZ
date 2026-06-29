#include <WebSocketsServer.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Hotspot credentials
const char *ssid = "FarhanCarESP32";
const char *password = "JamesBond";

WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Servo setup
Servo leftServo;
Servo rightServo;

int leftServoPin = 5;   // Left servo on pin 5
int rightServoPin = 6;  // Right servo on pin 6

// Movement functions
void moveForward() {
  Serial.println("Forward");
  leftServo.write(0);
  rightServo.write(0);
}

void moveBackward() {
  Serial.println("Backward");
  leftServo.write(180);
  rightServo.write(180);
}

void turnLeft() {
  Serial.println("Left");
  leftServo.write(90);   // stop left
  rightServo.write(0);   // right forward
}

void turnRight() {
  Serial.println("Right");
  rightServo.write(90);  // stop right
  leftServo.write(0);    // left forward
}

void stopCar() {
  Serial.println("Stop");
  leftServo.write(90);
  rightServo.write(90);
}

// Handle WebSocket messages
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if(type == WStype_TEXT) {
    String cmd = String((char*)payload);
    if(cmd == "F") moveForward();
    if(cmd == "B") moveBackward();
    if(cmd == "L") turnLeft();
    if(cmd == "R") turnRight();
    if(cmd == "S") stopCar();
  }
}

void setup() {
  Serial.begin(115200);

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  stopCar();

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started.");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("Password: "); Serial.println(password);
  Serial.print("AP IP address: "); Serial.println(WiFi.softAPIP());

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();

  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && currentLine.length() == 0) {
          // Serve HTML page
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          client.println("<!DOCTYPE html><html>");
          client.println("<head><title>ESP32 WebSocket Car Controller</title>");
          client.println("<style>");
          client.println("body { font-family: 'Segoe UI', sans-serif; background: linear-gradient(135deg,#74ebd5,#ACB6E5); text-align:center; margin:0; padding:0; }");
          client.println("h1 { color:#333; padding:20px; }");
          client.println("</style>");
          client.println("<script>");
          client.println("var ws = new WebSocket('ws://' + window.location.hostname + ':81/');");
          client.println("document.addEventListener('keydown', function(event){");
          client.println("  if(event.key==='ArrowUp') ws.send('F');");
          client.println("  if(event.key==='ArrowDown') ws.send('B');");
          client.println("  if(event.key==='ArrowLeft') ws.send('L');");
          client.println("  if(event.key==='ArrowRight') ws.send('R');");
          client.println("  if(event.key===' ') ws.send('S');");
          client.println("});");
          client.println("document.addEventListener('keyup', function(event){");
          client.println("  ws.send('S');"); // stop on key release
          client.println("});");
          client.println("</script>");
          client.println("</head><body>");
          client.println("<h1>🚗 ESP32 WebSocket Servo Car Remote</h1>");
          client.println("<p>Use arrow keys ↑ ↓ ← → to control. Spacebar = Stop.</p>");
          client.println("</body></html>");
          client.println();
          break;
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }
}
