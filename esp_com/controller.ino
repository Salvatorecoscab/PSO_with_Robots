// Controller to use with a flutter app to control the robot via WiFi
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Network credentials
const char* ssid = "Robot_ESP8266";
const char* password = "";

ESP8266WebServer server(80);

// Motor control pins
const int IN1 = 5;  const int IN2 = 4;
const int IN3 = 14; const int IN4 = 12;
const int SLEEP_PIN = 13;

int currentSpeed = 600; // Default speed (0-1023)


void stopMotors() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}

void forward(int speed) {
  analogWrite(IN1, speed); analogWrite(IN2, 0);
  analogWrite(IN3, speed); analogWrite(IN4, 0);
}

void backward(int speed) {
  analogWrite(IN1, 0); analogWrite(IN2, speed);
  analogWrite(IN3, 0); analogWrite(IN4, speed);
}

void left(int speed) {
  analogWrite(IN1, 0); analogWrite(IN2, speed);
  analogWrite(IN3, speed); analogWrite(IN4, 0);
}

void right(int speed) {
  analogWrite(IN1, speed); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, speed);
}

// Handle incoming HTTP requests to move the robot
void handleMove() {
  if (server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (server.hasArg("speed")) currentSpeed = server.arg("speed").toInt();

    if (dir == "F") forward(currentSpeed);
    else if (dir == "B") backward(currentSpeed);
    else if (dir == "L") left(currentSpeed);
    else if (dir == "R") right(currentSpeed);
    else stopMotors();
    
    server.send(200, "text/plain", "OK");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);
  
  analogWriteFreq(1000);
  analogWriteRange(1023);

// Start WiFi in AP mode
  WiFi.softAP(ssid, password);
  Serial.println("AP Iniciado: " + String(ssid));
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  server.on("/move", handleMove);
  server.on("/stop", stopMotors);
  server.begin();
}

void loop() {
  server.handleClient();
}