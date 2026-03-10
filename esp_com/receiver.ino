
// Recptor using ESP-NOW to receive commands and send sensor data back to the controller. Each robot has a unique ID (4, 5, 6) and listens for commands directed to it or broadcasted to all (ID 255). The code handles motor control based on received commands and periodically sends light sensor readings back to the controller.

#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}


const uint8_t MY_ROBOT_ID = 6;  // <-- change for each robot (4, 5, 6)
// MAC from emisor (controller) - update with your controller's MAC address
uint8_t emisorAddress[] = {0x50, 0x02, 0x91, 0xDF, 0x07, 0xA5};
// Pines Motor A
const int IN1 = 5;
const int IN2 = 4;
// Pines Motor B
const int IN3 = 14;
const int IN4 = 12;
// Pin Sleep
const int SLEEP_PIN = 13;
const int LDR_PIN = A0;  // Pin to read the light sensor (LDR)

int currentSpeed = 600;

unsigned long lastSensorSend = 0;
const int SENSOR_SEND_INTERVAL = 200;  // Send each 200ms

typedef struct __attribute__((packed)) {
  uint8_t robot_id;
  char command;
  int16_t speed;
  float target_x;
  float target_y;
} CommandMessage;

CommandMessage cmdMsg;
typedef struct __attribute__((packed)) {
  uint8_t robot_id;
  int16_t light_value;  // 0-1023
} SensorMessage;
SensorMessage sensorMsg;

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void forward(int speed) {
  analogWrite(IN1, speed);
  analogWrite(IN2, 0);
  analogWrite(IN3, speed);
  analogWrite(IN4, 0);
}

void backward(int speed) {
  analogWrite(IN1, 0);
  analogWrite(IN2, speed);
  analogWrite(IN3, 0);
  analogWrite(IN4, speed);
}

void left(int speed) {
  analogWrite(IN1, 0);
  analogWrite(IN2, speed);
  analogWrite(IN3, speed);
  analogWrite(IN4, 0);
}

void right(int speed) {
  analogWrite(IN1, speed);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, speed);
}

void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len != sizeof(CommandMessage)) {
    return;
  }
  
  memcpy(&cmdMsg, incomingData, sizeof(cmdMsg));
  
  
  // Verify if the command is for this robot or broadcast (255)
    if (cmdMsg.robot_id != MY_ROBOT_ID && cmdMsg.robot_id != 255) return;
  
  currentSpeed = constrain(cmdMsg.speed, 0, 1023);
  
  switch (cmdMsg.command) {
    case 'F':
      forward(currentSpeed);
      break;
      
    case 'B':
      backward(currentSpeed);
      break;
      
    case 'L':
      left(currentSpeed);
      break;
      
    case 'R':
      right(currentSpeed);
      break;
      
    case 'S':
      stopMotors();
      break;
      
  }
}

void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // Serial.print("Last Packet Send Status: ");
}
void setup() {
  Serial.begin(115200);
  Serial.print("\n\n=== ROBOT ID: ");
  Serial.print(MY_ROBOT_ID);
  Serial.println(" ===");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);
  pinMode(LDR_PIN, INPUT); 
  analogWriteFreq(1000);
  analogWriteRange(1023);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != 0) {
    Serial.println("❌ Error ESP-NOW");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);  // COMBO para enviar y recibir
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  esp_now_add_peer(emisorAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  
  Serial.println("✓ Receptor listo");
}

void loop() {
  // Send sensor data back to controller every SENSOR_SEND_INTERVAL ms`
  if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
    sensorMsg.robot_id = MY_ROBOT_ID;
    sensorMsg.light_value = analogRead(LDR_PIN);
    
    esp_now_send(emisorAddress, (uint8_t*)&sensorMsg, sizeof(sensorMsg));
    
    lastSensorSend = millis();
  }
  
  delay(10);
}