#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure of the ESP-NOW message (PC -> ESP -> Robot)
typedef struct __attribute__((packed)) {
  uint8_t robot_id;
  char command;
  int16_t speed;
  float target_x;
  float target_y;
} CommandMessage;

// Structure of the sensor message (Robot -> ESP -> PC)

typedef struct __attribute__((packed)) {
  uint8_t mac[6];       
  uint8_t robot_id;
  int16_t light_value;
} SensorMessage;

CommandMessage cmdMsg;
SensorMessage sensorMsg;

void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // Silent to avoid saturating serial --- IGNORE ---
}

// Receive data from robots and forward to PC
void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Verify if it's a command message (from PC) or sensor data (from robot)

  struct RobotData {
    uint8_t robot_id;
    int16_t light_value;
  } __attribute__((packed));

  if (len == sizeof(RobotData)) {
    RobotData* incoming = (RobotData*)incomingData;
    
    // Fill the strcture with the mac, robot_id and light_value
    memcpy(sensorMsg.mac, mac, 6);
    sensorMsg.robot_id = incoming->robot_id;
    sensorMsg.light_value = incoming->light_value;
    // Forward to PC
    Serial.write((uint8_t*)&sensorMsg, sizeof(sensorMsg));
    Serial.flush();
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  //Print the MAC address of the ESP8266 (bridge)
  Serial.print("Bridge MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != 0) {
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
}

void loop() {
  // Receive commands from PC and forward to robots
  if (Serial.available() >= sizeof(CommandMessage)) {
    Serial.readBytes((char*)&cmdMsg, sizeof(CommandMessage));
    esp_now_send(broadcastAddress, (uint8_t*)&cmdMsg, sizeof(cmdMsg));
  }
}