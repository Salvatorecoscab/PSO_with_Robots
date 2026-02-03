#include <ESP8266WiFi.h>
#include <espnow.h>

void onDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  // Encender LED (LOW suele ser encendido en ESP8266)
  digitalWrite(LED_BUILTIN, LOW); 
  
  char message[len + 1];
  memcpy(message, incomingData, len);
  message[len] = '\0';
  
  Serial.print("Mensaje recibido: ");
  Serial.println(message);
  
  delay(100); 
  // Apagar LED
  digitalWrite(LED_BUILTIN, HIGH); 
}

void setup() {
  Serial.begin(115200);
  
  // Configurar el LED interno
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Empezar apagado
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != 0) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
}
