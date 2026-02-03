#include <ESP8266WiFi.h>

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}
// black: BC:DD:C2:FD:F0:B9
// red: BC:DD:C2:FD:C5:9C