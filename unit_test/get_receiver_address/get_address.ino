#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("REAL_RECEIVER_MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}