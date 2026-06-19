#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  float yaw;
  float pitch;
  float roll;
} struct_message;

struct_message incomingData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  
  Serial.print("Yaw: ");
  Serial.print(incomingData.yaw, 2);
  Serial.print(", Pitch: ");
  Serial.print(incomingData.pitch, 2);
  Serial.print(", Roll: ");
  Serial.println(incomingData.roll, 2);
}

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  Serial.println("Receiver ready. Waiting for data...");
}

void loop() {
}