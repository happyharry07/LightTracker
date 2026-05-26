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
  
  // 【修正 1】在接收端電腦螢幕上印出，確認 ESP32 真的有收到空中的資料
  Serial.print("ESP32 Received Yaw: ");
  Serial.println(incomingData.yaw, 2);
  
  // 【修正 2】透過實體線路傳給 Mega，結尾務必加上 \n (println)
  Serial2.print("$");
  Serial2.println(incomingData.yaw, 2);
}

void setup() {
  Serial.begin(115200);
  
  // 初始化與 Mega 連接的序列埠 (TX2=GPIO 17, RX2=GPIO 16)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Receiver ready. Forwarding data to Mega...");
}

void loop() {
}