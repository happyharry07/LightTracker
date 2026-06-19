#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// 接收端（小車）的 MAC 地址 (維持你原本設定的地址)
uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x32, 0x1C};  

Adafruit_MPU6050 mpu;
float gz_offset = 0.0f;
float roll = 0.0f; float pitch = 0.0f; float yaw = 0.0f;
const float ALPHA = 0.98f;  
unsigned long prevMicros = 0;

// 1. 定義與小車對應的資料結構
typedef struct struct_message {
  float yaw; float pitch; float roll;
} __attribute__((packed)) struct_message; 

typedef struct return_message {
  float car_yaw;
  float distance;
} __attribute__((packed)) return_message;

struct_message myData;
return_message incomingCarData; // 儲存從小車收到的數據
esp_now_peer_info_t peerInfo;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 60; 

static inline float rad2deg(float r) { return r * 180.0f / PI; }
static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// 【完美修正！】新版發送狀態回呼函式，將第一個參數改為 const wifi_tx_info_t *tx_info
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

// 【完美修正！】新版接收回呼函式，參數符合 3.x SDK 規格
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len >= sizeof(return_message)) {
    memcpy(&incomingCarData, incomingDataRaw, sizeof(return_message));
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  // 註冊發送與接收的回呼函式（新版無需強制轉型）
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Wire.begin(21, 22);  
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500); 
  float sum_gz = 0.0f;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum_gz += g.gyro.z * 180.0f / PI;
    delay(3);
  }
  gz_offset = sum_gz / 300.0f;
  prevMicros = micros();
  lastSendTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

  float ax = a.acceleration.x; float ay = a.acceleration.y; float az = a.acceleration.z;
  float gx_dps = g.gyro.x * 180.0f / PI;
  float gy_dps = g.gyro.y * 180.0f / PI;
  float gz_dps = g.gyro.z * 180.0f / PI;

  roll = ALPHA * (roll + gx_dps * dt) + (1.0f - ALPHA) * rad2deg(atan2(ay, az));
  pitch = ALPHA * (pitch + gy_dps * dt) + (1.0f - ALPHA) * rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

  float corrected_gz_dps = gz_dps - gz_offset;
  if (abs(corrected_gz_dps) > 0.45f) { 
    yaw += corrected_gz_dps * dt;
  }
  yaw = wrap360(yaw);

  myData.yaw = yaw; myData.pitch = pitch; myData.roll = roll;

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }

  /* 定時監控 (遙控器端：同步顯示自己角度、小車角度、兩者距離) */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;
    Serial.print("My(Rem) Yaw: ");  Serial.print(yaw);
    Serial.print(" | Tgt(Car) Yaw: "); Serial.print(incomingCarData.car_yaw);
    Serial.print(" | UWB Dist: ");     Serial.print(incomingCarData.distance); Serial.println(" m");
  }
}