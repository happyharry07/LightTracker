#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// ==================== 【系統設定】 ====================
// 接收端（小車）的 MAC 地址
uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x32, 0x1C};

// 遙控器本地推算參數 (必須與小車端設定完全一致)
const float REMOTE_HEIGHT = 0.80;   // 遙控器離地高度 0.8m
const float MAX_TARGET_DIST = 4.5f; // 小車端設定的最大距離上限

Adafruit_MPU6050 mpu;
float gz_offset = 0.0f;
float roll = 0.0f; float pitch = 0.0f; float yaw = 0.0f;
const float ALPHA = 0.98f;  
unsigned long prevMicros = 0;

// ==================== 【ESP-NOW 資料結構】 ====================
typedef struct struct_message {
  float yaw;
  float pitch; 
  float roll;
} __attribute__((packed)) struct_message; 

typedef struct return_message {
  float car_yaw; 
  float distance; 
  float car_x;
  float car_y;
  float tgt_x;
  float tgt_y;
} __attribute__((packed)) return_message;

struct_message myData;
return_message incomingCarData;
esp_now_peer_info_t peerInfo;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 60; 

// ==================== 【數學輔助函數】 ====================
static inline float rad2deg(float r) { return r * 180.0f / PI; }

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// ==================== 【ESP-NOW 回呼】 ====================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len >= sizeof(return_message)) {
    memcpy(&incomingCarData, incomingDataRaw, sizeof(return_message));
  }
}

// ==================== 【主程式】 ====================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Wire.begin(21, 22);  
  Wire.setClock(400000); 

  if (!mpu.begin()) {
    Serial.println("MPU6050 Not Found");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("====== MPU6050 Calibrating... PLEASE KEEP REMOTE STILL ======");
  delay(500); 
  float sum_gz = 0.0f;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum_gz += g.gyro.z * 180.0f / PI;
    delay(3);
  }
  gz_offset = sum_gz / 300.0f;
  
  Serial.println("====== Remote Calibration Success! ======");
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

  myData.yaw = yaw; 
  myData.pitch = pitch; 
  myData.roll = roll;

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }

  // 定時監控與狀態顯示
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;

    // --- 本地端直接運算目標座標 (離線測試用) ---
    float pitchRad = abs(pitch) * PI / 180.0f;
    if(pitchRad < 0.05f) pitchRad = 0.05f; 
    
    float local_R_tgt = REMOTE_HEIGHT / tan(pitchRad);
    if(local_R_tgt > MAX_TARGET_DIST) local_R_tgt = MAX_TARGET_DIST; 
    
    float yawRad = yaw * PI / 180.0f;
    float local_tgt_x = local_R_tgt * cos(yawRad);
    float local_tgt_y = local_R_tgt * sin(yawRad);

    // 精簡標籤名稱，並將所有資訊合併在同一行輸出
    Serial.printf("Yaw:%5.1f | Dist:%4.2fm | Car(%5.2f,%5.2f) | RTgt(%5.2f,%5.2f) | CTgt(%5.2f,%5.2f)\n",
                  yaw, 
                  incomingCarData.distance, 
                  incomingCarData.car_x, 
                  incomingCarData.car_y,
                  local_tgt_x, 
                  local_tgt_y,
                  incomingCarData.tgt_x,
                  incomingCarData.tgt_y);
  }
}