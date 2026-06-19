#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <HardwareSerial.h>

// 發送端（遙控器）的 MAC 地址
uint8_t remoteAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x33, 0x3C}; 

// ==================== UWB (BU03-Kit) 配置 ====================
HardwareSerial UwbSerial(2);
const int BU03_RX = 16; 
const int BU03_TX = 17;
const unsigned long UWB_INTERVAL = 50;  // 每 50ms 請求一次 UWB 距離
unsigned long lastUwbTime = 0;
float currentDistance = -1.0;           // 儲存當前最新距離

// ==================== 距離控制參數 (P 控制) ====================
const float TARGET_DISTANCE = 1.50;    // 目標保持距離：1.5 公尺
const float KP_dist = 200.0;           // 距離比例增益
const float DEAD_ZONE_DIST = 0.10;     // 距離死區範圍 (10cm)

// ==================== 角度控制參數 (PD 控制) ====================
const double Kp_turn = 4.5;   
const double Kd_turn = 0.5;  
float last_error_yaw = 0.0f;     
bool is_moving_yaw = false;

// ==================== 馬達綜合限速 ====================
const int MAX_MOTOR_SPEED = 200;  
const int MIN_MOTOR_SPEED = 60;    

// ==================== 雙向 ESP-NOW 資料結構 ====================
typedef struct struct_message {
  float yaw; float pitch; float roll;
} __attribute__((packed)) struct_message; 

typedef struct return_message {
  float car_yaw;
  float distance; 
} __attribute__((packed)) return_message;

struct_message incomingData; 
return_message txData;        
esp_now_peer_info_t peerInfo;

volatile float raw_target_yaw = 0.0f; 
volatile bool first_data_received = false;

/* ========= 馬達與 I2C 腳位定義 ========= */
const int PWMA = 13; const int AIN2 = 14; const int AIN1 = 27;
const int BIN1 = 26; const int BIN2 = 25; const int PWMB = 33;
const int I2C_SDA = 21; const int I2C_SCL = 22;

Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float current_yaw = 0.0f;
unsigned long prevMicros = 0;

float yaw_offset = 0.0f;    
bool origin_set = false;    
bool mpu_ready = false;
unsigned long lastMpuReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 20; 

// 定時回傳給遙控器的時間計數器
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 60; // 每 60ms 主動回傳一次狀態給遙控器

// 前置宣告
static inline float wrap360(float a);
void MotorWriting(double vL, double vR);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len);
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
bool checkUwbModule();
float getDistanceFast();

void setup() {
  Serial.begin(115200);
  
  // 1. 初始化 UWB 序列埠 (AT 指令模式)
  UwbSerial.begin(115200, SERIAL_8N1, BU03_RX, BU03_TX);
  
  // 2. 初始化馬達引腳
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);

  // 3. 初始化 MPU6050
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (localMpu.begin()) {
    localMpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    localMpu.setGyroRange(MPU6050_RANGE_250_DEG);
    localMpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(500);
    float sum_gz = 0.0f;
    for (int i = 0; i < 200; i++) {
      sensors_event_t a, g, temp;
      if (localMpu.getEvent(&a, &g, &temp)) sum_gz += g.gyro.z * 180.0f / PI;
      delay(2);
    }
    gz_offset = sum_gz / 200.0f;
    mpu_ready = true;
  }

  // 4. 初始化 ESP-NOW 通訊
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // 5. 檢查 UWB 模組連線狀態
  delay(500);
  if (checkUwbModule()) {
    Serial.println("[SUCCESS] BU03-Kit Connected.");
  } else {
    Serial.println("[WARNING] BU03-Kit No Response. Check Wiring!");
  }

  prevMicros = micros();
  lastMpuReadTime = millis();
  lastTxTime = millis();
  lastUwbTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  /* ==================== 0. 定時排程主動獲取 UWB 距離 ==================== */
  if (currentMillis - lastUwbTime >= UWB_INTERVAL) {
    lastUwbTime = currentMillis;
    float distRead = getDistanceFast();
    if (distRead >= 0) {
      currentDistance = distRead; // 更新全域變數
    }
  }
  
  /* ==================== 1. 定時更新小車絕對角度 ==================== */
  if (mpu_ready && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;
    sensors_event_t a, g, temp;
    if (localMpu.getEvent(&a, &g, &temp)) {
      unsigned long now = micros();
      float dt = (now - prevMicros) * 1e-6f;
      prevMicros = now;
      if (dt <= 0.0f || dt > 0.04f) dt = 0.02f; 

      float gz_deg = (g.gyro.z * 180.0f / PI) - gz_offset;
      if (abs(gz_deg) > 0.50f) { 
        current_yaw += gz_deg * dt;
      }
      current_yaw = wrap360(current_yaw); 
    }
  }

  /* ==================== 2. 鎖定原點 180° Offset ==================== */
  if (first_data_received && mpu_ready && !origin_set) {
    yaw_offset = wrap360(raw_target_yaw - 180.0f); 
    current_yaw = 0.0f; 
    origin_set = true;
    Serial.println(">>>>>> ORIGIN LOCKED AT Relative 180° <<<<<<");
  }

  /* ==================== 3. 運動控制混算演算法（混控角度與距離） ==================== */
  double speedL = 0;
  double speedR = 0;

  if (first_data_received && mpu_ready && origin_set) {
    // ---- A. 角度控制（PD 控制計算） ----
    float translated_target_yaw = wrap360(raw_target_yaw - yaw_offset);
    float error_yaw = translated_target_yaw - current_yaw;
    if (error_yaw > 180.0f)  error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;
    
    float abs_error_yaw = abs(error_yaw);
    if (!is_moving_yaw) {
      if (abs_error_yaw > 5.0f) is_moving_yaw = true;
    } else {
      if (abs_error_yaw < 1.5f) is_moving_yaw = false;
    }

    float turnOutput = 0;
    if (is_moving_yaw) {
      float error_diff = error_yaw - last_error_yaw;
      last_error_yaw = error_yaw;
      turnOutput = error_yaw * Kp_turn + error_diff * Kd_turn;
    } else {
      last_error_yaw = error_yaw;
    }

    // ---- B. 距離控制（P 控制計算） ----
    float distOutput = 0;
    if (currentDistance >= 0) { // 確保有成功拿到距離才控制
      float error_dist = currentDistance - TARGET_DISTANCE;
      if (abs(error_dist) > DEAD_ZONE_DIST) {
        distOutput = error_dist * KP_dist;
      }
    }

    // ---- C. 混控核心 (左右輪速合成) ----
    // 轉向（turnOutput）：左輪加、右輪減（正值代表往右偏，左輪前進右輪後退來達成右轉）
    // 前進（distOutput）：兩輪同加同減
    speedL = distOutput + turnOutput;
    speedR = distOutput - turnOutput;

    // ---- D. 綜合動態速度限制與死區克服 ----
    // 檢查左輪
    if (abs(speedL) > 0.1) {
      if (speedL > MAX_MOTOR_SPEED)  speedL = MAX_MOTOR_SPEED;
      if (speedL < -MAX_MOTOR_SPEED) speedL = -MAX_MOTOR_SPEED;
      if (speedL > 0 && speedL < MIN_MOTOR_SPEED)  speedL = MIN_MOTOR_SPEED;
      if (speedL < 0 && speedL > -MIN_MOTOR_SPEED) speedL = -MIN_MOTOR_SPEED;
    }
    // 檢查右輪
    if (abs(speedR) > 0.1) {
      if (speedR > MAX_MOTOR_SPEED)  speedR = MAX_MOTOR_SPEED;
      if (speedR < -MAX_MOTOR_SPEED) speedR = -MAX_MOTOR_SPEED;
      if (speedR > 0 && speedR < MIN_MOTOR_SPEED)  speedR = MIN_MOTOR_SPEED;
      if (speedR < 0 && speedR > -MIN_MOTOR_SPEED) speedR = -MIN_MOTOR_SPEED;
    }
  }

  // 實際寫入馬達
  MotorWriting(speedL, speedR);

  /* ==================== 4. 定時主動將狀態回傳給遙控器 ==================== */
  if (first_data_received && (currentMillis - lastTxTime >= TX_INTERVAL)) {
    lastTxTime = currentMillis;
    
    txData.car_yaw = current_yaw;
    txData.distance = currentDistance; // 將最新的 UWB 距離同步打包回去

    esp_err_t result = esp_now_send(remoteAddress, (uint8_t *) &txData, sizeof(txData));
    if (result != ESP_OK) {
      Serial.println("Warning: ESP-NOW TX Failed!");
    }
  }

  /* ==================== 5. 定時序列埠監控 ==================== */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;
    Serial.print("Car Yaw: ");    Serial.print(current_yaw, 1);
    Serial.print(" | Tgt Yaw: ");   Serial.print(raw_target_yaw, 1);
    Serial.print(" | UWB Dist: "); Serial.print(currentDistance, 3); Serial.print(" m");
  }
}

// ==================== 馬達驅動核心（支援前後進） ====================
void MotorWriting(double vL, double vR) {
  if (vR >= 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    vR = -vR;
  }
  if (vL >= 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    vL = -vL;
  }
  analogWrite(PWMA, (int)vL);
  analogWrite(PWMB, (int)vR);
}

// ==================== ESP-NOW 接收狀態回呼 ====================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len >= sizeof(struct_message)) {
    memcpy(&incomingData, incomingDataRaw, sizeof(struct_message));
    raw_target_yaw = wrap360(incomingData.yaw);
    first_data_received = true; 
  }
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

// ==================== UWB (BU03) AT 指令驅動底層 ====================
bool checkUwbModule() {
  while(UwbSerial.available()) UwbSerial.read(); 
  UwbSerial.println("AT"); 
  delay(50); 
  String resp = "";
  while(UwbSerial.available()) {
    resp += (char)UwbSerial.read();
  }
  return (resp.indexOf("OK") != -1);
}

float getDistanceFast() {
  while(UwbSerial.available()) { UwbSerial.read(); }
  UwbSerial.println("AT+DISTANCE"); 
  
  unsigned long startTimeout = millis();
  String rawResponse = "";
  rawResponse.reserve(64); 
  
  while ((millis() - startTimeout) < 15) { 
    while (UwbSerial.available()) {
      char c = UwbSerial.read();
      rawResponse += c;
    }
    if (rawResponse.indexOf("OK") != -1) { 
      break;
    }
  }

  int keywordIndex = rawResponse.indexOf("distance:");
  if (keywordIndex != -1) {
    int startIndex = keywordIndex + 9; 
    int endIndex = rawResponse.indexOf("\r", startIndex); 
    if (endIndex == -1) endIndex = rawResponse.indexOf("\n", startIndex);
    
    if (endIndex != -1) {
      String distStr = rawResponse.substring(startIndex, endIndex);
      distStr.trim();          
      return distStr.toFloat(); 
    }
  }
  return -1.0; 
}

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}