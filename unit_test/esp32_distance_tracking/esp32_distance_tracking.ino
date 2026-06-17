#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <HardwareSerial.h>

// 發送端（遙控器）的 MAC 地址
uint8_t remoteAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x33, 0x3C}; 

// ==================== 【可調參數】 ====================
const float REMOTE_HEIGHT = 1.00;         // 單位：公尺。手持遙控器離地面的垂直高度
const float PITCH_STABLE_THRESHOLD = 2.0; // Pitch 變化超過此角度（度），視為晃動中
const unsigned long WAIT_STABLE_MS = 500; // Pitch 必須停止變化並保持 0.5 秒，才視為合法指令

// ==================== UWB (BU03-Kit) 配置 ====================
HardwareSerial UwbSerial(2);
const int BU03_RX = 16; 
const int BU03_TX = 17;
const unsigned long UWB_INTERVAL = 50;  
unsigned long lastUwbTime = 0;
float currentDistance = -1.0;           // 儲存當前最新 UWB 直線距離（斜邊）

// ==================== 距離控制參數 (P 控制) ====================
float dynamicTargetSlantDistance = 1.414; // 由遙控器俯角計算出來的目標【斜邊】距離
const float KP_dist = 220.0;             // 距離比例增益
const float DEAD_ZONE_DIST = 0.05;       // 距離死區 5cm

// ==================== 角度控制參數 (目標固定 0 度，PD 追隨) ====================
const double Kp_turn = 6.0;   // 角度比例增益
const double Kd_turn = 0.8;   
float last_error_yaw = 0.0f;     

// ==================== 馬達綜合限速 ====================
const int MAX_MOTOR_SPEED = 150;  // 限速，確保到點不暴衝
const int MIN_MOTOR_SPEED = 70;   

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

volatile float remote_pitch = 0.0f;   
volatile bool first_data_received = false;
volatile unsigned long packet_count = 0; // 計算開機後收到的合法封包數，過濾開機混亂期

// 追蹤 Pitch 穩定度的變數
float last_remote_pitch = 0.0f;
unsigned long stableStartTime = 0;
bool is_command_valid = false; // 是否通過 0.5 秒保持考驗的合法指令

/* ========= 馬達與 I2C 腳位定義 ========= */
const int PWMA = 13; const int AIN2 = 14; const int AIN1 = 27;
const int BIN1 = 26; const int BIN2 = 25; const int PWMB = 33;
const int I2C_SDA = 21; const int I2C_SCL = 22;

Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float current_yaw = 0.0f;      // 小車實時運算、累積的當前絕對角度
unsigned long prevMicros = 0;

bool mpu_ready = false;
bool system_fully_ready = false; // 整體系統安全就緒旗標
unsigned long lastMpuReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 15; // 採樣時間，讓動態角度更新更靈敏 

unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 60; 

// 前置宣告
static inline float wrap360(float a);
void MotorWriting(double vL, double vR);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len);
bool checkUwbModule();
float getDistanceFast();

void setup() {
  Serial.begin(115200);
  UwbSerial.begin(115200, SERIAL_8N1, BU03_RX, BU03_TX);
  
  // 嚴格阻斷：初始化第一時間強制關閉馬達，防止浮空電壓干擾
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (localMpu.begin()) {
    localMpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    localMpu.setGyroRange(MPU6050_RANGE_250_DEG);
    localMpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // 陀螺儀精準靜止校正（開機時請將車體擺正、靜止放置）
    Serial.println("====== MPU6050 Calibrating... PLEASE KEEP CAR STILL ======");
    MotorWriting(0, 0); // 再次確保校正期間馬達絕對不出力
    delay(600);
    
    float sum_gz = 0.0f;
    for (int i = 0; i < 250; i++) {
      sensors_event_t a, g, temp;
      if (localMpu.getEvent(&a, &g, &temp)) sum_gz += g.gyro.z * 180.0f / PI;
      delay(2);
    }
    gz_offset = sum_gz / 250.0f;
    
    current_yaw = 0.0f; 
    mpu_ready = true;
    Serial.println("====== Calibration Success! Initial Car Yaw locked at 0.0 ======");
  }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  delay(200);
  checkUwbModule();

  MotorWriting(0, 0); // 確保開機完畢時馬達處於煞停狀態

  prevMicros = micros();
  lastMpuReadTime = millis();
  lastTxTime = millis();
  lastUwbTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  /* ==================== 0. 定時獲取 UWB 距離 ==================== */
  if (currentMillis - lastUwbTime >= UWB_INTERVAL) {
    lastUwbTime = currentMillis;
    float distRead = getDistanceFast();
    if (distRead >= 0) {
      currentDistance = distRead; 
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
      if (dt <= 0.0f || dt > 0.04f) dt = 0.015f; 

      float gz_deg = (g.gyro.z * 180.0f / PI) - gz_offset;
      
      // 【核心修正】：將防震死區門檻從 0.55f 降到 0.15f
      if (abs(gz_deg) > 0.15f) { 
        current_yaw += gz_deg * dt;
      }
      current_yaw = wrap360(current_yaw); 
    }
  }

  /* ==================== 2. 安全解鎖檢查機制 ==================== */
  if (!system_fully_ready) {
    if (mpu_ready && first_data_received && packet_count > 5) {
      system_fully_ready = true;
      Serial.println("====== SAFETY LOCK UNLOCKED: MOTOR SYSTEM ACTIVE ======");
    }
  }

  /* ==================== 3. 指向追隨運動控制演算法 ==================== */
  double speedL = 0;
  double speedR = 0;

  if (system_fully_ready) {
    
    float pitchRadian = abs(remote_pitch) * PI / 180.0f; 
    if (pitchRadian < 0.05f) pitchRadian = 0.05f; 
    
    // ---- A. 幾何計算：計算目標【斜邊距離】 ----
    dynamicTargetSlantDistance = REMOTE_HEIGHT / sin(pitchRadian);
    
    if (dynamicTargetSlantDistance > 4.5f) dynamicTargetSlantDistance = 4.5f;
    if (dynamicTargetSlantDistance < 1.0f) dynamicTargetSlantDistance = 1.0f;

    // ---- B. 直線偏擺修正 ----
    float error_yaw = 0.0f - current_yaw; 
    
    while (error_yaw > 180.0f)  error_yaw -= 360.0f;
    while (error_yaw < -180.0f) error_yaw += 360.0f;
    
    if (error_yaw > 45.0f)  error_yaw = 45.0f;
    if (error_yaw < -45.0f) error_yaw = -45.0f;

    float error_diff_yaw = error_yaw - last_error_yaw;
    
    // 【修正 1】：收攏微分誤差，消除 180 度瞬間跳變產生的 360 度突波
    while (error_diff_yaw > 180.0f)  error_diff_yaw -= 360.0f;
    while (error_diff_yaw < -180.0f) error_diff_yaw += 360.0f;
    
    last_error_yaw = error_yaw;
    
    // ---- 加入「角度死區」聯鎖 ----
    float turnOutput = 0;
    if (abs(error_yaw) > 5.0f) {
      turnOutput = error_yaw * Kp_turn + error_diff_yaw * Kd_turn;
    }

    // ---- C. 距離前後追隨推力 ----
    float distOutput = 0;
    
    if (currentMillis - stableStartTime >= WAIT_STABLE_MS) {
      is_command_valid = true;
    } else {
      is_command_valid = false;
    }

    if (!is_command_valid) {
      distOutput = 0; 
    } else {
      if (currentDistance >= 0) {
        float error_dist = currentDistance - dynamicTargetSlantDistance;
        if (abs(error_dist) > DEAD_ZONE_DIST) {
          distOutput = error_dist * KP_dist; 
        }
      }
    }

    // ---- D. 混控核心 ----
    // 【修正 2】：對調轉向輸出極性，解決正回饋暴衝遠離 0 度的問題
    speedL = distOutput - turnOutput;
    speedR = distOutput + turnOutput;

    // ---- E. 到站絕對強行煞停死區 ----
    if (abs(error_yaw) < 3.0f && (!is_command_valid || (currentDistance >= 0 && abs(currentDistance - dynamicTargetSlantDistance) <= DEAD_ZONE_DIST))) {
      speedL = 0;
      speedR = 0;
      last_error_yaw = 0; // 清空偏擺微分，防止殘留震盪
    } else {
      // 限制最大/最小速度，克服馬達靜摩擦力
      if (abs(speedL) > 0.1) {
        if (speedL > MAX_MOTOR_SPEED)  speedL = MAX_MOTOR_SPEED;
        if (speedL < -MAX_MOTOR_SPEED) speedL = -MAX_MOTOR_SPEED;
        if (speedL > 0 && speedL < MIN_MOTOR_SPEED)  speedL = MIN_MOTOR_SPEED;
        if (speedL < 0 && speedL > -MIN_MOTOR_SPEED) speedL = -MIN_MOTOR_SPEED;
      }
      if (abs(speedR) > 0.1) {
        if (speedR > MAX_MOTOR_SPEED)  speedR = MAX_MOTOR_SPEED;
        if (speedR < -MAX_MOTOR_SPEED) speedR = -MAX_MOTOR_SPEED;
        if (speedR > 0 && speedR < MIN_MOTOR_SPEED)  speedR = MIN_MOTOR_SPEED;
        if (speedR < 0 && speedR > -MIN_MOTOR_SPEED) speedR = -MIN_MOTOR_SPEED;
      }
    }
  } else {
    // 系統未就緒前，強制速度為零
    speedL = 0; speedR = 0;
  }

  // --- 【修正 3】：加速度限制器核心 (Slew Rate Limiter) ---
  // 防止馬達瞬間吃載過大導致 ESP32 褐出重啟 (Brownout)
  static double actual_speedL = 0;
  static double actual_speedR = 0;
  const double MAX_ACCEL = 15.0; // 每個迴圈允許的最大加速度（可依據實測反應靈敏度微調）

  if (speedL > actual_speedL + MAX_ACCEL) actual_speedL += MAX_ACCEL;
  else if (speedL < actual_speedL - MAX_ACCEL) actual_speedL -= MAX_ACCEL;
  else actual_speedL = speedL;

  if (speedR > actual_speedR + MAX_ACCEL) actual_speedR += MAX_ACCEL;
  else if (speedR < actual_speedR - MAX_ACCEL) actual_speedR -= MAX_ACCEL;
  else actual_speedR = speedR;

  // 寫入馬達硬體
  MotorWriting(actual_speedL, actual_speedR);

  /* ==================== 4. 定時主動回傳狀態給遙控器 ==================== */
  if (system_fully_ready && (currentMillis - lastTxTime >= TX_INTERVAL)) {
    lastTxTime = currentMillis;
    txData.car_yaw = current_yaw; 
    txData.distance = currentDistance; 
    esp_now_send(remoteAddress, (uint8_t *) &txData, sizeof(txData));
  }

  /* ==================== 5. 定時序列埠監控 ==================== */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 250) {
    lastPrint = currentMillis;
    if (!system_fully_ready) {
      Serial.print("[SYSTEM LOCKING] Waiting for stable orientation... Packet Count: ");
      Serial.println(packet_count);
    } else {
      Serial.print("TgtYaw: 0.0");
      Serial.print(" | CarYaw: ");    Serial.print(current_yaw, 1);
      Serial.print(" | Pitch: ");   Serial.print(remote_pitch, 1);
      Serial.print(" | CmdStatus: "); Serial.print(is_command_valid ? "VALID" : "HOLD ");
      Serial.print(" | UWB Slant/Tgt: "); Serial.print(currentDistance, 2); Serial.print("/"); Serial.print(dynamicTargetSlantDistance, 2);
      Serial.print(" | Motor(Cmd/Act): "); Serial.print((int)speedL); Serial.print("/"); Serial.print((int)speedR);
      Serial.print(" -> "); Serial.print((int)actual_speedL); Serial.print("/"); Serial.println((int)actual_speedR);
    }
  }
}

// ==================== 馬達驅動底層 ====================
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
    
    packet_count++; // 累加合法封包數
    
    float new_pitch = incomingData.pitch;
    float pitch_delta = abs(new_pitch - last_remote_pitch);
    last_remote_pitch = new_pitch;
    remote_pitch = new_pitch;

    // 如果手部姿態晃動大於門檻值，重設 0.5 秒起算計時器
    if (pitch_delta > PITCH_STABLE_THRESHOLD) {
      stableStartTime = millis(); 
    }
    
    first_data_received = true; 
  }
}

// ==================== UWB (BU03) AT 指令驅動底層 ====================
bool checkUwbModule() {
  while(UwbSerial.available()) UwbSerial.read(); 
  UwbSerial.println("AT"); 
  delay(50); 
  String resp = "";
  while(UwbSerial.available()) { resp += (char)UwbSerial.read(); }
  return (resp.indexOf("OK") != -1);
}

float getDistanceFast() {
  while(UwbSerial.available()) { UwbSerial.read(); }
  UwbSerial.println("AT+DISTANCE"); 
  unsigned long startTimeout = millis();
  String rawResponse = "";
  rawResponse.reserve(64); 
  while ((millis() - startTimeout) < 15) { 
    while (UwbSerial.available()) { rawResponse += (char)UwbSerial.read(); }
    if (rawResponse.indexOf("OK") != -1) { break; }
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