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
    delay(500);
    float sum_gz = 0.0f;
    for (int i = 0; i < 250; i++) {
      sensors_event_t a, g, temp;
      if (localMpu.getEvent(&a, &g, &temp)) sum_gz += g.gyro.z * 180.0f / PI;
      delay(2);
    }
    gz_offset = sum_gz / 250.0f;
    
    // 【核心修正】：開機校正後的起點，強制設定為小車宇宙的絕對「0.0度」
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

  delay(500);
  checkUwbModule();

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
  
  /* ==================== 1. 定時更新小車絕對角度（留著校正、自由運算） ==================== */
  if (mpu_ready && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;
    sensors_event_t a, g, temp;
    if (localMpu.getEvent(&a, &g, &temp)) {
      unsigned long now = micros();
      float dt = (now - prevMicros) * 1e-6f;
      prevMicros = now;
      if (dt <= 0.0f || dt > 0.04f) dt = 0.015f; 

      float gz_deg = (g.gyro.z * 180.0f / PI) - gz_offset;
      if (abs(gz_deg) > 0.55f) { // 門檻值過濾馬達微小機體震動
        current_yaw += gz_deg * dt;
      }
      current_yaw = wrap360(current_yaw); // 保持在 0~360 之間自由變動
    }
  }

  /* ==================== 2. 指向追隨運動控制演算法 ==================== */
  double speedL = 0;
  double speedR = 0;

  // 只要收到通訊且陀螺儀就緒，立刻進行實時 PD 閉迴路控制
  if (first_data_received && mpu_ready) {
    
    float pitchRadian = abs(remote_pitch) * PI / 180.0f; 
    if (pitchRadian < 0.05f) pitchRadian = 0.05f; 
    
    // ---- A. 幾何計算：計算目標【斜邊距離】 ----
    dynamicTargetSlantDistance = REMOTE_HEIGHT / sin(pitchRadian);
    
    // 限制範圍限制
    if (dynamicTargetSlantDistance > 4.5f) dynamicTargetSlantDistance = 4.5f;
    if (dynamicTargetSlantDistance < 1.0f) dynamicTargetSlantDistance = 1.0f;

    // ---- B. 直線偏擺修正（防爆限幅版：永遠拿絕對 0 度來減 current_yaw） ----
    float error_yaw = 0.0f - current_yaw; // 目標固定為 0
    if (error_yaw > 180.0f)  error_yaw -= 360.0f;
    if (error_yaw < -180.0f) error_yaw += 360.0f;
    
    // 【防爆修正 1】限制角度誤差的最大補正衝擊力（最大只允許補正 45 度）
    // 防止誤差接近 180 度時，turnOutput 爆表導致馬達正負號瘋狂抖動而觸發重設
    if (error_yaw > 45.0f)  error_yaw = 45.0f;
    if (error_yaw < -45.0f) error_yaw = -45.0f;

    float error_diff_yaw = error_yaw - last_error_yaw;
    last_error_yaw = error_yaw;
    
    float turnOutput = error_yaw * Kp_turn + error_diff_yaw * Kd_turn;

    // 【防爆修正 2】限制 turnOutput 的絕對最大出力，不讓轉向完全吃掉馬達頻寬
    const float MAX_TURN_OUTPUT = 80.0f; 
    if (turnOutput > MAX_TURN_OUTPUT)  turnOutput = MAX_TURN_OUTPUT;
    if (turnOutput < -MAX_TURN_OUTPUT) turnOutput = -MAX_TURN_OUTPUT;

    // ---- C. 距離前後追隨推力 ----
    float distOutput = 0;
    
    // 檢查發送端角度是否已經保持不動超過 0.5 秒 (500ms)
    if (currentMillis - stableStartTime >= WAIT_STABLE_MS) {
      is_command_valid = true;
    } else {
      is_command_valid = false;
    }

    if (!is_command_valid) {
      distOutput = 0; // 指令未合法確認前，只修正角度直行，不給予前後前進推力
    } else {
      if (currentDistance >= 0) {
        // 直接比對 UWB 的實測斜邊距離與目標斜邊距離
        float error_dist = currentDistance - dynamicTargetSlantDistance;
        
        if (abs(error_dist) > DEAD_ZONE_DIST) {
          distOutput = error_dist * KP_dist; // 衝過頭誤差為負時，馬達會直接反轉後退，絕不迴轉
        }
      }
    }

    // ---- D. 混控核心（結合直行推力與偏擺修正，隨時收拾 current_yaw 偏離 0 度的殘局） ----
    speedL = distOutput + turnOutput;
    speedR = distOutput - turnOutput;

    // ---- E. 限制最大/最小速度，克服馬達靜摩擦力 ----
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

  // 寫入馬達
  MotorWriting(speedL, speedR);

  /* ==================== 3. 定時主動回傳狀態給遙控器 ==================== */
  if (first_data_received && (currentMillis - lastTxTime >= TX_INTERVAL)) {
    lastTxTime = currentMillis;
    txData.car_yaw = current_yaw; // 把小車實時更新的真實角度回傳給遙控器監控
    txData.distance = currentDistance; 
    esp_now_send(remoteAddress, (uint8_t *) &txData, sizeof(txData));
  }

  /* ==================== 4. 定時序列埠監控 ==================== */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 250) {
    lastPrint = currentMillis;
    Serial.print("TgtYaw: 0.0");
    Serial.print(" | CarYaw: ");    Serial.print(current_yaw, 1);
    Serial.print(" | Pitch: ");   Serial.print(remote_pitch, 1);
    Serial.print(" | CmdStatus: "); Serial.print(is_command_valid ? "VALID" : "HOLD ");
    Serial.print(" | UWB Slant/Tgt: "); Serial.print(currentDistance, 2); Serial.print("/"); Serial.print(dynamicTargetSlantDistance, 2);
    Serial.print(" | Motor: "); Serial.print((int)speedL); Serial.print("/"); Serial.println((int)speedR);
  }
}

// ==================== 馬達驅動核心 ====================
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
    
    float new_pitch = incomingData.pitch;
    float pitch_delta = abs(new_pitch - last_remote_pitch);
    last_remote_pitch = new_pitch;
    remote_pitch = new_pitch;

    // 狀態機：如果手在晃動，就不斷刷新計時器；手停住不動時，計時器才會開始累積時間
    if (pitch_delta > PITCH_STABLE_THRESHOLD) {
      stableStartTime = millis(); // 晃動中，重設 0.5 秒的起算點
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