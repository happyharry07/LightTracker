#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <HardwareSerial.h>

// ==================== 【系統參數配置】 ====================
uint8_t remoteAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x33, 0x3C}; 

const float REMOTE_HEIGHT = 1.00;         
const float PITCH_STABLE_THRESHOLD = 2.0; 
const float YAW_STABLE_THRESHOLD = 3.0;   // 發送端 Yaw 晃動容忍度
const unsigned long WAIT_STABLE_MS = 600; // 必須靜止 0.6 秒才判定為有效目標

// ==================== 全局座標系與推算變數 ====================
float X_car = 0.0f;
float Y_car = 0.0f;
float X_tgt = 0.0f;
float Y_tgt = 0.0f;

// UWB 濾波與位移計算
float filtered_uwb_dist = -1.0f;
float last_valid_uwb_dist = -1.0f;

// 運動狀態機定義
enum CarState {
  STATE_HOLD,    // 待命 / 煞車 / 等待目標穩定
  STATE_TURN,    // 原地旋轉對準目標
  STATE_FORWARD  // 直線前進至目標
};
CarState currentState = STATE_HOLD;

// ==================== 控制器參數 ====================
const float KP_dist = 180.0;             
const float DEAD_ZONE_DIST = 0.10;       // 到達目標的距離死區 10cm
const float TURN_ALIGN_TOLERANCE = 5.0;  // 對準目標的角度容差 (度)
const double Kp_turn = 5.0;   
const double Kd_turn = 0.8;   
float last_error_yaw = 0.0f;     
const int MAX_MOTOR_SPEED = 150;  
const int MIN_MOTOR_SPEED = 70;   

// ==================== 硬體與通訊配置 ====================
HardwareSerial UwbSerial(2);
const int BU03_RX = 16; 
const int BU03_TX = 17;

typedef struct struct_message {
  float yaw; float pitch; float roll; 
} __attribute__((packed)) struct_message; 

typedef struct return_message {
  float car_yaw; float distance; 
} __attribute__((packed)) return_message;

struct_message incomingData; 
return_message txData;        
esp_now_peer_info_t peerInfo;

volatile float remote_pitch = 0.0f;   
volatile float remote_yaw = 0.0f;     
volatile bool first_data_received = false;
volatile unsigned long packet_count = 0; 
float last_remote_pitch = 0.0f;
float last_remote_yaw = 0.0f;
unsigned long stableStartTime = 0;

const int PWMA = 13; const int AIN2 = 14; const int AIN1 = 27;
const int BIN1 = 26; const int BIN2 = 25; const int PWMB = 33;
const int I2C_SDA = 21; const int I2C_SCL = 22;

Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float current_yaw = 0.0f;      
unsigned long prevMicros = 0;
bool mpu_ready = false;
bool system_fully_ready = false; 

// ==================== 前置宣告 ====================
static inline float wrap360(float a);
static inline float wrap180(float a);
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
    
    Serial.println("====== MPU6050 Calibrating... PLEASE KEEP CAR STILL ======");
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
  }

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  delay(200);
  checkUwbModule();
  
  // 嚴格初始化座標：開機時取 10 次平均，建立精準的起始 X 軸
  Serial.println("====== Locking Initial Coordinate ======");
  float sum_dist = 0;
  int valid_reads = 0;
  for(int i=0; i<15; i++) {
    float d = getDistanceFast();
    if(d > 0) { sum_dist += d; valid_reads++; }
    delay(40);
  }
  
  if(valid_reads > 0) {
    filtered_uwb_dist = sum_dist / valid_reads;
    last_valid_uwb_dist = filtered_uwb_dist;
    X_car = filtered_uwb_dist; 
    Y_car = 0.0f;
    Serial.printf("Init Success! X: %.2f, Y: 0.00\n", X_car);
  }

  prevMicros = micros();
}

void loop() {
  unsigned long currentMillis = millis();
  
  /* ==================== 1. 更新感測器與座標推算 ==================== */
  // A. 陀螺儀更新 (15ms 一次)
  static unsigned long lastMpuTime = 0;
  if (mpu_ready && (currentMillis - lastMpuTime >= 15)) {
    lastMpuTime = currentMillis;
    unsigned long now = micros();
    float dt = (now - prevMicros) * 1e-6f;
    prevMicros = now;
    if (dt <= 0.0f || dt > 0.04f) dt = 0.015f; 

    sensors_event_t a, g, temp;
    if (localMpu.getEvent(&a, &g, &temp)) {
      float gz_deg = (g.gyro.z * 180.0f / PI) - gz_offset;
      if (abs(gz_deg) > 0.15f) current_yaw += gz_deg * dt;
      current_yaw = wrap360(current_yaw); 
    }
  }

  // B. UWB 測距與位移幾何推算 (50ms 一次)
  static unsigned long lastUwbTime = 0;
  if (currentMillis - lastUwbTime >= 50) {
    lastUwbTime = currentMillis;
    float raw_dist = getDistanceFast();
    
    if (raw_dist > 0 && system_fully_ready) {
      // 低通濾波消除雜訊突波
      filtered_uwb_dist = (filtered_uwb_dist < 0) ? raw_dist : (0.7 * filtered_uwb_dist + 0.3 * raw_dist);
      
      // 【核心邏輯】：只在「直線前進」狀態下，才把 UWB 的變化量轉換為 XY 位移
      if (currentState == STATE_FORWARD) {
        float delta_r = filtered_uwb_dist - last_valid_uwb_dist;
        
        // 算出從原點指向車子的單位向量
        float r_mag = sqrt(X_car*X_car + Y_car*Y_car);
        if (r_mag > 0.1f) {
          float rx = X_car / r_mag; 
          float ry = Y_car / r_mag;
          
          // 算出車子前進的朝向向量
          float yaw_rad = current_yaw * PI / 180.0f;
          float hx = cos(yaw_rad);
          float hy = sin(yaw_rad);
          
          // 計算朝向向量在半徑向量上的投影量 cos(alpha)
          float cos_alpha = rx * hx + ry * hy;
          
          // 避開「繞圓切線運動」造成的除以零盲區
          if (abs(cos_alpha) > 0.15f) {
            float d_moved = delta_r / cos_alpha;
            
            // 物理限制：50ms 內不可能移動超過 10 公分，過濾掉異常跳變
            if (abs(d_moved) < 0.1f) {
              X_car += d_moved * hx;
              Y_car += d_moved * hy;
            }
          }
        }
      }
      last_valid_uwb_dist = filtered_uwb_dist;
    }
  }

  /* ==================== 2. 安全解鎖檢查 ==================== */
  if (!system_fully_ready && packet_count > 5) {
    system_fully_ready = true;
  }

  /* ==================== 3. 狀態機導航控制器 ==================== */
  double speedL = 0, speedR = 0;
  
  if (system_fully_ready) {
    // 判斷遙控器是否穩定
    bool cmd_stable = (currentMillis - stableStartTime >= WAIT_STABLE_MS);

    if (!cmd_stable) {
      currentState = STATE_HOLD;
    } else {
      // 計算目標座標
      float pitchRad = abs(remote_pitch) * PI / 180.0f;
      if(pitchRad < 0.05f) pitchRad = 0.05f;
      float R_tgt = REMOTE_HEIGHT / tan(pitchRad);
      if(R_tgt > 4.5f) R_tgt = 4.5f;
      
      float remYawRad = remote_yaw * PI / 180.0f;
      X_tgt = R_tgt * cos(remYawRad);
      Y_tgt = R_tgt * sin(remYawRad);

      // 計算與目標點的向量
      float dX = X_tgt - X_car;
      float dY = Y_tgt - Y_car;
      float target_dist = sqrt(dX*dX + dY*dY);
      
      float target_yaw_deg = atan2(dY, dX) * 180.0f / PI;
      if(target_yaw_deg < 0) target_yaw_deg += 360.0f;
      
      float error_yaw = wrap180(target_yaw_deg - current_yaw);

      // 狀態切換邏輯
      if (target_dist <= DEAD_ZONE_DIST) {
        currentState = STATE_HOLD; // 到達目標
      } else if (abs(error_yaw) > TURN_ALIGN_TOLERANCE) {
        currentState = STATE_TURN; // 角度沒對準，強制原地轉向
      } else {
        currentState = STATE_FORWARD; // 角度對準，直線衝刺
      }

      // 依據狀態輸出推力
      if (currentState == STATE_HOLD) {
        speedL = 0; speedR = 0;
        last_error_yaw = 0;
      } 
      else if (currentState == STATE_TURN) {
        // 純粹原地轉向 (只用 PD 控制器)
        float turnOutput = error_yaw * Kp_turn + (error_yaw - last_error_yaw) * Kd_turn;
        last_error_yaw = error_yaw;
        
        speedL = -turnOutput;
        speedR = turnOutput;
      } 
      else if (currentState == STATE_FORWARD) {
        // 純粹直線前進 (P 控制器)
        float distOutput = (target_dist - DEAD_ZONE_DIST) * KP_dist;
        
        // 邊走邊稍微修正偏移 (維持走直線)
        float turnAssist = error_yaw * (Kp_turn * 0.5); 
        
        speedL = distOutput - turnAssist;
        speedR = distOutput + turnAssist;
      }

      // 馬達綜合限速
      if (abs(speedL) > 0.1 || abs(speedR) > 0.1) {
        speedL = constrain(speedL, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        speedR = constrain(speedR, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        if (speedL > 0 && speedL < MIN_MOTOR_SPEED) speedL = MIN_MOTOR_SPEED;
        if (speedL < 0 && speedL > -MIN_MOTOR_SPEED) speedL = -MIN_MOTOR_SPEED;
        if (speedR > 0 && speedR < MIN_MOTOR_SPEED) speedR = MIN_MOTOR_SPEED;
        if (speedR < 0 && speedR > -MIN_MOTOR_SPEED) speedR = -MIN_MOTOR_SPEED;
      }
    }
  }

  // --- 加速度限制器 (Slew Rate Limiter) ---
  static double actual_speedL = 0, actual_speedR = 0;
  const double MAX_ACCEL = 15.0; 
  
  if (speedL > actual_speedL + MAX_ACCEL) actual_speedL += MAX_ACCEL;
  else if (speedL < actual_speedL - MAX_ACCEL) actual_speedL -= MAX_ACCEL;
  else actual_speedL = speedL;

  if (speedR > actual_speedR + MAX_ACCEL) actual_speedR += MAX_ACCEL;
  else if (speedR < actual_speedR - MAX_ACCEL) actual_speedR -= MAX_ACCEL;
  else actual_speedR = speedR;

  MotorWriting(actual_speedL, actual_speedR);

  /* ==================== 4. 主動回傳與序列埠輸出 ==================== */
  static unsigned long lastTxTime = 0;
  if (system_fully_ready && (currentMillis - lastTxTime >= 60)) {
    lastTxTime = currentMillis;
    txData.car_yaw = current_yaw; 
    txData.distance = filtered_uwb_dist; 
    esp_now_send(remoteAddress, (uint8_t *) &txData, sizeof(txData));
  }

  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 250) {
    lastPrint = currentMillis;
    if (system_fully_ready) {
      String stateStr = (currentState == STATE_HOLD) ? "[HOLD] " : (currentState == STATE_TURN) ? "[TURN] " : "[GO!!] ";
      Serial.print(stateStr);
      Serial.printf("Car(%.2f, %.2f) -> Tgt(%.2f, %.2f) | ", X_car, Y_car, X_tgt, Y_tgt);
      Serial.printf("Yaw: %.0f/%.0f | Motor: %d/%d\n", current_yaw, wrap360(atan2(Y_tgt-Y_car, X_tgt-X_car)*180/PI), (int)actual_speedL, (int)actual_speedR);
    }
  }
}

// ==================== 底層控制與通訊函數 ====================
void MotorWriting(double vL, double vR) {
  if (vR >= 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); } 
  else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); vR = -vR; }
  if (vL >= 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); } 
  else { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); vL = -vL; }
  analogWrite(PWMA, (int)vL); analogWrite(PWMB, (int)vR);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len >= sizeof(struct_message)) {
    memcpy(&incomingData, incomingDataRaw, sizeof(struct_message));
    packet_count++;
    
    float new_pitch = incomingData.pitch;
    float new_yaw = incomingData.yaw;
    float pitch_delta = abs(new_pitch - last_remote_pitch);
    float yaw_delta = abs(wrap180(new_yaw - last_remote_yaw));
    
    last_remote_pitch = new_pitch; last_remote_yaw = new_yaw;
    remote_pitch = new_pitch; remote_yaw = new_yaw;

    if (pitch_delta > PITCH_STABLE_THRESHOLD || yaw_delta > YAW_STABLE_THRESHOLD) {
      stableStartTime = millis(); 
    }
    first_data_received = true; 
  }
}

bool checkUwbModule() {
  while(UwbSerial.available()) UwbSerial.read(); 
  UwbSerial.println("AT"); delay(50); 
  String resp = ""; while(UwbSerial.available()) { resp += (char)UwbSerial.read(); }
  return (resp.indexOf("OK") != -1);
}

float getDistanceFast() {
  while(UwbSerial.available()) UwbSerial.read(); 
  UwbSerial.println("AT+DISTANCE"); 
  unsigned long startTimeout = millis();
  String rawResponse = ""; rawResponse.reserve(64); 
  while ((millis() - startTimeout) < 15) { 
    while (UwbSerial.available()) rawResponse += (char)UwbSerial.read(); 
    if (rawResponse.indexOf("OK") != -1) break; 
  }
  int keywordIndex = rawResponse.indexOf("distance:");
  if (keywordIndex != -1) {
    int startIndex = keywordIndex + 9; 
    int endIndex = rawResponse.indexOf("\r", startIndex); 
    if (endIndex == -1) endIndex = rawResponse.indexOf("\n", startIndex);
    if (endIndex != -1) return rawResponse.substring(startIndex, endIndex).toFloat(); 
  }
  return -1.0; 
}

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}