#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/* ========= 腳位定義 (請依據您的 ESP32 實際接線調整) ========= */
const int PWMA = 13;
const int AIN2 = 14;
const int AIN1 = 27;
const int BIN1 = 26;
const int BIN2 = 25;
const int PWMB = 33;

// I2C 腳位定義 (ESP32 預設通常為 SDA=21, SCL=22)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

/* ========= ESP-NOW 資料結構 ========= */
typedef struct struct_message {
  float yaw;
  float pitch;
  float roll;
} struct_message;

struct_message incomingData;
volatile float target_yaw = 0.0f; // 已改為 volatile，確保跨核心/中斷存取安全

/* ========= 小車本地 MPU6050 變數 ========= */
Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float current_yaw = 0.0f;
const float ALPHA = 0.98f;
unsigned long prevMicros = 0;

/* ========= 追隨控制變數 ========= */
const double Kp_turn = 6.0;
const int MAX_SPEED = 130;
const int MIN_SPEED = 65;
bool mpu_ready = false;

/* ========= 執行緒排程時間變數 ========= */
unsigned long lastMpuReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 25; // 40Hz

/* ========= 工具函式 ========= */
static inline float rad2deg(float r) { 
  return r * 180.0f / PI;
}

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// 馬達驅動控制
void MotorWriting(double vL, double vR) {
  if (vR >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH); 
    digitalWrite(BIN2, LOW);
    vR = -vR;
  }
  
  if (vL >= 0) {
    digitalWrite(AIN1, LOW); 
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);  
    digitalWrite(AIN2, LOW);
    vL = -vL;
  }
  
  // ESP32 的 analogWrite 內部會自動配置 ledc 通道
  analogWrite(PWMA, (int)vL);
  analogWrite(PWMB, (int)vR);
}

/* ========= ESP-NOW 接收回呼函式 ========= */
// 注意：此函式在中斷環境執行，必須維持極簡，已移除 Serial.print 避免阻塞當機
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  target_yaw = wrap360(incomingData.yaw);
}

void setup() {
  // Serial1.begin(115200); // 註解掉此行：ESP32 若無重新定義腳位，使用預設 Serial1 可能會與 Flash 衝突
  Serial.begin(115200);
  Serial.println("\n--- ESP32 Single-Chip Controller Boot ---");
  Serial.print("This ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // 初始化馬達控制腳位
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);

  // 初始化 ESP32 的 I2C 總線
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  
  // 初始化 MPU6050
  if (!localMpu.begin()) {
    Serial.println("ERROR: Local MPU6050 missing!");
    mpu_ready = false;
  } else {
    Serial.println("Local MPU6050 Found!");
    localMpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    localMpu.setGyroRange(MPU6050_RANGE_250_DEG);
    localMpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Calibrating Local Gyro... Keep Car Still.");
    delay(400);
    float sum_gz = 0.0f;
    const int sample_count = 100; 
    for (int i = 0; i < sample_count; i++) {
      sensors_event_t a, g, temp;
      if (localMpu.getEvent(&a, &g, &temp)) {
        sum_gz += g.gyro.z * 180.0f / PI;
      }
      delay(2);
    }
    gz_offset = sum_gz / sample_count;
    mpu_ready = true;
    Serial.println("Gyro Calibration Done!");
  }

  // 初始化 Wi-Fi 與 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  prevMicros = micros();
  lastMpuReadTime = millis();
  Serial.println("--- System Online ---");
}

void loop() {
  unsigned long currentMillis = millis();
  
  /* 1. 時間排程非同步更新車子角度 */
  if (mpu_ready && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;
    sensors_event_t a, g, temp;
    if (localMpu.getEvent(&a, &g, &temp)) {
      unsigned long now = micros();
      float dt = (now - prevMicros) * 1e-6f;
      prevMicros = now;
      if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

      float roll_acc = rad2deg(atan2(a.acceleration.y, a.acceleration.z));
      float pitch_acc = rad2deg(atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)));
      roll = ALPHA * (roll + (g.gyro.x * 180.0f / PI) * dt) + (1.0f - ALPHA) * roll_acc;
      pitch = ALPHA * (pitch + (g.gyro.y * 180.0f / PI) * dt) + (1.0f - ALPHA) * pitch_acc;
      current_yaw += ((g.gyro.z * 180.0f / PI) - gz_offset) * dt;
      current_yaw = wrap360(current_yaw);
    }
  }

  /* 2. 最短路徑旋轉控制演算法 */
  float error = target_yaw - current_yaw;
  if (error > 180.0f)  error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  
  if (abs(error) < 2.0f) {
    MotorWriting(0, 0);
  } else {
    float control_output = error * Kp_turn;
    int motor_speed = (int)control_output;
    if (motor_speed > 0)  motor_speed += MIN_SPEED;
    if (motor_speed < 0)  motor_speed -= MIN_SPEED;
    motor_speed = constrain(motor_speed, -MAX_SPEED, MAX_SPEED);

    MotorWriting(motor_speed, -motor_speed);
  }

  /* 3. 固定定時除錯監控 (在這裡確認是否收到 ESP-NOW 資料) */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;
    Serial.print("Target: "); Serial.print(target_yaw);
    Serial.print(" | Current: "); Serial.print(current_yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }
}