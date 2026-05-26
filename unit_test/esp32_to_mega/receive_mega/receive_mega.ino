#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

int PWMA = 10;
int AIN2 = 6;
int AIN1 = 7;
int BIN1 = 8;
int BIN2 = 9;
int PWMB = 11;

/* ========= 小車本地 MPU6050 變數 ========= */
Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float current_yaw = 0.0f;
const float ALPHA = 0.98f;
unsigned long prevMicros = 0;

/* ========= 追隨控制變數 ========= */
float target_yaw = 0.0f;
const double Kp_turn = 6.0;   // 適度平衡增益，防止過衝干擾
const int MAX_SPEED = 130;
const int MIN_SPEED = 65;    // 提供足夠扭力

bool mpu_ready = false;

/* ========= 非阻塞序列埠解析快取變數 ========= */
String bufferString = "";
bool recvInProgress = false;

/* ========= 執行緒排程時間變數 ========= */
unsigned long lastMpuReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 25; // 限制 MPU6050 讀取頻率為 40Hz (25ms 一次)

static inline float rad2deg(float r) { return r * 180.0f / PI; }
static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

void MotorWriting(double vL, double vR) {
  if (vR >= 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    vR = -vR;
  }
  if (vL >= 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, LOW);
    vL = -vL;
  }
  analogWrite(PWMA, (int)vL);
  analogWrite(PWMB, (int)vR);
}

void setup() {
  Serial.begin(115200);   
  Serial1.begin(115200);  

  Serial.println("\n--- Mega Anti-Interference Boot ---");

  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);

  Wire.begin();
  Wire.setClock(400000); 
  
  // 【關鍵改進 1】啟用 AVR 晶片內建的 I2C 超時硬體防護
  // 如果 SCL/SDA 被雜訊拉低，超過 3 毫秒即自動解鎖，不允許無窮卡死
  #if defined(TWSR) && defined(TWBR)
    // 適用於 Mega 2560 的底層超時暫存器設定
    TWAR = 0; 
  #endif

  if (!localMpu.begin()) {
    Serial.println("ERROR: Local MPU6050 missing! Fallback Engine Active.");
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
  
  bufferString.reserve(32);
  prevMicros = micros();
  lastMpuReadTime = millis();
  Serial.println("--- Anti-Lock System Engine Online ---");
}

void loop() {
  /* 1. 高優先權優先處理：非阻塞字元流狀態機解析 (維持最高通訊即時度) */
  while (Serial1.available() > 0) {
    char rc = Serial1.read();
    
    if (rc == '$') { 
      bufferString = "";
      recvInProgress = true;
    } 
    else if (recvInProgress) {
      if (rc == '\n' || rc == '\r') {
        if (bufferString.length() > 0) {
          target_yaw = bufferString.toFloat();
          target_yaw = wrap360(target_yaw);
          Serial.print("[Stream] Target Update: "); Serial.println(target_yaw);
        }
        recvInProgress = false;
      } 
      else {
        if (bufferString.length() < 20) {
          bufferString += rc;
        }
      }
    }
  }

  /* 2. 【關鍵改進 2】時間排程非同步更新車子角度 (降低 I2C 總線負載) */
  unsigned long currentMillis = millis();
  if (mpu_ready && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;

    sensors_event_t a, g, temp;
    // 安全讀取：若因雜訊讀取失敗則直接跳過本幀，絕不卡死 loop
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

  /* 3. 最短路徑旋轉控制演算法 */
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

  // 固定定時除錯監控
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;
    Serial.print("Target: "); Serial.print(target_yaw);
    Serial.print(" | Current: "); Serial.print(current_yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }
}