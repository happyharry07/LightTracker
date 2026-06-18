#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

typedef struct struct_message {
  float yaw; float pitch; float roll;
} __attribute__((packed)) struct_message; 

struct_message incomingData;
volatile float raw_target_yaw = 0.0f; 
volatile bool first_data_received = false;

/* ========= 腳位定義 ========= */
const int PWMA = 13; const int AIN2 = 14; const int AIN1 = 27;
const int BIN1 = 26; const int BIN2 = 25; const int PWMB = 33;
const int I2C_SDA = 21; const int I2C_SCL = 22;

Adafruit_MPU6050 localMpu;
float gz_offset = 0.0f;
float current_yaw = 0.0f;
unsigned long prevMicros = 0;

float yaw_offset = 0.0f;    
bool origin_set = false;    

const double Kp_turn = 2;   
const double Kd_turn = 1;  
float last_error = 0.0f;     
bool is_moving = false;

const int MAX_SPEED = 130;  
const int MIN_SPEED = 85;     
bool mpu_ready = false;

unsigned long lastMpuReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 20; // 嚴格的 20 毫秒排程 (50Hz)

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

void MotorWriting(double vL, double vR) {
  if (vL == 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);   
  } else if (vL > 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, (int)vL);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  analogWrite(PWMA, (int)-vL);
  }
  if (vR == 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);   
  } else if (vR > 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, (int)vR);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  analogWrite(PWMB, (int)-vR);
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataRaw, int len) {
  if (len >= sizeof(struct_message)) {
    memcpy(&incomingData, incomingDataRaw, sizeof(struct_message));
    raw_target_yaw = wrap360(incomingData.yaw);
    first_data_received = true; 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);

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

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  prevMicros = micros();
  lastMpuReadTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  /* 1. 【防干擾修正】定時更新小車絕對角度 */
  if (mpu_ready && (currentMillis - lastMpuReadTime >= MPU_READ_INTERVAL)) {
    lastMpuReadTime = currentMillis;
    sensors_event_t a, g, temp;
    if (localMpu.getEvent(&a, &g, &temp)) {
      unsigned long now = micros();
      float dt = (now - prevMicros) * 1e-6f;
      prevMicros = now;
      
      // 強制限制 dt 的合理範圍，不讓通訊延遲帶偏積分
      if (dt <= 0.0f || dt > 0.04f) dt = 0.02f; 

      float gz_deg = (g.gyro.z * 180.0f / PI) - gz_offset;
      if (abs(gz_deg) > 0.50f) { // 微調死區
        current_yaw += gz_deg * dt;
      }
      current_yaw = wrap360(current_yaw); 
    }
  }

  /* 2. 鎖定原點 Offset */
  if (first_data_received && mpu_ready && !origin_set) {
    float diff = raw_target_yaw - current_yaw;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    yaw_offset = raw_target_yaw - diff; 
    origin_set = true;
  }

  if (!origin_set) {
    MotorWriting(0, 0); return;
  }

  /* 3. 絕對座標控制演算法 */
  float translated_target_yaw = wrap360(raw_target_yaw - yaw_offset);
  float error = translated_target_yaw - current_yaw;
  
  if (error > 180.0f)  error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  
  float abs_error = abs(error);

  if (!is_moving) {
    if (abs_error > 5.0f) is_moving = true;
  } else {
    if (abs_error < 1.5f) is_moving = false;
  }

  if (!is_moving) {
    MotorWriting(0, 0); last_error = error;
  } else {
    float error_diff = error - last_error; 
    last_error = error;
    float control_output = error * Kp_turn + error_diff * Kd_turn; 
    int motor_speed = (int)control_output;
    
    float min_speed_factor = 1.0f;
    if (abs_error < 10.0f) min_speed_factor = (abs_error - 1.5f) / (10.0f - 1.5f); 
    int dynamic_min_speed = (int)(MIN_SPEED * min_speed_factor);

    if (motor_speed > 0)  motor_speed += dynamic_min_speed;
    if (motor_speed < 0)  motor_speed -= dynamic_min_speed;
    
    motor_speed = constrain(motor_speed, -MAX_SPEED, MAX_SPEED);
    MotorWriting(motor_speed, -motor_speed);
  }

  /* 4. 定時監控 */
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 300) {
    lastPrint = currentMillis;
    Serial.print("Raw_Tgt: "); Serial.print(raw_target_yaw);
    Serial.print(" | Trans_Tgt: "); Serial.print(translated_target_yaw);
    Serial.print(" | Current: "); Serial.print(current_yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }
}