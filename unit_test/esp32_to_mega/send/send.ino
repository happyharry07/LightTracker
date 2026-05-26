#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0x0C, 0x32, 0x1C};  

Adafruit_MPU6050 mpu;
float gz_offset = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

const float ALPHA = 0.98f;  
unsigned long prevMicros = 0;

typedef struct struct_message {
  float yaw;
  float pitch;
  float roll;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

static inline float rad2deg(float r) {
  return r * 180.0f / PI;
}

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Fail");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Wire.begin(21, 22);  
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Please check wiring and power.");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating gyroscope... Keep MPU6050 still!");
  delay(500); 
  float sum_gz = 0.0f;
  const int sample_count = 300;

  for (int i = 0; i < sample_count; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum_gz += g.gyro.z * 180.0f / PI;
    delay(5);
  }
  gz_offset = sum_gz / sample_count;
  Serial.print("Gyro Z Offset calibrated: ");
  Serial.println(gz_offset, 4);

  prevMicros = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float gx_dps = g.gyro.x * 180.0f / PI;
  float gy_dps = g.gyro.y * 180.0f / PI;
  float gz_dps = g.gyro.z * 180.0f / PI;

  float roll_acc = rad2deg(atan2(ay, az));
  float pitch_acc = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

  roll = ALPHA * (roll + gx_dps * dt) + (1.0f - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + gy_dps * dt) + (1.0f - ALPHA) * pitch_acc;

  float corrected_gz_dps = gz_dps - gz_offset;
  yaw += corrected_gz_dps * dt;
  yaw = wrap360(yaw);

  myData.yaw = yaw;
  myData.pitch = pitch;
  myData.roll = roll;

  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(70);  
}