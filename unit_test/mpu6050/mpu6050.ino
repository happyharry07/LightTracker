#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/* ========= Pins ========= */
const uint8_t PIN_I2C_SDA = A4;
const uint8_t PIN_I2C_SCL = A5;
const uint8_t PIN_STATUS_LED = LED_BUILTIN;

/* ========= MPU6050 Object ========= */
Adafruit_MPU6050 mpu;

/* ========= Variables ========= */
float gz_offset = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;  // Yaw angle (will drift without a magnetometer)

const float ALPHA = 0.98f;  // Complementary filter coefficient
unsigned long prevMicros = 0;
bool blinkState = false;

static inline float rad2deg(float r) {
  return r * 180.0f / PI;
}

static inline float wrap360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_STATUS_LED, OUTPUT);
  Wire.begin();  // For ESP32, use Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Please check wiring and power.");
    while (1) {
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      delay(150);
    }
  }

  // Set measurement ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  prevMicros = micros();
  Serial.println("MPU6050 initialized successfully.");
  Serial.println("Output: Yaw(0~360), Pitch(-90~90), Roll");

  Serial.println("Calibrating gyroscope... Keep MPU6050 still!");
  delay(500); // 讓使用者手放開、穩定
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
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;

  // Sanity check for dt to prevent abnormalities on overflow or stuttering
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  // Accelerometer data (m/s^2) -> Used for tilt calculation
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Gyroscope data (rad/s) -> Convert to deg/s
  float gx_dps = g.gyro.x * 180.0f / PI;
  float gy_dps = g.gyro.y * 180.0f / PI;
  float gz_dps = g.gyro.z * 180.0f / PI;

  // Estimate Roll & Pitch from Accelerometer
  float roll_acc = rad2deg(atan2(ay, az));
  float pitch_acc = rad2deg(atan2(-ax, sqrt(ay * ay + az * az)));

  // Complementary Filter Fusion
  roll = ALPHA * (roll + gx_dps * dt) + (1.0f - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + gy_dps * dt) + (1.0f - ALPHA) * pitch_acc;

  // Yaw (Heading): Without a magnetometer, pure integration will drift over time
  // yaw += gz_dps * dt;
  // 扣除 Offset 之後再進行積分
  float corrected_gz_dps = gz_dps - gz_offset;
  yaw += corrected_gz_dps * dt;
  yaw = wrap360(yaw);

  // Print results
  Serial.print("Yaw: ");
  Serial.print(yaw, 2);
  Serial.print(", Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(", Roll: ");
  Serial.println(roll, 2);

  blinkState = !blinkState;
  digitalWrite(PIN_STATUS_LED, blinkState);

  delay(20);  // Approx. 50Hz loop rate
}
