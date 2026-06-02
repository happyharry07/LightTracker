#include <HardwareSerial.h>
#include <Wire.h>

// ==================== UWB 模組配置 ====================
HardwareSerial UwbSerial(2);
const unsigned long UWB_INTERVAL = 50; // 每 50ms 讀取一次 UWB 距離
unsigned long lastUwbTime = 0;
float currentDistance = -1.0;          // 全域變數：儲存當前最新距離

// ==================== 距離控制參數 ====================
const float TARGET_DISTANCE = 1.50;    // 目標保持距離：1.5 公尺
const float KP = 200.0;                 // 比例增益 (P Gain)，可根據車速與重量上下調整
const int MAX_MOTOR_SPEED = 150;        // 限制馬達最大 PWM 輸出 (0-255)，避免暴衝
const int MIN_MOTOR_SPEED = 50;         // 克服馬達靜摩擦力的最小啟動 PWM
const float DEAD_ZONE = 0.10;          // 死區範圍 (3cm)，在此誤差內車子靜止，防止微幅抖動

// ==================== 馬達引腳配置 ====================
const int PWMA = 13;
const int AIN2 = 14;
const int AIN1 = 27;
const int BIN1 = 26;
const int BIN2 = 25;
const int PWMB = 33;

// 前置宣告
void MotorWriting(double vL, double vR);
bool checkUwbModule();
float getDistanceFast();
void controlCarDistance();

void setup() {
  // 初始化電腦序列埠
  Serial.begin(115200);
  
  // 初始化 UWB 序列埠 (115200 Baud)
  UwbSerial.begin(115200, SERIAL_8N1, 16, 17);
  
  // 初始化馬達引腳
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0); // 初始靜止

  Serial.println("\n================================================");
  Serial.println("UWB Distance Tracking Car Initialized");
  Serial.println("================================================");
  
  delay(500);
  if (checkUwbModule()) {
    Serial.println("[SUCCESS] BU03-Kit Connected.");
  } else {
    Serial.println("[WARNING] BU03-Kit No Response.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. 定時排程：每 50ms 向 UWB 請求一次最新距離
  if (currentMillis - lastUwbTime >= UWB_INTERVAL) {
    lastUwbTime = currentMillis;
    float distRead = getDistanceFast();
    
    // 若讀取成功（非 -1），更新全域距離變數
    if (distRead >= 0) {
      currentDistance = distRead;
      
      // 除了解析，同時印出目前狀態供偵錯
      Serial.print("Dist: "); Serial.print(currentDistance, 3); Serial.print(" m | ");
    }
  }

  // 2. 持續執行閉迴路車距控制
  controlCarDistance();
}

// ==================== 距離控制邏輯（P 控制器） ====================
void controlCarDistance() {
  // 如果 UWB 還沒成功讀到任何一次距離，或是斷訊，先讓車子靜止以保安全
  if (currentDistance < 0) {
    MotorWriting(0, 0);
    return;
  }

  // 計算誤差 (Error = 當前距離 - 目標距離)
  // 若距離 > 1.5m，error 為正值 -> 車子需要「前進」來縮短距離
  // 若距離 < 1.5m，error 為負值 -> 車子需要「後退」來拉開距離
  float error = currentDistance - TARGET_DISTANCE;

  // 判斷死區：如果誤差在 ±3 公分以內，視為已到達目標，直接停下防止齒輪來回修正
  if (abs(error) < DEAD_ZONE) {
    MotorWriting(0, 0);
    Serial.println("Action: STOP (Target Reached)");
    return;
  }

  // 計算控制輸出值 (Speed = Error * KP)
  float speedOutput = error * KP;

  // 限制最大速度，避免誤差太大時馬達全速暴衝
  if (speedOutput > MAX_MOTOR_SPEED)  speedOutput = MAX_MOTOR_SPEED;
  if (speedOutput < -MAX_MOTOR_SPEED) speedOutput = -MAX_MOTOR_SPEED;

  // 克服基本低速死區（當速度過低帶不動車身時，強制給予最低啟動電壓）
  if (speedOutput > 0 && speedOutput < MIN_MOTOR_SPEED) speedOutput = MIN_MOTOR_SPEED;
  if (speedOutput < -0 && speedOutput > -MIN_MOTOR_SPEED) speedOutput = -MIN_MOTOR_SPEED;

  // 控制雙側馬達前進或後退（直線行駛，故左右輪給相同速度）
  MotorWriting(speedOutput, speedOutput);

  // 印出當前馬達控制輸出
  Serial.print("Error: "); Serial.print(error, 3);
  Serial.print(" | Motor Out: "); Serial.println(speedOutput);
}

// ==================== 馬達驅動函式 ====================
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
  analogWrite(PWMA, (int)vL);
  analogWrite(PWMB, (int)vR);
}

// ==================== UWB 驅動底層 ====================
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