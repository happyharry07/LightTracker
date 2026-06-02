#include <HardwareSerial.h>

HardwareSerial UwbSerial(2);

// 定時排程間隔：50ms (每秒更新 20 次)
const unsigned long MEASURE_INTERVAL = 50;
unsigned long lastMeasureTime = 0;

// 全域切換狀態機：用於交替發送指令，平衡序列埠頻寬
bool toggleRequest = true; 

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // 與 BU03-Kit 的高速序列埠通訊
  UwbSerial.begin(115200, SERIAL_8N1, 16, 17); 
  
  Serial.println("\n================================================");
  Serial.println("ESP32 to BU03-Kit Dual-Sensor High-Speed Mode");
  Serial.println("================================================");
  
  delay(500);
  if (checkUwbModule()) {
    Serial.println("[SUCCESS] BU03-Kit AT framework is working properly.");
  } else {
    Serial.println("[WARNING] BU03-Kit no response.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMeasureTime >= MEASURE_INTERVAL) {
    lastMeasureTime = currentMillis;
    
    if (toggleRequest) {
      // 這一輪：讀取並輸出距離
      float distance = getDistanceFast();
      if (distance >= 0) {
        // Serial.print("Distance: ");
        // Serial.print(distance, 4); 
        // Serial.println(" m");
      }
    } else {
      // 下一輪：讀取並輸出角度
      float angle = getAngleFast();
      if (angle != -999.0) { // 使用 -999.0 作為錯誤防禦邊界值
        Serial.print("Angle   : ");
        Serial.print(angle, 4); 
        Serial.println(" deg");
      }
    }
    
    // 狀態翻轉
    toggleRequest = !toggleRequest;
  }
}

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

// 測距函式 (維持不變)
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

// 新增：針對多行 Sensor 資料優化的高速角度解析函式
float getAngleFast() {
  // 1. 快速清空舊快取
  while(UwbSerial.available()) { UwbSerial.read(); }
  
  // 2. 發送生產測試感測器指令
  UwbSerial.println("AT+GETSENSOR"); 
  
  // 3. 壓縮超時等待：放寬至 30ms 以完整容納 5 行字串的傳輸延遲
  unsigned long startTimeout = millis();
  String rawResponse = "";
  rawResponse.reserve(128); // 擴大緩衝區空間
  
  while ((millis() - startTimeout) < 30) { 
    while (UwbSerial.available()) {
      char c = UwbSerial.read();
      rawResponse += c;
    }
    if (rawResponse.indexOf("OK") != -1) { 
      break;
    }
  }

  // 4. 精准定位 "angle:" 關鍵字進行切片
  int keywordIndex = rawResponse.indexOf("angle:");
  if (keywordIndex != -1) {
    int startIndex = keywordIndex + 6; // 跳過 "angle:" 這 6 個字元
    int endIndex = rawResponse.indexOf("\r", startIndex);
    if (endIndex == -1) endIndex = rawResponse.indexOf("\n", startIndex);
    
    if (endIndex != -1) {
      String angleStr = rawResponse.substring(startIndex, endIndex);
      angleStr.trim();          
      return angleStr.toFloat(); // 成功則回傳浮點數角度
    }
  }
  
  return -999.0; // 解析失敗或超時回傳防禦值
}