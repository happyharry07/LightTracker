#include <HardwareSerial.h>

HardwareSerial UwbSerial(2);

// 將量測間隔縮短至 50ms (每秒更新 20 次)，可依需求調小，如 20ms 或 30ms
const unsigned long MEASURE_INTERVAL = 50; 
unsigned long lastMeasureTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  
  // 維持 115200 高速通訊
  UwbSerial.begin(115200, SERIAL_8N1, 16, 17); 
  
  Serial.println("\n================================================");
  Serial.println("ESP32 to BU03-Kit High-Speed Mode");
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
    
    float distance = getDistanceFast();
    
    if (distance >= 0) {
      Serial.print("Distance: ");
      Serial.print(distance, 4); 
      Serial.println(" m");
    }
    // 高速模式下，略過錯誤訊息的列印，避免序列埠頻繁輸出文字造成阻塞
  }
}

bool checkUwbModule() {
  while(UwbSerial.available()) UwbSerial.read(); 
  UwbSerial.println("AT"); 
  
  delay(50); // 縮短測試等待時間
  String resp = "";
  while(UwbSerial.available()) {
    resp += (char)UwbSerial.read();
  }
  return (resp.indexOf("OK") != -1);
}

// 針對高速讀取優化的測距函式
float getDistanceFast() {
  // 1. 快速清空舊快取
  while(UwbSerial.available()) { UwbSerial.read(); }
  
  // 2. 發送測距指令
  UwbSerial.println("AT+DISTANCE"); 
  
  // 3. 壓縮超時等待：115200 鲍率下，模組回傳約 30 個字元僅需 ~3ms
  // 將原本 200ms 的等待極限縮短至 15ms，時間一到或收到 OK 立即放行
  unsigned long startTimeout = millis();
  String rawResponse = "";
  rawResponse.reserve(64); // 預先配置記憶體，加速字串相加效率
  
  while ((millis() - startTimeout) < 15) { 
    while (UwbSerial.available()) {
      char c = UwbSerial.read();
      rawResponse += c;
    }
    if (rawResponse.indexOf("OK") != -1) { 
      break;
    }
  }

  // 4. 解析字串
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