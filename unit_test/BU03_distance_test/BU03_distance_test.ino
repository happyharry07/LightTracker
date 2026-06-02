#include <HardwareSerial.h>

// 使用 ESP32 的硬體序列埠 2 (Serial2)
// 接線確認：ESP32 Pin 16 (RX2) <-> BU03-Kit Pin 6 (TX1)
//           ESP32 Pin 17 (TX2) <-> BU03-Kit Pin 7 (RX1)
HardwareSerial UwbSerial(2);

// 參數設定
const unsigned long MEASURE_INTERVAL = 1000; // 每隔 1 秒讀取一次距離
unsigned long lastMeasureTime = 0;

void setup() {
  // 1. 初始化與電腦連接的序列埠（用來看 Serial Monitor 輸出）
  Serial.begin(115200);
  while (!Serial); // 等待序列埠準備就緒
  
  // 2. 初始化與 BU03-Kit 連接的序列埠
  // 設定波特率為 115200，使用預設引腳（RX=16, TX=17）
  UwbSerial.begin(115200, SERIAL_8N1, 16, 17); 
  
  Serial.println("\n================================================");
  Serial.println("ESP32 to BU03-Kit (115200 Baud) Controller");
  Serial.println("================================================");
  
  // 測試 UWB 模組是否正常回應
  delay(500); // 給模組一點啟動時間
  if (checkUwbModule()) {
    Serial.println("[SUCCESS] BU03-Kit AT framework is working properly.");
  } else {
    Serial.println("[WARNING] BU03-Kit no response. Please double check wiring.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 定時執行測距流程
  if (currentMillis - lastMeasureTime >= MEASURE_INTERVAL) {
    lastMeasureTime = currentMillis;
    
    float distance = getDistance();
    
    if (distance >= 0) {
      Serial.print("Distance: ");
      Serial.print(distance, 4); // 印出解析後的距離（保留小數點後 4 位）
      Serial.println(" m");
    } else {
      Serial.println("[ERROR] Failed to read distance from UWB module.");
    }
  }
}

// 發送基礎 AT 測試指令
bool checkUwbModule() {
  while(UwbSerial.available()) UwbSerial.read(); // 清空舊快取
  UwbSerial.println("AT"); // 發送 AT 指令
  
  delay(100); // 等待模組回應
  String resp = "";
  while(UwbSerial.available()) {
    resp += (char)UwbSerial.read();
  }
  return (resp.indexOf("OK") != -1); // 預期收到 OK
}

// 發送 AT+DISTANCE 並精準解析出距離數字
float getDistance() {
  // 1. 清空序列埠殘留資料，確保讀到的是最新一筆
  while(UwbSerial.available()) UwbSerial.read();
  
  // 2. 發送測距指令
  UwbSerial.println("AT+DISTANCE"); 
  
  // 3. 等待模組回傳（設定 200ms 超時限制）
  unsigned long startTimeout = millis();
  String rawResponse = "";
  
  while ((millis() - startTimeout) < 200) {
    while (UwbSerial.available()) {
      char c = UwbSerial.read();
      rawResponse += c;
    }
    // 當收到關鍵字 "OK" 代表這輪指令回傳結束
    if (rawResponse.indexOf("OK") != -1) { 
      break;
    }
  }

  // 4. 解析字串，尋找 "distance:" 關鍵字
  // 預期回傳格式： "distance: 0.340000\r\n\r\nOK\r\n"
  int keywordIndex = rawResponse.indexOf("distance:");
  if (keywordIndex != -1) {
    int startIndex = keywordIndex + 9; // 跳過 "distance:" 這 9 個字元
    int endIndex = rawResponse.indexOf("\r", startIndex); // 尋找數值後的換行符
    if (endIndex == -1) endIndex = rawResponse.indexOf("\n", startIndex);
    
    if (endIndex != -1) {
      String distStr = rawResponse.substring(startIndex, endIndex);
      distStr.trim();          // 去除前後空白
      return distStr.toFloat(); // 將純數字字串轉換為 float 浮點數
    }
  }
  
  return -1.0; // 解析失敗或超時回傳 -1
}