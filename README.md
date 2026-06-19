# LightTracker

透過計算相對空間座標與雷射指向，設計一台能精準追隨雷射光點的智慧小車。
本專案全面採用 ESP32 雙核心架構，結合 MPU6050 姿態解算、BU03 (UWB) 公分級物理測距，以及 ESP-NOW 低延遲無線通訊技術，達成無光線限制的即時動態追隨。

---

## 📂 Project Structure

```text
Light_Tracker/
├── 01_Utilities/                  # 系統工具與前置作業
│   └── esp32_get_receiver_address/# 獲取 ESP32 MAC 地址的輔助程式，用於設定 ESP-NOW 綁定
│
├── 02_Unit_Tests/                 # 第一階段：單一硬體模組測試
│   ├── BU03_distance/             # UWB 模組 AT 指令測距基礎驗證
│   ├── esp32_espnow/              # ESP-NOW 雙向通訊與封包收發測試
│   ├── motor_motorcheck/          # TB6612 模組與 TT 馬達 PWM/方向控制驅動測試
│   └── mpu6050_angle/             # MPU6050 陀螺儀/加速度計讀取及基礎互補濾波測試
│
├── 03_Integration_Tests/          # 第二階段：次系統整合與演算法驗證
│   ├── esp32_angle_tracking/      # 【角度跟隨】小車端跟著手持端的 Yaw 角度原地旋轉 (v1)
│   ├── esp32_angle_tracking_v2/   #   ➔ 迭代：優化 MPU6050 零點偏差 (Zero-drift)
│   ├── esp32_angle_tracking_v3/   #   ➔ 迭代：修正初始角度錯誤，解決開機時會原地迴轉 180 度的 Bug
│   ├── esp32_angle_tracking_v4/   #   ➔ 迭代：增加回傳小車資料到接收端 (手持端) 的功能，以協助後續 Debug
│   ├── esp32_distance_tracking/   # 【距離跟隨】小車直線移動，與手持端保持固定距離的 P 控制測試
│   └── BU03_distance_angle/       # 【指向跟隨】小車根據手持端 Pitch 的角度，前後移動至手持端指向的目標點
│
├── 04_Final/                      # 第三階段：最終專案主程式
│   ├── final_send/                # 手持端主程式：負責姿態解算、雷射指向計算與指令發送
│   └── final_receive/             # 小車端主程式：負責接收指令、測距、空間座標推算與 P/PD 雙環控制
│
└── BU03_info/                     # UWB 測距模組參考資料庫
    ├── BU03_BU04_AT_command_en_v1.0.6.pdf # UWB AT 指令英文手冊
    ├── BU03_specification.pdf             # BU03 模組官方硬體規格書
    ├── BU03-Kit_specification.pdf         # BU03 開發板 (Kit) 腳位與使用說明
    └── BU03_data_calibration.xlsx         # UWB 測距數據校準與誤差分析表