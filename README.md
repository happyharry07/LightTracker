# LightTracker
Design a car that can follow the light spot of a laser by calculating their relative coordinate and estimate the laser direction.

Project_LightTracker/
├── 01_Unit_Tests/                   # First Stage: Hardware Validation Units
│   ├── test_mpu6050/                # IMU sensor zero-bias calibration test
│   ├── test_bu01/                   # UWB point-to-point ranging baseline test
│   └── test_espnow/                 # ESP-NOW transceiver and packet latency test
│
├── 02_Laser/                        # Laser Pen Transmitter Project (ESP32)
│   ├── Laser.ino                    # Main controller loop for transmitter
│   ├── angle.h                      # Acquires raw data from MPU6050 
│   ├── filter.h                     # Stabilizes data via Complementary Filter
│   └── bluetooth.h                  # Transmits data packets via ESP-NOW protocol
│
└── 03_Smart_Car/                    # Smart Car Integrated Project (Mega 2560)
    ├── Smart_Car.ino                # Main control loop and scheduling
    ├── bluetooth.h                  # Receives data packets from communication sub-board
    ├── position.h                   # Computes coordinate transformation and math logic
    └── control.h                    # Implements PID closed-loop tracking and motor drive