# Solar Panel Cleaning Robot (ESP32)

ESP32-based autonomous solar panel cleaning robot with obstacle detection, fall detection, LoRa telemetry, and WiFi web control. The robot is designed to clean solar panels using a rotating brush while safely navigating panel surfaces.

---

## Features

- Dual VL53L5CX Time-of-Flight sensors for obstacle detection
- ICM-20948 9-axis IMU for orientation and fall detection
- LoRa wireless telemetry for remote monitoring
- ESP32 WiFi access point with web-based control interface
- Motorized drive system for panel traversal
- Brush motor for cleaning
- Linear actuator for adjustable brush positioning
- Kalman filter based sensor fusion for stable orientation estimation

---

## Hardware Components

- ESP32 microcontroller
- 2 × VL53L5CX ToF distance sensors
- ICM-20948 9-axis IMU
- LoRa module (433 MHz)
- DC motors for robot movement
- Brush motor for cleaning
- Linear actuator
- Motor driver modules
- Battery power system

---

## System Architecture

The robot integrates multiple subsystems:

- **Obstacle Detection:** Dual ToF sensors measure distances ahead of the robot.
- **Fall Detection:** IMU pitch, roll, and yaw monitoring detects abnormal orientation.
- **Wireless Communication:** LoRa transmits telemetry data including sensor values and robot status.
- **Web Control:** ESP32 runs a WiFi access point allowing manual control through a web interface.
- **Motor Control:** PWM-based control for drive motors, brush motor, and actuator.

---

## Software Structure
solar-cleaning-robot-esp32
│
├── code/
│ ├── main_robot_code.ino
│ └── new_drive_bot_code.ino
│
├── web_interface/
│ └── control.html
│
├── hardware/
│ └── circuit_diagrams
│
└── README.md

---

## Communication

### WiFi Control
The ESP32 creates a WiFi Access Point.

SSID:
ESP_BOT

Commands sent via HTTP:
fXXX → move forward
rXXX → move reverse
b → brake
Where `XXX` is PWM speed (0–255).

---

### LoRa Telemetry

Robot transmits:

- ToF sensor distances
- Pitch, roll, yaw
- Accelerometer data
- Gyroscope data
- Magnetometer data
- Robot status flags

Transmission interval: **2 seconds**

---

## Safety Features

- Obstacle detection stops drive motors automatically
- IMU fall detection triggers emergency shutdown
- Robot enters safe state after fall detection
- Manual reset required after emergency

---

## Future Improvements

- Autonomous navigation across full panel arrays
- Dust detection using optical sensors
- Solar-powered charging dock
- Remote monitoring dashboard
- Computer vision based edge detection

---

## Author

Engineering project focused on embedded robotics, sensor fusion, and wireless telemetry using ESP32.
