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
