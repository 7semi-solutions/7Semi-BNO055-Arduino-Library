# 7Semi_BNO055

A lightweight and minimal Arduino library for the Bosch BNO055 9-DOF IMU sensor.

Supports initialization, raw data reading (Accel / Gyro / Mag / Linear / Gravity), Euler angles, quaternions, per-sensor configuration, offsets, and calibration helpers.  
Designed to be small in flash usage — suitable for AVR, ESP32/ESP8266, RP2040, and STM32.

![Platform](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## ✨ Features

- Detects and initializes the BNO055 automatically at I2C address `0x28` or `0x29`
- Supports custom I2C pins on supported platforms (ESP, STM32, RP2040, etc.)
- Raw data reading:
  - Accelerometer
  - Gyroscope
  - Magnetometer
  - Linear Acceleration
  - Gravity Vector
- Orientation data:
  - Euler angles (heading, roll, pitch)
  - Quaternions (w, x, y, z)
- Configuration options:
  - Sensor range, power mode, bandwidth settings
  - Axis remapping
- Calibration helpers:
  - Real-time calibration status
  - Saving and restoring calibration offsets

---

## ⚡ Getting Started

### 🧰 Hardware Requirements

- 7Semi BNO055 Sensor Module  
- Arduino board (UNO, Nano, ESP32, RP2040, STM32, etc.)  
- I2C connection (or UART if supported in the library)

---

### 🔌 Wiring (I2C Example)

| BNO055 Pin | Arduino Pin     |
|------------|------------------|
| VIN        | 3.3V or 5V       |
| GND        | GND              |
| SDA        | SDA (A4 on Uno)  |
| SCL        | SCL (A5 on Uno)  |
| ADR        | GND (I2C 0x28) or VCC (I2C 0x29) |

> **Note:** For ESP32, STM32, or RP2040, pass custom `SDA` and `SCL` pins using `begin(sda, scl)`.

---

### 🚀 Installation

#### ✅ Recommended: Arduino Library Manager

1. Open Arduino IDE (1.8.x or 2.x)  
2. Go to **Sketch > Include Library > Manage Libraries…**  
3. Search for:
