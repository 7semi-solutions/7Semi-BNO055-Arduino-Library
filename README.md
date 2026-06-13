# 7Semi BNO055 Arduino Library

Arduino library for the Bosch BNO055 9-axis Absolute Orientation Sensor.

The library provides access to accelerometer, gyroscope, magnetometer, Euler angles, quaternions, gravity vector, linear acceleration, calibration data, power management and device configuration.

---

## Features

* Accelerometer
* Gyroscope
* Magnetometer
* Euler Angles
* Quaternion Output
* Linear Acceleration
* Gravity Vector
* Temperature Sensor
* Calibration Management
* Device Information
* Power Mode Control
* Interrupt Management
* Sensor Configuration
* I2C Communication
* External Crystal Support

---

## Supported Platforms

* AVR
* ESP8266
* ESP32
* Arduino Compatible Boards

---

## Dependency

This library requires:

* 7Semi_BusCore

Install the dependency before using this library.

---

## Installation

### Arduino Library Manager

1. Open Arduino IDE.
2. Open Library Manager.
3. Search for **7Semi BNO055**.
4. Click Install.

### Manual Installation

1. Download this repository.
2. Install the library using:

   * Sketch
   * Include Library
   * Add ZIP Library
3. Install **7Semi_BusCore**.
4. Restart Arduino IDE.

---

## Hardware Connections

### I2C Interface

| BNO055 | MCU        |
| ------ | ---------- |
| VIN    | 3.3V / 5V  |
| GND    | GND        |
| SDA    | SDA        |
| SCL    | SCL        |
| ADR    | GND (0x28) |
| ADR    | VCC (0x29) |

---

## Quick Start

### Initialize Sensor

```cpp
#include <7Semi_BNO055.h>

BNO055_7Semi imu;

void setup()
{
    Serial.begin(115200);

    if (!imu.begin())
    {
        Serial.println("BNO055 not detected");

        while (1);
    }
}
```

---

## Set Operation Mode

```cpp
imu.setMode(
    BNO055_OP_Mode::NDOF);
```

Available modes:

```cpp
CONFIG
ACCONLY
MAGONLY
GYRONLY
ACCMAG
ACCGYRO
MAGGYRO
AMG
IMUPLUS
COMPASS
M4G
NDOF_FMC_OFF
NDOF
```

---

## Read Euler Angles

```cpp
float heading;
float roll;
float pitch;

imu.readEuler(
    heading,
    roll,
    pitch);
```

---

## Read Quaternion

```cpp
float w;
float x;
float y;
float z;

imu.readQuaternion(
    w,
    x,
    y,
    z);
```

---

## Read Accelerometer

```cpp
int16_t x;
int16_t y;
int16_t z;

imu.readAccel(
    x,
    y,
    z);
```

---

## Read Gyroscope

```cpp
int16_t x;
int16_t y;
int16_t z;

imu.readGyro(
    x,
    y,
    z);
```

---

## Read Magnetometer

```cpp
int16_t x;
int16_t y;
int16_t z;

imu.readMag(
    x,
    y,
    z);
```

---

## Read Linear Acceleration

```cpp
int16_t x;
int16_t y;
int16_t z;

imu.readLinear(
    x,
    y,
    z);
```

---

## Read Gravity Vector

```cpp
int16_t x;
int16_t y;
int16_t z;

imu.readGravity(
    x,
    y,
    z);
```

---

## Read Temperature

```cpp
int8_t temperature;

imu.readTemperature(
    temperature);
```

---

## Calibration

### Read Calibration Status

```cpp
uint8_t system;
uint8_t gyro;
uint8_t accel;
uint8_t mag;

imu.getCalibration(
    system,
    gyro,
    accel,
    mag);
```

### Read Calibration Data

```cpp
BNO055_CalibrationData data;

imu.readCalibrationData(
    data);
```

### Write Calibration Data

```cpp
BNO055_CalibrationData data;

imu.writeCalibrationData(
    data);
```

---

## Device Information

```cpp
DeviceInfo info;

imu.getDeviceInfo(
    info);
```

Available fields:

```cpp
info.chipID
info.accelID
info.magID
info.gyroID
info.swRevision
info.blRevision
```

---

## Power Modes

### Set Power Mode

```cpp
imu.setPowerMode(
    BNO055_Power_Mode::NORMAL);
```

### Get Power Mode

```cpp
BNO055_Power_Mode mode;

imu.getPowerMode(
    mode);
```

---

## Interrupt Management

### Enable Interrupt

```cpp
imu.enableInterrupt(
    BNO055_Interrupt::AccelAnyMotion);
```

### Disable Interrupt

```cpp
imu.disableInterrupt(
    BNO055_Interrupt::AccelAnyMotion);
```

### Read Interrupt State

```cpp
bool enabled;

imu.isInterruptEnabled(
    BNO055_Interrupt::AccelAnyMotion,
    enabled);
```

---

## Examples

The library includes the following examples:

* Basic
* Accelerometer
* Gyroscope
* Magnetometer
* LinearAcceleration
* Gravity
* Euler
* Quaternion

---

## License

MIT License

Copyright (c) 7Semi

---

## Links

Website

<https://7semi.com>

GitHub

<https://github.com/7semi-solutions/7Semi_BNO055_Arduino-Library>
