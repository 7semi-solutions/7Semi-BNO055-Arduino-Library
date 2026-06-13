/***************************************************************

* @file    Accelerometer.ino
* @brief   Read accelerometer data from the BNO055 sensor.
*
* Features:
* * Initialize BNO055
* * Configure accelerometer settings
* * Read raw accelerometer values
* * Read converted accelerometer values
* * Display active accelerometer configuration
*
* Accelerometer Configuration:
* * Range      : ±4g
* * Bandwidth  : 125 Hz
* * Power Mode : Normal
* * Unit       : m/s²
*
* Connections:
* * VIN -> 3.3V / 5V
* * GND -> GND
* * SDA -> SDA
* * SCL -> SCL
*
* Library     : 7Semi_BNO055
* Author      : 7Semi
* Version     : 1.0
  ***************************************************************/

#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu;

/**
* Print current accelerometer configuration.
*
* * Reads configuration from the sensor.
* * Displays range, bandwidth and power mode.
*/
void printAccelConfig() {
  AccelConfig config;

  if (!imu.getAccelConfig(config)) {
    Serial.println(F("Failed to read accelerometer configuration"));
    return;
  }

  Serial.println(F("\nAccelerometer Configuration"));

  Serial.print(F("Range      : "));
  Serial.println((uint8_t)config.range);

  Serial.print(F("Bandwidth  : "));
  Serial.println((uint8_t)config.bandwidth);

  Serial.print(F("Power Mode : "));
  Serial.println((uint8_t)config.powerMode);
}

void setup() {
  Serial.begin(115200);


  Serial.println(F("\n7Semi BNO055 Accelerometer Example"));

  // Initialize sensor
  if (!imu.begin()) {
    Serial.println(F("BNO055 not detected"));

    while (1)
      ;
  }

  // Configure accelerometer
  AccelConfig config;

  config.range = AccelRange::G4;
  config.bandwidth = AccelBandwidth::HZ125;
  config.powerMode = AccelPowerMode::Normal;
  config.unit = BNO055_Accel_Unit::MetersPerSecondSquared;

  if (!imu.setAccelConfig(config)) {
    Serial.println(F("Failed to configure accelerometer"));
  }

  printAccelConfig();

  Serial.println(F("\nReading accelerometer data..."));
}

void loop() {
  BNO055_Sensor_Data accel;


  if (imu.readAccel(accel)) {

    Serial.print(F("m/s²: "));
    Serial.print(accel.x, 3);
    Serial.print(F(", "));
    Serial.print(accel.y, 3);
    Serial.print(F(", "));
    Serial.println(accel.z, 3);
  } else {
    Serial.println(F("Failed to read accelerometer"));
  }

  delay(200);
}
