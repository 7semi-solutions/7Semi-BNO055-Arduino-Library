/***************************************************************

  * @file    Gyroscope.ino
  * @brief   Read gyroscope data from the BNO055 sensor.
  *
  * Features:
  * * Initialize BNO055
  * * Configure gyroscope settings
  * * Read converted gyroscope values
  * * Display active gyroscope configuration
  *
  * Gyroscope Configuration:
  * * Range      : ±500 dps
  * * Bandwidth  : 47 Hz
  * * Power Mode : Normal
  * * Unit       : Degrees Per Second
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
  * Print current gyroscope configuration.
  *
  * * Reads configuration from the sensor.
  * * Displays range, bandwidth and power mode.
  */
void printGyroConfig() {
  GyroConfig config;

  if (!imu.getGyroConfig(config)) {
    Serial.println(F("Failed to read gyroscope configuration"));
    return;
  }

  Serial.println(F("\nGyroscope Configuration"));

  Serial.print(F("Range      : "));
  Serial.println((uint8_t)config.range);

  Serial.print(F("Bandwidth  : "));
  Serial.println((uint8_t)config.bandwidth);

  Serial.print(F("Power Mode : "));
  Serial.println((uint8_t)config.powerMode);
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("\n7Semi BNO055 Gyroscope Example"));

  // Initialize sensor
  if (!imu.begin()) {
    Serial.println(F("BNO055 not detected"));

    while (1)
      ;
  }

  // Configure gyroscope
  GyroConfig config;

  config.range = GyroRange::DPS500;
  config.bandwidth = GyroBandwidth::HZ47;
  config.powerMode = GyroPowerMode::Normal;
  config.unit = BNO055_Gyro_Unit::DegreesPerSecond;

  if (!imu.setGyroConfig(config)) {
    Serial.println(F("Failed to configure gyroscope"));
  }

  printGyroConfig();

  Serial.println(F("\nReading gyroscope data..."));
}

void loop() {
  BNO055_Sensor_Data gyro;

  if (imu.readGyro(gyro)) {
    Serial.print(F("DPS : "));
    Serial.print(gyro.x, 3);
    Serial.print(F(", "));
    Serial.print(gyro.y, 3);
    Serial.print(F(", "));
    Serial.println(gyro.z, 3);
  } else {
    Serial.println(F("Failed to read gyroscope"));
  }

  delay(500);
}
