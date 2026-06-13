/***************************************************************           
  * @file    LinearAcceleration.ino
  * @brief   Read linear acceleration data from the BNO055 sensor.
  *
  * Features:
  * * Initialize BNO055
  * * Configure accelerometer settings
  * * Read raw linear acceleration values
  * * Read converted linear acceleration values
  * * Display active accelerometer configuration
  *
  * Linear Acceleration:
  * * Gravity component removed
  * * Useful for motion detection
  * * Useful for velocity estimation
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


  Serial.println(F("\n7Semi BNO055 Linear Acceleration Example"));

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

  // Enable fusion mode
  imu.setOpMode(BNO055_OP_Mode::NDOF);

  printAccelConfig();

  Serial.println(F("\nReading linear acceleration data..."));
}

void loop() {
  BNO055_Sensor_Data linear;

  if (imu.readLinear(linear)) {
    Serial.print(F("m/s²: "));
    Serial.print(linear.x, 3);
    Serial.print(F(", "));
    Serial.print(linear.y, 3);
    Serial.print(F(", "));
    Serial.println(linear.z, 3);
  } else {
    Serial.println(F("Failed to read linear acceleration"));
  }

  delay(200);
}
