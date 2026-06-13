/***************************************************************
  * @file    Quaternion.ino
  * @brief   Read orientation data from the BNO055 sensor as
  *          normalized quaternion values.    
  *
  * Features:
  * * Initialize BNO055
  * * Enable NDOF fusion mode
  * * Read quaternion orientation data
  * * Display W, X, Y and Z components
  *
  * Quaternion Data:
  * * Normalized orientation representation
  * * No gimbal lock
  * * Suitable for robotics and 3D tracking
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

void setup() {
  Serial.begin(115200);


  Serial.println(F("\n7Semi BNO055 Quaternion Example"));

  // Initialize sensor
  if (!imu.begin()) {
    Serial.println(F("BNO055 not detected"));

    while (1)
      ;
  }

  // Enable fusion mode
  imu.setOpMode(BNO055_OP_Mode::NDOF);

  Serial.println(F("Reading quaternion data..."));
}

void loop() {
  float w;
  float x;
  float y;
  float z;

  if (!imu.readQuaternion(w, x, y, z)) {
    Serial.println(F("Failed to read quaternion"));

    delay(200);
    return;
  }

  Serial.print(F("Quaternion W/X/Y/Z : "));
  Serial.print(w, 5);
  Serial.print(F(", "));
  Serial.print(x, 5);
  Serial.print(F(", "));
  Serial.print(y, 5);
  Serial.print(F(", "));
  Serial.println(z, 5);

  delay(200);
}
