/***************************************************************
  * @file    Euler.ino
  * @brief   Read orientation data from the BNO055 sensor as
  *          Euler angles.    
  *
  * Features:
  * * Initialize BNO055
  * * Enable NDOF fusion mode
  * * Read heading, roll and pitch angles
  * * Display orientation in degrees
  *
  * Sensor Configuration:
  * * Mode : NDOF
  * * Unit : Degrees
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


void setup() {
  Serial.begin(115200);


  Serial.println(F("\n7Semi BNO055 Euler Example"));

  // Initialize sensor
  if (!imu.begin()) {
    Serial.println(F("BNO055 not detected"));

    while (1)
      ;
  }

  // Enable fusion mode
  imu.setOpMode(BNO055_OP_Mode::NDOF);

  Serial.println(F("Reading Euler angles..."));
}

void loop() {
  float heading;
  float roll;
  float pitch;


  if (!imu.readEuler(heading, roll, pitch)) {
    Serial.println(F("Failed to read Euler angles"));
    delay(200);
    return;
  }

  Serial.print(F("Degrees H/R/P : "));
  Serial.print(heading, 2);
  Serial.print(F(", "));
  Serial.print(roll, 2);
  Serial.print(F(", "));
  Serial.println(pitch, 2);

  Serial.println();

  delay(200);
}
