/***************************************************************
  * @file    Basic.ino
  * @brief   Basic BNO055 example demonstrating orientation,
  * 
        sensor data, quaternion and calibration status.
    
  *
  * Features:
  * * Initialize BNO055
  * * Enable NDOF fusion mode
  * * Read Euler angles
  * * Read accelerometer data
  * * Read gyroscope data
  * * Read magnetometer data
  * * Read linear acceleration
  * * Read gravity vector
  * * Read quaternion data
  * * Read temperature
  * * Display calibration status
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
  * Print current calibration status.
  *
  * * Displays system calibration level.
  * * Displays gyro calibration level.
  * * Displays accelerometer calibration level.
  * * Displays magnetometer calibration level.
  */
void printCalibration() {
  BNO055_Calibration calib;

    imu.getCalibration(calib);

  Serial.print(F("Calibration  SYS:"));
  Serial.print(calib.system);

  Serial.print(F(" G:"));
  Serial.print(calib.gyro);

  Serial.print(F(" A:"));
  Serial.print(calib.accel);

  Serial.print(F(" M:"));
  Serial.println(calib.mag);
}

void setup() {
  Serial.begin(115200);


  Serial.println(F("\n7Semi BNO055 Basic Example"));

  // Initialize sensor
  if (!imu.begin()) {
    Serial.println(F("BNO055 not detected"));

    while (1)
      ;
  }

  // Enable fusion mode
  imu.setOpMode(BNO055_OP_Mode::NDOF);

  Serial.println(F("Sensor Ready"));

  printCalibration();

  Serial.println();
}

void loop() {
  float heading;
  float roll;
  float pitch;


  if (imu.readEuler(heading, roll, pitch)) {
    Serial.print(F("Euler H/R/P : "));
    Serial.print(heading, 1);
    Serial.print(F(", "));
    Serial.print(roll, 1);
    Serial.print(F(", "));
    Serial.println(pitch, 1);
  }

  int16_t ax;
  int16_t ay;
  int16_t az;

  if (imu.readAccel(ax, ay, az)) {
    Serial.print(F("Accel : "));
    Serial.print(ax);
    Serial.print(F(", "));
    Serial.print(ay);
    Serial.print(F(", "));
    Serial.println(az);
  }

  int16_t gx;
  int16_t gy;
  int16_t gz;

  if (imu.readGyro(gx, gy, gz)) {
    Serial.print(F("Gyro  : "));
    Serial.print(gx);
    Serial.print(F(", "));
    Serial.print(gy);
    Serial.print(F(", "));
    Serial.println(gz);
  }

  int16_t mx;
  int16_t my;
  int16_t mz;

  if (imu.readMag(mx, my, mz)) {
    Serial.print(F("Mag   : "));
    Serial.print(mx);
    Serial.print(F(", "));
    Serial.print(my);
    Serial.print(F(", "));
    Serial.println(mz);
  }

  int16_t lx;
  int16_t ly;
  int16_t lz;

  if (imu.readLinear(lx, ly, lz)) {
    Serial.print(F("Linear: "));
    Serial.print(lx);
    Serial.print(F(", "));
    Serial.print(ly);
    Serial.print(F(", "));
    Serial.println(lz);
  }

  int16_t gvx;
  int16_t gvy;
  int16_t gvz;

  if (imu.readGravity(gvx, gvy, gvz)) {
    Serial.print(F("Grav  : "));
    Serial.print(gvx);
    Serial.print(F(", "));
    Serial.print(gvy);
    Serial.print(F(", "));
    Serial.println(gvz);
  }

  float qw;
  float qx;
  float qy;
  float qz;

  if (imu.readQuaternion(qw, qx, qy, qz)) {
    Serial.print(F("Quat  : "));
    Serial.print(qw, 4);
    Serial.print(F(", "));
    Serial.print(qx, 4);
    Serial.print(F(", "));
    Serial.print(qy, 4);
    Serial.print(F(", "));
    Serial.println(qz, 4);
  }

  int8_t temperature;

  if (imu.readTemperature(temperature)) {
    Serial.print(F("Temp  : "));
    Serial.print(temperature);
    Serial.println(F(" C"));
  }

  static uint32_t lastCalibration = 0;

  if (millis() - lastCalibration >= 2000) {
    lastCalibration = millis();

    printCalibration();
  }

  Serial.println();

  delay(500);
}
