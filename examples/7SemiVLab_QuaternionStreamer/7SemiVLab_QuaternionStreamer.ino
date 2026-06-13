/***************************************************************
 * @file    QuaternionStreamer.ino
 * @brief   Streams quaternion orientation and calibration
 *          status from the BNO055 sensor.
 *
 * Features:
 * - Initialize BNO055
 * - Enable NDOF fusion mode
 * - Read quaternion orientation
 * - Read calibration status
 *
 * Connections:
 * - VIN -> 3.3V / 5V
 * - GND -> GND
 * - SDA -> SDA
 * - SCL -> SCL
 *
 * Library     : 7Semi_BNO055
 * Author      : 7Semi
 * Version     : 1.0
 ***************************************************************/

#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    if (!imu.begin())
    {
        Serial.println(F("BNO055 not detected"));

        while (1)
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(150);
        }
    }

    imu.setOpMode(BNO055_OP_Mode::NDOF);

    Serial.println(F("# 7Semi Quaternion Streamer"));
    Serial.println(F("# Format: Q,w,x,y,z,CAL,sys,g,a,m"));
}

void loop()
{
    BNO055_Calibration calib;

    imu.getCalibration(calib);

    float w;
    float x;
    float y;
    float z;

    if (imu.readQuaternion(w, x, y, z))
    {
        Serial.print(F("Q,"));

        Serial.print(w, 6);
        Serial.print(',');

        Serial.print(x, 6);
        Serial.print(',');

        Serial.print(y, 6);
        Serial.print(',');

        Serial.print(z, 6);

        Serial.print(F(",CAL,"));

        Serial.print(calib.system);
        Serial.print(',');

        Serial.print(calib.gyro);
        Serial.print(',');

        Serial.print(calib.accel);
        Serial.print(',');

        Serial.println(calib.mag);
    }

    static bool ledState = false;
    static uint32_t heartbeat = 0;

    if ((++heartbeat % 50) == 0)
    {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }

    delay(20);
}