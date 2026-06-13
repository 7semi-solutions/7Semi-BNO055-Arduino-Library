#ifndef _7Semi_BNO055_H_
#define _7Semi_BNO055_H_

#pragma once
#include <Arduino.h>
#include "7Semi_Interface.h"
#include "7Semi_I2C.h"
#include "7Semi_SPI.h"
#include "7Semi_Bus.h"

#define BNO055_REG_PAGE_ID 0x07

#define BNO055_REG_CHIP_ID      0x00
#define BNO055_REG_ACCEL_REV_ID 0x01
#define BNO055_REG_MAG_REV_ID   0x02
#define BNO055_REG_GYRO_REV_ID  0x03
#define BNO055_REG_SW_REV_LSB   0x04
#define BNO055_REG_SW_REV_MSB   0x05
#define BNO055_REG_BL_REV_ID    0x06

#define BNO055_REG_ACCEL_X_LSB 0x08
#define BNO055_REG_ACCEL_X_MSB 0x09
#define BNO055_REG_ACCEL_Y_LSB 0x0A
#define BNO055_REG_ACCEL_Y_MSB 0x0B
#define BNO055_REG_ACCEL_Z_LSB 0x0C
#define BNO055_REG_ACCEL_Z_MSB 0x0D

#define BNO055_REG_MAG_X_LSB 0x0E
#define BNO055_REG_MAG_X_MSB 0x0F
#define BNO055_REG_MAG_Y_LSB 0x10
#define BNO055_REG_MAG_Y_MSB 0x11
#define BNO055_REG_MAG_Z_LSB 0x12
#define BNO055_REG_MAG_Z_MSB 0x13

#define BNO055_REG_GYRO_X_LSB 0x14
#define BNO055_REG_GYRO_X_MSB 0x15
#define BNO055_REG_GYRO_Y_LSB 0x16
#define BNO055_REG_GYRO_Y_MSB 0x17
#define BNO055_REG_GYRO_Z_LSB 0x18
#define BNO055_REG_GYRO_Z_MSB 0x19

#define BNO055_REG_EULER_H_LSB 0x1A
#define BNO055_REG_EULER_H_MSB 0x1B
#define BNO055_REG_EULER_R_LSB 0x1C
#define BNO055_REG_EULER_R_MSB 0x1D
#define BNO055_REG_EULER_P_LSB 0x1E
#define BNO055_REG_EULER_P_MSB 0x1F

#define BNO055_REG_QUAT_W_LSB 0x20
#define BNO055_REG_QUAT_W_MSB 0x21
#define BNO055_REG_QUAT_X_LSB 0x22
#define BNO055_REG_QUAT_X_MSB 0x23
#define BNO055_REG_QUAT_Y_LSB 0x24
#define BNO055_REG_QUAT_Y_MSB 0x25
#define BNO055_REG_QUAT_Z_LSB 0x26
#define BNO055_REG_QUAT_Z_MSB 0x27


#define BNO055_REG_LINACC_X_LSB 0x28
#define BNO055_REG_LINACC_X_MSB 0x29
#define BNO055_REG_LINACC_Y_LSB 0x2A
#define BNO055_REG_LINACC_Y_MSB 0x2B
#define BNO055_REG_LINACC_Z_LSB 0x2C
#define BNO055_REG_LINACC_Z_MSB 0x2D

#define BNO055_REG_GRAV_X_LSB 0x2E
#define BNO055_REG_GRAV_X_MSB 0x2F
#define BNO055_REG_GRAV_Y_LSB 0x30
#define BNO055_REG_GRAV_Y_MSB 0x31
#define BNO055_REG_GRAV_Z_LSB 0x32
#define BNO055_REG_GRAV_Z_MSB 0x33

#define BNO055_REG_TEMP       0x34

#define BNO055_REG_CALIB_STAT      0x35
#define BNO055_REG_SELFTEST_RESULT 0x36
#define BNO055_REG_INTR_STAT       0x37
#define BNO055_REG_SYS_CLK_STAT    0x38
#define BNO055_REG_SYS_STATUS      0x39
#define BNO055_REG_SYS_ERR         0x3A

#define BNO055_REG_UNIT_SEL    0x3B
#define BNO055_REG_DATA_SELECT 0x3C

#define BNO055_REG_OPR_MODE    0x3D
#define BNO055_REG_PWR_MODE    0x3E
#define BNO055_REG_SYS_TRIGGER 0x3F
#define BNO055_REG_TEMP_SOURCE 0x40

#define BNO055_REG_AXIS_MAP_CONFIG 0x41
#define BNO055_REG_AXIS_MAP_SIGN   0x42

#define BNO055_REG_SIC_0_LSB 0x43
#define BNO055_REG_SIC_0_MSB 0x44
#define BNO055_REG_SIC_1_LSB 0x45
#define BNO055_REG_SIC_1_MSB 0x46
#define BNO055_REG_SIC_2_LSB 0x47
#define BNO055_REG_SIC_2_MSB 0x48
#define BNO055_REG_SIC_3_LSB 0x49
#define BNO055_REG_SIC_3_MSB 0x4A
#define BNO055_REG_SIC_4_LSB 0x4B
#define BNO055_REG_SIC_4_MSB 0x4C
#define BNO055_REG_SIC_5_LSB 0x4D
#define BNO055_REG_SIC_5_MSB 0x4E
#define BNO055_REG_SIC_6_LSB 0x4F
#define BNO055_REG_SIC_6_MSB 0x50
#define BNO055_REG_SIC_7_LSB 0x51
#define BNO055_REG_SIC_7_MSB 0x52
#define BNO055_REG_SIC_8_LSB 0x53
#define BNO055_REG_SIC_8_MSB 0x54

#define BNO055_REG_OFFSETS_START    0x55
#define BNO055_REG_ACCEL_OFF_X_LSB  0x55
#define BNO055_REG_ACCEL_OFF_X_MSB  0x56
#define BNO055_REG_ACCEL_OFF_Y_LSB  0x57
#define BNO055_REG_ACCEL_OFF_Y_MSB  0x58
#define BNO055_REG_ACCEL_OFF_Z_LSB  0x59
#define BNO055_REG_ACCEL_OFF_Z_MSB  0x5A
#define BNO055_REG_MAG_OFF_X_LSB    0x5B
#define BNO055_REG_MAG_OFF_X_MSB    0x5C
#define BNO055_REG_MAG_OFF_Y_LSB    0x5D
#define BNO055_REG_MAG_OFF_Y_MSB    0x5E
#define BNO055_REG_MAG_OFF_Z_LSB    0x5F
#define BNO055_REG_MAG_OFF_Z_MSB    0x60
#define BNO055_REG_GYRO_OFF_X_LSB   0x61
#define BNO055_REG_GYRO_OFF_X_MSB   0x62
#define BNO055_REG_GYRO_OFF_Y_LSB   0x63
#define BNO055_REG_GYRO_OFF_Y_MSB   0x64
#define BNO055_REG_GYRO_OFF_Z_LSB   0x65
#define BNO055_REG_GYRO_OFF_Z_MSB   0x66
#define BNO055_REG_ACCEL_RADIUS_LSB 0x67
#define BNO055_REG_ACCEL_RADIUS_MSB 0x68
#define BNO055_REG_MAG_RADIUS_LSB   0x69
#define BNO055_REG_MAG_RADIUS_MSB   0x6A

#define BNO055_REG_P1_ACCEL_CONFIG       0x08
#define BNO055_REG_P1_MAG_CONFIG         0x09
#define BNO055_REG_P1_GYRO_CONFIG0       0x0A
#define BNO055_REG_P1_GYRO_CONFIG1       0x0B
#define BNO055_REG_P1_ACCEL_SLEEP_CONFIG 0x0C
#define BNO055_REG_P1_GYRO_SLEEP_CONFIG  0x0D
#define BNO055_REG_P1_MAG_SLEEP_CONFIG   0x0E

#define BNO055_REG_P1_INT_MASK               0x0F
#define BNO055_REG_P1_INT_EN                 0x10
#define BNO055_REG_P1_ACCEL_ANY_MOTION_THRES 0x11
#define BNO055_REG_P1_ACCEL_INTR_SETTINGS    0x12
#define BNO055_REG_P1_ACCEL_HIGH_G_DURN      0x13
#define BNO055_REG_P1_ACCEL_HIGH_G_THRES     0x14
#define BNO055_REG_P1_ACCEL_NO_MOTION_THRES  0x15
#define BNO055_REG_P1_ACCEL_NO_MOTION_SET    0x16
#define BNO055_REG_P1_GYRO_INTR_SETTING      0x17
#define BNO055_REG_P1_GYRO_HIGHRATE_X_SET    0x18
#define BNO055_REG_P1_GYRO_DURN_X            0x19
#define BNO055_REG_P1_GYRO_HIGHRATE_Y_SET    0x1A
#define BNO055_REG_P1_GYRO_DURN_Y            0x1B
#define BNO055_REG_P1_GYRO_HIGHRATE_Z_SET    0x1C
#define BNO055_REG_P1_GYRO_DURN_Z            0x1D
#define BNO055_REG_P1_GYRO_ANY_MOTION_THRES  0x1E
#define BNO055_REG_P1_GYRO_ANY_MOTION_SET    0x1F

#define EXPECTED_CHIP_ID                     0xA0
#define EXPECTED_ACCEL_ID                    0xFB
#define EXPECTED_MAG_ID                      0x32
#define EXPECTED_GYRO_ID                     0x0F

enum class BNO055_Power_Mode : uint8_t
{
    NORMAL = 0x00,
    LOW_POWER = 0x01,
    SUSPEND = 0x02
};

enum class BNO055_Interrupt : uint8_t
{
    AccelNoMotion = 0,
    AccelAnyMotion = 1,
    AccelHighG = 2,
    GyroHighRate = 3,
    GyroAnyMotion = 4,
    GyroNoMotion = 5,
    AccelSlowMotion = 6,
    AccelDataReady = 7
};

struct BNO055_Sensor_Data
{
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float scale;
};

struct EulerData
{
    int16_t rawHeading = 0;
    int16_t rawRoll = 0;
    int16_t rawPitch = 0;

    float heading = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;

    bool valid = false;

    uint32_t timestamp = 0;
};

struct QuaternionData
{
    int16_t rawW = 0;
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float w = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    bool valid = false;

    uint32_t timestamp = 0;
};

enum class BNO055_Sensor_Type : uint8_t
{
    Accel,
    Gyro,
    Mag,
    Linear,
    Gravity
};

enum class BNO055_OP_Mode : uint8_t
{
    CONFIG = 0x00,
    ACCONLY = 0x01,
    MAGONLY = 0x02,
    GYRONLY = 0x03,
    ACCMAG = 0x04,
    ACCGYRO = 0x05,
    MAGGYRO = 0x06,
    AMG = 0x07,
    IMUPLUS = 0x08,
    COMPASS = 0x09,
    M4G = 0x0A,
    NDOF_FMC_OFF = 0x0B,
    NDOF = 0x0C
};

enum class AccelRange : uint8_t
{
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3
};

enum class BNO055_Accel_Unit : uint8_t
{
    MetersPerSecondSquared = 0,
    MilliG = 1
};

enum class BNO055_Gyro_Unit : uint8_t
{
    DegreesPerSecond = 0,
    RadiansPerSecond = 1
};

enum class BNO055_Euler_Unit : uint8_t
{
    Degrees = 0,
    Radians = 1
};

enum class BNO055_Temperature_Unit : uint8_t
{
    Celsius = 0,
    Fahrenheit = 1
};

enum class BNO055_Orientation_Mode : uint8_t
{
    Android = 0,
    Windows = 1
};

enum class AccelBandwidth : uint8_t
{
    HZ7_81 = 0,
    HZ15_63 = 1,
    HZ31_25 = 2,
    HZ62_5 = 3,
    HZ125 = 4,
    HZ250 = 5,
    HZ500 = 6,
    HZ1000 = 7
};

enum class MagDataRate : uint8_t
{
    HZ2 = 0,
    HZ6 = 1,
    HZ8 = 2,
    HZ10 = 3,
    HZ15 = 4,
    HZ20 = 5,
    HZ25 = 6,
    HZ30 = 7
};

enum class MagPowerMode : uint8_t
{
    Normal = 0,
    Sleep = 1,
    Suspend = 2,
    ForceMode = 3
};

enum class MagOperationMode : uint8_t
{
    LowPower = 0,
    Regular = 1,
    EnhancedRegular = 2,
    HighAccuracy = 3
};

enum class AccelPowerMode : uint8_t
{
    Normal = 0,
    Suspend = 1,
    LowPower1 = 2,
    Standby = 3,
    LowPower2 = 4,
    DeepSuspend = 5
};

enum class GyroRange : uint8_t
{
    DPS2000 = 0,
    DPS1000 = 1,
    DPS500 = 2,
    DPS250 = 3,
    DPS125 = 4
};

enum class GyroBandwidth : uint8_t
{
    HZ523 = 0,
    HZ230 = 1,
    HZ116 = 2,
    HZ47 = 3,
    HZ23 = 4,
    HZ12 = 5,
    HZ64 = 6,
    HZ32 = 7
};

enum class GyroPowerMode : uint8_t
{
    Normal = 0,
    FastPowerUp = 1,
    DeepSuspend = 2,
    Suspend = 3,
    Advanced = 4
};


struct AccelConfig
{
    AccelRange range;
    AccelBandwidth bandwidth;
    AccelPowerMode powerMode;
    BNO055_Accel_Unit unit;
};
struct MagConfig
{
    MagDataRate dataRate;
    MagPowerMode powerMode;
    MagOperationMode operationMode;
};

struct GyroConfig
{
    GyroRange range;
    GyroBandwidth bandwidth;
    GyroPowerMode powerMode;
    BNO055_Gyro_Unit unit;
};

struct BNO055_Calibration
{
    uint8_t system = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;

    bool systemReady = false;
    bool gyroReady = false;
    bool accelReady = false;
    bool magReady = false;

    bool fullyCalibrated = false;

    uint8_t raw = 0;
};

struct Euler
{
    float heading;
    float roll;
    float pitch;
};

struct BNO055_Offset
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct AxisRemap
{
    uint8_t config;
    uint8_t sign;
};

namespace AxisMap
{
    constexpr AxisRemap P0 = {0x21, 0x04}; // Default
    constexpr AxisRemap P1 = {0x24, 0x00};
    constexpr AxisRemap P2 = {0x24, 0x06};
    constexpr AxisRemap P3 = {0x21, 0x02};
    constexpr AxisRemap P4 = {0x24, 0x03};
    constexpr AxisRemap P5 = {0x21, 0x01};
    constexpr AxisRemap P6 = {0x21, 0x07};
    constexpr AxisRemap P7 = {0x24, 0x05};
}


struct BNO055_CalibrationData
{
    uint8_t data[22] = {0};
};

struct DeviceInfo
{
    uint8_t chipID;
    uint8_t accelID;
    uint8_t magID;
    uint8_t gyroID;

    uint16_t swRevision;
    uint8_t blRevision;
};

class BNO055_7Semi
{
public:
     BNO055_7Semi();

    ~BNO055_7Semi();

    /**
     * begin
     *
     * - Initializes the BNO055 sensor over I2C
     * - Verifies the device identity
     * - Performs hardware and software reset sequences
     * - Configures the sensor for NDOF fusion mode
     * - Validates internal accelerometer, magnetometer and gyroscope IDs
     *
     * - Initialize I2C communication
     * - Create BusIO interface
     * - Verify BNO055 chip ID
     * - Reset and configure the device
     * - Enable NDOF operating mode
     * - Verify internal sensor IDs
     *
     * Returns:
     * - true  -> Sensor initialized successfully
     * - false -> Initialization failed
     */
    bool begin(uint8_t i2cAddress = 0x28, TwoWire &i2cPort = Wire, uint32_t i2cSpeed = 400000);

    /**
     * readChipID
     *
     * - Reads the BNO055 chip identification register
     * - Used to verify communication with the device
     *
     * Returns:
     * - true  -> Chip ID read successfully
     * - false -> Read operation failed
     */
    bool readChipID(uint8_t &chipID);

    /**
     * readSensorID
     *
     * - Reads the revision ID of an internal sensor
     * - Supports accelerometer, magnetometer and gyroscope
     * - Useful for device validation during initialization
     *
     * Returns:
     * - true  -> Sensor ID read successfully
     * - false -> Invalid sensor type or read failure
     */
    bool readSensorID(BNO055_Sensor_Type sensor, uint8_t &sensorID);

    /**
     * softReset
     *
     * - Performs a software reset of the BNO055
     * - Switches the device to configuration mode before reset
     * - Waits for the device to reboot and become responsive
     * - Verifies communication by reading the chip ID
     *
     * Returns:
     * - true  -> Reset completed successfully
     * - false -> Reset or verification failed
     */
    bool softReset();

    /**
     * setOpMode
     *
     * - Changes the BNO055 operating mode
     * - Updates the cached operating mode value
     * - Waits for the mode transition to complete
     *
     * Returns:
     * - true  -> Mode changed successfully
     * - false -> Write operation failed
     */
    bool setOpMode(BNO055_OP_Mode mode);

    /**
     * getMode
     *
     * - Reads the current BNO055 operating mode
     *
     * Returns:
     * - true  -> Mode read successfully
     * - false -> Read operation failed
     */
    bool getMode(uint8_t &mode);

    /**
     * enableExternalCrystal
     *
     * - Enables or disables the external crystal oscillator
     * - Improves timing stability when an external crystal is available
     *
     * Returns:
     * - true  -> Configuration updated successfully
     * - false -> Configuration failed
     */
    bool enableExternalCrystal(bool enable = true);

    /**
     * getCalibration
     *
     * - Reads the current calibration status of the BNO055
     * - Extracts calibration levels for all internal sensors
     * - Updates individual ready flags
     * - Determines overall calibration state
     *
     * Returns:
     * - true  -> Calibration status read successfully
     * - false -> Read operation failed
     */
    bool getCalibration(BNO055_Calibration &calibration);

    /**
     * setAxis
     *
     * - Configures the BNO055 axis remap settings
     * - Updates axis mapping and sign configuration
     * - Temporarily switches to configuration mode
     * - Restores the previous operating mode after update
     *
     * Returns:
     * - true  -> Axis configuration updated successfully
     * - false -> Configuration failed
     */
    bool setAxis( AxisRemap &remap);

    /**
     * getAxis
     *
     * - Reads the current axis remap configuration
     * - Returns axis mapping and sign settings
     *
     * Returns:
     * - true  -> Axis configuration read successfully
     * - false -> Read operation failed
     */
    bool getAxis(AxisRemap &remap);

    /**
     * readAccel
     *
     * - Reads acceleration data in units
     * - Returns acceleration values in meters per second squared
     *
     * Returns:
     * - true  -> Acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readAccel(float &x, float &y, float &z);

    /**
     * readAccel
     *
     * - Reads raw accelerometer data
     * - Returns unscaled sensor values directly from the device
     *
     * Returns:
     * - true  -> Acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readAccel(int16_t &x, int16_t &y, int16_t &z);

    /**
     * readAccel
     *
     * - Reads accelerometer data into a sensor data structure
     * - Returns both raw and scaled measurements
     *
     * Returns:
     * - true  -> Acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readAccel(BNO055_Sensor_Data &accel);

    /**
     * readGyro
     *
     * - Reads gyroscope data in  units
     * - Returns angular velocity values in degrees per second
     *
     * Returns:
     * - true  -> Gyroscope data read successfully
     * - false -> Read operation failed
     */
    bool readGyro(float &x, float &y, float &z);

    /**
     * readGyro
     *
     * - Reads raw gyroscope data
     * - Returns unscaled sensor values directly from the device
     *
     * Returns:
     * - true  -> Gyroscope data read successfully
     * - false -> Read operation failed
     */
    bool readGyro(int16_t &x, int16_t &y, int16_t &z);

    /**
     * readGyro
     *
     * - Reads gyroscope data into a sensor data structure
     * - Returns both raw and scaled measurements
     *
     * Returns:
     * - true  -> Gyroscope data read successfully
     * - false -> Read operation failed
     */
    bool readGyro(BNO055_Sensor_Data &gyro);

    /**
     * readMag
     *
     * - Reads magnetometer data in  units
     * - Returns magnetic field measurements using the configured scaling factor
     *
     * Returns:
     * - true  -> Magnetometer data read successfully
     * - false -> Read operation failed
     */
    bool readMag(float &x, float &y, float &z);

    /**
     * readMag
     *
     * - Reads raw magnetometer data
     * - Returns unscaled sensor values directly from the device
     *
     * Returns:
     * - true  -> Magnetometer data read successfully
     * - false -> Read operation failed
     */
    bool readMag(int16_t &x, int16_t &y, int16_t &z);

    /**
     * readMag
     *
     * - Reads magnetometer data into a sensor data structure
     * - Returns both raw and scaled measurements
     *
     * Returns:
     * - true  -> Magnetometer data read successfully
     * - false -> Read operation failed
     */
    bool readMag(BNO055_Sensor_Data &mag);

    /**
     * readLinear
     *
     * - Reads linear acceleration data in units
     * - Returns acceleration values with gravity removed
     * - Uses the BNO055 sensor fusion engine
     *
     * Returns:
     * - true  -> Linear acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readLinear(float &x, float &y, float &z);

    /**
     * readLinear
     *
     * - Reads raw linear acceleration data
     * - Returns unscaled sensor values directly from the device
     * - Gravity component is removed by the fusion engine
     *
     * Returns:
     * - true  -> Linear acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readLinear(int16_t &x, int16_t &y, int16_t &z);

    /**
     * readLinear
     *
     * - Reads linear acceleration data into a sensor data structure
     * - Returns both raw and scaled measurements

     * - Gravity is removed from the reported acceleration values
     *
     * Returns:
     * - true  -> Linear acceleration data read successfully
     * - false -> Read operation failed
     */
    bool readLinear(BNO055_Sensor_Data &linear);

    /**
     * readGravity
     *
     * - Reads gravity vector data in units
     * - Returns the gravity component calculated by the fusion engine
     *
     * Returns:
     * - true  -> Gravity vector read successfully
     * - false -> Read operation failed
     */
    bool readGravity(float &x, float &y, float &z);

    /**
     * readGravity
     *
     * - Reads raw gravity vector data
     * - Returns unscaled sensor values directly from the device
     * - Gravity vector is calculated by the fusion engine
     *
     * Returns:
     * - true  -> Gravity vector read successfully
     * - false -> Read operation failed
     */
    bool readGravity(int16_t &x, int16_t &y, int16_t &z);

    /**
     * readGravity
     *
     * - Reads gravity vector data into a sensor data structure
     * - Returns both raw and scaled measurements
     * - Represents only the gravity component detected by the fusion engine
     *
     * Returns:
     * - true  -> Gravity vector read successfully
     * - false -> Read operation failed
     */
    bool readGravity(BNO055_Sensor_Data &gravity);

    /**
     * readEuler
     *
     * - Reads orientation data from the BNO055
     * - Returns heading, roll and pitch angles in degrees
     *
     * Returns:
     * - true  -> Euler data read successfully
     * - false -> Read operation failed
     */
    bool readEuler(float &heading, float &roll, float &pitch);

    /**
     * readEuler
     *
     * - Reads orientation data from the BNO055
     * - Returns heading, roll and pitch angles
     * - Converts raw sensor values into degrees
     *
     * Returns:
     * - true  -> Euler data read successfully
     * - false -> Read operation failed
     */
    bool readEuler(EulerData &euler);

    /**
     * readQuaternion
     *
     * - Reads orientation data as a quaternion
     * - Returns normalized quaternion components
     * - Useful for 3D orientation and motion tracking applications
     *
     * Returns:
     * - true  -> Quaternion data read successfully
     * - false -> Read operation failed
     */
    bool readQuaternion(float &w, float &x, float &y, float &z);

    /**
     * readQuaternion
     *
     * - Reads quaternion data from the BNO055
     * - Converts raw sensor values into normalized quaternion components
     *
     * Returns:
     * - true  -> Quaternion data read successfully
     * - false -> Read operation failed
     */
    bool readQuaternion(QuaternionData &quaternion);

    /**
     * readTemperature
     *
     * - Reads the internal BNO055 temperature sensor
     * - Returns the temperature value in degrees Celsius
     *
     * Returns:
     * - true  -> Temperature read successfully
     * - false -> Read operation failed
     */
    bool readTemperature(int8_t &temperature);

    /**
     * setAccelConfig
     *
     * - Configures the BNO055 accelerometer settings
     * - Updates measurement range, bandwidth and power mode
     *
     * Returns:
     * - true  -> Configuration updated successfully
     * - false -> Configuration failed
     */
    bool setAccelConfig( AccelConfig &config);

    /**
     * getAccelConfig
     *
     * - Reads the current accelerometer configuration
     * - Returns measurement range, bandwidth and power mode
     *
     * Returns:
     * - true  -> Configuration read successfully
     * - false -> Read operation failed
     */
    bool getAccelConfig(AccelConfig &config);

    /**
     * setGyroConfig
     *
     * - Configures the BNO055 gyroscope settings
     * - Updates measurement range, bandwidth and power mode
     *
     * Returns:
     * - true  -> Configuration updated successfully
     * - false -> Configuration failed
     */
    bool setGyroConfig( GyroConfig &config);

    /**
     * getGyroConfig
     *
     * - Reads the current gyroscope configuration
     * - Returns measurement range, bandwidth and power mode
     *
     * Returns:
     * - true  -> Configuration read successfully
     * - false -> Read operation failed
     */
    bool getGyroConfig(GyroConfig &config);

    /**
     * setMagConfig
     *
     * - Configures the BNO055 magnetometer settings
     * - Updates output data rate, operation mode and power mode
     *
     * Returns:
     * - true  -> Configuration updated successfully
     * - false -> Configuration failed
     */
    bool setMagConfig( MagConfig &config);

    /**
     * getMagConfig
     *
     * - Reads the current magnetometer configuration
     * - Returns output data rate, operation mode and power mode
     *
     * Returns:
     * - true  -> Configuration read successfully
     * - false -> Read operation failed
     */
    bool getMagConfig(MagConfig &config);

    /**
     * setAccelOffset
     *
     * - Sets accelerometer offset values
     * - Used for accelerometer calibration
     *
     * Returns:
     * - true  -> Offset updated successfully
     * - false -> Update failed
     */
    bool setAccelOffset(int16_t x, int16_t y, int16_t z);

    /**
     * setGyroOffset
     *
     * - Sets gyroscope offset values
     * - Used for gyroscope calibration
     *
     * Returns:
     * - true  -> Offset updated successfully
     * - false -> Update failed
     */
    bool setGyroOffset(int16_t x, int16_t y, int16_t z);

    /**
     * setMagOffset
     *
     * - Sets magnetometer offset values
     * - Used for magnetometer calibration
     *
     * Returns:
     * - true  -> Offset updated successfully
     * - false -> Update failed
     */
    bool setMagOffset(int16_t x, int16_t y, int16_t z);

    /**
     * getAccelOffset
     *
     * - Reads accelerometer offset values
     * - Returns X, Y and Z calibration offsets
     *
     * Returns:
     * - true  -> Offset read successfully
     * - false -> Read operation failed
     */
    bool getAccelOffset(int16_t &x, int16_t &y, int16_t &z);

    /**
     * getGyroOffset
     *
     * - Reads gyroscope offset values
     * - Returns X, Y and Z calibration offsets
     *
     * Returns:
     * - true  -> Offset read successfully
     * - false -> Read operation failed
     */
    bool getGyroOffset(int16_t &x, int16_t &y, int16_t &z);

    /**
     * getMagOffset
     *
     * - Reads magnetometer offset values
     * - Returns X, Y and Z calibration offsets
     *
     * Returns:
     * - true  -> Offset read successfully
     * - false -> Read operation failed
     */
    bool getMagOffset(int16_t &x, int16_t &y, int16_t &z);

    /**
     * enableInterrupt
     *
     * - Enables a specific interrupt source
     * - Updates the interrupt enable register mask
     *
     * Returns:
     * - true  -> Interrupt enabled successfully
     * - false -> Operation failed
     */
    bool enableInterrupt(BNO055_Interrupt interrupt);

    /**
     * disableInterrupt
     *
     * - Disables a specific interrupt source
     * - Updates the interrupt enable register mask
     *
     * Returns:
     * - true  -> Interrupt disabled successfully
     * - false -> Operation failed
     */
    bool disableInterrupt(BNO055_Interrupt interrupt);

    /**
     * isInterruptEnabled
     *
     * - Checks whether a specific interrupt is enabled
     * - Reads the interrupt enable register
     *
     * Returns:
     * - true  -> Status read successfully
     * - false -> Read operation failed
     */
    bool isInterruptEnabled(BNO055_Interrupt interrupt, bool &enabled);

    /**
     * getInterruptMask
     *
     * - Reads the current interrupt enable mask
     * - Returns the raw interrupt register value
     *
     * Returns:
     * - true  -> Mask read successfully
     * - false -> Read operation failed
     */
    bool getInterruptMask(uint8_t &mask);

    /**
     * disableAllInterrupts
     *
     * - Disables all interrupt sources
     * - Clears the interrupt enable register
     *
     * Returns:
     * - true  -> Interrupts disabled successfully
     * - false -> Operation failed
     */
    bool disableAllInterrupts();

    /**
     * enableAllInterrupts
     *
     * - Enables all interrupt sources
     * - Sets all interrupt enable bits
     *
     * Returns:
     * - true  -> Interrupts enabled successfully
     * - false -> Operation failed
     */
    bool enableAllInterrupts();

    /**
     * selectPage
     *
     * - Selects the active BNO055 register page
     * - Page 0 contains data and status registers
     * - Page 1 contains configuration registers
     *
     * Returns:
     * - true  -> Page selected successfully
     * - false -> Invalid page or write failed
     */
    bool selectPage(uint8_t page);

    /**
     * readCalibrationData
     *
     * - Reads all BNO055 calibration data
     * - Includes sensor offsets and radius values
     * - Can be stored and restored later
     *
     * Returns:
     * - true  -> Calibration data read successfully
     * - false -> Read operation failed
     */
    bool readCalibrationData(BNO055_CalibrationData &data);

    /**
     * writeCalibrationData
     *
     * - Writes BNO055 calibration data
     * - Restores sensor offsets and radius values
     * - Useful for skipping recalibration after startup
     *
     * Returns:
     * - true  -> Calibration data written successfully
     * - false -> Write operation failed
     */
    bool writeCalibrationData( BNO055_CalibrationData &data);

    /**
     * getAccelRadius
     *
     * - Reads accelerometer calibration radius
     * - Used by the BNO055 calibration engine
     *
     * Returns:
     * - true  -> Radius read successfully
     * - false -> Read operation failed
     */
    bool getAccelRadius(uint16_t &radius);

    /**
     * setAccelRadius
     *
     * - Updates the accelerometer calibration radius
     * - Stores the calibration reference value
     *
     * Returns:
     * - true  -> Radius written successfully
     * - false -> Write operation failed
     */
    bool setAccelRadius(uint16_t radius);

    /**
     * getMagRadius
     *
     * - Reads the magnetometer calibration radius
     * - Returns the stored calibration reference value
     *
     * Returns:
     * - true  -> Radius read successfully
     * - false -> Read operation failed
     */
    bool getMagRadius(uint16_t &radius);

    /**
     * setMagRadius
     *
     * - Updates the magnetometer calibration radius
     * - Stores the calibration reference value
     * - Available only in configuration mode
     *
     * Returns:
     * - true  -> Radius written successfully
     * - false -> Write operation failed
     */
    bool setMagRadius(uint16_t radius);

    /**
     * getDeviceInfo
     *
     * - Reads BNO055 device identification information
     * - Returns chip, sensor, software and bootloader revisions
     * - Useful for device verification and diagnostics
     *
     * Returns:
     * - true  -> Device information read successfully
     * - false -> Read operation failed
     */
    bool getDeviceInfo(DeviceInfo &info);

    /**
     * setPowerMode
     *
     * - Updates the BNO055 power mode
     * - Supports normal, low power and suspend modes
     *
     * Returns:
     * - true  -> Power mode updated successfully
     * - false -> Update operation failed
     */
    bool setPowerMode(BNO055_Power_Mode mode);

    /**
     * getPowerMode
     *
     * - Reads the current BNO055 power mode
     * - Returns the active power management setting
     *
     * Returns:
     * - true  -> Power mode read successfully
     * - false -> Read operation failed
     */
    bool getPowerMode(BNO055_Power_Mode &mode);

    /**
     * runSelfTest
     *
     * - Executes the BNO055 self-test
     * - Returns the self-test result bits
     * - Useful for device diagnostics
     *
     * Returns:
     * - true  -> Self-test completed successfully
     * - false -> Self-test failed
     */
    bool runSelfTest(uint8_t &result);

    /**
     * getSystemStatus
     *
     * - Reads the current system status
     * - Returns system status and error code
     * - Useful for diagnostics and fault detection
     *
     * Returns:
     * - true  -> Status read successfully
     * - false -> Read operation failed
     */
    bool getSystemStatus(uint8_t &status, uint8_t &error);

    bool setUnits(uint8_t units);

private:
    I2C_7Semi i2c;
    SPI_7Semi spi;

    BusIO_7Semi<BusLink_7Semi, 1, 1> *bus = nullptr;

    BNO055_OP_Mode user_op_mode = BNO055_OP_Mode::NDOF;

    BNO055_Accel_Unit accel_unit = BNO055_Accel_Unit::MetersPerSecondSquared;
    BNO055_Gyro_Unit gyro_unit = BNO055_Gyro_Unit::DegreesPerSecond;
    BNO055_Euler_Unit euler_unit = BNO055_Euler_Unit::Degrees;

    float accel_scale = 9.80665f / 1000.0f;
    float gyro_scale = 1.0f / 16.0f;
    float mag_scale = 1.0f / 16.0f;
    float linear_scale = 9.80665f / 1000.0f;
    float gravity_scale = 9.80665f / 1000.0f;
    float euler_scale = 1.0f / 16.0f;

    bool readSensorData(BNO055_Sensor_Type sensorType, int16_t &x, int16_t &y, int16_t &z);

    bool readSensor(BNO055_Sensor_Type sensorType, float scale, BNO055_Sensor_Data &data);

    bool setOffset(BNO055_Sensor_Type sensorType, int16_t x, int16_t y, int16_t z);

    bool getOffset(BNO055_Sensor_Type sensorType, int16_t &x, int16_t &y, int16_t &z);

    bool updateInterruptMask(BNO055_Interrupt interrupt, bool enable);

    bool setMode(BNO055_OP_Mode mode);
};
#endif
