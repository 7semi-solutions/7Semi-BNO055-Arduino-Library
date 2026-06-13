#include "7Semi_BNO055.h"

BNO055_7Semi::BNO055_7Semi() {}

BNO055_7Semi::~BNO055_7Semi()
{
  if (bus)
  {
    delete bus;
    bus = nullptr;
  }
}

bool BNO055_7Semi::begin(uint8_t i2cAddress, TwoWire &i2cPort, uint32_t i2cSpeed)
{
  // Remove existing bus instance
  if (bus)
  {
    delete bus;
    bus = nullptr;
  }

  // Initialize I2C communication
  if (!i2c.beginI2C(i2cAddress, i2cPort, i2cSpeed))
    return false;

  // Create BusIO interface
  bus = new BusIO_7Semi<BusLink_7Semi, 1, 1>(i2c);

  if (!bus)
    return false;

  uint8_t chip_id;

  // Verify BNO055 chip ID
  if (!bus->read(BNO055_REG_CHIP_ID, chip_id))
    return false;

  if (chip_id != EXPECTED_CHIP_ID)
    return false;

  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  delay(25);

  // Trigger device reset
  if (!bus->write(BNO055_REG_SYS_TRIGGER, 0x20))
    return false;

  delay(30);

  // Wait for device reboot
  uint32_t start = millis();

  while (true)
  {
    delay(10);

    if (bus->read(BNO055_REG_CHIP_ID, chip_id))
    {
      if (chip_id == EXPECTED_CHIP_ID)
      {
        break;
      }
    }

    if ((millis() - start) > 1000)
    {
      return false;
    }
  }

  // Configure normal power mode
  if (!bus->write(BNO055_REG_PWR_MODE, (uint8_t)(BNO055_Power_Mode::NORMAL)))
    return false;

  delay(10);

  // Select register page 0
  if (!bus->write(BNO055_REG_PAGE_ID, 0))
    return false;

  // Perform software reset
  if (!softReset())
    return false;

  uint8_t accelId;
  uint8_t magId;
  uint8_t gyroId;

  // Read accelerometer revision ID
  if (!bus->read(BNO055_REG_ACCEL_REV_ID, accelId))
    return false;

  // Read magnetometer revision ID
  if (!bus->read(BNO055_REG_MAG_REV_ID, magId))
    return false;

  // Read gyroscope revision ID
  if (!bus->read(BNO055_REG_GYRO_REV_ID, gyroId))
    return false;

  // Verify internal sensor identities
  if (accelId != EXPECTED_ACCEL_ID)
    return false;

  if (magId != EXPECTED_MAG_ID)
    return false;

  if (gyroId != EXPECTED_GYRO_ID)
    return false;

  // Enable NDOF fusion mode
  if (!setMode(BNO055_OP_Mode::NDOF))
    return false;

  user_op_mode = BNO055_OP_Mode::NDOF;

  // Sensor initialization successful
  return true;
}

bool BNO055_7Semi::readChipID(uint8_t &chipID)
{
  if (!bus)
    return false;

  // Read chip identification register
  return bus->read(BNO055_REG_CHIP_ID, chipID);
}

bool BNO055_7Semi::readSensorID(BNO055_Sensor_Type sensor, uint8_t &sensorID)
{
  if (!bus)
    return false;

  uint8_t reg;

  // Select sensor revision register
  switch (sensor)
  {
  case BNO055_Sensor_Type::Accel:
    reg = BNO055_REG_ACCEL_REV_ID;
    break;

  case BNO055_Sensor_Type::Mag:
    reg = BNO055_REG_MAG_REV_ID;
    break;

  case BNO055_Sensor_Type::Gyro:
    reg = BNO055_REG_GYRO_REV_ID;
    break;

  default:
    return false;
  }

  // Read selected sensor revision ID
  if (!bus->read(reg, sensorID))
    return false;

  return true;
}

bool BNO055_7Semi::softReset()
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Trigger software reset
  if (!bus->writeBit(BNO055_REG_SYS_TRIGGER, 5, true))
    return false;

  // Wait for device reboot
  uint32_t start = millis();
  uint8_t chip_Id;

  while ((millis() - start) < 1000)
  {
    if (readChipID(chip_Id) && (chip_Id == EXPECTED_CHIP_ID))
    {
      return setMode(user_op_mode);
    }

    delay(10);
  }
  delay(100);

  setMode(user_op_mode);

  return false;
}

bool BNO055_7Semi::setMode(BNO055_OP_Mode mode)
{
    if (!bus)
        return false;

    uint8_t currentMode;

    // Read current operating mode from sensor
    if (!bus->read(BNO055_REG_OPR_MODE, currentMode))
        return false;

    if ((currentMode & 0x0F) == uint8_t(mode))
        return true;

    // Write new operating mode
    if (!bus->write(BNO055_REG_OPR_MODE, uint8_t(mode)))
        return false;

    // Wait for mode transition
    if (mode == BNO055_OP_Mode::CONFIG)
        delay(25);
    else
        delay(10);

    return true;
}

bool BNO055_7Semi::setOpMode(BNO055_OP_Mode mode)
{
  user_op_mode = mode;
  Serial.println((uint8_t)user_op_mode);
  return setMode(mode);
}

bool BNO055_7Semi::getMode(uint8_t &mode)
{
  if (!bus)
    return false;

  // Read operating mode register
  if (!bus->read(BNO055_REG_OPR_MODE, mode))
    return false;

  mode &= 0x0F;

  return true;
}

bool BNO055_7Semi::enableExternalCrystal(bool enable)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  delay(25);

  // Update external crystal setting
  if (!bus->writeBit(BNO055_REG_SYS_TRIGGER, 7, enable))
    return false;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getCalibration(BNO055_Calibration &calibration)
{
  if (!bus)
    return false;

  // Read calibration status register
  if (!bus->read(BNO055_REG_CALIB_STAT, calibration.raw))
    return false;

  // Extract system calibration level
  calibration.system = (calibration.raw >> 6) & 0x03;

  // Extract gyroscope calibration level
  calibration.gyro = (calibration.raw >> 4) & 0x03;

  // Extract accelerometer calibration level
  calibration.accel = (calibration.raw >> 2) & 0x03;

  // Extract magnetometer calibration level
  calibration.mag = calibration.raw & 0x03;

  // Update system calibration status
  calibration.systemReady = (calibration.system == 3);

  // Update gyroscope calibration status
  calibration.gyroReady = (calibration.gyro == 3);

  // Update accelerometer calibration status
  calibration.accelReady = (calibration.accel == 3);

  // Update magnetometer calibration status
  calibration.magReady = (calibration.mag == 3);

  // Determine overall calibration status
  calibration.fullyCalibrated = calibration.systemReady && calibration.gyroReady &&
                                calibration.accelReady && calibration.magReady;

  return true;
}

bool BNO055_7Semi::setAxis(AxisRemap &remap)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  delay(20);

  uint8_t cfg[2] = {remap.config, remap.sign};

  // Write axis mapping configuration
  if (!bus->write(BNO055_REG_AXIS_MAP_CONFIG, cfg, 2))
    return false;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getAxis(AxisRemap &remap)
{
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  uint8_t cfg[2];

  // Read axis mapping configuration
  if (!bus->read(BNO055_REG_AXIS_MAP_CONFIG, cfg, 2))
    return false;

  remap.config = cfg[0];
  remap.sign = cfg[1];

  return setMode(user_op_mode);
}

bool BNO055_7Semi::readAccel(float &x, float &y, float &z)
{
  BNO055_Sensor_Data data;

  // Read accelerometer data
  if (!readAccel(data))
    return false;

  // Copy scaled values
  x = data.x;
  y = data.y;
  z = data.z;

  return true;
}

bool BNO055_7Semi::readAccel(int16_t &x, int16_t &y, int16_t &z)
{
  BNO055_Sensor_Data data;

  // Read accelerometer data
  if (!readAccel(data))
    return false;

  // Copy raw values
  x = data.rawX;
  y = data.rawY;
  z = data.rawZ;

  return true;
}

bool BNO055_7Semi::readAccel(BNO055_Sensor_Data &data)
{
  // Read accelerometer sensor data
  return readSensor(BNO055_Sensor_Type::Accel, accel_scale, data);
}

bool BNO055_7Semi::readGyro(float &x, float &y, float &z)
{
  BNO055_Sensor_Data gyro;

  // Read gyroscope data
  if (!readGyro(gyro))
    return false;

  // Copy scaled values
  x = gyro.x;
  y = gyro.y;
  z = gyro.z;

  return true;
}

bool BNO055_7Semi::readGyro(int16_t &x, int16_t &y, int16_t &z)
{
  BNO055_Sensor_Data gyro;

  // Read gyroscope data
  if (!readGyro(gyro))
    return false;

  // Copy raw values
  x = gyro.rawX;
  y = gyro.rawY;
  z = gyro.rawZ;

  return true;
}

bool BNO055_7Semi::readGyro(BNO055_Sensor_Data &gyro)
{
  // Read gyroscope sensor data
  return readSensor(BNO055_Sensor_Type::Gyro, gyro_scale, gyro);
}

bool BNO055_7Semi::readMag(float &x, float &y, float &z)
{
  BNO055_Sensor_Data mag;

  // Read magnetometer data
  if (!readMag(mag))
    return false;

  // Copy scaled values
  x = mag.x;
  y = mag.y;
  z = mag.z;

  return true;
}

bool BNO055_7Semi::readMag(int16_t &x, int16_t &y, int16_t &z)
{
  BNO055_Sensor_Data mag;

  // Read magnetometer data
  if (!readMag(mag))
    return false;

  // Copy raw values
  x = mag.rawX;
  y = mag.rawY;
  z = mag.rawZ;

  return true;
}

bool BNO055_7Semi::readMag(BNO055_Sensor_Data &mag)
{
  // Read magnetometer sensor data
  return readSensor(BNO055_Sensor_Type::Mag, mag_scale, mag);
}

bool BNO055_7Semi::readLinear(
    float &x,
    float &y,
    float &z)
{
  BNO055_Sensor_Data linear;

  // Read linear acceleration data
  if (!readLinear(linear))
    return false;

  // Copy scaled values
  x = linear.x;
  y = linear.y;
  z = linear.z;

  return true;
}

bool BNO055_7Semi::readLinear(int16_t &x, int16_t &y, int16_t &z)
{
  BNO055_Sensor_Data linear;

  // Read linear acceleration data
  if (!readLinear(linear))
    return false;

  // Copy raw values
  x = linear.rawX;
  y = linear.rawY;
  z = linear.rawZ;

  return true;
}

bool BNO055_7Semi::readLinear(BNO055_Sensor_Data &linear)
{
  // Read linear acceleration sensor data
  return readSensor(BNO055_Sensor_Type::Linear, linear_scale, linear);
}

bool BNO055_7Semi::readGravity(float &x, float &y, float &z)
{
  BNO055_Sensor_Data gravity;

  // Read gravity vector data
  if (!readGravity(gravity))
    return false;

  // Copy scaled values
  x = gravity.x;
  y = gravity.y;
  z = gravity.z;

  return true;
}

bool BNO055_7Semi::readGravity(int16_t &x, int16_t &y, int16_t &z)
{
  BNO055_Sensor_Data gravity;

  // Read gravity vector data
  if (!readGravity(gravity))
    return false;

  // Copy raw values
  x = gravity.rawX;
  y = gravity.rawY;
  z = gravity.rawZ;

  return true;
}

bool BNO055_7Semi::readGravity(BNO055_Sensor_Data &gravity)
{
  // Read gravity vector sensor data
  return readSensor(BNO055_Sensor_Type::Gravity, gravity_scale, gravity);
}

bool BNO055_7Semi::readEuler(float &heading, float &roll, float &pitch)
{
  EulerData euler;

  // Read Euler angle data
  if (!readEuler(euler))
    return false;

  // Copy converted values
  heading = euler.heading;
  roll = euler.roll;
  pitch = euler.pitch;

  return true;
}

bool BNO055_7Semi::readEuler(EulerData &euler)
{
  if (!selectPage(0))
    return false;

  if (!setMode(user_op_mode))
    return false;

  uint8_t buffer[6];

  // Read Euler angle registers
  if (!bus->read(BNO055_REG_EULER_H_LSB, buffer, sizeof(buffer)))
    return false;

  // Extract raw heading value
  euler.rawHeading = int16_t((uint16_t(buffer[1]) << 8) | buffer[0]);

  // Extract raw roll value
  euler.rawRoll = int16_t((uint16_t(buffer[3]) << 8) | buffer[2]);

  // Extract raw pitch value
  euler.rawPitch = int16_t((uint16_t(buffer[5]) << 8) | buffer[4]);

  float euler_scale = 1.0f / 16.0f;

  // Convert heading to degrees
  euler.heading = euler.rawHeading * euler_scale;

  // Convert roll to degrees
  euler.roll = euler.rawRoll * euler_scale;

  // Convert pitch to degrees
  euler.pitch = euler.rawPitch * euler_scale;

  return true;
}

bool BNO055_7Semi::readQuaternion(float &w, float &x, float &y, float &z)
{
  QuaternionData quaternion;

  // Read quaternion data
  if (!readQuaternion(quaternion))
    return false;

  // Copy converted values
  w = quaternion.w;
  x = quaternion.x;
  y = quaternion.y;
  z = quaternion.z;

  return true;
}

bool BNO055_7Semi::readQuaternion(QuaternionData &quaternion)
{
    if (!setMode(user_op_mode))
    return false;

  uint8_t buffer[8];

  // Read quaternion registers
  if (!bus->read(BNO055_REG_QUAT_W_LSB, buffer, sizeof(buffer)))
    return false;

  // Extract raw W component
  quaternion.rawW = int16_t((uint16_t(buffer[1]) << 8) | buffer[0]);

  // Extract raw X component
  quaternion.rawX = int16_t((uint16_t(buffer[3]) << 8) | buffer[2]);

  // Extract raw Y component
  quaternion.rawY = int16_t((uint16_t(buffer[5]) << 8) | buffer[4]);

  // Extract raw Z component
  quaternion.rawZ = int16_t((uint16_t(buffer[7]) << 8) | buffer[6]);

  float quad_scale = 1.0f / 16384.0f;

  // Convert W component
  quaternion.w = quaternion.rawW * quad_scale;

  // Convert X component
  quaternion.x = quaternion.rawX * quad_scale;

  // Convert Y component
  quaternion.y = quaternion.rawY * quad_scale;

  // Convert Z component
  quaternion.z = quaternion.rawZ * quad_scale;

  return true;
}

bool BNO055_7Semi::readSensor(BNO055_Sensor_Type sensorType, float scale, BNO055_Sensor_Data &data)
{
  // Read raw sensor values
  if (!readSensorData(sensorType, data.rawX, data.rawY, data.rawZ))
    return false;

  // Convert X axis value
  data.x = data.rawX * scale;

  // Convert Y axis value
  data.y = data.rawY * scale;

  // Convert Z axis value
  data.z = data.rawZ * scale;

  // Store conversion scale
  data.scale = scale;

  return true;
}

bool BNO055_7Semi::readTemperature(int8_t &temperature)
{
    if (!setMode(user_op_mode))
    return false;

  uint8_t value;

  // Read temperature register
  if (!bus->read(BNO055_REG_TEMP, value))
    return false;

  // Convert register value to signed temperature
  temperature = static_cast<int8_t>(value);

  return true;
}

bool BNO055_7Semi::setAccelConfig(AccelConfig &config)
{
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  if (!selectPage(1))
    return false;

  uint8_t value = (uint8_t(config.range) & 0x03) |
                  ((uint8_t(config.bandwidth) & 0x07) << 2) |
                  ((uint8_t(config.powerMode) & 0x07) << 5);

  if (!bus->write(BNO055_REG_P1_ACCEL_CONFIG, value))
    return false;

  if (!selectPage(0))
    return false;

  bool unitBit = (config.unit == BNO055_Accel_Unit::MilliG);

  if (!bus->writeBit(BNO055_REG_UNIT_SEL, 0, unitBit))
    return false;

  accel_scale = unitBit ? (1.0f / 1000.0f) : (9.80665f / 1000.0f);

  linear_scale = accel_scale;
  gravity_scale = accel_scale;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getAccelConfig(AccelConfig &config)
{
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  if (!selectPage(1))
    return false;

  uint8_t value;

  if (!bus->read(BNO055_REG_P1_ACCEL_CONFIG, value))
    return false;

  config.range = AccelRange(value & 0x03);

  config.bandwidth = AccelBandwidth((value >> 2) & 0x07);

  config.powerMode = AccelPowerMode((value >> 5) & 0x07);

  if (!selectPage(0))
    return false;

  uint8_t unitSel;

  if (!bus->read(BNO055_REG_UNIT_SEL, unitSel))
    return false;

  config.unit = (unitSel & 0x01) ? BNO055_Accel_Unit::MilliG : BNO055_Accel_Unit::MetersPerSecondSquared;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::setGyroConfig(GyroConfig &config)
{
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  if (!selectPage(1))
    return false;

  uint8_t cfg[2];
  cfg[0] = (uint8_t(config.range) & 0x07) | ((uint8_t(config.bandwidth) & 0x07) << 3);

  cfg[1] = uint8_t(config.powerMode) & 0x07;

  if (!bus->write(BNO055_REG_P1_GYRO_CONFIG0, cfg, 2))
    return false;

  if (!selectPage(0))
    return false;

  bool unitBit = (config.unit == BNO055_Gyro_Unit::RadiansPerSecond);

  if (!bus->writeBit(BNO055_REG_UNIT_SEL, 1, unitBit))
    return false;

  // Update scale from selected unit
  if (config.unit == BNO055_Gyro_Unit::DegreesPerSecond)
  {
    gyro_scale = 1.0f / 16.0f;
  }
  else
  {
    gyro_scale = 1.0f / 900.0f;
  }

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getGyroConfig(GyroConfig &config)
{
  if (!selectPage(1))
    return false;

  uint8_t cfg[2];

  if (!bus->read(BNO055_REG_P1_GYRO_CONFIG0, cfg, 2))
    return false;

  config.range = GyroRange(cfg[0] & 0x07);

  config.bandwidth = GyroBandwidth((cfg[0] >> 3) & 0x07);

  config.powerMode = GyroPowerMode(cfg[1] & 0x07);

  if (!selectPage(0))
    return false;

  uint8_t unitSel;

  if (!bus->read(BNO055_REG_UNIT_SEL, unitSel))
    return false;

  config.unit = (unitSel & 0x02) ? BNO055_Gyro_Unit::RadiansPerSecond : BNO055_Gyro_Unit::DegreesPerSecond;

  if (!selectPage(0))
    return false;

  return true;
}

bool BNO055_7Semi::setMagConfig(MagConfig &config)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Select configuration register page
  if (!selectPage(1))
    return false;

  // Build magnetometer configuration value
  uint8_t value = (uint8_t(config.dataRate) & 0x07) |
                  ((uint8_t(config.operationMode) & 0x03) << 3) |
                  ((uint8_t(config.powerMode) & 0x03) << 5);

  // Write magnetometer configuration
  if (!bus->write(BNO055_REG_P1_MAG_CONFIG, value))
    return false;

  // Restore default register page
  if (!selectPage(0))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::getMagConfig(MagConfig &config)
{
  // Select configuration register page
  if (!selectPage(1))
    return false;

  uint8_t value;

  // Read magnetometer configuration register
  if (!bus->read(BNO055_REG_P1_MAG_CONFIG, value))
    return false;

  // Extract output data rate
  config.dataRate = MagDataRate(value & 0x07);

  // Extract operating mode
  config.operationMode = MagOperationMode((value >> 3) & 0x03);

  // Extract power mode
  config.powerMode = MagPowerMode((value >> 5) & 0x03);

  if (!selectPage(0))
    return false;

  return true;
}

bool BNO055_7Semi::setAccelOffset(int16_t x, int16_t y, int16_t z)
{
  return setOffset(BNO055_Sensor_Type::Accel, x, y, z);
}

bool BNO055_7Semi::setGyroOffset(int16_t x, int16_t y, int16_t z)
{
  return setOffset(BNO055_Sensor_Type::Gyro, x, y, z);
}

bool BNO055_7Semi::setMagOffset(int16_t x, int16_t y, int16_t z)
{
  return setOffset(BNO055_Sensor_Type::Mag, x, y, z);
}

bool BNO055_7Semi::getAccelOffset(int16_t &x, int16_t &y, int16_t &z)
{
  return getOffset(BNO055_Sensor_Type::Accel, x, y, z);
}

bool BNO055_7Semi::getMagOffset(int16_t &x, int16_t &y, int16_t &z)
{
  return getOffset(BNO055_Sensor_Type::Mag, x, y, z);
}

bool BNO055_7Semi::getGyroOffset(int16_t &x, int16_t &y, int16_t &z)
{
  return getOffset(BNO055_Sensor_Type::Gyro, x, y, z);
}

bool BNO055_7Semi::setOffset(BNO055_Sensor_Type sensorType, int16_t x, int16_t y, int16_t z)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  uint8_t reg;

  // Select offset register block
  switch (sensorType)
  {
  case BNO055_Sensor_Type::Accel:
    reg = BNO055_REG_ACCEL_OFF_X_LSB;
    break;

  case BNO055_Sensor_Type::Gyro:
    reg = BNO055_REG_GYRO_OFF_X_LSB;
    break;

  case BNO055_Sensor_Type::Mag:
    reg = BNO055_REG_MAG_OFF_X_LSB;
    break;

  default:
    return setMode(user_op_mode);
  }

  // Build offset data buffer
  uint8_t buffer[6] = {uint8_t(x & 0xFF), uint8_t(x >> 8),

                       uint8_t(y & 0xFF), uint8_t(y >> 8),

                       uint8_t(z & 0xFF), uint8_t(z >> 8)};

  // Write offset values
  if (!bus->write(reg, buffer, sizeof(buffer)))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::getOffset(BNO055_Sensor_Type sensorType, int16_t &x, int16_t &y, int16_t &z)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  uint8_t reg;

  // Select offset register block
  switch (sensorType)
  {
  case BNO055_Sensor_Type::Accel:
    reg = BNO055_REG_ACCEL_OFF_X_LSB;
    break;

  case BNO055_Sensor_Type::Gyro:
    reg = BNO055_REG_GYRO_OFF_X_LSB;
    break;

  case BNO055_Sensor_Type::Mag:
    reg = BNO055_REG_MAG_OFF_X_LSB;
    break;

  default:

    return false;
  }

  uint8_t buffer[6];

  // Read offset values
  if (!bus->read(reg, buffer, sizeof(buffer)))
    return false;

  // Convert raw bytes into X offset
  x = int16_t((uint16_t(buffer[1]) << 8) | buffer[0]);

  // Convert raw bytes into Y offset
  y = int16_t((uint16_t(buffer[3]) << 8) | buffer[2]);

  // Convert raw bytes into Z offset
  z = int16_t((uint16_t(buffer[5]) << 8) | buffer[4]);

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::enableInterrupt(BNO055_Interrupt interrupt)
{
  return updateInterruptMask(interrupt, true);
}

bool BNO055_7Semi::disableInterrupt(BNO055_Interrupt interrupt)
{
  return updateInterruptMask(interrupt, false);
}

bool BNO055_7Semi::updateInterruptMask(BNO055_Interrupt interrupt, bool enable)
{
  if (!bus)
    return false;

  if (!selectPage(1))
    return false;

  if (!bus->writeBit(BNO055_REG_P1_INT_EN, (uint8_t)interrupt, enable))
    return false;

  if (!selectPage(0))
    return false;

  return true;
}

bool BNO055_7Semi::isInterruptEnabled(BNO055_Interrupt interrupt, bool &enabled)
{
  if (!bus)
    return false;

  if (!selectPage(1))
    return false;

  uint8_t mask;

  // Read interrupt enable register
  if (!bus->read(BNO055_REG_P1_INT_EN, mask))
    return false;

  // Extract interrupt state
  enabled = (mask & (1U << uint8_t(interrupt))) != 0;

  if (!selectPage(0))
    return false;

  return true;
}

bool BNO055_7Semi::getInterruptMask(uint8_t &mask)
{
  if (!bus)
    return false;

  if (!selectPage(1))
    return false;

  if (!bus->read(BNO055_REG_P1_INT_EN, mask))
    return false;

  return selectPage(0);
}

bool BNO055_7Semi::disableAllInterrupts()
{
  if (!bus)
    return false;

  if (!selectPage(1))
    return false;

  if (!bus->write(BNO055_REG_P1_INT_EN, 0x00))
    return false;

  return selectPage(0);
}

bool BNO055_7Semi::enableAllInterrupts()
{
  if (!bus)
    return false;

  if (!selectPage(1))
    return false;

  if (!bus->write(BNO055_REG_P1_INT_EN, 0xFF))
    return false;

  return selectPage(0);
}

bool BNO055_7Semi::selectPage(uint8_t page)
{
  // Validate page number
  if (page > 1)
    return false;

  // Update page register
  if (!bus->write(BNO055_REG_PAGE_ID, page))
    return false;

  return true;
}

bool BNO055_7Semi::readSensorData(BNO055_Sensor_Type sensorType, int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t reg = 0;

  if (!setMode(user_op_mode))
    return false;

  // Select sensor data register
  switch (sensorType)
  {
  case BNO055_Sensor_Type::Accel:
    reg = BNO055_REG_ACCEL_X_LSB;
    break;

  case BNO055_Sensor_Type::Gyro:
    reg = BNO055_REG_GYRO_X_LSB;
    break;

  case BNO055_Sensor_Type::Mag:
    reg = BNO055_REG_MAG_X_LSB;
    break;

  case BNO055_Sensor_Type::Linear:
    reg = BNO055_REG_LINACC_X_LSB;
    break;

  case BNO055_Sensor_Type::Gravity:
    reg = BNO055_REG_GRAV_X_LSB;
    break;

  default:
    return false;
  }

  // Select data register page
  if (!selectPage(0))
    return false;

  uint8_t buffer[6];

  // Read sensor data
  if (!bus->read(reg, buffer, sizeof(buffer)))
    return false;

  // Extract X axis value
  x = int16_t((uint16_t(buffer[1]) << 8) | buffer[0]);

  // Extract Y axis value
  y = int16_t((uint16_t(buffer[3]) << 8) | buffer[2]);

  // Extract Z axis value
  z = int16_t((uint16_t(buffer[5]) << 8) | buffer[4]);

  return true;
}

bool BNO055_7Semi::readCalibrationData(BNO055_CalibrationData &data)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Read calibration data block
  if (!bus->read(BNO055_REG_OFFSETS_START, data.data, sizeof(data.data)))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::writeCalibrationData(BNO055_CalibrationData &data)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Write calibration data block
  if (!bus->write(BNO055_REG_OFFSETS_START, data.data, sizeof(data.data)))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::getAccelRadius(uint16_t &radius)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  uint8_t buffer[2];

  // Read accelerometer radius
  if (!bus->read(BNO055_REG_ACCEL_RADIUS_LSB, buffer, sizeof(buffer)))
    return false;

  // Convert raw bytes into radius value
  radius = (uint16_t(buffer[1]) << 8) | buffer[0];

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::setAccelRadius(uint16_t radius)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Build radius data buffer
  uint8_t buffer[2] =
      {
          uint8_t(radius & 0xFF),
          uint8_t(radius >> 8)};

  // Write accelerometer radius
  if (!bus->write(BNO055_REG_ACCEL_RADIUS_LSB, buffer, sizeof(buffer)))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::getMagRadius(uint16_t &radius)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  uint8_t buffer[2];

  // Read magnetometer radius
  if (!bus->read(BNO055_REG_MAG_RADIUS_LSB, buffer, sizeof(buffer)))
    return false;

  // Convert raw bytes into radius value
  radius = (uint16_t(buffer[1]) << 8) | buffer[0];

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::setMagRadius(uint16_t radius)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Build radius data buffer
  uint8_t buffer[2] = {uint8_t(radius & 0xFF), uint8_t(radius >> 8)};

  // Write magnetometer radius
  if (!bus->write(BNO055_REG_MAG_RADIUS_LSB, buffer, sizeof(buffer)))
    return false;

  // Restore previous operating mode
  return setMode(user_op_mode);
}

bool BNO055_7Semi::getDeviceInfo(DeviceInfo &info)
{
  if (!bus)
    return false;

  // Read chip ID
  if (!bus->read(BNO055_REG_CHIP_ID, info.chipID))
    return false;

  // Read accelerometer ID
  if (!bus->read(BNO055_REG_ACCEL_REV_ID, info.accelID))
    return false;

  // Read magnetometer ID
  if (!bus->read(BNO055_REG_MAG_REV_ID, info.magID))
    return false;

  // Read gyroscope ID
  if (!bus->read(BNO055_REG_GYRO_REV_ID, info.gyroID))
    return false;

  uint8_t sw_revision[2];

  // Read software revision
  if (!bus->read(BNO055_REG_SW_REV_LSB, sw_revision, 2))
    return false;

  // Combine software revision bytes
  info.swRevision = (uint16_t(sw_revision[1]) << 8) | sw_revision[0];

  // Read bootloader revision
  if (!bus->read(BNO055_REG_BL_REV_ID, info.blRevision))
    return false;

  return true;
}

bool BNO055_7Semi::setPowerMode(BNO055_Power_Mode mode)
{
  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Write power mode register
  if (!bus->write(BNO055_REG_PWR_MODE, uint8_t(mode)))
    return false;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getPowerMode(BNO055_Power_Mode &mode)
{
  if (!bus)
    return false;

  uint8_t value;

  // Read power mode register
  if (!bus->read(BNO055_REG_PWR_MODE, value))
    return false;

  // Extract power mode value
  mode = static_cast<BNO055_Power_Mode>(value & 0x03);

  return true;
}

bool BNO055_7Semi::runSelfTest(uint8_t &result)
{

  // Switch to configuration mode
  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  // Start self-test
  if (!bus->writeBit(BNO055_REG_SYS_TRIGGER, 0, true))
    return false;

  // Wait for self-test to complete
  delay(1000);

  uint8_t value;

  // Read self-test result register
  if (!bus->read(BNO055_REG_SELFTEST_RESULT, value))
    return false;

  result = value & 0x0F;

  return setMode(user_op_mode);
}

bool BNO055_7Semi::getSystemStatus(uint8_t &status, uint8_t &error)
{

  // Read system status register
  if (!bus->read(BNO055_REG_SYS_STATUS, status))
    return false;

  // Read system error register
  if (!bus->read(BNO055_REG_SYS_ERR, error))
    return false;

  return true;
}

bool BNO055_7Semi::setUnits(uint8_t units)
{

  if (!setMode(BNO055_OP_Mode::CONFIG))
    return false;

  if (!bus->write(BNO055_REG_UNIT_SEL, units))
    return false;

  if (units & 0x01)
  {
    accel_scale = 9.80665f / 1000.0f;
    linear_scale = accel_scale;
    gravity_scale = accel_scale;
  }
  else
  {
    accel_scale = 1.0f / 100.0f;
    linear_scale = accel_scale;
    gravity_scale = accel_scale;
  }

  if (units & 0x02)
  {
    gyro_scale = 1.0f / 900.0f;
  }
  else
  {
    gyro_scale = 1.0f / 16.0f;
  }

  if (units & 0x04)
  {
    euler_scale = 1.0f / 900.0f;
  }
  else
  {
    euler_scale = 1.0f / 16.0f;
  }

  return setMode(user_op_mode);
}