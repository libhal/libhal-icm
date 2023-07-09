// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>

#include <libhal/accelerometer.hpp>
#include <libhal/gyroscope.hpp>
#include <libhal/magnetometer.hpp>
#include <libhal/temperature_sensor.hpp>

namespace hal::icm {

// Misc configuration macros
static constexpr int I2C_MASTER_RESETS_BEFORE_FAIL =
  5;  ///< The number of times to try resetting a stuck I2C master before giving
     ///< up
  static constexpr int NUM_FINISHED_CHECKS =
    100;  ///< How many times to poll I2C_SLV4_DONE before giving up and
         ///< resetting

  // Bank 0
  static constexpr hal::byte ICM20X_B0_WHOAMI = 0x00;  ///< Chip ID register
static constexpr hal::byte ICM20X_B0_USER_CTRL =
  0x03;  ///< User Control Reg. Includes I2C Master
static constexpr hal::byte ICM20X_B0_LP_CONFIG = 0x05;  ///< Low Power config
static constexpr hal::byte ICM20X_B0_REG_INT_PIN_CFG =
  0xF;  ///< Interrupt config register
static constexpr hal::byte ICM20X_B0_REG_INT_ENABLE =
  0x10;  ///< Interrupt enable register 0
static constexpr hal::byte ICM20X_B0_REG_INT_ENABLE_1 =
  0x11;  ///< Interrupt enable register 1
static constexpr hal::byte ICM20X_B0_I2C_MST_STATUS =
  0x17;  ///< Records if I2C master bus data is
         ///< finished
static constexpr hal::byte ICM20X_B0_REG_BANK_SEL =
  0x7F;  ///< register bank selection register
static constexpr hal::byte ICM20X_B0_PWR_MGMT_1 =
  0x06;  ///< primary power management register
static constexpr hal::byte ICM20X_B0_ACCEL_XOUT_H =
  0x2D;  ///< first byte of accel data
static constexpr hal::byte ICM20X_B0_GYRO_XOUT_H =
  0x33;  ///< first byte of accel data

// Bank 2
static constexpr hal::byte ICM20X_B2_GYRO_SMPLRT_DIV =
  0x00;  ///< Gyroscope data rate divisor
static constexpr hal::byte ICM20X_B2_GYRO_CONFIG_1 =
  0x01;  ///< Gyro config for range setting
static constexpr hal::byte ICM20X_B2_ACCEL_SMPLRT_DIV_1 =
  0x10;  ///< Accel data rate divisor MSByte
static constexpr hal::byte ICM20X_B2_ACCEL_SMPLRT_DIV_2 =
  0x11;  ///< Accel data rate divisor LSByte
static constexpr hal::byte ICM20X_B2_ACCEL_CONFIG_1 =
  0x14;  ///< Accel config for setting range

// Bank 3
static constexpr hal::byte ICM20X_B3_I2C_MST_ODR_CONFIG =
  0x0;  ///< Sets ODR for I2C master bus
static constexpr hal::byte ICM20X_B3_I2C_MST_CTRL =
  0x1;  ///< I2C master bus config
static constexpr hal::byte ICM20X_B3_I2C_MST_DELAY_CTRL =
  0x2;  ///< I2C master bus config
static constexpr hal::byte ICM20X_B3_I2C_SLV0_ADDR =
  0x3;  ///< Sets I2C address for I2C master bus slave
        ///< 0
static constexpr hal::byte ICM20X_B3_I2C_SLV0_REG =
  0x4;  ///< Sets register address for I2C master bus
        ///< slave 0
static constexpr hal::byte ICM20X_B3_I2C_SLV0_CTRL =
  0x5;  ///< Controls for I2C master bus slave 0
static constexpr hal::byte ICM20X_B3_I2C_SLV0_DO =
  0x6;  ///< Sets I2C master bus slave 0 data out

static constexpr hal::byte ICM20X_B3_I2C_SLV4_ADDR =
  0x13;  ///< Sets I2C address for I2C master bus slave
         ///< 4
static constexpr hal::byte ICM20X_B3_I2C_SLV4_REG =
  0x14;  ///< Sets register address for I2C master bus
         ///< slave 4
static constexpr hal::byte ICM20X_B3_I2C_SLV4_CTRL =
  0x15;  ///< Controls for I2C master bus slave 4
static constexpr hal::byte ICM20X_B3_I2C_SLV4_DO =
  0x16;  ///< Sets I2C master bus slave 4 data out
static constexpr hal::byte ICM20X_B3_I2C_SLV4_DI =
  0x17;  ///< Sets I2C master bus slave 4 data in

static constexpr hal::byte ICM20948_CHIP_ID =
  0xEA;  ///< ICM20948 default device id from WHOAMI
static constexpr hal::byte ICM20649_CHIP_ID =
  0xE1;  ///< ICM20649 default device id from WHOAMI

/** Options for `enableAccelDLPF` */
typedef enum
{
  ICM20X_ACCEL_FREQ_246_0_HZ = 0x1,
  ICM20X_ACCEL_FREQ_111_4_HZ = 0x2,
  ICM20X_ACCEL_FREQ_50_4_HZ = 0x3,
  ICM20X_ACCEL_FREQ_23_9_HZ = 0x4,
  ICM20X_ACCEL_FREQ_11_5_HZ = 0x5,
  ICM20X_ACCEL_FREQ_5_7_HZ = 0x6,
  ICM20X_ACCEL_FREQ_473_HZ = 0x7,
} icm20x_accel_cutoff_t;

/** Options for `enableGyroDLPF` */
typedef enum
{
  ICM20X_GYRO_FREQ_196_6_HZ = 0x0,
  ICM20X_GYRO_FREQ_151_8_HZ = 0x1,
  ICM20X_GYRO_FREQ_119_5_HZ = 0x2,
  ICM20X_GYRO_FREQ_51_2_HZ = 0x3,
  ICM20X_GYRO_FREQ_23_9_HZ = 0x4,
  ICM20X_GYRO_FREQ_11_6_HZ = 0x5,
  ICM20X_GYRO_FREQ_5_7_HZ = 0x6,
  ICM20X_GYRO_FREQ_361_4_HZ = 0x7,

} icm20x_gyro_cutoff_t;

class icm20xx;

/** libhal Unified Sensor interface for accelerometer component of ICM20X */
class icm20xx_Accelerometer : public accelerometer
{
public:
  icm20xx_Accelerometer(icm20xx* parent)
  {
    _theICM20X = parent;
  }
  hal::result<accelerometer::read_t> read_accel();

private:
  hal::byte _sensorID = 0x20A;
  icm20xx* _theICM20X = NULL;
};

/** libhal Unified Sensor interface for gyro component of ICM20X */
class icm20xx_Gyro
{
public:
  icm20xx_Gyro(icm20xx* parent)
  {
    _theICM20X = parent;
  }
  hal::result<gyroscope::read_t> read_gyro();

private:
  hal::byte _sensorID = 0x20B;
  icm20xx* _theICM20X = NULL;
};

/** libhal Unified Sensor interface for magnetometer component of ICM20X */
class icm20xx_Magnetometer : public magnetometer
{
public:
  icm20xx_Magnetometer(icm20xx* parent)
  {
    _theICM20X = parent;
  }
  hal::result<magnetometer::read_t> read_mag();

private:
  hal::byte _sensorID = 0x20C;
  icm20xx* _theICM20X = NULL;
};

/** libhal Unified Sensor interface for temperature component of ICM20X */
class icm20xx_Temp : public temperature_sensor
{
public:
  icm20xx_Temp(icm20xx* parent)
  {
    _theICM20X = parent;
  }

private:
  result<read_t> driver_read() override;
  hal::byte m_sensorID = 0x20D;
  icm20xx* m_theICM20X = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM20X 6-DoF Accelerometer and Gyro
 */
class icm20xx
{
public:
  icm20xx();
  ~icm20xx();

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  bool enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq);
  bool enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq);

  void reset(void);

  // TODO: bool-ify
  void setInt1ActiveLow(bool active_low);
  void setInt2ActiveLow(bool active_low);

  accelerometer* getAccelerometerSensor(void);
  gyroscope* getGyroSensor(void);
  magnetometer* getMagnetometerSensor(void);
  temperature_sensor* getTemperatureSensor(void);

  bool getEvent(hal::result<accelerometer::read_t>* accel,
                hal::result<gyroscope::read_t>* gyro,
                hal::result<temperature_sensor::read_t>* temp,
                hal::result<magnetometer::read_t>* mag = NULL);

  uint8_t readExternalRegister(uint8_t slv_addr, uint8_t reg_addr);
  bool writeExternalRegister(uint8_t slv_addr, uint8_t reg_addr, uint8_t value);
  bool configureI2CMaster(void);
  bool enableI2CMaster(bool enable_i2c_master);
  void resetI2CMaster(void);
  void setI2CBypass(bool bypass_i2c);

protected:
  float temperature,  ///< Last reading's temperature (C)
    accX,             ///< Last reading's accelerometer X axis m/s^2
    accY,             ///< Last reading's accelerometer Y axis m/s^2
    accZ,             ///< Last reading's accelerometer Z axis m/s^2
    gyroX,            ///< Last reading's gyro X axis in rad/s
    gyroY,            ///< Last reading's gyro Y axis in rad/s
    gyroZ,            ///< Last reading's gyro Z axis in rad/s
    magX,             ///< Last reading's mag X axis in rad/s
    magY,             ///< Last reading's mag Y axis in rad/s
    magZ;             ///< Last reading's mag Z axis in rad/s

  hal::i2c* i2c_dev = NULL;  ///< Pointer to I2C bus interface

  icm20xx_Accelerometer* accel_sensor = NULL;  ///< Accelerometer data object
  icm20xx_Gyro* gyro_sensor = NULL;            ///< Gyro data object
  icm20xx_Magnetometer* mag_sensor = NULL;  ///< Magnetometer sensor data object
  icm20xx_Temp* temp_sensor = NULL;         ///< Temp sensor data object
  uint16_t _sensorid_accel,                 ///< ID number for accelerometer
    _sensorid_gyro,                         ///< ID number for gyro
    _sensorid_mag,                          ///< ID number for mag
    _sensorid_temp;                         ///< ID number for temperature

  void _read(void);
  virtual void scaleValues(void);
  virtual bool begin_I2C(uint8_t i2c_add, hal::i2c* wire, int32_t sensor_id);
  bool _init(int32_t sensor_id);
  int16_t rawAccX,  ///< temp variables
    rawAccY,        ///< temp variables
    rawAccZ,        ///< temp variables
    rawTemp,        ///< temp variables
    rawGyroX,       ///< temp variables
    rawGyroY,       ///< temp variables
    rawGyroZ,       ///< temp variables
    rawMagX,        ///< temp variables
    rawMagY,        ///< temp variables
    rawMagZ;        ///< temp variables

  uint8_t current_accel_range;  ///< accelerometer range cache
  uint8_t current_gyro_range;   ///< gyro range cache
  // virtual void _setBank(uint8_t bank_number);
  void _setBank(uint8_t bank_number);

  uint8_t readAccelRange(void);
  void writeAccelRange(uint8_t new_accel_range);

  uint8_t readGyroRange(void);
  void writeGyroRange(uint8_t new_gyro_range);

private:
  friend class icm20xx_Accelerometer;  ///< Gives access to private
                                       ///< members to Accelerometer
                                       ///< data object
  friend class icm20xx_Gyro;           ///< Gives access to private members to
                                       ///< Gyro data object
  friend class icm20xx_Magnetometer;   ///< Gives access to private
                                       ///< members to Magnetometer data
                                       ///< object

  friend class icm20xx_Temp;  ///< Gives access to private members to
                              ///< Temp data object

  void fillAccelEvent(hal::result<accelerometer::read_t> *accel, uint32_t timestamp);
  void fillGyroEvent(hal::result<gyroscope::read_t> *gyro, uint32_t timestamp);
  void fillTempEvent(hal::result<temperature_sensor::read_t> *temp, uint32_t timestamp);
  void fillMagEvent(hal::result<magnetometer::read_t> *mag, uint32_t timestamp);
  uint8_t auxillaryRegisterTransaction(bool read, uint8_t slv_addr,
                                       uint8_t reg_addr, uint8_t value = -1);
};

}  // namespace hal::icm