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

#include <libhal-icm/xyzFloat.hpp>
#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::icm {

class icm20948
{

public:
  struct accel_read_t
  {
    float x;
    float y;
    float z;
  };

  struct gyro_read_t
  {
    float x;
    float y;
    float z;
  };

  struct mag_read_t
  {
    float x;
    float y;
    float z;
  };

  struct temp_read_t
  {
    float temp;
  };


  /**
   * @brief Read acceleration data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform acceleration conversion to g.
   */
  [[nodiscard]] hal::result<accel_read_t> read_acceleration();

  /**
   * @brief Read gyroscope data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform gyroscope conversion to rad/s.
   */
  [[nodiscard]] hal::result<gyro_read_t> read_gyroscope();

  /**
   * @brief Read magnetometer data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform magnetometer conversion to uT.
   */
  [[nodiscard]] hal::result<mag_read_t> read_magnetometer();


  /**
   * @brief Read pressure data from out_t_msb_r and out_t_lsb_r
   *        and perform temperature conversion to celsius.
   */
  [[nodiscard]] hal::result<temp_read_t> read_temperature();



  static result<icm20948> create(hal::i2c& p_i2c, hal::byte p_device_address);

  typedef enum ICM20948_CYCLE
  {
    ICM20948_NO_CYCLE = 0x00,
    ICM20948_GYR_CYCLE = 0x10,
    ICM20948_ACC_CYCLE = 0x20,
    ICM20948_ACC_GYR_CYCLE = 0x30,
    ICM20948_ACC_GYR_I2C_MST_CYCLE = 0x70
  } ICM20948_cycle;

  typedef enum ICM20948_INT_PIN_POL
  {
    ICM20948_ACT_HIGH,
    ICM20948_ACT_LOW
  } ICM20948_intPinPol;

  typedef enum ICM20948_INT_TYPE
  {
    ICM20948_FSYNC_INT = 0x01,
    ICM20948_WOM_INT = 0x02,
    ICM20948_DMP_INT = 0x04,
    ICM20948_DATA_READY_INT = 0x08,
    ICM20948_FIFO_OVF_INT = 0x10,
    ICM20948_FIFO_WM_INT = 0x20
  } ICM20948_intType;

  typedef enum ICM20948_FIFO_TYPE
  {
    ICM20948_FIFO_ACC = 0x10,
    ICM20948_FIFO_GYR = 0x0E,
    ICM20948_FIFO_ACC_GYR = 0x1E
  } ICM20948_fifoType;

  typedef enum ICM20948_FIFO_MODE_CHOICE
  {
    ICM20948_CONTINUOUS,
    ICM20948_STOP_WHEN_FULL
  } ICM20948_fifoMode;

  typedef enum ICM20948_GYRO_RANGE
  {
    ICM20948_GYRO_RANGE_250,
    ICM20948_GYRO_RANGE_500,
    ICM20948_GYRO_RANGE_1000,
    ICM20948_GYRO_RANGE_2000
  } ICM20948_gyroRange;

  typedef enum ICM20948_DLPF
  {
    ICM20948_DLPF_0,
    ICM20948_DLPF_1,
    ICM20948_DLPF_2,
    ICM20948_DLPF_3,
    ICM20948_DLPF_4,
    ICM20948_DLPF_5,
    ICM20948_DLPF_6,
    ICM20948_DLPF_7,
    ICM20948_DLPF_OFF
  } ICM20948_dlpf;

  typedef enum ICM20948_GYRO_AVG_LOW_PWR
  {
    ICM20948_GYR_AVG_1,
    ICM20948_GYR_AVG_2,
    ICM20948_GYR_AVG_4,
    ICM20948_GYR_AVG_8,
    ICM20948_GYR_AVG_16,
    ICM20948_GYR_AVG_32,
    ICM20948_GYR_AVG_64,
    ICM20948_GYR_AVG_128
  } ICM20948_gyroAvgLowPower;

  typedef enum ICM20948_ACC_RANGE
  {
    ICM20948_ACC_RANGE_2G,
    ICM20948_ACC_RANGE_4G,
    ICM20948_ACC_RANGE_8G,
    ICM20948_ACC_RANGE_16G
  } ICM20948_accRange;

  typedef enum ICM20948_ACC_AVG_LOW_PWR
  {
    ICM20948_ACC_AVG_4,
    ICM20948_ACC_AVG_8,
    ICM20948_ACC_AVG_16,
    ICM20948_ACC_AVG_32
  } ICM20948_accAvgLowPower;

  typedef enum ICM20948_WOM_COMP
  {
    ICM20948_WOM_COMP_DISABLE,
    ICM20948_WOM_COMP_ENABLE
  } ICM20948_womCompEn;

  typedef enum AK09916_OP_MODE
  {
    AK09916_PWR_DOWN = 0x00,
    AK09916_TRIGGER_MODE = 0x01,
    AK09916_CONT_MODE_10HZ = 0x02,
    AK09916_CONT_MODE_20HZ = 0x04,
    AK09916_CONT_MODE_50HZ = 0x06,
    AK09916_CONT_MODE_100HZ = 0x08
  } AK09916_opMode;

  typedef enum ICM20948_ORIENTATION
  {
    ICM20948_FLAT,
    ICM20948_FLAT_1,
    ICM20948_XY,
    ICM20948_XY_1,
    ICM20948_YX,
    ICM20948_YX_1
  } ICM20948_orientation;

  /* Basic settings */

  hal::status init();
  hal::status defaultSetup();
  hal::status autoOffsets();
  hal::status setAccOffsets(float xMin,
                            float xMax,
                            float yMin,
                            float yMax,
                            float zMin,
                            float zMax);
  hal::status setGyrOffsets(float xOffset, float yOffset, float zOffset);
  hal::result<hal::byte> whoAmI();

  // Delete Later
  hal::result<hal::byte> sleep_check();
  hal::result<hal::byte> accel_check();

  hal::status enableAcc(bool enAcc);
  hal::status setAccRange(ICM20948_accRange accRange);
  hal::status setAccDLPF(ICM20948_dlpf dlpf);
  hal::status setAccSampleRateDivider(uint16_t accSplRateDiv);
  hal::status enableGyr(bool enGyr);
  hal::status setGyrRange(ICM20948_gyroRange gyroRange);
  hal::status setGyrDLPF(ICM20948_dlpf dlpf);
  hal::status setGyrSampleRateDivider(hal::byte gyrSplRateDiv);
  hal::status setTempDLPF(ICM20948_dlpf dlpf);

  /* x,y,z results */

  hal::status readSensor();
  xyzFloat getAccRawValues();
  xyzFloat getCorrectedAccRawValues();
  xyzFloat getGValues();
  xyzFloat getAccRawValuesFromFifo();
  xyzFloat getCorrectedAccRawValuesFromFifo();
  xyzFloat getGValuesFromFifo();
  float getResultantG(xyzFloat gVal);
  float getTemperature();
  xyzFloat getGyrRawValues();
  xyzFloat getCorrectedGyrRawValues();
  xyzFloat getGyrValues();
  xyzFloat getGyrValuesFromFifo();
  xyzFloat getMagValues();

  /* Power, Sleep, Standby */

  hal::status enableCycle(ICM20948_cycle cycle);
  hal::status enableLowPower(bool enLP);
  hal::status setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg);
  hal::status setAccAverageInCycleMode(ICM20948_accAvgLowPower avg);
  hal::status sleep(bool sleep);

  /* Angles and Orientation */

  // xyzFloat getAngles();
  // ICM20948_orientation getOrientation();
  // std::string getOrientationAsString();
  // float getPitch();
  // float getRoll();

  /* Magnetometer */

  hal::status initMagnetometer();
  hal::result<hal::byte> whoAmIMag();
  void setMagOpMode(AK09916_opMode opMode);
  void resetMag();

private:
  hal::i2c* m_i2c;
  hal::byte m_address;
  hal::byte m_gscale = 0x00;

  hal::byte currentBank;
  std::array<hal::byte, 20> m_read_all_buffer{};
  std::array<hal::byte, 6> data{};
  xyzFloat accOffsetVal;
  xyzFloat accCorrFactor;
  xyzFloat gyrOffsetVal;
  hal::byte accRangeFactor;
  hal::byte gyrRangeFactor;
  hal::byte regVal;  // intermediate storage of register values

  explicit icm20948(hal::i2c& p_i2c, hal::byte p_device_address)
    : m_i2c(&p_i2c)
    , m_address(p_device_address)
  {
  }

  hal::status setClockToAutoSelect();
  xyzFloat correctAccRawValues(xyzFloat accRawVal);
  xyzFloat correctGyrRawValues(xyzFloat gyrRawVal);
  hal::status switchBank(hal::byte newBank);
  hal::status writeRegister8(hal::byte bank, hal::byte reg, hal::byte val);
  hal::status writeRegister16(hal::byte bank, hal::byte reg, int16_t val);

  hal::result<hal::byte> readRegister8(hal::byte bank, hal::byte reg);
  hal::result<hal::byte> readRegister16(hal::byte bank, hal::byte reg);

  hal::status readAllData(std::array<hal::byte, 20>& data);
  hal::status writeAK09916Register8(hal::byte reg, hal::byte val);
  hal::result<hal::byte> readAK09916Register8(hal::byte reg);
  hal::result<hal::byte> readAK09916Register16(hal::byte reg);

  hal::status reset_ICM20948();
  hal::status enableI2CMaster();

  hal::status enableMagDataRead(hal::byte reg, hal::byte bytes);
};

}  // namespace hal::icm