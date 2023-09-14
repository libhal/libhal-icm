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

#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::icm {

class icm20948
{

public:

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



  [[nodiscard]] static result<icm20948> create(hal::i2c& p_i2c, hal::byte p_device_address);

  hal::status init();
  hal::status auto_offsets();
  hal::status set_acc_offsets(float p_xMin,
                            float p_xMax,
                            float p_yMin,
                            float p_yMax,
                            float p_zMin,
                            float p_zMax);
  hal::status set_gyr_offsets(float p_xOffset, float p_yOffset, float p_zOffset);
  hal::result<hal::byte> whoami();

  hal::status enable_acc(bool p_enAcc);
  hal::status set_acc_range(ICM20948_accRange p_accRange);
  hal::status set_acc_DLPF(ICM20948_dlpf p_dlpf);
  hal::status set_acc_sample_rate_div(uint16_t p_accSplRateDiv);
  hal::status enable_gyr(bool p_enGyr);
  hal::status set_gyr_range(ICM20948_gyroRange gyroRange);
  hal::status set_gyr_DLPF(ICM20948_dlpf p_dlpf);
  hal::status set_gyr_sample_rate_div(hal::byte p_gyrSplRateDiv);
  hal::status set_temp_DLPF(ICM20948_dlpf p_dlpf);

  /* Power, Sleep, Standby */
  hal::status enable_cycle(ICM20948_cycle p_cycle);
  hal::status enable_low_power(bool p_enLP);
  hal::status set_gyr_Averg_cycle_mode(ICM20948_gyroAvgLowPower p_avg);
  hal::status set_acc_Averg_cycle_mode(ICM20948_accAvgLowPower p_avg);
  hal::status sleep(bool p_sleep);

  /* Magnetometer */
  hal::status init_mag();
  [[nodiscard]] hal::result<hal::byte> whoami_mag();
  void set_mag_op_mode(AK09916_opMode p_opMode);
  void reset_mag();

private:
  hal::i2c* m_i2c;
  hal::byte m_address;
  hal::byte m_gscale = 0x00;
  hal::byte m_currentBank;
  accel_read_t m_accOffsetVal;
  accel_read_t m_accCorrFactor;
  gyro_read_t m_gyrOffsetVal;
  hal::byte m_accRangeFactor;
  hal::byte m_gyrRangeFactor;
  hal::byte m_regVal;  // intermediate storage of register values

  explicit icm20948(hal::i2c& p_i2c, hal::byte p_device_address)
    : m_i2c(&p_i2c)
    , m_address(p_device_address)
  {
  }

  hal::status set_clock_auto_select();
  hal::status switch_bank(hal::byte p_newBank);
  hal::status write_register8(hal::byte p_bank, hal::byte p_reg, hal::byte p_val);
  hal::status write_register16(hal::byte p_bank, hal::byte reg, int16_t p_val);

  [[nodiscard]] hal::result<hal::byte> read_register8(hal::byte p_bank, hal::byte p_reg);
  [[nodiscard]] hal::result<hal::byte> read_register16(hal::byte p_bank, hal::byte p_reg);

  hal::status write_AK09916_register8(hal::byte reg, hal::byte p_val);
  [[nodiscard]] hal::result<hal::byte> read_AK09916_register8(hal::byte p_reg);
  [[nodiscard]] hal::result<int16_t> read_AK09916_register16(hal::byte p_reg);

  hal::status reset_icm20948();
  hal::status enable_i2c_host();

  hal::status enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes);
};

}  // namespace hal::icm