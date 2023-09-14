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


#include "icm20948_reg.hpp"
#include <cmath>
#include <array>
#include <libhal-icm/icm20948.hpp>
#include <libhal-util/i2c.hpp>

namespace hal::icm {
using namespace std::literals;

result<icm20948> icm20948::create(hal::i2c& p_i2c,
                                  hal::byte p_device_address = ICM20948_ADDRESS)
{
  icm20948 icm(p_i2c, p_device_address);

  HAL_CHECK(icm.init());

  return icm;
}

hal::status icm20948::init()
{
  m_currentBank = 0;
  reset_icm20948();
  if (HAL_CHECK(whoami()) != ICM20948_WHO_AM_I_CONTENT) {
    return hal::new_error();
  }

  m_accOffsetVal.x = 0.0;
  m_accOffsetVal.y = 0.0;
  m_accOffsetVal.z = 0.0;
  m_accCorrFactor.x = 1.0;
  m_accCorrFactor.y = 1.0;
  m_accCorrFactor.z = 1.0;
  m_accRangeFactor = 1.0;
  m_gyrOffsetVal.x = 0.0;
  m_gyrOffsetVal.y = 0.0;
  m_gyrOffsetVal.z = 0.0;
  m_gyrRangeFactor = 1.0;

  sleep(false);
  enable_acc(true);
  enable_gyr(true);

  write_register8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  return hal::success();
}

hal::status icm20948::auto_offsets()
{

  set_gyr_DLPF(ICM20948_DLPF_6);           // lowest noise
  set_gyr_range(ICM20948_GYRO_RANGE_250);  // highest resolution
  set_acc_range(ICM20948_ACC_RANGE_2G);
  set_acc_DLPF(ICM20948_DLPF_6);
  set_temp_DLPF(ICM20948_DLPF_6);

  set_mag_op_mode(AK09916_CONT_MODE_20HZ);  // For Mag

  return hal::success();
}

hal::status icm20948::set_acc_offsets(float p_xMin,
                                    float p_xMax,
                                    float p_yMin,
                                    float p_yMax,
                                    float p_zMin,
                                    float p_zMax)
{
  m_accOffsetVal.x = (p_xMax + p_xMin) * 0.5;
  m_accOffsetVal.y = (p_yMax + p_yMin) * 0.5;
  m_accOffsetVal.z = (p_zMax + p_zMin) * 0.5;
  m_accCorrFactor.x = (p_xMax + abs(p_xMin)) / 32768.0;
  m_accCorrFactor.y = (p_yMax + abs(p_yMin)) / 32768.0;
  m_accCorrFactor.z = (p_zMax + abs(p_zMin)) / 32768.0;

  return hal::success();
}

hal::status icm20948::set_gyr_offsets(float p_xOffset, float p_yOffset, float p_zOffset)
{
  m_gyrOffsetVal.x = p_xOffset;
  m_gyrOffsetVal.y = p_yOffset;
  m_gyrOffsetVal.z = p_zOffset;

  return hal::success();
}

hal::result<hal::byte> icm20948::whoami()
{
  return HAL_CHECK(read_register8(0, ICM20948_WHO_AM_I));
}

hal::status icm20948::enable_acc(bool p_enAcc)
{
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_PWR_MGMT_2));

  if (p_enAcc) {
    m_regVal &= ~ICM20948_ACC_EN;
  } else {
    m_regVal |= ICM20948_ACC_EN;
  }

  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_2, m_regVal));

  return hal::success();
}

hal::status icm20948::set_acc_range(ICM20948_accRange p_accRange)
{
  m_regVal = HAL_CHECK(read_register8(2, ICM20948_ACCEL_CONFIG));
  m_regVal &= ~(0x06);
  m_regVal |= (p_accRange << 1);
  HAL_CHECK(write_register8(2, ICM20948_ACCEL_CONFIG, m_regVal));

  return hal::success();
}

hal::status icm20948::set_acc_DLPF(ICM20948_dlpf p_dlpf)
{
  m_regVal = HAL_CHECK(read_register8(2, ICM20948_ACCEL_CONFIG));

  if (p_dlpf == ICM20948_DLPF_OFF) {
    m_regVal &= 0xFE;
    HAL_CHECK(write_register8(2, ICM20948_ACCEL_CONFIG, m_regVal));
    return hal::success();
  } else {
    m_regVal |= 0x01;
    m_regVal &= 0xC7;
    m_regVal |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, ICM20948_ACCEL_CONFIG, m_regVal));

  return hal::success();
}

hal::status icm20948::set_acc_sample_rate_div(uint16_t p_accSplRateDiv)
{
  HAL_CHECK(write_register16(2, ICM20948_ACCEL_SMPLRT_DIV_1, p_accSplRateDiv));
  return hal::success();
}

hal::status icm20948::enable_gyr(bool p_enGyr)
{
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_PWR_MGMT_2));
  if (p_enGyr) {
    m_regVal &= ~ICM20948_GYR_EN;
  } else {
    m_regVal |= ICM20948_GYR_EN;
  }
  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_2, m_regVal));

  return hal::success();
}

hal::status icm20948::set_gyr_range(ICM20948_gyroRange p_gyroRange)
{
  m_regVal = HAL_CHECK(read_register8(2, ICM20948_GYRO_CONFIG_1));
  m_regVal &= ~(0x06);
  m_regVal |= (p_gyroRange << 1);
  HAL_CHECK(write_register8(2, ICM20948_GYRO_CONFIG_1, m_regVal));

  return hal::success();
}

hal::status icm20948::set_gyr_DLPF(ICM20948_dlpf p_dlpf)
{
  m_regVal = HAL_CHECK(read_register8(2, ICM20948_GYRO_CONFIG_1));

  if (p_dlpf == ICM20948_DLPF_OFF) {
    m_regVal &= 0xFE;
    HAL_CHECK(write_register8(2, ICM20948_GYRO_CONFIG_1, m_regVal));
    return hal::success();
  } else {
    m_regVal |= 0x01;
    m_regVal &= 0xC7;
    m_regVal |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, ICM20948_GYRO_CONFIG_1, m_regVal));

  return hal::success();
}

hal::status icm20948::set_gyr_sample_rate_div(hal::byte p_gyrSplRateDiv)
{
  HAL_CHECK(write_register8(2, ICM20948_GYRO_SMPLRT_DIV, p_gyrSplRateDiv));
  return hal::success();
}

hal::status icm20948::set_temp_DLPF(ICM20948_dlpf p_dlpf)
{
  HAL_CHECK(write_register8(2, ICM20948_TEMP_CONFIG, p_dlpf));
  return hal::success();
}

hal::result<icm20948::accel_read_t> icm20948::read_acceleration()
{
  std::array<hal::byte, 6> data{};
  accel_read_t accel_read, accel_read_raw;
  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_ACCEL_OUT },
                            hal::never_timeout()));

  accel_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  accel_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  accel_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  accel_read.x =
    (accel_read.x - (m_accOffsetVal.x / m_accRangeFactor)) / m_accCorrFactor.x;
  accel_read.y =
    (accel_read.y - (m_accOffsetVal.y / m_accRangeFactor)) / m_accCorrFactor.y;
  accel_read.z =
    (accel_read.z - (m_accOffsetVal.z / m_accRangeFactor)) / m_accCorrFactor.z;

  accel_read.x = accel_read_raw.x * m_accRangeFactor / 16384.0;
  accel_read.y = accel_read_raw.y * m_accRangeFactor / 16384.0;
  accel_read.z = accel_read_raw.z * m_accRangeFactor / 16384.0;

  return accel_read;
}

hal::result<icm20948::gyro_read_t> icm20948::read_gyroscope()
{
  std::array<hal::byte, 6> data{};
  gyro_read_t gyro_read, gyro_read_raw;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_GYRO_OUT },
                            hal::never_timeout()));

  gyro_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  gyro_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  gyro_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  gyro_read.x -= (m_gyrOffsetVal.x / m_gyrRangeFactor);
  gyro_read.y -= (m_gyrOffsetVal.y / m_gyrRangeFactor);
  gyro_read.z -= (m_gyrOffsetVal.z / m_gyrRangeFactor);

  gyro_read.x = gyro_read_raw.x * m_gyrRangeFactor * 250.0 / 32768.0;
  gyro_read.y = gyro_read_raw.y * m_gyrRangeFactor * 250.0 / 32768.0;
  gyro_read.z = gyro_read_raw.z * m_gyrRangeFactor * 250.0 / 32768.0;

  return gyro_read;
}



hal::result<icm20948::mag_read_t> icm20948::read_magnetometer()
{
  // read_AK09916_register16(ICM20948_EXT_SLV_SENS_DATA_00);
  std::array<hal::byte, 6> data{};
  int16_t x,y,z;
  mag_read_t mag_read;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_EXT_SLV_SENS_DATA_00 },
                            hal::never_timeout()));

  x = static_cast<int16_t>((data[0]) << 8) | data[1];
  y = static_cast<int16_t>((data[2]) << 8) | data[3];
  z = static_cast<int16_t>((data[4]) << 8) | data[5];

  mag_read.x = x * AK09916_MAG_LSB;
  mag_read.y = y * AK09916_MAG_LSB;
  mag_read.z = z * AK09916_MAG_LSB;

  return mag_read;
}



hal::result<icm20948::temp_read_t> icm20948::read_temperature()
{
  std::array<hal::byte, 2> data{};
  temp_read_t temp_read;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<2>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_TEMP_OUT },
                            hal::never_timeout()));
  int16_t rawTemp = static_cast<int16_t>(((data[0]) << 8) | data[1]);
  temp_read.temp =
    (rawTemp * 1.0 - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0;
  return temp_read;
}

/********* Power, Sleep, Standby *********/

hal::status icm20948::enable_cycle(ICM20948_cycle p_cycle)
{
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_LP_CONFIG));
  m_regVal &= 0x0F;
  m_regVal |= p_cycle;

  HAL_CHECK(write_register8(0, ICM20948_LP_CONFIG, m_regVal));
  return hal::success();
}

hal::status icm20948::enable_low_power(bool p_enLP)
{
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_PWR_MGMT_1));
  if (p_enLP) {
    m_regVal |= ICM20948_LP_EN;
  } else {
    m_regVal &= ~ICM20948_LP_EN;
  }
  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_1, m_regVal));
  return hal::success();
}

hal::status icm20948::set_gyr_Averg_cycle_mode(ICM20948_gyroAvgLowPower p_avg)
{
  HAL_CHECK(write_register8(2, ICM20948_GYRO_CONFIG_2, p_avg));
  return hal::success();
}

hal::status icm20948::set_acc_Averg_cycle_mode(ICM20948_accAvgLowPower p_avg)
{
  HAL_CHECK(write_register8(2, ICM20948_ACCEL_CONFIG_2, p_avg));
  return hal::success();
}

hal::status icm20948::sleep(bool p_sleep)
{

  if (p_sleep) {
    m_regVal |= ICM20948_SLEEP;
  } else {
    m_regVal &= ~ICM20948_SLEEP;
  }
  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_1, m_regVal));
  return hal::success();
}

/************** Magnetometer **************/

hal::status icm20948::init_mag()
{
  enable_i2c_host();
  reset_mag();
  // reset_icm20948();
  // sleep(false);
  // write_register8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  enable_i2c_host();

  // auto whoAmI = HAL_CHECK(whoami_mag());
  // if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
  //   return hal::new_error();
  // }

  return hal::success();
}

hal::result<hal::byte> icm20948::whoami_mag()
{
  return HAL_CHECK(read_AK09916_register16(AK09916_WIA_2));
}

void icm20948::set_mag_op_mode(AK09916_opMode p_opMode)
{
  write_AK09916_register8(AK09916_CNTL_2, p_opMode);
  if (p_opMode != AK09916_PWR_DOWN) {
    enable_mag_data_read(AK09916_HXL, 0x08);
  }
}

void icm20948::reset_mag()
{
  write_AK09916_register8(AK09916_CNTL_3, 0x01);
}

/************************************************
     Private Functions
*************************************************/

hal::status icm20948::set_clock_auto_select()
{
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_PWR_MGMT_1));
  m_regVal |= 0x01;
  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_1, m_regVal));
  return hal::success();
}

hal::status icm20948::switch_bank(hal::byte p_newBank)
{
  if (p_newBank != m_currentBank) {
    m_currentBank = p_newBank;
    m_currentBank = m_currentBank << 4;
  }
  auto reg_buffer = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_REG_BANK_SEL },
                            hal::never_timeout()));

  hal::byte reg_val = reg_buffer[0];
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ reg_val, m_currentBank },
                       hal::never_timeout()));

  return hal::success();
}

hal::status icm20948::write_register8(hal::byte p_bank,
                                     hal::byte p_reg_addr,
                                     hal::byte p_val)
{
  switch_bank(p_bank);
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ p_reg_addr, p_val },
                       hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::write_register16(hal::byte p_bank,
                                      hal::byte p_reg,
                                      int16_t p_val)
{
  switch_bank(p_bank);
  int8_t MSByte = static_cast<int8_t>((p_val >> 8) & 0xFF);
  hal::byte LSByte = p_val & 0xFF;

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 3>{ p_reg, MSByte, LSByte },
                       hal::never_timeout()));

  return hal::success();
}

hal::result<hal::byte> icm20948::read_register8(hal::byte p_bank,
                                               hal::byte p_read_reg)
{
  switch_bank(p_bank);
  auto ctrl_buffer =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array<hal::byte, 1>{ p_read_reg },
                                      hal::never_timeout()));
  return ctrl_buffer[0];
}

hal::result<hal::byte> icm20948::read_register16(hal::byte p_bank, hal::byte p_reg)
{

  switch_bank(p_bank);
  // hal::byte MSByte = 0, LSByte = 0;
  hal::byte reg16Val = 0;

  auto MSByte = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, std::array<hal::byte, 1>{ p_reg }, hal::never_timeout()));
  auto LSByte = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, std::array<hal::byte, 1>{ p_reg }, hal::never_timeout()));

  reg16Val = (MSByte[0] << 8) + LSByte[1];
  return reg16Val;
}

hal::status icm20948::write_AK09916_register8(hal::byte p_reg, hal::byte p_val)
{

  write_register8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);  // write AK09916
  write_register8(
    3, ICM20948_I2C_SLV0_REG, p_reg);  // define AK09916 register to be written to
  write_register8(3, ICM20948_I2C_SLV0_DO, p_val);

  return hal::success();
}

hal::result<hal::byte> icm20948::read_AK09916_register8(hal::byte p_reg)
{
  enable_mag_data_read(p_reg, 0x01);
  enable_mag_data_read(AK09916_HXL, 0x08);
  m_regVal = HAL_CHECK(read_register8(0, ICM20948_EXT_SLV_SENS_DATA_00));
  return m_regVal;
}

hal::result<int16_t> icm20948::read_AK09916_register16(hal::byte p_reg)
{
  int16_t m_regValue = 0;
  enable_mag_data_read(p_reg, 0x02);
  m_regValue = HAL_CHECK(read_register16(0, ICM20948_EXT_SLV_SENS_DATA_00));
  enable_mag_data_read(AK09916_HXL, 0x08);
  return m_regValue;
}

hal::status icm20948::reset_icm20948()
{
  HAL_CHECK(write_register8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET));
  return hal::success();
}

hal::status icm20948::enable_i2c_host()
{
  HAL_CHECK(write_register8(
    0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN));  // enable I2C master
  HAL_CHECK(write_register8(
    3, ICM20948_I2C_MST_CTRL, 0x07));  // set I2C clock to 345.60 kHz

  return hal::success();
}

hal::status icm20948::enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes)
{
  HAL_CHECK(write_register8(3,
                           ICM20948_I2C_SLV0_ADDR,
                           AK09916_ADDRESS | AK09916_READ));  // read AK09916
  HAL_CHECK(write_register8(
    3, ICM20948_I2C_SLV0_REG, p_reg));  // define AK09916 register to be read
  HAL_CHECK(write_register8(
    3, ICM20948_I2C_SLV0_CTRL, 0x80 | p_bytes));  // enable read | number of byte

  return hal::success();
}

}  // namespace hal::icm