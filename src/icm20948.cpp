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

/************ Basic Settings ************/

hal::status icm20948::init()
{
  currentBank = 0;
  reset_ICM20948();
  if (HAL_CHECK(whoAmI()) != ICM20948_WHO_AM_I_CONTENT) {
    return hal::new_error();
  }

  accOffsetVal.x = 0.0;
  accOffsetVal.y = 0.0;
  accOffsetVal.z = 0.0;
  accCorrFactor.x = 1.0;
  accCorrFactor.y = 1.0;
  accCorrFactor.z = 1.0;
  accRangeFactor = 1.0;
  gyrOffsetVal.x = 0.0;
  gyrOffsetVal.y = 0.0;
  gyrOffsetVal.z = 0.0;
  gyrRangeFactor = 1.0;

  sleep(false);
  enableAcc(true);
  enableGyr(true);

  writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  return hal::success();
}

hal::status icm20948::autoOffsets()
{

  setGyrDLPF(ICM20948_DLPF_6);           // lowest noise
  setGyrRange(ICM20948_GYRO_RANGE_250);  // highest resolution
  setAccRange(ICM20948_ACC_RANGE_2G);
  setAccDLPF(ICM20948_DLPF_6);
  setTempDLPF(ICM20948_DLPF_6);

  setMagOpMode(AK09916_CONT_MODE_20HZ);  // For Mag

  return hal::success();
}

hal::status icm20948::setAccOffsets(float xMin,
                                    float xMax,
                                    float yMin,
                                    float yMax,
                                    float zMin,
                                    float zMax)
{
  accOffsetVal.x = (xMax + xMin) * 0.5;
  accOffsetVal.y = (yMax + yMin) * 0.5;
  accOffsetVal.z = (zMax + zMin) * 0.5;
  accCorrFactor.x = (xMax + abs(xMin)) / 32768.0;
  accCorrFactor.y = (yMax + abs(yMin)) / 32768.0;
  accCorrFactor.z = (zMax + abs(zMin)) / 32768.0;

  return hal::success();
}

hal::status icm20948::setGyrOffsets(float xOffset, float yOffset, float zOffset)
{
  gyrOffsetVal.x = xOffset;
  gyrOffsetVal.y = yOffset;
  gyrOffsetVal.z = zOffset;

  return hal::success();
}

hal::result<hal::byte> icm20948::whoAmI()
{
  return HAL_CHECK(readRegister8(0, ICM20948_WHO_AM_I));
}

hal::result<hal::byte> icm20948::sleep_check()
{
  return HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_1));
}

hal::result<hal::byte> icm20948::accel_check()
{
  return HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_2));
}

hal::status icm20948::enableAcc(bool enAcc)
{
  regVal = HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_2));

  if (enAcc) {
    regVal &= ~ICM20948_ACC_EN;
  } else {
    regVal |= ICM20948_ACC_EN;
  }

  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_2, regVal));

  return hal::success();
}

hal::status icm20948::setAccRange(ICM20948_accRange accRange)
{
  regVal = HAL_CHECK(readRegister8(2, ICM20948_ACCEL_CONFIG));
  regVal &= ~(0x06);
  regVal |= (accRange << 1);
  HAL_CHECK(writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal));

  return hal::success();
}

hal::status icm20948::setAccDLPF(ICM20948_dlpf dlpf)
{
  regVal = HAL_CHECK(readRegister8(2, ICM20948_ACCEL_CONFIG));

  if (dlpf == ICM20948_DLPF_OFF) {
    regVal &= 0xFE;
    HAL_CHECK(writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal));
    return hal::success();
  } else {
    regVal |= 0x01;
    regVal &= 0xC7;
    regVal |= (dlpf << 3);
  }
  HAL_CHECK(writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal));

  return hal::success();
}

hal::status icm20948::setAccSampleRateDivider(uint16_t accSplRateDiv)
{
  HAL_CHECK(writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv));
  return hal::success();
}

hal::status icm20948::enableGyr(bool enGyr)
{
  regVal = HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_2));
  if (enGyr) {
    regVal &= ~ICM20948_GYR_EN;
  } else {
    regVal |= ICM20948_GYR_EN;
  }
  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_2, regVal));

  return hal::success();
}

hal::status icm20948::setGyrRange(ICM20948_gyroRange gyroRange)
{
  regVal = HAL_CHECK(readRegister8(2, ICM20948_GYRO_CONFIG_1));
  regVal &= ~(0x06);
  regVal |= (gyroRange << 1);
  HAL_CHECK(writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal));

  return hal::success();
}

hal::status icm20948::setGyrDLPF(ICM20948_dlpf dlpf)
{
  regVal = HAL_CHECK(readRegister8(2, ICM20948_GYRO_CONFIG_1));

  if (dlpf == ICM20948_DLPF_OFF) {
    regVal &= 0xFE;
    HAL_CHECK(writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal));
    return hal::success();
  } else {
    regVal |= 0x01;
    regVal &= 0xC7;
    regVal |= (dlpf << 3);
  }
  HAL_CHECK(writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal));

  return hal::success();
}

hal::status icm20948::setGyrSampleRateDivider(hal::byte gyrSplRateDiv)
{
  HAL_CHECK(writeRegister8(2, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv));
  return hal::success();
}

hal::status icm20948::setTempDLPF(ICM20948_dlpf dlpf)
{
  HAL_CHECK(writeRegister8(2, ICM20948_TEMP_CONFIG, dlpf));
  return hal::success();
}

/********* Power, Sleep, Standby *********/

hal::status icm20948::enableCycle(ICM20948_cycle cycle)
{
  regVal = HAL_CHECK(readRegister8(0, ICM20948_LP_CONFIG));
  regVal &= 0x0F;
  regVal |= cycle;

  HAL_CHECK(writeRegister8(0, ICM20948_LP_CONFIG, regVal));
  return hal::success();
}

hal::status icm20948::enableLowPower(bool enLP)
{
  regVal = HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_1));
  if (enLP) {
    regVal |= ICM20948_LP_EN;
  } else {
    regVal &= ~ICM20948_LP_EN;
  }
  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_1, regVal));
  return hal::success();
}

hal::status icm20948::setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg)
{
  HAL_CHECK(writeRegister8(2, ICM20948_GYRO_CONFIG_2, avg));
  return hal::success();
}

hal::status icm20948::setAccAverageInCycleMode(ICM20948_accAvgLowPower avg)
{
  HAL_CHECK(writeRegister8(2, ICM20948_ACCEL_CONFIG_2, avg));
  return hal::success();
}

hal::status icm20948::sleep(bool sleep)
{

  if (sleep) {
    regVal |= ICM20948_SLEEP;
  } else {
    regVal &= ~ICM20948_SLEEP;
  }
  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_1, regVal));
  return hal::success();
}

/************** Magnetometer **************/

hal::status icm20948::initMagnetometer()
{
  enableI2CMaster();
  resetMag();
  // reset_ICM20948();
  // sleep(false);
  // writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  enableI2CMaster();

  // auto whoAmI = HAL_CHECK(whoAmIMag());
  // if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
  //   return hal::new_error();
  // }

  return hal::success();
}

hal::result<hal::byte> icm20948::whoAmIMag()
{
  // return HAL_CHECK(readRegister8(0, AK09916_WIA_2));
  return HAL_CHECK(readAK09916Register16(AK09916_WIA_2));
}

void icm20948::setMagOpMode(AK09916_opMode opMode)
{
  writeAK09916Register8(AK09916_CNTL_2, opMode);
  if (opMode != AK09916_PWR_DOWN) {
    enableMagDataRead(AK09916_HXL, 0x08);
  }
}

void icm20948::resetMag()
{
  writeAK09916Register8(AK09916_CNTL_3, 0x01);
}

/************************************************
     Private Functions
*************************************************/

hal::status icm20948::setClockToAutoSelect()
{
  regVal = HAL_CHECK(readRegister8(0, ICM20948_PWR_MGMT_1));
  regVal |= 0x01;
  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_1, regVal));
  return hal::success();
}

hal::status icm20948::switchBank(hal::byte newBank)
{
  if (newBank != currentBank) {
    currentBank = newBank;
    currentBank = currentBank << 4;
  }
  auto reg_buffer = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_REG_BANK_SEL },
                            hal::never_timeout()));

  hal::byte reg_val = reg_buffer[0];
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ reg_val, currentBank },
                       hal::never_timeout()));

  return hal::success();
}

hal::status icm20948::writeRegister8(hal::byte bank,
                                     hal::byte p_reg_addr,
                                     hal::byte p_val)
{
  switchBank(bank);
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 2>{ p_reg_addr, p_val },
                       hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::writeRegister16(hal::byte bank,
                                      hal::byte reg,
                                      int16_t p_val)
{
  switchBank(bank);
  int8_t MSByte = static_cast<int8_t>((p_val >> 8) & 0xFF);
  hal::byte LSByte = p_val & 0xFF;

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array<hal::byte, 3>{ reg, MSByte, LSByte },
                       hal::never_timeout()));

  return hal::success();
}

hal::result<hal::byte> icm20948::readRegister8(hal::byte bank,
                                               hal::byte read_reg)
{
  switchBank(bank);
  auto ctrl_buffer =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array<hal::byte, 1>{ read_reg },
                                      hal::never_timeout()));
  return ctrl_buffer[0];
}

hal::result<hal::byte> icm20948::readRegister16(hal::byte bank, hal::byte reg)
{

  switchBank(bank);
  // hal::byte MSByte = 0, LSByte = 0;
  hal::byte reg16Val = 0;

  auto MSByte = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, std::array<hal::byte, 1>{ reg }, hal::never_timeout()));
  auto LSByte = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, std::array<hal::byte, 1>{ reg }, hal::never_timeout()));

  reg16Val = (MSByte[0] << 8) + LSByte[1];
  return reg16Val;
}

hal::status icm20948::readAllData(std::array<hal::byte, 20>& data)
{
  switchBank(0);
  data = HAL_CHECK(
    hal::write_then_read<20>(*m_i2c,
                             m_address,
                             std::array<hal::byte, 1>{ ICM20948_ACCEL_OUT },
                             hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::writeAK09916Register8(hal::byte reg, hal::byte val)
{

  writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);  // write AK09916
  writeRegister8(
    3, ICM20948_I2C_SLV0_REG, reg);  // define AK09916 register to be written to
  writeRegister8(3, ICM20948_I2C_SLV0_DO, val);

  return hal::success();
}

hal::result<hal::byte> icm20948::readAK09916Register8(hal::byte reg)
{
  enableMagDataRead(reg, 0x01);
  enableMagDataRead(AK09916_HXL, 0x08);
  regVal = HAL_CHECK(readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00));
  return regVal;
}

hal::result<int16_t> icm20948::readAK09916Register16(hal::byte reg)
{
  int16_t regValue = 0;
  enableMagDataRead(reg, 0x02);
  regValue = HAL_CHECK(readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00));
  enableMagDataRead(AK09916_HXL, 0x08);
  return regValue;
}

hal::status icm20948::reset_ICM20948()
{
  HAL_CHECK(writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET));
  return hal::success();
}

hal::status icm20948::enableI2CMaster()
{
  HAL_CHECK(writeRegister8(
    0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN));  // enable I2C master
  HAL_CHECK(writeRegister8(
    3, ICM20948_I2C_MST_CTRL, 0x07));  // set I2C clock to 345.60 kHz

  return hal::success();
}

hal::status icm20948::enableMagDataRead(hal::byte reg, hal::byte bytes)
{
  HAL_CHECK(writeRegister8(3,
                           ICM20948_I2C_SLV0_ADDR,
                           AK09916_ADDRESS | AK09916_READ));  // read AK09916
  HAL_CHECK(writeRegister8(
    3, ICM20948_I2C_SLV0_REG, reg));  // define AK09916 register to be read
  HAL_CHECK(writeRegister8(
    3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes));  // enable read | number of byte

  return hal::success();
}


// =====================================================================
// Restructuring code for reading accel, gyro, and temp
// =====================================================================

hal::result<icm20948::accel_read_t> icm20948::read_acceleration()
{
  std::array<hal::byte, 6> data{};
  accel_read_t accel_read, accel_read_raw;
  switchBank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_ACCEL_OUT },
                            hal::never_timeout()));
  // auto accel_read = getGValues();

  accel_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  accel_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  accel_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  accel_read.x =
    (accel_read.x - (accOffsetVal.x / accRangeFactor)) / accCorrFactor.x;
  accel_read.y =
    (accel_read.y - (accOffsetVal.y / accRangeFactor)) / accCorrFactor.y;
  accel_read.z =
    (accel_read.z - (accOffsetVal.z / accRangeFactor)) / accCorrFactor.z;

  accel_read.x = accel_read_raw.x * accRangeFactor / 16384.0;
  accel_read.y = accel_read_raw.y * accRangeFactor / 16384.0;
  accel_read.z = accel_read_raw.z * accRangeFactor / 16384.0;

  // reset data buffer to 0
  return accel_read;
}



hal::result<icm20948::gyro_read_t> icm20948::read_gyroscope()
{
  std::array<hal::byte, 6> data{};
  gyro_read_t gyro_read, gyro_read_raw;

  switchBank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            m_address,
                            std::array<hal::byte, 1>{ ICM20948_GYRO_OUT },
                            hal::never_timeout()));

  gyro_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  gyro_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  gyro_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  gyro_read.x -= (gyrOffsetVal.x / gyrRangeFactor);
  gyro_read.y -= (gyrOffsetVal.y / gyrRangeFactor);
  gyro_read.z -= (gyrOffsetVal.z / gyrRangeFactor);

  gyro_read.x = gyro_read_raw.x * gyrRangeFactor * 250.0 / 32768.0;
  gyro_read.y = gyro_read_raw.y * gyrRangeFactor * 250.0 / 32768.0;
  gyro_read.z = gyro_read_raw.z * gyrRangeFactor * 250.0 / 32768.0;

  return gyro_read;
}



hal::result<icm20948::mag_read_t> icm20948::read_magnetometer()
{
  // readAK09916Register16(ICM20948_EXT_SLV_SENS_DATA_00);
  std::array<hal::byte, 6> data{};
  int16_t x,y,z;
  mag_read_t mag_read;

  switchBank(0);
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

  switchBank(0);
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

}  // namespace hal::icm