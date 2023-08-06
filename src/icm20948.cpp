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


// TODO: We should create the read and write functions in this code to
// encapsulate the read and write functions from libhal to make it easier to
// read and write since we do it very often in this code.
// https://github.com/wollewald/ICM20948_WE/tree/main
// Also need to understand the banks better.  I think we can just use bank 0 for
// everything.

#include "icm20948_reg.hpp"
#include <libhal-icm/icm20948.hpp>
#include <libhal-util/i2c.hpp>
#include <array>

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

  // hal::status icm20948::defaultSetup(){
  //   autoOffsets();
  //   setAccRange(icm20948::ICM20948_ACC_RANGE_2G);
  //   setGyrRange(icm20948::ICM20948_GYRO_RANGE_250);

  //   setAccDLPF(icm20948::ICM20948_DLPF_6);
  //   setGyrDLPF(icm20948::ICM20948_DLPF_6);
  //   setAccSampleRateDivider(10);
  //   return hal::success();
  // }


hal::status icm20948::autoOffsets()
{
  xyzFloat accRawVal, gyrRawVal;
  accOffsetVal.x = 0.0;
  accOffsetVal.y = 0.0;
  accOffsetVal.z = 0.0;

  setGyrDLPF(ICM20948_DLPF_6);           // lowest noise
  setGyrRange(ICM20948_GYRO_RANGE_250);  // highest resolution
  setAccRange(ICM20948_ACC_RANGE_2G);
  setAccDLPF(ICM20948_DLPF_6);

  setMagOpMode(AK09916_CONT_MODE_20HZ); // For Mag

  for (int i = 0; i < 50; i++) {
    readSensor();
    accRawVal = getAccRawValues();
    accOffsetVal.x += accRawVal.x;
    accOffsetVal.y += accRawVal.y;
    accOffsetVal.z += accRawVal.z;
    // delay(10);
  }

  accOffsetVal.x /= 50;
  accOffsetVal.y /= 50;
  accOffsetVal.z /= 50;
  accOffsetVal.z -= 16384.0;

  for (int i = 0; i < 50; i++) {
    readSensor();
    gyrRawVal = getGyrRawValues();
    gyrOffsetVal.x += gyrRawVal.x;
    gyrOffsetVal.y += gyrRawVal.y;
    gyrOffsetVal.z += gyrRawVal.z;
    // delay(1);
  }

  gyrOffsetVal.x /= 50;
  gyrOffsetVal.y /= 50;
  gyrOffsetVal.z /= 50;

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

// void icm20948::setI2CMstSampleRate(hal::byte rateExp)
// {
//   if (rateExp < 16) {
//     writeRegister8(3, ICM20948_I2C_MST_ODR_CFG, rateExp);
//     // HAL_CHECK(hal::write(
//     //   *m_i2c, ICM20948_I2C_MST_ODR_CFG, rateExp, hal::never_timeout()));
//   }
// }

/************* x,y,z results *************/

hal::status icm20948::readSensor()
{
  HAL_CHECK(readAllData(m_read_all_buffer));
  return hal::success();
}

xyzFloat icm20948::getAccRawValues()
{
  xyzFloat accRawVal;
  accRawVal.x = static_cast<int16_t>(((m_read_all_buffer[0]) << 8) | m_read_all_buffer[1]) * 1.0;
  accRawVal.y = static_cast<int16_t>(((m_read_all_buffer[2]) << 8) | m_read_all_buffer[3]) * 1.0;
  accRawVal.z = static_cast<int16_t>(((m_read_all_buffer[4]) << 8) | m_read_all_buffer[5]) * 1.0;
  return accRawVal;
}

xyzFloat icm20948::getCorrectedAccRawValues()
{
  xyzFloat accRawVal = getAccRawValues();
  accRawVal = correctAccRawValues(accRawVal);

  return accRawVal;
}

xyzFloat icm20948::getGValues()
{
  xyzFloat gVal, accRawVal;
  accRawVal = getCorrectedAccRawValues();

  gVal.x = accRawVal.x * accRangeFactor / 16384.0;
  gVal.y = accRawVal.y * accRangeFactor / 16384.0;
  gVal.z = accRawVal.z * accRangeFactor / 16384.0;
  return gVal;
}

float icm20948::getTemperature()
{
  int16_t rawTemp = static_cast<int16_t>(((m_read_all_buffer[12]) << 8) | m_read_all_buffer[13]);
  float tmp =
    (rawTemp * 1.0 - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0;
  return tmp;
}

xyzFloat icm20948::getGyrRawValues()
{
  xyzFloat gyrRawVal;

  gyrRawVal.x = static_cast<int16_t>(((m_read_all_buffer[6]) << 8) | m_read_all_buffer[7]) * 1.0;
  gyrRawVal.y = static_cast<int16_t>(((m_read_all_buffer[8]) << 8) | m_read_all_buffer[9]) * 1.0;
  gyrRawVal.z = static_cast<int16_t>(((m_read_all_buffer[10]) << 8) | m_read_all_buffer[11]) * 1.0;

  return gyrRawVal;
}

xyzFloat icm20948::getCorrectedGyrRawValues()
{
  auto gyrRawVal = getGyrRawValues();
  gyrRawVal = correctGyrRawValues(gyrRawVal);
  return gyrRawVal;
}

xyzFloat icm20948::getGyrValues()
{
  auto gyrVal = getCorrectedGyrRawValues();

  gyrVal.x = gyrVal.x * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.y = gyrVal.y * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.z = gyrVal.z * gyrRangeFactor * 250.0 / 32768.0;

  return gyrVal;
}

xyzFloat icm20948::getMagValues()
{
  int16_t x, y, z;
  xyzFloat mag;

  x = static_cast<int16_t>((m_read_all_buffer[15]) << 8) | m_read_all_buffer[14];
  y = static_cast<int16_t>((m_read_all_buffer[17]) << 8) | m_read_all_buffer[16];
  z = static_cast<int16_t>((m_read_all_buffer[19]) << 8) | m_read_all_buffer[18];

  mag.x = x * AK09916_MAG_LSB;
  mag.y = y * AK09916_MAG_LSB;
  mag.z = z * AK09916_MAG_LSB;

  return mag;
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

/******** Angles and Orientation *********/

// xyzFloat icm20948::getAngles()
// {
//   xyzFloat angleVal;
//   xyzFloat gVal = getGValues();
//   if (gVal.x > 1.0) {
//     gVal.x = 1.0;
//   } else if (gVal.x < -1.0) {
//     gVal.x = -1.0;
//   }
//   angleVal.x = (asin(gVal.x)) * 57.296;

//   if (gVal.y > 1.0) {
//     gVal.y = 1.0;
//   } else if (gVal.y < -1.0) {
//     gVal.y = -1.0;
//   }
//   angleVal.y = (asin(gVal.y)) * 57.296;

//   if (gVal.z > 1.0) {
//     gVal.z = 1.0;
//   } else if (gVal.z < -1.0) {
//     gVal.z = -1.0;
//   }
//   angleVal.z = (asin(gVal.z)) * 57.296;

//   return angleVal;
// }

// ICM20948_orientation icm20948::getOrientation()
// {
//   xyzFloat angleVal = getAngles();
//   ICM20948_orientation orientation = ICM20948_FLAT;
//   if (abs(angleVal.x) < 45) {    // |x| < 45
//     if (abs(angleVal.y) < 45) {  // |y| < 45
//       if (angleVal.z > 0) {      //  z  > 0
//         orientation = ICM20948_FLAT;
//       } else {  //  z  < 0
//         orientation = ICM20948_FLAT_1;
//       }
//     } else {                 // |y| > 45
//       if (angleVal.y > 0) {  //  y  > 0
//         orientation = ICM20948_XY;
//       } else {  //  y  < 0
//         orientation = ICM20948_XY_1;
//       }
//     }
//   } else {                 // |x| >= 45
//     if (angleVal.x > 0) {  //  x  >  0
//       orientation = ICM20948_YX;
//     } else {  //  x  <  0
//       orientation = ICM20948_YX_1;
//     }
//   }
//   return orientation;
// }

// std::string icm20948::getOrientationAsString()
// {
//   ICM20948_orientation orientation = getOrientation();
//   std::string orientationAsString = "";
//   switch (orientation) {
//     case ICM20948_FLAT:
//       orientationAsString = "z up";
//       break;
//     case ICM20948_FLAT_1:
//       orientationAsString = "z down";
//       break;
//     case ICM20948_XY:
//       orientationAsString = "y up";
//       break;
//     case ICM20948_XY_1:
//       orientationAsString = "y down";
//       break;
//     case ICM20948_YX:
//       orientationAsString = "x up";
//       break;
//     case ICM20948_YX_1:
//       orientationAsString = "x down";
//       break;
//   }
//   return orientationAsString;
// }

// float icm20948::getPitch()
// {
//   xyzFloat angleVal = getAngles();
//   float pitch =
//     (atan2(-angleVal.x,
//            sqrt(abs((angleVal.y * angleVal.y + angleVal.z * angleVal.z)))) *
//      180.0) /
//     M_PI;
//   return pitch;
// }

// float icm20948::getRoll()
// {
//   xyzFloat angleVal = getAngles();
//   float roll = (atan2(angleVal.y, angleVal.z) * 180.0) / M_PI;
//   return roll;
// }

/************** Magnetometer **************/

hal::status icm20948::initMagnetometer()
{
  enableI2CMaster();
  resetMag();
  reset_ICM20948();
  sleep(false);
  writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1);  // aligns ODR
  enableI2CMaster();
  // //delay(10);

  int16_t whoAmI = HAL_CHECK(whoAmIMag());
  if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
    return hal::new_error();
  }
  setMagOpMode(AK09916_CONT_MODE_100HZ);

  return hal::success();
}

hal::result<uint16_t> icm20948::whoAmIMag()
{
  return static_cast<uint16_t>(HAL_CHECK(readAK09916Register16(AK09916_WIA_1)));
}

void icm20948::setMagOpMode(AK09916_opMode opMode)
{
  writeAK09916Register8(AK09916_CNTL_2, opMode);
  // HAL_CHECK(hal::write(*m_i2c, AK09916_CNTL_2, opMode,
  // hal::never_timeout()));
  // delay(10);
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
  // delay(10);
}

xyzFloat icm20948::correctAccRawValues(xyzFloat accRawVal)
{
  accRawVal.x =
    (accRawVal.x - (accOffsetVal.x / accRangeFactor)) / accCorrFactor.x;
  accRawVal.y =
    (accRawVal.y - (accOffsetVal.y / accRangeFactor)) / accCorrFactor.y;
  accRawVal.z =
    (accRawVal.z - (accOffsetVal.z / accRangeFactor)) / accCorrFactor.z;

  return accRawVal;
}

xyzFloat icm20948::correctGyrRawValues(xyzFloat gyrRawVal)
{
  gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
  gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
  gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);

  return gyrRawVal;
}




hal::status icm20948::switchBank(hal::byte newBank)
{
  if (newBank != currentBank) {
    currentBank = newBank;
    currentBank = currentBank << 4;
  }
    
    // std::array<hal::byte, 1> reg_bank_sel = {ICM20948_REG_BANK_SEL};
    // std::array<hal::byte, 1> currentBank_reg = {currentBank};


    auto reg_buffer = HAL_CHECK(hal::write_then_read<1>(*m_i2c,
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

hal::status icm20948::writeRegister8(hal::byte bank, hal::byte p_reg_addr, hal::byte p_val)
{
  switchBank(bank);
  HAL_CHECK(hal::write(*m_i2c,
                      m_address,
                      std::array<hal::byte, 2>{ p_reg_addr, p_val },
                      hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::writeRegister16(hal::byte bank, hal::byte reg, int16_t p_val)
{
  switchBank(bank);
  int8_t MSByte = static_cast<int8_t>((p_val >> 8) & 0xFF);
  hal::byte LSByte = p_val & 0xFF;

  HAL_CHECK(hal::write(*m_i2c,
                      m_address,
                      std::array<hal::byte, 3>{ reg, MSByte, LSByte},
                      hal::never_timeout()));

  return hal::success();
}

hal::result<hal::byte> icm20948::readRegister8(hal::byte bank, hal::byte read_reg)
{
  switchBank(bank);
  auto ctrl_buffer = HAL_CHECK(hal::write_then_read<1>(*m_i2c, m_address, std::array<hal::byte, 1>{ read_reg }, hal::never_timeout()));
  return ctrl_buffer[0];
}

hal::result<int16_t> icm20948::readRegister16(hal::byte bank, hal::byte reg)
{

  switchBank(bank);
  hal::byte MSByte = 0, LSByte = 0;
  int16_t reg16Val = 0;

  auto ctrl_buffer = HAL_CHECK(hal::write_then_read<3>(*m_i2c, m_address, std::array<hal::byte, 3>{ reg, MSByte, LSByte}, hal::never_timeout()));

  reg16Val = (ctrl_buffer[0] << 8) + ctrl_buffer[1];
  return reg16Val;
}

hal::status icm20948::readAllData(std::array<hal::byte, 20>& data)
{
  switchBank(0);
  data = HAL_CHECK(hal::write_then_read<20>(*m_i2c, m_address, std::array<hal::byte, 1>{ ICM20948_ACCEL_OUT }, hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::writeAK09916Register8(hal::byte reg, hal::byte val)
{
  writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);  // write AK09916
  writeRegister8(3, ICM20948_I2C_SLV0_REG, reg);  // define AK09916 register to be written to
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
  HAL_CHECK(writeRegister8(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN));  // enable I2C master
  HAL_CHECK(writeRegister8(3, ICM20948_I2C_MST_CTRL, 0x07));  // set I2C clock to 345.60 kHz
  return hal::success();
}

hal::status icm20948::enableMagDataRead(hal::byte reg, hal::byte bytes)
{
  HAL_CHECK(writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ));  // read AK09916
  HAL_CHECK(writeRegister8(3, ICM20948_I2C_SLV0_REG, reg));  // define AK09916 register to be read
  HAL_CHECK(writeRegister8(3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes));  // enable read | number of byte
  return hal::success();
}

}  // namespace hal::icm