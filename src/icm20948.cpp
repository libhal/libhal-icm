
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



// TODO: We should create the read and write functions in this code to encapsulate the read and write functions from
// libhal to make it easier to read and write since we do it very often in this code.
// https://github.com/wollewald/ICM20948_WE/tree/main
// Also need to understand the banks better.  I think we can just use bank 0 for everything.

#include "icm20948_reg.hpp"
#include <libhal-icm/icm20948.hpp>

namespace hal::icm {

result<icm20948> icm20948::create(hal::i2c& p_i2c,
                                  hal::byte p_device_address = address_default)
{
  icm20948 icm(p_i2c, p_device_address);
  return icm;
}

/************ Basic Settings ************/

bool icm20948::init()
{
  currentBank = 0;
  reset_ICM20948();
  if (whoAmI() != ICM20948_WHO_AM_I_CONTENT) {
    return false;
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
  fifoType = ICM20948_FIFO_ACC;

  sleep(false);
  writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR
  // HAL_CHECK(hal::write(*m_i2c, ICM20948_ODR_ALIGN_EN, 1, hal::never_timeout()));
  return true;
}

void icm20948::autoOffsets()
{
  xyzFloat accRawVal, gyrRawVal;
  accOffsetVal.x = 0.0;
  accOffsetVal.y = 0.0;
  accOffsetVal.z = 0.0;

  setGyrDLPF(ICM20948_DLPF_6);           // lowest noise
  setGyrRange(ICM20948_GYRO_RANGE_250);  // highest resolution
  setAccRange(ICM20948_ACC_RANGE_2G);
  setAccDLPF(ICM20948_DLPF_6);
  //delay(100);

  for (int i = 0; i < 50; i++) {
    readSensor();
    accRawVal = getAccRawValues();
    accOffsetVal.x += accRawVal.x;
    accOffsetVal.y += accRawVal.y;
    accOffsetVal.z += accRawVal.z;
    //delay(10);
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
    //delay(1);
  }

  gyrOffsetVal.x /= 50;
  gyrOffsetVal.y /= 50;
  gyrOffsetVal.z /= 50;
}

void icm20948::setAccOffsets(float xMin,
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
}

void icm20948::setGyrOffsets(float xOffset, float yOffset, float zOffset)
{
  gyrOffsetVal.x = xOffset;
  gyrOffsetVal.y = yOffset;
  gyrOffsetVal.z = zOffset;
}

uint8_t icm20948::whoAmI()
{
  return readRegister8(0, ICM20948_WHO_AM_I);
  // std::array<hal::byte, 1> whoami_payload{ ICM20948_WHO_AM_I };
  // std::array<hal::byte, 1> whoami_buffer{};
  // HAL_CHECK(hal::write_then_read(*m_i2c,
  //                                device_address,
  //                                whoami_payload,
  //                                whoami_buffer,
  //                                hal::never_timeout()));
  // return whoami_buffer;
}

void icm20948::enableAcc(bool enAcc)
{
  regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
  std::array<hal::byte, 1> pwr_mgmt{ ICM20948_PWR_MGMT_2 };
  // std::array<hal::byte, 1> regVal{};

  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, pwr_mgmt, regVal, hal::never_timeout()));

  if (enAcc) {
    regVal &= ~ICM20948_ACC_EN;
  } else {
    regVal |= ICM20948_ACC_EN;
  }
  writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_PWR_MGMT_2, regVal, hal::never_timeout()));
}

void icm20948::setAccRange(ICM20948_accRange accRange)
{
  regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
  // std::array<hal::byte, 1> acc_config{ ICM20948_ACCEL_CONFIG };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, acc_config, regVal, hal::never_timeout()));

  regVal &= ~(0x06);
  regVal |= (accRange << 1);
  writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_ACCEL_CONFIG, regVal, hal::never_timeout()));
  // accRangeFactor = 1 << accRange;
}

void icm20948::setAccDLPF(ICM20948_dlpf dlpf)
{
  regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);

  // std::array<hal::byte, 1> acc_config{ ICM20948_ACCEL_CONFIG };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, acc_config, regVal, hal::never_timeout()));

  if (dlpf == ICM20948_DLPF_OFF) {
    regVal[0] &= 0xFE;
    writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
    // HAL_CHECK(
    //   hal::write(*m_i2c, ICM20948_ACCEL_CONFIG, regVal, hal::never_timeout()));
    return;
  } else {
    regVal[0] |= 0x01;
    regVal[0] &= 0xC7;
    regVal[0] |= (dlpf << 3);
  }
  writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_ACCEL_CONFIG, regVal, hal::never_timeout()));
}

void icm20948::setAccSampleRateDivider(uint16_t accSplRateDiv)
{
  writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv);
  // HAL_CHECK(hal::write(
  //   *m_i2c, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv, hal::never_timeout()));
}

void icm20948::enableGyr(bool enGyr)
{
  regVal = readRegister8(0, ICM20948_PWR_MGMT_2);

  // std::array<hal::byte, 1> pwr_mgmt{ ICM20948_PWR_MGMT_2 };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, pwr_mgmt, regVal, hal::never_timeout()));

  if (enGyr) {
    regVal[0] &= ~ICM20948_GYR_EN;
  } else {
    regVal[0] |= ICM20948_GYR_EN;
  }
  writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_PWR_MGMT_2, regVal, hal::never_timeout()));
}

void icm20948::setGyrRange(ICM20948_gyroRange gyroRange)
{
  regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);

  // std::array<hal::byte, 1> gyr_config{ ICM20948_GYRO_CONFIG_1 };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, gyr_config, regVal, hal::never_timeout()));

  regVal[0] &= ~(0x06);
  regVal[0] |= (gyroRange << 1);
  writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_GYRO_CONFIG_1, regVal, hal::never_timeout()));
  // gyrRangeFactor = (1 << gyroRange);
}

void icm20948::setGyrDLPF(ICM20948_dlpf dlpf)
{
  regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);

  // std::array<hal::byte, 1> gyr_config{ ICM20948_GYRO_CONFIG_1 };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, gyr_config, regVal, hal::never_timeout()));

  if (dlpf == ICM20948_DLPF_OFF) {
    regVal &= 0xFE;
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
    // HAL_CHECK(
    //   hal::write(*m_i2c, ICM20948_GYRO_CONFIG_1, regVal, hal::never_timeout()));
    return;
  } else {
    regVal |= 0x01;
    regVal &= 0xC7;
    regVal |= (dlpf << 3);
  }
  writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_GYRO_CONFIG_1, regVal, hal::never_timeout()));
}

void icm20948::setGyrSampleRateDivider(uint8_t gyrSplRateDiv)
{
  writeRegister8(2, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv);
  // HAL_CHECK(hal::write(
  //   *m_i2c, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv, hal::never_timeout()));
}

void icm20948::setTempDLPF(ICM20948_dlpf dlpf)
{
  writeRegister8(2, ICM20948_TEMP_CONFIG, dlpf);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_TEMP_CONFIG, dlpf, hal::never_timeout()));
}

void icm20948::setI2CMstSampleRate(uint8_t rateExp)
{
  if (rateExp < 16) {
    writeRegister8(3, ICM20948_I2C_MST_ODR_CFG, rateExp);
    // HAL_CHECK(hal::write(
    //   *m_i2c, ICM20948_I2C_MST_ODR_CFG, rateExp, hal::never_timeout()));
  }
}

void icm20948::setSPIClockSpeed(unsigned long clock)
{
  mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

/************* x,y,z results *************/

void icm20948::readSensor()
{

  readAllData(buffer);
}

xyzFloat icm20948::getAccRawValues()
{
  xyzFloat accRawVal;
  accRawVal.x = static_cast<int16_t>(((buffer[0]) << 8) | buffer[1]) * 1.0;
  accRawVal.y = static_cast<int16_t>(((buffer[2]) << 8) | buffer[3]) * 1.0;
  accRawVal.z = static_cast<int16_t>(((buffer[4]) << 8) | buffer[5]) * 1.0;
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

xyzFloat icm20948::getAccRawValuesFromFifo()
{
  xyzFloat accRawVal = readICM20948xyzValFromFifo();
  return accRawVal;
}

xyzFloat icm20948::getCorrectedAccRawValuesFromFifo()
{
  xyzFloat accRawVal = getAccRawValuesFromFifo();
  accRawVal = correctAccRawValues(accRawVal);

  return accRawVal;
}

xyzFloat icm20948::getGValuesFromFifo()
{
  xyzFloat gVal, accRawVal;
  accRawVal = getCorrectedAccRawValuesFromFifo();

  gVal.x = accRawVal.x * accRangeFactor / 16384.0;
  gVal.y = accRawVal.y * accRangeFactor / 16384.0;
  gVal.z = accRawVal.z * accRangeFactor / 16384.0;
  return gVal;
}

float icm20948::getResultantG(xyzFloat gVal)
{
  float resultant = 0.0;
  resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));

  return resultant;
}

float icm20948::getTemperature()
{
  int16_t rawTemp = static_cast<int16_t>(((buffer[12]) << 8) | buffer[13]);
  float tmp =
    (rawTemp * 1.0 - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0;
  return tmp;
}

xyzFloat icm20948::getGyrRawValues()
{
  xyzFloat gyrRawVal;

  gyrRawVal.x = (int16_t)(((buffer[6]) << 8) | buffer[7]) * 1.0;
  gyrRawVal.y = (int16_t)(((buffer[8]) << 8) | buffer[9]) * 1.0;
  gyrRawVal.z = (int16_t)(((buffer[10]) << 8) | buffer[11]) * 1.0;

  return gyrRawVal;
}

xyzFloat icm20948::getCorrectedGyrRawValues()
{
  xyzFloat gyrRawVal = getGyrRawValues();
  gyrRawVal = correctGyrRawValues(gyrRawVal);
  return gyrRawVal;
}

xyzFloat icm20948::getGyrValues()
{
  xyzFloat gyrVal = getCorrectedGyrRawValues();

  gyrVal.x = gyrVal.x * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.y = gyrVal.y * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.z = gyrVal.z * gyrRangeFactor * 250.0 / 32768.0;

  return gyrVal;
}

xyzFloat icm20948::getGyrValuesFromFifo()
{
  xyzFloat gyrVal;
  xyzFloat gyrRawVal = readICM20948xyzValFromFifo();

  gyrRawVal = correctGyrRawValues(gyrRawVal);
  gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
  gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;

  return gyrVal;
}

xyzFloat icm20948::getMagValues()
{
  int16_t x, y, z;
  xyzFloat mag;

  x = static_cast<int16_t>((buffer[15]) << 8) | buffer[14];
  y = static_cast<int16_t>((buffer[17]) << 8) | buffer[16];
  z = static_cast<int16_t>((buffer[19]) << 8) | buffer[18];

  mag.x = x * AK09916_MAG_LSB;
  mag.y = y * AK09916_MAG_LSB;
  mag.z = z * AK09916_MAG_LSB;

  return mag;
}

/********* Power, Sleep, Standby *********/

void icm20948::enableCycle(ICM20948_cycle cycle)
{
  regVal = readRegister8(0, ICM20948_LP_CONFIG);

  // std::array<hal::byte, 1> lp_config{ ICM20948_LP_CONFIG };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, lp_config, regVal, hal::never_timeout()));
  regVal[0] &= 0x0F;
  regVal[0] |= cycle;

  writeRegister8(0, ICM20948_LP_CONFIG, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_LP_CONFIG, regVal, hal::never_timeout()));
}

void icm20948::enableLowPower(bool enLP)
{
  regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
  // std::array<hal::byte, 1> pwr_mgmt{ ICM20948_PWR_MGMT_1 };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, pwr_mgmt, regVal, hal::never_timeout()));
  if (enLP) {
    regVal[0] |= ICM20948_LP_EN;
  } else {
    regVal[0] &= ~ICM20948_LP_EN;
  }
  writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_PWR_MGMT_1, regVal, hal::never_timeout()));
}

void icm20948::setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg)
{
  writeRegister8(2, ICM20948_GYRO_CONFIG_2, avg);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_GYRO_CONFIG_2, avg, hal::never_timeout()));
}

void icm20948::setAccAverageInCycleMode(ICM20948_accAvgLowPower avg)
{
  writeRegister8(2, ICM20948_ACCEL_CONFIG_2, avg);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_ACCEL_CONFIG_2, avg, hal::never_timeout()));
}

void icm20948::sleep(bool sleep)
{
  regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
  // std::array<hal::byte, 1> pwr_mgmt{ ICM20948_PWR_MGMT_1 };
  // std::array<hal::byte, 1> regVal{};
  // HAL_CHECK(hal::write_then_read(
  //   *m_i2c, device_address, pwr_mgmt, regVal, hal::never_timeout()));

  if (sleep) {
    regVal[0] |= ICM20948_SLEEP;
  } else {
    regVal[0] &= ~ICM20948_SLEEP;
  }
  writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
  // HAL_CHECK(
  //   hal::write(*m_i2c, ICM20948_PWR_MGMT_1, regVal, hal::never_timeout()));
}

/******** Angles and Orientation *********/

xyzFloat icm20948::getAngles()
{
  xyzFloat angleVal;
  xyzFloat gVal = getGValues();
  if (gVal.x > 1.0) {
    gVal.x = 1.0;
  } else if (gVal.x < -1.0) {
    gVal.x = -1.0;
  }
  angleVal.x = (asin(gVal.x)) * 57.296;

  if (gVal.y > 1.0) {
    gVal.y = 1.0;
  } else if (gVal.y < -1.0) {
    gVal.y = -1.0;
  }
  angleVal.y = (asin(gVal.y)) * 57.296;

  if (gVal.z > 1.0) {
    gVal.z = 1.0;
  } else if (gVal.z < -1.0) {
    gVal.z = -1.0;
  }
  angleVal.z = (asin(gVal.z)) * 57.296;

  return angleVal;
}

ICM20948_orientation icm20948::getOrientation()
{
  xyzFloat angleVal = getAngles();
  ICM20948_orientation orientation = ICM20948_FLAT;
  if (abs(angleVal.x) < 45) {    // |x| < 45
    if (abs(angleVal.y) < 45) {  // |y| < 45
      if (angleVal.z > 0) {      //  z  > 0
        orientation = ICM20948_FLAT;
      } else {  //  z  < 0
        orientation = ICM20948_FLAT_1;
      }
    } else {                 // |y| > 45
      if (angleVal.y > 0) {  //  y  > 0
        orientation = ICM20948_XY;
      } else {  //  y  < 0
        orientation = ICM20948_XY_1;
      }
    }
  } else {                 // |x| >= 45
    if (angleVal.x > 0) {  //  x  >  0
      orientation = ICM20948_YX;
    } else {  //  x  <  0
      orientation = ICM20948_YX_1;
    }
  }
  return orientation;
}

std::string icm20948::getOrientationAsString()
{
  ICM20948_orientation orientation = getOrientation();
  std::string orientationAsString = "";
  switch (orientation) {
    case ICM20948_FLAT:
      orientationAsString = "z up";
      break;
    case ICM20948_FLAT_1:
      orientationAsString = "z down";
      break;
    case ICM20948_XY:
      orientationAsString = "y up";
      break;
    case ICM20948_XY_1:
      orientationAsString = "y down";
      break;
    case ICM20948_YX:
      orientationAsString = "x up";
      break;
    case ICM20948_YX_1:
      orientationAsString = "x down";
      break;
  }
  return orientationAsString;
}

float icm20948::getPitch()
{
  xyzFloat angleVal = getAngles();
  float pitch =
    (atan2(-angleVal.x,
           sqrt(abs((angleVal.y * angleVal.y + angleVal.z * angleVal.z)))) *
     180.0) /
    M_PI;
  return pitch;
}

float icm20948::getRoll()
{
  xyzFloat angleVal = getAngles();
  float roll = (atan2(angleVal.y, angleVal.z) * 180.0) / M_PI;
  return roll;
}

/************** Magnetometer **************/

bool icm20948::initMagnetometer()
{
  enableI2CMaster();
  resetMag();
  reset_ICM20948();
  sleep(false);
  writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR
  // HAL_CHECK(hal::write(*m_i2c, ICM20948_ODR_ALIGN_EN, 1, hal::never_timeout()));
  // //delay(10);
  enableI2CMaster();
  // //delay(10);

  int16_t whoAmI = whoAmIMag();
  if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
    return false;
  }
  setMagOpMode(AK09916_CONT_MODE_100HZ);

  return true;
}

uint16_t icm20948::whoAmIMag()
{
  return static_cast<uint16_t>(readAK09916Register16(AK09916_WIA_1));
}

void icm20948::setMagOpMode(AK09916_opMode opMode)
{
  writeAK09916Register8(AK09916_CNTL_2, opMode);
  // HAL_CHECK(hal::write(*m_i2c, AK09916_CNTL_2, opMode, hal::never_timeout()));
  //delay(10);
  if (opMode != AK09916_PWR_DOWN) {
    enableMagDataRead(AK09916_HXL, 0x08);
  }
}

void icm20948::resetMag()
{
  writeAK09916Register8(AK09916_CNTL_3, 0x01);
  // HAL_CHECK(hal::write(*m_i2c, AK09916_CNTL_3, 0x01, hal::never_timeout()));
  //delay(100);
}

/************************************************
     Private Functions
*************************************************/

void icm20948::setClockToAutoSelect()
{
  regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
  regVal |= 0x01;
  writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
  //delay(10);
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

void icm20948::switchBank(uint8_t newBank)
{
  if (newBank != currentBank) {
    currentBank = newBank;
    HAL_CHECK(hal::write(*m_i2c, m_address, ICM20948_REG_BANK_SEL, hal::never_timeout()));
    HAL_CHECK(hal::write(*m_i2c, m_address, currentBank << 4, hal::never_timeout()));

      // _wire->beginTransmission(m_address);
      // _wire->write(ICM20948_REG_BANK_SEL);
      // _wire->write(currentBank << 4);
      // _wire->endTransmission();

  }
}

void icm20948::writeRegister8(uint8_t bank, uint8_t reg, uint8_t val)
{
  switchBank(bank);

  HAL_CHECK(hal::write(*m_i2c, m_address, reg, hal::never_timeout()));
  HAL_CHECK(hal::write(*m_i2c, m_address, val, hal::never_timeout()));

    // _wire->beginTransmission(m_address);
    // _wire->write(reg);
    // _wire->write(val);
    // _wire->endTransmission();

}

void icm20948::writeRegister16(uint8_t bank, uint8_t reg, int16_t val)
{
  switchBank(bank);
  int8_t MSByte = static_cast<int8_t>((val >> 8) & 0xFF);
  uint8_t LSByte = val & 0xFF;


  HAL_CHECK(hal::write(*m_i2c, m_address, reg, hal::never_timeout()));
  HAL_CHECK(hal::write(*m_i2c, m_address, MSByte, hal::never_timeout()));
  HAL_CHECK(hal::write(*m_i2c, m_address, LSByte, hal::never_timeout()));

    // _wire->beginTransmission(m_address);
    // _wire->write(reg);
    // _wire->write(MSByte);
    // _wire->write(LSByte);
    // _wire->endTransmission();
}

uint8_t icm20948::readRegister8(uint8_t bank, uint8_t reg)
{
  switchBank(bank);
  uint8_t regValue = 0;
  
std::array<hal::byte, 1> ctrl_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, reg, ctrl_buffer, hal::never_timeout()));

    // _wire->beginTransmission(m_address);
    // _wire->write(reg);
    // _wire->endTransmission(false);
    // _wire->requestFrom(m_address, static_cast<uint8_t>(1));
    // regValue = _wire->read();
    // return regValue;
  return ctrl_buffer[0];
}

int16_t icm20948::readRegister16(uint8_t bank, uint8_t reg)
{
  switchBank(bank);
  uint8_t MSByte = 0, LSByte = 0;
  int16_t reg16Val = 0;

  std::array<hal::byte, 2> ctrl_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, reg, ctrl_buffer, hal::never_timeout()));

    // _wire->beginTransmission(m_address);
    // _wire->write(reg);
    // _wire->endTransmission(false);
    // _wire->requestFrom(m_address, static_cast<uint8_t>(2));
      // MSByte = _wire->read();
      // LSByte = _wire->read();
  // reg16Val = (MSByte << 8) + LSByte;

  reg16Val = (ctrl_buffer[0] << 8) + ctrl_buffer[1];
  return reg16Val;
}

void icm20948::readAllData(uint8_t* data)
{
  switchBank(0);
  std::array<hal::byte, 2> read_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, ICM20948_ACCEL_OUT, read_buffer, hal::never_timeout()));


    // _wire->beginTransmission(m_address);
    // _wire->write(ICM20948_ACCEL_OUT);
    // _wire->endTransmission(false);
    // _wire->requestFrom(m_address, static_cast<uint8_t>(20));
    //   for (int i = 0; i < 20; i++) {
    //     data[i] = _wire->read();
    //   }

  for (int i = 0; i < 20; i++) {
    data[i] = read_buffer[i];
  }

}

void icm20948::writeAK09916Register8(uint8_t reg, uint8_t val)
{
  writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);  // write AK09916
  writeRegister8(
    3, ICM20948_I2C_SLV0_REG, reg);  // define AK09916 register to be written to
  writeRegister8(3, ICM20948_I2C_SLV0_DO, val);
}

uint8_t icm20948::readAK09916Register8(uint8_t reg)
{
  enableMagDataRead(reg, 0x01);
  enableMagDataRead(AK09916_HXL, 0x08);
  regVal = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);
  return regVal;
}

int16_t icm20948::readAK09916Register16(uint8_t reg)
{
  int16_t regValue = 0;
  enableMagDataRead(reg, 0x02);
  regValue = readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00);
  enableMagDataRead(AK09916_HXL, 0x08);
  return regValue;
}

void icm20948::reset_ICM20948()
{
  writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
  //delay(10);  // wait for registers to reset
}

void icm20948::enableI2CMaster()
{
  writeRegister8(
    0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN);  // enable I2C master
  writeRegister8(
    3, ICM20948_I2C_MST_CTRL, 0x07);  // set I2C clock to 345.60 kHz
  //delay(10);
}

void icm20948::enableMagDataRead(uint8_t reg, uint8_t bytes)
{
  writeRegister8(
    3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ);  // read AK09916
  writeRegister8(
    3, ICM20948_I2C_SLV0_REG, reg);  // define AK09916 register to be read
  writeRegister8(
    3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes);  // enable read | number of byte
  //delay(10);
}

}  // namespace hal::icm