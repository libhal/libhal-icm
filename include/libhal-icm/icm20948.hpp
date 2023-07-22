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
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>

namespace hal::icm {

class icm20948
{

public:
  explicit constexpr icm20948(hal::i2c& p_i2c, hal::byte p_device_address)
    : m_i2c(&p_i2c)
    , m_address(p_device_address)
  {
  }

  /* Basic settings */

  bool init();
  void autoOffsets();
  void setAccOffsets(float xMin,
                     float xMax,
                     float yMin,
                     float yMax,
                     float zMin,
                     float zMax);
  void setGyrOffsets(float xOffset, float yOffset, float zOffset);
  uint8_t whoAmI();
  void enableAcc(bool enAcc);
  void setAccRange(ICM20948_accRange accRange);
  void setAccDLPF(ICM20948_dlpf dlpf);
  void setAccSampleRateDivider(uint16_t accSplRateDiv);
  void enableGyr(bool enGyr);
  void setGyrRange(ICM20948_gyroRange gyroRange);
  void setGyrDLPF(ICM20948_dlpf dlpf);
  void setGyrSampleRateDivider(uint8_t gyrSplRateDiv);
  void setTempDLPF(ICM20948_dlpf dlpf);

  /* x,y,z results */

  void readSensor();
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

  /* Angles and Orientation */

  xyzFloat getAngles();
  ICM20948_orientation getOrientation();
  std::string getOrientationAsString();
  float getPitch();
  float getRoll();

  /* Power, Sleep, Standby */

  void enableCycle(ICM20948_cycle cycle);
  void enableLowPower(bool enLP);
  void setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg);
  void setAccAverageInCycleMode(ICM20948_accAvgLowPower avg);
  void sleep(bool sleep);

  /* Magnetometer */

  bool initMagnetometer();
  uint16_t whoAmIMag();
  void setMagOpMode(AK09916_opMode opMode);
  void resetMag();

private:
  hal::i2c* m_i2c;
  hal::byte m_address;
  hal::byte m_gscale = 0x00;

  uint8_t currentBank;
  std::array<hal::byte, 20> buffer {};
  xyzFloat accOffsetVal;
  xyzFloat accCorrFactor;
  xyzFloat gyrOffsetVal;
  uint8_t accRangeFactor;
  uint8_t gyrRangeFactor;
  uint8_t regVal;  // intermediate storage of register values

  void setClockToAutoSelect();
  xyzFloat correctAccRawValues(xyzFloat accRawVal);
  xyzFloat correctGyrRawValues(xyzFloat gyrRawVal);
  void switchBank(uint8_t newBank);
  void writeRegister8(uint8_t bank, uint8_t reg, uint8_t val);
  void writeRegister16(uint8_t bank, uint8_t reg, int16_t val);

  uint8_t readRegister8(uint8_t bank, uint8_t reg);
  int16_t readRegister16(uint8_t bank, uint8_t reg);

  void readAllData(uint8_t* data);
  void writeAK09916Register8(uint8_t reg, uint8_t val);
  uint8_t readAK09916Register8(uint8_t reg);
  int16_t readAK09916Register16(uint8_t reg);

  void reset_ICM20948();
  void enableI2CMaster();

  void enableMagDataRead(uint8_t reg, uint8_t bytes);

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
};

}  // namespace hal::icm