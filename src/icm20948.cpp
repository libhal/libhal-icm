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
#include <array>
#include <cmath>

#include <libhal-icm/icm20948.hpp>
#include <libhal-util/i2c.hpp>

#include "icm20948_reg.hpp"

namespace hal::icm {
using namespace icm20948_reg;
using namespace std::literals;

result<icm20948> icm20948::create(hal::i2c& p_i2c)
{
  icm20948 icm(p_i2c);

  HAL_CHECK(icm.init());

  return icm;
}

hal::status icm20948::init()
{
  m_current_bank = 0;
  reset_icm20948();
  reset_mag();
  if (HAL_CHECK(whoami()) != who_am_i_content) {
    return hal::new_error(std::errc::no_such_device);
  }

  m_acc_offset_val.x = 0.0;
  m_acc_offset_val.y = 0.0;
  m_acc_offset_val.z = 0.0;
  m_acc_corr_factor.x = 1.0;
  m_acc_corr_factor.y = 1.0;
  m_acc_corr_factor.z = 1.0;
  m_acc_range_factor = 1.0;
  m_gyro_offset_val.x = 0.0;
  m_gyro_offset_val.y = 0.0;
  m_gyro_offset_val.z = 0.0;
  m_gyro_range_factor = 1.0;

  sleep(false);
  enable_acc(true);
  enable_gyr(true);

  write_register8(2, odr_align_en, 1);  // aligns ODR
  return hal::success();
}

hal::status icm20948::auto_offsets()
{

  set_gyro_dlpf(dlpf_6);           // lowest noise
  set_gyro_range(gyro_range_250);  // highest resolution
  set_acc_range(acc_range_2g);
  set_acc_dlpf(dlpf_6);
  set_temp_dlpf(dlpf_6);
  return hal::success();
}

hal::status icm20948::set_acceleration_offsets(const acceleration_offset_t& acc_offsets)
{
  m_acc_offset_val.x = (acc_offsets.xmax + acc_offsets.xmin) * 0.5;
  m_acc_offset_val.y = (acc_offsets.ymax + acc_offsets.ymin) * 0.5;
  m_acc_offset_val.z = (acc_offsets.zmax + acc_offsets.zmin) * 0.5;
  m_acc_corr_factor.x = (acc_offsets.xmax + abs(acc_offsets.xmin)) / 32768.0;
  m_acc_corr_factor.y = (acc_offsets.ymax + abs(acc_offsets.ymin)) / 32768.0;
  m_acc_corr_factor.z = (acc_offsets.zmax + abs(acc_offsets.zmin)) / 32768.0;

  return hal::success();
}

hal::status icm20948::set_gyro_offsets(const gyro_offset_t& gyr_offsets)
{
  m_gyro_offset_val.x = gyr_offsets.x_offset;
  m_gyro_offset_val.y = gyr_offsets.y_offset;
  m_gyro_offset_val.z = gyr_offsets.z_offset;

  return hal::success();
}

hal::result<hal::byte> icm20948::whoami()
{
  return HAL_CHECK(read_register8(0, who_am_i));
}

hal::status icm20948::enable_acc(bool p_en_acc)
{
  m_reg_val = HAL_CHECK(read_register8(0, pwr_mgmt_2));

  if (p_en_acc) {
    m_reg_val &= ~acc_en;
  } else {
    m_reg_val |= acc_en;
  }

  HAL_CHECK(write_register8(0, pwr_mgmt_2, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_range(acc_range p_accRange)
{
  m_reg_val = HAL_CHECK(read_register8(2, accel_config));
  m_reg_val &= ~(0x06);
  m_reg_val |= (p_accRange << 1);
  HAL_CHECK(write_register8(2, accel_config, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_dlpf(digital_lowpass_filter p_dlpf)
{
  m_reg_val = HAL_CHECK(read_register8(2, accel_config));

  if (p_dlpf == dlpf_off) {
    m_reg_val &= 0xFE;
    HAL_CHECK(write_register8(2, accel_config, m_reg_val));
    return hal::success();
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, accel_config, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_acc_sample_rate_div(uint16_t p_acc_spl_rate_div)
{
  HAL_CHECK(
    write_register16(2, accel_smplrt_div_1, p_acc_spl_rate_div));
  return hal::success();
}

hal::status icm20948::enable_gyr(bool p_enGyr)
{
  m_reg_val = HAL_CHECK(read_register8(0, pwr_mgmt_2));
  if (p_enGyr) {
    m_reg_val &= ~gyro_en;
  } else {
    m_reg_val |= gyro_en;
  }
  HAL_CHECK(write_register8(0, pwr_mgmt_2, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyro_range(gyro_range p_gyro_range)
{
  m_reg_val = HAL_CHECK(read_register8(2, gyro_config_1));
  m_reg_val &= ~(0x06);
  m_reg_val |= (static_cast<hal::byte>(p_gyro_range) << 1);
  HAL_CHECK(write_register8(2, gyro_config_1, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyro_dlpf(digital_lowpass_filter p_dlpf)
{
  m_reg_val = HAL_CHECK(read_register8(2, gyro_config_1));

  if (p_dlpf == dlpf_off) {
    m_reg_val &= 0xFE;
    HAL_CHECK(write_register8(2, gyro_config_1, m_reg_val));
    return hal::success();
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  HAL_CHECK(write_register8(2, gyro_config_1, m_reg_val));

  return hal::success();
}

hal::status icm20948::set_gyro_sample_rate_div(hal::byte p_gyro_spl_rate_div)
{
  HAL_CHECK(write_register8(2, gyro_smplrt_div, p_gyro_spl_rate_div));
  return hal::success();
}

hal::status icm20948::set_temp_dlpf(digital_lowpass_filter p_dlpf)
{
  HAL_CHECK(write_register8(2, temp_config, p_dlpf));
  return hal::success();
}

/************** Read Functions **************/

hal::result<icm20948::accel_read_t> icm20948::read_acceleration()
{
  std::array<hal::byte, 6> data{};
  accel_read_t accel_read = { 0, 0, 0 }, accel_read_raw;
  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ accel_out },
                            hal::never_timeout()));

  accel_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  accel_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  accel_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  accel_read.x = (accel_read.x - (m_acc_offset_val.x / m_acc_range_factor)) /
                 m_acc_corr_factor.x;
  accel_read.y = (accel_read.y - (m_acc_offset_val.y / m_acc_range_factor)) /
                 m_acc_corr_factor.y;
  accel_read.z = (accel_read.z - (m_acc_offset_val.z / m_acc_range_factor)) /
                 m_acc_corr_factor.z;

  accel_read.x = accel_read_raw.x * m_acc_range_factor / 16384.0;
  accel_read.y = accel_read_raw.y * m_acc_range_factor / 16384.0;
  accel_read.z = accel_read_raw.z * m_acc_range_factor / 16384.0;

  return accel_read;
}

hal::result<icm20948::gyro_read_t> icm20948::read_gyroscope()
{
  std::array<hal::byte, 6> data{};
  gyro_read_t gyro_read = { 0, 0, 0 }, gyro_read_raw;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<6>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ gyro_out },
                            hal::never_timeout()));

  gyro_read_raw.x = static_cast<int16_t>(((data[0]) << 8) | data[1]) * 1.0;
  gyro_read_raw.y = static_cast<int16_t>(((data[2]) << 8) | data[3]) * 1.0;
  gyro_read_raw.z = static_cast<int16_t>(((data[4]) << 8) | data[5]) * 1.0;

  gyro_read.x -= (m_gyro_offset_val.x / m_gyro_range_factor);
  gyro_read.y -= (m_gyro_offset_val.y / m_gyro_range_factor);
  gyro_read.z -= (m_gyro_offset_val.z / m_gyro_range_factor);

  gyro_read.x = gyro_read_raw.x * m_gyro_range_factor * 250.0 / 32768.0;
  gyro_read.y = gyro_read_raw.y * m_gyro_range_factor * 250.0 / 32768.0;
  gyro_read.z = gyro_read_raw.z * m_gyro_range_factor * 250.0 / 32768.0;

  return gyro_read;
}

hal::result<icm20948::mag_read_t> icm20948::read_magnetometer()
{
  mag_read_t mag_read;

  int polling_attempts = 0;
  const int max_polling_attempts = 1000;

  while (true) {
    auto status = HAL_CHECK(
      hal::write_then_read<1>(*m_i2c,
                              ak09916_address,
                              std::array<hal::byte, 1>{ ak09916_status_1 },
                              hal::never_timeout()));

    if (status[0] & 0x01) {  // Check if data ready bit is set
      break;
    }

    if (++polling_attempts > max_polling_attempts) {
      return hal::new_error(std::errc::timed_out);
    }
  }

  // Read Mag Data
  auto data =
    HAL_CHECK(hal::write_then_read<6>(*m_i2c,
                                      ak09916_address,
                                      std::array<hal::byte, 1>{ ak09916_hxl },
                                      hal::never_timeout()));

  int16_t x = static_cast<int16_t>((data[1] << 8) | data[0]);
  int16_t y = static_cast<int16_t>((data[3] << 8) | data[2]);
  int16_t z = static_cast<int16_t>((data[5] << 8) | data[4]);

  mag_read.x = x;
  mag_read.y = y;
  mag_read.z = z;

  HAL_CHECK(mag_status1());
  HAL_CHECK(mag_status2());

  return mag_read;
}

hal::result<icm20948::temp_read_t> icm20948::read_temperature()
{
  std::array<hal::byte, 2> data{};
  temp_read_t temp_read;

  switch_bank(0);
  data = HAL_CHECK(
    hal::write_then_read<2>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ temp_out },
                            hal::never_timeout()));
  int16_t rawTemp = static_cast<int16_t>(((data[0]) << 8) | data[1]);
  temp_read.temp =
    (rawTemp * 1.0 - room_temp_offset) / t_sensitivity + 21.0;
  return temp_read;
}

/********* Power, Sleep, Standby *********/

hal::status icm20948::enable_cycle(cycle p_cycle)
{
  m_reg_val = HAL_CHECK(read_register8(0, lp_config));
  m_reg_val &= 0x0F;
  m_reg_val |= static_cast<hal::byte>(p_cycle);

  HAL_CHECK(write_register8(0, lp_config, m_reg_val));
  return hal::success();
}

hal::status icm20948::enable_low_power(bool p_enable_low_power)
{
  m_reg_val = HAL_CHECK(read_register8(0, pwr_mgmt_1));
  if (p_enable_low_power) {
    m_reg_val |= lp_en;
  } else {
    m_reg_val &= ~lp_en;
  }
  HAL_CHECK(write_register8(0, pwr_mgmt_1, m_reg_val));
  return hal::success();
}

hal::status icm20948::set_gyro_averg_cycle_mode(gyro_avg_low_power p_avg)
{
  HAL_CHECK(write_register8(2, gyro_config_2, p_avg));
  return hal::success();
}

hal::status icm20948::set_acc_averg_cycle_mode(acc_avg_low_power p_avg)
{
  HAL_CHECK(write_register8(2, accel_config_2, p_avg));
  return hal::success();
}

hal::status icm20948::sleep(bool p_sleep)
{

  if (p_sleep) {
    m_reg_val |= icm_sleep;
  } else {
    m_reg_val &= ~icm_sleep;
  }
  HAL_CHECK(write_register8(0, pwr_mgmt_1, m_reg_val));
  return hal::success();
}

/************** Magnetometer **************/

hal::status icm20948::init_mag()
{
  enable_bypass_mode();
  set_mag_op_mode(ak09916_cont_mode_20hz);
  return hal::success();
}

hal::status icm20948::set_mag_op_mode(ak09916_op_mode p_op_mode)
{
  write_ak09916_register8(ak09916_cntl_2, p_op_mode);
  if (p_op_mode != ak09916_pwr_down) {
    enable_mag_data_read(ak09916_hxl, 0x08);
  }

  HAL_CHECK(hal::write(
    *m_i2c,
    ak09916_address,
    std::array<hal::byte, 2>{ ak09916_cntl_2, ak09916_cont_mode_20hz },
    hal::never_timeout()));

  return hal::success();
}

hal::status icm20948::write_ak09916_register8(hal::byte p_reg, hal::byte p_val)
{

  write_register8(3, i2c_slv0_addr, ak09916_address);  // write AK09916
  write_register8(3,
                  i2c_slv0_reg,
                  p_reg);  // define AK09916 register to be written to
  write_register8(3, i2c_slv0_do, p_val);

  return hal::success();
}

hal::status icm20948::reset_mag()
{
  enable_bypass_mode();
  HAL_CHECK(
    hal::write(*m_i2c,
               ak09916_address,
               std::array<hal::byte, 2>{ ak09916_cntl_3, 0x01 },  // Soft Reset
               hal::never_timeout()));

  return hal::success();
}

hal::result<hal::byte> icm20948::check_mag_mode()
{
  enable_bypass_mode();
  auto mode = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_cntl_2 },
                            hal::never_timeout()));
  return mode[0];
}

hal::result<hal::byte> icm20948::mag_status1()
{
  auto status = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_status_1 },
                            hal::never_timeout()));

  return status[0];
}

hal::result<hal::byte> icm20948::mag_status2()
{
  auto status = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_status_2 },
                            hal::never_timeout()));

  return status[0];
}

hal::result<hal::byte> icm20948::whoami_ak09916_wia1_direct()
{
  auto result =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      ak09916_address,
                                      std::array<hal::byte, 1>{ ak09916_wia_1 },
                                      hal::never_timeout()));
  return result[0];
}

hal::result<hal::byte> icm20948::whoami_ak09916_wia2_direct()
{
  auto result =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      ak09916_address,
                                      std::array<hal::byte, 1>{ ak09916_wia_2 },
                                      hal::never_timeout()));
  return result[0];
}

/************************************************
     Private Functions
*************************************************/

hal::status icm20948::set_clock_auto_select()
{
  m_reg_val = HAL_CHECK(read_register8(0, pwr_mgmt_1));
  m_reg_val |= 0x01;
  HAL_CHECK(write_register8(0, pwr_mgmt_1, m_reg_val));
  return hal::success();
}

hal::status icm20948::switch_bank(hal::byte p_newBank)
{
  if (p_newBank != m_current_bank) {
    m_current_bank = p_newBank;
    m_current_bank = m_current_bank << 4;
  }
  auto reg_buffer = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ reg_bank_sel },
                            hal::never_timeout()));

  hal::byte reg_val = reg_buffer[0];
  HAL_CHECK(hal::write(*m_i2c,
                       icm20948_address,
                       std::array<hal::byte, 2>{ reg_val, m_current_bank },
                       hal::never_timeout()));

  return hal::success();
}

hal::status icm20948::write_register8(hal::byte p_bank,
                                      hal::byte p_reg_addr,
                                      hal::byte p_val)
{
  switch_bank(p_bank);
  HAL_CHECK(hal::write(*m_i2c,
                       icm20948_address,
                       std::array<hal::byte, 2>{ p_reg_addr, p_val },
                       hal::never_timeout()));
  return hal::success();
}

hal::status icm20948::write_register16(hal::byte p_bank,
                                       hal::byte p_reg,
                                       int16_t p_val)
{
  switch_bank(p_bank);
  hal::byte MSByte = static_cast<int8_t>((p_val >> 8) & 0xFF);
  hal::byte LSByte = p_val & 0xFF;

  HAL_CHECK(hal::write(*m_i2c,
                       icm20948_address,
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
                                      icm20948_address,
                                      std::array<hal::byte, 1>{ p_read_reg },
                                      hal::never_timeout()));
  return ctrl_buffer[0];
}

hal::result<hal::byte> icm20948::read_register16(hal::byte p_bank,
                                                 hal::byte p_reg)
{

  switch_bank(p_bank);
  hal::byte reg16Val = 0;

  auto MSByte =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      icm20948_address,
                                      std::array<hal::byte, 1>{ p_reg },
                                      hal::never_timeout()));
  auto LSByte =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      icm20948_address,
                                      std::array<hal::byte, 1>{ p_reg },
                                      hal::never_timeout()));

  reg16Val = (MSByte[0] << 8) + LSByte[1];
  return reg16Val;
}

hal::status icm20948::reset_icm20948()
{
  HAL_CHECK(write_register8(0, pwr_mgmt_1, icm_reset));
  return hal::success();
}

hal::status icm20948::enable_bypass_mode()
{
  HAL_CHECK(write_register8(0, int_pin_cfg, bypass_en));

  return hal::success();
}

hal::status icm20948::enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes)
{
  HAL_CHECK(write_register8(3,
                            i2c_slv0_addr,
                            ak09916_address | ak09916_read));  // read AK09916
  HAL_CHECK(write_register8(
    3, i2c_slv0_reg, p_reg));  // define AK09916 register to be read
  HAL_CHECK(write_register8(3,
                            i2c_slv0_ctrl,
                            0x80 | p_bytes));  // enable read | number of byte

  return hal::success();
}

}  // namespace hal::icm