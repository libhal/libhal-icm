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

namespace hal::icm {
class icm20948_accelerometer : public accelerometer
{
public:
  hal::i2c* m_i2c;
  hal::byte m_address;
  hal::byte m_gscale = 0x00;
  static constexpr hal::byte address_default = 0x69;
  static constexpr hal::byte who_am_i_register = 0x00;
  static constexpr hal::byte power_mgmt_1_register = 0x06;
  static constexpr hal::byte power_mgmt_2_register = 0x07;
  static constexpr hal::byte xyz_accel_register = 0x2D;

  explicit constexpr icm20948_accelerometer(hal::i2c& p_i2c,
                                            hal::byte p_device_address)
    : m_i2c(&p_i2c)
    , m_address(p_device_address)
  {
  }

  static result<icm20948_accelerometer> create(hal::i2c& p_i2c,
                                               hal::byte p_device_address);

  [[nodiscard]] result<accelerometer::read_t> driver_read() override;

  hal::status is_valid_device();

  hal::status active_mode(bool p_is_active);

  [[nodiscard]] hal::status configure_power_mgmt_2_register();

  [[nodiscard]] hal::status power_on();

  [[nodiscard]] hal::status power_off();
};
}  // namespace hal::icm