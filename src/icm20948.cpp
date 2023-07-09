
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

#include "libhal-icm/icm20948.hpp"

namespace hal::icm {

result<icm20948_accelerometer> icm20948_accelerometer::init(
  hal::i2c& p_i2c,
  hal::byte p_device_address = address_default)
{
  icm20948_accelerometer icm(p_i2c, p_device_address);
  HAL_CHECK(icm.power_on());
  return icm;
}

[[nodiscard]] result<accelerometer::read_t>
icm20948_accelerometer::driver_read()
{
  accelerometer::read_t accel_data;
  constexpr uint16_t bytes_per_axis = 2;
  constexpr uint8_t number_of_axis = 3;

  std::array<hal::byte, bytes_per_axis * number_of_axis> xyz_accel;
  HAL_CHECK(
    hal::write_then_read(*m_i2c,
                         m_address,
                         std::span<const unsigned char>(
                           std::array<unsigned char, 1>{ xyz_accel_register }),
                         std::span<hal::byte>(xyz_accel),
                         hal::never_timeout()));

  const int16_t x = static_cast<int16_t>(xyz_accel[0] << 8 | xyz_accel[1]);
  const int16_t y = static_cast<int16_t>(xyz_accel[2] << 8 | xyz_accel[3]);
  const int16_t z = static_cast<int16_t>(xyz_accel[4] << 8 | xyz_accel[5]);

  // Do the conversion here based on the accelerometer range

  // Convert the 16 bit value into a floating point value m/S^2
  constexpr float max = static_cast<float>(std::numeric_limits<int16_t>::max());
  constexpr float min = static_cast<float>(std::numeric_limits<int16_t>::min());

  const float output_limits =
    static_cast<float>(1 << (static_cast<int>(m_gscale) + 1));
  auto input_range = std::make_pair(max, min);
  auto output_range = std::make_pair(-output_limits, output_limits);

  accel_data.x = hal::map(x, input_range, output_range);
  accel_data.y = hal::map(y, input_range, output_range);
  accel_data.z = hal::map(z, input_range, output_range);

  return accel_data;
}

hal::status icm20948_accelerometer::is_valid_device()
{
  static constexpr hal::byte expected_device_id = 0x69;
  // Read out the identity register
  auto device_id = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c,
    m_address,
    std::array{ icm20948_accelerometer::who_am_i_register },
    hal::never_timeout()));

  if (device_id[0] != expected_device_id) {
    return hal::new_error(std::errc::illegal_byte_sequence);
  }

  return hal::success();
}

hal::status icm20948_accelerometer::active_mode(bool p_is_active = true)
{
  constexpr auto sleep_mask = hal::bit::mask::from<6>();

  auto control =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array{ power_mgmt_1_register },
                                      hal::never_timeout()));

  hal::bit::modify(control[0]).insert<sleep_mask>(!p_is_active);

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array{ power_mgmt_1_register, control[0] },
                       hal::never_timeout()));
  return hal::success();
}

[[nodiscard]] hal::status icm20948_accelerometer::power_on()
{
  HAL_CHECK(is_valid_device());
  return active_mode(true);
}

[[nodiscard]] hal::status icm20948_accelerometer::power_off()
{
  return active_mode(false);
}

}  // namespace hal::icm
