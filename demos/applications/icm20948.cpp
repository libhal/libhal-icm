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

#include "../hardware_map.hpp"
#include <libhal-icm/icm20948.hpp>
#include <libhal-icm/xyzFloat.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& i2c = *p_map.i2c;

  hal::print(console, "icm Application Starting...\n\n");
  auto icm_accel = HAL_CHECK(hal::icm::icm20948::create(i2c, 0x69));

  while (true) {

    (void)hal::delay(clock, 500ms);
    hal::print(console, "\n\n================Reading IMU================\n\n");
    auto WAI = HAL_CHECK(icm_accel.whoAmI());

    if (WAI != 0xEA) {
      hal::print(console, "\n\ndata wrong\n");
      hal::print<32>(console, "Data: 0x%x\n", WAI);
    } else {
      hal::print(console, "data right\n");
      hal::print<32>(console, "Data: 0x%x\n", WAI);
    }

    auto sleep = HAL_CHECK(icm_accel.sleep_check());

    if (sleep != 0x01) {
      hal::print(console, "\n\nsleep data wrong\n");
      hal::print<32>(console, "Data: 0x%x\n", sleep);
    } else {
      hal::print(console, "sleep data right\n");
      hal::print<32>(console, "Data: 0x%x\n", sleep);
    }


    auto accel = HAL_CHECK(icm_accel.accel_check());

    if (accel != 0x07) {
      hal::print(console, "\n\naccel data wrong\n");
      hal::print<32>(console, "Data: 0x%x\n", accel);
    } else {
      hal::print(console, "accel data right\n");
      hal::print<32>(console, "Data: 0x%x\n", accel);
    }

    

    (void)hal::delay(clock, 500ms);
    // HAL_CHECK(icm_accel.readSensor());
    auto accelerations = icm_accel.getAccRawValues();
    auto acceleration = *accelerations;
    // hal::icm::icm20948::accel_data_t acceleration;
    hal::print<128>(console,
                    "Accelerometer: x = %fg, y = %fg, z = %fg",
                    acceleration.x,
                    acceleration.y,
                    acceleration.z);

    hal::print(console,"\n");
  }
  return hal::success();
}