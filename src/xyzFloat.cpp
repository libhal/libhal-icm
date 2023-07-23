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

#include <libhal-icm/xyzFloat.hpp>

namespace hal::icm {


xyzFloat::xyzFloat(): x(0.f), y(0.f), z(0.f) {}

xyzFloat::xyzFloat(float const x, float const y, float const z)
    : x(x)
    , y(y)
    , z(z)
{
    // intentionally empty
}

xyzFloat xyzFloat::operator+() const
{
    return *this;
}

xyzFloat xyzFloat::operator-() const
{
    return xyzFloat{-x,
                    -y,
                    -z};
}

xyzFloat xyzFloat::operator+(xyzFloat const & summand) const
{
    return xyzFloat{x + summand.x,
                    y + summand.y,
                    z + summand.z};
}

xyzFloat xyzFloat::operator-(xyzFloat const & subtrahend) const
{
    return xyzFloat{x - subtrahend.x,
                    y - subtrahend.y,
                    z - subtrahend.z};
}

xyzFloat xyzFloat::operator*(float const operand) const
{
    return xyzFloat{x * operand,
                    y * operand,
                    z * operand};
}

xyzFloat xyzFloat::operator/(float const divisor) const
{
    return xyzFloat{x / divisor,
                    y / divisor,
                    z / divisor};
}

xyzFloat & xyzFloat::operator+=(xyzFloat const & summand)
{
    x += summand.x;
    y += summand.y;
    z += summand.z;
    return *this;
}

xyzFloat & xyzFloat::operator-=(xyzFloat const & subtrahend)
{
    x -= subtrahend.x;
    y -= subtrahend.y;
    z -= subtrahend.z;
    return *this;
}

xyzFloat & xyzFloat::operator*=(float const operand)
{
    x *= operand;
    y *= operand;
    z *= operand;
    return *this;
}

xyzFloat & xyzFloat::operator/=(float const divisor)
{
    x /= divisor;
    y /= divisor;
    z /= divisor;
    return *this;
}


}  // namespace hal::icm