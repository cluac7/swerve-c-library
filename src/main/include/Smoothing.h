/* Thunder Lib
 * Copyright (C) 2023 FRC Team 4739
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, If not, see <http://www.gnu.org/licenses/>.
 */
#include <cmath>

namespace thunder {
constexpr auto SmoothInput(double input, double deadzone, double exponent)
    -> double {
    // Within deadzone
    if (std::abs(input) < deadzone) return 0.0;
    return std::copysign(
               // Smooth ramp based on distance from the end of the deadzone
               std::pow(std::abs(input) - deadzone, exponent),
               input  // Align with original direction
           ) /
           std::pow(1 - deadzone, exponent);  // Maintain range of [-1, +1]
}

struct Joystick2D {
  public:
    [[nodiscard]] constexpr auto Norm() const -> double {
        return std::sqrt(x * x + y * y);
    }

    // NOLINT
    constexpr auto operator*(double s) -> Joystick2D {
        x *= s;
        y *= s;
        return *this;
    }

    double x {};
    double y {};
};

constexpr auto SmoothInput(Joystick2D input, double deadzone, double exponent)
    -> Joystick2D {
    // Within deadzone
    if (input.Norm() < deadzone) return {};
    // Normalise for direction vector
    return (input * (1 / input.Norm())) *
           // Smooth ramp based on distance from the end of the deadzone
           std::pow(input.Norm() - deadzone, exponent) *
           // Maintain magnitude range of [0, +1]
           (1 / std::pow(1 - deadzone, exponent));
}
}  // namespace thunder
