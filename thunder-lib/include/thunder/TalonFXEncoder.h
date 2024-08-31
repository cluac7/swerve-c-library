// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

#pragma once

// NOLINTBEGIN
#include <frc/Errors.h>
#include <units/time.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

namespace thunder {
/**
 * A class to read encoder data from CTRE motors, frc::Encoder compatible.
 */
class TalonFXEncoder : public wpi::Sendable,
                       public wpi::SendableHelper<TalonFXEncoder> {
  public:
    explicit TalonFXEncoder(ctre::phoenix::motorcontrol::can::WPI_TalonFX& motor
    );

    /**
     * Gets the current count.
     *
     * Returns the current count on the Encoder.
     *
     * @return Current count from the Encoder.
     */
    [[nodiscard]] auto Get() const -> int;

    /**
     * Reset the Encoder distance to zero.
     *
     * Resets the current count to zero on the encoder.
     */
    auto Reset() -> void;

    /**
     * Get the distance the robot has driven since the last reset.
     *
     * @return The distance driven since the last reset as scaled by the value
     *         from SetDistancePerPulse().
     */
    [[nodiscard]] auto GetDistance() const -> double;

    /**
     * Get the current rate of the encoder.
     *
     * Units are distance per second as scaled by the value from
     * SetDistancePerPulse().
     *
     * @return The current rate of the encoder.
     */
    [[nodiscard]] auto GetRate() const -> double;

    /**
     * The last direction the encoder value changed.
     *
     * @return The last direction the encoder value changed.
     */
    [[nodiscard]] auto GetDirection() const -> bool;

    /**
     * Set the distance per pulse for this encoder.
     *
     * This sets the multiplier used to determine the distance driven based on
     the
    * count value from the encoder.
    *
    * Do not include the decoding type in this scale.  The library already
    * compensates for the decoding type.
    *
    * Set this value based on the encoder's rated Pulses per Revolution and
    * factor in gearing reductions following the encoder shaft.
    *
    * This distance can be in any units you like, linear or angular.
    *
    * @param distancePerPulse The scale factor that will be used to convert
    *                         pulses to useful units.
    */
    auto SetDistancePerPulse(double distancePerPulse) -> void;

    /**
     * Get the distance per pulse for this encoder.
     *
     * @return The scale factor that will be used to convert pulses to useful
     *         units.
     */
    [[nodiscard]] auto GetDistancePerPulse() const -> double;

    /**
     * Initializes the Sendable object
     *
     * Allows this to be seen in LiveWindow and such
     */
    auto InitSendable(wpi::SendableBuilder& builder) -> void;

    auto GetMotor() -> ctre::phoenix::motorcontrol::can::WPI_TalonFX&;

  private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX& m_motor;
    double                                         m_distance_per_pulse = 1;
};
}  // namespace thunder

// NOLINTEND
