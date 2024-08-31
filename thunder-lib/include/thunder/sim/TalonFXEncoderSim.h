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

#include <ctre/phoenix/motorcontrol/TalonFXSimCollection.h>
#include <ctre/phoenix/motorcontrol/TalonSRXSimCollection.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include "thunder/TalonFXEncoder.h"

namespace thunder {
class TalonFXEncoderSim {
  public:
    /**
     * Constructs from an Encoder object.
     *
     * @param encoder Encoder to simulate
     */
    [[nodiscard]] explicit TalonFXEncoderSim(TalonFXEncoder& encoder);

    // frc::sim::EncoderSim functions

    /**
     * Read the count of the encoder.
     *
     * @return the count
     */
    [[nodiscard]] auto GetCount() const -> int;

    /**
     * Change the count of the encoder.
     *
     * @param count the new count
     */
    auto SetCount(int count) -> void;

    /**
     * The last direction the encoder value changed.
     *
     * @return The last direction the encoder value changed.
     */
    [[nodiscard]] auto GetDirection() const -> bool;

    /**
     * Read the distance per pulse of the encoder.
     *
     * @return the encoder distance per pulse
     */
    [[nodiscard]] auto GetDistancePerPulse() const -> double;

    /**
     * Change the encoder distance per pulse.
     *
     * @param distancePerPulse the new distance per pulse
     */
    auto SetDistancePerPulse(double distancePerPulse) -> void;

    /**
     * Resets all simulation data for this encoder.
     */
    auto ResetData() -> void;

    /**
     * Change the encoder distance.
     *
     * @param distance the new distance
     */
    auto SetDistance(double distance) -> void;

    /**
     * Read the distance of the encoder.
     *
     * @return the encoder distance
     */
    [[nodiscard]] auto GetDistance() -> double;

    /**
     * Change the rate of the encoder.
     *
     * @param rate the new rate
     */
    auto SetRate(double rate) -> void;

    /**
     * Get the rate of the encoder.
     *
     * @return the rate of change
     */
    [[nodiscard]] auto GetRate() -> double;

  private:
    TalonFXEncoder&                                   m_encoder;
    ctre::phoenix::motorcontrol::TalonFXSimCollection m_sim_collection;
};
}  // namespace thunder

// NOLINTEND
