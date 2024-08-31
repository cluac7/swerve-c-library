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

// NOLINTBEGIN
#include "thunder/sim/TalonFXEncoderSim.h"

namespace thunder {
TalonFXEncoderSim::TalonFXEncoderSim(TalonFXEncoder& encoder)
    : m_encoder(encoder), m_simCollection(encoder.GetMotor()) {}

int TalonFXEncoderSim::GetCount() const { return m_encoder.Get(); }

void TalonFXEncoderSim::SetCount(int count) {
    // Sim ignores motor inversion.
    m_simCollection.SetIntegratedSensorRawPosition(
        count * (m_encoder.GetMotor().GetInverted() ? -1 : 1)
    );
}

bool TalonFXEncoderSim::GetDirection() const {
    return m_encoder.GetDirection();
}

double TalonFXEncoderSim::GetDistancePerPulse() const {
    return m_encoder.GetDistancePerPulse();
}

void TalonFXEncoderSim::SetDistancePerPulse(double distancePerPulse) {
    m_encoder.SetDistancePerPulse(distancePerPulse);
}

void TalonFXEncoderSim::ResetData() {
    // This seems to be for the purposes of tests.
    SetDistance(0);
    SetRate(0);
}

void TalonFXEncoderSim::SetDistance(double distance) {
    SetCount(distance / GetDistancePerPulse());
}

double TalonFXEncoderSim::GetDistance() { return m_encoder.GetDistance(); }

void TalonFXEncoderSim::SetRate(double rate) {
    // Convert distance/second into pulses / 100ms
    double rateInNativeUnits = rate / (10 * GetDistancePerPulse());

    m_simCollection.SetIntegratedSensorVelocity(rateInNativeUnits);
}

double TalonFXEncoderSim::GetRate() { return m_encoder.GetRate(); }
}  // namespace thunder

// NOLINTEND
