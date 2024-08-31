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
#include "thunder/TalonFXEncoder.h"

#include <wpi/sendable/SendableRegistry.h>

namespace thunder {
TalonFXEncoder::TalonFXEncoder(
    ctre::phoenix::motorcontrol::can::WPI_TalonFX& motor
)
    : m_motor(motor) {
    wpi::SendableRegistry::AddLW(
        this, "TalonFX Encoder", m_motor.GetDeviceID()
    );
}

int TalonFXEncoder::Get() const { return m_motor.GetSelectedSensorPosition(); }

void TalonFXEncoder::Reset(void) { m_motor.SetSelectedSensorPosition(0); }

double TalonFXEncoder::GetDistance() const {
    return Get() * m_distancePerPulse;
}

double TalonFXEncoder::GetRate() const {
    // units per 100 milliseconds to metres per second
    return m_motor.GetSelectedSensorVelocity() * 10 * m_distancePerPulse;
}

bool TalonFXEncoder::GetDirection() const {
    if (GetRate() >= 0)
        return true;
    else
        return false;
}

void TalonFXEncoder::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

double TalonFXEncoder::GetDistancePerPulse() const {
    return m_distancePerPulse;
}

ctre::phoenix::motorcontrol::can::WPI_TalonFX& TalonFXEncoder::GetMotor() {
    return m_motor;
}

void TalonFXEncoder::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("TalonFX Encoder");
    builder.AddDoubleProperty("Speed", [this] { return GetRate(); }, nullptr);
    builder.AddDoubleProperty(
        "Distance", [this] { return GetDistance(); }, nullptr
    );
    builder.AddDoubleProperty(
        "Distance per Tick", [this] { return GetDistancePerPulse(); }, nullptr
    );
}
}  // namespace thunder

// NOLINTEND
