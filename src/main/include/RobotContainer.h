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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/PowerDistribution.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "thunder/Tuneable.h"

class RobotContainer {
  public:
    RobotContainer();

    auto GetAutonomousCommand() -> frc2::CommandPtr;
    auto LogSystemData() -> void;

  private:
    void ConfigureBindings();

    frc2::CommandXboxController m_controller {
        constants::controller::kDriverControllerPort
    };
    frc2::CommandXboxController m_operator_controller {
        constants::controller::kOperatorControllerPort
    };

    DriveSubsystem m_drive {};

    IntakeSubsystem  m_intake;
    ShooterSubsystem m_shooter;

    frc::SlewRateLimiter<units::scalar> m_drive_x_slew_rate_limiter{constants::controller::kDriveSlewRate};
    frc::SlewRateLimiter<units::scalar> m_drive_y_slew_rate_limiter{constants::controller::kDriveSlewRate};
    frc::SlewRateLimiter<units::scalar> m_turn_slew_rate_limiter{constants::controller::kTurnSlewRate};
};
