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

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"
#include "TrapezoidProfile.h"

class SwerveModule {
  public:
    SwerveModule(
        int drive_motor_channel, int turn_motor_channel, std::string mod_name,
        units::radian_t encoder_offset
    );

    auto GetState() -> frc::SwerveModuleState;

    auto GetPosition() -> frc::SwerveModulePosition;

    auto GetCurrentDraw() -> units::ampere_t;

    void SetDesiredState(const frc::SwerveModuleState& state);

    void LogModuleData(
        frc::TrapezoidProfile<units::angle::radian>::State turn_profile_output,
        double turn_ff_value, double drive_ff_value, double turn_motor_value
    );

    void ResetEncoders();

  private:
    rev::CANSparkMax m_drive_motor;
    rev::CANSparkMax m_turn_motor;

    struct ResetDelay {
        inline ResetDelay(rev::CANSparkMax& drive, rev::CANSparkMax& turn) {
            drive.RestoreFactoryDefaults();
            turn.RestoreFactoryDefaults();
            frc::Wait(250_ms);
        }
    } m_resetter;

    rev::SparkRelativeEncoder m_drive_encoder;
    rev::SparkAbsoluteEncoder m_turn_encoder;
    rev::SparkRelativeEncoder m_turn_rel_encoder;

    thunder::Tuneable<bool> m_enabled;

    rev::SparkPIDController m_drive_pid_controller;
    rev::SparkPIDController m_turn_pid_controller;

    frc::SimpleMotorFeedforward<units::radian> m_turn_feed_forward {
        static_cast<units::volt_t>(constants::swerve::turn::kTurnFF_kS),
        static_cast<Vs_per_rad_t>(constants::swerve::turn::kTurnFF_kV)
    };
    frc::SimpleMotorFeedforward<units::meter> m_drive_feed_forward {
        static_cast<units::volt_t>(constants::swerve::drive::kDriveFF_kS),
        static_cast<Vs_per_meter_t>(constants::swerve::drive::kDriveFF_kV)
    };

    // linter thinks "Constraints" is our code and should be renamed to
    // m_constraints
    frc::TrapezoidProfile<units::radian>::Constraints  // NOLINT
        m_turn_profile_constraints {
            static_cast<units::radians_per_second_t>(
                constants::swerve::profile::kMaxTurnVelocity
            ),
            static_cast<units::radians_per_second_squared_t>(
                constants::swerve::profile::kMaxTurnAcceleration
            )
        };
    thunder::TrapezoidProfile<units::radian> m_turn_profile {
        m_turn_profile_constraints
    };

    std::string m_module_name;

    auto GetHeading(rev::SparkAbsoluteEncoder encoder) -> units::radian_t;
};
