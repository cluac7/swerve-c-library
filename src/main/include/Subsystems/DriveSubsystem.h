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

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <units/math.h>

#include <AHRS.h>
#include <networktables/StructArrayTopic.h>

#include "Constants.h"
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
  public:
    DriveSubsystem();

    void Periodic() override;

    void Drive(frc::ChassisSpeeds chassis_speeds, bool fieldRelative);

    auto ResetPoseOdom() -> frc2::CommandPtr;
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    auto GetHeading() -> units::degree_t;
    void ZeroHeading();
    auto GetTurnRate() -> double;
    auto GetPose() -> frc::Pose2d;
    void ResetOdometry(frc::Pose2d pose);
    void Log();

  private:
    frc::Field2d m_field;

    frc::SwerveDriveKinematics<4> m_drive_kinematics {
        frc::Translation2d {
                            constants::swerve::kHeight / 2,  constants::swerve::kWidth / 2 },
        frc::Translation2d {
                            constants::swerve::kHeight / 2, -constants::swerve::kWidth / 2 },
        frc::Translation2d {
                            -constants::swerve::kHeight / 2,  constants::swerve::kWidth / 2},
        frc::Translation2d {
                            -constants::swerve::kHeight / 2, -constants::swerve::kWidth / 2}
    };
    SwerveModule       m_front_left;
    SwerveModule       m_rear_left;
    SwerveModule       m_front_right;
    SwerveModule       m_rear_right;
    AHRS               m_navx {constants::swerve::kNavXPort};
    frc::ADXRS450_Gyro m_gyro;

    frc::SwerveDriveOdometry<4>                      m_odometry;
    nt::StructArrayPublisher<frc::SwerveModuleState> m_desired_state_publisher;
    nt::StructArrayPublisher<frc::SwerveModuleState> m_real_state_publisher;
};
