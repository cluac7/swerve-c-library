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

#include "Subsystems/DriveSubsystem.h"

#include <iostream>

#include <frc/RobotController.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "BasicLogger.h"
#include "Constants.h"

DriveSubsystem::DriveSubsystem() : 
m_front_left{constants::swerve::kFrontLeftDriveMotorPort,constants::swerve::kFrontLeftTurnMotorPort, "FL", 2.507_rad},
m_front_right{constants::swerve::kFrontRightDriveMotorPort, constants::swerve::kFrontRightTurnMotorPort, "FR", 5.887_rad},
m_rear_left{constants::swerve::kBackLeftDriveMotorPort, constants::swerve::kBackLeftTurnMotorPort, "BL", 3.896_rad},
m_rear_right{constants::swerve::kBackRightDriveMotorPort, constants::swerve::kBackRightTurnMotorPort, "BR", 0.737_rad},
m_odometry{m_drive_kinematics, constants::swerve::kUseNavX ? m_navx.GetRotation2d() : m_gyro.GetRotation2d(), 
  {m_front_left.GetPosition(), m_front_right.GetPosition(), m_rear_left.GetPosition(), m_rear_right.GetPosition()}, 
  frc::Pose2d{constants::swerve::kStartingXPos, constants::swerve::kStartingYPos, constants::swerve::kStartingRotation}}{
    m_desired_state_publisher =
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>(
                "/Swerve/Desired Module States"
            )
            .Publish();
    m_real_state_publisher = nt::NetworkTableInstance::GetDefault()
                                 .GetStructArrayTopic<frc::SwerveModuleState>(
                                     "/Swerve/Real Module States"
                                 )
                                 .Publish();
    frc::SmartDashboard::PutData("Field", &m_field);
};

void DriveSubsystem::Periodic() {
    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(
        constants::swerve::kUseNavX ? m_navx.GetRotation2d()
                                    : m_gyro.GetRotation2d(),
        {m_front_left.GetPosition(), m_front_right.GetPosition(),
         m_rear_left.GetPosition(), m_rear_right.GetPosition()}
    );

    m_field.SetRobotPose(m_odometry.GetPose());

    Log();
}

void DriveSubsystem::Drive(
    frc::ChassisSpeeds chassis_speeds, bool fieldRelative
) {
    auto states =
        m_drive_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
            fieldRelative ? frc::ChassisSpeeds::FromRobotRelativeSpeeds(
                                chassis_speeds, constants::swerve::kUseNavX
                                                    ? m_navx.GetRotation2d()
                                                    : m_gyro.GetRotation2d()
                            )
                          : chassis_speeds,
            constants::kPeriod
        ));

    m_desired_state_publisher.Set(
        std::vector {states[0], states[1], states[2], states[3]}
    );
    m_real_state_publisher.Set(std::vector<frc::SwerveModuleState> {
        m_front_left.GetState(), m_front_right.GetState(),
        m_rear_left.GetState(), m_rear_right.GetState()
    });

    m_drive_kinematics.DesaturateWheelSpeeds(
        &states, constants::swerve::kMaxSpeed
    );

    auto [fl, fr, bl, br] = states;

    auto fakestate =
        frc::SwerveModuleState(chassis_speeds.vy, chassis_speeds.omega * 1_s);

    if (static_cast<bool>(constants::swerve::debugging::kUseFakeModuleState)) {
        std::cout << "using fake state" << std::endl;
        m_rear_left.SetDesiredState(fakestate);
        m_rear_right.SetDesiredState(fakestate);
        m_front_left.SetDesiredState(fakestate);
        m_front_right.SetDesiredState(fakestate);
    } else {
        m_rear_left.SetDesiredState(bl);
        m_rear_right.SetDesiredState(br);
        m_front_left.SetDesiredState(fl);
        m_front_right.SetDesiredState(fr);
    }
}

auto DriveSubsystem::GetHeading() -> units::degree_t {
    return constants::swerve::kUseNavX ? m_navx.GetRotation2d().Degrees()
                                       : m_gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
    m_navx.Reset();
    m_gyro.Reset();
}

auto DriveSubsystem::GetTurnRate() -> double {
    return constants::swerve::kUseNavX ? -m_navx.GetRate() : -m_gyro.GetRate();
}

auto DriveSubsystem::GetPose() -> frc::Pose2d { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
    m_odometry.ResetPosition(
        GetHeading(),
        {m_front_left.GetPosition(), m_front_right.GetPosition(),
         m_rear_left.GetPosition(), m_rear_right.GetPosition()},
        pose
    );
}

auto DriveSubsystem::ResetPoseOdom() -> frc2::CommandPtr {
    return frc2::InstantCommand([&] {
               ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)));
               ZeroHeading();
               std::cout << "reset odom and pose" << std::endl;
           })
        .ToPtr()
        .WithName("Drive Reset Pose Odom");
}

void DriveSubsystem::Log() {
    basic_logger::PutNumber(
        "/Swerve/Odometry/X Pos (m)", m_odometry.GetPose().X().value()
    );
    basic_logger::PutNumber(
        "/Swerve/Odometry/Y Pos (m)", m_odometry.GetPose().Y().value()
    );
    basic_logger::PutNumber(
        "/Swerve/Odometry/Rotation Position (rad)",
        m_odometry.GetPose().Rotation().Radians().value()
    );
    basic_logger::PutNumber(
        "/Odometry/Direct read rotation (rad)",
        constants::swerve::kUseNavX ? m_navx.GetRotation2d().Radians().value()
                                    : m_gyro.GetRotation2d().Radians().value()
    );

    basic_logger::PutNumber("/Swerve/NavX/Heading (deg)", GetHeading().value());
    basic_logger::PutNumber(
        "/Swerve/NavX/Roll (rad)", m_navx.GetRotation3d().X().value()
    );
    basic_logger::PutNumber(
        "/Swerve/NavX/Yaw (rad)", m_navx.GetRotation3d().Z().value()
    );
    basic_logger::PutNumber(
        "/Swerve/NavX/Pitch (rad)", m_navx.GetRotation3d().Y().value()
    );
}
