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

#include "RobotContainer.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "Smoothing.h"

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
    m_drive.SetDefaultCommand(
        frc2::RunCommand(
            [&] {
                const auto scale {
                    m_controller.GetRawButton(5)
                        ? static_cast<double>(constants::controller::kSlowScale)
                        : 1.0
                };
                const auto [smoothed_x, smoothed_y] =
                    SmoothInput(
                        {m_drive_x_slew_rate_limiter.Calculate(m_controller.GetRawAxis(1)),
                         m_drive_y_slew_rate_limiter.Calculate(m_controller.GetRawAxis(0))},
                        constants::controller::kDeadZone,
                        constants::controller::kExponent
                    ) *
                    scale;
                frc::SmartDashboard::PutNumber("/Controller/smoothed x",smoothed_x);
                frc::SmartDashboard::PutNumber("/Controller/smoothed y",smoothed_y);

                auto smoothed_r = thunder::SmoothInput(
                                    //   -m_turn_slew_rate_limiter.Calculate(m_controller.GetRawAxis(2)),
                                      -m_controller.GetRawAxis(4),
                                      constants::controller::kRotDeadZone,
                                      constants::controller::kExponent
                                  ) *
                                  scale;

                frc::ChassisSpeeds chassis_speeds {
                    smoothed_x * constants::swerve::kMaxSpeed,
                    smoothed_y * constants::swerve::kMaxSpeed,
                    smoothed_r * constants::swerve::kJoystickConversionFactor
                };

                m_drive.Drive(chassis_speeds, !m_controller.GetRawButton(6));
            },
            {&m_drive}
        )
            .ToPtr()
            .WithName("Drive Command")
    );

}

auto RobotContainer::GetAutonomousCommand() -> frc2::CommandPtr {  // NOLINT
    frc::ChassisSpeeds drive_forward {
        constants::autonomous::kSpeed, 0_mps, 0_tps
    };
    if (!constants::autonomous::kUseAuto) return frc2::cmd::Print("we no auto");
    return frc2::cmd::Wait(constants::autonomous::kWaitPeriod)
        .AndThen(frc2::RunCommand(
                     [&, drive_forward] { m_drive.Drive(drive_forward, true); }, {&m_drive}
        )
                     .WithTimeout(
                         static_cast<units::meter_t>(
                             constants::autonomous::kDistance
                         ) /
                         static_cast<units::meters_per_second_t>(
                             constants::autonomous::kSpeed
                         )
                     )
                     .AndThen(m_shooter.SpinUpCmd().AndThen(
                         frc2::cmd::Parallel(
                             std::move(m_shooter.ShootCmd()),
                             std::move(m_intake.ShootCmd())
                         )
                             .WithTimeout(2_s)
                     )));
}

auto RobotContainer::LogSystemData() -> void {}
