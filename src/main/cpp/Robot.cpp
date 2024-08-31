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

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc2/command/CommandScheduler.h>

#include <wpi/DataLog.h>

#include "Constants.h"

Robot::Robot() : frc::TimedRobot(constants::kPeriod) {}

void Robot::RobotInit() {
    if constexpr (constants::kUseDataLog) {
        frc::DataLogManager::Start();
        auto&                           log {frc::DataLogManager::GetLog()};
        static wpi::log::StringLogEntry command_events {log, "messages"};

        frc2::CommandScheduler::GetInstance().OnCommandExecute(
            [&](const frc2::Command& command) {
                command_events.Append("EXECUTING: " + command.GetName());
            }
        );
        frc2::CommandScheduler::GetInstance().OnCommandInterrupt(
            [&](const frc2::Command&                 command,
                const std::optional<frc2::Command*>& interruptor) {
                if (interruptor)
                    command_events.Append(
                        "INTERRUPTING: " + command.GetName() +
                        " (interupted by: " + interruptor.value()->GetName() +
                        ")"
                    );
                else
                    command_events.Append("INTERRUPTING: " + command.GetName());
            }
        );
        frc2::CommandScheduler::GetInstance().OnCommandFinish(
            [&](const frc2::Command& command) {
                command_events.Append("FINISHING: " + command.GetName());
            }
        );
    }
    frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());

    m_camera = frc::CameraServer::StartAutomaticCapture();
    m_camera.SetVideoMode(cs::VideoMode::PixelFormat::kMJPEG, 240, 135, 10);
}

void Robot::RobotPeriodic() {
    m_container.LogSystemData();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomous_command = m_container.GetAutonomousCommand();
    if (m_autonomous_command) m_autonomous_command->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomous_command) m_autonomous_command->Cancel();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
auto main() -> int { return frc::StartRobot<Robot>(); }
#endif
