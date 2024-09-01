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

#include "Subsystems/SwerveModule.h"

#include <functional>
#include <iostream>

#include <frc2/command/PrintCommand.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/velocity.h>

#include <rev/CANSparkLowLevel.h>

#include "BasicLogger.h"

SwerveModule::SwerveModule(
    int drive_motor_channel, int turn_motor_channel, std::string mod_name,
    units::radian_t encoder_offset
)
    : m_drive_motor(
          drive_motor_channel, rev::CANSparkLowLevel::MotorType::kBrushless
      ),
      m_turn_motor(
          turn_motor_channel, rev::CANSparkLowLevel::MotorType::kBrushless
      ),
      m_resetter(m_drive_motor, m_turn_motor),
      m_drive_encoder(
          m_drive_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)
      ),
      m_turn_encoder(m_turn_motor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle
      )),
      m_turn_rel_encoder(
          m_turn_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)
      ),
      m_drive_pid_controller(m_drive_motor.GetPIDController()),
      m_turn_pid_controller(m_turn_motor.GetPIDController()),
      m_module_name {mod_name},
      m_enabled {
          thunder::Tuneable(true, "/Debugging/Enable " + mod_name + " Module?")
      } {
    // Binds all PID values to their update functions
    constants::swerve::drive::kP |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetP),
            &m_drive_pid_controller, std::placeholders::_1, 0
        );
    constants::swerve::drive::kI |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetI),
            &m_drive_pid_controller, std::placeholders::_1, 0
        );
    constants::swerve::drive::kD |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetD),
            &m_drive_pid_controller, std::placeholders::_1, 0
        );
    constants::swerve::turn::kP |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetP), &m_turn_pid_controller,
            std::placeholders::_1, 0
        );
    constants::swerve::turn::kI |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetI), &m_turn_pid_controller,
            std::placeholders::_1, 0
        );
    constants::swerve::turn::kD |
        std::bind(
            std::mem_fn(&rev::SparkPIDController::SetD), &m_turn_pid_controller,
            std::placeholders::_1, 0
        );

    // Set minimum and maximum output to full power
    m_drive_pid_controller.SetOutputRange(-1, 1);
    m_turn_pid_controller.SetOutputRange(-1, 1);

    // Tell PID loops which encoder they are using
    m_turn_pid_controller.SetFeedbackDevice(m_turn_encoder);
    m_drive_pid_controller.SetFeedbackDevice(m_drive_encoder);
    m_turn_motor.SetInverted(constants::swerve::kInvertMotors);

    // Set all conversion factors between encoders and real positions/velocities
    m_drive_encoder.SetPositionConversionFactor(
        constants::swerve::conversion::kDrivePosition.value()
    );
    m_drive_encoder.SetVelocityConversionFactor(
        constants::swerve::conversion::kDriveVelocity.value()
    );

    m_turn_encoder.SetPositionConversionFactor(
        constants::swerve::conversion::kTurnPosition.value()
    );
    m_turn_encoder.SetVelocityConversionFactor(
        constants::swerve::conversion::kTurnVelocity.value()
    );
    m_turn_rel_encoder.SetPositionConversionFactor(
        constants::swerve::conversion::kTurnPosition.value()
    );
    m_turn_rel_encoder.SetVelocityConversionFactor(
        constants::swerve::conversion::kTurnVelocity.value()
    );

    // Wraps input references to between 0 and 2 pi (-pi and pi in practice)
    // because the turning motors are like a circle yk
    m_turn_pid_controller.SetPositionPIDWrappingEnabled(true);
    m_turn_pid_controller.SetPositionPIDWrappingMaxInput(0);
    m_turn_pid_controller.SetPositionPIDWrappingMinInput(2 * std::numbers::pi);

    m_turn_encoder.SetAverageDepth(1);

    // Battery constantly browns out if not current limited
    constants::swerve::kDriveCurrentLimit |
        [&](int current) { m_drive_motor.SetSmartCurrentLimit(current); };
    constants::swerve::kTurnCurrentLimit |
        [&](int current) { m_turn_motor.SetSmartCurrentLimit(current); };

    m_drive_motor
        .GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(false);
    m_drive_motor
        .GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(false);
    m_turn_motor
        .GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(false);
    m_turn_motor
        .GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(false);

    m_enabled | [&](bool enabled) {
        if (enabled) {
            m_turn_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_drive_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        } else {
            m_turn_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_drive_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        }
    };

    m_turn_encoder.SetZeroOffset(encoder_offset.value());

    m_turn_motor.BurnFlash();
    m_drive_motor.BurnFlash();
}

auto SwerveModule::GetState() -> frc::SwerveModuleState {
    return {
        units::meters_per_second_t {m_drive_encoder.GetVelocity()},
        GetHeading(m_turn_encoder)
    };
}

auto SwerveModule::GetPosition() -> frc::SwerveModulePosition {
    return {
        units::meter_t {m_drive_encoder.GetPosition()},
        GetHeading(m_turn_encoder)
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState
) {
    if (!static_cast<bool>(m_enabled)) {
        m_turn_motor.SetVoltage(0_V);
        m_drive_motor.SetVoltage(0_V);
        std::cout << "hit disable on " << m_module_name << std::endl;
        return;
    }

    frc::Rotation2d encoder_rotation {
        GetHeading(m_turn_encoder),
    };

    auto state = referenceState;

    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/turn/Unoptimized Angle (rad)",
        state.angle.Radians().value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/drive/Unoptimized Speed (mps)",
        state.speed.value()
    );
    if (!static_cast<double>(constants::swerve::debugging::kSkipOptimisation)) {
        state =
            frc::SwerveModuleState::Optimize(referenceState, encoder_rotation);
    }

    // Takes into account/scales down movement that is perpendicular to the
    //   direction you're turning to make driving a bit smoother on sharp turns
    state.speed *= (state.angle - encoder_rotation).Cos();
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Optimized Angle (rad)",
        state.angle.Radians().value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/drive/Optimized Speed (mps)",
        state.speed.value()
    );

    auto turn_profile_output =
        m_turn_profile.Calculate(frc::TrapezoidProfile<units::radian>::State {
            state.angle.Radians(), 0_rad_per_s
        });

    auto turn_ff_output =
        m_turn_feed_forward
            .Calculate(
                static_cast<double>(
                    constants::swerve::debugging::kNoTurnMotionProfile
                )
                    ? 0_rad_per_s
                    : turn_profile_output.velocity,
                0_rad_per_s_sq
            )
            .value();

    auto turn_ff_value =
        static_cast<double>(constants::swerve::debugging::kNoTurnFeedforward)
            ? 0
            : turn_ff_output;

    double turn_motor_value;  // NOLINT
    if (static_cast<bool>(constants::swerve::debugging::kNoTurnMotionProfile)) {
        turn_motor_value = state.angle.Radians().value();
    } else {
        turn_motor_value =
            m_turn_profile
                .Calculate(frc::TrapezoidProfile<units::radian>::State {
                    static_cast<double>(
                        constants::swerve::debugging::kFakeTurnMotionProfile
                    )
                        ? 1_rad
                        : state.angle.Radians(),
                    0_rad_per_s
                })
                .position.value();
    }

    auto drive_ff_value =
        static_cast<double>(constants::swerve::debugging::kNoDriveFeedforward)
            ? 0
            : m_drive_feed_forward.Calculate(state.speed, 0_mps_sq).value();

    m_drive_pid_controller.SetReference(
        // state.speed.value(), rev::CANSparkMax::ControlType::kVelocity, 0,
        state.speed.value(), rev::CANSparkLowLevel::ControlType::kVelocity, 0,
        drive_ff_value
    );
    if (m_enabled) {
        m_turn_pid_controller.SetReference(
            turn_motor_value + std::numbers::pi,
            //rev::CANSparkMax::ControlType::kPosition, 0, turn_ff_value
            rev::CANSparkLowLevel::ControlType::kPosition, 0, turn_ff_value
        );
    }

    LogModuleData(
        turn_profile_output, turn_ff_value, drive_ff_value, turn_motor_value
    );
};

auto SwerveModule::GetHeading(rev::SparkAbsoluteEncoder encoder)
    -> units::radian_t {
    return units::radian_t {encoder.GetPosition() - std::numbers::pi};
}

void SwerveModule::LogModuleData(
    frc::TrapezoidProfile<units::angle::radian>::State turn_profile_output,
    double turn_ff_value, double drive_ff_value, double turn_motor_value
) {
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Output Voltage (V)",
        m_turn_motor.GetAppliedOutput() * m_turn_motor.GetBusVoltage()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Output Current (A)",
        m_turn_motor.GetOutputCurrent()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Encoder Position (rad)",
        GetHeading(m_turn_encoder).value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Encoder Velocity (rad per sec)",
        m_turn_encoder.GetVelocity()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Motion Profile Position (rad)",
        turn_profile_output.position.value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name +
            "/Turn/Motion Profile Velocity (rad per sec)",
        turn_profile_output.velocity.value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Feedforward Voltage (V)",
        turn_ff_value
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Motion Profile Position (rad)",
        turn_profile_output.position.value()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name +
            "/Turn/Motion Profile Velocity (rad per sec)",
        turn_profile_output.velocity.value()
    );

    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Drive/Encoder Position (m)",
        m_drive_encoder.GetPosition()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Drive/Encoder Velocity (mps)",
        m_drive_encoder.GetVelocity()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Drive/Feedforward Voltage (V)",
        drive_ff_value
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Drive/Output Voltage (V)",
        m_drive_motor.GetAppliedOutput() * m_drive_motor.GetBusVoltage()
    );
    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Drive/Output Current (A)",
        m_turn_motor.GetOutputCurrent()
    );

    basic_logger::PutNumber(
        "/Swerve/" + m_module_name + "/Turn/Turn Motor Reference (rad)",
        turn_motor_value
    );
}
