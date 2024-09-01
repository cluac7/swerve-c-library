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

#include <numbers>

#include <frc/SPI.h>
#include <frc/SerialPort.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/frequency.h>

#include "thunder/Tuneable.h"

ADD_COMPOUND_UNIT(
    , units::volt_t {} * units::second_t {} / units::radian_t {}, Vs_per_rad,
    "Vs/rad"
);
ADD_COMPOUND_UNIT(
    , units::volt_t {} * units::second_t {} / units::meter_t {}, Vs_per_meter,
    "Vs/meter"
);

// using Vs_per_rad = decltype(units::volt_t{}*units::seco)

namespace constants {
constexpr auto kUseDataLog {true};
constexpr auto kPeriod {20_ms};

namespace swerve {
namespace drive {
inline const thunder::Tuneable<double> kP {0.24, "/Swerve/Drive/kP"};
inline const thunder::Tuneable<double> kI {0.0, "/Swerve/Drive/kI"};
inline const thunder::Tuneable<double> kD {10.0, "/Swerve/Drive/kD"};
constexpr auto                         kDriveFF_kV {2.65_V / 1.0_mps};
constexpr auto                         kDriveFF_kS {0.115_V};
}  // namespace drive

namespace turn {
inline const thunder::Tuneable<double> kP {2.1, "/Swerve/Turn/kP"};
inline const thunder::Tuneable<double> kI {0.0018, "/Swerve/Turn/kI"};
inline const thunder::Tuneable<double> kD {27.0, "/Swerve/Turn/kD"};

constexpr auto kTurnFF_kV {0.426_V / 1_rad_per_s};
constexpr auto kTurnFF_kS {0.223_V};
}  // namespace turn

namespace profile {
inline const thunder::Tuneable<units::radians_per_second_t> kMaxTurnVelocity {
    20_rad_per_s, "/Swerve/Profile/Maximum Turning Velocity"
};
inline const thunder::Tuneable<units::radians_per_second_squared_t>
    kMaxTurnAcceleration {
        120_rad_per_s_sq, "/Swerve/Profile/Maximum Turning Acceleration"
    };
}  // namespace profile

namespace conversion {
constexpr auto           kTurnGearRatio {150 / 7};
constexpr auto           kGearRatio {6.75};
constexpr units::meter_t kWheelDiameter {4_in};

inline const decltype(units::meter_t {} / units::turn_t {}) kDrivePosition {
    kGearRatio * (kWheelDiameter / 2) / (2 * std::numbers::pi * 1_tr)
};
constexpr decltype(units::meters_per_second_t {} / units::revolutions_per_minute_t {})
    kDriveVelocity {(kWheelDiameter / 2) / 1_rad / kGearRatio};
// Converts between the output of the GetVelocity function (RPM) and what we
// want (Rad/s)
constexpr decltype(units::radians_per_second_t {} / units::revolutions_per_minute_t {})
    kTurnVelocity {2 * std::numbers::pi / 60};
constexpr decltype(units::radian_t {} / units::turn_t {}) kTurnPosition {
    2 * std::numbers::pi
};
}  // namespace conversion

constexpr int kFrontLeftDriveMotorPort {36};
constexpr int kFrontLeftTurnMotorPort {37};

constexpr int kFrontRightDriveMotorPort {30};
constexpr int kFrontRightTurnMotorPort {31};

constexpr int kBackLeftDriveMotorPort {34};
constexpr int kBackLeftTurnMotorPort {35};

constexpr int kBackRightDriveMotorPort {32};
constexpr int kBackRightTurnMotorPort {33};

constexpr frc::SPI::Port             kNavXPort = frc::SPI::Port::kMXP;
inline const thunder::Tuneable<bool> kUseNavX {
    true, "/Drive Team/Use NavX (when false uses ADRXS450)?"
};

constexpr auto kStartingXPos {0_m};
constexpr auto kStartingYPos {0_m};
constexpr auto kStartingRotation {0_rad};

constexpr auto                        kMaxSpeed {15_fps};
constexpr units::radians_per_second_t kJoystickConversionFactor {
    6 * std::numbers::pi
};

constexpr auto kWidth {0.566_m};
constexpr auto kHeight {0.566_m};

constexpr auto                      kInvertMotors {true};
inline const thunder::Tuneable<int> kDriveCurrentLimit {
    60, "/Swerve/Driving Current Limit"
};
inline const thunder::Tuneable<int> kTurnCurrentLimit {
    40, "/Swerve/Turning Current Limit"
};

namespace debugging {
inline const thunder::Tuneable<bool> kOverrideControllerTurn {
    false, "/Swerve/Debugging/Direct Joystick Turn?"
};
inline const thunder::Tuneable<bool> kUseFakeModuleState {
    false, "/Swerve/Debugging/Direct Joystick Drive?"
};
inline const thunder::Tuneable<bool> kSkipOptimisation {
    false, "/Swerve/Debugging/Skip Optimisation?"
};
inline const thunder::Tuneable<bool> kNoTurnFeedforward {
    false, "/Swerve/Debugging/Disable Turn Feedforward?"
};
inline const thunder::Tuneable<bool> kNoTurnMotionProfile {
    true, "/Swerve/Debugging/Disable Motion Profile?"
};
inline const thunder::Tuneable<bool> kFakeTurnMotionProfile {
    false, "/Swerve/Debugging/Fake Motion Profile?"
};
inline const thunder::Tuneable<bool> kNoDriveFeedforward {
    false, "/Swerve/Debugging/Disable Drive Feedforward?"
};
}  // namespace debugging
}  // namespace swerve

namespace controller {
constexpr int kDriverControllerPort   = 1;
constexpr int kOperatorControllerPort = 0;

inline const thunder::Tuneable<double> kDeadZone {
    0.3, "/Swerve/Controller/Joystick Deadzone"
};
inline const thunder::Tuneable<double> kRotDeadZone {
    0.3, "/Swerve/Controller/Rotation Deadzone"
};
inline const thunder::Tuneable<double> kExponent {
    2, "/Swerve/Controller/Smoothening Exponent"
};
inline const thunder::Tuneable<double> kSlowScale {
    0.15, "/Swerve/Controller/Slow Scale"
};

constexpr auto kDriveSlewRate{3_Hz};
constexpr auto kTurnSlewRate{3_Hz};
}  // namespace controller
