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

#include <map>
#include <mutex>
#include <string>

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/DataLog.h>

#include "Constants.h"

namespace basic_logger {
inline auto PutNumber(std::string key, double value) -> void {
    frc::SmartDashboard::PutNumber(key, value);

    if constexpr (constants::kUseDataLog) {
        static std::mutex logger_lock {};
        static auto&      log {frc::DataLogManager::GetLog()};
        static std::map<std::string, wpi::log::DoubleLogEntry> log_entries {};

        std::lock_guard lock {logger_lock};
        if (!log_entries.contains(key))
            log_entries[key] = wpi::log::DoubleLogEntry(log, key);
        log_entries[key].Append(value);
    }
}

inline auto PutBoolean(std::string key, bool value) -> void {
    frc::SmartDashboard::PutBoolean(key, value);

    if constexpr (constants::kUseDataLog) {
        static std::mutex logger_lock {};
        static auto&      log {frc::DataLogManager::GetLog()};
        static std::map<std::string, wpi::log::BooleanLogEntry> log_entries {};

        std::lock_guard lock {logger_lock};
        if (!log_entries.contains(key))
            log_entries[key] = wpi::log::BooleanLogEntry(log, key);
        log_entries[key].Append(value);
    }
}
}  // namespace basic_logger
