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

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <units/base.h>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/UnitTopic.h>
#include <ntcore_cpp.h>

/**
 * @brief A neat way to add compound types while defining their
 * abbreviations. E.g ADD_COMPOUND_UNIT(, units::volt_t {} /
 * units::second_t {}, volts_per_second, vps)
 * @param nspace The namespace in which the type should be defined.
 * @param type The expression to be used as the unit's type (e.g units::volt_t{}
 * / units::second_t{}).
 * @param abbr The abbreviation of the unit (e.g "vps").
 */

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define ADD_COMPOUND_UNIT(nspace, type, name, abbr) \
    namespace nspace {                              \
    typedef decltype(type) name##_t;                \
    }  // namespace nspace                                                                \
    template<>                                                       \
    constexpr const char*                                            \
    units::abbreviation<nspace::name##_t>(const nspace::name##_t&) { \
        return #abbr;                                                \
    }

namespace thunder {
template<typename T>
concept Tuneable_T = std::is_integral_v<T> || std::is_floating_point_v<T> ||
                     units::traits::is_unit_t_v<T>;

/**
 * @brief A wrapper around NetworkTables entries which templates over each type.
 * @param T The type of the value to be stored in the entry.
 */
template<Tuneable_T T>
struct TuneableEntry {
    nt::UnitEntry<T> m_nt_entry {};
};

template<>
struct TuneableEntry<double> {
    nt::DoubleEntry m_nt_entry {};
};

template<>
struct TuneableEntry<int> {
    nt::IntegerEntry m_nt_entry {};
};

template<>
struct TuneableEntry<bool> {
    nt::BooleanEntry m_nt_entry {};
};

/**
 * @brief A wrapper around NetworkTables entries which allows for easy
 * modification of values at runtime. Compound units require specialisation of
 * units::abbreviation() to prevent linker errors through ADD_COMPOUND_TYPE.
 * @param T The type of the value to be stored in the entry.
 * @param def The default value of the entry.
 * @param entry The name of the entry.
 * @param suffix Whether or not to suffix the entry name with the units
 * abbreviation.
 */
template<Tuneable_T T>
class Tuneable {
  public:
    using Bindeable = std::function<void(T)>;

    Tuneable(T def, std::string entry, bool suffix)
        requires units::traits::is_unit_t<T>::value
        : Tuneable(def, suffix ? SuffixEntry(entry) : entry) {}

    Tuneable(T def, const std::string entry) : m_val(def) {
        nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

        if constexpr (units::traits::is_unit_t_v<T>) {
            m_entry.m_nt_entry =
                nt::UnitTopic<T>(inst.GetTopic(entry)).GetEntry(def);
        } else if constexpr (std::is_floating_point_v<T>) {
            m_entry.m_nt_entry = inst.GetDoubleTopic(entry).GetEntry(def);
        } else if constexpr (std::is_same_v<T, bool>) {
            m_entry.m_nt_entry = inst.GetBooleanTopic(entry).GetEntry(def);
        } else if constexpr (std::is_integral_v<T>) {
            m_entry.m_nt_entry = inst.GetIntegerTopic(entry).GetEntry(def);
        } else {
            throw std::runtime_error(
                "ERROR (Tuneable): Initialising an unsupported type, "
                "ensure "
                "type "
                "satisfies Tuneable_T concept"
            );
        }

        inst.AddListener(
            m_entry.m_nt_entry,
            nt::EventFlags::kValueRemote | nt::EventFlags::kValueLocal,
            [this](const nt::Event& ev) {
                this->m_val = Load(ev.GetValueEventData()->value);

                std::lock_guard lock(m_lock);
                for (auto& func : m_functions) func(this->m_val);
            }
        );

        m_entry.m_nt_entry.Set(m_val);
    }

    // NOLINTBEGIN(google-explicit-constructor, hicpp-explicit-conversions)
    template<typename U>
    operator U() const
        requires std::convertible_to<T, U>
    {
        return U(m_val.load());
    }

    // NOLINTEND(google-explicit-constructor, hicpp-explicit-conversions)

    /**
     * @brief Binds a function, and calls it.
     * @return f The function to be bound.
     */
    auto operator|(Bindeable f) const  // NOLINT(fuchsia-overloaded-operator)
        -> const Tuneable<T>& {
        f(m_val);

        m_functions.push_back(f);

        return *this;
    }

    /**
     * @brief Binds a function without calling it. Useful when desigining
     * wrappers around multiple Tuneable's to avoid multiple calls.
     * @param f The function to be bound.
     */
    auto Bind(Bindeable f) const -> void { m_functions.push_back(f); }

    auto operator=(const Tuneable<T>&) = delete;
    auto operator=(Tuneable<T>&&)      = delete;
    Tuneable(Tuneable<T>&)             = delete;
    Tuneable(Tuneable<T>&&)            = delete;
    ~Tuneable()                        = default;

  private:
    [[nodiscard]] auto SuffixEntry(const std::string& entry
    ) const -> std::string;

    [[nodiscard]] auto Load(const nt::Value& value) const -> T;

    mutable std::mutex m_lock {};
    std::atomic<T>     m_val {};

    mutable std::vector<Bindeable> m_functions {};

    TuneableEntry<T> m_entry {};
};

template<Tuneable_T T>
auto Tuneable<T>::SuffixEntry(const std::string& entry) const -> std::string {
    if constexpr (units::traits::is_unit_t<T>::value) {
        std::stringstream ss;
        ss << entry << " (" << m_val.load().abbreviation() << ")";

        return ss.str();
    }

    throw std::runtime_error(
        "ERROR (Tuneable): Attempted to suffix an entry which is not of type "
        "wpi::units::unit_t"
    );
}

template<Tuneable_T T>
auto Tuneable<T>::Load(const nt::Value& value) const -> T {
    if constexpr (units::traits::is_unit_t_v<T>) {
        return T(value.GetDouble());
    } else if constexpr (std::is_floating_point_v<T>) {
        return value.GetDouble();
    } else if constexpr (std::is_same_v<T, bool>) {
        return value.GetBoolean();
    } else if constexpr (std::is_integral_v<T>) {
        return value.GetInteger();
    } else {
        throw std::runtime_error(
            "ERROR (Tuneable): Initialising an unsupported type, ensure "
            "type "
            "satisfies Tuneable_T concept"
        );
    }
}

/**
 * @brief A wrapper around multiple Tuneables to allow for binding of functions
 * that require multiple tuned values as their arguments.
 * @param T The types of the values to be stored in the entry.
 */

template<Tuneable_T... T>
class Compound {
  public:
    using Bindeable = std::function<void(T...)>;

    explicit Compound(const Tuneable<T>&... t) : m_members(std::tie(t...)) {}

    auto operator|(Bindeable f) const  // NOLINT(fuchsia-overloaded-operator)
        -> const Compound<T...>& {
        std::apply(
            [&](const Tuneable<T>&... args) {
                std::apply(f, std::make_tuple(T(args)...));
            },
            m_members
        );

        auto iter = [&](const Tuneable<T>&... t) {
            (
                [&]() {
                    t.Bind([&](T) {
                        auto vals = std::make_tuple(T(t)...);

                        std::apply(f, vals);
                    });
                }(),
                ...
            );
        };

        std::apply(iter, m_members);

        return *this;
    }

  private:
    std::tuple<const Tuneable<T>&...> m_members {};
};

}  // namespace thunder
