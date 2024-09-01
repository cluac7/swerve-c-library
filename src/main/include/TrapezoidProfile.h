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
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>

namespace thunder {
template<typename Distance>
class TrapezoidProfile {
  public:
    using Constraints = frc::TrapezoidProfile<Distance>::Constraints;
    using State       = frc::TrapezoidProfile<Distance>::State;

    // Linter thinks `constraints` aren't initialized and shouldn't be
    // used???
    explicit TrapezoidProfile(Constraints constraints)  // NOLINT
        : m_profile(constraints) {                      // NOLINT
        m_timer.Restart();
    }

    auto Calculate(State goal) -> State {
        m_current = m_profile.Calculate(m_timer.Get(), m_current, goal);
        m_timer.Restart();
        return m_current;
    }

  private:
    frc::TrapezoidProfile<Distance> m_profile;
    frc::Timer                      m_timer {};
    State                           m_current {};
};
}  // namespace thunder
