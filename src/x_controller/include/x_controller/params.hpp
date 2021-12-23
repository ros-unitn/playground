/**
  Copyright (C) 2021 Filippo Rossi

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <array>
#include <bitset>

template <typename JointParamType, typename JointType, int JointCount>
class GenericJointParams {
  public:
    JointParamType &get(JointType joint) {
      return m_data[joint];
    }
    const JointParamType &get(JointType joint) const {
      return m_data[joint];
    }
    void set(JointType joint, JointParamType v) {
      m_ignored[joint] = false;
      m_data[joint] = v;
    }

    void ignore_all() {
      m_ignored.set();
    }
    void unignore_all() {
      m_ignored.reset();
    }
    void ignore(JointType joint, bool ignore = true) {
      m_ignored[joint] = ignore;
    }
    bool is_ignored(JointType joint) const {
      return m_ignored[joint];
    }

    JointParamType &operator[](int i) {
      return m_data[i];
    };
    const JointParamType &operator[](int i) const {
      return m_data[i];
    };

  private:
    std::bitset<JointCount> m_ignored = std::bitset<JointCount>().set();
    std::array<JointParamType, JointCount> m_data = {0};
  };