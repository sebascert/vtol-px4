// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <cstdint>

namespace vtol_pso {
constexpr uint64_t timeout_us = 250000;
inline bool hil_profile(int autostart, int hitl) {
  return autostart == 22001 && hitl == 1;
}
inline bool fresh(uint64_t stamp, uint64_t now) {
  return stamp && stamp <= now && now - stamp < timeout_us;
}
template <class Limits>
inline float normalize(float value, unsigned i, const Limits &minimum,
                       const Limits &maximum) {
  const float fraction =
      fmaxf(0.f, fminf(1.f, (value - minimum[i]) / (maximum[i] - minimum[i])));
  return i < 3 ? fraction : 2.f * fraction - 1.f;
}

template <class Packet, class Limits>
bool usable(const Packet &p, uint64_t now, uint64_t enabled_since,
            const float *position, const float *orientation,
            const Limits &minimum, const Limits &maximum) {
  if (p.protocol_version != 1 || p.source_system != 245 ||
      p.source_component != 191 || !fresh(p.timestamp, now) ||
      !fresh(p.timestamp_remote, now) || p.timestamp_remote <= enabled_since ||
      p.timestamp < enabled_since) {
    return false;
  }

  for (unsigned i = 0; i < 9; ++i)
    if (!std::isfinite(p.actuators[i]) || p.actuators[i] < minimum[i] ||
        p.actuators[i] > maximum[i]) {
      return false;
    }

  for (unsigned i = 0; i < 3; ++i) {
    if (!std::isfinite(position[i]) || !std::isfinite(orientation[i]) ||
        !std::isfinite(p.target_position[i]) ||
        !std::isfinite(p.target_orientation[i]) ||
        fabsf(p.target_position[i] - position[i]) > 1e-4f ||
        fabsf(remainderf(p.target_orientation[i] - orientation[i],
                         6.28318530718f)) > 0.05f) {
      return false;
    }
  }

  return true;
}
} // namespace vtol_pso
