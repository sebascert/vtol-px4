// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <cstring>
#include <mavlink.h>

namespace vtol_pso {

inline bool valid_allocation(const mavlink_message_t &message,
                             const mavlink_debug_float_array_t &packet) {
  constexpr char name[10] = "PSO_ALLOC";

  if (message.sysid != 245 || message.compid != 191 || packet.array_id != 0 ||
      std::memcmp(packet.name, name, sizeof(name)) != 0) {
    return false;
  }

  for (unsigned i = 0; i < 13; ++i) {
    if (!std::isfinite(packet.data[i])) {
      return false;
    }
  }
  if (!std::isfinite(packet.data[13]) || packet.data[13] < 0.f ||
      packet.data[13] > 1.f || std::floor(packet.data[13]) < packet.data[13]) {
    return false;
  }
  if (packet.data[13] > 0.5f) {
    for (unsigned i = 14; i < 20; ++i) {
      if (!std::isfinite(packet.data[i])) {
        return false;
      }
    }
  }

  return packet.data[9] >= 0.f && packet.data[10] >= 0.f &&
         packet.data[11] >= 0.f && packet.data[11] <= 1.f &&
         std::floor(packet.data[11]) >= packet.data[11] &&
         packet.data[12] >= 1.f && packet.data[12] <= 16777216.f &&
         std::floor(packet.data[12]) >= packet.data[12];
}

} // namespace vtol_pso
