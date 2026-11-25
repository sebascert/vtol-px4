// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "policy.hpp"

namespace vtol_pso {
template <class Outputs, class Mapping, class Limits>
bool hil_controls(const Outputs &outputs, uint64_t now, float (&channels)[16],
                  const Mapping &mapping, const Limits &minimum,
                  const Limits &maximum, const Limits &disarmed) {
  bool valid = outputs.active && outputs.timestamp &&
               outputs.timestamp <= now && now - outputs.timestamp < 100000;

  for (unsigned i = 0; i < 9; ++i) {
    valid &= std::isfinite(outputs.controls[i]) && outputs.controls[i] <= 1.f &&
             outputs.controls[i] >= (i < 3 ? 0.f : -1.f);
  }

  for (float &value : channels) {
    value = 0.f;
  }

  for (unsigned i = 0; i < 9; ++i) {
    channels[mapping[i]] = valid ? outputs.controls[i]
                                 : normalize(disarmed[i], i, minimum, maximum);
  }

  return valid;
}
} // namespace vtol_pso
