#include "hil_output.hpp"
#include "policy.hpp"
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#define CHECK(v)                                                               \
  do {                                                                         \
    if (!(v)) {                                                                \
      std::cerr << "Line " << __LINE__ << ": " #v << '\n';                     \
      std::exit(1);                                                            \
    }                                                                          \
  } while (0)
struct Packet {
  uint64_t timestamp = 990000, timestamp_remote = 980000;
  int protocol_version = 1, source_system = 245, source_component = 191;
  float actuators[9] = {500, 600, 700, 0, 0, 0, 0, 0, 0};
  float target_position[3] = {0, 0, -1}, target_orientation[3] = {0, 0, 0};
};
int main() {
  CHECK(vtol_pso::hil_profile(22001, 1));
  CHECK(!vtol_pso::hil_profile(22001, 0));
  CHECK(!vtol_pso::hil_profile(4501, 1));
  const std::array<float, 9> lo = {0, 0, 0, 0, 0, -.5f, -.5f, -.5f, -.5f};
  const std::array<float, 9> hi = {2000, 2000, 2000, 1.5f, 1.5f,
                                   .5f,  .5f,  .5f,  .5f};
  float pos[] = {0, 0, -1}, orientation[] = {0, 0, 0};
  auto valid = [&](const Packet &p, uint64_t now = 1000000,
                   uint64_t armed = 900000) {
    return vtol_pso::usable(p, now, armed, pos, orientation, lo, hi);
  };
  Packet p;
  CHECK(valid(p));
  CHECK(!valid(p, 1230000));
  CHECK(!valid(p, 1000000, 985000));
  auto bad = p;
  bad.timestamp_remote = 1000001;
  CHECK(!valid(bad));
  bad = p;
  bad.timestamp = 1000001;
  CHECK(!valid(bad));
  bad = p;
  bad.protocol_version = 0;
  CHECK(!valid(bad));
  bad = p;
  bad.source_system = 1;
  CHECK(!valid(bad));
  bad = p;
  bad.source_component = 1;
  CHECK(!valid(bad));

  for (unsigned i = 0; i < 9; ++i) {
    bad = p;
    bad.actuators[i] = std::numeric_limits<float>::quiet_NaN();
    CHECK(!valid(bad));
    bad = p;
    bad.actuators[i] = hi[i] + 1;
    CHECK(!valid(bad));
    bad = p;
    bad.actuators[i] = lo[i] - 1;
    CHECK(!valid(bad));
    CHECK(vtol_pso::normalize(lo[i], i, lo, hi) == (i < 3 ? 0 : -1));
    CHECK(vtol_pso::normalize(hi[i], i, lo, hi) == 1);
  }

  bad = p;
  bad.target_position[2] = 0;
  CHECK(!valid(bad));
  bad = p;
  bad.target_orientation[0] = .1f;
  CHECK(!valid(bad));
  bad = p;
  bad.target_orientation[0] = 6.28318530718f;
  CHECK(valid(bad));
  CHECK(vtol_pso::normalize(1000, 0, lo, hi) == .5f);
  CHECK(vtol_pso::normalize(0, 3, lo, hi) == -1);
  CHECK(vtol_pso::normalize(0, 5, lo, hi) == 0);
  struct Outputs {
    uint64_t timestamp = 990000;
    bool active = true;
    float controls[9] = {.2f, .3f, .4f, -1, 1, -.2f, .2f, -.3f, .3f};
  } output;
  const std::array<unsigned, 9> mapping = {15, 0, 8, 1, 9, 2, 10, 3, 11};
  const std::array<float, 9> disarmed{};
  float channels[16];
  CHECK(vtol_pso::hil_controls(output, 1000000, channels, mapping, lo, hi,
                               disarmed));

  for (unsigned i = 0; i < 9; ++i) {
    CHECK(channels[mapping[i]] == output.controls[i]);
  }

  CHECK(channels[4] == 0);
  output.active = false;
  CHECK(!vtol_pso::hil_controls(output, 1000000, channels, mapping, lo, hi,
                                disarmed));
  CHECK(channels[15] == 0 && channels[1] == -1 && channels[9] == -1);
  output.active = true;
  CHECK(!vtol_pso::hil_controls(output, 1200000, channels, mapping, lo, hi,
                                disarmed));
  CHECK(!vtol_pso::hil_controls(output, 980000, channels, mapping, lo, hi,
                                disarmed));
  output.controls[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(!vtol_pso::hil_controls(output, 1000000, channels, mapping, lo, hi,
                                disarmed));
  std::cout << "PASS state/receipt age, arming boundary, source/version, "
               "target, bounds and normalization\n";
}
