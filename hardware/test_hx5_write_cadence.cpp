#include "hx5_write_cadence.hpp"
#include <cassert>
#include <vector>
#include <stdexcept>

using dynamixel_hardware_interface::HX5WriteCadence;

int main()
{
  HX5WriteCadence gate(30.0);
  assert(gate.identify(110, 110, 260));
  assert(gate.identify(140, 140, 261));
  assert(!gate.identify(111, 111, 1710));
  assert(!gate.identify(110, 111, 260));
  std::vector<int> sends;
  int arm_sends = 0;
  for (int tick = 0; tick < 100; ++tick) {
    const auto now = 1000000000LL + tick * 10000000LL;
    assert(gate.due(21, now));
    ++arm_sends;
    if (gate.due(110, now)) {
      sends.push_back(tick);
      gate.sent(110, now);
      gate.sent(140, now);
    }
  }
  assert(sends.size() == 30 && arm_sends == 100);
  assert((std::vector<int>(sends.begin(), sends.begin() + 6) ==
    std::vector<int>{0, 4, 7, 10, 14, 17}));
  assert(gate.due(110, 5000000000LL));
  gate.sent(110, 5000000000LL);
  assert(!gate.due(110, 5000000000LL));  // No burst after a communication gap.

  HX5WriteCadence failed(30.0);
  failed.identify(110, 110, 260);
  assert(failed.due(110, 1000000000LL));
  assert(failed.due(110, 1010000000LL));  // Failed packet was not committed.
  failed.sent(110, 1010000000LL);
  assert(!failed.due(110, 1020000000LL));

  HX5WriteCadence disabled;
  assert(!disabled.identify(110, 110, 260));
  assert(disabled.due(110, 1000000000LL));
  assert(disabled.due(110, 1000000001LL));
  for (double rate : {-1.0, 1001.0, std::nan("")}) {
    bool rejected = false;
    try {HX5WriteCadence invalid(rate);} catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }
}
