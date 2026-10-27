#ifndef AI_WORKER_HX5_WRITE_CADENCE_HPP_
#define AI_WORKER_HX5_WRITE_CADENCE_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <stdexcept>

namespace dynamixel_hardware_interface
{
// Called by the existing hardware write loop. No queue, timer or interpolation.
class HX5WriteCadence
{
public:
  explicit HX5WriteCadence(double hz = 0.0)
  {
    if (!std::isfinite(hz) || hz < 0.0 || (hz > 0.0 && hz < 1.0) || hz > 1000.0) {
      throw std::invalid_argument("HX5 write Hz must be zero or in [1, 1000]");
    }
    period_ns_ = hz == 0.0 ? 0 : static_cast<int64_t>(std::llround(1e9 / hz));
  }

  bool identify(uint8_t comm_id, uint8_t id, uint16_t model)
  {
    if (period_ns_ == 0 || comm_id != id || (model != 260 && model != 261)) {
      return false;
    }
    deadlines_[comm_id] = 0;
    reports_[comm_id] = Report{};
    return true;
  }

  bool due(uint8_t comm_id, int64_t now) const
  {
    const auto it = deadlines_.find(comm_id);
    return it == deadlines_.end() || now >= it->second;
  }

  void sent(uint8_t comm_id, int64_t now)
  {
    auto it = deadlines_.find(comm_id);
    if (it == deadlines_.end()) {return;}
    auto & next = it->second;
    if (next == 0) {
      next = now + period_ns_;
    } else if (now >= next) {
      // Preserve 30 Hz on a 100 Hz loop (30/30/40 ms), without catch-up bursts.
      next += ((now - next) / period_ns_ + 1) * period_ns_;
    }
    auto & report = reports_[comm_id];
    if (report.start == 0) {report.start = now;}
    ++report.count;
    if (now - report.start >= 5000000000LL) {
      std::fprintf(stderr, "[HX5 WRITE] ID%u: %.2f Hz (%llu successful packets)\n",
        static_cast<unsigned>(comm_id),
        static_cast<double>(report.count - 1) * 1e9 / (now - report.start),
        static_cast<unsigned long long>(report.count));
      report.start = now;
      report.count = 1;
    }
  }

  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

private:
  struct Report {int64_t start = 0; uint64_t count = 0;};
  int64_t period_ns_;
  std::map<uint8_t, int64_t> deadlines_;
  std::map<uint8_t, Report> reports_;
};
}  // namespace dynamixel_hardware_interface
#endif  // AI_WORKER_HX5_WRITE_CADENCE_HPP_
