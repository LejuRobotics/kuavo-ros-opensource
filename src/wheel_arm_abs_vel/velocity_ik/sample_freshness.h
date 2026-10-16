#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

namespace velocity_ik {
// ROS stamps are compared only with the same ROS clock. Expiry after receipt
// uses steady time, so a wall-clock adjustment cannot extend a sample lease.
inline double transportAge(int64_t published_ns, int64_t received_ns) {
  if (published_ns <= 0 || received_ns < published_ns)
    return std::numeric_limits<double>::infinity();
  return static_cast<double>(received_ns - published_ns) * 1e-9;
}

class MeasurementFreshness {
 public:
  bool observe(int64_t source_ns, int64_t ros_now_ns, int64_t steady_now_ns,
               double timeout_s) {
    const double transport_age = transportAge(source_ns, ros_now_ns);
    if (!std::isfinite(timeout_s) || timeout_s <= 0 ||
        !std::isfinite(transport_age) || transport_age > timeout_s ||
        source_ns <= last_source_ns_ || steady_now_ns <= 0 ||
        (received_ns_ && steady_now_ns < received_ns_)) return false;
    last_source_ns_ = source_ns;
    received_ns_ = steady_now_ns;
    source_age_s_ = transport_age;
    return true;
  }
  double age(int64_t steady_now_ns) const {
    if (!received_ns_ || steady_now_ns < received_ns_)
      return std::numeric_limits<double>::infinity();
    return source_age_s_ + static_cast<double>(steady_now_ns - received_ns_) * 1e-9;
  }
 private:
  int64_t last_source_ns_{0}, received_ns_{0};
  double source_age_s_{0};
};
}  // namespace velocity_ik
