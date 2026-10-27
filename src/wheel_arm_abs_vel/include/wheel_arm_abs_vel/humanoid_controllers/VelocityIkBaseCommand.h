#pragma once
#include <cmath>
#include <mutex>

namespace humanoidController_wheel_wbc {
// Separate body-frame wheel command. It never writes the fixed-base IK tuple.
// The caller supplies steady-clock seconds; stale/invalid input is a stop.
template <class Twist>
class VelocityIkBaseCommand {
 public:
  void update(const Twist& command, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = std::isfinite(now) && std::isfinite(command.linear.x) &&
             std::isfinite(command.linear.y) && std::isfinite(command.angular.z) &&
             command.linear.z == 0.0 && command.angular.x == 0.0 && command.angular.y == 0.0;
    command_ = valid_ ? command : Twist{};
    received_ = now;
  }
  bool sample(bool enabled, double now, Twist& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = now - received_;
    if (!enabled || !valid_ || !std::isfinite(age) || age < 0.0 || age >= 0.2) {
      valid_ = false;
      out = Twist{};
      return false;
    }
    out = command_;
    return true;
  }
 private:
  std::mutex mutex_;
  Twist command_{};
  double received_{0.0};
  bool valid_{false};
};
}  // namespace humanoidController_wheel_wbc
