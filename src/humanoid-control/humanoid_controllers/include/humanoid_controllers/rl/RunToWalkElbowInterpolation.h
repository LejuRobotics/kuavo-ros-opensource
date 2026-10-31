#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace humanoid_controller {
// Reference-only override. The controller retains its normal PD gains and torque limits.
class RunToWalkElbowInterpolation {
 public:
  struct Config {
    bool enabled = false;
    std::array<double,2> target{{-.8,-.8}};
    double duration = .6, release_duration = .4;
  };
  enum class Phase { Idle, Moving, Holding, Releasing };
  void configure(const Config& c) {
    if (!std::isfinite(c.duration) || c.duration <= 0 ||
        !std::isfinite(c.release_duration) || c.release_duration <= 0 ||
        !std::isfinite(c.target[0]) || !std::isfinite(c.target[1]))
      throw std::invalid_argument("Invalid RUN elbow interpolation targets/durations");
    config_ = c; reset();
  }
  void reset() { phase_ = Phase::Idle; valid_ = requested_ = false; output_ = {{0,0}}; }
  std::array<double,2> update(double now, bool requested, const std::array<double,2>& rl) {
    if (!config_.enabled || !std::isfinite(now) || !std::isfinite(rl[0]) || !std::isfinite(rl[1])) {
      reset(); return rl;
    }
    if (valid_ && now < last_time_) reset();
    if (requested != requested_) {
      start_ = valid_ ? output_ : rl;
      start_time_ = now;
      phase_ = requested ? Phase::Moving : Phase::Releasing;
      requested_ = requested;
    }
    if (phase_ == Phase::Idle) output_ = rl;
    else if (phase_ == Phase::Holding) output_ = config_.target;
    else {
      const double duration = requested_ ? config_.duration : config_.release_duration;
      const double u = std::clamp((now-start_time_)/duration, 0., 1.);
      // Quintic smoothstep has zero first/second derivatives at both endpoints.
      const double w = u*u*u*(10. + u*(-15. + 6.*u));
      const auto& target = requested_ ? config_.target : rl;
      for (size_t i=0;i<2;++i) output_[i] = (1.-w)*start_[i] + w*target[i];
      if (u>=1.) phase_ = requested_ ? Phase::Holding : Phase::Idle;
    }
    last_time_ = now; valid_ = true; return output_;
  }
  bool active() const { return phase_ != Phase::Idle; }
  bool enabled() const { return config_.enabled; }
  const std::array<double,2>& reference() const { return output_; }
  const char* phaseName() const {
    switch (phase_) {
      case Phase::Moving: return "interpolating";
      case Phase::Holding: return "holding";
      case Phase::Releasing: return "releasing";
      default: return "idle";
    }
  }
 private:
  Config config_;
  Phase phase_ = Phase::Idle;
  bool valid_ = false, requested_ = false;
  double start_time_ = 0., last_time_ = 0.;
  std::array<double,2> start_{{0,0}}, output_{{0,0}};
};
}
