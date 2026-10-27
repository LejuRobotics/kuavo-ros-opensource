#pragma once
#include <algorithm>
#include <cmath>

namespace velocity_ik {
struct FramePredictionSettings {
  double min_hold=.040, max_hold=.060, decay=.030, max_alignment_age=.060;
  bool valid() const {
    return std::isfinite(min_hold) && std::isfinite(max_hold) &&
        std::isfinite(decay) && std::isfinite(max_alignment_age) &&
        min_hold>=.020 && max_hold>=min_hold && max_hold<=.100 &&
        decay>0. && decay<=.100 && max_alignment_age>0. && max_alignment_age<=.100;
  }
};

// Observation age compensates latency; SILENCE since the last new accepted
// observation controls extrapolation decay. Replayed/invalid packets cannot
// renew this clock. Independent instance per hand/elbow, reset on transaction.
struct FramePrediction {
  double observation_ns=0., received_ns=0., source_stamp=0., period=.014;
  double alignment_age=0., silence=0., age=0., rate=0., hold=.040;
  bool valid=false;
  void reset() { *this=FramePrediction{}; }
  void update(double observed, double received, double source, double now,
              const FramePredictionSettings& settings) {
    if (!std::isfinite(observed) || !std::isfinite(received) ||
        !std::isfinite(source) || !std::isfinite(now) || observed<=0. ||
        received<observed) { rate=0.; return; }
    if (!valid || observed>observation_ns) {
      if (valid && source>source_stamp) {
        const double interval=source-source_stamp;
        // A long outage is not evidence of a new nominal frame period.
        if(interval>=.004 && interval<=.040)
          period+=.1*(interval-period);
      }
      observation_ns=observed;received_ns=received;source_stamp=source;
      alignment_age=std::clamp((received-observed)*1e-9,0.,settings.max_alignment_age);
      hold=std::clamp(2.5*period+.002,settings.min_hold,settings.max_hold);
      valid=true;
    }
    silence=std::max(0.,(now-received_ns)*1e-9);
    const double excess=std::max(0.,silence-hold);
    rate=std::exp(-excess/settings.decay);
    // Position prediction is the integral of its exported velocity scale.
    // Finite horizon even if the higher-level watchdog is disabled in a test.
    age=alignment_age+std::min(silence,hold)+settings.decay*(1.-rate);
  }
};
} // namespace velocity_ik
