#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace velocity_ik {
// Shared by the absolute IK producer and controller. No live retuning.
// Historical API/parameter name retained for existing launch overrides. The
// full-model contract admits wheel layouts; model numbers do not select tuning.
struct S6MotionProfile {
  struct Limit { double velocity, acceleration, jerk; };
  double kp = 250., j3_kd = 18.;
  std::array<Limit, 4> limits{{{2.2,18.,2000.}, {4.5,45.,4000.},
                              {5.5,55.,6000.}, {5.5,55.,6000.}}};
  void validate(int version) const {
    if (version <= 0)
      throw std::runtime_error("Wheel motion profile requires a resolved robot version");
    if (!std::isfinite(kp) || kp <= 0. || kp > 250. ||
        !std::isfinite(j3_kd) || j3_kd <= 0. || j3_kd > 50.)
      throw std::runtime_error("Invalid S6 motion profile gains");
    for (const auto& x : limits)
      if (!std::isfinite(x.velocity) || x.velocity <= 0. ||
          !std::isfinite(x.acceleration) || x.acceleration <= 0. ||
          !std::isfinite(x.jerk) || x.jerk <= 0.)
        throw std::runtime_error("Invalid S6 motion profile dynamic limits");
  }
  template<class Vector, class NameToIndex>
  void applyDynamics(Vector& velocity, Vector& acceleration, Vector& braking,
                     Vector& outward, Vector& jerk, NameToIndex index) const {
    if (velocity.size()!=21 || acceleration.size()!=21 || braking.size()!=21 ||
        outward.size()!=21 || jerk.size()!=21)
      throw std::runtime_error("S6 motion profile requires canonical 21-D layout");
    constexpr std::array<int,4> joints{{2,4,5,6}};
    // Validate all names before touching any component.
    for (int arm=0; arm<2; ++arm) for (int joint:joints) {
      const std::string name="zarm_"+std::string(arm==0?"l":"r")+
                             std::to_string(joint+1)+"_joint";
      if (index(name)!=7+arm*7+joint)
        throw std::runtime_error("S6 motion profile joint layout mismatch: "+name);
    }
    for (int arm=0; arm<2; ++arm) for (int k=0; k<4; ++k) {
      const int i=7+arm*7+joints[k]; const auto& cap=limits[k];
      // Preserve tighter operator limits. Producer, stopping certificate and
      // outward-delay envelope must use the same acceleration bound.
      velocity[i]=std::min(velocity[i],cap.velocity);
      const double a=std::min({acceleration[i],braking[i],outward[i],cap.acceleration});
      acceleration[i]=braking[i]=outward[i]=a;
      jerk[i]=std::min(jerk[i],cap.jerk);
    }
  }
};
// NodeHandle-like adapter keeps the contract testable without a ROS master.
// Enabled profiles require the full YAML; missing fields cannot silently fall
// back to a different profile in one of the two processes.
template<class Node>
S6MotionProfile readS6MotionProfile(const Node& node, int version) {
  S6MotionProfile p;
  auto read=[&](const std::string& key, double& value) {
    if (!node.getParam(key,value)) throw std::runtime_error("Missing S6 motion profile parameter: "+key);
  };
  read("kp",p.kp); read("j3_kd",p.j3_kd);
  constexpr std::array<int,4> joints{{3,5,6,7}};
  for (int k=0;k<4;++k) {
    const auto prefix="j"+std::to_string(joints[k])+"/";
    read(prefix+"velocity",p.limits[k].velocity);
    read(prefix+"acceleration",p.limits[k].acceleration);
    read(prefix+"jerk",p.limits[k].jerk);
  }
  p.validate(version); return p;
}
} // namespace velocity_ik
