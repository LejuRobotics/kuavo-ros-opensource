#pragma once
#include "fixed_bandwidth_reference.h"

namespace velocity_ik {
// Causal source-time secant velocity. Observations keep their original arrival order.
// The shaper still exports the derivative of its own continuous reference state.
struct SourceConsistentHandPrediction {
  bool valid{false};
  double stamp{0.};
  Eigen::Vector3d position=Eigen::Vector3d::Zero(), linear=Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular=Eigen::Vector3d::Zero();

  void reset(double time,const Eigen::Vector3d& p,const Eigen::Quaterniond& q,
             const Eigen::Vector3d& v,const Eigen::Vector3d& w) {
    valid=true; stamp=time; position=p; orientation=q; linear=v; angular=w;
  }
  void observe(double time,const Eigen::Vector3d& p,const Eigen::Quaterniond& q,double tau) {
    if(time<=stamp)return; // A 500 Hz sub-tick is not a new observation.
    const double dt=time-stamp;
    const double alpha=-std::expm1(-dt/tau);
    linear+=alpha*((p-position)/dt-linear);
    angular+=alpha*(fixedRotationError(q,orientation)/dt-angular);
    position=p; orientation=q; stamp=time;
  }
};
} // namespace velocity_ik
