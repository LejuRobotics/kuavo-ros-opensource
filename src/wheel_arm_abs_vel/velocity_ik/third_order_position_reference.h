#pragma once
#include "continuous_reference.h"

namespace velocity_ik {
// H(s)=w^3/(s+w)^3. Preserve the actual -3dB cutoff when changing order.
// Exact ZOH integration: a new observation changes jerk, while p/v/a remain
// continuous. All three states describe the same Cartesian trajectory.
inline double thirdOrderPositionNaturalHz(double cutoff) {
  return cutoff/std::sqrt(std::cbrt(2.)-1.);
}
inline void stepThirdOrderPosition(ContinuousVector& r,
                                  const Eigen::Vector3d& target,
                                  double dt,double cutoff) {
  if(!r.valid)r.reset(target);
  const double w=2.*3.14159265358979323846*thirdOrderPositionNaturalHz(cutoff);
  const Eigen::Vector3d A=r.p-target;
  const Eigen::Vector3d B=r.v+w*A;
  const Eigen::Vector3d C=.5*(r.a+2.*w*r.v+w*w*A);
  const Eigen::Vector3d S=A+B*dt+C*dt*dt;
  const Eigen::Vector3d D=B+2.*C*dt;
  const double decay=std::exp(-w*dt);
  r.p=target+S*decay;
  r.v=(D-w*S)*decay;
  r.a=(2.*C-2.*w*D+w*w*S)*decay;
}
} // namespace velocity_ik
