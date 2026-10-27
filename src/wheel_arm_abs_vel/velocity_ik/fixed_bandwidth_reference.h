#pragma once
#include "continuous_reference.h"

namespace velocity_ik {
// Actual -3dB cutoff of a critically damped second-order lowpass.
// No age lead, velocity-input bypass, innovation integrator or activity gate.
inline double fixedNaturalHz(double cutoff) {
  return cutoff / std::sqrt(std::sqrt(2.) - 1.);
}
template<class Ref>
inline void fixedReferenceDiagnostics(Ref& r, double cutoff, double error) {
  r.innovation.valid=false;
  r.gate.activity=0.; r.gate.frequency=fixedNaturalHz(cutoff);
  r.gate.error=error; r.gate.speed=r.v.norm(); r.gate.guide=0.;
  r.transition.frequency=fixedNaturalHz(cutoff); r.transition.guide=0.;
  r.transition.acceleration_change=0.; r.transition.limited=false;
  r.transition.valid=true;
}
inline void stepFixedVector(AdaptiveVectorReference& r,
                            const Eigen::Vector3d& target, double dt, double cutoff) {
  // Exact ZOH solution. v is the derivative of this same trajectory.
  r.step(target,Eigen::Vector3d::Zero(),dt,fixedNaturalHz(cutoff));
  fixedReferenceDiagnostics(r,cutoff,(target-r.p).norm());
}
inline Eigen::Vector3d fixedRotationError(const Eigen::Quaterniond& target,
                                        const Eigen::Quaterniond& q) {
  Eigen::Quaterniond d=(target.normalized()*q.conjugate()).normalized();
  if(d.w()<0.)d.coeffs()*=-1.;
  const Eigen::AngleAxisd a(d);
  return a.angle()*a.axis();
}
inline Eigen::Quaterniond fixedRotationAdvance(const Eigen::Quaterniond& q,
                                               const Eigen::Vector3d& rotation) {
  const double angle=rotation.norm();
  if(angle<1.e-14)return q;
  return (Eigen::Quaterniond(Eigen::AngleAxisd(angle,rotation/angle))*q).normalized();
}
inline void stepFixedRotation(AdaptiveRotationReference& r,
                              const Eigen::Quaterniond& target, double dt, double cutoff) {
  if(!r.valid)r.reset(target);
  // World/spatial angular velocity; midpoint integration on SO(3).
  // The linearized transfer function has exactly the stated cutoff.
  const double w=2.*3.14159265358979323846*fixedNaturalHz(cutoff);
  const int steps=std::max(1,int(std::ceil(dt/.0005)));
  const double h=dt/steps;
  for(int i=0;i<steps;++i) {
    const Eigen::Vector3d a=w*w*fixedRotationError(target,r.q)-2.*w*r.v;
    const Eigen::Vector3d vm=r.v+.5*h*a;
    const Eigen::Quaterniond qm=fixedRotationAdvance(r.q,.5*h*r.v);
    const Eigen::Vector3d am=w*w*fixedRotationError(target,qm)-2.*w*vm;
    r.q=fixedRotationAdvance(r.q,h*vm);
    r.v+=h*am;
  }
  fixedReferenceDiagnostics(r,cutoff,fixedRotationError(target,r.q).norm());
}
} // namespace velocity_ik
