#pragma once
#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <stdexcept>
namespace wheel_arm_abs_vel::direct_tcp {
// Quest transports float quaternions. Accept only small numerical drift, then
// normalize once at the transport boundary; the core remains strictly unit.
// This is representation cleanup, not a pose filter or an anchor correction.
inline Sample questTransportSample(const std::array<double,7>& values) {
  Eigen::Quaterniond q(values[6],values[3],values[4],values[5]);
  Eigen::Map<const Eigen::Matrix<double,7,1>> all(values.data());
  const double norm2=q.squaredNorm();
  if(!all.allFinite() || !std::isfinite(norm2) || std::abs(norm2-1.)>1.e-4)
    throw std::invalid_argument("Invalid Quest position/quaternion (zero, nonfinite or outside numerical tolerance)");
  q.normalize();auto x=values;x[3]=q.x();x[4]=q.y();x[5]=q.z();x[6]=q.w();
  return sample(x,Vec::Zero(),Vec::Zero());
}
// Exact inverse of the existing IK's shoulder-translation alignment. The
// incoming command remains a robot-base TCP; this is a wire-frame conversion.
inline Sample sourceCoordinates(const Sample& base, const Sample& torso,
                                const Vec& shoulder_in_torso, const Vec& source_shoulder) {
  Sample s=base; const Pose torso_from_base=torso.pose.inverse();
  s.pose=torso_from_base*base.pose;
  s.pose.translation()-=shoulder_in_torso-source_shoulder;
  s.linear=torso_from_base.linear()*(base.linear-torso.linear-torso.angular.cross(base.pose.translation()-torso.pose.translation()));
  s.angular=torso_from_base.linear()*(base.angular-torso.angular);
  return s;
}
inline Sample poseDerivative(const Sample& now,const Sample& before,double dt) {
  if (!std::isfinite(dt) || dt<=0.) throw std::invalid_argument("Non-increasing receiver timestamp");
  Sample s=now;s.linear=(now.pose.translation()-before.pose.translation())/dt;
  Eigen::AngleAxisd rotation(now.pose.linear()*before.pose.linear().transpose());
  s.angular=rotation.angle()*rotation.axis()/dt;return s;
}
}  // namespace
