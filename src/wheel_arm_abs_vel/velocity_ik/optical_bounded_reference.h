#pragma once
#include "fixed_bandwidth_reference.h"

namespace velocity_ik {
// Reuse the reference activity gate and its acceleration-preserving bandwidth
// transition. Activity follows the continuous reference velocity, never a
// derivative of the noisy/held optical packets.
struct OpticalAdaptiveCutoff {
  AdaptiveReferenceGate gate;
  ReferenceParameterTransition transition;
  Eigen::Vector3d acceleration=Eigen::Vector3d::Zero();
  bool initialized=false;
  void reset() { *this=OpticalAdaptiveCutoff{}; }
  double step(const Eigen::Vector3d& error, const Eigen::Vector3d& velocity,
              double dt, double slow, double fast, bool angular=false) {
    if(!initialized) { gate.activity=0.; initialized=true; }
    AdaptiveReferenceSettings settings;
    settings.min_frequency=slow;
    settings.speed_time_constant=.040;
    settings.attack_time_constant=.020;
    settings.release_time_constant=.150;
    AdaptiveReferenceLimits limits;
    limits.speed_start=angular ? .10 : .03;
    limits.speed_full=angular ? 1.20 : .30;
    gate.update(0.,velocity,dt,fast,settings,limits,true);
    transition.update(fixedNaturalHz(gate.frequency),0.,error,
        Eigen::Vector3d::Zero(),velocity,dt,300.,true);
    return transition.frequency*std::sqrt(std::sqrt(2.)-1.);
  }
};
namespace detail {
inline Eigen::Vector3d boundedOpticalVelocity(const Eigen::Vector3d& velocity,
                                              const Eigen::Vector3d& desired_acceleration,
                                              double h, double max_speed,
                                              double max_acceleration) {
  const double speed=velocity.norm();
  if(speed>max_speed) {
    // An inherited over-limit reference must brake continuously; projecting
    // it straight onto the speed sphere would violate the acceleration bound.
    const double brake=std::min(max_acceleration,(speed-max_speed)/h);
    return velocity-(brake*h/speed)*velocity;
  }
  Eigen::Vector3d acceleration=desired_acceleration;
  if(acceleration.norm()>max_acceleration)
    acceleration*=max_acceleration/acceleration.norm();
  Eigen::Vector3d next=velocity+h*acceleration;
  if(next.norm()>max_speed)
    next*=max_speed/next.norm();
  return next;
}
}  // namespace detail

inline void stepBoundedOpticalVector(ContinuousVector& r,
                                    const Eigen::Vector3d& target, double dt,
                                    double cutoff, double max_speed,
                                    double max_acceleration) {
  if(!r.valid)r.reset(target);
  const double w=2.*3.14159265358979323846*fixedNaturalHz(cutoff);
  const int steps=std::max(1,int(std::ceil(dt/.0005)));
  const double h=dt/steps;
  for(int i=0;i<steps;++i) {
    const Eigen::Vector3d acceleration=w*w*(target-r.p)-2.*w*r.v;
    const Eigen::Vector3d next=detail::boundedOpticalVelocity(
        r.v,acceleration,h,max_speed,max_acceleration);
    r.p+=.5*(r.v+next)*h;
    r.v=next;
  }
}

inline void stepBoundedOpticalRotation(ContinuousRotation& r,
                                      const Eigen::Quaterniond& target, double dt,
                                      double cutoff, double max_speed,
                                      double max_acceleration) {
  if(!r.valid)r.reset(target);
  const double w=2.*3.14159265358979323846*fixedNaturalHz(cutoff);
  const int steps=std::max(1,int(std::ceil(dt/.0005)));
  const double h=dt/steps;
  for(int i=0;i<steps;++i) {
    const Eigen::Vector3d acceleration=
        w*w*fixedRotationError(target,r.q)-2.*w*r.v;
    const Eigen::Vector3d next=detail::boundedOpticalVelocity(
        r.v,acceleration,h,max_speed,max_acceleration);
    r.q=fixedRotationAdvance(r.q,.5*(r.v+next)*h);
    r.v=next;
  }
}

// Bandwidth release alone makes held observation updates more visible. Shape
// acceleration too, while preserving one position/velocity trajectory. Hard
// speed/acceleration bounds take priority if an inherited state hits a cap.
inline Eigen::Vector3d smoothOpticalAcceleration(OpticalAdaptiveCutoff& activity,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& proposed_velocity,
    double dt,double speed,double acceleration,double jerk) {
  const Eigen::Vector3d desired=(proposed_velocity-velocity)/dt;
  Eigen::Vector3d delta=desired-activity.acceleration;
  if(delta.norm()>jerk*dt)delta*=jerk*dt/delta.norm();
  const Eigen::Vector3d next=detail::boundedOpticalVelocity(
      velocity,activity.acceleration+delta,dt,speed,acceleration);
  activity.acceleration=(next-velocity)/dt;
  return next;
}

inline void stepAdaptiveOpticalVector(ContinuousVector& r,
    const Eigen::Vector3d& target,double dt,double cutoff,double speed,
    double acceleration,OpticalAdaptiveCutoff& activity) {
  if(!r.valid) { r.reset(target);activity.acceleration.setZero(); }
  auto proposed=r;
  stepBoundedOpticalVector(proposed,target,dt,cutoff,speed,acceleration);
  const Eigen::Vector3d next=smoothOpticalAcceleration(
      activity,r.v,proposed.v,dt,speed,acceleration,500.);
  r.p+=.5*(r.v+next)*dt;r.v=next;
}

inline void stepAdaptiveOpticalRotation(ContinuousRotation& r,
    const Eigen::Quaterniond& target,double dt,double cutoff,double speed,
    double acceleration,OpticalAdaptiveCutoff& activity) {
  if(!r.valid) { r.reset(target);activity.acceleration.setZero(); }
  auto proposed=r;
  stepBoundedOpticalRotation(proposed,target,dt,cutoff,speed,acceleration);
  const Eigen::Vector3d next=smoothOpticalAcceleration(
      activity,r.v,proposed.v,dt,speed,acceleration,1000.);
  r.q=fixedRotationAdvance(r.q,.5*(r.v+next)*dt);r.v=next;
}
}  // namespace velocity_ik
