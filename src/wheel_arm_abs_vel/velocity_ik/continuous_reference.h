#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include "innovation_reference.h"

namespace velocity_ik {
// Causal reference state, never a queue. Position and exported velocity are
// one trajectory. Exact critical-damping step for a constant-velocity input.
struct ContinuousVector {
  Eigen::Vector3d p=Eigen::Vector3d::Zero(), v=Eigen::Vector3d::Zero();
  bool valid=false;
  void reset(const Eigen::Vector3d& position) {p=position;v.setZero();valid=true;}
  void step(const Eigen::Vector3d& target, const Eigen::Vector3d& velocity,
            double dt, double hz) {
    if(!valid) reset(target);
    const double w=2.*3.14159265358979323846*hz;
    const Eigen::Vector3d e=p-(target-velocity*dt), d=v-velocity, c=d+w*e;
    const double decay=std::exp(-w*dt);
    p=target+(e+c*dt)*decay;
    v=velocity+(d-w*c*dt)*decay;
  }
};
struct ContinuousRotation {
  Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
  Eigen::Vector3d v=Eigen::Vector3d::Zero();
  bool valid=false;
  void reset(const Eigen::Quaterniond& rotation) {q=rotation.normalized();v.setZero();valid=true;}
  void step(const Eigen::Quaterniond& target, const Eigen::Vector3d& omega,
            double dt, double hz) {
    if(!valid) reset(target);
    // Small fixed substeps also cover occasional delayed host cycles.
    const int steps=std::max(1,int(std::ceil(dt/.002)));
    const double h=dt/steps,w=2.*3.14159265358979323846*hz;
    for(int i=0;i<steps;++i) {
      Eigen::Quaterniond delta=target*q.conjugate();
      if(delta.w()<0) delta.coeffs()*=-1;
      const Eigen::AngleAxisd angle(delta);
      const Eigen::Vector3d a=w*w*angle.angle()*angle.axis()+2.*w*(omega-v);
      const Eigen::Vector3d next=v+a*h, rotation=.5*(v+next)*h;
      if(rotation.norm()>1e-12)
        q=Eigen::Quaterniond(Eigen::AngleAxisd(rotation.norm(),rotation.normalized()))*q;
      q.normalize();v=next;
    }
  }
};

// Startup-only tuning. Error thresholds are SOFT bandwidth-release scales,
// not dead zones, certified tracking bounds, or actuator safety limits.
struct AdaptiveReferenceLimits {
  double error_start=.004, error_full=.012;
  double speed_start=.030, speed_full=.120;
  bool valid() const {
    return std::isfinite(error_start) && std::isfinite(error_full) &&
        std::isfinite(speed_start) && std::isfinite(speed_full) &&
        error_start>0. && error_full>error_start &&
        speed_start>=0. && speed_full>speed_start;
  }
};
struct AdaptiveReferenceSettings {
  InnovationReferenceSettings innovation;
  double min_frequency=1.0;
  double speed_time_constant=.020;
  double attack_time_constant=.010, release_time_constant=.200;
  // Limit only acceleration changes caused by bandwidth/guide modulation.
  // This is NOT a global Cartesian acceleration/jerk or hand speed cap.
  bool bumpless_enabled=true;
  double position_parameter_jerk=300., orientation_parameter_jerk=300.;
  double active_parameter_jerk_multiplier=8.;
  AdaptiveReferenceLimits position;
  // radians and radians/s: 0.5/1.5 deg, 3/15 deg/s.
  AdaptiveReferenceLimits orientation{.00872664625997165,.026179938779915,
                                      .05235987755983,.261799387799149};
  bool valid(double fast_frequency) const {
    return innovation.valid() && std::isfinite(fast_frequency) && std::isfinite(min_frequency) &&
        min_frequency>=.25 && min_frequency<=fast_frequency &&
        std::isfinite(speed_time_constant) && speed_time_constant>0. &&
        std::isfinite(attack_time_constant) && attack_time_constant>0. &&
        std::isfinite(release_time_constant) &&
        release_time_constant>=attack_time_constant &&
        std::isfinite(position_parameter_jerk) && position_parameter_jerk>0. &&
        std::isfinite(orientation_parameter_jerk) && orientation_parameter_jerk>0. &&
        std::isfinite(active_parameter_jerk_multiplier) && active_parameter_jerk_multiplier>=1. &&
        active_parameter_jerk_multiplier<=10. &&
        position.valid() && orientation.valid();
  }
};

struct ReferenceParameterTransition {
  double frequency=12., guide=.1, acceleration_change=0.;
  bool valid=false, limited=false;
  void reset() { *this=ReferenceParameterTransition{}; }
  void update(double requested_frequency, double requested_guide,
              const Eigen::Vector3d& error, const Eigen::Vector3d& input_velocity,
              const Eigen::Vector3d& reference_velocity, double dt,
              double jerk, bool enabled) {
    limited=false;acceleration_change=0.;
    if (!valid || !enabled) {
      frequency=requested_frequency;guide=requested_guide;valid=true;return;
    }
    const auto acceleration=[&](double fraction) -> Eigen::Vector3d {
      const double hz=frequency+fraction*(requested_frequency-frequency);
      const double g=guide+fraction*(requested_guide-guide);
      const double w=2.*3.14159265358979323846*hz;
      return w*w*error+2.*w*(g*input_velocity-reference_velocity);
    };
    const Eigen::Vector3d baseline=acceleration(0.);
    const double budget=jerk*dt;
    double fraction=1.;
    if ((acceleration(1.)-baseline).norm()>budget) {
      limited=true;
      // Conservative prefix bound: a(f)-a(0)=f*B+f²*C. Norm triangle
      // inequality avoids assuming a vector norm is monotonic in f.
      const double dw=2.*3.14159265358979323846*(requested_frequency-frequency);
      const double w=2.*3.14159265358979323846*frequency;
      const double dg=requested_guide-guide;
      const Eigen::Vector3d B=2.*w*dw*error+
          2.*dw*(guide*input_velocity-reference_velocity)+2.*w*dg*input_velocity;
      const Eigen::Vector3d C=dw*dw*error+2.*dw*dg*input_velocity;
      const double b=B.norm(),c=C.norm();
      fraction=std::clamp(2.*budget/(b+std::sqrt(b*b+4.*c*budget)+1e-30),0.,1.);
    }
    acceleration_change=(acceleration(fraction)-baseline).norm();
    frequency+=fraction*(requested_frequency-frequency);
    guide+=fraction*(requested_guide-guide);
  }
};

struct AdaptiveReferenceGate {
  // Start permissive: no extra slow takeover after transaction/reset.
  double activity=1., frequency=12., error=0., speed=0., guide=.1;
  Eigen::Vector3d filtered_velocity=Eigen::Vector3d::Zero();
  void reset() { *this=AdaptiveReferenceGate{}; }
  static double smoothRelease(double value, double start, double full) {
    const double u=std::clamp((value-start)/(full-start),0.,1.);
    return u*u*(3.-2.*u);
  }
  void update(double position_error, const Eigen::Vector3d& velocity,
              double dt, double fast_frequency,
              const AdaptiveReferenceSettings& settings,
              const AdaptiveReferenceLimits& limits, bool enabled) {
    error=position_error;
    if (!enabled) {
      activity=1.; frequency=fast_frequency; guide=.1;
      filtered_velocity=velocity; speed=velocity.norm(); return;
    }
    // Filter the vector before its norm; raw noise magnitude must not
    // instantly open the bandwidth. No numerical pose differentiation.
    filtered_velocity+=(1.-std::exp(-dt/settings.speed_time_constant))*
        (velocity-filtered_velocity);
    speed=filtered_velocity.norm();
    const double desired=std::max(
        smoothRelease(error,limits.error_start,limits.error_full),
        smoothRelease(speed,limits.speed_start,limits.speed_full));
    const double tau=desired>activity ? settings.attack_time_constant
                                      : settings.release_time_constant;
    activity=std::clamp(activity+(1.-std::exp(-dt/tau))*(desired-activity),0.,1.);
    frequency=settings.min_frequency+(fast_frequency-settings.min_frequency)*activity;
    guide=.1*activity;
  }
};

// These wrappers are used by the node AND the offline replay/tests. The
// exported velocity always belongs to the filtered trajectory; unfiltered
// input feedforward cannot bypass the shaper, even at low activity.
struct AdaptiveVectorReference : ContinuousVector {
  InnovationVectorReference innovation;
  AdaptiveReferenceGate gate;
  ReferenceParameterTransition transition;
  void stepAdaptive(const Eigen::Vector3d& target, const Eigen::Vector3d& velocity,
                    double dt, double fast_frequency, bool enabled,
                    const AdaptiveReferenceSettings& settings) {
    if (!valid) { gate.reset();transition.reset();innovation.valid=false; }
    if(settings.innovation.enabled) {
      // A positional observation correction must not itself open the fast
      // motion bandwidth. Latest residual is handled by its own budget.
      gate.update(0.,velocity,dt,fast_frequency,settings,settings.position,enabled);
      gate.guide=1.;
      auto correction_limits=settings.innovation.position;
      correction_limits.frequency=std::min(correction_limits.frequency,gate.frequency);
      innovation.step(target,velocity,dt,gate.frequency,correction_limits);
      p=innovation.p;v=innovation.v;valid=true;
      transition.frequency=gate.frequency;transition.guide=1.;
      transition.acceleration_change=0.;transition.limited=false;transition.valid=true;
      return;
    }
    gate.update(valid ? (target-p).norm() : 0.,velocity,dt,fast_frequency,
                settings,settings.position,enabled);
    const double active=AdaptiveReferenceGate::smoothRelease(gate.speed,
        settings.position.speed_start,settings.position.speed_full);
    const double jerk=settings.position_parameter_jerk*
        (1.+(settings.active_parameter_jerk_multiplier-1.)*active);
    transition.update(gate.frequency,gate.guide,valid ? Eigen::Vector3d(target-p) : Eigen::Vector3d::Zero(),
        velocity,v,dt,jerk,settings.bumpless_enabled && enabled);
    step(target,transition.guide*velocity,dt,transition.frequency);
  }
};
struct AdaptiveRotationReference : ContinuousRotation {
  InnovationRotationReference innovation;
  AdaptiveReferenceGate gate;
  ReferenceParameterTransition transition;
  void stepAdaptive(const Eigen::Quaterniond& target, const Eigen::Vector3d& velocity,
                    double dt, double fast_frequency, bool enabled,
                    const AdaptiveReferenceSettings& settings) {
    if (!valid) { gate.reset();transition.reset();innovation.valid=false; }
    if(settings.innovation.enabled) {
      gate.update(0.,velocity,dt,fast_frequency,settings,settings.orientation,enabled);
      gate.guide=1.;
      auto correction_limits=settings.innovation.orientation;
      correction_limits.frequency=std::min(correction_limits.frequency,gate.frequency);
      innovation.step(target,velocity,dt,gate.frequency,correction_limits);
      q=innovation.q;v=innovation.v;valid=true;
      transition.frequency=gate.frequency;transition.guide=1.;
      transition.acceleration_change=0.;transition.limited=false;transition.valid=true;
      return;
    }
    gate.update(valid ? q.angularDistance(target) : 0.,velocity,dt,fast_frequency,
                settings,settings.orientation,enabled);
    Eigen::Quaterniond delta=target*q.conjugate();
    if(delta.w()<0.)delta.coeffs()*=-1.;
    const Eigen::AngleAxisd angle(delta);
    const Eigen::Vector3d error=valid ? Eigen::Vector3d(angle.angle()*angle.axis()) : Eigen::Vector3d::Zero();
    const double active=AdaptiveReferenceGate::smoothRelease(gate.speed,
        settings.orientation.speed_start,settings.orientation.speed_full);
    const double jerk=settings.orientation_parameter_jerk*
        (1.+(settings.active_parameter_jerk_multiplier-1.)*active);
    transition.update(gate.frequency,gate.guide,error,velocity,v,dt,jerk,
        settings.bumpless_enabled && enabled);
    step(target,transition.guide*velocity,dt,transition.frequency);
  }
};
} // namespace velocity_ik
