#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "frame_prediction.h"

namespace velocity_ik {
// Budgets apply ONLY to the positional innovation correction, never to the
// nominal motion or to the final joint command. Axes use an inscribed box so
// the vector norms also satisfy the configured budgets.
struct InnovationLimits {
  double frequency=12., damping=1., speed=3., acceleration=36., jerk=3600.;
  bool valid() const {
    return std::isfinite(frequency) && frequency>=.25 && frequency<=20. &&
      std::isfinite(damping) && damping>=1. && damping<=4. &&
      std::isfinite(speed) && speed>0. && std::isfinite(acceleration) &&
      acceleration>0. && std::isfinite(jerk) && jerk>0.;
  }
};
struct InnovationReferenceSettings {
  // Legacy unit tests/replay keep their old path unless explicitly selected.
  bool enabled=false;
  InnovationLimits position;
  InnovationLimits orientation{12.,1.,9.,108.,10800.};
  bool valid() const {return position.valid() && orientation.valid();}
};

// Critically damped velocity tracker with exact constant-input integration.
// v and a are continuous at observation AND bandwidth changes. Its nominal
// position lag at fixed bandwidth is 2*v/w+a/w^2. Subtracting that modeled
// lag from the position residual prevents a second loop from chasing the
// intentional motion-filter delay (and recreating a resonant gain).
struct InnovationMotion {
  Eigen::Vector3d v=Eigen::Vector3d::Zero(),a=Eigen::Vector3d::Zero();
  Eigen::Vector3d step(const Eigen::Vector3d& input,double dt,double hz) {
    const double w=2.*3.14159265358979323846*hz,z=w*dt,decay=std::exp(-z);
    const double i0=-std::expm1(-z)/w;
    const double i1=(-std::expm1(-z)-z*decay)/(w*w);
    const Eigen::Vector3d e=v-input,c=a+w*e;
    const Eigen::Vector3d displacement=input*dt+e*i0+c*i1;
    v=input+(e+c*dt)*decay;
    a=(a-w*c*dt)*decay;
    return displacement;
  }
  Eigen::Vector3d lag(double hz) const {
    const double w=2.*3.14159265358979323846*hz;
    return 2.*v/w+a/(w*w);
  }
};

struct InnovationCorrection {
  Eigen::Vector3d v=Eigen::Vector3d::Zero(),a=Eigen::Vector3d::Zero();
  Eigen::Vector3d requested_acceleration=Eigen::Vector3d::Zero();
  bool limited=false;
  Eigen::Vector3d step(const Eigen::Vector3d& error,double dt,
                       const InnovationLimits& limits) {
    const double w=2.*3.14159265358979323846*limits.frequency;
    // Third-order error dynamics, not a second-order acceleration request
    // followed by an unmodeled rate limiter. At damping=1 the unsaturated
    // characteristic is (s+w)^3; step correction starts with continuous a.
    const Eigen::Vector3d desired_jerk=w*w*w*error-3.*limits.damping*w*w*v-3.*limits.damping*w*a;
    requested_acceleration=a+dt*desired_jerk;
    Eigen::Vector3d displacement;limited=false;
    const double scale=1./std::sqrt(3.);
    const double V=limits.speed*scale,A=limits.acceleration*scale,J=limits.jerk*scale;
    for(int i=0;i<3;++i) {
      double lo=std::max(-A,a[i]-J*dt),hi=std::min(A,a[i]+J*dt);
      const auto next_v=[&](double next_a){return v[i]+.5*(a[i]+next_a)*dt;};
      // Preserve enough velocity headroom to stop acceleration with bounded
      // jerk on following ticks. A final output clip would hide windup and
      // violate trajectory consistency; constrain the actual internal state.
      const auto upper=[&](double x){const double b=std::max(0.,x);
        double r=next_v(x)+b*b/(2.*J);
        if(a[i]>0. && x<0.)r=std::max(r,v[i]+a[i]*a[i]*dt/(2.*(a[i]-x)));
        return r;};
      const auto lower=[&](double x){const double b=std::min(0.,x);
        double r=next_v(x)-b*b/(2.*J);
        if(a[i]<0. && x>0.)r=std::min(r,v[i]-a[i]*a[i]*dt/(2.*(x-a[i])));
        return r;};
      if(upper(hi)>V) {
        double l=lo,h=hi;for(int k=0;k<40;++k){const double m=.5*(l+h);if(upper(m)>V)h=m;else l=m;}hi=l;
      }
      if(lower(lo)<-V) {
        double l=lo,h=hi;for(int k=0;k<40;++k){const double m=.5*(l+h);if(lower(m)<-V)l=m;else h=m;}lo=h;
      }
      const double next_a=std::clamp(requested_acceleration[i],lo,std::max(lo,hi));
      limited=limited || std::abs(next_a-requested_acceleration[i])>1e-9;
      displacement[i]=v[i]*dt+.5*a[i]*dt*dt+(next_a-a[i])*dt*dt/6.;
      v[i]=next_v(next_a);a[i]=next_a;
    }
    return displacement;
  }
};

inline Eigen::Quaterniond innovationExp(const Eigen::Vector3d& phi) {
  const double angle=phi.norm();
  return angle>1e-12?Eigen::Quaterniond(Eigen::AngleAxisd(angle,phi/angle)):Eigen::Quaterniond::Identity();
}
inline Eigen::Vector3d innovationLog(Eigen::Quaterniond delta) {
  delta.normalize();if(delta.w()<0.)delta.coeffs()*=-1.;
  const Eigen::AngleAxisd angle(delta);return angle.angle()*angle.axis();
}
// Read-only audit. Both anchors are evaluated at the SAME host time; do not
// confuse normal frame-to-frame displacement with observation innovation.
struct ReferenceInnovationAudit {
  bool valid=false;
  double update_ns=0.;
  FramePrediction anchor;
  Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero(),w=Eigen::Vector3d::Zero();
  Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
  Eigen::Vector3d dp=Eigen::Vector3d::Zero(),dr=Eigen::Vector3d::Zero(),dv=Eigen::Vector3d::Zero(),dw=Eigen::Vector3d::Zero();
  void update(const FramePrediction& next,const FramePredictionSettings& cfg,double now,
              const Eigen::Vector3d& position,const Eigen::Quaterniond& rotation,
              const Eigen::Vector3d& velocity,const Eigen::Vector3d& omega) {
    if(!next.valid || (valid && next.observation_ns<=anchor.observation_ns))return;
    if(valid) {
      auto previous=anchor;
      previous.update(anchor.observation_ns,anchor.received_ns,anchor.source_stamp,now,cfg);
      dp=position+next.age*velocity-(p+previous.age*v);
      dr=innovationLog(innovationExp(next.age*omega)*rotation*(innovationExp(previous.age*w)*q).conjugate());
      dv=next.rate*velocity-previous.rate*v;dw=next.rate*omega-previous.rate*w;
      update_ns=now;
    }
    p=position;q=rotation;v=velocity;w=omega;anchor=next;valid=true;
  }
};
inline void innovationValidate(double dt,double hz) {
  if(!std::isfinite(dt) || dt<=0. || dt>.020 || !std::isfinite(hz) || hz<.25 || hz>20.)
    throw std::invalid_argument("invalid innovation reference step");
}
struct InnovationVectorReference {
  bool valid=false;
  Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero();
  Eigen::Vector3d residual=Eigen::Vector3d::Zero(),modeled_lag=Eigen::Vector3d::Zero();
  InnovationMotion motion;InnovationCorrection correction;
  void reset(const Eigen::Vector3d& target) {
    *this=InnovationVectorReference{};p=target;valid=true;
  }
  void step(const Eigen::Vector3d& target,const Eigen::Vector3d& input,
            double dt,double hz,const InnovationLimits& limits) {
    innovationValidate(dt,hz);
    if(!target.allFinite() || !input.allFinite()) throw std::invalid_argument("nonfinite innovation target");
    if(!valid)reset(target);
    // Small substeps bound the nonlinear budget integration, not robot ticks.
    const int n=std::max(1,int(std::ceil(dt/.002)));const double h=dt/n;
    for(int k=0;k<n;++k) {
      const Eigen::Vector3d nominal=motion.step(input,h,hz);
      modeled_lag=motion.lag(hz);
      const Eigen::Vector3d endpoint=target-input*((n-1-k)*h);
      residual=endpoint-(p+nominal)-modeled_lag;
      p+=nominal+correction.step(residual,h,limits);
      v=motion.v+correction.v;
    }
  }
};
struct InnovationRotationReference {
  bool valid=false;
  Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
  Eigen::Vector3d v=Eigen::Vector3d::Zero(),residual=Eigen::Vector3d::Zero(),modeled_lag=Eigen::Vector3d::Zero();
  InnovationMotion motion;InnovationCorrection correction;
  void reset(const Eigen::Quaterniond& target) {
    *this=InnovationRotationReference{};q=target.normalized();valid=true;
  }
  void step(const Eigen::Quaterniond& target,const Eigen::Vector3d& input,
            double dt,double hz,const InnovationLimits& limits) {
    innovationValidate(dt,hz);
    if(!target.coeffs().allFinite() || target.norm()<1e-10 || !input.allFinite())
      throw std::invalid_argument("invalid innovation rotation");
    if(!valid)reset(target);
    const int n=std::max(1,int(std::ceil(dt/.002)));const double h=dt/n;
    for(int k=0;k<n;++k) {
      const Eigen::Vector3d nominal=motion.step(input,h,hz);
      modeled_lag=motion.lag(hz);
      const Eigen::Quaterniond endpoint=innovationExp(-input*((n-1-k)*h))*target.normalized();
      const Eigen::Quaterniond nominal_q=innovationExp(nominal)*q;
      residual=innovationLog(innovationExp(-modeled_lag)*endpoint*nominal_q.conjugate());
      const Eigen::Vector3d correction_step=correction.step(residual,h,limits);
      q=(innovationExp(nominal+correction_step)*q).normalized();
      v=motion.v+correction.v;
    }
  }
};
} // namespace velocity_ik
