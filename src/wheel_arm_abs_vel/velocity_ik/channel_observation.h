#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace velocity_ik {
// Content changes are evidence of a changed redirected sample, not a new
// camera observation. Equal content holds this channel's estimate for a finite
// hold/decay interval. Neither another channel nor a control tick updates it.
struct ChannelObservation {
  Eigen::Vector3d position=Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear=Eigen::Vector3d::Zero(),angular=Eigen::Vector3d::Zero();
  double source_time=0, update_host_ns=0, difference_dt=0;
  uint64_t updates=0;
  bool valid=false, changed=false;
};
inline ChannelObservation observeChannel(
    const ChannelObservation& prior, const Eigen::Vector3d& p,
    const Eigen::Quaterniond& q, bool valid, bool continuous,
    double source_time, double host_ns, double tau,
    double position_epsilon, double orientation_epsilon) {
  ChannelObservation next;
  next.updates=prior.updates;
  if (!valid || !p.allFinite() || !q.coeffs().allFinite()) return next;
  const bool history=continuous && prior.valid;
  Eigen::Quaterniond delta=q*prior.orientation.conjugate();
  if (delta.w()<0) delta.coeffs()*=-1;
  const Eigen::AngleAxisd angle(delta);
  const Eigen::Vector3d rotation=angle.angle()*angle.axis();
  if (history && (p-prior.position).norm()<=position_epsilon &&
      rotation.norm()<=orientation_epsilon) {
    next=prior; next.changed=false; next.difference_dt=0; return next;
  }
  next.valid=true;next.changed=true;next.position=p;next.orientation=q;
  next.source_time=source_time;next.update_host_ns=host_ns;++next.updates;
  if (history) {
    next.difference_dt=source_time>prior.source_time && prior.source_time>0
        ? source_time-prior.source_time : (host_ns-prior.update_host_ns)*1e-9;
    if (next.difference_dt>=.003 && next.difference_dt<=.100) {
      const double alpha=std::clamp(next.difference_dt/std::max(next.difference_dt+tau,1e-6),0.,1.);
      next.linear=(1-alpha)*prior.linear+alpha*(p-prior.position)/next.difference_dt;
      next.angular=(1-alpha)*prior.angular+alpha*rotation/next.difference_dt;
    }
  }
  return next;
}
// A repeated packet does not create a new pose origin. Latency continues
// to use the packet's own receive time; prediction uses this channel's epoch.
inline double channelPredictionAge(const ChannelObservation& c,double now_ns,double lead) {
  if (!c.valid || c.update_host_ns<=0. || !std::isfinite(now_ns)) return 0.;
  return std::max(0.,(now_ns-c.update_host_ns)*1e-9)+lead;
}
inline double channelDecay(const ChannelObservation& c,double now_ns,double hold,double tau) {
  if (!c.valid) return 0.;
  const double age=std::max(0.,(now_ns-c.update_host_ns)*1e-9);
  return age<=hold ? 1. : std::exp(-(age-hold)/std::max(tau,1e-6));
}
}
