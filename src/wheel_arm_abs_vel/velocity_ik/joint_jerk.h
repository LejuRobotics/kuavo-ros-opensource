#pragma once
#include <algorithm>
#include <cmath>
namespace velocity_ik {
struct JointJerkInterval {
  bool valid=false, safety_relaxed=false;
  double lower=0., upper=0.;
};
// h is the reference owner's integration step. Units: v rad/s, a rad/s^2,
// jerk rad/s^3. Never infer h from VR or bag arrival timestamps.
inline JointJerkInterval jointJerkInterval(double lo,double hi,double v,
                                          double a,double h,double jerk) {
  JointJerkInterval out;
  if(!std::isfinite(lo)||!std::isfinite(hi)||lo>hi||!std::isfinite(v)||
     !std::isfinite(a)||!std::isfinite(h)||h<=0.||!std::isfinite(jerk)||jerk<=0.)return out;
  const double center=v+h*a, radius=jerk*h*h;
  if(!std::isfinite(center)||!std::isfinite(radius)||radius<=0.||
     !std::isfinite(center-radius)||!std::isfinite(center+radius))return out;
  out.lower=std::max(lo,center-radius);out.upper=std::min(hi,center+radius);
  if(out.lower>out.upper) {
    // Closest originally feasible point minimizes the necessary jerk excess.
    // Only the new smoothness preference yields; no original bound moves.
    out.lower=out.upper=std::clamp(center,lo,hi);out.safety_relaxed=true;
  }
  out.valid=true;return out;
}
}  // namespace velocity_ik
