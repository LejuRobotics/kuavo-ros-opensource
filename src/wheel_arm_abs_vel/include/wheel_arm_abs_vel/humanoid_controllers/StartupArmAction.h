#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace wheel_absolute_control {
struct StartupArmSample { Eigen::VectorXd q, v; bool complete; };
// Same configured out-and-back action as dev, with zero endpoint velocity.
inline StartupArmSample sampleStartupArmAction(const Eigen::VectorXd& start,
    const Eigen::VectorXd& delta, double elapsed, double duration) {
  if (start.size()!=delta.size() || !start.allFinite() || !delta.allFinite() ||
      !std::isfinite(elapsed) || elapsed<0 || !std::isfinite(duration) || duration<=0)
    throw std::invalid_argument("invalid startup action");
  const double half=duration/2.;
  const bool returning=elapsed>=half;
  const double u=std::clamp((returning ? elapsed-half : elapsed)/half,0.,1.);
  const double h=u*u*u*(10.+u*(-15.+6.*u));
  const double dh=30.*u*u*(1.-u)*(1.-u)/half;
  return {start+(returning ? 1.-h : h)*delta,
          (returning ? -dh : dh)*delta, elapsed>=duration};
}
}
