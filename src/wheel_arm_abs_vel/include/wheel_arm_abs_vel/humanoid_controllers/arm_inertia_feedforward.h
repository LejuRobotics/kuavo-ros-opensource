#pragma once
#include <Eigen/Core>
#include <cmath>
namespace humanoid_controller {
// Stateless gain on the ALREADY admitted arm acceleration only. This never
// edits q/dq, lower-body acceleration, reference validity or safety ownership.
inline bool scaleArmInertiaFeedforward(Eigen::VectorXd* acceleration,
                                      int arms,double scale) {
  if (!acceleration || arms<0 || arms>acceleration->size() ||
      !std::isfinite(scale) || scale<0. || scale>1. || !acceleration->allFinite())return false;
  if (scale!=1.) acceleration->tail(arms)*=scale;
  return true;
}
} // namespace humanoid_controller
