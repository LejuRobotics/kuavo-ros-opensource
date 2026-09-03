#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"

#include <cmath>
#include <iostream>

namespace {

constexpr double kTolerance = 1.0e-12;

bool expectNear(double actual, double expected, const char* label) {
  const double error = std::abs(actual - expected);
  if (error < kTolerance) return true;
  std::cerr << label << " error: " << error << std::endl;
  return false;
}

}  // namespace

int main() {
  using ocs2::mobile_manipulator::KinemicLimitFilter;

  constexpr int kDof = 4;
  constexpr double kDt = 0.01;
  KinemicLimitFilter filter(kDof, kDt);
  KinemicLimitFilter control(kDof, kDt);

  const Eigen::VectorXd velocityLimit = Eigen::VectorXd::Constant(kDof, 4.0);
  const Eigen::VectorXd accelerationLimit = Eigen::VectorXd::Constant(kDof, 20.0);
  const Eigen::VectorXd jerkLimit = Eigen::VectorXd::Constant(kDof, 100.0);
  for (KinemicLimitFilter* candidate : {&filter, &control}) {
    candidate->setFirstOrderDerivativeLimit(velocityLimit);
    candidate->setSecondOrderDerivativeLimit(accelerationLimit);
    candidate->setThirdOrderDerivativeLimit(jerkLimit);
  }

  const Eigen::VectorXd initial =
      (Eigen::VectorXd(kDof) << 0.10, 0.20, 0.30, 0.40).finished();
  filter.reset(initial);
  control.reset(initial);

  const Eigen::VectorXd movingTarget =
      (Eigen::VectorXd(kDof) << 0.80, -0.50, 1.10, -0.70).finished();
  for (int i = 0; i < 8; ++i) {
    filter.update(movingTarget);
    control.update(movingTarget);
  }

  const Eigen::VectorXd velocityBeforeReset = filter.getFirstOrderDerivative();
  const Eigen::VectorXd accelerationBeforeReset = filter.getSecondOrderDerivative();
  const Eigen::VectorXd resetValues =
      (Eigen::VectorXd(2) << -0.25, 0.65).finished();

  bool success = filter.resetRange(1, resetValues);
  const Eigen::VectorXd velocityAfterReset = filter.getFirstOrderDerivative();
  const Eigen::VectorXd accelerationAfterReset = filter.getSecondOrderDerivative();
  success &= expectNear(velocityAfterReset(1), 0.0, "selected velocity 1 reset");
  success &= expectNear(velocityAfterReset(2), 0.0, "selected velocity 2 reset");
  success &= expectNear(accelerationAfterReset(1), 0.0, "selected acceleration 1 reset");
  success &= expectNear(accelerationAfterReset(2), 0.0, "selected acceleration 2 reset");
  success &= expectNear(velocityAfterReset(0), velocityBeforeReset(0),
                        "left outside velocity preservation");
  success &= expectNear(velocityAfterReset(3), velocityBeforeReset(3),
                        "right outside velocity preservation");
  success &= expectNear(accelerationAfterReset(0), accelerationBeforeReset(0),
                        "left outside acceleration preservation");
  success &= expectNear(accelerationAfterReset(3), accelerationBeforeReset(3),
                        "right outside acceleration preservation");

  Eigen::VectorXd nextTarget = movingTarget;
  nextTarget.segment(1, 2) = resetValues;
  const Eigen::VectorXd resetOutput = filter.update(nextTarget);
  const Eigen::VectorXd controlOutput = control.update(nextTarget);
  success &= expectNear(resetOutput(1), resetValues(0), "selected position 1 reset");
  success &= expectNear(resetOutput(2), resetValues(1), "selected position 2 reset");

  // Ruckig DOFs are independent in this filter.  The untouched DOFs must
  // evolve exactly like an identical control filter that was never rebased.
  success &= expectNear(resetOutput(0), controlOutput(0), "left outside trajectory preservation");
  success &= expectNear(resetOutput(3), controlOutput(3), "right outside trajectory preservation");

  // Rejected ranges are non-mutating.
  const Eigen::VectorXd velocityBeforeReject = filter.getFirstOrderDerivative();
  const Eigen::VectorXd accelerationBeforeReject = filter.getSecondOrderDerivative();
  success &= !filter.resetRange(4, Eigen::VectorXd::Ones(1));
  success &= expectNear((filter.getFirstOrderDerivative() - velocityBeforeReject).norm(),
                        0.0,
                        "invalid range velocity mutation");
  success &= expectNear((filter.getSecondOrderDerivative() - accelerationBeforeReject).norm(),
                        0.0,
                        "invalid range acceleration mutation");

  if (!success) return 1;
  std::cout << "wheel_kinemic_limit_filter_range_reset_test: PASS" << std::endl;
  return 0;
}
