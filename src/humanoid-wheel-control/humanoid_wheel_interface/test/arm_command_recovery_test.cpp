#include <cmath>
#include <iostream>
#include "humanoid_controllers/ArmTrajectoryInterpolator.h"
#include "humanoid_wheel_interface/filters/ConstantVelocityCommandKalmanFilter.h"

int main() {
  ros::Time::init();
  using ocs2::mobile_manipulator::ConstantVelocityCommandKalmanFilter;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(14), v = q;
  ConstantVelocityCommandKalmanFilter filter(14, 0.001);
  filter.setKinematicLimits(Eigen::VectorXd::Ones(14));
  filter.reset(q);
  // Large position/velocity discontinuities must recover without an output jump.
  Eigen::VectorXd target = Eigen::VectorXd::Constant(14, 0.5);
  for (int n = 0; n < 300; ++n) {
    const auto previous = q.eval();
    const double dt = (n % 2 == 0) ? 0.003 : 0.004;
    if (!filter.updateTarget(target, Eigen::VectorXd::Constant(14, 10.0)) ||
        !filter.step(q, v, dt) || !q.allFinite() || !v.allFinite() ||
        (q - previous).cwiseAbs().maxCoeff() > dt + 1e-12 ||
        v.cwiseAbs().maxCoeff() > 1.0 + 1e-12) {
      std::cerr << "Recovery exceeded kinematic limits\n";
      return 1;
    }
  }
  if ((q - target).cwiseAbs().maxCoeff() > 0.01) {
    std::cerr << "Recovery did not converge\n";
    return 1;
  }
  const auto previousQ = q.eval();
  const auto previousV = v.eval();
  if (!filter.predict(q, v, 0.007) ||
      (q - previousQ - previousV * 0.007).cwiseAbs().maxCoeff() > 1e-12) {
    std::cerr << "Prediction ignored actual elapsed time\n";
    return 1;
  }
  // Exercise position-only recovery (the q=1, dq=0 path seen in the bag).
  filter.setKalmanParameters(10.0, 1.0, 1.0, 0.001, 0.001, 1.0);
  q.setZero();
  filter.reset(q);
  filter.updateTarget(target, Eigen::VectorXd::Zero(14));
  if (!filter.step(q, v, 0.003) || (q.array() - 0.003).abs().maxCoeff() > 1e-12 ||
      (v.array() - 1.0).abs().maxCoeff() > 1e-12) {
    std::cerr << "Position-only recovery jumped to raw\n";
    return 1;
  }

  using humanoidController_wheel_wbc::ArmTrajectoryInterpolator;
  ArmTrajectoryInterpolator interpolator;
  ArmTrajectoryInterpolator::Config config;
  config.controlCycleSec = 0.001;
  config.kalmanVLimit = 1.0;
  interpolator.configure(config);
  ArmTrajectoryInterpolator::ModeFlags mode;
  mode.useArmTrajectoryControl = true;
  ros::Time now(100, 0);
  q.setZero();
  interpolator.compute(now, mode, q);
  interpolator.ingestRawTarget(now, target, Eigen::VectorXd::Zero(14));
  now += ros::Duration(0.004);
  auto output = interpolator.compute(now, mode, q);
  if (!output.valid || output.smoothQ.maxCoeff() > 0.004 + 1e-12 ||
      output.smoothQ.minCoeff() < 0.0039) {
    std::cerr << "Interpolator did not use actual cycle time: valid=" << output.valid
              << " q=" << output.smoothQ.transpose() << "\n";
    return 1;
  }
  const auto heldQ = output.smoothQ.eval();
  output = interpolator.compute(now, mode, q);
  if ((output.smoothQ - heldQ).norm() > 1e-12) {
    std::cerr << "Duplicate timestamp advanced prediction\n";
    return 1;
  }
  // Actual controller path: identical zero-speed inputs are deduplicated.
  for (int n = 0; n < 200; ++n) {
    const auto previous = output.smoothQ.eval();
    now += ros::Duration(0.004);
    interpolator.ingestRawTarget(now, target, Eigen::VectorXd::Zero(14));
    output = interpolator.compute(now, mode, q);
    if (!output.valid || output.timeout || !output.smoothQ.allFinite() ||
        output.smoothQ.maxCoeff() > 0.5 + 1e-12 ||
        (output.smoothQ - previous).minCoeff() < -1e-12 ||
        (output.smoothQ - previous).maxCoeff() > 0.004 + 1e-12) {
      std::cerr << "Repeated static target drifted, overshot or timed out\n";
      return 1;
    }
  }
  if ((output.smoothQ - target).norm() > 1e-10 || output.smoothV.norm() > 1e-10) {
    std::cerr << "Repeated static target did not settle at zero velocity\n";
    return 1;
  }
  // Start a reverse recovery, then stop input before reaching the target.
  target.setConstant(-0.25);
  now += ros::Duration(0.004);
  interpolator.ingestRawTarget(now, target, Eigen::VectorXd::Zero(14));
  output = interpolator.compute(now, mode, q);
  for (int n = 0; n < 20; ++n) {
    const auto previous = output.smoothQ.eval();
    now += ros::Duration(0.004);
    output = interpolator.compute(now, mode, q);
    if ((output.smoothQ - previous).maxCoeff() > 1e-12 ||
        (output.smoothQ - previous).minCoeff() < -0.004 - 1e-12 ||
        output.smoothQ.minCoeff() < -0.25 - 1e-12) {
      std::cerr << "Recovery without new measurements lost its target\n";
      return 1;
    }
  }
  const auto timeoutQ = output.smoothQ.eval();
  now += ros::Duration(0.25);
  output = interpolator.compute(now, mode, q);
  if (!output.timeout || (output.smoothQ - timeoutQ).norm() > 1e-12 ||
      output.smoothV.norm() > 1e-12) {
    std::cerr << "Timeout did not hold position at zero velocity\n";
    return 1;
  }
  // Resuming the same target after timeout must restart bounded recovery.
  now += ros::Duration(0.004);
  interpolator.ingestRawTarget(now, target, Eigen::VectorXd::Zero(14));
  output = interpolator.compute(now, mode, q);
  if (output.timeout || (output.smoothQ - timeoutQ).minCoeff() < -0.004 - 1e-12 ||
      (output.smoothQ - timeoutQ).maxCoeff() >= 0.0) {
    std::cerr << "Recovery did not resume safely after timeout\n";
    return 1;
  }
  // Actual disconnection path: control keeps ingesting the last cached sample.
  // Its receive timestamp must remain frozen even though compute time advances.
  ArmTrajectoryInterpolator disconnected;
  disconnected.configure(config);
  now = ros::Time(200, 0);
  disconnected.compute(now, mode, q);
  target.setConstant(0.5);
  const Eigen::VectorXd movingV = Eigen::VectorXd::Constant(14, 0.2);
  ros::Time receivedAt;
  for (int n = 0; n < 10; ++n) {
    now += ros::Duration(0.004);
    receivedAt = now;
    disconnected.ingestRawTarget(receivedAt, target, movingV);
    output = disconnected.compute(now, mode, q);
    if (output.timeout) {
      std::cerr << "Fresh identical messages did not keep input alive\n";
      return 1;
    }
  }
  bool sawTimeout = false;
  Eigen::VectorXd stoppedQ;
  for (int n = 0; n < 200; ++n) {
    now += ros::Duration(0.004);
    disconnected.ingestRawTarget(receivedAt, target, movingV);
    output = disconnected.compute(now, mode, q);
    if ((now - receivedAt).toSec() > config.timeoutSec && !output.timeout) {
      std::cerr << "Cached reads masked upstream disconnection\n";
      return 1;
    }
    if (output.timeout) {
      if (!sawTimeout) stoppedQ = output.smoothQ;
      sawTimeout = true;
      if (output.smoothV.norm() > 1e-12 || (output.smoothQ - stoppedQ).norm() > 1e-12) {
        std::cerr << "Disconnected nonzero-speed command did not stop\n";
        return 1;
      }
    }
  }
  if (!sawTimeout) {
    std::cerr << "Disconnection never triggered timeout\n";
    return 1;
  }
  now += ros::Duration(0.004);
  disconnected.ingestRawTarget(now, target, movingV);
  output = disconnected.compute(now, mode, q);
  if (output.timeout || (output.smoothQ - stoppedQ).cwiseAbs().maxCoeff() > 0.004 + 1e-12) {
    std::cerr << "Fresh input did not resume safely after cached-read timeout\n";
    return 1;
  }
  std::cout << "Arm command recovery checks passed\n";
  return 0;
}
