#include <cmath>
#include <iostream>
#include "humanoid_controllers/ArmTrajectoryInterpolator.h"

int main() {
  ros::Time::init();
  using humanoidController_wheel_wbc::ArmTrajectoryInterpolator;
  ArmTrajectoryInterpolator tracker;
  ArmTrajectoryInterpolator::Config config;
  config.boundedCommandTracking = true;
  config.kalmanVLimit = 30.0;
  config.controlCycleSec = 0.001;
  tracker.configure(config);
  ArmTrajectoryInterpolator::ModeFlags mode;
  mode.useArmTrajectoryControl = true;
  ros::Time now(100, 0), receivedAt;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(14), v = q, rawQ = q, rawV = q;
  tracker.compute(now, mode, q);
  for (int n = 1; n <= 1200; ++n) {
    const double dt = n % 3 == 0 ? 0.004 : 0.003;
    now += ros::Duration(dt);
    const double t = (now - ros::Time(100, 0)).toSec();
    // Linear motion followed by deceleration while 50 ms of input is missing.
    const double ramp = std::min(t, 0.5);
    rawQ.setConstant(2.0 * ramp);
    rawV.setConstant(t < 0.5 ? 2.0 : 0.0);
    if (n % 3 == 0 && !(t > 0.48 && t < 0.53)) {
      receivedAt = now;
      tracker.ingestRawTarget(receivedAt, rawQ, rawV);
    }
    const auto output = tracker.compute(now, mode, q);
    if (!output.valid || !output.smoothQ.allFinite() || !output.smoothV.allFinite() ||
        (output.smoothV - v).cwiseAbs().maxCoeff() > config.commandAccelerationLimit * dt + 1e-9 ||
        output.smoothV.cwiseAbs().maxCoeff() > config.kalmanVLimit + 1e-9 ||
        // Velocity can peak inside a cycle, so use the acceleration-integral bound.
        (output.smoothQ - q - v * dt).cwiseAbs().maxCoeff() >
            0.5 * config.commandAccelerationLimit * dt * dt + 1e-9) {
      std::cerr << "Gap tracking violated command kinematics: n=" << n
                << " dq=" << (output.smoothQ - q).cwiseAbs().maxCoeff()
                << " dv=" << (output.smoothV - v).cwiseAbs().maxCoeff()
                << " prevV=" << v[0] << " nextV=" << output.smoothV[0] << "\n";
      return 1;
    }
    q = output.smoothQ;
    v = output.smoothV;
  }
  if ((q.array() - 1.0).abs().maxCoeff() > 1e-6 || v.cwiseAbs().maxCoeff() > 1e-6) {
    std::cerr << "Static target did not converge\n";
    return 1;
  }
  // Control loop still reads the same cache after upstream stops.
  now += ros::Duration(0.25);
  tracker.ingestRawTarget(receivedAt, rawQ, rawV);
  const auto stopped = tracker.compute(now, mode, q);
  if (!stopped.timeout || (stopped.smoothQ - q).norm() > 1e-12 || stopped.smoothV.norm() > 1e-12) {
    std::cerr << "Cached input masked timeout\n";
    return 1;
  }
  std::cout << "Gap tracking and timeout checks passed\n";
  return 0;
}
