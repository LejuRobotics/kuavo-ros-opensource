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

  // Short ~50 ms gap must coast at last velocity, not brake toward 0 after 10 ms.
  {
    ArmTrajectoryInterpolator coastTracker;
    ArmTrajectoryInterpolator::Config coastConfig;
    coastConfig.boundedCommandTracking = true;
    coastConfig.kalmanVLimit = 30.0;
    coastConfig.controlCycleSec = 0.002;
    coastConfig.timeoutSec = 0.2;
    coastConfig.referenceUpdatePeriodSec = 0.010;
    coastConfig.commandAccelerationLimit = 60.0;
    coastTracker.configure(coastConfig);
    const ros::Time t0(300, 0);
    now = t0;
    q.setZero();
    v.setZero();
    rawQ.setZero();
    rawV.setConstant(2.0);
    coastTracker.compute(now, mode, q);
    for (int n = 0; n < 200; ++n) {
      const double dt = 0.002;
      now += ros::Duration(dt);
      if (n % 5 == 0) {
        rawQ.setConstant(2.0 * (now - t0).toSec());
        coastTracker.ingestRawTarget(now, rawQ, rawV);
      }
      const auto output = coastTracker.compute(now, mode, q);
      if (!output.valid || output.timeout ||
          (output.smoothV - v).cwiseAbs().maxCoeff() > coastConfig.commandAccelerationLimit * dt + 1e-9 ||
          output.smoothV.cwiseAbs().maxCoeff() > coastConfig.kalmanVLimit + 1e-9) {
        std::cerr << "Kinematics violated while settling before coast gap\n";
        return 1;
      }
      q = output.smoothQ;
      v = output.smoothV;
    }
    const double preGapV = v[0];
    const Eigen::VectorXd preGapQ = q;
    if (std::abs(preGapV - 2.0) > 0.05) {
      std::cerr << "Failed to settle before gap: v=" << preGapV << "\n";
      return 1;
    }

    double minMidGapAbsV = 1e9;
    ros::Time gapStart = now;
    for (int n = 0; n < 25; ++n) {
      const double dt = 0.002;
      now += ros::Duration(dt);
      const auto previousV = v;
      const auto output = coastTracker.compute(now, mode, q);
      const double age = (now - gapStart).toSec();
      if (!output.valid || output.timeout ||
          (output.smoothV - previousV).cwiseAbs().maxCoeff() > coastConfig.commandAccelerationLimit * dt + 1e-9 ||
          output.smoothV.cwiseAbs().maxCoeff() > coastConfig.kalmanVLimit + 1e-9) {
        std::cerr << "Kinematics violated during 50 ms coast gap, age=" << age
                  << " v=" << output.smoothV[0] << "\n";
        return 1;
      }
      // After the old 20 ms brake-to-zero window, velocity must stay near pre-gap v.
      if (age >= 0.020) {
        minMidGapAbsV = std::min(minMidGapAbsV, std::abs(output.smoothV[0]));
      }
      q = output.smoothQ;
      v = output.smoothV;
    }
    if (minMidGapAbsV < 0.8 * std::abs(preGapV)) {
      std::cerr << "Short gap braked instead of coasting: mid-gap min |v|=" << minMidGapAbsV
                << " pre-gap v=" << preGapV << "\n";
      return 1;
    }
    const double coastDq = q[0] - preGapQ[0];
    const double expectedDq = preGapV * (now - gapStart).toSec();
    if (std::abs(coastDq - expectedDq) > 0.02) {
      std::cerr << "Coast distance mismatch: dq=" << coastDq << " expected=" << expectedDq << "\n";
      return 1;
    }

    // New frame after the hole reconnects with the accel clamp; no +/-30 rad/s spikes.
    // Typical SHM hole: ~11 deg position jump, often with a much smaller (here zero) velocity.
    rawQ = q + Eigen::VectorXd::Constant(14, 0.2);
    rawV.setZero();
    for (int n = 0; n < 150; ++n) {
      const double dt = 0.002;
      now += ros::Duration(dt);
      if (n == 0 || n % 5 == 0) {
        coastTracker.ingestRawTarget(now, rawQ, rawV);
      }
      const auto previousV = v;
      const auto output = coastTracker.compute(now, mode, q);
      if (!output.valid || output.timeout ||
          (output.smoothV - previousV).cwiseAbs().maxCoeff() > coastConfig.commandAccelerationLimit * dt + 1e-9 ||
          output.smoothV.cwiseAbs().maxCoeff() > 8.0) {
        std::cerr << "Reconnect exceeded accel clamp or produced a velocity spike: v="
                  << output.smoothV[0] << " dv=" << (output.smoothV - previousV)[0] << "\n";
        return 1;
      }
      q = output.smoothQ;
      v = output.smoothV;
    }

    // A true 200 ms timeout still holds pose and zeros velocity immediately.
    const auto holdQ = q.eval();
    now += ros::Duration(0.25);
    const auto timedOut = coastTracker.compute(now, mode, q);
    if (!timedOut.timeout || (timedOut.smoothQ - holdQ).norm() > 1e-12 || timedOut.smoothV.norm() > 1e-12) {
      std::cerr << "200 ms timeout did not hold pose at zero velocity after coasting\n";
      return 1;
    }
  }

  std::cout << "Gap tracking and timeout checks passed\n";
  return 0;
}
