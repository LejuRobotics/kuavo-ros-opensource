#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <cstdint>

#include <Eigen/Core>
#include <ros/ros.h>

namespace ocs2 {
namespace mobile_manipulator {

// Command smoother/extrapolator based on a linear Kalman model.
// Observation q is command/reference position rather than sensor position.
class ConstantVelocityCommandKalmanFilter {
public:
  explicit ConstantVelocityCommandKalmanFilter(int dofNum, double cycleTime = 0.002);
  ~ConstantVelocityCommandKalmanFilter() = default;

  void setKinematicLimits(const Eigen::VectorXd& maxVel);
  void setKalmanParameters(double processModelNoise,
                           double measurementQNoise,
                           double measurementDqNoise,
                           double initialPosVar,
                           double initialVelVar,
                           double fastUpdateRScale);
  void reset(const Eigen::VectorXd& initialPosition);

  bool updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity);
  bool predict(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity);
  bool update();
  bool step(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity);
  bool isOperational() const;
  std::string statusMessage() const;
  uint64_t predictCount() const;
  uint64_t updateCount() const;

private:
  using Matrix2 = Eigen::Matrix2d;
  using Vector2 = Eigen::Vector2d;

  bool checkVectorSize(const Eigen::VectorXd& vec, const char* name) const;
  void updateSystemMatrices();
  void sanitizeState(Vector2& state) const;
  bool validateAndReport(const char* source) const;

  size_t dofNum_{1};
  double cycleTime_{0.002};
  mutable std::mutex mutex_;
  bool initialized_{false};
  bool dofValid_{false};
  bool hasPendingMeasurement_{false};
  mutable bool dofErrorLogged_{false};
  std::string statusMessage_{"Not initialized"};
  uint64_t predictCount_{0};
  uint64_t updateCount_{0};

  Matrix2 transitionMatrix_{Matrix2::Identity()};
  Matrix2 processNoise_{Matrix2::Identity()};
  Matrix2 observationMatrix_{Matrix2::Identity()};
  Matrix2 measurementNoise_{Matrix2::Identity()};
  Matrix2 measurementNoiseFast_{Matrix2::Identity()};
  Matrix2 identityMatrix_{Matrix2::Identity()};

  Eigen::VectorXd maxVel_;
  Eigen::VectorXd targetPosition_;
  Eigen::VectorXd targetVelocity_;
  Eigen::MatrixXd state_;
  std::vector<Matrix2> covariance_;
  Eigen::VectorXd stepPositionBuffer_;
  Eigen::VectorXd stepVelocityBuffer_;

  double processModelNoise_{10.0};
  double measurementQNoise_{1e-4};
  double measurementDqNoise_{1e-3};
  double initialPosVar_{1e-3};
  double initialVelVar_{1e-2};
  double fastUpdateRScale_{0.1};
};

}  // namespace mobile_manipulator
}  // namespace ocs2
