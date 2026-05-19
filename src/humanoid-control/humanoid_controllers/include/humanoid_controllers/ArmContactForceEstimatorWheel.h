#pragma once

#include <deque>
#include <memory>
#include <ros/duration.h>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_interface/common/TopicLogger.h"
#include "humanoid_wheel_interface/HumanoidWheelInterface.h"

namespace humanoidController_wheel_wbc {

using namespace ocs2;

class ArmContactForceEstimatorWheel {
 public:
  ArmContactForceEstimatorWheel(std::shared_ptr<PinocchioInterface> pinocchioInterface,
                                const mobile_manipulator::ManipulatorModelInfo& manipulatorModelInfo,
                                size_t lowJointNum, humanoid::TopicLogger* rosLogger);

  void setCmdTorque(const vector_t& cmdTorque);
  void update(const vector_t& state, const vector_t& input, const ros::Duration& period);
  void estArmContactForce_wheel(const vector_t& state, const vector_t& input, const ros::Duration& period);
  const vector_t& getEstimatedArmContactForce() const;

 private:
  static vector6_t dlsSolve6(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double lambda);

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo_;
  size_t lowJointNum_{0};
  humanoid::TopicLogger* ros_logger_{nullptr};

  // 力估计状态
  vector_t estArmContactforce_;
  vector_t estArmContactforceLast_;
  vector_t pSCgArmZinvlast_;
  vector_t cmdTorque_;
  std::deque<Eigen::VectorXd> pSCgArmSmoothWindow_;
  std::deque<Eigen::VectorXd> armForceMedianWindow_;

  // 力估计参数
  scalar_t cutoffFrequency_ = 150.0;
  double armJacMinSingularThresh_ = 0.02;
  double armJacCondThresh_ = 800.0;
  double armDampingLambda0_ = 0.01;
  double armDampingGain_ = 0.4;
  double armForceMax_ = 1000.0;
  double armMomentMax_ = 60.0;
  const int pSCgSmoothWindowSize_ = 3;
  size_t armDofPerSide_ = 7;
};

}  // namespace humanoidController_wheel_wbc
