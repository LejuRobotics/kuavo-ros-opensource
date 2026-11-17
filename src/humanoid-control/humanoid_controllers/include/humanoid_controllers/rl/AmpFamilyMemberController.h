#pragma once
#include "humanoid_controllers/rl/AmpWalkController.h"
#include "humanoid_controllers/rl/AmpFamilyCommandFilter.h"
#include "humanoid_controllers/rl/RunToWalkElbowInterpolation.h"

namespace humanoid_controller {
// AMP family adaptation shared by ordinary, WAQ and impedance members.
// Model observation, action scaling and PD mapping remain in their existing controllers.
class AmpFamilyMemberController : public AmpWalkController {
 public:
  AmpFamilyMemberController(const std::string& name, const std::string& config_file,
    ros::NodeHandle& nh, ocs2::humanoid::TopicLogger* logger = nullptr);
  bool initialize() override;
  void reset() override;
  Eigen::VectorXd getCurrentJointReference() const override;
  std::array<double,3> getFamilyCommandLimits() const {
    return {{commandVelocityLimits()(0),commandVelocityLimits()(1),commandVelocityLimits()(3)}};
  }

  void setThermalVelocityLimit(double limit) {
    command_filter_->setThermalVelocityLimit(limit);
  }
  void setRunToWalkSpeedLimit(double limit) {
    command_filter_->setRunToWalkSpeedLimit(limit);
  }
  void configureRunElbowInterpolation(const RunToWalkElbowInterpolation::Config& config,
                                     const std::array<int,2>& indices) {
    std::lock_guard<std::mutex> lock(run_elbow_mutex_);
    run_elbow_interpolation_.configure(config); run_elbow_indices_ = indices;
    run_elbow_requested_ = false;
  }
  void requestRunElbowInterpolation(bool requested, bool executing) {
    std::lock_guard<std::mutex> lock(run_elbow_mutex_);
    run_elbow_requested_ = requested && executing;
    if (!executing) run_elbow_interpolation_.reset();
  }
  RunToWalkElbowInterpolation runElbowInterpolationSnapshot() const {
    std::lock_guard<std::mutex> lock(run_elbow_mutex_); return run_elbow_interpolation_;
  }
  void resumeExternalVelocityAfterThermalExit() {
    command_filter_->resumeExternalVelocityAfterThermalExit();
  }
  void setForwardVelocitySmoothing(double decel_tau, double accel_tau = 0) {
    command_filter_->setForwardVelocitySmoothing(decel_tau, accel_tau);
  }
  void setRobotActionStateEnabled(bool enabled) {
    if (gaitReceiver()) gaitReceiver()->setRobotActionStateEnabled(enabled);
  }

 protected:
  AmpFamilyMemberController(const std::string& name, RLControllerType type,
    const std::string& config_file, ros::NodeHandle& nh,
    ocs2::humanoid::TopicLogger* logger = nullptr);
  void processJointReference(Eigen::VectorXd& joint_reference) override;

 private:
  std::shared_ptr<AmpFamilyCommandFilter> command_filter_ = std::make_shared<AmpFamilyCommandFilter>();
  mutable std::mutex run_elbow_mutex_;
  RunToWalkElbowInterpolation run_elbow_interpolation_;
  std::array<int,2> run_elbow_indices_{{16,23}};
  bool run_elbow_requested_ = false;
};
}  // namespace humanoid_controller
