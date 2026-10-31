#pragma once
#include "humanoid_controllers/rl/AmpFamilyMemberController.h"
#include "humanoid_controllers/rl/AmpWaqObservation.h"

namespace humanoid_controller {
// S54-trained WAQ policy deployed on the compatible S55 27-DOF ordering.
class AmpWaqController : public AmpFamilyMemberController {
 public:
  AmpWaqController(const std::string& name, const std::string& config_file,
                   ros::NodeHandle& nh, ocs2::humanoid::TopicLogger* logger = nullptr);
  ~AmpWaqController() override;
  bool loadConfig(const std::string& config_file) override;
  void reset() override;
  void resume() override;
  void resumeWarm() override;
  void prepareForActivation() override { reset(); }
  bool isReadyForInterpolation() const override;
  bool supportsWalkingSwitch() const override { return true; }
  bool supportsWalkingPhaseSyncSwitch() const override { return false; }
  bool hasValidWalkingPhase() const override { return false; }
  bool requestToExit() const override;
  bool requiresEmergencyStop() const override { return policy_fault_.load(); }
 protected:
  std::recursive_mutex* inferenceCycleMutex() override { return &inference_cycle_mtx_; }
  bool updateImpl(const ros::Time& time, const SensorData& sensors,
                  const Eigen::VectorXd& state, kuavo_msgs::jointCmd& joint_cmd) override;
  void updateObservation(const Eigen::VectorXd& state, const SensorData& sensors) override;
  bool inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action) override;
 private:
  mutable std::recursive_mutex inference_cycle_mtx_;
  std::mutex command_snapshot_mtx_;
  CommandDataRL command_snapshot_;
  AmpWaqObservation history_;
  int successful_inference_cycles_{0};
  std::atomic<bool> policy_fault_{false};
  double activity_command_{1.0};
  bool publish_network_input_{false};
};
}  // namespace humanoid_controller
