#include "humanoid_controllers/rl/AmpFamilyMemberController.h"

namespace humanoid_controller {
AmpFamilyMemberController::AmpFamilyMemberController(const std::string& name,
    const std::string& config_file, ros::NodeHandle& nh,
    ocs2::humanoid::TopicLogger* logger)
    : AmpFamilyMemberController(name, RLControllerType::AMP_CONTROLLER, config_file, nh, logger) {}

AmpFamilyMemberController::AmpFamilyMemberController(const std::string& name,
    RLControllerType type, const std::string& config_file, ros::NodeHandle& nh,
    ocs2::humanoid::TopicLogger* logger)
    : AmpWalkController(name, type, config_file, nh, logger) {}

bool AmpFamilyMemberController::initialize() {
  if (!AmpWalkController::initialize()) return false;
  if (gaitReceiver()) gaitReceiver()->setCommandFilter(command_filter_);
  return true;
}

void AmpFamilyMemberController::reset() {
  { std::lock_guard<std::mutex> lock(run_elbow_mutex_);
    run_elbow_interpolation_.reset(); run_elbow_requested_ = false; }
  AmpWalkController::reset();
}

Eigen::VectorXd AmpFamilyMemberController::getCurrentJointReference() const {
  Eigen::VectorXd q_ref = AmpWalkController::getCurrentJointReference();
  { std::lock_guard<std::mutex> lock(run_elbow_mutex_);
    if (run_elbow_interpolation_.active())
      for (size_t j=0;j<2;++j)
        if (run_elbow_indices_[j] >= 0 && run_elbow_indices_[j] < q_ref.size())
          q_ref[run_elbow_indices_[j]] = run_elbow_interpolation_.reference()[j];
  }
  return q_ref;
}

void AmpFamilyMemberController::processJointReference(Eigen::VectorXd& joint_reference) {
  {
    std::lock_guard<std::mutex> lock(run_elbow_mutex_);
    if (run_elbow_interpolation_.enabled()) {
      const bool valid_indices = run_elbow_indices_[0] >= 0 && run_elbow_indices_[1] >= 0 &&
          run_elbow_indices_[0] < joint_reference.size() && run_elbow_indices_[1] < joint_reference.size();
      // Only replace RL arm references. Explicit external arm control retains priority.
      if (valid_indices && (!arm_controller_ || arm_controller_->getMode() == 1)) {
        const std::array<double,2> rl{{joint_reference[run_elbow_indices_[0]], joint_reference[run_elbow_indices_[1]]}};
        const auto reference = run_elbow_interpolation_.update(ros::Time::now().toSec(), run_elbow_requested_, rl);
        for (size_t j=0;j<2;++j) joint_reference[run_elbow_indices_[j]] = reference[j];
      } else run_elbow_interpolation_.reset();
    }
  }
}
}  // namespace humanoid_controller
