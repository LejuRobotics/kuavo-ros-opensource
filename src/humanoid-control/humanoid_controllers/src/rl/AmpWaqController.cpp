#include <pinocchio/fwd.hpp>
#include "humanoid_controllers/rl/AmpWaqController.h"
#include <boost/property_tree/info_parser.hpp>
#include <cmath>

namespace humanoid_controller {
AmpWaqController::AmpWaqController(const std::string& name, const std::string& config_file,
    ros::NodeHandle& nh, ocs2::humanoid::TopicLogger* logger)
    : AmpFamilyMemberController(name, config_file, nh, logger) {
  type_ = RLControllerType::AMP_WAQ_CONTROLLER;
}

AmpWaqController::~AmpWaqController() {
  // Stop before the history, model and lifecycle mutex are destroyed.
  stop();
}

bool AmpWaqController::loadConfig(const std::string& config_file) {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  boost::property_tree::ptree config;
  boost::property_tree::read_info(config_file, config);
  int version = 0;
  nh_.getParam("/robot_version", version);
  activity_command_ = config.get<double>("waqActivityCommand", 1.0);
  publish_network_input_ = config.get<bool>("waqLogging.publishNetworkInput", false);
  if ((version != 54 && version != 55) || jointNum_ != 12 || waistNum_ != 1 || jointArmNum_ != 14 ||
      config.get<int>("frameStack") != 10 || config.get<int>("numSingleObs") != 91 ||
      std::abs(config.get<double>("inferenceFrequency") - 100.0) > 1e-6 ||
      std::abs(config.get<double>("actionScale") - 0.25) > 1e-6 ||
      !std::isfinite(activity_command_) || activity_command_ < 0.0 || activity_command_ > 1.0 ||
      !config.get<bool>("withArm") ||
      config.get<bool>("use_external_waist_controller", false)) {
    ROS_ERROR("[%s] AMP WAQ requires S54/S55, 27 joints, 10x91 input, 100 Hz, scale .25, withArm=true and external waist control disabled", name_.c_str());
    return false;
  }
  // Reuse AMP's optional arm-mode controller after policy actuation. The WAQ
  // observation still uses measured joints and the previous raw policy action.
  if (!AmpFamilyMemberController::loadConfig(config_file)) return false;
  const auto& model = policyModel();
  if (model.inputs().size() != 1 || model.outputs().size() != 1 ||
      model.input().get_shape() != ov::Shape({1, AmpWaqObservation::kInput}) ||
      model.output().get_shape() != ov::Shape({1, AmpWaqObservation::kJoints}) ||
      model.input().get_element_type() != ov::element::f32 ||
      model.output().get_element_type() != ov::element::f32) {
    ROS_ERROR("[%s] AMP WAQ model must have float32 obs[1,910] -> actions[1,27]", name_.c_str());
    return false;
  }
  history_.reset();
  successful_inference_cycles_ = 0;
  policy_fault_ = false;
  ROS_INFO("[%s] AMP WAQ loaded: 9 previous frames + current, 100 Hz, 27 actions, activity=%.2f",
           name_.c_str(), activity_command_);
  return true;
}

void AmpWaqController::reset() {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  AmpFamilyMemberController::reset();
  history_.reset();
  successful_inference_cycles_ = 0;
  {
    std::lock_guard<std::mutex> command_lock(command_snapshot_mtx_);
    command_snapshot_ = CommandDataRL();
  }
  setCurrentAction(Eigen::VectorXd::Zero(AmpWaqObservation::kJoints));
  policy_fault_ = false;
}

void AmpWaqController::resume() {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  AmpFamilyMemberController::resume();
}

void AmpWaqController::resumeWarm() {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  // Incoming activations reset in prepareForActivation(). Keep the outgoing
  // policy's history/action intact when it supplies a live blend source.
  AmpFamilyMemberController::resumeWarm();
}

bool AmpWaqController::isReadyForInterpolation() const {
  // The base publishes the action before releasing this cycle lock, so a
  // completed count also guarantees the corresponding action is available.
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  return !policy_fault_ && successful_inference_cycles_ >= AmpWaqObservation::kHistory + 1;
}

bool AmpWaqController::requestToExit() const {
  // Keep policy faults separate from physical falls: the fault may be raised
  // asynchronously after the main loop's emergency-stop check.
  return AmpFamilyMemberController::requestToExit();
}

bool AmpWaqController::updateImpl(const ros::Time& time, const SensorData& sensors,
    const Eigen::VectorXd& state, kuavo_msgs::jointCmd& joint_cmd) {
  const bool success = AmpFamilyMemberController::updateImpl(time, sensors, state, joint_cmd);
  // getPolicyCommand can call back into the manager. Resolve it on the control
  // thread, outside the inference lock, to preserve manager -> controller order.
  const auto command = policyCommand();
  {
    std::lock_guard<std::mutex> command_lock(command_snapshot_mtx_);
    command_snapshot_ = command;
  }
  return success;
}

void AmpWaqController::updateObservation(const Eigen::VectorXd&, const SensorData& sensors) {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  try {
    CommandDataRL command;
    {
      std::lock_guard<std::mutex> command_lock(command_snapshot_mtx_);
      command = command_snapshot_;
    }
    command.scale();
    const bool standing = command.cmdStance_ >= 0.5;
    Eigen::Vector3d velocity(command.cmdVelLineX_, command.cmdVelLineY_, command.cmdVelAngularZ_);
    if (standing) velocity.setZero();
    if (sensors.jointPos_.size() < AmpWaqObservation::kJoints ||
        sensors.jointVel_.size() < AmpWaqObservation::kJoints)
      throw std::invalid_argument("AMP WAQ sensor sample has fewer than 27 joints");
    const Eigen::VectorXd current = AmpWaqObservation::current(sensors.angularVel_, sensors.quat_,
        velocity, standing ? 0.0 : activity_command_, sensors.jointPos_.head(27),
        defalutJointPosRL_, sensors.jointVel_.head(27), getCurrentAction());
    networkInputDataRL_ = history_.push(current);
    if (ros_logger_) {
      // Same field names as AMP and singleInputData in the info file. Log the
      // actual model input: relative joint positions and previous raw action.
      publishPolicyObservation("InputData/bodyAngVel", current.segment(0, 3));
      publishPolicyObservation("InputData/gravity_body", current.segment(3, 3));
      publishPolicyObservation("InputData/command_scalar_state", current.segment(6, 4));
      publishPolicyObservation("InputData/jointPos", current.segment(10, 27));
      publishPolicyObservation("InputData/jointVel", current.segment(37, 27));
      publishPolicyObservation("InputData/action", current.segment(64, 27));
      publishPolicyObservation("singleInputData", current);
      // Full history is useful for diagnosis but needlessly increases bags in
      // normal runs. Actions, torque and timing are already logged by the base.
      if (publish_network_input_)
        ros_logger_->publishVector("/rl_controller/" + name_ + "/input", networkInputDataRL_);
    }
  } catch (const std::exception& error) {
    policy_fault_ = true;
    ROS_ERROR_THROTTLE(1.0, "[%s] Invalid WAQ observation: %s", name_.c_str(), error.what());
  }
}

bool AmpWaqController::inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action) {
  std::lock_guard<std::recursive_mutex> lock(inference_cycle_mtx_);
  if (policy_fault_) return false;
  try {
    if (observation.size() != AmpWaqObservation::kInput || !observation.allFinite())
      throw std::invalid_argument("AMP WAQ input must contain 910 finite values");
    Eigen::VectorXf input = observation.cast<float>();
    auto request = policyModel().create_infer_request();
    request.set_input_tensor(ov::Tensor(ov::element::f32, {1, AmpWaqObservation::kInput}, input.data()));
    request.infer();
    const auto output = request.get_output_tensor();
    if (output.get_shape() != ov::Shape({1, AmpWaqObservation::kJoints}) ||
        output.get_element_type() != ov::element::f32)
      throw std::runtime_error("AMP WAQ output must be float32 actions[1,27]");
    action = Eigen::Map<const Eigen::VectorXf>(output.data<const float>(), AmpWaqObservation::kJoints).cast<double>();
    if (!action.allFinite()) throw std::runtime_error("Nonfinite WAQ action");
    if (successful_inference_cycles_ < AmpWaqObservation::kHistory + 1)
      ++successful_inference_cycles_;
    // No action clipping or AMP offsets: training used an unclipped Gaussian action.
    // Existing actuation code applies the configured joint torque limits.
    return true;
  } catch (const std::exception& error) {
    policy_fault_ = true;
    ROS_ERROR_THROTTLE(1.0, "[%s] WAQ inference failed: %s", name_.c_str(), error.what());
    return false;
  }
}
}  // namespace humanoid_controller
