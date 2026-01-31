#include "motion_capture_ik/Quest3HandEE_ROS.h"

namespace HighlyDynamic {

Quest3HandEE_ROS::Quest3HandEE_ROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint)
    : ArmControlBaseROS(nodeHandle, publishRate, debugPrint) {}

Quest3HandEE_ROS::~Quest3HandEE_ROS() {
  shouldStop_ = true;
  if (processThread_.joinable()) {
    processThread_.join();
  }
}

void Quest3HandEE_ROS::initialize(const nlohmann::json& configJson) {
  initializeBase(configJson);

  // launch 中通过remap接入
  kuavoHandPosePublisher_ = nodeHandle_.advertise<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd_cpp", 1);
  changeArmModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
  fkServiceClient_ = nodeHandle_.serviceClient<kuavo_msgs::fkSrv>("/ik/fk_srv");
}

void Quest3HandEE_ROS::run() {
  processThread_ = std::thread(&Quest3HandEE_ROS::processHandSE3ThreadFunction, this);
  ros::spin();
}

void Quest3HandEE_ROS::processHandSE3ThreadFunction() {
  ros::Rate rate(publishRate_);

  while (!shouldStop() && ros::ok()) {
    if (shouldEnterIncrementalMode()) {
      if (shouldUpdateHandAnchorPose()) {
        updateHandAnchorPose(getVrLeftHandPose(), getVrRightHandPose());
      }
      computeHandPoseDifference(getVrLeftHandPose(), getVrRightHandPose());
      computeFinalHandePose(leftAnchorPose_, leftDiffPose_);
      computeFinalHandePose(rightAnchorPose_, rightDiffPose_);

      publishTwoHandPose();
    }

    // 夹爪/灵巧手的开合数据正常发布
    publishEndEffectorControlData();
  }

  rate.sleep();
}

void Quest3HandEE_ROS::publishTwoHandPose() {
  // TODO: 发布转换后的/mm/two_arm_hand_pose_cmd_cpp
}

bool Quest3HandEE_ROS::shouldEnterIncrementalMode() const {
  return true;  // TODO:
}

bool Quest3HandEE_ROS::shouldUpdateHandAnchorPose() const {
  return true;  // TODO:
}

void Quest3HandEE_ROS::updateHandAnchorPose(const ArmPose& leftAnchorPose, const ArmPose& rightAnchorPose) {
  // TODO:
}

void Quest3HandEE_ROS::computeHandPoseDifference(const ArmPose& leftHandPose, const ArmPose& rightHandPose) {
  // TODO:
}

ArmPose Quest3HandEE_ROS::computeFinalHandePose(const ArmPose& leftAnchorPose, const ArmPose& leftDiffPose) {
  return interpolateArmPose(leftAnchorPose, leftDiffPose);
}

ArmPose Quest3HandEE_ROS::interpolateArmPose(const ArmPose& anchorPose, const ArmPose& diffPose) {
  return ArmPose();  // TODO:
}

bool Quest3HandEE_ROS::changeMobileCtrlModeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  return false;  // TODO:
}

const ArmPose& Quest3HandEE_ROS::getVrLeftHandPose() const { return quest3ArmInfoTransformerPtr_->getLeftHandPose(); }

const ArmPose& Quest3HandEE_ROS::getVrRightHandPose() const { return quest3ArmInfoTransformerPtr_->getRightHandPose(); }

}  // namespace HighlyDynamic
