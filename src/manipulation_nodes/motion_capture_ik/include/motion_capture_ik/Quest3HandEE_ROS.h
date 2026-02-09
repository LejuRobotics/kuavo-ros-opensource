#pragma once

#include <thread>
#include "leju_utils/define.hpp"
#include "motion_capture_ik/json.hpp"
// #include "leju_utils/math.hpp"  // 插值的公式实现代码

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"

#include <kuavo_msgs/twoArmHandPoseCmd.h>
#include <kuavo_msgs/fkSrv.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
namespace HighlyDynamic {

class Quest3HandEE_ROS final : public ArmControlBaseROS {
 public:
  explicit Quest3HandEE_ROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint = false);

  ~Quest3HandEE_ROS();

  void initialize(const nlohmann::json& configJson) override;
  void run() override;

 private:
  ros::Publisher kuavoHandPosePublisher_;
  ros::ServiceClient fkServiceClient_;
  ros::ServiceClient changeArmModeClient_;


  std::thread processThread_;

  ArmPose leftAnchorPose_;
  ArmPose rightAnchorPose_;

  ArmPose leftDiffPose_;
  ArmPose rightDiffPose_;

  ArmPose leftFinalPose_;
  ArmPose rightFinalPose_;

  void processHandSE3ThreadFunction();

  void publishTwoHandPose();
  bool changeMobileCtrlModeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool shouldEnterIncrementalMode() const;
  bool shouldUpdateHandAnchorPose() const;

  void updateHandAnchorPose(const ArmPose& leftAnchorPose, const ArmPose& rightAnchorPose);
  void computeHandPoseDifference(const ArmPose& leftHandPose, const ArmPose& rightHandPose);

  ArmPose computeFinalHandePose(const ArmPose& leftAnchorPose, const ArmPose& leftDiffPose);
  ArmPose interpolateArmPose(const ArmPose& anchorPose, const ArmPose& diffPose);

  const ArmPose& getVrLeftHandPose() const;
  const ArmPose& getVrRightHandPose() const;
};

}  // namespace HighlyDynamic
