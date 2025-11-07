#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include "motion_capture_ik/json.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/Quest3Debugger.h"
#include "motion_capture_ik/TwoStageTorsoIKRec.h"

namespace HighlyDynamic {

class Quest3IkROS final : public ArmControlBaseROS {
 public:
  explicit Quest3IkROS(ros::NodeHandle& nodeHandle,
                       double publishRate,
                       bool debugPrint = false,
                       ArmIdx ctrlArmIdx = ArmIdx::BOTH);

  ~Quest3IkROS();

  void initialize(const nlohmann::json& configJson) override;
  void run() override;

 private:
  std::thread ikSolveThread_;
  std::mutex bonePoseHandElbowMutex_;

  int jointStateSize_;
  ArmIdx ctrlArmIdx_;  // 控制哪个手臂：LEFT, RIGHT, 或 BOTH

  std::unique_ptr<TwoStageTorsoIK> twoStageTorsoIkPtr_;
  std::unique_ptr<Quest3Debugger> quest3DebuggerPtr_;

  // Drake diagram and context for plant
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;

  std::mutex ikResultMutex_;

  void processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) override;

  void solveIkHandElbowThreadFuntion();

  void publishJointStates(const Eigen::VectorXd& jointPositions);

  void publishHeadBodyPose(const HeadBodyPose& headBodyPose);

  Eigen::VectorXd interpolateJointAngles(const Eigen::VectorXd& current,
                                         const Eigen::VectorXd& target,
                                         double threshold,
                                         double maxSpeed,
                                         bool& isFinished);
};

}  // namespace HighlyDynamic
