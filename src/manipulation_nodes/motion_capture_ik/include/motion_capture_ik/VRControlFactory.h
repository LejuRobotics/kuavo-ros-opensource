#pragma once

#include <ros/ros.h>

#include <memory>

#include "motion_capture_ik/Quest3HandEE_ROS.h"
#include "motion_capture_ik/Quest3IkROS.h"

namespace HighlyDynamic {
/*
* kmpc模式：
* VR数据 → Quest3ArmInfoTransformer转换 → 发布/mm/two_arm_hand_pose_cmd消息
* → kmpc优化出关节参考轨迹 → WBC → 关节指令
ik模式：
* VR数据 → Quest3ArmInfoTransformer转换 → 基于Drake的IK逆解获得关节参考轨迹
* → /kuavo_arm_traj → WBC或（SQP-MPC→WBC） → 关节指令
*/
enum class VRControlMode {
  KMPC_INCREMENTAL,  // bone_pose→transform→two_hand_pse
  DRAKE_IK_DIRECT,   // bone_pose→transform→[two_hand_pose, two_elbow_position]→IK→kuavo_arm_traj
};

std::unique_ptr<Quest3IkROS> createQuest3IkROS(ros::NodeHandle& nodeHandle,
                                               double publishRate,
                                               bool debugPrint = false);

std::unique_ptr<Quest3HandEE_ROS> createQuest3HandPoseROS(ros::NodeHandle& nodeHandle,
                                                          double publishRate,
                                                          bool debugPrint = false);

std::unique_ptr<ArmControlBaseROS> createVRController(ros::NodeHandle& nodeHandle,
                                                      VRControlMode mode,
                                                      double publishRate,
                                                      bool debugPrint = false);

}  // namespace HighlyDynamic
