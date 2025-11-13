#include "motion_capture_ik/VRControlFactory.h"

#include <stdexcept>

namespace HighlyDynamic {

std::unique_ptr<ArmControlBaseROS> createVRController(ros::NodeHandle& nodeHandle,
                                                      VRControlMode mode,
                                                      double publishRate,
                                                      bool debugPrint) {
  switch (mode) {
    case VRControlMode::DRAKE_IK_DIRECT:
      return createQuest3IkROS(nodeHandle, publishRate, debugPrint);

    case VRControlMode::KMPC_INCREMENTAL:
      return createQuest3HandPoseROS(nodeHandle, publishRate, debugPrint);

    default:
      throw std::invalid_argument("Unsupported VR control mode: " + std::to_string(static_cast<int>(mode)));
  }
}
std::unique_ptr<Quest3IkROS> createQuest3IkROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint) {
  return std::make_unique<Quest3IkROS>(nodeHandle, publishRate, debugPrint);
}

std::unique_ptr<Quest3HandEE_ROS> createQuest3HandPoseROS(ros::NodeHandle& nodeHandle,
                                                          double publishRate,
                                                          bool debugPrint) {
  return std::make_unique<Quest3HandEE_ROS>(nodeHandle, publishRate, debugPrint);
}
}  // namespace HighlyDynamic
