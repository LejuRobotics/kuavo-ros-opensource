#include <ros/package.h>
#include <ros/ros.h>

#include <fstream>

#include "motion_capture_ik/VRControlFactory.h"
#include "motion_capture_ik/json.hpp"

using namespace HighlyDynamic;

int getRobotVersion(ros::NodeHandle& nodeHandle) {
  int robot_version = 45;
  while (!nodeHandle.hasParam("/robot_version") && ros::ok()) {
    ros::Duration(0.1).sleep();
  }
  nodeHandle.getParam("/robot_version", robot_version);
  return robot_version;
}

void loadJsonConfig(nlohmann::json& jsonData, const std::string& filename) {
  std::ifstream file(filename);
  if (file.is_open()) {
    file >> jsonData;
  } else {
    throw std::runtime_error("Failed to load JSON configuration file");
  }
}

VRControlMode getVRControlMode(ros::NodeHandle& nodeHandle) {
  std::string vrModeStr;
  while (!nodeHandle.hasParam("vr_control_mode") && ros::ok()) {
    ros::Duration(0.1).sleep();
  }
  nodeHandle.param<std::string>("vr_control_mode", vrModeStr, "DRAKE_IK_DIRECT");

  if (vrModeStr == "KMPC_INCREMENTAL") {
    return VRControlMode::KMPC_INCREMENTAL;
  } else if (vrModeStr == "DRAKE_IK_DIRECT") {
    return VRControlMode::DRAKE_IK_DIRECT;
  } else {
    throw std::invalid_argument("Invalid VR control mode: " + vrModeStr);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nodeHandle;

  int robotVersionInt = getRobotVersion(nodeHandle);
  std::string modelConfigFile =
      ros::package::getPath("kuavo_assets") + "/config/kuavo_v" + std::to_string(robotVersionInt) + "/kuavo.json";

  nlohmann::json jsonData;
  loadJsonConfig(jsonData, modelConfigFile);

  VRControlMode vrControlMode = getVRControlMode(nodeHandle);
  auto controller = createVRController(nodeHandle, vrControlMode, 100.0);

  controller->initialize(jsonData);
  controller->run();

  return 0;
}
