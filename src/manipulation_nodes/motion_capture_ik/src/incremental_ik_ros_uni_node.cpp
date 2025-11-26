#include <ros/package.h>
#include <ros/ros.h>

#include <fstream>
#include <iostream>

#include "motion_capture_ik/Quest3IkIncrementalROS.h"
#include "motion_capture_ik/json.hpp"
#include "leju_utils/define.hpp"

int getRobotVersion(ros::NodeHandle& nodeHandle) {
  // 持续查询robot_version参数，否则阻塞程序
  while (!nodeHandle.hasParam("/robot_version") && ros::ok()) {
    ROS_INFO("Waiting for robot_version parameter...");
    ros::Duration(0.1).sleep();
  }

  int robot_version = 42;
  nodeHandle.getParam("/robot_version", robot_version);
  std::cout << "robotVersionInt: " << robot_version << std::endl;
  return robot_version;
}

void loadJsonConfig(nlohmann::json& jsonData, const std::string& filename) {
  std::ifstream file(filename);
  if (file.is_open()) {
    file >> jsonData;
    std::cout << "Successfully loaded config file: " << filename << std::endl;
  } else {
    std::cerr << "Failed to open config file: " << filename << std::endl;
    throw std::runtime_error("Failed to load JSON configuration file");
  }
}

ArmIdx getCtrlArmIdx(ros::NodeHandle& nodeHandle) {
  int ctrl_arm_idx = 2;  // 默认值：控制双臂
  nodeHandle.param("incremental_ik_ros_uni_node/ctrl_arm_idx", ctrl_arm_idx, 2);
  ROS_INFO("Read ctrl_arm_idx parameter: %d", ctrl_arm_idx);

  // 转换为ArmIdx枚举
  switch (ctrl_arm_idx) {
    case 0:
      return ArmIdx::LEFT;
    case 1:
      return ArmIdx::RIGHT;
    case 2:
      return ArmIdx::BOTH;
    default:
      ROS_WARN("Invalid ctrl_arm_idx value: %d, using default BOTH", ctrl_arm_idx);
      return ArmIdx::BOTH;
  }
}

int main(int argc, char** argv) {
  std::cout << "\033[92mRunning incremental_ik_ros_uni_node\033[0m" << std::endl;

  // Initialize ROS node
  ros::init(argc, argv, "incremental_ik_ros_uni_node");
  ros::NodeHandle nodeHandle;

  // 从ROS参数服务器读取ctrl_arm_idx参数
  ArmIdx ctrlArmIdx = getCtrlArmIdx(nodeHandle);
  std::string armControlMsg;
  switch (ctrlArmIdx) {
    case ArmIdx::LEFT:
      armControlMsg = "LEFT";
      break;
    case ArmIdx::RIGHT:
      armControlMsg = "RIGHT";
      break;
    case ArmIdx::BOTH:
      armControlMsg = "BOTH";
      break;
  }
  ROS_INFO("\033[92mControl %s arms.\033[0m", armControlMsg.c_str());

  int robotVersionInt = getRobotVersion(nodeHandle);
  std::string modelConfigFile =
      ros::package::getPath("kuavo_assets") + "/config/kuavo_v" + std::to_string(robotVersionInt) + "/kuavo.json";

  nlohmann::json jsonData;
  loadJsonConfig(jsonData, modelConfigFile);

  HighlyDynamic::Quest3IkIncrementalROS quest3IkIncrementalROS(nodeHandle, 100, false, ctrlArmIdx);
  quest3IkIncrementalROS.initialize(jsonData);
  quest3IkIncrementalROS.run();

  return 0;
}
