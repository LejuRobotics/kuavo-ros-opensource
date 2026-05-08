#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>

#include "motion_capture_ik/WheelQuest3IkIncrementalROS.h"
#include "motion_capture_ik/json.hpp"
#include "leju_utils/define.hpp"

int wheelGetRobotVersion(ros::NodeHandle& nodeHandle) {
  // 持续查询robot_version参数，否则阻塞程序
  while (!nodeHandle.hasParam("/robot_version") && ros::ok()) {
    ROS_INFO("Waiting for robot_version parameter...");
    ros::Duration(0.1).sleep();
  }

  int robotVersion = 42;
  nodeHandle.getParam("/robot_version", robotVersion);
  std::cout << "robotVersionInt: " << robotVersion << std::endl;
  return robotVersion;
}

void wheelLoadJsonConfig(nlohmann::json& jsonData, const std::string& filename) {
  std::ifstream file(filename);
  if (file.is_open()) {
    file >> jsonData;
    std::cout << "Successfully loaded config file: " << filename << std::endl;
  } else {
    std::cerr << "Failed to open config file: " << filename << std::endl;
    throw std::runtime_error("Failed to load JSON configuration file");
  }
}

ArmIdx wheelGetCtrlArmIdx(ros::NodeHandle& nodeHandle) {
  int ctrlArmIdx = 2;  // 默认值：控制双臂
  nodeHandle.param("ik_ros_uni_cpp_node/ctrl_arm_idx", ctrlArmIdx, 2);
  ROS_INFO("Read ctrl_arm_idx parameter: %d", ctrlArmIdx);

  // 转换为ArmIdx枚举
  switch (ctrlArmIdx) {
    case 0:
      return ArmIdx::LEFT;
    case 1:
      return ArmIdx::RIGHT;
    case 2:
      return ArmIdx::BOTH;
    default:
      ROS_WARN("Invalid ctrl_arm_idx value: %d, using default BOTH", ctrlArmIdx);
      return ArmIdx::BOTH;
  }
}

static ros::NodeHandle* g_nh = nullptr;
static std::atomic<bool> g_teardown_done{false};

static void callVrIncrementalService(ros::NodeHandle& nh, const std::string& service_name, bool enable)
{
  constexpr double kWaitTimeout = 30.0;
  if (enable && !ros::service::waitForService(service_name, ros::Duration(kWaitTimeout))) {
    ROS_WARN("[wheel_ik] Timeout waiting for service: %s", service_name.c_str());
    return;
  }
  if (!enable && !ros::service::exists(service_name, false)) {
    return;  // 退出时服务不存在则跳过
  }
  ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(service_name);
  std_srvs::SetBool srv;
  srv.request.data = enable;
  if (client.call(srv) && srv.response.success) {
    ROS_INFO("[wheel_ik] %s set to %s -> OK", service_name.c_str(), enable ? "true" : "false");
  } else {
    ROS_WARN("[wheel_ik] %s -> FAILED: %s", service_name.c_str(), srv.response.message.c_str());
  }
}

static void setupVrIncrementalMode(ros::NodeHandle& nh)
{
  ROS_INFO("[wheel_ik] Configuring VR incremental mode via controller services...");
  callVrIncrementalService(nh, "/enable_vr_arm_kpkd",          true);
  callVrIncrementalService(nh, "/enable_vr_arm_accel_task",    true);
  callVrIncrementalService(nh, "/enable_arm_traj_interpolator", true);
  ROS_INFO("[wheel_ik] VR incremental mode configuration done.");
}

static void teardownVrIncrementalMode()
{
  if (g_teardown_done.exchange(true) || !g_nh) return;
  ROS_INFO("[wheel_ik] Restoring VR incremental mode parameters to default...");
  callVrIncrementalService(*g_nh, "/enable_vr_arm_kpkd",          false);
  callVrIncrementalService(*g_nh, "/enable_vr_arm_accel_task",    false);
  callVrIncrementalService(*g_nh, "/enable_arm_traj_interpolator", false);
  ROS_INFO("[wheel_ik] VR incremental mode parameters restored.");
}

static void signalHandler(int /*sig*/)
{
  teardownVrIncrementalMode();
  ros::shutdown();
}

int main(int argc, char** argv) {
  std::cout << "\033[92mRunning ik_ros_uni_cpp_node\033[0m" << std::endl;

  // Initialize ROS node（关闭默认 SIGINT 处理，由 signalHandler 接管）
  ros::init(argc, argv, "ik_ros_uni_cpp_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nodeHandle;
  g_nh = &nodeHandle;
  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);

  // 从ROS参数服务器读取ctrl_arm_idx参数
  ArmIdx ctrlArmIdx = wheelGetCtrlArmIdx(nodeHandle);
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

  int robotVersionInt = wheelGetRobotVersion(nodeHandle);
  std::string modelConfigFile =
      ros::package::getPath("kuavo_assets") + "/config/kuavo_v" + std::to_string(robotVersionInt) + "/kuavo.json";

  nlohmann::json jsonData;
  wheelLoadJsonConfig(jsonData, modelConfigFile);

  HighlyDynamic::WheelQuest3IkIncrementalROS quest3IkIncrementalROS(nodeHandle, 100, false, ctrlArmIdx);
  quest3IkIncrementalROS.initialize(jsonData);

  setupVrIncrementalMode(nodeHandle);

  quest3IkIncrementalROS.run();

  // run() 正常返回时也执行复位（与信号处理共用原子标志，保证只执行一次）
  teardownVrIncrementalMode();

  return 0;
}
