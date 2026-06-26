// v62 (5W) 平台专用：半身（pure arm 14 DOF）增量 IK 节点。
//
// 跟 ik_ros_uni_cpp_node 共用 Quest3IkIncrementalROS 实现，但用单独的可执行 + 节点名，
// 方便 launch 按 robot_version 切换、并与 v52 入口完全解耦。
//
// 关键约束：
//   - 该节点不参与 chest 跟踪（无 chest 4 关节）、不发 cmd_vel；
//   - 加载 kuavo_v62/kuavo.json 时，Quest3IkIncrementalROS 会优先读 arm_only_urdf
//     字段（v62 提供，14 DOF 纯双臂 URDF），fallback 才走 arm_urdf；
//   - chest / 底盘控制由 5W 平台原生方案（lb_ctrl_api → MPC）负责。
#include <ros/package.h>
#include <ros/ros.h>

#include <cmath>
#include <fstream>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <noitom_hi5_hand_udp_python/JoySticks.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include "motion_capture_ik/Quest3IkIncrementalROS.h"
#include "motion_capture_ik/json.hpp"
#include "leju_utils/define.hpp"

// 5W 轮式底盘 cmd_vel 转发器：v52 IK 类不发 cmd_vel（v52 平台无底盘），
// 但 v62 5W 平台需要把摇杆映射到底盘速度。这里独立做这件事，跟 IK 解耦。
// 若 single_hand_mode=true 且任一 grip 按下，或 /single_hand_torso_active=true（含 reset busy），让位 torso_joystick_node 独占摇杆。
class ChassisCmdVelForwarder {
 public:
  explicit ChassisCmdVelForwarder(ros::NodeHandle& nh) {
    nh.param("/single_hand_mode", singleHandTorsoMode_, false);
    nh.param("/vr_cmd_vel/linear_scale_x",  linearXLimit_,  0.4);
    nh.param("/vr_cmd_vel/linear_scale_y",  linearYLimit_,  0.4);
    nh.param("/vr_cmd_vel/angular_scale_z", angularZLimit_, 0.25);
    cmdVelPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    joySub_ = nh.subscribe("/quest_joystick_data", 10, &ChassisCmdVelForwarder::joystickCallback, this);
    singleHandTorsoActiveSub_ = nh.subscribe("/single_hand_torso_active", 1,
                                             &ChassisCmdVelForwarder::singleHandTorsoActiveCallback, this);
    ROS_INFO("[ChassisCmdVelForwarder] singleHandTorsoMode_=%s, limits lin_x=%.2f lin_y=%.2f ang_z=%.2f",
             singleHandTorsoMode_ ? "true" : "false", linearXLimit_, linearYLimit_, angularZLimit_);
  }

 private:
  void singleHandTorsoActiveCallback(const std_msgs::Bool::ConstPtr& msg) {
    singleHandTorsoActive_ = msg->data;
  }

  void joystickCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
    // single_hand_mode 下，grip 按住或 /single_hand_torso_active=true（包含 reset busy）时，torso_joystick_node 独占摇杆，不发 cmd_vel
    const bool leftGrip = msg->left_grip > 0.7;
    const bool rightGrip = msg->right_grip > 0.7;
    if (singleHandTorsoMode_ && (leftGrip || rightGrip || singleHandTorsoActive_)) return;

    const double nx = std::max(-1.0, std::min(1.0, static_cast<double>(msg->left_y)));
    const double ny = std::max(-1.0, std::min(1.0, -static_cast<double>(msg->left_x)));
    const double nw = std::max(-1.0, std::min(1.0, -static_cast<double>(msg->right_x)));

    geometry_msgs::Twist twist;
    twist.linear.x = nx * linearXLimit_;
    twist.linear.y = ny * linearYLimit_;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = nw * angularZLimit_;

    if (std::abs(twist.linear.x) > 1e-2 || std::abs(twist.linear.y) > 1e-2 ||
        std::abs(twist.angular.z) > 1e-2) {
      cmdVelPub_.publish(twist);
    }
  }

  ros::Publisher cmdVelPub_;
  ros::Subscriber joySub_;
  ros::Subscriber singleHandTorsoActiveSub_;
  bool singleHandTorsoMode_ = false;
  bool singleHandTorsoActive_ = false;
  double linearXLimit_ = 0.4;
  double linearYLimit_ = 0.4;
  double angularZLimit_ = 0.25;
};

int getRobotVersion(ros::NodeHandle& nodeHandle) {
  while (!nodeHandle.hasParam("/robot_version") && ros::ok()) {
    ROS_INFO("Waiting for robot_version parameter...");
    ros::Duration(0.1).sleep();
  }

  int robotVersion = 42;
  nodeHandle.getParam("/robot_version", robotVersion);
  std::cout << "robotVersionInt: " << robotVersion << std::endl;
  return robotVersion;
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
  int ctrlArmIdx = 2;
  // launch 里 <node name="ik_ros_uni_cpp_node"> 跟 v52 入口同名（互斥不冲突），
  // 跟 v52 cpp 的 path 保持一致，避免 launch <param name="/quest3/..."> 的 namespace 跟 code 不匹配
  nodeHandle.param("ik_ros_uni_cpp_node/ctrl_arm_idx", ctrlArmIdx, 2);
  ROS_INFO("Read ctrl_arm_idx parameter: %d", ctrlArmIdx);

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

int main(int argc, char** argv) {
  std::cout << "\033[92mRunning wheel_arm_only_ik_ros_uni_cpp_node (v62 half-body IK)\033[0m" << std::endl;

  ros::init(argc, argv, "wheel_arm_only_ik_ros_uni_cpp_node");
  ros::NodeHandle nodeHandle;

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

  // ChassisCmdVelForwarder 的 subscriber 用默认 callback queue，
  // 会被 Quest3IkIncrementalROS::run() 里的 ros::spin 自动处理；不要再额外加 AsyncSpinner。
  ChassisCmdVelForwarder chassisCmdVelForwarder(nodeHandle);

  HighlyDynamic::Quest3IkIncrementalROS quest3IkIncrementalROS(nodeHandle, 100, false, ctrlArmIdx);
  quest3IkIncrementalROS.initialize(jsonData);

  // [wheel_arm_only] 5W 平台 controller 必需的 VR 增量模式开关
  // 不调这 3 个 service，下游 controller 不会接收 /kuavo_arm_traj 的关节命令
  auto callVrSvc = [&nodeHandle](const std::string& name) {
    if (!ros::service::waitForService(name, ros::Duration(30.0))) {
      ROS_WARN("[wheel_arm_only_ik] Timeout waiting for service: %s", name.c_str());
      return;
    }
    ros::ServiceClient client = nodeHandle.serviceClient<std_srvs::SetBool>(name);
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (client.call(srv) && srv.response.success) {
      ROS_INFO("[wheel_arm_only_ik] %s set to true -> OK", name.c_str());
    } else {
      ROS_WARN("[wheel_arm_only_ik] %s -> FAILED: %s", name.c_str(), srv.response.message.c_str());
    }
  };
  ROS_INFO("[wheel_arm_only_ik] Configuring VR incremental mode via controller services...");
  callVrSvc("/enable_vr_arm_kpkd");
  callVrSvc("/enable_vr_arm_accel_task");
  callVrSvc("/enable_arm_traj_interpolator");
  ROS_INFO("[wheel_arm_only_ik] VR incremental mode configuration done.");

  quest3IkIncrementalROS.run();

  return 0;
}
