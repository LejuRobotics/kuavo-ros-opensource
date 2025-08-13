#include "mobile_manipulator_controllers/mobileManipulatorController.h"
#include <thread>

using namespace mobile_manipulator_controller;

int main(int argc, char **argv)
{
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  // 等待并获取关键参数
  ROS_INFO("Waiting for required parameters...");
  
  // 等待必需参数
  std::string taskFile, libFolder, urdfFile;
  // 获取可选参数，带重试机制

  MpcType mpcType;
  int dummySimBase = 1;
  int dummySimArm = 1;
  bool visualizeMm = true;
  int mpcTypeInt = 0;
  while (!nodeHandle.hasParam("/mm/taskFile") || !nodeHandle.hasParam("/mm/libFolder") || !nodeHandle.hasParam("/mm/urdfFile")
  || !nodeHandle.hasParam("/mm/mpcType") || !nodeHandle.hasParam("/dummy_sim") || !nodeHandle.hasParam("/dummy_sim_arm")
  || !nodeHandle.hasParam("/visualize_mm"))
  {
    ROS_WARN("Waiting for required parameters: /mm/taskFile, /mm/libFolder, /mm/urdfFile, /mm/mpcType, /dummy_sim, /dummy_sim_arm, /visualize_mm");
    ros::Duration(1.0).sleep();
    if (!ros::ok()) return 1;
  }
  
  nodeHandle.getParam("/mm/taskFile", taskFile);
  nodeHandle.getParam("/mm/libFolder", libFolder);
  nodeHandle.getParam("/mm/urdfFile", urdfFile);
  nodeHandle.getParam("/dummy_sim", dummySimBase);
  nodeHandle.getParam("/dummy_sim_arm", dummySimArm);
  nodeHandle.getParam("/visualize_mm", visualizeMm);
  nodeHandle.getParam("/mm/mpcType", mpcTypeInt);
  mpcType = static_cast<MpcType>(mpcTypeInt);
  ROS_INFO("Required parameters loaded successfully");

  // 打印参数
  ROS_INFO("taskFile: %s", taskFile.c_str());
  ROS_INFO("libFolder: %s", libFolder.c_str());
  ROS_INFO("urdfFile: %s", urdfFile.c_str());
  ROS_INFO("mpcType: %d", mpcTypeInt);
  ROS_INFO("dummySimBase: %d", dummySimBase);
  ROS_INFO("dummySimArm: %d", dummySimArm);
  ROS_INFO("visualizeMm: %d", visualizeMm);

  int frequency = 100;
  // Initialize controller
  MobileManipulatorController controller(nodeHandle, taskFile, libFolder, urdfFile, mpcType, frequency, dummySimBase, dummySimArm, visualizeMm);
  controller.init(nodeHandle);

  // 创建独立线程处理ROS消息
  std::thread spin_thread([&](){
    ros::spin();
  });
  spin_thread.detach();

  // 主控制循环，不会被ROS服务阻塞
  ros::Rate loopRate(frequency);
  while(ros::ok())
  {
    controller.update();
    loopRate.sleep();
  }
  
  return 0;
}