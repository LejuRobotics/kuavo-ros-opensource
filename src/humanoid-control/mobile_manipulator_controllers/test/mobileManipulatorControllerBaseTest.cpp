#include <pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>

using namespace ocs2;
using namespace mobile_manipulator_controller;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "mobile_manipulator_controller_base_test");
  ros::NodeHandle nodeHandle;

  std::string taskFile, libFolder, urdfFile;
  MpcType mpcType = MpcType::SQP;

  nodeHandle.getParam("/mm/taskFile", taskFile);
  nodeHandle.getParam("/mm/libFolder", libFolder);
  nodeHandle.getParam("/mm/urdfFile", urdfFile);
  ROS_INFO_STREAM("taskFile: " << taskFile);
  ROS_INFO_STREAM("libFolder: " << libFolder);
  ROS_INFO_STREAM("urdfFile: " << urdfFile);

  ControlType control_type = ControlType::None;
  MobileManipulatorControllerBase controller(nodeHandle, taskFile, libFolder, urdfFile, mpcType, 100, control_type, true, true);

  ros::Rate rate(100);
  size_t count = 0;
  vector_t nextState = vector_t::Zero(20);

  vector_t target_state = vector_t::Zero(20);
  target_state(6 + 3) = -M_PI/2;
  target_state(6 + 3 + 7) = -M_PI/2;
  vector_t externalTarget = controller.getTargetFromState(target_state);
  const TargetTrajectories target_trajectories({0.0, 1.0}, {externalTarget, externalTarget}, {vector_t::Zero(20), vector_t::Zero(20)});
  controller.setTargetTrajectory(target_trajectories);

  vector_t externalState = vector_t::Zero(20);
  while(ros::ok())
  {
    if(count < 300)
    {
      externalState(2) = 0.2*sin(count*0.02);
      externalState(3) = 0.5*sin(count*0.02);
      externalState(4) = -0.3*(cos(count*0.02) - 1);
    }
    // 停止
    if(count == 400)
    {
      controller.stop();
      // externalState(3) = 0.3*(sin(count*0.01) + 1);
    }
    // 继续计算，无跳变
    if(count == 500)
    {
      controller.start();
    }
    // 停止
    if(count == 600)
    {
      controller.stop();
      // externalState(3) = 0.3*(sin(count*0.01) + 1);
    }
    // 继续计算，有跳变
    if(count == 700)
    {
      controller.start();
      externalState(2) = -0.3;
    }
    // 调用reset方法重置，无跳变
    if(count ==800)
    {
      externalState(2) = 0.3;
      controller.reset(externalState);
      // 设置目标轨迹是没有必要的，因为reset方法会自动设置目标轨迹
      // vector_t tmp_eef_target = controller.getTargetFromState(externalState);
      // const TargetTrajectories tmp_target_trajectories({0.0, 1.0}, {tmp_eef_target, tmp_eef_target}, {vector_t::Zero(20), vector_t::Zero(20)});
      // controller.setTargetTrajectory(tmp_target_trajectories);
    }
    vector_t optimizedInput = vector_t::Zero(20);
    controller.update(externalState, nextState, optimizedInput);
    externalState = nextState;
    // std::cout << "next state base:      " << nextState.head(6).transpose() << std::endl;
    // std::cout << "optimized input base:      " << optimizedInput.head(6).transpose() << std::endl;
    // std::cout << "next state left arm:  " << nextState.segment(6, 7).transpose() << std::endl;
    // std::cout << "next state right arm: " << nextState.segment(13, 7).transpose() << std::endl;
    ros::spinOnce();
    rate.sleep();
    count = (count < 1000) ? count+1 : 0;
    if(count == 1000)
    {
      controller.reset(target_state);
      controller.setTargetTrajectory(target_trajectories);
    }
  }
  return 0;
}