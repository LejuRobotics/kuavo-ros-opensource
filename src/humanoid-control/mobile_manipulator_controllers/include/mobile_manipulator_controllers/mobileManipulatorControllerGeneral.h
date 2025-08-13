#include <pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>
#include <sensor_msgs/JointState.h>
#include "kuavo_msgs/changeTorsoCtrlMode.h"

using namespace ocs2;
namespace mobile_manipulator_controller
{
  class MobileManipulatorControllerGeneral : public MobileManipulatorControllerBase
  {
    public:
      MobileManipulatorControllerGeneral(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile, MpcType mpcType, int freq, 
        ControlType control_type=ControlType::BaseArm, bool dummySimArm=true, bool visualizeMm=true);
      bool init(double comHeight);
      void update();
    
    private:
      void humanoidStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
      sensor_msgs::JointState getJointStatesMsg(const vector_t& q_arm, const vector_t& dq_arm);
      int controlHumanoid(const vector_t& mmState, const vector_t& mmInput);
      bool controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
      void controlBasePos(const vector_t& mmState);

      double comHeight_{0};
      // ControlType controlType_ = ControlType::None;
      // ControlType lastControlType_ = ControlType::ArmOnly; // 初始化为ArmOnly，在第一次启动时，可以打印出MPC的初始状态

      // ros
      ros::NodeHandle nh_;
      ros::Subscriber humanoidStateSubscriber_;
      ros::Publisher armTrajPublisher_;
      ros::Publisher humanoidCmdPosPublisher_;
      ros::ServiceServer kinematicMpcControlSrv_;
      ocs2::vector_t humanoidState_;
      bool recievedObservation_{false};
  };
} // namespace mobile_manipulator_controller