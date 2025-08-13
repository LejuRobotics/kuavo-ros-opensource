#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <string>
#include <geometry_msgs/Twist.h>

namespace GrabBox
{
  class ResetState : public BT::SyncActionNode
  {
  public:
    ResetState(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      eef_wrench_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/hand_wrench_cmd", 10);
      enable_wbc_arm_trajectory_control_srv_ = nh.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_mm_wbc_arm_trajectory_control");
      cmd_pose_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_pose", 10);
    }
    static BT::PortsList providedPorts()
    {
      return {};
    }
    BT::NodeStatus tick() override final
    {
      if(!changeArmCtrlModeSrv(1))
      {
        ROS_ERROR("[ResetState] Failed to change arm control mode to swing arm.");
        return BT::NodeStatus::FAILURE;
      }
      pubZeroEefWrench();
      ROS_INFO("[ResetState] Published zero eef wrench command.");
      enableWbcArmCtrl(false);
      for(int i = 0; i < 10; i++)
      {
        pubZeroCmdPose();
        ros::Duration(0.01).sleep();
      }
      if(!changeKinematicMpcControlMode(0))// 0:Do NOT control, 1:arm only control mode, 2:base only control mode, 3: control base and arm mode
        return BT::NodeStatus::FAILURE;
      return BT::NodeStatus::SUCCESS;
    }
  private:

    bool enableWbcArmCtrl(int mode)
    {
      kuavo_msgs::changeArmCtrlMode srv;
      srv.request.control_mode = mode;
      if (enable_wbc_arm_trajectory_control_srv_.call(srv)) 
      {
        return true;
      }
      else
        ROS_ERROR("Failed to call service enable_mm_wbc_arm_trajectory_control");
      return false;
    }
    void pubZeroEefWrench()
    {
      Vector6d zero_wrench = Vector6d::Zero();
      eef_wrench_pub_.publish(getEefWrenchCmdMsg(zero_wrench, zero_wrench));
    }
    void pubZeroCmdPose()
    {
      geometry_msgs::Twist cmd_pose;
      cmd_pose.linear.x = 0.0;
      cmd_pose.linear.y = 0.0;
      cmd_pose.linear.z = 0.0;
      cmd_pose.angular.x = 0.0;
      cmd_pose.angular.y = 0.0;
      cmd_pose.angular.z = 0.0;
      cmd_pose_pub_.publish(cmd_pose);
    }

    ros::Publisher eef_wrench_pub_;
    ros::Publisher cmd_pose_pub_;
    ros::ServiceClient enable_wbc_arm_trajectory_control_srv_;
  };
} // namespace GrabBox