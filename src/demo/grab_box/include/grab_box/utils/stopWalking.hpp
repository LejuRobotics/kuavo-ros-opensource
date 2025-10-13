#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <string>
#include <geometry_msgs/Twist.h>

namespace GrabBox
{
  class StopWalking : public BT::SyncActionNode
  {
  public:
    StopWalking(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
      ros::NodeHandle nh;
      cmd_pose_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_pose", 10);
    }
    static BT::PortsList providedPorts()
    {
      return {};
    }
    BT::NodeStatus tick() override final
    {
      for(int i = 0; i < 10; i++)
      {
        pubZeroCmdPose();
        ros::Duration(0.01).sleep();
      }
      return BT::NodeStatus::SUCCESS;
    }
  private:

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

    ros::Publisher cmd_pose_pub_;
  };
} // namespace GrabBox