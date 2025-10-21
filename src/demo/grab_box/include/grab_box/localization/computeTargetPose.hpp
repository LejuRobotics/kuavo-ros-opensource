#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Dense>
#include <cmath>
#include "grab_box/utils/poseTransformer.h"


using namespace autoHeadChase;
namespace GrabBox
{
  class ComputeTargetPose : public BT::SyncActionNode
  {
  public:
    ComputeTargetPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {
      }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<Eigen::Vector3d>("box_pos"),
        BT::InputPort<Eigen::Vector4d>("box_quat"),
        BT::InputPort<Eigen::Vector3d>("target_box_offset"),
        BT::OutputPort<Eigen::Vector4d>("target_pose")
      };
    }

    // 计算 target_pose 
    BT::NodeStatus tick() override
    {
      double com_height;

      while(!ros::param::has("/com_height"))
      {
        ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      // ROS_INFO_STREAM("com_height parameter is founded.");
      ros::param::get("/com_height", com_height);
      // std::cout << "[ComputeTargetPose] comHeight: " << com_height<<std::endl;

      Eigen::Vector3d box_pos;
      Eigen::Vector4d box_quat;
      Eigen::Vector3d target_box_offset;

      // 获取 box_pos 和 box_quat 
      if (!getInput("box_pos", box_pos) || !getInput("box_quat", box_quat) || !getInput("target_box_offset", target_box_offset)) {
        ROS_ERROR("Failed to get input ports for box_pos or box_quat");
        return BT::NodeStatus::FAILURE;
      }

      Eigen::VectorXd box_pose_world(7);
      box_pose_world << box_pos(0), box_pos(1), box_pos(2), box_quat(0), box_quat(1), box_quat(2), box_quat(3);

      // std::cout << "tag_pose_world_ : " << box_pose_world.transpose() << std::endl;

      Eigen::VectorXd destination_pose_box(7);
      destination_pose_box << target_box_offset(0), target_box_offset(1), target_box_offset(2), 0.0, 0.0, 0.0, 1.0;

    double delta_roll = 0.0, delta_pitch = M_PI/2, delta_yaw = M_PI/2;
    
    //根据设定的相对位置和BOX生成目标位置
      Eigen::VectorXd destination_pose_ = getDestination(destination_pose_box, box_pose_world);

      // std::cout << "destination_pose_:  " << destination_pose_.transpose() << std::endl;

      // 创建目标位姿的四维数组（X, Y, Z, YAW）
      Eigen::Vector4d target_pose;
      target_pose << destination_pose_(0), destination_pose_(1), com_height, getYawFromQuaternion(destination_pose_.segment<4>(3)) * 180 / M_PI;

      // std::cout << "target_pose: " << target_pose << std::endl;

      // 设置输出端口
      setOutput("target_pose", target_pose); 

      return BT::NodeStatus::SUCCESS;
    }
  private:

   Eigen::VectorXd getDestination(const Eigen::VectorXd& destination_pose_box, const Eigen::VectorXd& box_pose)
    {
      return PoseTransformer::transformPoseToWorld(destination_pose_box, box_pose);
    }

    // 使用四元数计算 Yaw
    double getYawFromQuaternion(const Eigen::Vector4d& quat)
    {
      Eigen::Quaterniond q(quat(3), quat(0), quat(1), quat(2));
      return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

  };
} // namespace GrabBox
