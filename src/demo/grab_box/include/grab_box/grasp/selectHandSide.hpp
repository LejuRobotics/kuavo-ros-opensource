#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/Odometry.h> // For robot's pose, if needed directly (though TF is preferred)
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

#include "grab_box/utils/poseTransformer.h" // Assuming this is for local_offset to world_target

// Using namespace autoHeadChase for PoseTransformer might be too broad here
// It's better to qualify it like autoHeadChase::PoseTransformer::transformPoseToWorld

namespace GrabBox
{
  class SelectHandSide : public BT::StatefulActionNode // Changed to StatefulActionNode
  {
  public:
    SelectHandSide(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config), nh_() 
    {
        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Note: Odom subscription might not be strictly necessary if TF tree is complete
        // from "odom" to "base_link", but can be kept for direct robot pose if needed.
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<Eigen::Vector3d>("part_pos"),         // Part's position in world (odom)
        BT::InputPort<Eigen::Vector4d>("part_quat"),        // Part's orientation in world (odom)
        BT::InputPort<Eigen::Vector3d>("target_part_offset"), // Offset from part frame
        BT::OutputPort<int>("hand_side")                 // 0 for Left, 1 for Right
        // BT::OutputPort<Eigen::Vector4d>("target_pose") // Removing this as per new requirements
      };
    }

    BT::NodeStatus onStart() override 
    {
        ROS_INFO("[%s] Node started.", name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      Eigen::Vector3d part_pos_world_in;
      Eigen::Vector4d part_quat_world_in;
      Eigen::Vector3d target_offset_in_part_frame;

      if (!getInput("part_pos", part_pos_world_in) || 
          !getInput("part_quat", part_quat_world_in) || 
          !getInput("target_part_offset", target_offset_in_part_frame)) 
      {
        ROS_ERROR("[%s] Failed to get input ports.", name().c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 1. Construct part_pose_world (Eigen::VectorXd 7D) from inputs
      Eigen::VectorXd part_pose_world(7);
      part_pose_world.head<3>() = part_pos_world_in;
      part_pose_world.tail<4>() = part_quat_world_in; // Assuming (x,y,z,w) from input

      // 2. Construct offset_pose_relative_to_part (Eigen::VectorXd 7D)
      Eigen::VectorXd offset_pose_in_part_frame(7);
      offset_pose_in_part_frame.head<3>() = target_offset_in_part_frame;
      offset_pose_in_part_frame.tail<4>() << 0.0, 0.0, 0.0, 1.0; // No rotation for the offset itself

      // 3. Calculate target_grasp_pose in world frame
      // This uses the part_pose_world as the "frame_pose_in_world" for the offset_pose_in_part_frame
      Eigen::VectorXd target_grasp_pose_world = 
          autoHeadChase::PoseTransformer::transformPoseToWorld(offset_pose_in_part_frame, part_pose_world);

      // 4. Transform target_grasp_pose_world to "base_link" frame
      geometry_msgs::PoseStamped target_pose_world_stamped;
      target_pose_world_stamped.header.frame_id = "odom"; // Assuming inputs are in "odom"
      target_pose_world_stamped.header.stamp = ros::Time(0); // Use latest available transform
      target_pose_world_stamped.pose.position.x = target_grasp_pose_world(0);
      target_pose_world_stamped.pose.position.y = target_grasp_pose_world(1);
      target_pose_world_stamped.pose.position.z = target_grasp_pose_world(2);
      target_pose_world_stamped.pose.orientation.x = target_grasp_pose_world(3);
      target_pose_world_stamped.pose.orientation.y = target_grasp_pose_world(4);
      target_pose_world_stamped.pose.orientation.z = target_grasp_pose_world(5);
      target_pose_world_stamped.pose.orientation.w = target_grasp_pose_world(6);

      geometry_msgs::PoseStamped target_pose_baselink_stamped;
      std::string target_tf_frame = "base_link";

      try {
        target_pose_baselink_stamped = tf_buffer_->transform(
            target_pose_world_stamped, 
            target_tf_frame, 
            ros::Duration(0.5) // Timeout for the transform
        );
      } catch (tf2::TransformException &ex) {
        ROS_ERROR("[%s] Failed to transform target pose from '%s' to '%s': %s", 
                  name().c_str(), 
                  target_pose_world_stamped.header.frame_id.c_str(), 
                  target_tf_frame.c_str(), 
                  ex.what());
        return BT::NodeStatus::FAILURE;
      }

      // 5. Determine hand side based on Y coordinate in base_link frame
      double y_in_baselink = target_pose_baselink_stamped.pose.position.y;
      int hand_side_output = 1; // Default to Right Hand (1)

      if (y_in_baselink > 0.0) {
        hand_side_output = 0; // Left Hand
        ROS_INFO("[%s] Target Y in base_link: %.2f. Selecting LEFT hand (0).", name().c_str(), y_in_baselink);
      } else {
        ROS_INFO("[%s] Target Y in base_link: %.2f. Selecting RIGHT hand (1).", name().c_str(), y_in_baselink);
      }

      setOutput("hand_side", hand_side_output);
      return BT::NodeStatus::SUCCESS;
    }
    
    void onHalted() override {
        ROS_WARN("[%s] Node Halted.", name().c_str());
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Removed getDestination and getYawFromQuaternion as primary logic changed to TF
    // and direct calculation of target pose in world, then TF to baselink.
  };
} // namespace GrabBox
