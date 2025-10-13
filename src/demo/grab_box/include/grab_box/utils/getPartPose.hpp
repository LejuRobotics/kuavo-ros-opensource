#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <mutex>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Service header (ensure this is generated after adding .srv to CMakeLists and building)
#include "kuavo_msgs/GetTargetPartPoseInCamera.h" 
// Pose transformation utility (ensure path is correct and accessible)
#include "grab_box/utils/poseTransformer.h" 

namespace GrabBox
{

class GetPartPose : public BT::StatefulActionNode
{
public:
  GetPartPose(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config), nh_(), odom_received_(false)
  {
    // Subscriber for robot's odometry
    odom_sub_ = nh_.subscribe("/odom", 10, &GetPartPose::odomCallback, this);

    // TF Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ros::param::get("/sim_mode", sim_mode_);

    // Service client to get part's pose in camera frame
    part_pose_service_client_ = nh_.serviceClient<kuavo_msgs::GetTargetPartPoseInCamera>("get_target_part_pose_in_camera");
    
    robot_pose_world_ = Eigen::VectorXd::Zero(7);
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::OutputPort<Eigen::Vector3d>("part_pos"),  // Part's position in world frame
        BT::OutputPort<Eigen::Vector4d>("part_quat") // Part's orientation (quaternion x,y,z,w) in world frame
    };
  }

  BT::NodeStatus onStart() override
  {
    odom_received_ = false; // Reset flag to ensure fresh odom is considered
    ROS_INFO("[%s] Node started. Waiting for odom and service.", name().c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // 1. Check if odometry has been received
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!odom_received_)
      {
        ROS_WARN_THROTTLE(1.0, "[%s] Waiting for odometry on /odom topic...", name().c_str());
        return BT::NodeStatus::RUNNING;
      }
    }

    // 2. Check service client and server availability
    if (!part_pose_service_client_) {
        ROS_ERROR("[%s] Part pose service client not initialized.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!part_pose_service_client_.exists()) {
        ROS_WARN_THROTTLE(1.0, "[%s] Waiting for 'get_target_part_pose_in_camera' service...", name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // 3. Call the service to get part's pose in camera frame
    kuavo_msgs::GetTargetPartPoseInCamera srv_call;
    
    if (part_pose_service_client_.call(srv_call))
    {
      if (srv_call.response.success)
      {
        ROS_INFO("[%s] Successfully received part pose in camera frame.", name().c_str());

        geometry_msgs::PoseStamped pose_stamped_camera;
        pose_stamped_camera.header.stamp = ros::Time::now(); // Or srv_call.response.header.stamp if available
        // IMPORTANT: Replace "camera_color_optical_frame" with your actual camera frame ID from your TF tree
        if (sim_mode_ == 1) {
          pose_stamped_camera.header.frame_id = "camera_base"; 
        }
        else {
          pose_stamped_camera.header.frame_id = "camera_color_optical_frame"; 
        }
        pose_stamped_camera.pose = srv_call.response.pose_in_camera;

        geometry_msgs::PoseStamped pose_stamped_odom;
        Eigen::VectorXd part_pose_world_eigen(7);

        try {
          // Transform to odom frame
          pose_stamped_odom = tf_buffer_->transform(pose_stamped_camera, "odom", ros::Duration(1.0));
          
          part_pose_world_eigen(0) = pose_stamped_odom.pose.position.x;
          part_pose_world_eigen(1) = pose_stamped_odom.pose.position.y;
          part_pose_world_eigen(2) = pose_stamped_odom.pose.position.z;
          part_pose_world_eigen(3) = pose_stamped_odom.pose.orientation.x;
          part_pose_world_eigen(4) = pose_stamped_odom.pose.orientation.y;
          part_pose_world_eigen(5) = pose_stamped_odom.pose.orientation.z;
          part_pose_world_eigen(6) = pose_stamped_odom.pose.orientation.w;

        } catch (tf2::TransformException &ex) {
          ROS_ERROR("[%s] Failed to transform pose from %s to odom: %s", 
                    name().c_str(), pose_stamped_camera.header.frame_id.c_str(), ex.what());
          return BT::NodeStatus::FAILURE;
        }
        
        // 5. Set output ports from the Eigen vector derived from TF-transformed pose_stamped_odom
        Eigen::Vector3d part_pos(part_pose_world_eigen(0), part_pose_world_eigen(1), part_pose_world_eigen(2));
        Eigen::Vector4d part_quat(part_pose_world_eigen(3), part_pose_world_eigen(4), part_pose_world_eigen(5), part_pose_world_eigen(6));

        setOutput("part_pos", part_pos);
        setOutput("part_quat", part_quat);


        ROS_INFO("[%s] Successfully calculated part pose in world (odom) frame. Pos: [%.2f, %.2f, %.2f], Quat: [%.2f, %.2f, %.2f, %.2f]",
                 name().c_str(), part_pos.x(), part_pos.y(), part_pos.z(), part_quat.x(), part_quat.y(), part_quat.z(), part_quat.w());
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        ROS_ERROR("[%s] Service 'get_target_part_pose_in_camera' failed: %s", name().c_str(), srv_call.response.message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      ROS_ERROR("[%s] Failed to call service 'get_target_part_pose_in_camera'.", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    // Should not be reached if logic is correct, but as a fallback:
    return BT::NodeStatus::RUNNING; 
  }

  void onHalted() override
  {
    ROS_WARN("[%s] Node halted.", name().c_str());
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_world_(0) = msg->pose.pose.position.x;
    robot_pose_world_(1) = msg->pose.pose.position.y;
    robot_pose_world_(2) = msg->pose.pose.position.z;
    robot_pose_world_(3) = msg->pose.pose.orientation.x;
    robot_pose_world_(4) = msg->pose.pose.orientation.y;
    robot_pose_world_(5) = msg->pose.pose.orientation.z;
    robot_pose_world_(6) = msg->pose.pose.orientation.w;
    odom_received_ = true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::ServiceClient part_pose_service_client_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Eigen::VectorXd robot_pose_world_; // Robot's current pose in world (x,y,z,qx,qy,qz,qw)
  bool odom_received_;
  std::mutex data_mutex_;
  int sim_mode_;
};

} // namespace GrabBox
