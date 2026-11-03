/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_estimation/StateEstimateBase.h"

#include <realtime_tools/realtime_buffer.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h> 
#pragma once
namespace ocs2
{
namespace humanoid
{

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics, ::ros::NodeHandle& nh);
  void set_intial_state(const vector_t& state) override
  {
    // nav_msgs::Odometry odom_msg;
    // odom_msg.pose.pose.position.x = state[6]; 
    // odom_msg.pose.pose.position.y = state[7];  
    // odom_msg.pose.pose.position.z = state[8]; 
    
    // const Eigen::Quaternion<scalar_t> q_world_base = ocs2::getQuaternionFromEulerAnglesZyx(vector3_t(state.segment<3>(9)));
    // odom_msg.pose.pose.orientation.w = q_world_base.w();
    // odom_msg.pose.pose.orientation.x = q_world_base.x();
    // odom_msg.pose.pose.orientation.y = q_world_base.y();
    // odom_msg.pose.pose.orientation.z = q_world_base.z();
    // odom_msg.twist.twist.angular.x = 0;
    // odom_msg.twist.twist.angular.y = 0;
    // odom_msg.twist.twist.angular.z = 0;
    // odom_msg.twist.twist.linear.x = 0;  
    // odom_msg.twist.twist.linear.y = 0;   
    // odom_msg.twist.twist.linear.z = 0;  

    // buffer_.writeFromNonRT(odom_msg);
    // nav_msgs::Odometry odom_msg;
    // odom_msg.pose.pose.position.x = base_pose[0]; 
    // odom_msg.pose.pose.position.y = base_pose[1];  
    // odom_msg.pose.pose.position.z = base_pose[2]; 
    
    // const Eigen::Quaternion<scalar_t> q_world_base = ocs2::getQuaternionFromEulerAnglesZyx(vector3_t(base_pose.tail<3>()));
    // odom_msg.pose.pose.orientation.w = q_world_base.w();
    // odom_msg.pose.pose.orientation.x = q_world_base.x();
    // odom_msg.pose.pose.orientation.y = q_world_base.y();
    // odom_msg.pose.pose.orientation.z = q_world_base.z();
    // odom_msg.twist.twist.angular.x = 0;
    // odom_msg.twist.twist.angular.y = 0;
    // odom_msg.twist.twist.angular.z = 0;
    // odom_msg.twist.twist.linear.x = 0;  
    // odom_msg.twist.twist.linear.y = 0;   
    // odom_msg.twist.twist.linear.z = 0;  

    // buffer_.writeFromNonRT(odom_msg);
    std::cout << "wait for FromTopicStateEstimate initialization" << std::endl;
    while (!is_initialized_)
    {
      usleep(1000);
    }
    std::cout << "FromTopicStateEstimate initialization finished" << std::endl;
  };
  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                 const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                 const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) override{}; // from ground truth topic, not used this

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);
  bool is_initialized_ = false;
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace humanoid
}  // namespace ocs2
