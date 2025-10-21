/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_estimation/LinearKalmanFilter.h"

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "humanoid_interface/common/TopicLogger.h"

namespace ocs2
{
namespace humanoid
{
#define num_contact_points_est  8
#define num_q_est (6 + num_contact_points_est*3)
#define num_r_est (num_contact_points_est*3*2 + num_contact_points_est)
#define num_contact_dof  (num_contact_points_est*3)

class KalmanFilterEstimate : public StateEstimateBase
{
public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                       const PinocchioEndEffectorKinematics& eeKinematics);
  void set_intial_state(const vector_t& state) override;
  
  vector_t update(const ros::Time& time, const ros::Duration& period) override;
  void setFixFeetHeights(bool isFix) 
  {
    isFixHeight_ = isFix;
  }
  nav_msgs::Odometry updateKinematics(const ros::Time &time, const Eigen::Quaterniond &imu_quat, const ros::Duration &period) override;

  void loadSettings(const std::string& taskFile, bool verbose);
  void reset() override;
protected:
  void updateFromTopic();

  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t imuProcessNoiseZPosition_ = 0.02;
  scalar_t imuProcessNoiseZVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;
  bool resetGroundHeight_ = true;

private:
  TopicLogger *ros_logger_{nullptr};

  Eigen::Matrix<scalar_t, num_q_est, 1> xHat_;// base_pos,base_vel,contact_pos(6,num_contact_points_est*3)
  Eigen::Matrix<scalar_t, num_contact_points_est*3, 1> ps_;
  Eigen::Matrix<scalar_t, num_contact_points_est*3, 1> vs_;
  Eigen::Matrix<scalar_t, num_q_est, num_q_est> a_;
  Eigen::Matrix<scalar_t, num_q_est, num_q_est> q_; // 测量协方差,
  Eigen::Matrix<scalar_t, num_q_est, num_q_est> p_;
  Eigen::Matrix<scalar_t, num_r_est, num_r_est> r_; //预测协方差 point_pos, point_vel, contact_height
  Eigen::Matrix<scalar_t, num_q_est, 3> b_;
  Eigen::Matrix<scalar_t, num_r_est, num_q_est> c_;
  Eigen::Vector3d base_pos_;
  Eigen::Vector3d base_ang_;
  Eigen::Quaterniond base_quat_;
  Eigen::Vector3d last_base_pos_;
  Eigen::Vector3d last_base_ang_;
  Eigen::Vector3d last_foot_pos_;
  Eigen::Vector3d last_foot_ang_;
  Eigen::Quaterniond last_foot_quat_;
  int last_contact_point_index_;
  // Topic
  ros::Subscriber sub_;
  ros::Subscriber footContactPointSubscriber_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
  bool isFixHeight_;
};

}  // namespace humanoid
}  // namespace ocs2
