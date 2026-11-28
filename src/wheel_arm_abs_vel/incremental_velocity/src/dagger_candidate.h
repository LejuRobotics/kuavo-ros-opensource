#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <array>
#include <functional>
#include <string>
#include "weighted_qp.h"

namespace velocity_ik {

// All callbacks and step functions run on the candidate loop's single callback queue.
class DaggerCandidate {
 public:
  using Pose = Eigen::Isometry3d;
  using ArmVector = Eigen::Matrix<double, 14, 1>;
  using WaistFK = std::function<Eigen::Vector4d(const Eigen::Vector4d&, Eigen::Matrix4d*)>;
  struct Hand {
    int failed_ticks = 0;
    bool pressed = false, requested = false, ready = false;
    uint64_t generation = 0;
    ros::Time heartbeat, bone_stamp, grip_edge;
    Pose human = Pose::Identity(), human_anchor = Pose::Identity(), robot_anchor = Pose::Identity();
    Eigen::Vector3d elbow = Eigen::Vector3d::Zero(), elbow_anchor = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_elbow = Eigen::Vector3d::Zero();
    void reset();
  };

  DaggerCandidate(ros::NodeHandle& nh, ros::NodeHandle& priv);
  void updateRequests(const ros::Time& now);
  void invalidateFeedback();
  Pose target(int arm, const Eigen::Vector3d& tcp_to_wrist) const;
  void updateWaist(const ros::Time& now, const ros::Time& sensor_stamp,
                   const Eigen::Vector4d& measured, const Eigen::Vector4d& lower,
                   const Eigen::Vector4d& upper, const WaistFK& fk);
  void publish(const ros::Time& now, const ArmVector& q, const ArmVector& dq, const ArmVector& ddq);
  static bool fresh(const ros::Time& now, const ros::Time& stamp, double timeout);
  std::array<Hand, 2> hands;
  Eigen::Vector3d translation_scale;
 private:
  void grip(int index, bool pressed);
  void bones(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  void config(const std_msgs::String::ConstPtr& msg);
  void body(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void resetWaist();

  struct WaistConfig {
    std::string session, side;
    Eigen::Vector4d lower, upper;
    double max_speed = 0., action_timeout = .2, signal_timeout = .2;
  } config_;
  Pose chest_ = Pose::Identity(), chest_anchor_ = Pose::Identity();
  ros::Time chest_stamp_, waist_stamp_, last_step_, waist_velocity_until_;
  Eigen::Vector4d waist_velocity_ = Eigen::Vector4d::Zero();
  int64_t bone_sequence_ = -1;
  ros::Time last_bones_;
  bool waist_requested_ = false, waist_ready_ = false, incremental_orientation_ = true;
  Eigen::Vector4d waist_position_ = Eigen::Vector4d::Zero(), anchor_feature_ = Eigen::Vector4d::Zero();
  uint64_t waist_generation_ = 0, sequence_ = 0;
  std::string session_;
  ros::Publisher publisher_, local_sample_pub_;
  ros::Subscriber bones_sub_, body_sub_, config_sub_;
  std::array<ros::Subscriber, 2> grip_sub_;
  WeightedQp waist_solver_;
};
}  // namespace velocity_ik
