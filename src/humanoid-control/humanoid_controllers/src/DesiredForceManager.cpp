#include "humanoid_controllers/DesiredForceManager.h"
#include "humanoid_wheel_wbc/ContactForceWbc.h"
#include <ocs2_core/misc/LoadData.h>
#include <ros/ros.h>
#include <boost/bind.hpp>

namespace humanoidController_wheel_wbc
{

DesiredForceManager::DesiredForceManager(ros::NodeHandle& nh, double interpolationSpeed)
  : nh_(nh), interpolation_speed_(interpolationSpeed)
{
  // 初始化frame_id映射
  ee_frame_id_map_["left_hand"] = "zarm_l7_end_effector";
  ee_frame_id_map_["right_hand"] = "zarm_r7_end_effector";
  std::string taskFile;
  if (nh_.getParam("/taskFile", taskFile)) {
    std::vector<std::string> eeFrames;
    ocs2::loadData::loadStdVector<std::string>(taskFile, "model_settings.eeFrame", eeFrames, false);
    if (eeFrames.size() >= 2) {
      ee_frame_id_map_["left_hand"] = eeFrames[0];
      ee_frame_id_map_["right_hand"] = eeFrames[1];
    }
  }

  // 订阅两个独立的话题，分别对应左右手
  sub_desired_force_left_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
      "/desired_ee_force/left", 10, 
      boost::bind(&DesiredForceManager::desiredForceCallback, this, _1, "left_hand"));
  sub_desired_force_right_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
      "/desired_ee_force/right", 10, 
      boost::bind(&DesiredForceManager::desiredForceCallback, this, _1, "right_hand"));
  
  // 初始化当前力和目标力为零向量
  current_forces_["left_hand"] = Vector6d::Zero();
  current_forces_["right_hand"] = Vector6d::Zero();
  target_forces_["left_hand"] = Vector6d::Zero();
  target_forces_["right_hand"] = Vector6d::Zero();
  interpolation_start_forces_["left_hand"] = Vector6d::Zero();
  interpolation_start_forces_["right_hand"] = Vector6d::Zero();
  interpolation_start_times_["left_hand"] = ros::Time::now();
  interpolation_start_times_["right_hand"] = ros::Time::now();

  // 初始化期望力控制话题订阅者
  sub_enable_force_empty_detact_ = nh_.subscribe<std_msgs::Bool>(
      "/enable_force_empty_detact", 10,
      &DesiredForceManager::enableForceEmptyDetactCallback, this);
  sub_force_disable_ = nh_.subscribe<std_msgs::Bool>(
      "/force_disable", 10,
      &DesiredForceManager::forceDisableCallback, this);

  ROS_INFO("[DesiredForceManager] Initialized. Subscribing to /desired_ee_force/left and /desired_ee_force/right");
  ROS_INFO("[DesiredForceManager] Subscribed to /enable_force_empty_detact and /force_disable");
  ROS_INFO("[DesiredForceManager] Interpolation speed: %.2f N/s", interpolation_speed_);
}

void DesiredForceManager::desiredForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& hand_id)
{
  std::lock_guard<std::mutex> lock(forces_mutex_);
  
  geometry_msgs::WrenchStamped wrench_msg = *msg;
  wrench_msg.header.frame_id = hand_id;  // 统一使用 hand_id 作为标识
  
  desired_forces_[hand_id] = wrench_msg;
  
  // 更新目标力值（用于插值）
  Vector6d target_force;
  target_force << wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z,
                  wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z;
  
  // 如果目标力改变，更新插值起始点和时间
  bool force_changed = (target_forces_.find(hand_id) == target_forces_.end()) ||
                       ((target_force - target_forces_[hand_id]).cwiseAbs().maxCoeff() > 1e-6);
  if (force_changed) {
    interpolation_start_forces_[hand_id] = current_forces_[hand_id];
    interpolation_start_times_[hand_id] = ros::Time::now();
  }
  target_forces_[hand_id] = target_force;
  
  ROS_DEBUG("[DesiredForceManager] Updated target force for %s from topic /desired_ee_force/%s", 
            hand_id.c_str(), (hand_id == "left_hand" ? "left" : "right"));
}

DesiredForceManager::Vector6d DesiredForceManager::getDesiredForce(const std::string& ee_frame_id) const
{
  std::lock_guard<std::mutex> lock(forces_mutex_);

  // 先尝试直接使用frame_id获取当前插值中的力值
  if (current_forces_.find(ee_frame_id) != current_forces_.end()) {
    return current_forces_.at(ee_frame_id);
  }

  // 如果都没有找到，返回零向量
  return Vector6d::Zero();
}

bool DesiredForceManager::hasDesiredForce(const std::string& ee_frame_id) const
{
  std::lock_guard<std::mutex> lock(forces_mutex_);
  
  if (desired_forces_.find(ee_frame_id) != desired_forces_.end()) {
    return true;
  }
  
  if (ee_frame_id_map_.find(ee_frame_id) != ee_frame_id_map_.end()) {
    const std::string& mapped_id = ee_frame_id_map_.at(ee_frame_id);
    return desired_forces_.find(mapped_id) != desired_forces_.end();
  }
  
  return false;
}

void DesiredForceManager::clearAllForces()
{
  std::lock_guard<std::mutex> lock(forces_mutex_);
  desired_forces_.clear();
  target_forces_["left_hand"] = Vector6d::Zero();
  target_forces_["right_hand"] = Vector6d::Zero();
  current_forces_["left_hand"] = Vector6d::Zero();
  current_forces_["right_hand"] = Vector6d::Zero();
  interpolation_start_forces_["left_hand"] = Vector6d::Zero();
  interpolation_start_forces_["right_hand"] = Vector6d::Zero();
  // interpolation_start_times_["left_hand"] = now;
  // interpolation_start_times_["right_hand"] = now;
  ROS_INFO("[DesiredForceManager] Cleared all desired forces");
}

void DesiredForceManager::clearForce(const std::string& ee_frame_id)
{
  std::lock_guard<std::mutex> lock(forces_mutex_);
  desired_forces_.erase(ee_frame_id);
  target_forces_[ee_frame_id] = Vector6d::Zero();
  current_forces_[ee_frame_id] = Vector6d::Zero();
  interpolation_start_forces_[ee_frame_id] = Vector6d::Zero();
  // interpolation_start_times_[ee_frame_id] = ros::Time::now();
  ROS_INFO("[DesiredForceManager] Cleared desired force for %s", ee_frame_id.c_str());
}

void DesiredForceManager::update(const ros::Time& current_time)
{
  std::lock_guard<std::mutex> lock(forces_mutex_);
  
  for (auto& hand_pair : target_forces_) {
    const std::string& hand_id = hand_pair.first;
    const Vector6d& target = hand_pair.second;
    
    // 初始化（如果需要）
    if (current_forces_.find(hand_id) == current_forces_.end()) {
      current_forces_[hand_id] = Vector6d::Zero();
      interpolation_start_forces_[hand_id] = Vector6d::Zero();
      interpolation_start_times_[hand_id] = current_time;
    }
    
    Vector6d& current = current_forces_[hand_id];
    Vector6d& start_force = interpolation_start_forces_[hand_id];
    ros::Time& start_time = interpolation_start_times_[hand_id];
    
    // 计算距离
    double max_distance = (target - start_force).cwiseAbs().maxCoeff();
    if (max_distance < 1e-6) {
      current = target;
      continue;
    }
    
    // 三次多项式插值：s = 3τ² - 2τ³
    double elapsed_time = std::max(0.0, (current_time - start_time).toSec());
    double tau = std::min(1.0, elapsed_time * interpolation_speed_ / max_distance);
    double s = tau * tau * (3.0 - 2.0 * tau);
    
    // 插值更新
    current = start_force + s * (target - start_force);
    
    // 到达目标后更新起始点
    if (tau >= 1.0) {
      start_force = target;
      start_time = current_time;
    }
  }
}

void DesiredForceManager::setContactForceWbc(std::shared_ptr<ocs2::mobile_manipulator::ContactForceWbc> contact_force_wbc)
{
  contact_force_wbc_ = contact_force_wbc;
  ROS_INFO("[DesiredForceManager] ContactForceWbc pointer set");
}

void DesiredForceManager::applyContactForceInterpParams(double transition_time, double interpolation_speed)
{
  if (transition_time <= 0.0 || interpolation_speed <= 0.0) return;
  if (contact_force_wbc_) {
    contact_force_wbc_->setTransitionTime(transition_time);   //设置力任务开启后的手臂权重过渡时间
    contact_force_wbc_->setInterpolationSpeed(interpolation_speed);  //设置期望力插值速度
  }
  std::lock_guard<std::mutex> lock(forces_mutex_);
  interpolation_speed_ = interpolation_speed;
}

void DesiredForceManager::enableForceEmptyDetactCallback(const std_msgs::Bool::ConstPtr& msg)
{
  enable_force_empty_detact_ = msg->data;
  ROS_INFO("[DesiredForceManager] Swing empty force disable: %s", enable_force_empty_detact_ ? "enabled" : "disabled");
}

void DesiredForceManager::forceDisableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data) {
    // 主动失效期望力
    if (contact_force_wbc_) {
      contact_force_wbc_->disableForce();
    }
    clearAllForces();
    ROS_INFO("[DesiredForceManager] Force disabled manually");
  }
}

} // namespace humanoidController_wheel_wbc

