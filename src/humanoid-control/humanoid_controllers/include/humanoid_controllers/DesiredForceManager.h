#pragma once
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <memory>

// 前向声明
namespace ocs2 {
namespace mobile_manipulator {
class ContactForceWbc;
} // namespace mobile_manipulator
} // namespace ocs2

namespace humanoidController_wheel_wbc
{
class DesiredForceManager
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;

  DesiredForceManager(ros::NodeHandle& nh, double interpolationSpeed);
  ~DesiredForceManager() = default;

  Vector6d getDesiredForce(const std::string& ee_frame_id) const;

  bool hasDesiredForce(const std::string& ee_frame_id) const;

  void clearAllForces();

  void clearForce(const std::string& ee_frame_id);

  void update(const ros::Time& current_time);

  /**
   * @brief 设置ContactForceWbc指针，用于期望力控制
   * @param contact_force_wbc ContactForceWbc的共享指针
   */
  void setContactForceWbc(std::shared_ptr<ocs2::mobile_manipulator::ContactForceWbc> contact_force_wbc);

  /** 对应 task.info contact_force_params；更新 WBC 过渡时间 + 末端力插值速度 */
  void applyContactForceInterpParams(double transition_time, double interpolation_speed);

  /**
   * @brief 获取挥空检测失效功能是否启用
   */
  bool getEnableForceEmptyDetact() const { return enable_force_empty_detact_; }

  /**
   * @brief 设置挥空检测失效功能是否启用
   */
  void setEnableForceEmptyDetact(bool enable) { enable_force_empty_detact_ = enable; }

private:
  void desiredForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, const std::string& hand_id);
  void enableForceEmptyDetactCallback(const std_msgs::Bool::ConstPtr& msg);
  void forceDisableCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_desired_force_left_;   // 订阅 /desired_ee_force/left
  ros::Subscriber sub_desired_force_right_; // 订阅 /desired_ee_force/right
  ros::Subscriber sub_enable_force_empty_detact_;  // 订阅控制挥空检测失效功能的开关
  ros::Subscriber sub_force_disable_;  // 订阅主动失效期望力的话题
  mutable std::mutex forces_mutex_;
  std::map<std::string, geometry_msgs::WrenchStamped> desired_forces_; // key: "left_hand" 或 "right_hand"
  std::map<std::string, std::string> ee_frame_id_map_; // Maps generic names to actual frame_ids
  
  // 插值相关成员变量
  std::map<std::string, Vector6d> current_forces_;  // 当前插值中的力值
  std::map<std::string, Vector6d> target_forces_;    // 目标力值
  std::map<std::string, Vector6d> interpolation_start_forces_;  // 插值起始力值
  std::map<std::string, ros::Time> interpolation_start_times_;  // 每个分量的插值开始时间
  double interpolation_speed_;                       // 插值速度 (N/s)

  // 期望力控制相关
  std::shared_ptr<ocs2::mobile_manipulator::ContactForceWbc> contact_force_wbc_;  // ContactForceWbc指针
  bool enable_force_empty_detact_{true};  // 是否启用挥空后令期望力失效（默认启用）
};
} // namespace humanoidController_wheel_wbc

