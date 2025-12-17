#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS消息类型
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>

namespace leju_utils {
namespace ros_msg_convertor {

// ============== Point 转换函数 ==============

/**
 * @brief 将Eigen::Vector3d转换为geometry_msgs::Point
 * @param vector Eigen 3D向量
 * @return geometry_msgs::Point ROS Point消息
 */
inline geometry_msgs::Point eigenToPoint(const Eigen::Vector3d& vector) {
  geometry_msgs::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

/**
 * @brief 将geometry_msgs::Point转换为Eigen::Vector3d
 * @param point ROS Point消息
 * @return Eigen::Vector3d Eigen 3D向量
 */
inline Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

// ============== Vector3 转换函数 ==============

/**
 * @brief 将Eigen::Vector3d转换为geometry_msgs::Vector3
 * @param vector Eigen 3D向量
 * @return geometry_msgs::Vector3 ROS Vector3消息
 */
inline geometry_msgs::Vector3 eigenToVector3(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 vec3;
  vec3.x = vector.x();
  vec3.y = vector.y();
  vec3.z = vector.z();
  return vec3;
}

/**
 * @brief 将geometry_msgs::Vector3转换为Eigen::Vector3d
 * @param vector3 ROS Vector3消息
 * @return Eigen::Vector3d Eigen 3D向量
 */
inline Eigen::Vector3d vector3ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

// ============== Quaternion 转换函数 ==============

/**
 * @brief 将Eigen::Quaterniond转换为geometry_msgs::Quaternion
 * @param quat Eigen四元数
 * @return geometry_msgs::Quaternion ROS四元数消息
 */
inline geometry_msgs::Quaternion eigenToQuaternion(const Eigen::Quaterniond& quat) {
  geometry_msgs::Quaternion quatMsg;
  quatMsg.x = quat.x();
  quatMsg.y = quat.y();
  quatMsg.z = quat.z();
  quatMsg.w = quat.w();
  return quatMsg;
}

/**
 * @brief 将geometry_msgs::Quaternion转换为Eigen::Quaterniond
 * @param quatMsg ROS四元数消息
 * @return Eigen::Quaterniond Eigen四元数
 */
inline Eigen::Quaterniond quaternionToEigen(const geometry_msgs::Quaternion& quatMsg) {
  return Eigen::Quaterniond(quatMsg.w, quatMsg.x, quatMsg.y, quatMsg.z);
}

// ============== Pose 转换函数 ==============

/**
 * @brief 将Eigen位姿转换为geometry_msgs::Pose
 * @param position Eigen位置向量
 * @param orientation Eigen四元数
 * @return geometry_msgs::Pose ROS位姿消息
 */
inline geometry_msgs::Pose eigenToPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  geometry_msgs::Pose pose;
  pose.position = eigenToPoint(position);
  pose.orientation = eigenToQuaternion(orientation);
  return pose;
}

/**
 * @brief 将geometry_msgs::Pose转换为Eigen位姿
 * @param pose ROS位姿消息
 * @return std::pair<Eigen::Vector3d, Eigen::Quaterniond> Eigen位置和四元数对
 */
inline std::pair<Eigen::Vector3d, Eigen::Quaterniond> poseToEigen(const geometry_msgs::Pose& pose) {
  Eigen::Vector3d position = pointToEigen(pose.position);
  Eigen::Quaterniond orientation = quaternionToEigen(pose.orientation);
  return std::make_pair(position, orientation);
}

// ============== Transform 转换函数 ==============

/**
 * @brief 将Eigen变换转换为geometry_msgs::Transform
 * @param translation Eigen平移向量
 * @param rotation Eigen四元数
 * @return geometry_msgs::Transform ROS变换消息
 */
inline geometry_msgs::Transform eigenToTransform(const Eigen::Vector3d& translation,
                                                 const Eigen::Quaterniond& rotation) {
  geometry_msgs::Transform transform;
  transform.translation = eigenToVector3(translation);
  transform.rotation = eigenToQuaternion(rotation);
  return transform;
}

/**
 * @brief 将geometry_msgs::Transform转换为Eigen变换
 * @param transform ROS变换消息
 * @return std::pair<Eigen::Vector3d, Eigen::Quaterniond> Eigen平移和旋转对
 */
inline std::pair<Eigen::Vector3d, Eigen::Quaterniond> transformToEigen(const geometry_msgs::Transform& transform) {
  Eigen::Vector3d translation = vector3ToEigen(transform.translation);
  Eigen::Quaterniond rotation = quaternionToEigen(transform.rotation);
  return std::make_pair(translation, rotation);
}

// ============== Twist 转换函数 ==============

/**
 * @brief 将Eigen速度转换为geometry_msgs::Twist
 * @param linear Eigen线速度向量
 * @param angular Eigen角速度向量
 * @return geometry_msgs::Twist ROS速度消息
 */
inline geometry_msgs::Twist eigenToTwist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
  geometry_msgs::Twist twist;
  twist.linear = eigenToVector3(linear);
  twist.angular = eigenToVector3(angular);
  return twist;
}

/**
 * @brief 将geometry_msgs::Twist转换为Eigen速度
 * @param twist ROS速度消息
 * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> Eigen线速度和角速度对
 */
inline std::pair<Eigen::Vector3d, Eigen::Vector3d> twistToEigen(const geometry_msgs::Twist& twist) {
  Eigen::Vector3d linear = vector3ToEigen(twist.linear);
  Eigen::Vector3d angular = vector3ToEigen(twist.angular);
  return std::make_pair(linear, angular);
}

// ============== Wrench 转换函数 ==============

/**
 * @brief 将Eigen力/力矩转换为geometry_msgs::Wrench
 * @param force Eigen力向量
 * @param torque Eigen力矩向量
 * @return geometry_msgs::Wrench ROS力/力矩消息
 */
inline geometry_msgs::Wrench eigenToWrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) {
  geometry_msgs::Wrench wrench;
  wrench.force = eigenToVector3(force);
  wrench.torque = eigenToVector3(torque);
  return wrench;
}

/**
 * @brief 将geometry_msgs::Wrench转换为Eigen力/力矩
 * @param wrench ROS力/力矩消息
 * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> Eigen力和力矩对
 */
inline std::pair<Eigen::Vector3d, Eigen::Vector3d> wrenchToEigen(const geometry_msgs::Wrench& wrench) {
  Eigen::Vector3d force = vector3ToEigen(wrench.force);
  Eigen::Vector3d torque = vector3ToEigen(wrench.torque);
  return std::make_pair(force, torque);
}

// ============== 带时间戳的消息转换函数 ==============

/**
 * @brief 将Eigen位姿转换为geometry_msgs::PoseStamped
 * @param position Eigen位置向量
 * @param orientation Eigen四元数
 * @param frame_id 坐标系ID
 * @param stamp 时间戳
 * @return geometry_msgs::PoseStamped ROS带时间戳位姿消息
 */
inline geometry_msgs::PoseStamped eigenToPoseStamped(const Eigen::Vector3d& position,
                                                     const Eigen::Quaterniond& orientation,
                                                     const std::string& frame_id = "base_link",
                                                     const ros::Time& stamp = ros::Time::now()) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = frame_id;
  poseStamped.header.stamp = stamp;
  poseStamped.pose = eigenToPose(position, orientation);
  return poseStamped;
}

/**
 * @brief 将Eigen变换转换为geometry_msgs::TransformStamped
 * @param translation Eigen平移向量
 * @param rotation Eigen四元数
 * @param parent_frame_id 父坐标系ID
 * @param child_frame_id 子坐标系ID
 * @param stamp 时间戳
 * @return geometry_msgs::TransformStamped ROS带时间戳变换消息
 */
inline geometry_msgs::TransformStamped eigenToTransformStamped(const Eigen::Vector3d& translation,
                                                               const Eigen::Quaterniond& rotation,
                                                               const std::string& parent_frame_id = "base_link",
                                                               const std::string& child_frame_id = "end_effector",
                                                               const ros::Time& stamp = ros::Time::now()) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.frame_id = parent_frame_id;
  transformStamped.header.stamp = stamp;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform = eigenToTransform(translation, rotation);
  return transformStamped;
}

// ============== JointState 转换函数 ==============

/**
 * @brief 将Eigen关节角度转换为sensor_msgs::JointState
 * @param joint_positions Eigen关节位置向量
 * @param joint_names 关节名称列表
 * @param joint_velocities Eigen关节速度向量（可选）
 * @param joint_efforts Eigen关节力矩向量（可选）
 * @param stamp 时间戳
 * @return sensor_msgs::JointState ROS关节状态消息
 */
inline sensor_msgs::JointState eigenToJointState(const Eigen::VectorXd& joint_positions,
                                                 const std::vector<std::string>& joint_names,
                                                 const Eigen::VectorXd& joint_velocities = Eigen::VectorXd(),
                                                 const Eigen::VectorXd& joint_efforts = Eigen::VectorXd(),
                                                 const ros::Time& stamp = ros::Time::now()) {
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name = joint_names;

  // 转换位置
  joint_state.position.resize(joint_positions.size());
  for (int i = 0; i < joint_positions.size(); ++i) {
    joint_state.position[i] = joint_positions(i);
  }

  // 转换速度（如果提供）
  if (joint_velocities.size() > 0) {
    joint_state.velocity.resize(joint_velocities.size());
    for (int i = 0; i < joint_velocities.size(); ++i) {
      joint_state.velocity[i] = joint_velocities(i);
    }
  }

  // 转换力矩（如果提供）
  if (joint_efforts.size() > 0) {
    joint_state.effort.resize(joint_efforts.size());
    for (int i = 0; i < joint_efforts.size(); ++i) {
      joint_state.effort[i] = joint_efforts(i);
    }
  }

  return joint_state;
}

/**
 * @brief 将sensor_msgs::JointState转换为Eigen向量
 * @param joint_state ROS关节状态消息
 * @return std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 位置、速度、力矩向量元组
 */
inline std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> jointStateToEigen(
    const sensor_msgs::JointState& joint_state) {
  // 转换位置
  Eigen::VectorXd positions(joint_state.position.size());
  for (size_t i = 0; i < joint_state.position.size(); ++i) {
    positions(i) = joint_state.position[i];
  }

  // 转换速度
  Eigen::VectorXd velocities(joint_state.velocity.size());
  for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
    velocities(i) = joint_state.velocity[i];
  }

  // 转换力矩
  Eigen::VectorXd efforts(joint_state.effort.size());
  for (size_t i = 0; i < joint_state.effort.size(); ++i) {
    efforts(i) = joint_state.effort[i];
  }

  return std::make_tuple(positions, velocities, efforts);
}

// ============== 批量转换函数 ==============

/**
 * @brief 将Eigen::Vector3d向量批量转换为geometry_msgs::Point向量
 * @param eigen_points Eigen 3D向量列表
 * @return std::vector<geometry_msgs::Point> ROS Point消息列表
 */
inline std::vector<geometry_msgs::Point> eigenPointsToRos(const std::vector<Eigen::Vector3d>& eigen_points) {
  std::vector<geometry_msgs::Point> ros_points;
  ros_points.reserve(eigen_points.size());
  for (const auto& point : eigen_points) {
    ros_points.push_back(eigenToPoint(point));
  }
  return ros_points;
}

/**
 * @brief 将geometry_msgs::Point向量批量转换为Eigen::Vector3d向量
 * @param ros_points ROS Point消息列表
 * @return std::vector<Eigen::Vector3d> Eigen 3D向量列表
 */
inline std::vector<Eigen::Vector3d> rosPointsToEigen(const std::vector<geometry_msgs::Point>& ros_points) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.reserve(ros_points.size());
  for (const auto& point : ros_points) {
    eigen_points.push_back(pointToEigen(point));
  }
  return eigen_points;
}

// ============== 验证函数 ==============

/**
 * @brief 验证geometry_msgs::Point是否有效
 * @param point ROS Point消息
 * @param context 上下文信息（用于错误输出）
 * @return bool 是否有效
 */
inline bool validatePoint(const geometry_msgs::Point& point, const std::string& context = "") {
  bool isValid = std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  if (!isValid && !context.empty()) {
    std::cerr << "⚠️ [" << context << "] Invalid Point: (" << point.x << ", " << point.y << ", " << point.z << ")"
              << std::endl;
  }
  return isValid;
}

/**
 * @brief 验证geometry_msgs::Quaternion是否有效
 * @param quat ROS四元数消息
 * @param context 上下文信息（用于错误输出）
 * @return bool 是否有效
 */
inline bool validateQuaternion(const geometry_msgs::Quaternion& quat, const std::string& context = "") {
  double norm = sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
  bool isValid = std::isfinite(norm) && norm > 1e-6;
  if (!isValid && !context.empty()) {
    std::cerr << "⚠️ [" << context << "] Invalid Quaternion norm: " << norm << std::endl;
  }
  return isValid;
}

}  // namespace ros_msg_convertor
}  // namespace leju_utils
