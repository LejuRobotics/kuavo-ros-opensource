#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <ocs2_core/Types.h>  // 包含vector_t, scalar_t等类型定义
#include <humanoid_interface/common/Types.h>  // 包含vector3_t, matrix3_t等类型定义（在ocs2::humanoid命名空间中）

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;  // 需要humanoid命名空间以使用vector3_t, matrix3_t

  /**
   * @brief 传感器数据结构
   * 用于存储机器人的传感器数据
   */
  struct SensorData
  {
    ros::Time timeStamp_;
    vector_t jointPos_;
    vector_t jointVel_;
    vector_t jointAcc_;
    vector_t jointTorque_;
    vector_t jointCurrent_;
    vector3_t angularVel_;
    vector3_t linearAccel_;
    vector3_t freeLinearAccel_;
    Eigen::Quaternion<scalar_t> quat_;
    matrix3_t orientationCovariance_;
    matrix3_t angularVelCovariance_;
    matrix3_t linearAccelCovariance_;
    Eigen::Quaternion<scalar_t> quat_offset_;
    
    // 默认构造函数
    SensorData() = default;
    
    // 拷贝构造函数 - 深拷贝
    SensorData(const SensorData& other)
      : timeStamp_(other.timeStamp_),
        jointPos_(other.jointPos_),
        jointVel_(other.jointVel_),
        jointAcc_(other.jointAcc_),
        jointTorque_(other.jointTorque_),
        jointCurrent_(other.jointCurrent_),
        angularVel_(other.angularVel_),
        linearAccel_(other.linearAccel_),
        freeLinearAccel_(other.freeLinearAccel_),
        quat_(other.quat_),
        orientationCovariance_(other.orientationCovariance_),
        angularVelCovariance_(other.angularVelCovariance_),
        linearAccelCovariance_(other.linearAccelCovariance_),
        quat_offset_(other.quat_offset_)
    {
    }
    
    // 深拷贝函数
    SensorData copy() const
    {
      SensorData result;
      result.timeStamp_ = timeStamp_;
      result.jointPos_ = jointPos_;
      result.jointVel_ = jointVel_;
      result.jointAcc_ = jointAcc_;
      result.jointTorque_ = jointTorque_;
      result.jointCurrent_ = jointCurrent_;
      result.angularVel_ = angularVel_;
      result.linearAccel_ = linearAccel_;
      result.freeLinearAccel_ = freeLinearAccel_;
      result.quat_ = quat_;
      result.orientationCovariance_ = orientationCovariance_;
      result.angularVelCovariance_ = angularVelCovariance_;
      result.linearAccelCovariance_ = linearAccelCovariance_;
      result.quat_offset_ = quat_offset_;
      return result;
    }
    
    void resize_joint(size_t num)
    {
      jointPos_.resize(num);
      jointVel_.resize(num);
      jointAcc_.resize(num);
      jointTorque_.resize(num);
      jointCurrent_.resize(num);
    }
  };

} // namespace humanoid_controller

