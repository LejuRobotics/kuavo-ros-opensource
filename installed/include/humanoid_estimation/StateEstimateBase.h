/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <humanoid_common/hardware_interface/ContactSensorInterface.h>
#include <humanoid_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <humanoid_interface/common/ModelSettings.h>
#include <humanoid_interface/common/Types.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include "std_msgs/Float64MultiArray.h"
#include "humanoid_interface/common/TopicLogger.h"

namespace ocs2
{
  namespace humanoid
  {

    class StateEstimateBase
    {
    public:
      StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics &eeKinematics);
      virtual void updateJointStates(const vector_t &jointPos, const vector_t &jointVel);
      virtual void getJointStates(vector_t &jointPos, vector_t &jointVel)
      {
        jointPos = rbdState_.segment(6, info_.actuatedDofNum);
        jointVel = rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);
      }
      virtual void updateContact(contact_flag_t contactFlag)
      {
        contactFlag_ = contactFlag;
        mode_ = stanceLeg2ModeNumber(contactFlag_);
      }
      virtual void updateMode(size_t mode)
      {
        mode_ = mode;
        contactFlag_ = modeNumber2StanceLeg(mode_);
      }
      virtual void updateGait(const std::string &gait)
      {
        prev_gait_ = gait_;
        gait_ = gait;
      }
      virtual void set_intial_state(const vector_t &ocs2_state) {};
      virtual Eigen::Quaternion<scalar_t> updateIntialEulerAngles(const Eigen::Quaternion<scalar_t> &quat_init);
      virtual void updateImu(const Eigen::Quaternion<scalar_t> &quat, const vector3_t &angularVelLocal,
                             const vector3_t &linearAccelLocal, const matrix3_t &orientationCovariance,
                             const matrix3_t &angularVelCovariance, const matrix3_t &linearAccelCovariance);
      virtual void setFixFeetHeights(bool isFix) {};

      virtual Eigen::Quaternion<scalar_t> getImuOrientation()
      {
        return quat_;
      } 
      
      virtual vector_t update(const ros::Time &time, const ros::Duration &period) = 0;
      virtual nav_msgs::Odometry updateKinematics(const ros::Time &time, const Eigen::Quaterniond &imu_quat, const ros::Duration &period)
      {
        return nav_msgs::Odometry();
      }

      inline void updateFootPosDesired(const feet_array_t<vector3_t> &foot_pos_desired)
      {
        if(!update_foot_pos_desired_)
          update_foot_pos_desired_ = true;
        foot_pos_desired_ = foot_pos_desired;
        // std::cout << "[StateEstimateBase] Foot pos desired[0]: " << foot_pos_desired_[0].transpose() << std::endl;
      }

      size_t ContactDetection(const size_t nextMode_, const bool stanceMode_, const size_t plannedMode_, double robotMass, const double fzLeft, const double fzRight, double dt);
      void updateContactProbabilities(double l_Fz_filter, double r_Fz_filter, double robotMass, double dt);

      size_t getMode()
      {
        return stanceLeg2ModeNumber(contactFlag_);
      }

      feet_array_t<vector3_t> &getLatestStancePos()
      {
        return latestStanceposition_;
      }
      bool checkPullUp(double mass_threshold = 0.70, double alpha = 0.01)
      {
        if (estContactforce_.size() < 12)
          return false;
          
        double new_total_est_contact_force_ = std::max(estContactforce_[2], 0.0) + std::max(estContactforce_[8], 0.0);
        // ros_logger_->publishValue("/state_estimate/checkPullUp/new_total_est_contact_force_", new_total_est_contact_force_);
        total_est_contact_force_ = total_est_contact_force_ * (1 - alpha) + new_total_est_contact_force_ * alpha;
        ros_logger_->publishValue("/state_estimate/checkPullUp/total_est_contact_force_", total_est_contact_force_);

        // 计算当前时刻的判断结果
        bool current_result = (total_est_contact_force_ < mass_threshold * robotMass_ * 9.81);
        
        // 更新滑动窗口
        pullup_window_.push_back(current_result);
        if (pullup_window_.size() > pullup_window_size_) {
            pullup_window_.pop_front();
        }
        
        if (pullup_window_.size() < pullup_window_size_) {
            return last_pullup_state_;
        }
        
        // 检查是否所有值都相同
        bool first_value = pullup_window_.front();
        for (const bool value : pullup_window_) {
            if (value != first_value) {
                return last_pullup_state_;
            }
        }
        
        // 如果所有值都相同，更新状态并返回
        last_pullup_state_ = first_value;
        return first_value;
      }

      vector_t getBodyVelWorld()
      {
        vector_t body_vel(6);
        body_vel.head(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
        body_vel.tail(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum);
        return std::move(body_vel);
      }

      void setStartStopTime4Legs(const feet_array_t<std::array<scalar_t, 2>> &start_stop_time_4_legs)
      {
        StartStopTime4Legs_ = start_stop_time_4_legs;
      }
      void updateCmdContact(contact_flag_t cmd_contact_flag)
      {
        cmdContactflag_ = std::move(cmd_contact_flag);
      }
      void setCmdTorque(const vector_t &cmd_torque)
      {
        cmdTorque_ = cmd_torque;
      }
      void estContactForce(const ros::Duration &period);

      vector_t getEstArmContactForce(vector_t &jointPosWBC, vector_t &jointVelWBC, vector_t &cmd_torque_wbc, const ros::Duration &period);

      contact_flag_t estContactState(const scalar_t &time);
      void loadSettings(const std::string &taskFile, bool verbose);

      const vector_t &getEstContactForce()
      {
        return estContactforce_;
      }
      const vector_t &getEstDisturbanceTorque()
      {
        return estDisturbancetorque_;
      }

      const std::array<contact_flag_t, 2> &getEarlyLateContact()
      {
        return earlyLatecontact_;
      }

      vector_t getRbdState()
      {
        return rbdState_;
      }

      virtual void reset()
      {

      }

      void initializeEstArmContactForce(PinocchioInterface &pinocchioInterfaceWBC, CentroidalModelInfo &infoWBC);
      Eigen::VectorXd lowPassFilter(const Eigen::VectorXd& currentFrame, Eigen::VectorXd& previousOutput, double alpha);

    protected:
      void earlyContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time);
      void lateContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time);
      void updateAngular(const vector3_t &zyx, const vector_t &angularVel);
      void updateLinear(const vector_t &pos, const vector_t &linearVel);
      void publishMsgs(const nav_msgs::Odometry &odom);

      PinocchioInterface pinocchioInterface_;
      CentroidalModelInfo info_;
      std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;



      vector3_t zyxOffset_ = vector3_t::Zero();
      vector_t rbdState_;
      vector3_t prev_zyx_ = vector3_t::Zero();
      contact_flag_t contactFlag_{};
      Eigen::Quaternion<scalar_t> quat_;
      Eigen::Quaternion<scalar_t> yaw_offset_quat_;
      vector3_t angle_zyx_init_;
      double stance_angle_yaw_init_ = 0.0;
      vector3_t angularVelLocal_, linearAccelLocal_;
      vector3_t angularVelWorld_, linearAccelWorld_;
      vector_t jointPos_, jointVel_;
      vector_t cmdTorqueLast_;

      matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

      std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
      std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
      ros::Time lastPub_;

      std::string gait_ = "stance";
      std::string prev_gait_ = "stance";
      feet_array_t<vector3_t> latestStanceposition_;

      vector_t pSCgZinvlast_;
      vector_t vMeasuredLast_;
      vector_t estContactforce_;
      vector_t estDisturbancetorque_;
      vector_t cmdTorque_;

      std::unique_ptr<PinocchioInterface> pinocchioInterfaceWBC_;
      std::unique_ptr<CentroidalModelInfo> infoWBC_;
      vector_t rbdStateWBC_;
      vector_t cmdTorqueWBC_;
      vector_t pSCgZinvlastWBC_;
      vector_t estDisturbancetorqueWBC_;
      vector_t estArmContactforce_;

      scalar_t cutoffFrequency_ = 150;
      scalar_t detectCutoffFrequency_ = 150;
      scalar_t contactThreshold_ = 23;
      contact_flag_t cmdContactflag_{};
      feet_array_t<std::array<scalar_t, 2>> StartStopTime4Legs_;

      std::array<contact_flag_t, 2> earlyLatecontact_;
      std_msgs::Float64MultiArray earlyLateContactMsg_;
      std::deque<std::pair<scalar_t, contact_flag_t>> estConHistory_;

      // contact detection
      double preFzFilterRight_{0};
      double preFzFilterLeft_{0};
      double preDFzFilterLeft_{0};
      double preDFzFilterRight_{0};
      // size_t contact_state_ = ModeNumber::SS;
      size_t prePlannedMode_ = ModeNumber::SS;
      size_t preContactState_ = ModeNumber::SS;
      // double preContactProbability_ = 0;
      size_t mode_ = ModeNumber::SS;
      // bool contact_updata_check_{true};
      double contactHoldTime_{0};
      double leftCountHoldTime_{0};
      double rightCountHoldTime_{0};
      std::chrono::steady_clock::time_point contactEstimateTime_ = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point contactPlannedTime_ = std::chrono::steady_clock::now();
      TopicLogger *ros_logger_{nullptr};
      double max_energy_threshold_ = 10, min_energy_threshold_ = -10;
      double max_energy_threshold2_ = 20, min_energy_threshold2_ = -20;
      double holdTime_ = 0.15;
      double contactProbabilityLeft_ = 0.5;
      double contactProbabilityRight_ = 0.5;
      double preContactProbabilityLeft_ = 0.5;
      double preContactProbabilityRight_ = 0.5;
      double alpha_ = 0.1;  // 平滑因子
      feet_array_t<vector3_t> foot_pos_desired_;
      bool update_foot_pos_desired_{false};
      bool usePlannedMode_{true};
      // bool delayContact_{false};
      bool upChangeLeftContact_{true};
      bool upChangeRightContact_{true};
      bool downChangeLeftContact_{true};
      bool downChangeRightContact_{true};
      bool unknewContact_{false};
      int cantactLeft_ = 1;
      int cantactRight_ = 1;
      int plannedModeCount_ = 0;
      int estModeCount_ = 0;
      double totalValue = 0.0;
      int sumCount_ = 0;
      int leftCount_ = 0;
      int rightCount_ = 0;
      double robotMass_= 56.0; // kg
      double total_est_contact_force_ = 500;
      std::deque<bool> pullup_window_;  // 滑动窗口用于存储历史判断结果
      const size_t pullup_window_size_ = 20;  // 滑动窗口大小
      bool last_pullup_state_ = false;  // 保存上一次的pullup状态
      
    };

    template <typename T>
    T square(T a)
    {
      return a * a;
    }

    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
    {
      Eigen::Matrix<SCALAR_T, 3, 1> zyx;

      SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
      zyx(0) =
          std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
      zyx(1) = std::asin(as);
      zyx(2) =
          std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
      return zyx;
    }

  } // namespace humanoid
} // namespace ocs2
