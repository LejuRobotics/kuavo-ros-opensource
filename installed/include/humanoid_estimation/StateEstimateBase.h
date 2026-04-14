/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

// Use OCS2's PinocchioInterface aliases to avoid pinocchio::Model/Data compatibility issues across Pinocchio versions.
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

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
#include "humanoid_estimation/ArmContactForceKalmanFilter.h"

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
      
      // 获取足端位置接口
      virtual vector_t getEndEffectorPositions() const { std::cout << "[StateEstimateBase] getEndEffectorPositions not implemented" << std::endl; return vector_t::Zero(0); }
      
      // 计算双脚支撑中心点
      virtual vector3_t getFeetCenterPosition() const { std::cout << "[StateEstimateBase] getFeetCenterPosition not implemented" << std::endl; return vector3_t::Zero(); }
      
      // 获取躯干状态接口 (位置、姿态、线速度、角速度)
      virtual vector_t getTorsoState() const
      {
        vector_t torsoState(12);
        
        // rbdState_ 结构: [x, y, z, yaw, pitch, roll, vx, vy, vz, angularVx, angularVy, angularVz]
        // 前6个元素: 位置和姿态
        torsoState.segment<3>(0) = rbdState_.segment<3>(3);  // x, y, z
        torsoState.segment<3>(3) = rbdState_.head<3>();      // yaw, pitch, roll
        
        // 后6个元素: 线速度和角速度
        torsoState.segment<3>(6) = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);  // vx, vy, vz
        torsoState.segment<3>(9) = rbdState_.segment<3>(info_.generalizedCoordinatesNum);      // angularVx, angularVy, angularVz
        
        return torsoState;
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
      virtual bool updateKinematicsRL(const ros::Time &time, const ros::Duration &period)
      {
        return false;
      }

      inline void updateFootPosDesired(const feet_array_t<vector3_t> &foot_pos_desired)
      {
        if(!update_foot_pos_desired_)
          update_foot_pos_desired_ = true;
        foot_pos_desired_ = foot_pos_desired;
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
        
        // 检查时间间隔，如果相邻两次调用时间>0.5s，则重置滤波器
        ros::Time current_time = ros::Time::now();
        if (last_pullup_check_time_.isValid()) {
          ros::Duration time_diff = current_time - last_pullup_check_time_;
          if (time_diff.toSec() > 0.5) {
            total_est_contact_force_ = robotMass_ * 9.81;
            last_pullup_state_ = false;
            pullup_window_.clear();
            std::cout << "[StateEstimateBase] checkPullUp reset filter" << std::endl;
          }
        }
        last_pullup_check_time_ = current_time;
          
        double new_total_est_contact_force_ = std::max(estContactforce_[2], 0.0) + std::max(estContactforce_[8], 0.0);
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

      void resetPullUpFilter()
      {
        total_est_contact_force_ = robotMass_ * 9.81;
        last_pullup_state_ = false;
        pullup_window_.clear();
        last_pullup_check_time_ = ros::Time(); // 重置时间戳，让下次调用时重新初始化
        std::cout << "[StateEstimateBase] resetPullUpFilter called" << std::endl;
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

      /**
       * 设置“手臂接触力估计”使用的完整模型（real/WBC）。
       * 用途：主估计器模型为简化维度时（如手臂DOF降维），末端frame/动力学不匹配，需要用完整模型计算手臂末端力。
       */
      void setFullArmForceModel(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info)
      {
        armForcePinocchioInterfacePtr_.reset(new PinocchioInterface(pinocchioInterface));
        armForceInfoPtr_.reset(new CentroidalModelInfo(info));
        use_full_arm_model_ = true;
      }

      /** 清空“手臂接触力估计”的完整模型配置（切回非简化模型/切换URDF等场景可用）。 */
      void clearArmForceModel()
      {
        armForcePinocchioInterfacePtr_.reset();
        armForceInfoPtr_.reset();
        armForceRbdState_.resize(0);
        armForceCmdTorque_.resize(0);
        use_full_arm_model_ = false;
      }

      /**
       * 更新“手臂接触力估计”的输入（完整维度）。
       * 维度要求：rbdStateReal.size = 2 * info_real.generalizedCoordinatesNum，cmdTorqueReal.size = info_real.actuatedDofNum。
       */
      void setArmForceInputs(const vector_t& rbdStateReal, const vector_t& cmdTorqueReal)
      {
        armForceRbdState_ = rbdStateReal;
        armForceCmdTorque_ = cmdTorqueReal;
      }
      
      void estContactForce(const ros::Duration &period);
      void estArmContactForce(const ros::Duration &period);
      bool checkArmLoad(double force_threshold = 30.0);

      contact_flag_t estContactState(const scalar_t &time);
      void loadSettings(const std::string &taskFile, bool verbose, const std::string &referenceFile);

      const vector_t &getEstContactForce()
      {
        return estContactforce_;
      }
      const vector_t &getEstDisturbanceTorque()
      {
        return estDisturbancetorque_;
      }
      const vector_t &getEstArmContactForce()
      {
        return estArmContactforce_;
      }

      const std::array<contact_flag_t, 2> &getEarlyLateContact()
      {
        return earlyLatecontact_;
      }

      vector_t getRbdState()
      {
        return rbdState_;
      }

      /**
       * @brief 更新躯干速度稳定性状态（应在控制循环中持续调用）
       * @param time 当前时间
       * @param period 控制周期
       */
      void updateTorsoStability(const ros::Time &time, const ros::Duration &period);

      /**
       * @brief 检查躯干速度是否稳定
       * @return 如果速度稳定返回true，否则返回false
       */
      bool isTorsoVelocityStable() const
      {
        return is_torso_velocity_stable_;
      }

      /**
       * @brief 设置躯干稳定性检测参数
       * @param threshold 速度阈值（m/s）
       * @param duration 需要稳定的持续时间（秒）
       */
      void setTorsoStabilityParams(double threshold, double duration)
      {
        torsoVelocityThreshold_ = threshold;
        torsoVelocityDuration_ = duration;
      }

      virtual void reset()
      {

      }

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

      // 主模型简化时启用，用于手臂接触力估计的完整模型
      bool use_full_arm_model_{false};
      std::unique_ptr<PinocchioInterface> armForcePinocchioInterfacePtr_;
      std::unique_ptr<CentroidalModelInfo> armForceInfoPtr_;
      vector_t armForceRbdState_;
      vector_t armForceCmdTorque_;

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
      vector_t pSCgArmZinvlast_;  // 手臂接触力估计的低通滤波状态
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
      size_t prePlannedMode_ = ModeNumber::SS;
      size_t preContactState_ = ModeNumber::SS;
      size_t mode_ = ModeNumber::SS;
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
      ros::Time last_pullup_check_time_;
      int waistNum_ = 0;
      
      // 躯干速度稳定性检测
      double torsoVelocityThreshold_ = 0.05;  // 速度阈值（m/s，默认0.05）
      double torsoVelocityDuration_ = 1.0;    // 需要稳定的持续时间（秒，默认1.0）
      ros::Time torso_velocity_stable_start_time_;  // 速度开始稳定的时间戳
      bool torso_velocity_stable_tracking_ = false;  // 是否正在跟踪稳定状态
      bool is_torso_velocity_stable_ = false;       // 当前是否稳定
      
      vector_t estArmContactforceLast_;
      std::deque<double> armForceWindow_;
      size_t armForceWindowSize_ = 50;
      size_t armDofPerSide_ = 7;
      
      // 卡尔曼滤波器（手臂接触力估计）
      bool useArmForceKalmanFilter_ = true;  // 是否使用卡尔曼滤波（默认开启，基于奇异性自适应）
      std::unique_ptr<ArmContactForceKalmanFilter> armForceKF_;
      
      // 动力学补偿相关
      vector_t jointVelLast_;
      vector_t jointAccel_;
      bool dynamicsCompensationInitialized_ = false;

      // -------------------------- 新增：7自由度手臂近奇异处理核心参数 --------------------------
      double armNullSpaceGain_{0.3};            // 零空间运动增益（0.2~0.5，轻量调整无末端影响）
      double armJacMinSingularThresh_{0.02};     // 雅克比最小奇异值近奇异阈值（0.005）
      double armJacCondThresh_{800.0};            // 雅克比条件数近奇异阈值（2000）
      double armDampingLambda0_{0.01};           // 非奇异基础阻尼（0.002）
      double armDampingGain_{0.4};              // 阻尼非线性增长系数（0.08）
      double armForceMax_{1000.0};                 // 末端最大力限幅（80N，物理特性适配）
      double armMomentMax_{60.0};                // 末端最大力矩限幅（15N·m，物理特性适配）
      const int pSCgSmoothWindowSize_{1};     // 动力学项pSCg滑窗平滑窗口大小（固定3）
      std::deque<Eigen::VectorXd> pSCgArmSmoothWindow_; // pSCg滑窗缓存
      // ----------------------------------------------------------------------------------------
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
