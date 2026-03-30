#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <angles/angles.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <chrono>
#include <unordered_set>

namespace ocs2 {
namespace mobile_manipulator {

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
  // 选取轴向变动最少，且变化范围为 initial_zyx 在180度以内的解
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q, 
                                           const Eigen::Matrix<SCALAR_T, 3, 1>& initial_zyx)
  {
      // 计算原始解
      Eigen::Matrix<SCALAR_T, 3, 1> zyx = quatToZyx(q);

      // 候选解集合
      std::vector<Eigen::Matrix<SCALAR_T, 3, 1>> candidates;

      // 对于yaw（索引0），保持其连续性，不强制归一化到[-π, π)
      // 而是基于初始值生成多个可能的yaw值
      SCALAR_T yaw_base = zyx(0);

      // 归一化yaw_base到[-π, π)范围用于比较
      SCALAR_T yaw_base_norm = yaw_base;
      while (yaw_base_norm >= M_PI) yaw_base_norm -= 2 * M_PI;
      while (yaw_base_norm < -M_PI) yaw_base_norm += 2 * M_PI;

      // 计算yaw_base相对于initial_zyx(0)的偏移圈数
      SCALAR_T yaw_diff = yaw_base_norm - fmod(initial_zyx(0), 2 * M_PI);
      // 将差值归一化到[-π, π)范围
      while (yaw_diff >= M_PI) yaw_diff -= 2 * M_PI;
      while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

      // 计算最佳圈数偏移
      int n_round = static_cast<int>(std::round((initial_zyx(0) - yaw_diff) / (2 * M_PI) - 
                                                std::floor(initial_zyx(0) / (2 * M_PI))));

      // 生成yaw候选值：基于初始值附近的最佳匹配
      std::vector<SCALAR_T> yaw_candidates;

      // 主要候选：基于圈数计算的最佳匹配
      SCALAR_T yaw_main = yaw_base_norm + n_round * 2 * M_PI + 
                          std::floor(initial_zyx(0) / (2 * M_PI)) * 2 * M_PI;
      yaw_candidates.push_back(yaw_main);

      // 考虑相邻圈数作为备选
      for (int i = -1; i <= 1; i++) {
          if (i != 0) {
              yaw_candidates.push_back(yaw_main + i * 2 * M_PI);
          }
      }

      // pitch（索引1）和roll（索引2）保持归一化到[-π, π)
      SCALAR_T pitch_base = zyx(1);
      SCALAR_T roll_base = zyx(2);

      // 归一化pitch和roll到[-π, π)
      while (pitch_base >= M_PI) pitch_base -= 2 * M_PI;
      while (pitch_base < -M_PI) pitch_base += 2 * M_PI;

      while (roll_base >= M_PI) roll_base -= 2 * M_PI;
      while (roll_base < -M_PI) roll_base += 2 * M_PI;

      // 生成所有候选解
      for (SCALAR_T yaw_cand : yaw_candidates) {
          // 原始pitch和roll
          Eigen::Matrix<SCALAR_T, 3, 1> cand;
          cand << yaw_cand, pitch_base, roll_base;
          candidates.push_back(cand);

          // pitch和roll的等价变换
          if (std::abs(std::cos(pitch_base)) > 1e-6) {
              // 非奇异性情况：pitch取π-pitch，roll取roll+π
              SCALAR_T pitch_alt = M_PI - pitch_base;
              SCALAR_T roll_alt = roll_base + M_PI;

              // 归一化pitch_alt和roll_alt到[-π, π)
              while (pitch_alt >= M_PI) pitch_alt -= 2 * M_PI;
              while (pitch_alt < -M_PI) pitch_alt += 2 * M_PI;

              while (roll_alt >= M_PI) roll_alt -= 2 * M_PI;
              while (roll_alt < -M_PI) roll_alt += 2 * M_PI;

              // 对应的yaw需要加π（保持连续性）
              SCALAR_T yaw_alt = yaw_cand + M_PI;
              // 不需要归一化yaw_alt，保持连续性

              Eigen::Matrix<SCALAR_T, 3, 1> cand2;
              cand2 << yaw_alt, pitch_alt, roll_alt;
              candidates.push_back(cand2);
          } else {
              // 奇异性情况
              SCALAR_T sign = pitch_base > 0 ? 1 : -1;

              // 解1
              Eigen::Matrix<SCALAR_T, 3, 1> cand_alt1;
              cand_alt1 << yaw_cand + roll_base, sign * M_PI / 2, 0;
              candidates.push_back(cand_alt1);

              // 解2
              Eigen::Matrix<SCALAR_T, 3, 1> cand_alt2;
              cand_alt2 << yaw_cand + roll_base + sign * M_PI, sign * M_PI / 2, 0;
              candidates.push_back(cand_alt2);
          }
      }

      // 寻找最优解
      auto best_candidate = candidates[0];
      int min_adjusted_axes = 3;
      SCALAR_T min_diff = std::numeric_limits<SCALAR_T>::max();

      for (const auto& cand : candidates) {
          // 计算yaw的差值（考虑连续性，不需要周期性处理）
          SCALAR_T diff_yaw = std::abs(cand(0) - initial_zyx(0));

          // 计算pitch和roll的差值（需要周期性处理）
          SCALAR_T diff_pitch = std::abs(cand(1) - initial_zyx(1));
          diff_pitch = std::min(diff_pitch, 2 * M_PI - diff_pitch);

          SCALAR_T diff_roll = std::abs(cand(2) - initial_zyx(2));
          diff_roll = std::min(diff_roll, 2 * M_PI - diff_roll);

          // 检查是否所有轴的变化都在180度以内
          if (diff_yaw <= M_PI && diff_pitch <= M_PI && diff_roll <= M_PI) 
          {
            // 总差值（可以对不同轴设置不同权重）
            SCALAR_T total_diff = diff_yaw + diff_pitch + diff_roll;

              // 计算调整的轴数
              int adjusted_axes = 0;
              if (diff_yaw > 1e-6) adjusted_axes++;
              if (diff_pitch > 1e-6) adjusted_axes++;
              if (diff_roll > 1e-6) adjusted_axes++;

              // 优先选择调整轴数少的解
              if (adjusted_axes < min_adjusted_axes) {
                  min_adjusted_axes = adjusted_axes;
                  min_diff = total_diff;
                  best_candidate = cand;
              } else if (adjusted_axes == min_adjusted_axes && total_diff < min_diff) {
                  min_diff = total_diff;
                  best_candidate = cand;
              }
          }
      }

      // 如果没有找到满足180度限制的解，放宽限制
      if (min_adjusted_axes == 3 && min_diff == std::numeric_limits<SCALAR_T>::max()) {
          // 重新评估所有候选，但不施加180度限制
          for (const auto& cand : candidates) {
              SCALAR_T diff_yaw = std::abs(cand(0) - initial_zyx(0));
              SCALAR_T diff_pitch = std::abs(cand(1) - initial_zyx(1));
              diff_pitch = std::min(diff_pitch, 2 * M_PI - diff_pitch);
              SCALAR_T diff_roll = std::abs(cand(2) - initial_zyx(2));
              diff_roll = std::min(diff_roll, 2 * M_PI - diff_roll);

              int adjusted_axes = 0;
              if (diff_yaw > 1e-6) adjusted_axes++;
              if (diff_pitch > 1e-6) adjusted_axes++;
              if (diff_roll > 1e-6) adjusted_axes++;

              SCALAR_T total_diff = diff_yaw + diff_pitch + diff_roll;

              if (adjusted_axes < min_adjusted_axes || 
                  (adjusted_axes == min_adjusted_axes && total_diff < min_diff)) {
                  min_adjusted_axes = adjusted_axes;
                  min_diff = total_diff;
                  best_candidate = cand;
              }
          }
      }

      return best_candidate;
  }

  /**
  * @brief 从旋转矩阵提取 pitch 和 yaw (假设 roll=0)
  * @param R 3x3旋转矩阵
  * @return std::pair<double, double> (pitch, yaw) 弧度
  * @throw 如果矩阵不符合 roll=0 的约束
  */
  std::pair<double, double> rotationMatrixToPitchYaw(const Eigen::Matrix3d& R) {
      // 静态变量用于存储上一次的 yaw 值
      static double last_yaw = 0.0;
      static bool has_last = false;

      // 验证 roll 接近0 (通过检查 R(2,1) 是否接近0)
      const double eps = 1e-6;
      if (std::abs(R(1,2)) > eps) {
          throw std::runtime_error("旋转矩阵不满足 roll=0 约束");
      }

      // pitch: 从 R(0,2) 和 R(2,2) 提取
      double pitch = std::atan2(R(0,2), R(2,2));

      // yaw: 从 R(1,0) 和 R(1,1) 提取
      double yaw_raw = std::atan2(R(1,0), R(1,1));

      double yaw = 0;
      if (!has_last) {
          // 首次调用，直接使用原始值
          yaw = yaw_raw;
          has_last = true;
      } else {
          // 计算差值，确保连续性
          double delta = yaw_raw - last_yaw;
          // 处理角度跳变（超过π的跳变）
          if (delta > M_PI) {
              delta -= 2 * M_PI;
          } else if (delta < -M_PI) {
              delta += 2 * M_PI;
          }
          // 累积更新，保持连续
          yaw = last_yaw + delta;
      }
        // 更新上一次的值
        last_yaw = yaw;

      return {pitch, yaw};
  }

  MobileManipulatorReferenceManager::MobileManipulatorReferenceManager(const ManipulatorModelInfo& info, const PinocchioInterface& pinocchioInterface, const std::string& taskFile)
  : ReferenceManager(TargetTrajectories(), ModeSchedule())
  , info_(info), singleArmJointDim_((info.armDim - 4) / 2)
  , pinocchioInterface_(pinocchioInterface)
  , taskFile_(taskFile)
  , stateInputTargetTrajectories_(TargetTrajectories({0}, {vector_t::Zero(info_.stateDim)}, {vector_t::Zero(info_.inputDim)}))
  , torsoTargetTrajectories_(TargetTrajectories({0}, {vector_t::Zero(6)}, {vector_t::Zero(6)}))
  , eeTargetTrajectories_{TargetTrajectories({0}, {vector_t::Zero(6)}, {vector_t::Zero(6)}), 
                          TargetTrajectories({0}, {vector_t::Zero(6)}, {vector_t::Zero(6)})}
  , currentActualState_(vector_t::Zero(info_.stateDim))
  {

    loadParamFromTaskFile();  // 加载配置参数

    baseDim_ = info_.stateDim-info_.armDim;
    cmd_arm_zyx_[0] = vector_t::Zero(6);
    cmd_arm_zyx_[1] = vector_t::Zero(6);
    
    // 初始化MPC控制模式为NoControl（完全接收上层下发的 TargetTrajectory, 其余话题无法接收）
    currentMpcControlMode_ = 2;  // NoControl

    /*************cmdPose相关初始化********************/
    currentCmdPose_.setZero();
    /************************************************/

    // 躯干相对底盘位姿指令初始化
    cmdTorsoPose_.setZero(6);

    // 注册日志记录器
    ros_logger_ = new humanoid::TopicLogger(nodeHandle_);

    /****************************ruckig位置规划器初始化*********************************/ 
    auto createCmdPosePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig>
    {
      int dofPose = baseDim_;
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(dofPose);
      Eigen::VectorXd max_velocity_ruckig, max_acceleration_ruckig, max_jerk_ruckig;
      max_velocity_ruckig.setZero(dofPose);
      max_acceleration_ruckig.setZero(dofPose);
      max_jerk_ruckig.setZero(dofPose);
      for(int i = 0; i < dofPose; i++)
      {
        max_velocity_ruckig[i] = wheel_move_spd_[i];
        max_acceleration_ruckig[i] = wheel_move_acc_[i];
        max_jerk_ruckig[i] = wheel_move_jerk_[i];
      }
      plannerPtr->setVelocityLimits(max_velocity_ruckig, -max_velocity_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_ruckig, -max_acceleration_ruckig);
      plannerPtr->setJerkLimits(max_jerk_ruckig);
      return plannerPtr;
    };
    cmdPosePlannerRuckigPtr_ = createCmdPosePlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createCmdPosePlanner());  // 一个世界系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createCmdPosePlanner());  // 一个局部系
    prevTargetPose_.setZero(baseDim_);
    prevTargetVel_.setZero(baseDim_);
    prevTargetAcc_.setZero(baseDim_);
    /*********************************************************************************/ 

    /****************************ruckig速度规划器初始化**********************************/ 
    Eigen::VectorXd max_acceleration_ruckig, max_jerk_ruckig;
    max_acceleration_ruckig.setZero(baseDim_);
    max_jerk_ruckig.setZero(baseDim_);
    for(int i = 0; i < baseDim_; i++)
    {
      max_acceleration_ruckig[i] = wheel_move_acc_[i];
      max_jerk_ruckig[i] = wheel_move_jerk_[i];
    }
    cmdVelPlannerRuckigPtr_ = std::make_shared<cmdVelPlannerWithRuckig>(baseDim_);
    cmdVel_prevTargetPose_.setZero(baseDim_);
    cmdVel_prevTargetVel_.setZero(baseDim_);
    cmdVel_prevTargetAcc_.setZero(baseDim_);

    cmdVelPlannerRuckigPtr_->setAccelerationLimits(max_acceleration_ruckig, -max_acceleration_ruckig);
    cmdVelPlannerRuckigPtr_->setJerkLimits(max_jerk_ruckig);
    /*********************************************************************************/

    /*******************ruckig躯干笛卡尔规划器初始化 (自由度:x, z, yaw, pitch)***************************/ 
    auto createTorsoPosePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(4, true);
      Eigen::VectorXd max_velocity_torsoPose_ruckig, max_acceleration_torsoPose_ruckig, max_jerk_torsoPose_ruckig;
      max_velocity_torsoPose_ruckig.setZero(4);
      max_acceleration_torsoPose_ruckig.setZero(4);
      max_jerk_torsoPose_ruckig.setZero(4);
      for(int i=0; i<4; i++)
      {
        max_velocity_torsoPose_ruckig[i] = torsoPose_move_spd_[i];
        max_acceleration_torsoPose_ruckig[i] = torsoPose_move_acc_[i];
        max_jerk_torsoPose_ruckig[i] = torsoPose_move_jerk_[i];
      }
      plannerPtr->setVelocityLimits(max_velocity_torsoPose_ruckig, -max_velocity_torsoPose_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_torsoPose_ruckig, -max_acceleration_torsoPose_ruckig);
      plannerPtr->setJerkLimits(max_jerk_torsoPose_ruckig);
      return plannerPtr;
    };
    torsoPosePlannerRuckigPtr_ = createTorsoPosePlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createTorsoPosePlanner());
    torsoPose_prevTargetPose_.setZero(4);
    torsoPose_prevTargetVel_.setZero(4);
    torsoPose_prevTargetAcc_.setZero(4);
    /*********************************************************************************/

    /********************ruckig下肢关节规划器初始化 (单位: 弧度)***************************/
    auto createLegJointPlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(4, true);
      Eigen::VectorXd max_velocity_legJoint_ruckig, max_acceleration_legJoint_ruckig, max_jerk_legJoint_ruckig;
      max_velocity_legJoint_ruckig.setZero(4);
      max_acceleration_legJoint_ruckig.setZero(4);
      max_jerk_legJoint_ruckig.setZero(4);
      for(int i=0; i<4; i++)
      {
        max_velocity_legJoint_ruckig.segment(i, 1) = legJoint_move_spd_.head<1>();
        max_acceleration_legJoint_ruckig.segment(i, 1) = legJoint_move_acc_.head<1>();
        max_jerk_legJoint_ruckig.segment(i, 1) = legJoint_move_jerk_.head<1>();
      }
      plannerPtr->setVelocityLimits(max_velocity_legJoint_ruckig, -max_velocity_legJoint_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_legJoint_ruckig, -max_acceleration_legJoint_ruckig);
      plannerPtr->setJerkLimits(max_jerk_legJoint_ruckig);
      return plannerPtr;
    };
    legJointPlannerRuckigPtr_ = createLegJointPlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createLegJointPlanner());
    legJoint_prevTargetPose_.setZero(4);
    legJoint_prevTargetVel_.setZero(4);
    legJoint_prevTargetAcc_.setZero(4);
    /*********************************************************************************/

    /*******************ruckig双臂笛卡尔规划器初始化 (Zyx欧拉角)***************************/ 
    auto createSingleArmEePlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(6, true);
      Eigen::VectorXd max_velocity_singleArm_ruckig, max_acceleration_singleArm_ruckig, max_jerk_singleArm_ruckig;
      max_velocity_singleArm_ruckig.setZero(6);
      max_acceleration_singleArm_ruckig.setZero(6);
      max_jerk_singleArm_ruckig.setZero(6);
      max_velocity_singleArm_ruckig.head<6>() = dualArm_move_spd_.head<6>();
      max_acceleration_singleArm_ruckig.head<6>() = dualArm_move_acc_.head<6>();
      max_jerk_singleArm_ruckig.head<6>() = dualArm_move_jerk_.head<6>();
      
      plannerPtr->setVelocityLimits(max_velocity_singleArm_ruckig, -max_velocity_singleArm_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_singleArm_ruckig, -max_acceleration_singleArm_ruckig);
      plannerPtr->setJerkLimits(max_jerk_singleArm_ruckig);
      return plannerPtr;
    };
    cmdDualArmEePlannerRuckigPtr_[0] = createSingleArmEePlanner();  // 左臂笛卡尔
    cmdDualArmEePlannerRuckigPtr_[1] = createSingleArmEePlanner();  // 右臂笛卡尔
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmEePlanner());  // 左臂世界系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmEePlanner());  // 右臂世界系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmEePlanner());  // 左臂局部系
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmEePlanner());  // 右臂局部系
    for(int armIdx = 0; armIdx < info_.eeFrames.size(); armIdx++)
    {
      cmdDualArm_prevTargetPose_[armIdx].setZero(6);
      cmdDualArm_prevTargetVel_[armIdx].setZero(6);
      cmdDualArm_prevTargetAcc_[armIdx].setZero(6);
    }
    /*********************************************************************************/

    /********************ruckig上肢关节规划器初始化 (单位: 弧度)***************************/
    auto createSingleArmJointPlanner = [&]() -> std::shared_ptr<cmdPosePlannerWithRuckig> 
    {
      auto plannerPtr = std::make_shared<cmdPosePlannerWithRuckig>(singleArmJointDim_, true);
      Eigen::VectorXd max_velocity_singleArmJoint_ruckig, max_acceleration_singleArmJoint_ruckig, max_jerk_singleArmJoint_ruckig;
      max_velocity_singleArmJoint_ruckig.setZero(singleArmJointDim_);
      max_acceleration_singleArmJoint_ruckig.setZero(singleArmJointDim_);
      max_jerk_singleArmJoint_ruckig.setZero(singleArmJointDim_);
      for(int i=0; i<singleArmJointDim_; i++)
      {
        max_velocity_singleArmJoint_ruckig.segment(i, 1) = armJoint_move_spd_.head<1>();
        max_acceleration_singleArmJoint_ruckig.segment(i, 1) = armJoint_move_acc_.head<1>();
        max_jerk_singleArmJoint_ruckig.segment(i, 1) = armJoint_move_jerk_.head<1>();
      }
      plannerPtr->setVelocityLimits(max_velocity_singleArmJoint_ruckig, -max_velocity_singleArmJoint_ruckig);
      plannerPtr->setAccelerationLimits(max_acceleration_singleArmJoint_ruckig, -max_acceleration_singleArmJoint_ruckig);
      plannerPtr->setJerkLimits(max_jerk_singleArmJoint_ruckig);
      return plannerPtr;
    };
    armJointPlannerRuckigPtr_[0] = createSingleArmJointPlanner();
    armJointPlannerRuckigPtr_[1] = createSingleArmJointPlanner();
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmJointPlanner());   // 左臂关节
    timedPlannerScheduler_.addTimedPlannerPosePtr(createSingleArmJointPlanner());   // 右臂关节
    for(int armIdx = 0; armIdx < 2; armIdx++)
    {
      armJoint_prevTargetPose_[armIdx].setZero(singleArmJointDim_);
      armJoint_prevTargetVel_[armIdx].setZero(singleArmJointDim_);
      armJoint_prevTargetAcc_[armIdx].setZero(singleArmJointDim_);
    }
    /*********************************************************************************/

    /******************** 时间调度器初始化相关 ***************************/
    isTimedPlannerUpdated_.resize(timedPlannerScheduler_.getPlannersNum(), false);
    desireTime_.resize(timedPlannerScheduler_.getPlannersNum(), 0.0);
    timedCmdVec_.resize(timedPlannerScheduler_.getPlannersNum());
    for (size_t i = 0; i < timedPlannerScheduler_.getPlannersNum(); ++i) {
        timedCmdVecMtx_.push_back(std::make_unique<std::mutex>());
    }
    timedPlannerScheduler_.setDiffDt(ruckigDt_);
    /******************************************************************/

    /************************躯干重置的参数相关*******************************/
    torsoResetMaxVel_.setZero(6);
    torsoResetMaxVel_ << 0.2, 0.2, 0.2, 0.5235, 0.5235, 0.5235;
    /**********************************************************************/
  }

  void MobileManipulatorReferenceManager::loadParamFromTaskFile(void)
  {
    // 参数初始化
    wheel_move_spd_ .setZero(3);
    wheel_move_acc_.setZero(3);
    wheel_move_jerk_.setZero(3);

    torsoPose_move_spd_.setZero(4);
    torsoPose_move_acc_.setZero(4);
    torsoPose_move_jerk_.setZero(4);

    dualArm_move_spd_.setZero(6);
    dualArm_move_acc_.setZero(6);
    dualArm_move_jerk_.setZero(6);

    legJoint_move_spd_.setZero(1);
    legJoint_move_acc_.setZero(1);
    legJoint_move_jerk_.setZero(1);

    armJoint_move_spd_.setZero(1);
    armJoint_move_acc_.setZero(1);
    armJoint_move_jerk_.setZero(1);

    // 从任务文件中加载参数
    std::string prefix = "referencekinematicLimit.";

    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_vel", wheel_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_acc", wheel_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "wheel_move.max_jerk", wheel_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_vel", torsoPose_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_acc", torsoPose_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "torsoPose_move.max_jerk", torsoPose_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_vel", dualArm_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_acc", dualArm_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "dualArm_move.max_jerk", dualArm_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_vel", legJoint_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_acc", legJoint_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "legJoint_move.max_jerk", legJoint_move_jerk_);

    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_vel", armJoint_move_spd_);
    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_acc", armJoint_move_acc_);
    loadData::loadEigenMatrix(taskFile_, prefix + "armJoint_move.max_jerk", armJoint_move_jerk_);

    // 打印加载的参数
    std::cout << "[MobileManipulatorReferenceManager] Loaded Parameters from Task File:" << std::endl;
    std::cout << "  wheel_move_spd_: " << wheel_move_spd_.transpose() << std::endl;
    std::cout << "  wheel_move_acc_: " << wheel_move_acc_.transpose() << std::endl;
    std::cout << "  wheel_move_jerk_: " << wheel_move_jerk_.transpose() << std::endl;

    std::cout << "  torsoPose_move_spd_: " << torsoPose_move_spd_.transpose() << std::endl;
    std::cout << "  torsoPose_move_acc_: " << torsoPose_move_acc_.transpose() << std::endl;
    std::cout << "  torsoPose_move_jerk_: " << torsoPose_move_jerk_.transpose() << std::endl;

    std::cout << "  dualArm_move_spd_: " << dualArm_move_spd_.transpose() << std::endl;
    std::cout << "  dualArm_move_acc_: " << dualArm_move_acc_.transpose() << std::endl;
    std::cout << "  dualArm_move_jerk_: " << dualArm_move_jerk_.transpose() << std::endl;

    std::cout << "  legJoint_move_spd_: " << legJoint_move_spd_ << std::endl;
    std::cout << "  legJoint_move_acc_: " << legJoint_move_acc_ << std::endl;
    std::cout << "  legJoint_move_jerk_: " << legJoint_move_jerk_ << std::endl;

    std::cout << "  armJoint_move_spd_: " << armJoint_move_spd_ << std::endl;
    std::cout << "  armJoint_move_acc_: " << armJoint_move_acc_ << std::endl;
    std::cout << "  armJoint_move_jerk_: " << armJoint_move_jerk_ << std::endl;

    /**************************ruckig 时间周期获取************************************/
    double desiredFreq = 0.0;
    loadData::loadCppDataType(taskFile_, "mpc.mpcDesiredFrequency", desiredFreq);
    ruckigDt_ = 1 / desiredFreq;
    /*******************************************************************************/
  }

  void MobileManipulatorReferenceManager::setRobotInitialArmJointTarget(ros::NodeHandle& input_nh)
  {
    std::vector<double> initialStateVector;
    while (!input_nh.hasParam("/robot_init_state_param"))
    {
        static bool first = true;
        if(first)
        {
          ROS_INFO("Waiting for '/robot_init_state_param' parameter to be set...");
          first = false;
        } 
        ros::Duration(0.2).sleep(); // 等待1秒后再次尝试
    }
    input_nh.getParam("/robot_init_state_param", initialStateVector);
    ROS_INFO("Set '/robot_init_state_param' parameter success !!!");

    Eigen::VectorXd initialState(initialStateVector.size());
    for (size_t i = 0; i < initialStateVector.size(); ++i)
    {
        initialState(i) = initialStateVector[i];
    }

    std::cout << "[MobileManipulatorReferenceManager] robot_init_state_param: " << initialState.transpose() << std::endl;

    arm_init_joint_traj_ = initialState.segment(7 + 4, info_.armDim - 4);   // 从初始获取手臂期望

  }

  void MobileManipulatorReferenceManager::setupSubscriptions(std::string nodeHandleName)
  {
    // 从参数服务器中更新初始期望
    setRobotInitialArmJointTarget(nodeHandle_);

    // 设置服务服务器
    controlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_mpc_control", 
                                                           &MobileManipulatorReferenceManager::controlModeService, this);
    getMpcControlModeServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_get_mpc_control_mode",
                                                           &MobileManipulatorReferenceManager::getMpcControlModeService, this);
    changeArmControlService_ = nodeHandle_.advertiseService("wheel_arm_change_arm_ctrl_mode", 
                                                           &MobileManipulatorReferenceManager::armControlModeSrvCallback, this);
    get_arm_control_mode_service_ = nodeHandle_.advertiseService("/humanoid_get_arm_ctrl_mode", 
                                                           &MobileManipulatorReferenceManager::getArmControlModeCallback, this);
    resetCmdVelRuckigServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_reset_cmd_vel_ruckig",
                                                           &MobileManipulatorReferenceManager::resetCmdVelRuckigService, this);
    // 设置各 ruckigPlanner 的参数的服务
    setRuckigPlannerParamsServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_set_ruckig_planner_params", 
                                                           &MobileManipulatorReferenceManager::setRuckigPlannerParamsService, this);
    // 设置获取躯干初始位姿服务
    getLbTorsoInitialPoseServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_get_torso_initial_pose", 
                                                           &MobileManipulatorReferenceManager::getLbTorsoInitialPoseService, this);
    
    // 设置基于时间的指令服务
    setLbTimedPosCmdServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_timed_single_cmd", 
                                                           &MobileManipulatorReferenceManager::setLbTimedPosCmdService, this);

    setLbMultiTimedPosCmdServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_timed_multi_cmd", 
                                                           &MobileManipulatorReferenceManager::setLbMultiTimedPosCmdService, this);
    
    setLbMultiTimedOfflineTrajServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_timed_offline_traj",
                                                           &MobileManipulatorReferenceManager::setLbMultiTimedOfflineTrajService, this);
    
    setLbOfflineTrajEnableServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_timed_offline_traj_enable",
                                                           &MobileManipulatorReferenceManager::setLbOfflineTrajEnableService, this);

    // 设置重置躯干指令
    resetTorsoStatusServiceServer_ = nodeHandle_.advertiseService("/mobile_manipulator_reset_torso", 
                                                           &MobileManipulatorReferenceManager::setLbResetTorsoService, this);
    // 订阅速度控制状态
    vel_control_state_sub_ = nodeHandle_.subscribe<std_msgs::Bool>(
      "/enable_vel_control_state", 1,
      [this](const std_msgs::Bool::ConstPtr& msg) {
        use_vel_control_.store(msg->data, std::memory_order_release);
      });


    auto targetVelocityCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdvel_mtx_.lock();
      isCmdVelUpdated_ = true;
      isCmdVelTimeUpdate_ = true;
      cmdVel_[0] = msg->linear.x;
      cmdVel_[1] = msg->linear.y;
      cmdVel_[2] = msg->angular.z;
      cmdvel_mtx_.unlock();
    };
    targetVelocitySubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, targetVelocityCallback);
    
    auto targetVelocityWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdvelWorld_mtx_.lock();
      isCmdVelWorldUpdated_ = true;
      isCmdVelTimeUpdate_ = true;
      cmdVelWorld_[0] = msg->linear.x;
      cmdVelWorld_[1] = msg->linear.y;
      cmdVelWorld_[2] = msg->angular.z;
      cmdvelWorld_mtx_.unlock();
    };
    targetVelocityWorldSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel_world", 1, targetVelocityWorldCallback);
    
    auto targetLbTorsoPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdTorsoPose_mtx_.lock();
      isCmdTorsoPoseUpdated_ = true;
      cmdTorsoPoseDesiredTime_ = 0.0;
      cmdTorsoPose_[0] = msg->linear.x;
      cmdTorsoPose_[1] = msg->linear.y;
      cmdTorsoPose_[2] = msg->linear.z;
      cmdTorsoPose_[3] = msg->angular.z;
      cmdTorsoPose_[4] = msg->angular.y;
      cmdTorsoPose_[5] = msg->angular.x;
      std::cout << "Received cmdTorsoPose: "<< cmdTorsoPose_.transpose() << std::endl;
      cmdTorsoPose_mtx_.unlock();
    };
    targetTorsoPoseSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_lb_torso_pose", 1, targetLbTorsoPoseCallback);
    
    targetTorsoPoseReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_torso_pose_reach_time", 10, false);
    
    auto targetPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdPose_mtx_.lock();
      isCmdPoseUpdated_ = true;
      cmdPoseDesiredTime_ = 0.0;
      cmdPose_[0] = msg->linear.x;
      cmdPose_[1] = msg->linear.y;
      cmdPose_[2] = msg->angular.z;
      std::cout << "Received cmdPose: [" << cmdPose_[0] << ", " << cmdPose_[1] << ", " << cmdPose_[2] << std::endl;
      cmdPose_mtx_.unlock();
    };
    targetPoseSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose", 1, targetPoseCallback);

    auto targetPoseWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
    {
      cmdPoseWorld_mtx_.lock();
      isCmdPoseWorldUpdated_ = true;
      cmdPoseDesiredTime_ = 0.0;
      cmdPoseWorld_[0] = msg->linear.x;
      cmdPoseWorld_[1] = msg->linear.y;
      cmdPoseWorld_[2] = msg->angular.z;
      std::cout << "Received cmdPoseWorld: [" << cmdPoseWorld_[0] << ", " << cmdPoseWorld_[1] << ", " << cmdPoseWorld_[2] << std::endl;
      cmdPoseWorld_mtx_.unlock();
    };
    targetPoseWorldSubscriber_ =
        nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose_world", 1, targetPoseWorldCallback);
    
    targetCmdPoseReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_cmd_pose_reach_time", 10, false);

    // 订阅双臂末端执行器位姿指令
    auto armEndEffectorCallback = [this](const kuavo_msgs::twoArmHandPoseCmd::ConstPtr &msg)
    {
      for(int armIdx = 0; armIdx < info_.eeFrames.size(); armIdx++)
      {
        desireMode_[armIdx] = handPoseCmdFrameToLbArmMode(msg->frame);
        if(desireMode_[armIdx] == LbArmControlMode::FalseMode) 
        {
          desireMode_[armIdx] = LbArmControlMode::JointSpace;
          continue;
        }

        if(desireMode_[armIdx] == LbArmControlMode::WorldFrame || 
           desireMode_[armIdx] == LbArmControlMode::LocalFrame)
        {
          if(armIdx == 0 && msg->hand_poses.left_pose.joint_angles.size() != 7)
          {
            ROS_ERROR("Left arm joint angles size is not 7");
            return;
          }

          if(armIdx == 1 && msg->hand_poses.right_pose.joint_angles.size() != 7)
          {
            ROS_ERROR("Right arm joint angles size is not 7");
            return;
          }

          Eigen::VectorXd currentEePose, currentEeVel, currentEeAcc;
          switch (desireMode_[armIdx])
          {
            case LbArmControlMode::WorldFrame: 
              timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? 
                                                            LbTimedPosCmdType::LEFT_ARM_WORLD_CMD : 
                                                            LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD, 
                                                            currentEePose, currentEeVel, currentEeAcc);
              break;
            case LbArmControlMode::LocalFrame: 
              timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? 
                                                            LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD : 
                                                            LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD, 
                                                            currentEePose, currentEeVel, currentEeAcc);
              break;
          }

          // 解析位姿 (x,y,z,qx,qy,qz,qw)
          vector_t arm_traj_pose = vector_t::Zero(7);
          switch(armIdx)
          {
            case 0: // 左臂
              arm_traj_pose[0] = msg->hand_poses.left_pose.pos_xyz[0];  // x
              arm_traj_pose[1] = msg->hand_poses.left_pose.pos_xyz[1];  // y
              arm_traj_pose[2] = msg->hand_poses.left_pose.pos_xyz[2];  // z
              arm_traj_pose[3] = msg->hand_poses.left_pose.quat_xyzw[0]; // qx
              arm_traj_pose[4] = msg->hand_poses.left_pose.quat_xyzw[1]; // qy
              arm_traj_pose[5] = msg->hand_poses.left_pose.quat_xyzw[2]; // qz
              arm_traj_pose[6] = msg->hand_poses.left_pose.quat_xyzw[3]; // qw
              break;
            case 1: // 右臂
              arm_traj_pose[0] = msg->hand_poses.right_pose.pos_xyz[0];  // x
              arm_traj_pose[1] = msg->hand_poses.right_pose.pos_xyz[1];  // y
              arm_traj_pose[2] = msg->hand_poses.right_pose.pos_xyz[2];  // z
              arm_traj_pose[3] = msg->hand_poses.right_pose.quat_xyzw[0]; // qx
              arm_traj_pose[4] = msg->hand_poses.right_pose.quat_xyzw[1]; // qy
              arm_traj_pose[5] = msg->hand_poses.right_pose.quat_xyzw[2]; // qz
              arm_traj_pose[6] = msg->hand_poses.right_pose.quat_xyzw[3]; // qw
              break;
          }

          Eigen::Vector3d initial_zyx = currentEePose.segment(3, 3);

          armPose_mtx_[armIdx].lock();
          cmd_arm_zyx_[armIdx].head(3) = arm_traj_pose.head(3);
          cmd_arm_zyx_[armIdx].segment(3, 3) = quatToZyx(Eigen::Quaterniond(arm_traj_pose.tail<4>()), 
                                                 initial_zyx);

          armPose_mtx_[armIdx].unlock();

          isCmdDualArmPoseUpdated_[armIdx] = true;
          cmdDualArmPoseDesiredTime_[armIdx] = 0.0;
        }

        if(desireMode_[armIdx] == LbArmControlMode::JointSpace) //进行关节控制时, 进行慢速稳定控制
        {
          arm_joint_traj_[armIdx] = vector_t::Zero(singleArmJointDim_);
          armJoint_mtx_[armIdx].lock();
          for (size_t i = 0; i < msg->hand_poses.left_pose.joint_angles.size(); ++i)
          {
            arm_joint_traj_[armIdx][i] = (armIdx == 0) ?
                                  msg->hand_poses.left_pose.joint_angles[i] * M_PI / 180.0 : // 转换为弧度
                                  msg->hand_poses.right_pose.joint_angles[i] * M_PI / 180.0; // 转换为弧度
          }
          armJoint_mtx_[armIdx].unlock();

          isCmdArmJointUpdated_[armIdx] = true;   // 触发关节指令规划
          cmdArmJointDesiredTime_[armIdx] = 0.0;
        }
      }
    };
    armEndEffectorSubscriber_ =
        nodeHandle_.subscribe<kuavo_msgs::twoArmHandPoseCmd>("/mm/two_arm_hand_pose_cmd", 1, armEndEffectorCallback);
    
    armEndEffectorReachTimePub_[0] = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_ee_reach_time/left", 10, false);
    armEndEffectorReachTimePub_[1] = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_ee_reach_time/right", 10, false);
    
    // 添加订阅/kuavo_arm_traj话题
    auto armJointTrajCallback = [this](const sensor_msgs::JointState::ConstPtr &msg)
    {
      // 解析关节角度数据
      arm_joint_traj_[0] = vector_t::Zero(singleArmJointDim_);
      arm_joint_traj_[1] = vector_t::Zero(singleArmJointDim_);

      armJoint_mtx_[0].lock();
      armJoint_mtx_[1].lock();
      for (size_t i = 0; i < msg->position.size() / 2; ++i)
      {
        arm_joint_traj_[0][i] = msg->position[i] * M_PI / 180.0; // 转换为弧度
        arm_joint_traj_[1][i] = msg->position[singleArmJointDim_ + i] * M_PI / 180.0; // 转换为弧度
      }
      armJoint_mtx_[1].unlock();
      armJoint_mtx_[0].unlock();

      desireMode_[0] = LbArmControlMode::JointSpace;  // 切换模式
      desireMode_[1] = LbArmControlMode::JointSpace;  // 切换模式

      cmdArmJointDesiredTime_[0] = 0.0;
      cmdArmJointDesiredTime_[1] = 0.0;
      isCmdArmJointUpdated_[0] = true;   // 触发关节指令规划
      isCmdArmJointUpdated_[1] = true;   // 触发关节指令规划
    };
    arm_joint_traj_sub_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, armJointTrajCallback);

    targetArmJointReachTimePub_[0] = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_joint_reach_time/left", 10, false);
    targetArmJointReachTimePub_[1] = nodeHandle_.advertise<std_msgs::Float32>("/lb_arm_joint_reach_time/right", 10, false);

    // 添加订阅/lb_leg_traj话题
    auto lbLegJointTrajCallback = [this](const sensor_msgs::JointState::ConstPtr &msg)
    {
      if(msg->position.size() != 4)  // 数据维度检查
      {
        std::cout << "[MobileManipulatorReferenceManager] 下肢关节轨迹维度错误! 期望4, 实际 " 
                  << msg->position.size() << std::endl;
        return;
      }
      
      // 解析关节角度数据
      lbLegJoint_mtx_.lock();
      lb_leg_traj_ = vector_t::Zero(msg->position.size());
      for (size_t i = 0; i < msg->position.size(); ++i)
      {
        lb_leg_traj_[i] = msg->position[i] * M_PI / 180.0; // 转换为弧度
      }
      cmdLegJointDesiredTime_ = 0.0;
      lbLegJoint_mtx_.unlock();

      isCmdLegJointUpdated_ = true;
    };
    lb_leg_joint_traj_sub_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/lb_leg_traj", 10, lbLegJointTrajCallback);

    // 设置笛卡尔跟踪 focus 躯干还是末端
    auto setFocusEeCallback = [this](const std_msgs::Bool::ConstPtr &msg)
    {
      bool flag = msg->data;
      setIsFocusEeStatus(flag);
      ROS_INFO_STREAM("[setFocusEeCallback] focus_ee: [ " << flag << " ]");
    };
    set_focus_ee_sub_ = nodeHandle_.subscribe<std_msgs::Bool>("/mobile_manipulator_focus_ee", 10, setFocusEeCallback);

    targetLegJointReachTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/lb_leg_joint_reach_time", 10, false);

    // 发布轮臂MPC当前控制模式
    mpcControlModePub_ = nodeHandle_.advertise<std_msgs::Int8>("/mobile_manipulator/lb_mpc_control_mode", 10, false);

    // 发布轮臂MPC的约束使用情况
    mpcConstraintUsagePub_ = nodeHandle_.advertise<std_msgs::Int8MultiArray>("/mobile_manipulator/lb_mpc_constraint_usage", 10, false);

    // 发布在modifyReference函数消耗的时间
    modifyReferenceTimePub_ = nodeHandle_.advertise<std_msgs::Float32>("/mobile_manipulator/lb_mpc_modify_ref_use_time", 10, false);
  }

  // // 获取第一次的目标轨迹，并分配到不同的约束轨迹，后续添加额外约束, 也需要在此初始化
  // void MobileManipulatorReferenceManager::getFirstTargetTrajectories(const TargetTrajectories& targetTrajectories)
  // {
  //   // 第一次的轨迹, 包括 state, input, 躯干相对底座位姿, 所有手臂末端位姿
  //   stateInputTargetTrajectories_.stateTrajectory.front()= targetTrajectories.stateTrajectory.front();
  //   stateInputTargetTrajectories_.inputTrajectory.front() = targetTrajectories.inputTrajectory.front();
  //   stateInputTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();

  //   torsoTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
  //   torsoTargetTrajectories_.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_, 7);

  //   eeTargetTrajectories_.timeTrajectory.front() = targetTrajectories.timeTrajectory.front();
  //   eeTargetTrajectories_.stateTrajectory.front() = targetTrajectories.stateTrajectory.front().segment(baseDim_ + 7, info_.eeFrames.size() * 6);
  // }

  // // 获取所有的目标轨迹，并分配到不同的约束轨迹
  // void MobileManipulatorReferenceManager::getAllTargetTrajectories(const TargetTrajectories& targetTrajectories)
  // {

  //   for(int i=0; i<targetTrajectories.timeTrajectory.size(); i++)
  //   {
  //     stateInputTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
  //     stateInputTargetTrajectories_.stateTrajectory[i].head(baseDim_) = targetTrajectories.stateTrajectory[i].head(baseDim_);

  //     torsoTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
  //     torsoTargetTrajectories_.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_, 7);

  //     eeTargetTrajectories_.timeTrajectory[i] = targetTrajectories.timeTrajectory[i];
  //     eeTargetTrajectories_.stateTrajectory[i] = targetTrajectories.stateTrajectory[i].segment(baseDim_ + 7, info_.eeFrames.size() * 6);
  //   }
  // }

  // 删除 TargetTrajectories 中 initTime 之前的所有帧，保留 initTime 前一个关键帧及之后的所有帧
  void MobileManipulatorReferenceManager::trimTargetTrajectoriesBeforeTime(scalar_t startTime)
  {
    // 辅助函数：修剪单个轨迹
    auto trimTrajectory = [startTime](TargetTrajectories& trajectory) {
      if (trajectory.timeTrajectory.empty() || trajectory.timeTrajectory.front() >= startTime) 
      {
        return;
      }

      // 如果只有一个元素，不做删减
      if (trajectory.timeTrajectory.size() <= 1) {
        return;
      }

      // 查找第一个大于或等于 startTime 的元素
      auto index = std::lower_bound(trajectory.timeTrajectory.begin(), 
                                trajectory.timeTrajectory.end(), 
                                startTime);
      
      // 计算要删除的元素数量, 保留 startTime 的之前一个及之后所有
      size_t eraseCount = std::distance(trajectory.timeTrajectory.begin(), index) - 1;

      // 如果存在需要删除的轨迹, 执行删除
      if (eraseCount > 0) {
        trajectory.timeTrajectory.erase(trajectory.timeTrajectory.begin(),  trajectory.timeTrajectory.begin() + eraseCount);
        // 删除 stateTrajectory 前 eraseCount 个元素
        if (!trajectory.stateTrajectory.empty() && trajectory.stateTrajectory.size() >= eraseCount) {
          trajectory.stateTrajectory.erase(trajectory.stateTrajectory.begin(), 
                                         trajectory.stateTrajectory.begin() + eraseCount);
        }
        if (!trajectory.inputTrajectory.empty() && trajectory.inputTrajectory.size() >= eraseCount) {
          trajectory.inputTrajectory.erase(trajectory.inputTrajectory.begin(), 
                                         trajectory.inputTrajectory.begin() + eraseCount);
        }
      }
    };

    // 对所有轨迹应用修剪
    trimTrajectory(stateInputTargetTrajectories_);
    trimTrajectory(torsoTargetTrajectories_);
    for(size_t i=0; i<info_.eeFrames.size(); i++)
    {
      trimTrajectory(eeTargetTrajectories_[i]);
    }
    if(isOfflineTrajUpdate_ && startTime > isofflineTrajUpdateStartTime_ + 1.0)  // 避免删除起始的未执行的数据, 1秒内需要将轨迹更新完成
    {
      trimTrajectory(torsoOfflineTraj_);
      for(size_t i=0; i<info_.eeFrames.size(); i++)
      {
        trimTrajectory(armEeOfflineTraj_[i]);
      }
    }
  }

  vector_t MobileManipulatorReferenceManager::targetTrajToPose6D(const TargetTrajectories& Traj, scalar_t initTime)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    vector_t zyx = quatToZyx(orientation);
    vector_t pose6D = vector_t::Zero(6);
    pose6D << position, zyx;

    return pose6D;
  }

  vector_t MobileManipulatorReferenceManager::targetTorsoTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static ContinuousEulerAnglesFromMatrix eulerUnwrapper;
    Eigen::Vector3d zyx = eulerUnwrapper.update(orientation);
    
    vector_t torsoPose6D = vector_t::Zero(6);
    torsoPose6D << position, zyx;

    return torsoPose6D;
  }

  vector_t MobileManipulatorReferenceManager::targetEeTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime, int eeInx)
  {
    const auto& timeTraj = Traj.timeTrajectory;
    const auto& stateTraj = Traj.stateTrajectory;

    vector_t position;
    Eigen::Quaterniond orientation;

    if (stateTraj.size() > 1) {
      // Normal interpolation case
      int index;
      scalar_t alpha;
      std::tie(index, alpha) = LinearInterpolation::timeSegment(initTime, timeTraj);

      const auto& lhs = stateTraj[index].head(7);
      const auto& rhs = stateTraj[index + 1].head(7);
      const Eigen::Quaterniond q_lhs(lhs.tail<4>());
      const Eigen::Quaterniond q_rhs(rhs.tail<4>());

      position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
      orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
      position = stateTraj.front().head(7).head(3);
      orientation = Eigen::Quaterniond(stateTraj.front().head(7).tail<4>());
    }

    // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static std::vector<ContinuousEulerAnglesFromMatrix> eulerUnwrapper(info_.eeFrames.size());
    Eigen::Vector3d zyx = eulerUnwrapper[eeInx].update(orientation);

    vector_t pose6D = vector_t::Zero(6);
    pose6D << position, zyx;

    return pose6D;
  }

  void MobileManipulatorReferenceManager::publishTargetTrajectoriesNear(scalar_t initTime)
  {
    ros_logger_->publishVector("mobile_manipulator/currentMpcTarget/state", stateInputTargetTrajectories_.getDesiredState(initTime));
    ros_logger_->publishVector("mobile_manipulator/currentMpcTarget/input", stateInputTargetTrajectories_.getDesiredInput(initTime));

    // vector_t torsoTraj = targetTorsoTrajToPose6DContinous(torsoTargetTrajectories_, initTime);
    ros_logger_->publishVector("mobile_manipulator/torso_target_6D", torsoTargetTrajectories_.getDesiredState(initTime));
 
    for(int i=0; i < info_.eeFrames.size(); i++)
    {
      ros_logger_->publishVector("mobile_manipulator/ee_target_6D/point" + std::to_string(i), eeTargetTrajectories_[i].getDesiredState(initTime));
    }
  }

  void MobileManipulatorReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule)
  {
    // 更新当前实际机器人状态
    {
      std::lock_guard<std::mutex> lock(currentActualState_mtx_);
      currentActualState_ = initState;
    }

    // 更新形参作为全局变量
    initTime_ = initTime;
    finalTime_ = finalTime;
    initState_ = initState;

    // if (use_vel_control_.load(std::memory_order_acquire))
    // {
    //   // 底盘状态采用期望, 避免模式切换时滑动
    //   initState_.head(baseDim_) = stateInputTargetTrajectories_.getDesiredState(initTime).head(baseDim_);
    // }
    
    /************************************ 更新状态差分 **************************************/
    Eigen::VectorXd dState, ddState;
    static CentralDifferenceDifferentiator stateDiff;
    static double stateDiffTime;
    stateDiffTime += ruckigDt_;   // 采用稳定的时间步长进行差分
    stateDiff.differentiate(initState_, stateDiffTime, dState, ddState);
    /**************************************************************************************/

    // 获取开始时间点
    auto startTime = std::chrono::system_clock::now();

    // 第一次进入，需要对原始轨迹数据进行初始化
    static bool firstRun{true};
    if(firstRun)
    {
      // 获取最初的躯干位姿期望
      initialTorsoPos_ = targetTrajectories.stateTrajectory.front().segment(baseDim_, 3);
      initialTorsoQuat_ = targetTrajectories.stateTrajectory.front().segment(baseDim_ + 3, 4);
      firstRun = false;
    }

    // 更新当前状态
    updateTimedSchedulerCurrentState(initTime_, initState_);

    // 更新当前期望
    updateTimedSchedulerTargetTraj();
    
    // 判断离线轨迹下发状态和给予对应期望赋值
    updateTimedOfflineTraj(initTime, finalTime);
    
    // 获取当前控制模式
    // controlMode_mtx_.lock();
    // int currentMode = currentMpcControlMode_;
    // controlMode_mtx_.unlock();

    // 判断模式是否发生切换，切换则使能切换标志
    static bool isChange{false}; // = getControlModeIsChange(currentMode);

    // switch(currentMode) // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
    // {
    //   case MpcControlMode::NoControl: 
    //     // updateNoControl(initTime, targetTrajectories, isChange); break;           // 模式0: 使用上层下发的 targetTrajectories, 

    //   case MpcControlMode::ArmOnly:   
    //     // updateArmOnlyControl(initTime, finalTime, initState, isChange); break;               // 模式1: 关节可动, 底盘锁住

    //   case MpcControlMode::BaseOnly:  
    //     // updateBaseOnlyControl(initTime, finalTime, initState, isChange); break;   // 模式2: 底盘可动, 下肢和手臂锁住

      // 确认是否需要重置
      if(isResetTorso_)
      {
        resetAllMpcTrajAndTarget(initTime, initState_);
        isResetTorso_ = false;
      }
      // case MpcControlMode::BaseArm:   
      updateBaseArmControl(initTime, finalTime, initState_, isChange);   // 模式3: 必须控制底盘, 手臂支持局部系和世界系笛卡尔和关节两种轨迹
        
      // case MpcControlMode::ArmEeOnly: 
      //   updateArmEeOnlyControl(initTime, finalTime, initState, isChange); break;             // 模式4: 底盘随末端移动, 不可控制, 手臂支持世界系笛卡尔轨迹
        
    //   default: std::cout << "设置了错误的控制模式, 请检查!!" << std::endl;
    // }

    // 对多个轨迹进行裁剪, 删除之前无效的轨迹
    trimTargetTrajectoriesBeforeTime(initTime);

    // 发布时间最近的目标轨迹
    publishTargetTrajectoriesNear(initTime);

    // 发布当前MPC控制模式
    std_msgs::Int8 modeMsg;
    modeMsg.data = MpcControlMode::BaseArm;
    mpcControlModePub_.publish(modeMsg);

    // 发布mpc使能各约束的标志, 按底盘, 下肢关节, 躯干, 手臂关节, 手臂末端轨迹顺序
    std_msgs::Int8MultiArray constraintUsageMsg;
    std::vector<int8_t> constraintUsageVec;
    constraintUsageVec.push_back(getEnableBaseTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableLegJointTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableTorsoPoseTargetTrajectories() ? 1 : 0);
    constraintUsageVec.push_back(getEnableArmJointTrack() ? 1 : 0);
    constraintUsageVec.push_back(getEnableEeTargetTrajectories() ? 1 : 0);
    constraintUsageVec.push_back(getEnableEeTargetLocalTrajectories() ? 1 : 0);
    constraintUsageMsg.data = constraintUsageVec;
    mpcConstraintUsagePub_.publish(constraintUsageMsg);

    // 获取结束时间点
    auto endTime = std::chrono::system_clock::now();

    // state
    ros_logger_->publishVector("/mobile_manipulator/initState_state", initState_);
    ros_logger_->publishVector("/mobile_manipulator/initState_dState", dState);
    ros_logger_->publishVector("/mobile_manipulator/initState_ddState", ddState);

    publishMultiPointPose_World(initState_);
    publishMultiPointPose_Local(initState_);

    // 发布 modifyReference 消耗时间，单位: 毫秒
    std_msgs::Float32 cntTimeMsg;
    cntTimeMsg.data = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    modifyReferenceTimePub_.publish(cntTimeMsg);
  }

  // 本体系的末端ruckig轨迹生成
  void MobileManipulatorReferenceManager::calcRuckigTrajWithEePose(int armIdx, double initTime, const vector_t &targetArmEePose, double desiredTime)
  {
    assert(targetArmEePose.size() == 6 && "dualArmPose dimension must be 6!");

    cmdDualArm_plannerInitialTime_[armIdx] = initTime;
    cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentPose(cmdDualArm_prevTargetPose_[armIdx]);
    cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentVelocity(cmdDualArm_prevTargetVel_[armIdx]);
    cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentAcceleration(cmdDualArm_prevTargetAcc_[armIdx]);

    cmdDualArmEePlannerRuckigPtr_[armIdx]->setTargetPose(targetArmEePose);
    double durationTime = cmdDualArmEePlannerRuckigPtr_[armIdx]->calcTrajectory(desiredTime);
   
    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    armEndEffectorReachTimePub_[armIdx].publish(time_msg); // 发布到达时间
  }

  // 从 ruckig 中取出对应时间戳的6D位姿, 转换为约束可接收的 target 格式, 下发
  void MobileManipulatorReferenceManager::generateDualArmEeTargetWithRuckig(int armIdx, double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的手臂末端位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(6);
    vector_t targetInput = vector_t::Zero(6);

    for(int i=0; i<timeIncrement+1; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度（双臂轨迹）
      double currentTime = initTime + i * dt;

      cmdDualArmEePlannerRuckigPtr_[armIdx]->getTrajectoryAtTime(currentTime - cmdDualArm_plannerInitialTime_[armIdx], 
                                                         currentTargetPose, currentTargetVel, currentTargetAcc);

      targetState = currentTargetPose.head(6);
      
      // 使用 Eigen 将 ZYX 欧拉角转换为四元数
      // Eigen::Quaterniond quat = Eigen::AngleAxisd(currentTargetPose(i*6+3), Eigen::Vector3d::UnitZ())   // yaw (Z)
      //                         * Eigen::AngleAxisd(currentTargetPose(i*6+4), Eigen::Vector3d::UnitY()) // pitch (Y)
      //                         * Eigen::AngleAxisd(currentTargetPose(i*6+5), Eigen::Vector3d::UnitX()); // roll (X)
      // targetState.segment(i*6 + 3, 4) = quat.coeffs();
      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    eeTargetTrajectories_[armIdx].timeTrajectory = timeTraj;
    eeTargetTrajectories_[armIdx].stateTrajectory = stateTraj;
    eeTargetTrajectories_[armIdx].inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdDualArmEePlannerRuckigPtr_[armIdx]->getTrajectoryAtTime(initTime - cmdDualArm_plannerInitialTime_[armIdx] + dt, 
                                                       cmdDualArm_prevTargetPose_[armIdx],
                                                       cmdDualArm_prevTargetVel_[armIdx],
                                                       cmdDualArm_prevTargetAcc_[armIdx]);
  }

  // 重置双臂末端Zyx插值器的初值
  void MobileManipulatorReferenceManager::resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode)
  {
    Eigen::VectorXd currentEePose, currentEeVel, currentEeAcc;

    switch (desireMode)
    {
      case LbArmControlMode::WorldFrame:
        timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? LbTimedPosCmdType::LEFT_ARM_WORLD_CMD : 
                                                                     LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::LocalFrame: 
        timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD : 
                                                                     LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::JointSpace: return;
      default:
        std::cerr << "[resetDualArmRuckig] 不支持该模式的末端轨迹生成, 返回" << std::endl;
        return;
    }

    cmdDualArm_prevTargetPose_[armIdx] = currentEePose;
    cmdDualArm_prevTargetVel_[armIdx] = currentEeVel;
    cmdDualArm_prevTargetAcc_[armIdx] = currentEeAcc;

    if(rePlanning)
    {
      cmdDualArm_plannerInitialTime_[armIdx] = initTime;
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentPose(cmdDualArm_prevTargetPose_[armIdx]);
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentVelocity(cmdDualArm_prevTargetVel_[armIdx]);
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentAcceleration(cmdDualArm_prevTargetAcc_[armIdx]);

      cmdDualArmEePlannerRuckigPtr_[armIdx]->setTargetPose(cmdDualArm_prevTargetPose_[armIdx]);
      double durationTime = cmdDualArmEePlannerRuckigPtr_[armIdx]->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, 
                                                             LbArmControlMode desireMode, const vector_t& targetArmEePose)
  {
    vector_t eeState = vector_t::Zero(info_.eeFrames.size() * 7);
    switch(desireMode)
    {
      case LbArmControlMode::WorldFrame:
        getCurrentEeWorldPose(eeState, initState);
        break;
      case LbArmControlMode::LocalFrame:
        getCurrentEeBasePose(eeState, initState);
        break;
      case LbArmControlMode::JointSpace: return;
      default:
        std::cerr << "[resetDualArmRuckig] 不支持该模式的末端轨迹生成, 返回" << std::endl;
        return;
    }
    Eigen::Vector3d initial_zyx = targetArmEePose.segment<3>(3);  // 采用期望作为初值, 猜测出最近的欧拉角
    Eigen::Vector3d zyx = quatToZyx(Eigen::Quaterniond(eeState.segment<4>(armIdx * 7 + 3)), initial_zyx);

    Eigen::VectorXd currentEePose, currentEeVel, currentEeAcc;

    switch (desireMode)
    {
      case LbArmControlMode::WorldFrame:
        timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? LbTimedPosCmdType::LEFT_ARM_WORLD_CMD : 
                                                                     LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::LocalFrame: 
        timedPlannerScheduler_.getTimedPlannerStates((armIdx == 0) ? LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD : 
                                                                     LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD, 
                                                     currentEePose, currentEeVel, currentEeAcc);
        break;
      case LbArmControlMode::JointSpace: return;
      default:
        std::cerr << "[resetDualArmRuckig] 不支持该模式的末端轨迹生成, 返回" << std::endl;
        return;
    }

    cmdDualArm_prevTargetPose_[armIdx] = currentEePose;
    cmdDualArm_prevTargetPose_[armIdx].tail<3>() = zyx;  // 替换为最近的欧拉角
    cmdDualArm_prevTargetVel_[armIdx] = currentEeVel;
    cmdDualArm_prevTargetAcc_[armIdx] = currentEeAcc;

    if(rePlanning)
    {
      cmdDualArm_plannerInitialTime_[armIdx] = initTime;
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentPose(cmdDualArm_prevTargetPose_[armIdx]);
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentVelocity(cmdDualArm_prevTargetVel_[armIdx]);
      cmdDualArmEePlannerRuckigPtr_[armIdx]->setCurrentAcceleration(cmdDualArm_prevTargetAcc_[armIdx]);

      cmdDualArmEePlannerRuckigPtr_[armIdx]->setTargetPose(cmdDualArm_prevTargetPose_[armIdx]);
      double durationTime = cmdDualArmEePlannerRuckigPtr_[armIdx]->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithTorsoPose(double initTime, const vector_t &targetTorsoPose, double desiredTime)
  {
    assert(targetTorsoPose.size() == 4 && "torsoPose dimension must be 4!");

    torsoPose_plannerInitialTime_ = initTime;
    torsoPosePlannerRuckigPtr_->setCurrentPose(torsoPose_prevTargetPose_);
    torsoPosePlannerRuckigPtr_->setCurrentVelocity(torsoPose_prevTargetVel_);
    torsoPosePlannerRuckigPtr_->setCurrentAcceleration(torsoPose_prevTargetAcc_);

    torsoPosePlannerRuckigPtr_->setTargetPose(targetTorsoPose);
    double durationTime = torsoPosePlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetTorsoPoseReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::generateTorsoPoseTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的躯干位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose_torso, currentTargetVel_torso, currentTargetAcc_torso;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(6);
    vector_t targetInput = vector_t::Zero(6);

    for(int i=0; i<timeIncrement+1; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + i * dt;

      torsoPosePlannerRuckigPtr_->getTrajectoryAtTime(currentTime - torsoPose_plannerInitialTime_, 
                                                      currentTargetPose_torso, 
                                                      currentTargetVel_torso, 
                                                      currentTargetAcc_torso);
      
      targetState[0] = currentTargetPose_torso[0];
      targetState[1] = initialTorsoPos_[1];
      targetState[2] = currentTargetPose_torso[1];

      targetState[3] = currentTargetPose_torso[2]; // yaw
      targetState[4] = currentTargetPose_torso[3]; // pitch
      targetState[5] = 0.0; // roll 固定为0

       // 使用 Eigen 将 ZYX 欧拉角转换为四元数
      // Eigen::Quaterniond quat = Eigen::AngleAxisd(currentTargetPose_torso[2], Eigen::Vector3d::UnitZ())   // yaw (Z)
      //                         * Eigen::AngleAxisd(currentTargetPose_torso[3], Eigen::Vector3d::UnitY()) // pitch (Y)
      //                         * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()); // roll (X)
      
      // targetState.tail(4) = quat.coeffs();

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    torsoTargetTrajectories_.timeTrajectory = timeTraj;
    torsoTargetTrajectories_.stateTrajectory = stateTraj;
    torsoTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    torsoPosePlannerRuckigPtr_->getTrajectoryAtTime(initTime - torsoPose_plannerInitialTime_ + dt, 
                                                    torsoPose_prevTargetPose_,
                                                    torsoPose_prevTargetVel_,
                                                    torsoPose_prevTargetAcc_);

  }

  void MobileManipulatorReferenceManager::resetTorsoPoseRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::TORSO_POSE_CMD, 
                                                 torsoPose_prevTargetPose_, 
                                                 torsoPose_prevTargetVel_, 
                                                 torsoPose_prevTargetAcc_);

    if(rePlanning)
    {
      torsoPose_plannerInitialTime_ = initTime;
      torsoPosePlannerRuckigPtr_->setCurrentPose(torsoPose_prevTargetPose_);
      torsoPosePlannerRuckigPtr_->setCurrentVelocity(torsoPose_prevTargetVel_);
      torsoPosePlannerRuckigPtr_->setCurrentAcceleration(torsoPose_prevTargetAcc_);

      torsoPosePlannerRuckigPtr_->setTargetPose(torsoPose_prevTargetPose_);
      double durationTime = torsoPosePlannerRuckigPtr_->calcTrajectory();
    }

  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithCmdPose(double initTime, const vector_t &targetBasePose, double desiredTime)
  {
    assert(targetBasePose.size() == baseDim_ && "cmdPose dimension must be baseDim_!");

    plannerInitialTime_ = initTime;
    cmdPosePlannerRuckigPtr_->setCurrentPose(prevTargetPose_);
    cmdPosePlannerRuckigPtr_->setCurrentVelocity(prevTargetVel_);
    cmdPosePlannerRuckigPtr_->setCurrentAcceleration(prevTargetAcc_);

    cmdPosePlannerRuckigPtr_->setTargetPose(targetBasePose);
    double durationTime = cmdPosePlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetCmdPoseReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::generatePoseTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_leftArm, currentTargetVel_leftArm, currentTargetAcc_leftArm;
    Eigen::VectorXd currentTargetPose_rightArm, currentTargetVel_rightArm, currentTargetAcc_rightArm;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);
    
    for(int i=0; i<timeIncrement+1; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + i * dt;

      cmdPosePlannerRuckigPtr_->getTrajectoryAtTime(currentTime - plannerInitialTime_, 
                                                    currentTargetPose, 
                                                    currentTargetVel, 
                                                    currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, 
                                                     currentTargetPose_legJoint, 
                                                     currentTargetVel_legJoint, 
                                                     currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[0], 
                                                        currentTargetPose_leftArm, 
                                                        currentTargetVel_leftArm, 
                                                        currentTargetAcc_leftArm);
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[1], 
                                                        currentTargetPose_rightArm, 
                                                        currentTargetVel_rightArm, 
                                                        currentTargetAcc_rightArm);

      targetState.head(baseDim_) = currentTargetPose; // [x, y, yaw]
      targetInput.head(baseDim_) = currentTargetVel;  // [vx, vy, wz]

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetPose_leftArm;  // 上肢左臂7自由度
      targetInput.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetVel_leftArm;
      targetState.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetPose_rightArm;  // 上肢右臂7自由度
      targetInput.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetVel_rightArm;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdPosePlannerRuckigPtr_->getTrajectoryAtTime(initTime - plannerInitialTime_ + dt, 
                                                  prevTargetPose_,
                                                  prevTargetVel_,
                                                  prevTargetAcc_);
    if(getEnableLegJointTrack() == true)
    {
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    }
    if(getEnableArmJointTrackForArm(0) == true)
    {
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[0] + dt, 
                                                   armJoint_prevTargetPose_[0], 
                                                   armJoint_prevTargetVel_[0], 
                                                   armJoint_prevTargetAcc_[0]);
    }
    if(getEnableArmJointTrackForArm(1) == true)
    {
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[1] + dt, 
                                                   armJoint_prevTargetPose_[1], 
                                                   armJoint_prevTargetVel_[1], 
                                                   armJoint_prevTargetAcc_[1]);
    }

  }

  void MobileManipulatorReferenceManager::resetCmdPoseRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::BASE_POS_WORLD_CMD, 
                                                 prevTargetPose_, prevTargetVel_, prevTargetAcc_);

    if(rePlanning)
    {
      plannerInitialTime_ = initTime;
      cmdPosePlannerRuckigPtr_->setCurrentPose(prevTargetPose_);
      cmdPosePlannerRuckigPtr_->setCurrentVelocity(prevTargetVel_);
      cmdPosePlannerRuckigPtr_->setCurrentAcceleration(prevTargetAcc_);

      cmdPosePlannerRuckigPtr_->setTargetPose(prevTargetPose_);
      double durationTime = cmdPosePlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::resetCmdPoseRuckigFromActualState(double initTime, const vector_t& initState, bool rePlanning)
  {
    // 使用 initState 的当前位置
    prevTargetPose_ = initState.head(baseDim_);
    prevTargetVel_.setZero(baseDim_);
    prevTargetAcc_.setZero(baseDim_);

    if(rePlanning)
    {
      plannerInitialTime_ = initTime;
      cmdPosePlannerRuckigPtr_->setCurrentPose(prevTargetPose_);
      cmdPosePlannerRuckigPtr_->setCurrentVelocity(prevTargetVel_);
      cmdPosePlannerRuckigPtr_->setCurrentAcceleration(prevTargetAcc_);

      cmdPosePlannerRuckigPtr_->setTargetPose(prevTargetPose_);
      double durationTime = cmdPosePlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithCmdVel(double initTime, const vector_t &targetBaseVel)
  {
    assert(targetBasePose.size() == baseDim_ && "cmdPose dimension must be baseDim_!");

    cmdVel_plannerInitialTime_ = initTime;
    cmdVelPlannerRuckigPtr_->setCurrentPose(cmdVel_prevTargetPose_);
    cmdVelPlannerRuckigPtr_->setCurrentVelocity(cmdVel_prevTargetVel_);
    cmdVelPlannerRuckigPtr_->setCurrentAcceleration(cmdVel_prevTargetAcc_);
    cmdVelPlannerRuckigPtr_->setTargetVelocity(targetBaseVel);

    cmdVelPlannerRuckigPtr_->calcTrajectory();
  }

  void MobileManipulatorReferenceManager::generateVelTargetBaseWithRuckig(double initTime, double finalTime, double dt, 
                                                                          const vector_t &initState)
  {
    // 提取初始位姿 [x, y, yaw]
    Eigen::Vector2d currentPos = initState.head(2);  // [x, y]
    scalar_t currentYaw = initState(2);              // yaw角度

    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::Vector3d velWorld = Eigen::Vector3d::Zero();   // 世界系速度

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_leftArm, currentTargetVel_leftArm, currentTargetAcc_leftArm;
    Eigen::VectorXd currentTargetPose_rightArm, currentTargetVel_rightArm, currentTargetAcc_rightArm;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);

    for(int i=1; i<timeIncrement+1; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + i * dt;

      cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - cmdVel_plannerInitialTime_, 
                                                   currentTargetPose, 
                                                   currentTargetVel, 
                                                   currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, 
                                                     currentTargetPose_legJoint, 
                                                     currentTargetVel_legJoint, 
                                                     currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[0], 
                                                        currentTargetPose_leftArm, 
                                                        currentTargetVel_leftArm, 
                                                        currentTargetAcc_leftArm);
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[1], 
                                                        currentTargetPose_rightArm, 
                                                        currentTargetVel_rightArm, 
                                                        currentTargetAcc_rightArm);

      if (i > 0) 
      {
          // 从第二个时间点开始才进行积分更新
          currentYaw += currentTargetVel[2] * dt;  // 更新偏航角期望

          Eigen::Matrix3d rotMat = Eigen::AngleAxisd(currentYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          velWorld = rotMat * currentTargetVel;

          currentPos(0) += velWorld(0) * dt ;  // x
          currentPos(1) += velWorld(1) * dt;  // y

          // 记录当前下一帧的位置期望
          static bool isFirst = true;
          if(isFirst)
          {
            cmdVel_prevTargetPose_.head(2) = currentPos;
            cmdVel_prevTargetPose_(2) = currentYaw;
            isFirst = false;
          }
      }

      targetState.head(2) = currentPos.head(2); // [x, y]
      targetState(2) = currentYaw;

      targetInput.head(2) = velWorld.head(3);  // [vx, vy, vyaw]
      targetInput(2) = currentTargetVel[2];

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetPose_leftArm;  // 上肢左臂7自由度
      targetInput.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetVel_leftArm;
      targetState.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetPose_rightArm;  // 上肢右臂7自由度
      targetInput.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetVel_rightArm;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存下一帧的规划期望
    Eigen::VectorXd dummy_position;
    cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdVel_plannerInitialTime_ + dt, 
                                                  dummy_position,
                                                  cmdVel_prevTargetVel_,
                                                  cmdVel_prevTargetAcc_);
    if(getEnableLegJointTrack() == true)
    {
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    }
    if(getEnableArmJointTrackForArm(0) == true)
    {
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[0] + dt, 
                                                   armJoint_prevTargetPose_[0], 
                                                   armJoint_prevTargetVel_[0], 
                                                   armJoint_prevTargetAcc_[0]);
    }
    if(getEnableArmJointTrackForArm(1) == true)
    {
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[1] + dt, 
                                                   armJoint_prevTargetPose_[1], 
                                                   armJoint_prevTargetVel_[1], 
                                                   armJoint_prevTargetAcc_[1]);
    }
  }

  void MobileManipulatorReferenceManager::generateVelTargetWithRuckig(double initTime, double finalTime, double dt)
  {
    // 使用 Ruckig 库生成平滑的底盘位姿轨迹
    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;

    int timeIncrement = (finalTime - initTime) / dt;

    Eigen::VectorXd currentTargetPose, currentTargetVel, currentTargetAcc;
    Eigen::VectorXd currentTargetPose_legJoint, currentTargetVel_legJoint, currentTargetAcc_legJoint;
    Eigen::VectorXd currentTargetPose_leftArm, currentTargetVel_leftArm, currentTargetAcc_leftArm;
    Eigen::VectorXd currentTargetPose_rightArm, currentTargetVel_rightArm, currentTargetAcc_rightArm;

    // 构建目标状态和输入
    vector_t targetState = vector_t::Zero(info_.stateDim);
    vector_t targetInput = vector_t::Zero(info_.inputDim);

    for(int i=0; i<timeIncrement+1; i++)
    {
      // 计算每个时间点的期望位姿、速度和加速度
      double currentTime = initTime + i * dt;

      cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - cmdVel_plannerInitialTime_, 
                                                   currentTargetPose, 
                                                   currentTargetVel, 
                                                   currentTargetAcc);
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(currentTime - legJoint_plannerInitialTime_, 
                                                     currentTargetPose_legJoint, 
                                                     currentTargetVel_legJoint, 
                                                     currentTargetAcc_legJoint);
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[0], 
                                                     currentTargetPose_leftArm, 
                                                     currentTargetVel_leftArm, 
                                                     currentTargetAcc_leftArm);
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(currentTime - armJoint_plannerInitialTime_[1], 
                                                     currentTargetPose_rightArm, 
                                                     currentTargetVel_rightArm, 
                                                     currentTargetAcc_rightArm);

      targetState.head(3) = currentTargetPose; // [x, y, yaw]
      targetInput.head(3) = currentTargetVel;  // [vx, vy, wz]

      targetState.segment(baseDim_, 4) = currentTargetPose_legJoint;  // 下肢4自由度
      targetInput.segment(baseDim_, 4) = currentTargetVel_legJoint;

      targetState.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetPose_leftArm;  // 上肢左臂7自由度
      targetInput.tail(info_.armDim - 4).head(singleArmJointDim_) = currentTargetVel_leftArm;
      targetState.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetPose_rightArm;  // 上肢右臂7自由度
      targetInput.tail(info_.armDim - 4).tail(singleArmJointDim_) = currentTargetVel_rightArm;

      timeTraj.push_back(currentTime);
      stateTraj.push_back(targetState);
      inputTraj.push_back(targetInput);
    }

    // 更新整段预测轨迹
    stateInputTargetTrajectories_.timeTrajectory = timeTraj;
    stateInputTargetTrajectories_.stateTrajectory = stateTraj;
    stateInputTargetTrajectories_.inputTrajectory = inputTraj;

    // 保存当前时间的规划期望
    cmdVelPlannerRuckigPtr_->getTrajectoryAtTime(initTime - cmdVel_plannerInitialTime_ + dt, 
                                                  cmdVel_prevTargetPose_,
                                                  cmdVel_prevTargetVel_,
                                                  cmdVel_prevTargetAcc_);
    if(getEnableLegJointTrack() == true)
    {
      legJointPlannerRuckigPtr_->getTrajectoryAtTime(initTime - legJoint_plannerInitialTime_ + dt, 
                                                   legJoint_prevTargetPose_, 
                                                   legJoint_prevTargetVel_, 
                                                   legJoint_prevTargetAcc_);
    }
    if(getEnableArmJointTrackForArm(0) == true)
    {
      armJointPlannerRuckigPtr_[0]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[0] + dt, 
                                                   armJoint_prevTargetPose_[0], 
                                                   armJoint_prevTargetVel_[0], 
                                                   armJoint_prevTargetAcc_[0]);
    }
    if(getEnableArmJointTrackForArm(1) == true)
    {
      armJointPlannerRuckigPtr_[1]->getTrajectoryAtTime(initTime - armJoint_plannerInitialTime_[1] + dt, 
                                                   armJoint_prevTargetPose_[1], 
                                                   armJoint_prevTargetVel_[1], 
                                                   armJoint_prevTargetAcc_[1]);
    }
  }

  void MobileManipulatorReferenceManager::resetCmdVelRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::BASE_POS_WORLD_CMD, 
                                                 cmdVel_prevTargetPose_, cmdVel_prevTargetVel_, cmdVel_prevTargetAcc_);

    if(rePlanning)
    {
      cmdVel_plannerInitialTime_ = initTime;
      cmdVelPlannerRuckigPtr_->setCurrentPose(cmdVel_prevTargetPose_);
      cmdVelPlannerRuckigPtr_->setCurrentVelocity(cmdVel_prevTargetVel_);
      cmdVelPlannerRuckigPtr_->setCurrentAcceleration(cmdVel_prevTargetAcc_);
      cmdVelPlannerRuckigPtr_->setTargetVelocity(cmdVel_prevTargetVel_);

      cmdVelPlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::resetCmdVelRuckigFromActualState(double initTime, const vector_t& initState, bool rePlanning)
  {

    // 使用 initState 重置速度规划
    cmdVel_prevTargetPose_ = initState.head(baseDim_);
    cmdVel_prevTargetVel_.setZero(baseDim_);
    cmdVel_prevTargetAcc_.setZero(baseDim_);

    if(rePlanning)
    {
      cmdVel_plannerInitialTime_ = initTime;
      cmdVelPlannerRuckigPtr_->setCurrentPose(cmdVel_prevTargetPose_);
      cmdVelPlannerRuckigPtr_->setCurrentVelocity(cmdVel_prevTargetVel_);
      cmdVelPlannerRuckigPtr_->setCurrentAcceleration(cmdVel_prevTargetAcc_);
      cmdVelPlannerRuckigPtr_->setTargetVelocity(cmdVel_prevTargetVel_);

      cmdVelPlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithLegJoint(double initTime, const vector_t &targetLegJoint, double desiredTime)
  {
    assert(targetLegJoint.size() == 4 && "armJoint dimension must be 4!");

    legJoint_plannerInitialTime_ = initTime;
    legJointPlannerRuckigPtr_->setCurrentPose(legJoint_prevTargetPose_);
    legJointPlannerRuckigPtr_->setCurrentVelocity(legJoint_prevTargetVel_);
    legJointPlannerRuckigPtr_->setCurrentAcceleration(legJoint_prevTargetAcc_);

    legJointPlannerRuckigPtr_->setTargetPose(targetLegJoint);
    double durationTime = legJointPlannerRuckigPtr_->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetLegJointReachTimePub_.publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::resetLegJointRuckig(double initTime, const vector_t& initState, bool rePlanning)
  {
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::LEG_JOINT_CMD, 
                                                 legJoint_prevTargetPose_, 
                                                 legJoint_prevTargetVel_, 
                                                 legJoint_prevTargetAcc_);

    if(rePlanning)
    {
      legJoint_plannerInitialTime_ = initTime;
      legJointPlannerRuckigPtr_->setCurrentPose(legJoint_prevTargetPose_);
      legJointPlannerRuckigPtr_->setCurrentVelocity(legJoint_prevTargetVel_);
      legJointPlannerRuckigPtr_->setCurrentAcceleration(legJoint_prevTargetAcc_);

      legJointPlannerRuckigPtr_->setTargetPose(legJoint_prevTargetPose_);
      double durationTime = legJointPlannerRuckigPtr_->calcTrajectory();
    }
  }

  void MobileManipulatorReferenceManager::calcRuckigTrajWithArmJoint(int armIdx, double initTime, const vector_t &targetArmJoint, double desiredTime)
  {
    assert(targetArmJoint.size() == singleArmJointDim_ && "armJoint dimension must be singleArmJointDim_ !");

    armJoint_plannerInitialTime_[armIdx] = initTime;
    armJointPlannerRuckigPtr_[armIdx]->setCurrentPose(armJoint_prevTargetPose_[armIdx]);
    armJointPlannerRuckigPtr_[armIdx]->setCurrentVelocity(armJoint_prevTargetVel_[armIdx]);
    armJointPlannerRuckigPtr_[armIdx]->setCurrentAcceleration(armJoint_prevTargetAcc_[armIdx]);

    armJointPlannerRuckigPtr_[armIdx]->setTargetPose(targetArmJoint);
    double durationTime = armJointPlannerRuckigPtr_[armIdx]->calcTrajectory(desiredTime);

    std_msgs::Float32 time_msg;
    time_msg.data = durationTime;
    targetArmJointReachTimePub_[armIdx].publish(time_msg); // 发布到达时间
  }

  void MobileManipulatorReferenceManager::resetArmJointRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning)
  {
    LbTimedPosCmdType cmdType = (armIdx == 0) ? LbTimedPosCmdType::LEFT_ARM_JOINT_CMD : 
                                                LbTimedPosCmdType::RIGHT_ARM_JOINT_CMD;

    timedPlannerScheduler_.getTimedPlannerStates(cmdType, 
                                                 armJoint_prevTargetPose_[armIdx], 
                                                 armJoint_prevTargetVel_[armIdx], 
                                                 armJoint_prevTargetAcc_[armIdx]);

    if(rePlanning)
    {
      armJoint_plannerInitialTime_[armIdx] = initTime;
      armJointPlannerRuckigPtr_[armIdx]->setCurrentPose(armJoint_prevTargetPose_[armIdx]);
      armJointPlannerRuckigPtr_[armIdx]->setCurrentVelocity(armJoint_prevTargetVel_[armIdx]);
      armJointPlannerRuckigPtr_[armIdx]->setCurrentAcceleration(armJoint_prevTargetAcc_[armIdx]);

      armJointPlannerRuckigPtr_[armIdx]->setTargetPose(armJoint_prevTargetPose_[armIdx]);
      double durationTime = armJointPlannerRuckigPtr_[armIdx]->calcTrajectory();
    }
  }

  bool MobileManipulatorReferenceManager::controlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, 
                                                           kuavo_msgs::changeTorsoCtrlMode::Response& res)
  {
    controlMode_mtx_.lock();
    
    // 验证控制模式的有效性
    if (req.control_mode < 0 || req.control_mode > 4) {
      res.result = false;
      res.mode = currentMpcControlMode_;
      res.message = "Invalid control mode. Valid modes: 0(NoControl), 1(ArmOnly), 2(BaseOnly), 3(BaseArm), 4(ArmEeOnly)";
      controlMode_mtx_.unlock();
      return true;
    }
    
    // 更新控制模式
    int previousMode = currentMpcControlMode_;
    currentMpcControlMode_ = req.control_mode;
    
    // 根据控制模式设置相应的行为
    switch (currentMpcControlMode_) {
      case 0:  // NoControl
        res.message = "Switched to NoControl mode - no active control";
        break;
      case 1:  // ArmOnly
        res.message = "Switched to ArmOnly mode - controlling arms only, base fixed";
        break;
      case 2:  // BaseOnly
        res.message = "Switched to BaseOnly mode - controlling base only, arms fixed";
        break;
      case 3:  // BaseArm
        res.message = "Switched to BaseArm mode - controlling both base and arms";
        break;
      case 4: //ArmEeOnly
        res.message = "Switched to ArmEeOnly mode - controlling arms Ee only";
        break;
      default:
        res.message = "Unknown control mode";
        break;
    }
    
    res.result = true;
    res.mode = currentMpcControlMode_;
    
    // 打印模式切换信息
    std::cout << "[MobileManipulatorReferenceManager] MPC Control mode changed from " 
              << previousMode << " to " << currentMpcControlMode_ << ": " << res.message << std::endl;
    
    controlMode_mtx_.unlock();
    return true;
  }

  LbArmControlMode MobileManipulatorReferenceManager::handPoseCmdFrameToLbArmMode(int frame)
  {
    static LbArmControlMode lastMode = LbArmControlMode::JointSpace;
    LbArmControlMode mode = lastMode;
    switch(frame)
    {
      case 0:   break;                                // keep current frame            
      case 1: 
        mode = LbArmControlMode::WorldFrame; break;   // world frame (based on odom)
      case 2: 
        mode = LbArmControlMode::LocalFrame; break;   // local frame
      case 5: 
        mode = LbArmControlMode::JointSpace; break;   // joint space
      default:
        mode = LbArmControlMode::FalseMode; 
        // std::cerr << "[MobileManipulatorReferenceManager] Unsupported frame type '" 
        // << frame << "' in handPoseCmdFrameToLbArmMode, change To JointSpace." << std::endl;
    }
    return mode;
  }

  bool MobileManipulatorReferenceManager::getMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req,
                                                                   kuavo_msgs::changeTorsoCtrlMode::Response& res)
  {
    std::lock_guard<std::mutex> lock(controlMode_mtx_);
    res.result = true;
    res.mode = currentMpcControlMode_;
    res.message = "Success";
    return true;
  }

  bool MobileManipulatorReferenceManager::armControlModeSrvCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {

    res.result = true;
    switch (req.control_mode)
    {
    case 0:
      res.message = "Arm control mode 0: keep current control position";
      break;
    case 1:
      res.message = "Arm control mode 1: reset arm to initial Target";
      break;
    case 2:
      res.message = "Arm control mode 2: using external controller";
      break;
    default:
      res.result = false;
      res.message = "Invalid control mode :" + std::to_string(req.control_mode);
      break;
    }
    if (res.result)
    {
      offlineTrajDisable_ = true;
      currentArmControlMode_ = static_cast<LbArmControlServiceMode>(req.control_mode);

      std::cout << "currentArmControlMode_:"<< currentArmControlMode_ << std::endl;
      res.mode = currentArmControlMode_;
      ROS_INFO_STREAM(res.message);
      vector_t arm_control_mode_vec(1);
      arm_control_mode_vec << currentArmControlMode_;
      ros_logger_->publishVector("/humanoid/mpc/arm_control_mode", arm_control_mode_vec);

      for(int armIdx=0; armIdx<2; armIdx++)
      {
        isCmdArmJointUpdated_[armIdx] = true; // 触发关节指令规划
        desireMode_[armIdx] = LbArmControlMode::JointSpace;
      }
    }
    else
    {
      ROS_ERROR_STREAM(res.message);
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::resetCmdVelRuckigService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    // 使用当前时间进行重置
    double initTime = ros::Time::now().toSec();

    // 获取当前实际机器人状态
    vector_t initState;
    {
      std::lock_guard<std::mutex> lock(currentActualState_mtx_);
      if(currentActualState_.size() == info_.stateDim)
      {
        initState = currentActualState_;
      }
      else
      {
        // 如果还没有实际状态，使用轨迹中的位置作为后备
        initState = stateInputTargetTrajectories_.getDesiredState(initTime);
        ROS_WARN_STREAM("[resetCmdVelRuckigService] Using trajectory state as fallback, actual state not available yet");
      }
    }

    // 清空指令和标志位
    {
      cmdvel_mtx_.lock();
      cmdVel_.setZero();
      isCmdVelUpdated_ = false;
      cmdvel_mtx_.unlock();

      cmdvelWorld_mtx_.lock();
      cmdVelWorld_.setZero();
      isCmdVelWorldUpdated_ = false;
      cmdvelWorld_mtx_.unlock();
    }

    // 清空指令和标志位
    {
      cmdPose_mtx_.lock();
      isCmdPoseUpdated_ = false;
      cmdPose_mtx_.unlock();

      cmdPoseWorld_mtx_.lock();
      isCmdPoseWorldUpdated_ = false;
      cmdPoseWorld_mtx_.unlock();
    }

    // 重置位置规划器
    resetCmdPoseRuckigFromActualState(initTime, initState, req.data);

    // 重置速度规划器
    resetCmdVelRuckigFromActualState(initTime, initState, req.data);

    res.success = true;
    res.message = "Successfully reset cmdVel and cmdPose Ruckig planners using current position. All commands cleared. Robot will stop moving. re_planning=" + std::string(req.data ? "true" : "false");

    return true;
  }


  bool MobileManipulatorReferenceManager::getArmControlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    res.result = true;
    res.mode = currentArmControlMode_;
    return true;
  };

  bool MobileManipulatorReferenceManager::setRuckigPlannerParamsService(kuavo_msgs::setRuckigPlannerParams::Request &req, 
                                                                        kuavo_msgs::setRuckigPlannerParams::Response &res)
  {
    res.result = true;
    res.message = "Set Ruckig Planner Limits Success";

    // 确认规划器类型
    std::shared_ptr<cmdPosePlannerWithRuckig> plannerPosePtr;
    std::shared_ptr<cmdVelPlannerWithRuckig> plannerVelPtr;
    bool is_pos_planner = true; // true: pos planner, false: vel planner
    switch(req.planner_index)
    {
      case 0: // Base Pose Planner
        plannerPosePtr = cmdPosePlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Base Pose Planner Params.");
        break;
      case 1: // Base Velocity Planner
        plannerVelPtr = cmdVelPlannerRuckigPtr_;
        is_pos_planner = false;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Base Velocity Planner Params.");
        break;
      case 2: // Torso Pose Planner
        plannerPosePtr = torsoPosePlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Torso Pose Planner Params.");
        break;
      case 3: // Leg Joint Planner
        plannerPosePtr = legJointPlannerRuckigPtr_;
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Leg Joint Planner Params.");
        break;
      case 4: // Left Arm EE Pose Planner
        plannerPosePtr = cmdDualArmEePlannerRuckigPtr_[0];
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Left Arm EE Pose Planner Params.");
        break;
      case 5: // Right Arm EE Pose Planner
        plannerPosePtr = cmdDualArmEePlannerRuckigPtr_[1];
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Right Arm EE Pose Planner Params.");
        break;
      case 6: // Left Arm Joint Planner
        plannerPosePtr = armJointPlannerRuckigPtr_[0];
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Left Arm Joint Pose Planner Params.");
        break;
      case 7: // Right Arm Joint Planner
        plannerPosePtr = armJointPlannerRuckigPtr_[1];
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Setting Right Arm Joint Pose Planner Params.");
        break;
      default:
        res.result = false;
        res.message = "Invalid planner index: " + std::to_string(req.planner_index);
        ROS_ERROR_STREAM(res.message);
        break;
    }

    // 设置同步模式
    if (is_pos_planner)
    {
      plannerPosePtr->setPlannerSyncMode(req.is_sync);

      int cmdType = SrvRequestIndexToCmdType(req.planner_index);
      if(cmdType != -1)
      {
        timedPlannerScheduler_.setTimedPlannerSyncMode(cmdType, req.is_sync);
      }
      if(cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
      {
        timedPlannerScheduler_.setTimedPlannerSyncMode(cmdType+1, req.is_sync);
      }
      if(cmdType == static_cast<int>(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD) ||
         cmdType == static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD))  // 世界系时需要同步更新局部系设置
      {
        timedPlannerScheduler_.setTimedPlannerSyncMode(cmdType+2, req.is_sync);
      }

      ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Planner to " 
                   << (req.is_sync ? "Sync" : "Async") << " Mode.");
    }
    
    // 转换数据到Eigen::VectorXd并处理可选的负向限制
    Eigen::VectorXd vel_max, vel_min, acc_max, acc_min, jerk_max;
    bool has_velocity_limits = false;
    bool has_acceleration_limits = false;
    bool has_jerk_limits = false;
    
    if (!req.velocity_max.empty())
    {
      vel_max = Eigen::Map<Eigen::VectorXd>(req.velocity_max.data(), req.velocity_max.size());
      if (!req.velocity_min.empty() && 
          req.velocity_min.size() == req.velocity_max.size())
      {
        vel_min = Eigen::Map<Eigen::VectorXd>(req.velocity_min.data(), req.velocity_min.size());
      } 
      else 
      {
        ROS_WARN_STREAM("[setRuckigPlannerParamsService] velocity_min size mismatch, using -velocity_max");
        vel_min = -vel_max;
      }
      has_velocity_limits = true;
    }

    if (!req.acceleration_max.empty())
    {
      acc_max = Eigen::Map<Eigen::VectorXd>(req.acceleration_max.data(), req.acceleration_max.size());
      if (!req.acceleration_min.empty() && 
          req.acceleration_min.size() == req.acceleration_max.size())
      {
        acc_min = Eigen::Map<Eigen::VectorXd>(req.acceleration_min.data(), req.acceleration_min.size());
      } 
      else 
      {
        ROS_WARN_STREAM("[setRuckigPlannerParamsService] acceleration_min size mismatch, using -acceleration_max");
        acc_min = -acc_max;
      }
      has_acceleration_limits = true;
    }

    if (!req.jerk_max.empty())
    {
      jerk_max = Eigen::Map<Eigen::VectorXd>(req.jerk_max.data(), req.jerk_max.size());
      has_jerk_limits = true;
    }
    
    if (is_pos_planner)
    {
      if (has_velocity_limits && vel_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's velocity DOF size mismatch! input size: " + std::to_string(vel_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_acceleration_limits && acc_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's acceleration DOF size mismatch! input size: " + std::to_string(acc_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_jerk_limits && jerk_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Pose Planner's jerk DOF size mismatch! input size: " + std::to_string(jerk_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_velocity_limits)
      {
        plannerPosePtr->setVelocityLimits(vel_max, vel_min);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerVelocityLimits(cmdType, vel_max, vel_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerVelocityLimits(cmdType+1, vel_max, vel_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD) ||
           cmdType == static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerVelocityLimits(cmdType+2, vel_max, vel_min);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Velocity Limits.");
        ROS_INFO_STREAM("  vel_max: [" << vel_max.transpose() << "]");
        ROS_INFO_STREAM("  vel_min: [" << vel_min.transpose() << "]");
      }
      if (has_acceleration_limits)
      {
        plannerPosePtr->setAccelerationLimits(acc_max, acc_min);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerAccelerationLimits(cmdType, acc_max, acc_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerAccelerationLimits(cmdType+1, acc_max, acc_min);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD) ||
           cmdType == static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerAccelerationLimits(cmdType+2, acc_max, acc_min);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Acceleration Limits.");
        ROS_INFO_STREAM("  acc_max: [" << acc_max.transpose() << "]");
        ROS_INFO_STREAM("  acc_min: [" << acc_min.transpose() << "]");
      }
      if (has_jerk_limits)
      {
        plannerPosePtr->setJerkLimits(jerk_max);
        int cmdType = SrvRequestIndexToCmdType(req.planner_index);
        if(cmdType != -1)
        {
          timedPlannerScheduler_.updateTimedPlannerJerkLimits(cmdType, jerk_max);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerJerkLimits(cmdType+1, jerk_max);
        }
        if(cmdType == static_cast<int>(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD) ||
           cmdType == static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD))  // 世界系时需要同步更新局部系设置
        {
          timedPlannerScheduler_.updateTimedPlannerJerkLimits(cmdType+2, jerk_max);
        }
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Pose Planner Jerk Limits.");
        ROS_INFO_STREAM("  jerk_max: [" << jerk_max.transpose() << "]");
      }
    }
    else
    {
      if (has_acceleration_limits && acc_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Vel Planner's acceleration DOF size mismatch! input size: " + std::to_string(acc_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }
      if (has_jerk_limits && jerk_max.size() != plannerPosePtr->getDofNum()) 
      {
        res.result = false;
        res.message = "Vel Planner's jerk DOF size mismatch! input size: " + std::to_string(jerk_max.size()) + 
                      ", required size: " + std::to_string(plannerPosePtr->getDofNum());
        ROS_ERROR_STREAM("[setRuckigPlannerParamsService]  " + res.message);
        return true;
      }

      if (has_acceleration_limits)
      {
        plannerVelPtr->setAccelerationLimits(acc_max, acc_min);
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Vel Planner Acceleration Limits.");
        ROS_INFO_STREAM("  acc_max: [" << acc_max.transpose() << "]");
        ROS_INFO_STREAM("  acc_min: [" << acc_min.transpose() << "]");
      }
      if (has_jerk_limits)
      {
        plannerVelPtr->setJerkLimits(jerk_max);
        ROS_INFO_STREAM("[setRuckigPlannerParamsService] Set Vel Planner Jerk Limits.");
        ROS_INFO_STREAM("  jerk_max: [" << jerk_max.transpose() << "]");
      }
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::getLbTorsoInitialPoseService(kuavo_msgs::getLbTorsoInitialPose::Request &req, 
                                                                       kuavo_msgs::getLbTorsoInitialPose::Response &res)
  {
    if (req.isNeed)
    {
      Eigen::Vector3d zyx = quatToZyx(Eigen::Quaterniond(initialTorsoQuat_.head<4>()));

      res.linear.x = initialTorsoPos_(0);
      res.linear.y = initialTorsoPos_(1);
      res.linear.z = initialTorsoPos_(2);
      res.angular.z = zyx(0);
      res.angular.y = zyx(1);
      res.angular.x = zyx(2);
      
      res.result = true;
      res.message = "Get torso initial pose success.";
      ROS_INFO_STREAM("[MobileManipulatorReferenceManager] Get torso initial pose: "
                      << "Position [" << initialTorsoPos_.transpose() << "], "
                      << "Orientation (ZYX) [" << zyx.transpose() << "]");
    }
    else
    {
      res.result = false;
      res.message = "No need to get torso initial pose.";
    }

    return true;
  }

  bool MobileManipulatorReferenceManager::setLbTimedPosCmdService(kuavo_msgs::lbTimedPosCmd::Request &req, 
                                                                  kuavo_msgs::lbTimedPosCmd::Response &res)
  {
    res.isSuccess = true;
    res.message = "Set Lb Timed Position Command Success.";

    Eigen::VectorXd eigenCmdVec = Eigen::Map<Eigen::VectorXd>(req.cmdVec.data(), req.cmdVec.size());
    // 1. 处理不同的 planner_index
    switch (static_cast<LbTimedPosCmdType>(req.planner_index)) 
    {
      case LbTimedPosCmdType::BASE_POS_WORLD_CMD:
        eigenCmdVec[2] = targetYawPreProcess(initState_[2], eigenCmdVec[2]);
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Base World Pose Command.");
        break;
      case LbTimedPosCmdType::BASE_POS_LOCAL_CMD:
      {
        // 1. 获取当前的位姿
        Eigen::Vector2d currentPos = initState_.head(2);  // 当前世界系位置 [x, y]
        scalar_t currentYaw = initState_[2];              // 当前世界系偏航角
        // 2. 构建世界系的位姿增量
        Eigen::Matrix2d currentRot = Eigen::Rotation2D<scalar_t>(initState_[2]).toRotationMatrix();
        Eigen::Vector2d displacementWorld = currentRot * eigenCmdVec.head(2);
        // 3. 进行增量处理
        eigenCmdVec[0] = currentPos[0] + displacementWorld[0];
        eigenCmdVec[1] = currentPos[1] + displacementWorld[1];
        eigenCmdVec[2] = currentYaw + eigenCmdVec[2];
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Base Local Pose Command.");
        break;
      }
      case LbTimedPosCmdType::TORSO_POSE_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Torso Pose Command.");
        break;
      case LbTimedPosCmdType::LEG_JOINT_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Leg Joint Command.");
        break;
      case LbTimedPosCmdType::LEFT_ARM_WORLD_CMD:
        eigenCmdVec[3] = targetYawPreProcess(initState_[2], eigenCmdVec[3]);  // 世界系指令的yaw需要处理成多圈
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Left Arm World Frame EE Command.");
        break;
      case LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD:
        eigenCmdVec[3] = targetYawPreProcess(initState_[2], eigenCmdVec[3]);  // 世界系指令的yaw需要处理成多圈
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Right Arm World Frame EE Command.");
        break;
      case LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Left Arm Local Frame EE Command.");
        break;
      case LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Right Arm Local Frame EE Command.");
        break;
      case LbTimedPosCmdType::LEFT_ARM_JOINT_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Left Arm Joint Command.");
        break;
      case LbTimedPosCmdType::RIGHT_ARM_JOINT_CMD:
        ROS_INFO_STREAM("[setLbTimedPosCmdService] Setting Right Arm Joint Command.");
        break;
      default:
        res.isSuccess = false;
        res.message = "Invalid planner index.";
        ROS_ERROR_STREAM("[setLbTimedPosCmdService] " + res.message);
        return true;
    }
    
    // 调用对应的轨迹规划器计算所需时间
    double desiredTime = timedPlannerScheduler_.calcTimedTrajectory(req.planner_index, eigenCmdVec, req.desireTime);

    if(desiredTime == -1)
    {
      int desiredSize = timedPlannerScheduler_.getTimedPlannerDofNum(req.planner_index);
      res.isSuccess = false;
      std::string strReturn = "Command vector size mismatch. Input size: " 
                              + std::to_string(eigenCmdVec.size()) + 
                              ", required size: " + std::to_string(desiredSize) + ".";
      res.message = strReturn;
      ROS_ERROR_STREAM("[setLbTimedPosCmdService] " + res.message);
      return true;
    }
  
    // 保存对应轨迹和时间
    res.actualTime = desiredTime;
    desireTime_[req.planner_index] = desiredTime;

    timedCmdVecMtx_[req.planner_index]->lock();
    timedCmdVec_[req.planner_index] = Eigen::Map<Eigen::VectorXd>(req.cmdVec.data(), req.cmdVec.size());
    timedCmdVecMtx_[req.planner_index]->unlock();

    isTimedPlannerUpdated_[req.planner_index] = true;

    isUpdateTimedTarget_ = true;  // 用于对齐后续多指令更新的同步方法

    return true;
  }

  bool MobileManipulatorReferenceManager::setLbMultiTimedPosCmdService(kuavo_msgs::lbMultiTimedPosCmd::Request &req, 
                                                                       kuavo_msgs::lbMultiTimedPosCmd::Response &res)
  {
      res.isSuccess = true;
      res.message = "Set Lb Timed Position Command Success.";
      res.actualTime = 0.0;  // 初始化为0

      // 先判断主循环是否正在更新指令, 是则返回失败
      if( isUpdateTimedTarget_ == true)
      {
        res.isSuccess = false;
        res.message = "main loop is updating the command and is busy.";
        res.actualTime = -1;  // 表示不执行
        return true;
      }

      // 检查是否有重复的规划器索引 - 使用最快的哈希表
      std::unordered_set<int> planner_indices_set;
      planner_indices_set.reserve(req.timedCmdVec.size());  // 预分配空间，避免重哈希
      
      for (const auto& timedCmd: req.timedCmdVec)
      {
        // 如果插入失败，说明已经存在相同索引
        if (!planner_indices_set.insert(timedCmd.planner_index).second)
        {
          res.isSuccess = false;
          res.message = "Duplicate planner index detected: " + std::to_string(timedCmd.planner_index) + 
                       ". Each planner index can only appear once.";
          res.actualTime = -1;
          ROS_ERROR_STREAM("[setLbMultiTimedPosCmdService] " + res.message);
          return true;
        }
      }

      // 遍历所有下发的指令
      for (const auto& timedCmd: req.timedCmdVec)
      {
        Eigen::VectorXd eigenCmdVec = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(timedCmd.cmdVec.data()), timedCmd.cmdVec.size());

        // 1. 处理不同的 planner_index
        switch (static_cast<LbTimedPosCmdType>(timedCmd.planner_index)) 
        {
          case LbTimedPosCmdType::BASE_POS_WORLD_CMD:
            eigenCmdVec[2] = targetYawPreProcess(initState_[2], eigenCmdVec[2]);
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Base World Pose Command.");
            break;
          case LbTimedPosCmdType::BASE_POS_LOCAL_CMD:
          {
            // 1. 获取当前的位姿
            Eigen::Vector2d currentPos = initState_.head(2);  // 当前世界系位置 [x, y]
            scalar_t currentYaw = initState_[2];              // 当前世界系偏航角
            // 2. 构建世界系的位姿增量
            Eigen::Matrix2d currentRot = Eigen::Rotation2D<scalar_t>(initState_[2]).toRotationMatrix();
            Eigen::Vector2d displacementWorld = currentRot * eigenCmdVec.head(2);
            // 3. 进行增量处理
            eigenCmdVec[0] = currentPos[0] + displacementWorld[0];
            eigenCmdVec[1] = currentPos[1] + displacementWorld[1];
            eigenCmdVec[2] = currentYaw + eigenCmdVec[2];
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Base Local Pose Command.");
            break;
          }
          case LbTimedPosCmdType::TORSO_POSE_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Torso Pose Command.");
            break;
          case LbTimedPosCmdType::LEG_JOINT_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Leg Joint Command.");
            break;
          case LbTimedPosCmdType::LEFT_ARM_WORLD_CMD:
            eigenCmdVec[3] = targetYawPreProcess(initState_[2], eigenCmdVec[3]);  // 世界系指令的yaw需要处理成多圈
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Left Arm World Frame EE Command.");
            break;
          case LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD:
            eigenCmdVec[3] = targetYawPreProcess(initState_[2], eigenCmdVec[3]);  // 世界系指令的yaw需要处理成多圈
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Right Arm World Frame EE Command.");
            break;
          case LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Left Arm Local Frame EE Command.");
            break;
          case LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Right Arm Local Frame EE Command.");
            break;
          case LbTimedPosCmdType::LEFT_ARM_JOINT_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Left Arm Joint Command.");
            break;
          case LbTimedPosCmdType::RIGHT_ARM_JOINT_CMD:
            ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] Setting Right Arm Joint Command.");
            break;
          default:
            res.isSuccess = false;
            res.message = "Invalid planner index for one of the commands.";
            ROS_ERROR_STREAM("[setLbMultiTimedPosCmdService] " + res.message);
            return true;
        }

        // 调用对应的轨迹规划器计算所需时间
        double desiredTime = timedPlannerScheduler_.calcTimedTrajectory(timedCmd.planner_index, eigenCmdVec, timedCmd.desireTime);

        if(desiredTime == -1)
        {
          int desiredSize = timedPlannerScheduler_.getTimedPlannerDofNum(timedCmd.planner_index);
          res.isSuccess = false;
          std::string strReturn = "Command vector size mismatch for planner " 
                                  + std::to_string(timedCmd.planner_index) + ". Input size: " 
                                  + std::to_string(eigenCmdVec.size()) + 
                                  ", required size: " + std::to_string(desiredSize) + ".";
          res.message = strReturn;
          ROS_ERROR_STREAM("[setLbMultiTimedPosCmdService] " + res.message);
          return true;
        }

        // 更新最长时间
        if (desiredTime > res.actualTime) 
        {
          res.actualTime = desiredTime;
        }

        // 保存对应轨迹和时间
        desireTime_[timedCmd.planner_index] = desiredTime;

        timedCmdVecMtx_[timedCmd.planner_index]->lock();
        timedCmdVec_[timedCmd.planner_index] = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(timedCmd.cmdVec.data()), timedCmd.cmdVec.size());
        timedCmdVecMtx_[timedCmd.planner_index]->unlock();
        
        isTimedPlannerUpdated_[timedCmd.planner_index] = true;
      }

      // 如果设置了同步判断, 则修改执行时间来保证同步
      if(req.isSync == true)
      {
        for (const auto& timedCmd: req.timedCmdVec)
        {
          desireTime_[timedCmd.planner_index] = res.actualTime;
        }
      }

      // 触发响应
      if(isUpdateTimedTarget_ != true) isUpdateTimedTarget_ = true;

      ROS_INFO_STREAM("[setLbMultiTimedPosCmdService] All commands processed. Max execution time: " << res.actualTime);
      return true;
  }

  bool MobileManipulatorReferenceManager::setLbMultiTimedOfflineTrajService(kuavo_msgs::lbMultiTimedOfflineTraj::Request &req, 
                                                                            kuavo_msgs::lbMultiTimedOfflineTraj::Response &res)
  {
    // 初始化响应
    res.isSuccess = false;
    res.message = "Successfully processed all trajectories";

    if (req.offlineTraj.empty())   // 判断请求合法
    {
      res.message = "Invalid request: empty or malformed trajectory data";
      ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
      return true;  // 服务调用成功，但处理失败
    }

    size_t trajCount = req.offlineTraj.size();
    ROS_INFO_STREAM("[setLbMultiTimedOfflineTrajService] Processing " << trajCount << " offline trajectories");

    for (size_t i = 0; i < trajCount; ++i)
    {
      // 获取当前轨迹
      const auto& offlineTraj = req.offlineTraj[i];
      const auto& timedTraj = offlineTraj.timedTraj;

      // 判断是否合法
      if(req.offlineTraj[i].timedTraj.empty())
      {
        res.message = "Invalid request: empty or malformed timedTraj " + std::to_string(i);
        ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
        return true;
      }
      
      size_t trajNum = req.offlineTraj[i].timedTraj.size();
      ROS_INFO_STREAM("[setLbMultiTimedOfflineTrajService] Processing trajectory " << i << " with " << trajNum << " points");

      // 遍历当前轨迹的每个点
      for(size_t j = 0; j < trajNum; j++)
      {
        const auto& timedCmd = timedTraj[j];
        Eigen::VectorXd eigenCmdVec = Eigen::Map<const Eigen::VectorXd>(timedCmd.cmdVec.data(), timedCmd.cmdVec.size());

        // 对时间第一帧判断是否等于0, 浮点型需要近似等于
        if(j == 0 && std::fabs(timedCmd.desireTime) > 1e-6)
        {
          res.message = "Invalid trajectory: first time is not 0";
          ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
          return true;
        }
        // 对时间进行递增校验
        if(j > 0 && timedCmd.desireTime <= timedTraj[j-1].desireTime)
        {
          res.message = "Invalid trajectory: time is not strictly increasing";
          ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
          return true;
        }

        // 对 cmdVec 长度进行校验，左右臂轨迹为 6, 躯干轨迹为 4
        if(offlineTraj.plannerIndex == 0 || offlineTraj.plannerIndex == 1) // 左右臂轨迹
        {
          if(eigenCmdVec.size() != 6)
          {
            res.message = "Invalid trajectory: cmdVec size is not 6 for left or right arm";
            ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
            return true;
          }
          int index = offlineTraj.plannerIndex;
          isArmEeOfflineTrajUpdate_[index] = true;
          eeOfflineTrajFrame_[index] = offlineTraj.frame;
          armEeOfflineTraj_[index].timeTrajectory.push_back(timedCmd.desireTime);
          armEeOfflineTraj_[index].stateTrajectory.push_back(eigenCmdVec);
        }
        else if(offlineTraj.plannerIndex == 2) // 躯干轨迹
        {
          if(eigenCmdVec.size() != 4)
          {
            res.message = "Invalid trajectory: cmdVec size is not 4 for torso";
            ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
            return true;
          }
          isTorsoOfflineTrajUpdate_ = true;
          torsoOfflineTraj_.timeTrajectory.push_back(timedCmd.desireTime);
          torsoOfflineTraj_.stateTrajectory.push_back(eigenCmdVec);
        }
        else
        {
          res.message = "Invalid planner index for trajectory " + std::to_string(i);
          ROS_ERROR_STREAM("[setLbMultiTimedOfflineTrajService] " + res.message);
          return true;
        }
      }
      
      ROS_INFO_STREAM("[setLbMultiTimedOfflineTrajService] trajectory have been load");
    }

    res.isSuccess = true;
    return true;
  }

  bool MobileManipulatorReferenceManager::setLbOfflineTrajEnableService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    res.success = true;

    if(req.data == false)
    {
      offlineTrajDisable_ = true;
      res.message = "Offline trajectory disable successfully";
      ROS_INFO_STREAM("[setLbOfflineTrajEnableService] " + res.message);
    }

    if(req.data == true)
    {
      offlineTrajDisable_ = false;
      trajFrameUpdate_ = true;
      res.message = "Offline trajectory enable successfully";
      ROS_INFO_STREAM("[setLbOfflineTrajEnableService] " + res.message);
    }
    
    return true;
  }

  bool MobileManipulatorReferenceManager::setLbResetTorsoService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if(req.data)
    {
      offlineTrajDisable_ = true;
      /***********************根据期望速度, 设置切换时间************************/
      vector_t torsoPose = vector_t::Zero(6);
      getCurrentTorsoPoseInBasePitchYaw(torsoPose, initState_);
      Eigen::VectorXd err = Eigen::VectorXd::Zero(6);
      err[0] = std::fabs(initialTorsoPos_[0] - torsoPose[0]);
      err[2] = std::fabs(initialTorsoPos_[2] - torsoPose[2]);
      err[3] = std::fabs(torsoPose[3] - 0.0);
      err[4] = std::fabs(torsoPose[4] - 0.0);
      Eigen::ArrayXd validTimes = err.array() / torsoResetMaxVel_.array();
      double actualTime = validTimes.maxCoeff();
      /*********************************************************************/
      res.success = true;
      res.message = std::to_string(actualTime);  // 直接转换
      isResetTorso_ = true;
      return true;
    }

    res.success = false;
    res.message = "0.0";
    return true;
  }

  void MobileManipulatorReferenceManager::getCurrentTorsoPoseInBase(vector_t& torsoPose, const vector_t& initState)
  {
    assert(torsoPose.size() == 7 && "torsoPose dimension must be 7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取躯干坐标系帧ID
    pinocchio::FrameIndex torsoFrameId = model.getFrameId(info_.torsoFrame);
    // 获取躯干在世界坐标系中的位姿
    const pinocchio::SE3& worldToTorso = data.oMf[torsoFrameId];

    // 获取基坐标系帧ID
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    // 获取基座在世界坐标系中的位姿
    const pinocchio::SE3& worldToBase = data.oMf[baseFrameId];

    // 计算躯干在基坐标系中的位姿: baseToTorso = worldToBase.inverse() * worldToTorso
    pinocchio::SE3 baseToTorso = worldToBase.actInv(worldToTorso);

    torsoPose.segment<3>(0) = baseToTorso.translation();
    torsoPose.segment<4>(3) = Eigen::Quaterniond(baseToTorso.rotation()).coeffs();
  }

  void MobileManipulatorReferenceManager::getCurrentEeWorldPoseContinuous(vector_t& EeState, const vector_t& initState)
  {
    // 输出6D位姿: [x, y, z, yaw, pitch, roll] (ZYX顺序)
    assert(EeState.size() == info_.eeFrames.size() * 6 && "EeState dimension must be info_.eeFrames.size()*6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取基座的世界系位姿
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    const pinocchio::SE3& base_pose_world = data.oMf[baseFrameId];

    // 计算世界到基座的变换矩阵
    pinocchio::SE3 world_to_base = base_pose_world.inverse();

    // 初始化连续欧拉角跟踪器
    static std::vector<ContinuousEulerAnglesFromMatrix> eeBaseUnwrappers(info_.eeFrames.size());

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose_world = data.oMf[frameId];

      // 将末端位姿从世界坐标系转换到基座坐标系
      pinocchio::SE3 ee_pose_base = world_to_base * ee_pose_world;

      EeState.segment<3>(ee_idx*6) = ee_pose_world.translation();

      // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
      Eigen::Vector3d eulerZyx = eeBaseUnwrappers[ee_idx].update(ee_pose_base.rotation());
      eulerZyx[0] += initState_[2];
      EeState.segment<3>(ee_idx*6+3) = eulerZyx;
    }
  }

  void MobileManipulatorReferenceManager::getCurrentEeBasePoseContinuous(vector_t& EeState, const vector_t& initState)
  {
    // 输出6D位姿: [x, y, z, yaw, pitch, roll] (ZYX顺序，基座坐标系下)
    assert(EeState.size() == info_.eeFrames.size() * 6 && "EeState dimension must be info_.eeFrames.size()*6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取基座的世界系位姿
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    const pinocchio::SE3& base_pose_world = data.oMf[baseFrameId];

    // 计算世界到基座的变换矩阵
    pinocchio::SE3 world_to_base = base_pose_world.inverse();

    // 初始化连续欧拉角跟踪器
    static std::vector<ContinuousEulerAnglesFromMatrix> eeBaseUnwrappers(info_.eeFrames.size());

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose_world = data.oMf[frameId];

      // 将末端位姿从世界坐标系转换到基座坐标系
      pinocchio::SE3 ee_pose_base = world_to_base * ee_pose_world;

      EeState.segment<3>(ee_idx*6) = ee_pose_base.translation();

      // 使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
      Eigen::Vector3d eulerZyx = eeBaseUnwrappers[ee_idx].update(ee_pose_base.rotation());
      EeState.segment<3>(ee_idx*6+3) = eulerZyx;
    }
  }

  void MobileManipulatorReferenceManager::getCurrentTorsoPoseInBaseContinuous(vector_t& torsoPose, const vector_t& initState)
  {
    assert(torsoPose.size() == 6 && "torsoPose dimension must be 6!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取躯干坐标系帧ID
    pinocchio::FrameIndex torsoFrameId = model.getFrameId(info_.torsoFrame);
    // 获取躯干在世界坐标系中的位姿
    const pinocchio::SE3& worldToTorso = data.oMf[torsoFrameId];

    // 获取基坐标系帧ID
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    // 获取基座在世界坐标系中的位姿
    const pinocchio::SE3& worldToBase = data.oMf[baseFrameId];

    // 计算躯干在基坐标系中的位姿: baseToTorso = worldToBase.inverse() * worldToTorso
    pinocchio::SE3 baseToTorso = worldToBase.actInv(worldToTorso);

    torsoPose.segment<3>(0) = baseToTorso.translation();
    
    // 姿态：使用连续欧拉角跟踪器获取无跳变的ZYX欧拉角
    static ContinuousEulerAnglesFromMatrix torsoUnwrapper;
    Eigen::Vector3d eulerZyx = torsoUnwrapper.update(baseToTorso.rotation());
    torsoPose.segment<3>(3) = eulerZyx;  // yaw, pitch, roll
  }

  void MobileManipulatorReferenceManager::getCurrentTorsoPoseInBasePitchYaw(vector_t& torsoPose, const vector_t& initState)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取躯干坐标系帧ID
    pinocchio::FrameIndex torsoFrameId = model.getFrameId(info_.torsoFrame);
    // 获取躯干在世界坐标系中的位姿
    const pinocchio::SE3& worldToTorso = data.oMf[torsoFrameId];

    // 获取基坐标系帧ID
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    // 获取基座在世界坐标系中的位姿
    const pinocchio::SE3& worldToBase = data.oMf[baseFrameId];

    // 计算躯干在基坐标系中的位姿: baseToTorso = worldToBase.inverse() * worldToTorso
    pinocchio::SE3 baseToTorso = worldToBase.actInv(worldToTorso);

    torsoPose.segment<3>(0) = baseToTorso.translation();

    std::pair<double, double> pitch_yaw = rotationMatrixToPitchYaw(baseToTorso.rotation());

    torsoPose[3] = pitch_yaw.second;  // 按 zyx 表达
    torsoPose[4] = pitch_yaw.first;
    torsoPose[5] = 0.0;
  }

  void MobileManipulatorReferenceManager::publishMultiPointPose_World(const vector_t& initState)
  {
    Eigen::VectorXd eeState_world, d_eeState_world, dd_eeState_world;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD, 
                                                 eeState_world, d_eeState_world, dd_eeState_world);

    ros_logger_->publishVector("/mobile_manipulator/eeStateWorld_initState/left", eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateWorld_initState/left", d_eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateWorld_initState/left", dd_eeState_world);

    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD, 
                                                 eeState_world, d_eeState_world, dd_eeState_world);

    ros_logger_->publishVector("/mobile_manipulator/eeStateWorld_initState/right", eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateWorld_initState/right", d_eeState_world);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateWorld_initState/right", dd_eeState_world);
  }

  void MobileManipulatorReferenceManager::publishMultiPointPose_Local(const vector_t& initState)
  {
    vector_t eeState_local, d_eeState_local, dd_eeState_local;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD, 
                                                 eeState_local, d_eeState_local, dd_eeState_local);

    ros_logger_->publishVector("/mobile_manipulator/eeStateLocal_initState/left", eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateLocal_initState/left", d_eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateLocal_initState/left", dd_eeState_local);

    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD, 
                                                 eeState_local, d_eeState_local, dd_eeState_local);

    ros_logger_->publishVector("/mobile_manipulator/eeStateLocal_initState/right", eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/d_eeStateLocal_initState/right", d_eeState_local);
    ros_logger_->publishVector("/mobile_manipulator/dd_eeStateLocal_initState/right", dd_eeState_local);
    /********************************************************************************************************************/
    
    vector_t torsoState_local, d_torsoState_local, dd_torsoState_local;
    timedPlannerScheduler_.getTimedPlannerStates(LbTimedPosCmdType::TORSO_POSE_CMD, 
                              torsoState_local, d_torsoState_local, dd_torsoState_local);
    
    ros_logger_->publishVector("/mobile_manipulator/torsoStateLocal_initState", torsoState_local);
    ros_logger_->publishVector("/mobile_manipulator/d_torsoStateLocal_initState", d_torsoState_local);
    ros_logger_->publishVector("/mobile_manipulator/dd_torsoStateLocal_initState", dd_torsoState_local);
  }

  void MobileManipulatorReferenceManager::getCurrentEeWorldPose(vector_t& EeState, const vector_t& initState)
  {
    assert(EeState.size() == info_.eeFrames.size() * 7 && "EeState dimension must be info_.eeFrames.size()*7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose = data.oMf[frameId];

      EeState.segment<3>(ee_idx*7) = ee_pose.translation();
      EeState.segment<4>(ee_idx*7+3) = Eigen::Quaterniond(ee_pose.rotation()).coeffs();
    }
  }

  void MobileManipulatorReferenceManager::getCurrentEeBasePose(vector_t& EeState, const vector_t& initState)
  {
    assert(EeState.size() == info_.eeFrames.size() * 7 && "EeState dimension must be info_.eeFrames.size()*7!");

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, initState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    // 获取基座的世界系位姿
    pinocchio::FrameIndex baseFrameId = model.getFrameId(info_.baseFrame);
    const pinocchio::SE3& base_pose_world = data.oMf[baseFrameId];

    // 计算世界到基座的变换矩阵
    pinocchio::SE3 world_to_base = base_pose_world.inverse();

    // 遍历每个末端执行器
    for (size_t ee_idx = 0; ee_idx < info_.eeFrames.size(); ++ee_idx) 
    {
      // 获取末端执行器帧ID（这里需要您根据实际情况获取）
      pinocchio::FrameIndex frameId = model.getFrameId(info_.eeFrames[ee_idx]);
      // 获取末端在世界坐标系中的位姿
      const pinocchio::SE3& ee_pose_world = data.oMf[frameId];

      // 将末端位姿从世界坐标系转换到基座坐标系
      pinocchio::SE3 ee_pose_base = world_to_base * ee_pose_world;

      EeState.segment<3>(ee_idx*7) = ee_pose_base.translation();
      EeState.segment<4>(ee_idx*7+3) = Eigen::Quaterniond(ee_pose_base.rotation()).coeffs();
    }
  }

  // void MobileManipulatorReferenceManager::updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange)
  // {
  //   static bool firstRun{true};
  //   if(isChange || firstRun)
  //   {
  //     setEnableEeTargetTrajectories(true); // 开启末端笛卡尔跟踪
  //     setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
  //     setEnableArmJointTrack(false); // 关闭手臂跟踪
  //     setEnableBaseTrack(false);   // 关闭底盘跟踪

  //     firstRun = false;
  //     // return;
  //   }

  //   // getAllTargetTrajectories(targetTrajectories);
  // }

  // void MobileManipulatorReferenceManager::updateArmOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  // {
  //   if(isChange)
  //   {
  //     vector_t eeState_6d = vector_t::Zero(info_.eeFrames.size() * 6);
  //     getCurrentEeWorldPoseContinuous(eeState_6d, initState);
  //     cmd_arm_zyx_ = eeState_6d;

  //     lb_leg_traj_ = initState.segment(baseDim_, 4);
  //     arm_joint_traj_ = initState.tail(info_.armDim - 4);

  //     resetAllMpcTraj(initTime_, initState_);

  //     // return;
  //   }

  //   if(initTime >= resetTorsoTime_ + resetTorsoInitTime_)
  //   {
  //     setArmControl(initTime, finalTime, initState);
  //     setTorsoControl(initTime, finalTime, initState);
  //   }

  //   generatePoseTargetWithRuckig(initTime, finalTime, ruckigDt_);
  // }

  // void MobileManipulatorReferenceManager::updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  // {
  //   if(isChange)
  //   {
  //     resetAllMpcTraj(initTime_, initState_);
  //     // return;
  //   }

  //   setChassisControl(initTime, finalTime, initState);
  // }

  void MobileManipulatorReferenceManager::updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  {
    static bool isFirstRun = true;
    if(isFirstRun)
    {
      resetAllMpcTrajAndTarget(initTime_, initState_);

      isFirstRun = false;
      // return;
    }

    if(initTime >= resetTorsoTime_ + resetTorsoInitTime_)
    {
      setArmControl(0, initTime, finalTime, initState);
      setArmControl(1, initTime, finalTime, initState);
      setTorsoControl(initTime, finalTime, initState);
    }
    setChassisControl(initTime, finalTime, initState);
  }

  // void MobileManipulatorReferenceManager::updateArmEeOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange)
  // {
  //   if(isChange)
  //   {
  //     resetAllMpcTraj(initTime, initState);

  //     // return;
  //   }

  //   static vector_t armEeTarget = vector_t::Zero(info_.eeFrames.size() * 7); // 双臂末端轨迹
    

  //   if(isCmdDualArmPoseUpdated_)
  //   {
  //     bool isChange = getLbArmControlModeIsChange(desireMode_);
  //     // TODO: 如果 isChange 为 true, 则根据当前模式重置一次初值
  //     if(isChange)  resetDualArmRuckig(initTime, initState, false, desireMode_);
  //     if(desireMode_ == LbArmControlMode::WorldFrame)
  //     {
  //       armPose_mtx_.lock();
  //       armEeTarget = cmd_arm_zyx_;
  //       calcRuckigTrajWithEePose(initTime, armEeTarget, cmdDualArmPoseDesiredTime_);
  //       armPose_mtx_.unlock();
  //     }
  //     isCmdDualArmPoseUpdated_ = false;
  //   }
  //   else
  //   {
  //     resetDualArmRuckig(initTime, initState, false, desireMode_);
  //   }

  //   if(desireMode_ == LbArmControlMode::WorldFrame) // 世界系的笛卡尔末端控制
  //   {
  //     setEnableArmJointTrack(false); // 关闭手臂跟踪
  //     setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
  //     setEnableBaseTrack(false);   // 关闭底盘跟踪
  //     setEnableEeTargetTrajectories(true); // 开启末端笛卡尔
  //     setEnableTorsoPoseTargetTrajectories(true); // 开启躯干笛卡尔

  //     generateDualArmEeTargetWithRuckig(initTime, finalTime, ruckigDt_);
  //   }
  // }

  double MobileManipulatorReferenceManager::targetYawPreProcess(double currentYaw, double targetYaw)
  {
    // 规范化角度到 [-π, π]
    auto normalize = [](double angle) {
        angle = std::fmod(angle, 2.0 * M_PI);
        if (angle > M_PI) angle -= 2.0 * M_PI;
        if (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };

    // 规范化角度用于计算最短路径
    double normalizedCurrent = normalize(currentYaw);
    double normalizedTarget = normalize(targetYaw);

    // std::cout << "Original currentYaw: " << currentYaw << std::endl;
    // std::cout << "Original targetYaw: " << targetYaw << std::endl;
    // std::cout << "Normalized currentYaw: " << normalizedCurrent << std::endl;
    // std::cout << "Normalized targetYaw: " << normalizedTarget << std::endl;
    
    // 计算标准化后的角度差（考虑最短路径）
    double rawDiff = normalizedTarget - normalizedCurrent;
    
    // std::cout << "rawDiff: " << rawDiff << std::endl;
    // 找到最短路径的目标角度
    double bestNormalizedTarget = 0.0;
    if(rawDiff > M_PI)
    {
      // 正转路径太长, 选择反转(减去2π)
      bestNormalizedTarget = normalizedTarget - 2.0 * M_PI;
    }
    else if(rawDiff < -M_PI)
    {
      // 反转路径太长，选择正转(加上2π)
      bestNormalizedTarget = normalizedTarget + 2.0 * M_PI;
    }
    else
    {
      bestNormalizedTarget = normalizedTarget;
    }

    // 计算 currentYaw 所在的圈数
    auto cycle = currentYaw / (2.0 * M_PI);
    int currentCycle = std::round(cycle);
    // std::cout << "Current cycle: " << currentCycle << "[" << cycle << "]" << std::endl;

    // 将目标角度锁定在当前圈数
    double newTargetYaw = currentCycle * 2.0 * M_PI + bestNormalizedTarget;
    // std::cout << "Best normalized target: " << bestNormalizedTarget << std::endl;
    // std::cout << "Cur targetYaw: " << currentYaw << ", New targetYaw: " << newTargetYaw << std::endl;
    // std::cout << "Final rotation: " << (newTargetYaw - currentYaw) << " radians" << std::endl;
    return newTargetYaw;
  }

  void MobileManipulatorReferenceManager::setChassisControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    // 生成底盘指令轨迹（包含速度指令和位置指令处理）
    if(isCmdPoseUpdated_)
    {
      // 计算期望位置
      cmdPose_mtx_.lock();
      Eigen::Vector3d cmdPoseBase = cmdPose_;
      cmdPose_mtx_.unlock();

      // 将本体系的位置期望，转换成世界系的
      // 1. 获取当前的位姿
      Eigen::Vector2d currentPos = initState.head(2);  // 当前世界系位置 [x, y]
      scalar_t currentYaw = initState[2];              // 当前世界系偏航角
      // 2. 构建世界系的位姿增量
      Eigen::Matrix2d currentRot = Eigen::Rotation2D<scalar_t>(initState[2]).toRotationMatrix();
      Eigen::Vector2d displacementWorld = currentRot * cmdPoseBase.head(2);
      // 3. 进行增量处理
      currentCmdPose_[0] = currentPos[0] + displacementWorld[0];
      currentCmdPose_[1] = currentPos[1] + displacementWorld[1];
      currentCmdPose_[2] = currentYaw + cmdPoseBase[2];
    }

    if(isCmdPoseWorldUpdated_)
    {
      // 计算期望位置
      cmdPoseWorld_mtx_.lock();
      currentCmdPose_ = cmdPoseWorld_;
      currentCmdPose_[2] = targetYawPreProcess(initState[2], cmdPoseWorld_[2]);
      cmdPoseWorld_mtx_.unlock();
    }

    if(isCmdPoseUpdated_ || isCmdPoseWorldUpdated_)
    {
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdPose(initTime, currentCmdPose_, cmdPoseDesiredTime_);
      /*****************************************************************************************/

      // 清空位置指令更新标志
      isCmdPoseUpdated_ = false;
      isCmdPoseWorldUpdated_ = false;
    }
    else  // 未收到位置指令
    {
      resetCmdPoseRuckig(initTime, initState, false);
    }

    // 更新速度指令时间，超时0.3s后未更新指令则清0
    if(isCmdVelTimeUpdate_)
    {
      lastCmdVelTime_ = initTime;
      isCmdVelTimeUpdate_ = false;
    }
    if((initTime - lastCmdVelTime_) > 0.3)
    {
      cmdVel_.setZero();
      cmdVelWorld_.setZero();
    }

    if(!isCmdVelUpdated_ && !isCmdVelWorldUpdated_)
    {
      resetCmdVelRuckig(initTime, initState, false);
    }
  
    // 更新速度指令，通过互斥锁维护数据一致性
    if(cmdVel_.isZero(1e-6) && cmdVel_prevTargetVel_.isZero(1e-6))
    {
      isCmdVelUpdated_ = false;
    }
    else
    {
      cmdvel_mtx_.lock();
      currentCmdVel_ = cmdVel_;
      cmdvel_mtx_.unlock();
    }

    if(cmdVelWorld_.isZero(1e-6) && cmdVel_prevTargetVel_.isZero(1e-6))
    {
      isCmdVelWorldUpdated_ = false;
    }
    else
    {
      cmdvelWorld_mtx_.lock();
      currentCmdVelWorld_ = cmdVelWorld_;
      cmdvelWorld_mtx_.unlock();
    }

    // 跟踪上一次使用的速度控制模式，用于检测模式切换
    static bool lastWasCmdVel = false;
    static bool lastWasCmdVelWorld = false;
    if(isCmdVelUpdated_)
    {
      // 如果之前使用的是 cmd_vel_world，现在切换到 cmd_vel，更新标志位
      lastWasCmdVel = true;
      lastWasCmdVelWorld = false;
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdVel(initTime, currentCmdVel_);

      generateVelTargetBaseWithRuckig(initTime, finalTime, ruckigDt_, initState);
      /*****************************************************************************************/

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVel_prevTargetVel_.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelUpdated_ = false;
      }

      resetCmdPoseRuckig(initTime, initState, true);
    }
    else if(isCmdVelWorldUpdated_)
    {
      // 如果之前使用的是 cmd_vel，现在切换到 cmd_vel_world，需要重置 prevTargetPose 为当前实际位置
      if(lastWasCmdVel && !lastWasCmdVelWorld)
      {
        cmdVel_prevTargetPose_ = initState.head(3);  // 重置为当前实际位置，避免返回原点
        cmdVel_prevTargetVel_.setZero(3);
        cmdVel_prevTargetAcc_.setZero(3);
      }
      lastWasCmdVel = false;
      lastWasCmdVelWorld = true;
      /*****************************更新ruckig规划器所需实时数据************************************/
      calcRuckigTrajWithCmdVel(initTime, currentCmdVelWorld_);

      generateVelTargetWithRuckig(initTime, finalTime, ruckigDt_);
      /*****************************************************************************************/

      // 判断速度为0，跳出速度控制
      static int zero_vel_cnt;
      if(zero_vel_cnt < 5)
      {
        if(cmdVel_prevTargetVel_.isZero(1e-6)) zero_vel_cnt++;
        else zero_vel_cnt = 0;
      }
      else
      {
        zero_vel_cnt = 0;
        isCmdVelWorldUpdated_ = false;
      }

      resetCmdPoseRuckig(initTime, initState, true);
    }
    else    // 默认跟踪位置
    {
      generatePoseTargetWithRuckig(initTime, finalTime, ruckigDt_);
    }
  }

  void MobileManipulatorReferenceManager::setArmControl(int armIdx, scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    // 手臂控制模式接受三种收发逻辑（desireMode_用于选择）: 
    // 0. 发手臂末端世界系轨迹; //有些危险, 上层需关注清楚
    // 1. 发手臂末端局部系轨迹;
    // 2. 发手臂关节轨迹;
    
    static vector_t armJointTarget[2] = {vector_t::Zero(singleArmJointDim_), 
                                         vector_t::Zero(singleArmJointDim_)}; // 双臂关节轨迹
    static vector_t armEeTarget[2] = {vector_t::Zero(6), 
                                      vector_t::Zero(6)}; // 双臂末端轨迹
    
    static bool isArmEeOfflineTrajUpdate_prev[2]{false, false};
    static bool armEeOfflineEnd[2]{false, false};

    if(isArmEeOfflineTrajUpdate_prev[armIdx] == true && 
       isArmEeOfflineTrajUpdate_[armIdx] == false)    // 从离线调整为在线的第一次执行, 从离线轨迹最后一帧获取期望
    {
      armEeOfflineEnd[armIdx] = true;
    }
    isArmEeOfflineTrajUpdate_prev[armIdx] = isArmEeOfflineTrajUpdate_[armIdx];

    if(armEeOfflineEnd[armIdx] == true && 
      isArmEeOfflineTrajUpdate_[armIdx] != true && isOfflineTrajUpdate_ != true)
    {
      cmd_arm_zyx_[armIdx] = eeTargetTrajectories_[armIdx].getDesiredState(initTime);
      cmdDualArm_prevTargetPose_[armIdx] = cmd_arm_zyx_[armIdx];
      cmdDualArm_prevTargetVel_[armIdx] = vector_t::Zero(cmd_arm_zyx_[armIdx].size());
      cmdDualArm_prevTargetAcc_[armIdx] = vector_t::Zero(cmd_arm_zyx_[armIdx].size());
      calcRuckigTrajWithEePose(armIdx, initTime, cmd_arm_zyx_[armIdx], 0.0);
      resetArmJointRuckig(armIdx, initTime, initState, false);
      armEeOfflineEnd[armIdx] = false;
    }

    if(isCmdDualArmPoseUpdated_[armIdx] && isArmEeOfflineTrajUpdate_[armIdx] == false)
    {
      bool isChange = getLbArmControlModeIsChange(armIdx, desireMode_[armIdx]);
      // TODO: 如果 isChange 为 true, 则根据当前模式重置一次初值
      armPose_mtx_[armIdx].lock();
      armEeTarget[armIdx] = cmd_arm_zyx_[armIdx];
      armPose_mtx_[armIdx].unlock();

      if(isChange)  resetDualArmRuckig(armIdx, initTime, initState, false, desireMode_[armIdx], armEeTarget[armIdx]);
      if(desireMode_[armIdx] == LbArmControlMode::WorldFrame || desireMode_[armIdx] == LbArmControlMode::LocalFrame)
      {
        calcRuckigTrajWithEePose(armIdx, initTime, armEeTarget[armIdx], cmdDualArmPoseDesiredTime_[armIdx]);
      }
      isCmdDualArmPoseUpdated_[armIdx] = false;
    }

    if(isCmdArmJointUpdated_[armIdx] && isArmEeOfflineTrajUpdate_[armIdx] == false)
    {
      bool isChange = getLbArmControlModeIsChange(armIdx, desireMode_[armIdx]);
      if(isChange && desireMode_[armIdx] == LbArmControlMode::JointSpace)
      {
        resetArmJointRuckig(armIdx, initTime, initState, false);
      }
    }
    
    if(currentArmControlMode_  == LbArmControlServiceMode::EXTERN_CONTROL)
    {
      if(desireMode_[armIdx] == LbArmControlMode::WorldFrame) // 世界系的笛卡尔末端控制
      {
        if(isArmEeOfflineTrajUpdate_[armIdx] != true && isOfflineTrajUpdate_ != true)
        {
          setEnableArmJointTrackForArm(armIdx, false); // 关闭手臂跟踪
          setEnableEeTargetLocalTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔局部跟踪
          setEnableEeTargetTrajectoriesForArm(armIdx, true); // 开启末端笛卡尔

          generateDualArmEeTargetWithRuckig(armIdx, initTime, finalTime, ruckigDt_);
        }
      }
      else if(desireMode_[armIdx] == LbArmControlMode::LocalFrame)  // 局部系的笛卡尔末端控制
      {
        if(isArmEeOfflineTrajUpdate_[armIdx] != true && isOfflineTrajUpdate_ != true)
        {
          setEnableArmJointTrackForArm(armIdx, false); // 关闭手臂跟踪
          setEnableEeTargetTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔跟踪
          setEnableEeTargetLocalTrajectoriesForArm(armIdx, true); // 开启末端笛卡尔局部跟踪
      
          generateDualArmEeTargetWithRuckig(armIdx, initTime, finalTime, ruckigDt_);
        }
      }
      else if(desireMode_[armIdx] == LbArmControlMode::JointSpace)  // 关节控制
      {
        if(isOfflineTrajUpdate_ != true)
        {
          setEnableEeTargetTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔跟踪
          setEnableEeTargetLocalTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔局部跟踪
          setEnableArmJointTrackForArm(armIdx, true); // 开启手臂跟踪

          armJoint_mtx_[armIdx].lock();
          armJointTarget[armIdx] = arm_joint_traj_[armIdx];
          armJoint_mtx_[armIdx].unlock();

          if(isCmdArmJointUpdated_[armIdx])
          {
            calcRuckigTrajWithArmJoint(armIdx, initTime, armJointTarget[armIdx], cmdArmJointDesiredTime_[armIdx]);
            isCmdArmJointUpdated_[armIdx] = false;
          }
        }

        // resetLegJointRuckig(initTime, initState, false);  // 笛卡尔控制影响下肢关节, 重置关节轨迹初值
      }
    }
    else if(currentArmControlMode_  == LbArmControlServiceMode::KEEP)
    {
      if(isOfflineTrajUpdate_ != true)
      {
        setEnableEeTargetTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔跟踪
        setEnableEeTargetLocalTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔局部跟踪
        setEnableArmJointTrackForArm(armIdx, true); // 开启手臂跟踪

        armJointTarget[armIdx] = initState.tail(info_.armDim - 4).segment(armIdx * singleArmJointDim_, singleArmJointDim_);

        if(isCmdArmJointUpdated_[armIdx])
        {
          cmdArmJointDesiredTime_[armIdx] = 0.0;
          calcRuckigTrajWithArmJoint(armIdx, initTime, armJointTarget[armIdx], cmdArmJointDesiredTime_[armIdx]);
          isCmdArmJointUpdated_[armIdx] = false;
        }
      }
    }
    else if(currentArmControlMode_ == LbArmControlServiceMode::AUTO_SWING)
    {
      if(isOfflineTrajUpdate_ != true)
      {
        setEnableEeTargetTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔跟踪
        setEnableEeTargetLocalTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔局部跟踪
        setEnableArmJointTrackForArm(armIdx, true); // 开启手臂跟踪
        
        isArmEeOfflineTrajUpdate_[armIdx] = false;
        armJointTarget[armIdx] = arm_init_joint_traj_.segment(armIdx * singleArmJointDim_, singleArmJointDim_);
        arm_joint_traj_[armIdx] = arm_init_joint_traj_.segment(armIdx * singleArmJointDim_, singleArmJointDim_);
        
        if(isCmdArmJointUpdated_[armIdx])
        {
          cmdArmJointDesiredTime_[armIdx] = 1.0;
          calcRuckigTrajWithArmJoint(armIdx, initTime, armJointTarget[armIdx], cmdArmJointDesiredTime_[armIdx]);
          isCmdArmJointUpdated_[armIdx] = false;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("[MobileManipulatorReferenceManager] 错误设置 currentArmControlMode_");
    }
  }

  void MobileManipulatorReferenceManager::resetTorsoControlPoseWithRuckig(scalar_t initTime, const vector_t& initState)
  {
    setEnableLegJointTrack(false); // 关闭下肢关节跟踪
    isCmdLegJointUpdated_ = false;  // 关闭关节控制标志位
    setEnableTorsoPoseTargetTrajectories(true); // 开启躯干

    vector_t resetPose = vector_t::Zero(4);
    resetPose[0] = initialTorsoPos_[0];
    resetPose[1] = initialTorsoPos_[2];
    resetTorsoPoseRuckig(initTime, initState, false);

    /*************************根据期望速度, 设置切换时间*******************************/
    vector_t torsoPose = vector_t::Zero(6);
    getCurrentTorsoPoseInBasePitchYaw(torsoPose, initState);
    Eigen::VectorXd err = Eigen::VectorXd::Zero(6);
    err[0] = std::fabs(initialTorsoPos_[0] - torsoPose[0]);
    err[2] = std::fabs(initialTorsoPos_[2] - torsoPose[2]);
    err[3] = std::fabs(torsoPose[3] - 0.0);
    err[4] = std::fabs(torsoPose[4] - 0.0);

    Eigen::ArrayXd validTimes = err.array() / torsoResetMaxVel_.array();
    resetTorsoTime_ = validTimes.maxCoeff();
    resetTorsoInitTime_ = initTime;
    ROS_INFO_STREAM("[MobileManipulatorReferenceManager] reset torso require time: " << resetTorsoTime_ << " s, err: " << err.transpose());
    /*****************************************************************************/

    calcRuckigTrajWithTorsoPose(initTime, resetPose, resetTorsoTime_);

    cmdTorsoPose_.setZero();
    cmdTorsoPose_[0] = initialTorsoPos_[0];
    cmdTorsoPose_[1] = initialTorsoPos_[1];
    cmdTorsoPose_[2] = initialTorsoPos_[2];
    cmdTorsoPose_[3] = 0;
    cmdTorsoPose_[4] = 0;

    generateTorsoPoseTargetWithRuckig(initTime, initTime + resetTorsoTime_ + 0.5, ruckigDt_);
  }
  
  void MobileManipulatorReferenceManager::setTorsoControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState)
  {
    static bool isTorsoOfflineTrajUpdate_prev{false};
    static bool torsoOfflineEnd = false;

    if(isTorsoOfflineTrajUpdate_prev == true && 
       isTorsoOfflineTrajUpdate_ == false)    // 从离线调整为在线的第一次执行, 从离线轨迹最后一帧获取期望
    {
      torsoOfflineEnd = true;
    }
    isTorsoOfflineTrajUpdate_prev = isTorsoOfflineTrajUpdate_;

    if(torsoOfflineEnd == true && 
      isTorsoOfflineTrajUpdate_ != true && isOfflineTrajUpdate_ != true)
    {
      cmdTorsoPose_ = torsoTargetTrajectories_.getDesiredState(initTime);
      vector_t torsoTarget4Dof = vector_t::Zero(4);
      torsoTarget4Dof << cmdTorsoPose_[0],  // x, z, yaw, pitch
                         cmdTorsoPose_[2], 
                         cmdTorsoPose_[3], 
                         cmdTorsoPose_[4];

      torsoPose_prevTargetPose_ = torsoTarget4Dof;
      torsoPose_prevTargetVel_ = vector_t::Zero(torsoTarget4Dof.size());
      torsoPose_prevTargetAcc_ = vector_t::Zero(torsoTarget4Dof.size());
      calcRuckigTrajWithTorsoPose(initTime, torsoTarget4Dof, 0.0);

      torsoModeFlag_ = true;
      torsoOfflineEnd = false;
    }
    static vector_t torsoTargetPose = vector_t::Zero(6);

    if(isCmdTorsoPoseUpdated_ && isTorsoOfflineTrajUpdate_ != true)
    {
      // resetTorsoPoseRuckig(initTime, initState, false);

      std::cout << "[MobileManipulatorReferenceManager] 进入躯干笛卡尔控制 " << std::endl;

      cmdTorsoPose_mtx_.lock();
      torsoTargetPose = cmdTorsoPose_;
      cmdTorsoPose_mtx_.unlock();

      vector_t torsoPose4Dof = vector_t::Zero(4);
      torsoPose4Dof << torsoTargetPose[0],  // x, z, yaw, pitch
                       torsoTargetPose[2], 
                       torsoTargetPose[3], 
                       torsoTargetPose[4];

      calcRuckigTrajWithTorsoPose(initTime, torsoPose4Dof, cmdTorsoPoseDesiredTime_);

      isCmdTorsoPoseUpdated_ = false;
      torsoModeFlag_ = true;
    }

    if(isCmdLegJointUpdated_ && isTorsoOfflineTrajUpdate_ != true)
    {
      setEnableLegJointTrack(true); // 开启下肢关节跟踪
      setEnableTorsoPoseTargetTrajectories(false); // 关闭躯干

      static vector_t legJointTarget = vector_t::Zero(4); // 下肢关节轨迹

      lbLegJoint_mtx_.lock();
      legJointTarget = lb_leg_traj_;
      lbLegJoint_mtx_.unlock();

      calcRuckigTrajWithLegJoint(initTime, legJointTarget, cmdLegJointDesiredTime_);

      isCmdLegJointUpdated_ = false;
      torsoModeFlag_ = false;
    }
    
    if(torsoModeFlag_)
    {
      if(isTorsoOfflineTrajUpdate_ != true && isOfflineTrajUpdate_ != true)
      {
        setEnableLegJointTrack(false); // 关闭下肢关节跟踪
        setEnableTorsoPoseTargetTrajectories(true); // 开启躯干

        generateTorsoPoseTargetWithRuckig(initTime, finalTime, ruckigDt_);
      }

      resetLegJointRuckig(initTime, initState, false); // 躯干笛卡尔控制影响下肢关节, 重置关节轨迹初值
    }
    else
    {
      resetTorsoPoseRuckig(initTime, initState, false);
    }
  }

  void MobileManipulatorReferenceManager::resetAllMpcTraj(scalar_t initTime, const vector_t& initState)
  {
    // 默认以 baseArm 关节控制切换
      setEnableEeTargetTrajectories(false); // 关闭末端笛卡尔跟踪
      setEnableEeTargetLocalTrajectories(false); // 关闭末端笛卡尔局部跟踪
      setEnableArmJointTrack(true); // 关闭手臂跟踪
      setEnableBaseTrack(true);   // 关闭底盘跟踪

      resetTorsoControlPoseWithRuckig(initTime, initState); // 重置躯干位置

      resetCmdPoseRuckig(initTime, initState, true);    // 重置底盘轨迹
      resetLegJointRuckig(initTime, initState, true);   // 重置下肢关节轨迹
      for(int i = 0; i < 2; ++i)
      {
        resetArmJointRuckig(i, initTime, initState, true);   // 重置上肢关节轨迹
        resetDualArmRuckig(i, initTime, initState, true, desireMode_[i]);
        desireMode_[i] = LbArmControlMode::JointSpace;
      }
  }

  void MobileManipulatorReferenceManager::resetAllMpcTrajAndTarget(scalar_t initTime, const vector_t& initState)
  {
    vector_t eeState_6d = vector_t::Zero(info_.eeFrames.size() * 6);
    getCurrentEeWorldPoseContinuous(eeState_6d, initState);
    for(int i = 0; i < info_.eeFrames.size(); i++)
    {
      cmd_arm_zyx_[i].head(6) = eeState_6d.segment(i*6, 6);
    }

    lb_leg_traj_ = initState.segment(baseDim_, 4);

    for(int i = 0; i < 2; ++i)
    {
      arm_joint_traj_[i] = initState.tail(info_.armDim - 4).segment(i * singleArmJointDim_, singleArmJointDim_);
    }
      
    resetAllMpcTraj(initTime, initState);
  }

  void MobileManipulatorReferenceManager::updateTimedSchedulerCurrentState(scalar_t initTime, const vector_t& initState)
  {
    std::vector<Eigen::VectorXd> currentPos;

    Eigen::VectorXd tmpPos;

    tmpPos.setZero(baseDim_);                                       // 底盘世界系, 自由度3: x, y, yaw
    tmpPos = initState.head(baseDim_);
    currentPos.push_back(tmpPos);

    tmpPos.setZero(baseDim_);                                       // 底盘局部系, 采用世界系做反馈, 自由度3: x, y, yaw
    tmpPos = initState.head(baseDim_);
    currentPos.push_back(tmpPos);

    vector_t torsoPose = vector_t::Zero(6);                         // 躯干, 自由度4: x, z, yaw, pitch
    getCurrentTorsoPoseInBasePitchYaw(torsoPose, initState);
    tmpPos.setZero(4);   
    tmpPos << torsoPose[0], torsoPose[2], torsoPose[3], torsoPose[4];
    currentPos.push_back(tmpPos);

    tmpPos.setZero(4);                                              // 下肢, 自由度4
    tmpPos = initState.segment(baseDim_, 4);
    currentPos.push_back(tmpPos);
    
    vector_t eePose = vector_t::Zero(info_.eeFrames.size() * 6);    // 双臂世界系, 自由度(末端数*6)
    getCurrentEeWorldPoseContinuous(eePose, initState);
    for(int i=0; i<info_.eeFrames.size(); i++)
    {
      tmpPos.setZero(6);
      tmpPos.head(6) = eePose.segment(i*6, 6);
      currentPos.push_back(tmpPos);
    }

    getCurrentEeBasePoseContinuous(eePose, initState);                        // 双臂局部系, 自由度(末端数*6)
    for(int i=0; i<info_.eeFrames.size(); i++)
    {
      tmpPos.setZero(6);
      tmpPos.head(6) = eePose.segment(i*6, 6);
      currentPos.push_back(tmpPos);
    }

    for(int i = 0; i < 2; ++i) // 上肢单臂设置, 自由度(全身关节数-4)/2
    {
      tmpPos.setZero(singleArmJointDim_);                               
      tmpPos = initState.tail(singleArmJointDim_ * 2).segment(i * singleArmJointDim_, singleArmJointDim_);
      currentPos.push_back(tmpPos);
    }

    timedPlannerScheduler_.setTimedPlannerStates(currentPos);
  }

  void MobileManipulatorReferenceManager::updateTimedSchedulerTargetTraj(void)
  {
    if(isUpdateTimedTarget_ == true)
    {
      for(int i = 0; i < timedPlannerScheduler_.getPlannersNum(); i++)
      {
        if(isTimedPlannerUpdated_[i])
        {
          timedCmdVecMtx_[i]->lock();
          Eigen::VectorXd targetPos = timedCmdVec_[i];
          timedCmdVecMtx_[i]->unlock();

          // double duration = timedPlannerScheduler_.calcTimedTrajectory(i, targetPos, desireTime_[i]);
          updateIndexRuckigPlanner(i, desireTime_[i], targetPos);

          std::cout << "planner " << i << " duration: " << desireTime_[i] << std::endl;
          isTimedPlannerUpdated_[i] = false;
        }
      }
      isUpdateTimedTarget_ = false;
    }
  }

  void MobileManipulatorReferenceManager::updateTimedOfflineTraj(scalar_t initTime, scalar_t finalTime)
  {
    static bool offlineTrajDisable_prev = false;
    if(offlineTrajDisable_ == true && offlineTrajDisable_prev == false)
    {
      isOfflineTrajUpdate_ = false;   // 关闭离线轨迹执行
      isTorsoOfflineTrajUpdate_ = false;
      for(int i=0; i<info_.eeFrames.size(); i++)
      {
        isArmEeOfflineTrajUpdate_[i] = false;
      }
    }
    else if(offlineTrajDisable_ == false)
    {
      isOfflineTrajUpdate_ = true;
    }
    offlineTrajDisable_prev = offlineTrajDisable_;
    if(trajFrameUpdate_ == true)
    {
      isofflineTrajUpdateStartTime_ = initTime;
      for(int armIdx = 0; armIdx < info_.eeFrames.size(); armIdx++)
      {
        if(isArmEeOfflineTrajUpdate_[armIdx])
        {
          if(eeOfflineTrajFrame_[armIdx] == 0)
          {
            desireMode_[armIdx] = LbArmControlMode::WorldFrame;
          }
          else if(eeOfflineTrajFrame_[armIdx] == 1)
          {
            desireMode_[armIdx] = LbArmControlMode::LocalFrame;
          }
        }
      }
      trajFrameUpdate_ = false;
    }

    int timeIncrement = (finalTime - initTime) / ruckigDt_;

    if(isOfflineTrajUpdate_)
    {
      scalar_array_t timeTraj;
      vector_array_t torsoStateTraj, armEeStateTraj[2];

      for(int i = 0; i < timeIncrement + 1; i++)
      {
        double currentTime = initTime + i * ruckigDt_;
        timeTraj.push_back(currentTime);

        // 每次只取其中一段
        if(isTorsoOfflineTrajUpdate_)
        {
          torsoStateTraj.push_back(torsoOfflineTraj_.getDesiredState(currentTime - isofflineTrajUpdateStartTime_));
        }

        for(int armIdx = 0; armIdx < info_.eeFrames.size(); armIdx++)
        {
          if(isArmEeOfflineTrajUpdate_[armIdx])
          {
            armEeStateTraj[armIdx].push_back(armEeOfflineTraj_[armIdx].getDesiredState(currentTime - isofflineTrajUpdateStartTime_));
          }
        }
      }
      // 将片段赋值到实际执行的轨迹
      if(isTorsoOfflineTrajUpdate_)
      {
        setEnableLegJointTrack(false); // 关闭下肢关节跟踪
        setEnableTorsoPoseTargetTrajectories(true); // 开启躯干
        torsoModeFlag_ = true;
        torsoTargetTrajectories_.timeTrajectory = timeTraj;
        torsoTargetTrajectories_.stateTrajectory = torsoStateTraj;
      }
      for(int armIdx = 0; armIdx < info_.eeFrames.size(); armIdx++)
      {
        if(isArmEeOfflineTrajUpdate_[armIdx])
        {
          switch(eeOfflineTrajFrame_[armIdx])
          {
            case 0: 
            {
              setEnableArmJointTrackForArm(armIdx, false); // 关闭手臂跟踪
              setEnableEeTargetLocalTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔局部跟踪
              setEnableEeTargetTrajectoriesForArm(armIdx, true); // 开启末端笛卡尔
              break;
            }
            case 1:
            {
              setEnableArmJointTrackForArm(armIdx, false); // 关闭手臂跟踪
              setEnableEeTargetLocalTrajectoriesForArm(armIdx, true); // 开启末端笛卡尔局部跟踪
              setEnableEeTargetTrajectoriesForArm(armIdx, false); // 关闭末端笛卡尔
              break;
            }
          }
          eeTargetTrajectories_[armIdx].timeTrajectory = timeTraj;
          eeTargetTrajectories_[armIdx].stateTrajectory = armEeStateTraj[armIdx];
        }
      }
    }
  }

  void MobileManipulatorReferenceManager::updateIndexRuckigPlanner(int plannerIndex, double desireTime, 
                                                                   const Eigen::VectorXd& cmd_vec)
  {
    int desiredSize = timedPlannerScheduler_.getTimedPlannerDofNum(plannerIndex);
  
    switch(static_cast<LbTimedPosCmdType>(plannerIndex))
    {
      case LbTimedPosCmdType::BASE_POS_WORLD_CMD:
      {
        isCmdPoseWorldUpdated_ = true;
        cmdPoseDesiredTime_ = desireTime;
        cmdPoseWorld_ = cmd_vec.head(desiredSize);
        std::cout << "Received cmdPoseWorld: " << cmdPoseWorld_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::BASE_POS_LOCAL_CMD:
      {
        isCmdPoseUpdated_ = true;
        cmdPoseDesiredTime_ = desireTime;
        cmdPose_ = cmd_vec.head(desiredSize);
        std::cout << "Received cmdPose: " << cmdPose_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::TORSO_POSE_CMD:
      {
        isCmdTorsoPoseUpdated_ = true;
        cmdTorsoPoseDesiredTime_ = desireTime;
        /*******************************赋值躯干指令********************************/
        vector_t tmpCmdTorsoPose = cmd_vec.head(desiredSize);
        cmdTorsoPose_[0] = tmpCmdTorsoPose[0];
        cmdTorsoPose_[2] = tmpCmdTorsoPose[1];
        cmdTorsoPose_[3] = tmpCmdTorsoPose[2];
        cmdTorsoPose_[4] = tmpCmdTorsoPose[3];
        /*************************************************************************/
        std::cout << "desiredSize: "<< desiredSize << std::endl;
        std::cout << "Received cmdTorsoPose: "<< cmdTorsoPose_.transpose() << std::endl;
        break;
      }
      case LbTimedPosCmdType::LEG_JOINT_CMD:
      {
        isCmdLegJointUpdated_ = true;
        cmdLegJointDesiredTime_ = desireTime;
        lb_leg_traj_ = cmd_vec.head(desiredSize);
        break;
      }
      case LbTimedPosCmdType::LEFT_ARM_WORLD_CMD:
      {
        isCmdDualArmPoseUpdated_[0] = true;
        desireMode_[0] = LbArmControlMode::WorldFrame;
        cmdDualArmPoseDesiredTime_[0] = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpLeftArmCmd = cmd_vec.head(desiredSize);
        cmd_arm_zyx_[0] = tmpLeftArmCmd;
        cmd_arm_zyx_[0][3] = targetYawPreProcess(initState_[2], cmd_arm_zyx_[0][3]);  // 世界系指令的yaw需要处理成多圈
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD:
      {
        isCmdDualArmPoseUpdated_[1] = true;
        desireMode_[1] = LbArmControlMode::WorldFrame;
        cmdDualArmPoseDesiredTime_[1] = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpRightArmCmd = cmd_vec.head(desiredSize);
        cmd_arm_zyx_[1] = tmpRightArmCmd;
        cmd_arm_zyx_[1][3] = targetYawPreProcess(initState_[2], cmd_arm_zyx_[1][3]);  // 世界系指令的yaw需要处理成多圈
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::LEFT_ARM_LOCAL_CMD:
      {
        isCmdDualArmPoseUpdated_[0] = true;
        desireMode_[0] = LbArmControlMode::LocalFrame;
        cmdDualArmPoseDesiredTime_[0] = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpLeftArmCmd = cmd_vec.head(desiredSize);
        cmd_arm_zyx_[0] = tmpLeftArmCmd;
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::RIGHT_ARM_LOCAL_CMD:
      {
        isCmdDualArmPoseUpdated_[1] = true;
        desireMode_[1] = LbArmControlMode::LocalFrame;
        cmdDualArmPoseDesiredTime_[1] = desireTime;
        /*******************************赋值双臂指令********************************/
        Eigen::VectorXd tmpRightArmCmd = cmd_vec.head(desiredSize);
        cmd_arm_zyx_[1] = tmpRightArmCmd;
        /*************************************************************************/
        break;
      }
      case LbTimedPosCmdType::LEFT_ARM_JOINT_CMD:
      {
        isCmdArmJointUpdated_[0] = true;
        desireMode_[0] = LbArmControlMode::JointSpace;  // 切换模式
        cmdArmJointDesiredTime_[0] = desireTime;
        arm_joint_traj_[0] = cmd_vec.head(desiredSize);
        break;
      }
      case LbTimedPosCmdType::RIGHT_ARM_JOINT_CMD:
      {
        isCmdArmJointUpdated_[1] = true;
        desireMode_[1] = LbArmControlMode::JointSpace;  // 切换模式
        cmdArmJointDesiredTime_[1] = desireTime;
        arm_joint_traj_[1] = cmd_vec.head(desiredSize);
        break;
      }
    }
  }

}  // namespace mobile_manipulator
}  // namespace ocs2