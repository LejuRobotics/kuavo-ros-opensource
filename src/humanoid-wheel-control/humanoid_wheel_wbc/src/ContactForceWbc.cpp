#include "humanoid_wheel_wbc/ContactForceWbc.h"
#include <boost/property_tree/info_parser.hpp> // For loadPtreeValue
#include <ros/ros.h>
#include <cmath>

namespace ocs2
{
  namespace mobile_manipulator
  {
    Task ContactForceWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired)
    {
      // 使用动态权重重写基类的formulateWeightedTasks
      // 注意：需要访问基类的其他权重和任务，但手臂任务权重使用动态权重
      // std::cout << "weightArmAccelCurrent_: " << weightArmAccelCurrent_ << std::endl;
      Task totalTask = formulateLowJointAccelTask() * getWeightLowJointAccel() + 
                       formulateArmJointAccelTask() * weightArmAccelCurrent_ +  // 使用动态权重
                       formulateTorsoZeroAccTask() * getWeightTorsoZeroAccel();
      
      // 如果设置了期望力，添加接触力任务
      // 注意：contact_force_size_ 是基类WbcBase的protected成员，可以直接访问
      if (desired_contact_force_.size() == contact_force_size_) {
        // 令期望力失效：f_contact是决策变量，在浮基动力学约束中：M*ddq - J^T*f_contact - S*tau = -nle
        // 如果权重为0，f_contact在优化目标中无约束，QP求解器可能给它任意值，导致手臂乱甩
        // 因此将期望接触力设为0使期望力失效（在手臂挥空时生效）
        vector_t effective_desired_force = force_disabled_ ? 
                                          vector_t::Zero(contact_force_size_) : 
                                          desired_contact_force_;
        // std::cout << "weightContactForce_: " << weightContactForce_ << std::endl;
        totalTask = totalTask + 
                    formulateContactForceTask(effective_desired_force) * 
                    weightContactForce_;
      }
      
      return totalTask;
    }

    Task ContactForceWbc::formulateContactForceTask(const vector_t& desiredForce)
    {
      // 接触力任务：直接跟踪期望接触力
      // 如果未来需要实现基于末端位置的力控制，可以使用基类的 j_ee_ 和 dj_ee_, 在WbcBase::updateMeasured()中已经计算好
      
      size_t contact_force_dim = contact_force_size_;  // 基类protected成员
      matrix_t a(contact_force_dim, numDecisionVars_); // 基类protected成员
      vector_t b(contact_force_dim);
      
      a.setZero();
      // 决策变量顺序：x = [ddq_stateDim, f_contact, tau_armDim]
      // 接触力在决策变量中的位置：stateDim 之后
      size_t contact_force_start_idx = info_.stateDim; // 基类protected成员
      a.block(0, contact_force_start_idx, contact_force_dim, contact_force_dim) = 
          matrix_t::Identity(contact_force_dim, contact_force_dim);
      
      b = desiredForce;
      
      return {a, b, matrix_t(), vector_t()};
    }

    void ContactForceWbc::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      // 先调用基类的loadTasksSetting
      WeightedWbc::loadTasksSetting(taskFile, verbose, is_real);

      // 加载接触力任务权重
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string weight_prefix = "weight.";
      loadData::loadPtreeValue(pt, weightContactForce_, weight_prefix + "contactForce", verbose);
      
      // 加载接触力参数（从新的 contact_force_params 部分）
      std::string contact_force_prefix = "contact_force_params.";
      loadData::loadPtreeValue(pt, error_threshold_, contact_force_prefix + "contactForceErrorThreshold", verbose);
      
      // 加载自适应权重参数
      weightArmAccelReduced_ = 3.0;  // 默认值
      loadData::loadPtreeValue(pt, weightArmAccelReduced_, contact_force_prefix + "weightArmAccelReduced", verbose);
      
      forceThreshold_ = 5.0;  // 默认值 5.0N
      loadData::loadPtreeValue(pt, forceThreshold_, contact_force_prefix + "forceThreshold", verbose);
      
      transitionTime_ = 1.0;
      loadData::loadPtreeValue(pt, transitionTime_, contact_force_prefix + "transitionTime", verbose);
      interpolationSpeed_ = 15.0;
      loadData::loadPtreeValue(pt, interpolationSpeed_, contact_force_prefix + "interpolationSpeed", verbose);
      // 获取原始手臂任务权重（从基类加载的配置值）
      weightArmAccelOriginal_ = getWeightArmAccel();
      
      // 初始化当前权重和目标权重
      weightArmAccelCurrent_ = weightArmAccelOriginal_;
      weightArmAccelTarget_ = weightArmAccelOriginal_;
      transitionStartTime_ = ros::Time::now();
      transitionStartWeight_ = weightArmAccelOriginal_;
      
      armJointKpOriginal_ = armJointKp_;  // 保存基类的原始值
      armJointKdOriginal_ = armJointKd_;  // 保存基类的原始值
      
      // 加载力控制kp、kd参数（施加期望力时使用）
      std::string arm_task_prefix = "armAccelTask.";
      armJointKpForceControl_.resize(arm_nums_);
      armJointKdForceControl_.resize(arm_nums_);
      loadData::loadEigenMatrix(taskFile, arm_task_prefix + "kp_force_control", armJointKpForceControl_);
      loadData::loadEigenMatrix(taskFile, arm_task_prefix + "kd_force_control", armJointKdForceControl_);
      
      // 初始化使用标志
      useForceControlKpKd_ = false;
      
      if (verbose)
      {
        std::cerr << "\n #### Contact Force Task Weight:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### weightContactForce: " << weightContactForce_ << "\n";
        std::cerr << "\n #### contactForceErrorThreshold: " << error_threshold_ << " rad\n";
        std::cerr << "\n #### Adaptive Weight Settings:";
        std::cerr << "\n ####   weightArmAccelOriginal: " << weightArmAccelOriginal_ << "\n";
        std::cerr << "\n ####   weightArmAccelReduced: " << weightArmAccelReduced_ << "\n";
        std::cerr << "\n ####   forceThreshold: " << forceThreshold_ << " N\n";
        std::cerr << "\n ####   transitionTime: " << transitionTime_ << " s\n";
        std::cerr << "\n ####   interpolationSpeed: " << interpolationSpeed_ << " N/s\n";
        std::cerr << " #### =============================================================================\n";
        
        std::cerr << "\n #### Force Control Kp/Kd Settings:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### armJointKpForceControl: " << armJointKpForceControl_.transpose() << "\n";
        std::cerr << "\n #### armJointKdForceControl: " << armJointKdForceControl_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
      }

      // 初始化期望接触力
      desired_contact_force_ = vector_t::Zero(contact_force_size_);
      
      // 初始化自适应失效相关参数
      force_disabled_ = false;
    }

    void ContactForceWbc::updateJointPositionError(const vector_t& jointPositionError, bool enable_swing_empty_disable)
    {
      // 获取关节误差向量中最大的单关节误差（取绝对值）
      double joint_error_max = jointPositionError.cwiseAbs().maxCoeff();
      // 一旦检测到挥空（误差超过阈值），且启用了挥空检测失效功能，立即将期望接触力设为0
      if (!force_disabled_ && joint_error_max > error_threshold_ && enable_swing_empty_disable) {
        ROS_WARN("Detected joint error exceeding contactForceErrorThreshold (%.3f), triggering empty-swing detection.", static_cast<double>(error_threshold_));
        force_disabled_ = true;
        desired_contact_force_ = vector_t::Zero(contact_force_size_);
      }
      // 注意：不再自动恢复，只有当用户更新期望接触力时才会重置 force_disabled_ = false
    }

    void ContactForceWbc::updateAdaptiveWeights(const ros::Time& current_time)
    {
      // 检测期望力大小
      double force_magnitude = 0.0;
      if (desired_contact_force_.size() == contact_force_size_) {
        force_magnitude = desired_contact_force_.norm();
      }
      
      // 根据期望力状态立即切换kp、kd（无过渡）
      bool use_force_control = (force_magnitude > forceThreshold_ && !force_disabled_);
      
      // 如果状态改变，立即切换kp、kd
      if (use_force_control != useForceControlKpKd_) {
        useForceControlKpKd_ = use_force_control;
        if (useForceControlKpKd_) {
          // 切换到力控制kp、kd
          armJointKp_ = armJointKpForceControl_;
          armJointKd_ = armJointKdForceControl_;
        } else {
          // 切换回原始kp、kd（修改基类的成员变量）
          armJointKp_ = armJointKpOriginal_;
          armJointKd_ = armJointKdOriginal_;
        }
      }
      
      // 根据期望力是否超过阈值，设置目标权重
      scalar_t new_target = weightArmAccelTarget_;
      if (force_magnitude > forceThreshold_ && !force_disabled_) {
        // 有期望力且未禁用，降低手臂任务权重
        new_target = weightArmAccelReduced_;  // 3.0
      } else {
        // 无期望力或已禁用，恢复原始权重
        new_target = weightArmAccelOriginal_;  // 30.0
      }
      
      // 如果目标权重改变，重置过渡开始时间和开始权重
      if (std::abs(new_target - weightArmAccelTarget_) > 1e-6) {
        weightArmAccelTarget_ = new_target;
        transitionStartTime_ = current_time;
        transitionStartWeight_ = weightArmAccelCurrent_;
      }
      
      // 使用线性插值在1秒内完成过渡
      double elapsed_time = std::max(0.0, (current_time - transitionStartTime_).toSec());
      double progress = std::min(1.0, elapsed_time / static_cast<double>(transitionTime_));
      weightArmAccelCurrent_ = transitionStartWeight_ + 
                               progress * (weightArmAccelTarget_ - transitionStartWeight_);
    }

  } // namespace mobile_manipulator
} // namespace ocs2

