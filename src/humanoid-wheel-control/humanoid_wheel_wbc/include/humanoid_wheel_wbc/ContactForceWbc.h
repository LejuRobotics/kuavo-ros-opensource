//
// Created for contact force control task
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wheel_wbc/WeightedWbc.h"

namespace ocs2
{
  namespace mobile_manipulator
  {
    /**
     * @brief ContactForceWbc - 接触力控制WBC类
     * 
     * 该类继承自WeightedWbc，添加了接触力跟踪任务。
     * 
     * 注意：末端执行器雅可比矩阵（j_ee_和dj_ee_）在基类WbcBase::updateMeasured()中计算，
     * 该类可以直接访问这些protected成员，用于未来实现更复杂的接触力控制任务。
     * 
     * 可访问的基类成员（来自WbcBase）：
     * - j_ee_: 末端执行器雅可比矩阵（6维，世界坐标系）
     * - dj_ee_: 末端执行器雅可比时间导数
     * - contact_force_size_: 接触力维度
     * - numDecisionVars_: 决策变量维度
     * - info_: 机器人模型信息
     */
    class ContactForceWbc : public WeightedWbc
    {
    public:
      using WeightedWbc::WeightedWbc;

      void loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) override;

      void setDesiredContactForce(const vector_t& desiredForce) {
        // 当期望力改变时，才重置force_disabled_为false
        bool force_changed = (desired_contact_force_.size() != desiredForce.size()) ||
                             (desired_contact_force_ - desiredForce).cwiseAbs().maxCoeff() > 1e-6;
        
        desired_contact_force_ = desiredForce;
        if (force_changed) 
        {
          // 当用户重新设置期望力时，重置禁用状态，允许重新启用接触力任务
          force_disabled_ = false;
        }
      }

      /**
       * @brief 更新关节位置误差并检测手臂是否挥空
       * @param jointPositionError 手臂关节位置误差（只包含arm joints）
       */
      void updateJointPositionError(const vector_t& jointPositionError, bool enable_swing_empty_disable = true);

      /**
       * @brief 检查接触力任务是否被禁用
       */
      bool isForceDisabled() const {
        return force_disabled_;
      }

      /**
       * @brief 重置挥空标志，允许重新启用接触力任务
       */
      void resetForceDisabled() {
        force_disabled_ = false;
      }

      /**
       * @brief 主动失效期望力（无需满足关节误差条件）
       */
      void disableForce() {
        force_disabled_ = true;
        desired_contact_force_ = vector_t::Zero(contact_force_size_);
      }

      /**
       * @brief 更新自适应权重（根据期望力大小动态调整手臂任务权重）
       * @param current_time 当前时间（用于平滑过渡）
       */
      void updateAdaptiveWeights(const ros::Time& current_time);

      /**
       * @brief 获取当前有效的手臂任务权重
       */
      scalar_t getAdaptiveArmWeight() const { return weightArmAccelCurrent_; }

      /**
       * @brief 获取期望力插值速度
       */
      scalar_t getInterpolationSpeed() const { return interpolationSpeed_; }

      /** 与 task.info contact_force_params 对应，运行时可改（>0 才生效） */
      void setTransitionTime(scalar_t t) {
        if (t > scalar_t(1e-9)) transitionTime_ = t;
      }
      void setInterpolationSpeed(scalar_t s) {
        if (s > scalar_t(1e-9)) interpolationSpeed_ = s;
      }

    protected:
      virtual Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired) override;

    private:
      Task formulateContactForceTask(const vector_t& desiredForce);
      
      vector_t desired_contact_force_; // 期望接触力
      scalar_t weightContactForce_;    // 接触力任务权重
      
      scalar_t error_threshold_;        // 误差阈值（rad），超过此值禁用接触力
      bool force_disabled_;             // 当前是否禁用接触力任务
      
      // 自适应权重相关
      scalar_t weightArmAccelOriginal_;    // 原始手臂任务权重（从配置文件加载，默认30）
      scalar_t weightArmAccelCurrent_;     // 当前有效的手臂任务权重（动态调整）
      scalar_t weightArmAccelTarget_;      // 目标手臂任务权重（根据期望力状态确定：3或30）
      scalar_t weightArmAccelReduced_;     // 降低后的手臂任务权重（施加期望力时，默认3）
      scalar_t forceThreshold_;            // 期望力阈值（5.0N），超过此值认为施加了期望力
      scalar_t transitionTime_;    // 手臂任务权重过渡时间（秒）
      ros::Time transitionStartTime_;      // 过渡开始时间（用于线性插值）
      scalar_t transitionStartWeight_;      // 过渡开始时的权重值（用于线性插值）
      scalar_t interpolationSpeed_;        // 期望力插值速度（N/s）
      
      // 力控制kp、kd相关
      vector_t armJointKpForceControl_;
      vector_t armJointKdForceControl_;
      vector_t armJointKpOriginal_;
      vector_t armJointKdOriginal_;
      bool useForceControlKpKd_;
    };

  } // namespace mobile_manipulator
} // namespace ocs2

