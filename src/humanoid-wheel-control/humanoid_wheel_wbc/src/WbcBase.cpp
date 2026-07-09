//
// Created by qiayuan on 2022/7/1.
//

// some ref: https://github.com/skywoodsz/qm_control

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "humanoid_wheel_wbc/WbcBase.h"

#include "humanoid_wheel_interface/AccessHelperFunctions.h"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <utility>


namespace ocs2
{
  namespace mobile_manipulator
  {
    // stateDesired/inputDesired 与 MPC 向量同布局：head(3)=base(x,y,yaw)，[3:7]=下肢4关节，tail(14)=双臂
    WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, const ManipulatorModelInfo& info)
        : pinocchioInterfaceMeasured_(pinocchioInterface), pinocchioInterfaceDesired_(pinocchioInterface),
          info_(info)
    {
      Eigen::setNbThreads(1);  // No multithreading within Eigen.
      Eigen::initParallel();
      
      // 决策变量：x = [3*base_acc + (4+7*2)*joint_acc, f_contact, (4+7*2)*torque]
      // f_contact: 12维，双臂各6维（力+力矩）
      std::cout << "[wbcBase] info_.armDim: " << info_.armDim << std::endl;
      contact_force_size_ = 6 * info_.eeFrames.size(); // 每个末端执行器6维
      numDecisionVars_ = info_.stateDim + contact_force_size_ + info_.armDim;
      
      const auto &model = pinocchioInterfaceMeasured_.getModel();
      int nq = model.nq;
      int nv = model.nv;
      qMeasured_ = vector_t(nq);
      vMeasured_ = vector_t(nv);
      qDesired_ = vector_t(nq);
      vDesired_ = vector_t(nv);

      torso_id_ = model.getBodyId(info_.torsoFrame);
      base_id_ = model.getBodyId(info_.baseFrame);
      ee_ids_.resize(info_.eeFrames.size());
      for(int i = 0; i < ee_ids_.size(); i++)
      {
        ee_ids_[i] = model.getBodyId(info_.eeFrames[i]);
      }

      // 初始化末端执行器雅可比矩阵
      j_ee_.resize(ee_ids_.size());
      dj_ee_.resize(ee_ids_.size());
      for (size_t i = 0; i < ee_ids_.size(); ++i) {
        j_ee_[i] = matrix_t::Zero(6, info_.stateDim);
        dj_ee_[i] = matrix_t::Zero(6, info_.stateDim);
      }

      std::cout << "[wbcBase] model.nq: " << model.nq << std::endl;
      std::cout << "[wbcBase] model.nv: " << model.nv << std::endl;
      std::cout << "[wbcBase] contact_force_size_: " << contact_force_size_ << std::endl;
    }

    vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation)
    {
      updateMeasured(observation.state, observation.input);
      updateDesired(stateDesired, inputDesired);

      return {};
    }

    void WbcBase::updateMeasured(const vector_t &stateMeasured, const vector_t &inputMeasured)
    {
      const auto &model = pinocchioInterfaceMeasured_.getModel();
      auto &data = pinocchioInterfaceMeasured_.getData();
      // std::cout << "stateMeasured: " << stateMeasured.size() << std::endl;
      // std::cout << "inputMeasured: " << inputMeasured.size() << std::endl;

      qMeasured_ = stateMeasured;
      vMeasured_ = inputMeasured;
      // vMeasured_.head(3) = bodyToWorldVelocity(inputMeasured(0), inputMeasured(1), qMeasured_(2));
      // vMeasured_.tail(info_.armDim) = inputMeasured.tail(info_.armDim);

      
      // For floating base EoM task
      pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
      pinocchio::computeJointJacobians(model, data);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::crba(model, data, qMeasured_);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

      // For Torso Zero Acc task
      matrix_t j_base = matrix_t::Zero(6, info_.stateDim);
      pinocchio::getFrameJacobian(model, data, base_id_, pinocchio::LOCAL_WORLD_ALIGNED, j_base);
      j_torso_ = matrix_t::Zero(6, info_.stateDim);
      pinocchio::getFrameJacobian(model, data, torso_id_, pinocchio::LOCAL_WORLD_ALIGNED, j_torso_);
      j_torso_ = j_torso_ - j_base;

      // 计算每个末端执行器的6维雅可比矩阵
      for (size_t i = 0; i < ee_ids_.size(); ++i) {
        pinocchio::getFrameJacobian(model, data, ee_ids_[i],
                                    pinocchio::LOCAL_WORLD_ALIGNED, j_ee_[i]);
      }

      j_base = matrix_t::Zero(6, info_.stateDim);
      pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
      dj_torso_ = matrix_t::Zero(6, info_.stateDim);
      pinocchio::getFrameJacobianTimeVariation(model, data, torso_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, dj_torso_);
      pinocchio::getFrameJacobianTimeVariation(model, data, base_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, j_base);
      dj_torso_ = dj_torso_ - j_base;

      // 计算雅可比时间导数
      for (size_t i = 0; i < ee_ids_.size(); ++i) {
        pinocchio::getFrameJacobianTimeVariation(model, data, ee_ids_[i],
                                                 pinocchio::LOCAL_WORLD_ALIGNED, dj_ee_[i]);
      }
    }

    void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
    {
      const auto &model = pinocchioInterfaceDesired_.getModel();
      auto &data = pinocchioInterfaceDesired_.getData();
      // std::cout << "stateDesired: " << stateDesired.size() << std::endl;
      // std::cout << "inputDesired: " << inputDesired.size() << std::endl;

      // qDesired_/vDesired_ 与 optimizedState_wbc/optimizedInput_wbc 同索引：input[i]=d(state[i])/dt
      qDesired_ = stateDesired;
      vDesired_ = inputDesired;
      // vDesired_.head(3) = bodyToWorldVelocity(inputDesired(0), inputDesired(1), qDesired_(2));
      // vDesired_.tail(info_.armDim) = inputDesired.tail(info_.armDim);

      // For task each
      pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);
      pinocchio::computeJointJacobians(model, data);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::crba(model, data, vDesired_);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      pinocchio::nonLinearEffects(model, data, vDesired_, vDesired_);
    }

    Task WbcBase::formulateFloatingBaseEomTask()
    {
      auto &data = pinocchioInterfaceMeasured_.getData();
      matrix_t s = matrix_t::Zero(info_.stateDim, info_.armDim);
      s.bottomRows(info_.armDim) = matrix_t::Identity(info_.armDim, info_.armDim);

      // 组装所有末端执行器的雅可比矩阵
      matrix_t j_contact = matrix_t::Zero(info_.stateDim, contact_force_size_);
      for (size_t i = 0; i < ee_ids_.size(); ++i) {
        // J^T: 将6维雅可比转置后放入对应位置
        j_contact.block(0, i*6, info_.stateDim, 6) = j_ee_[i].transpose();
      }

      matrix_t a(info_.stateDim, numDecisionVars_);
      a.setZero();
      // A = [M, -J^T, -S]
      // 决策变量顺序：x = [ddq_stateDim, f_contact, tau_armDim]
      matrix_t M = data.M;
      // 保持底盘、下肢、上肢各自的对角块，但清零它们之间的耦合项
      size_t base_dim = info_.stateDim - info_.armDim;
      
      // 清零底盘与下肢/上肢的耦合项
      M.block(0, base_dim, base_dim, info_.armDim).setZero();
      M.block(base_dim, 0, info_.armDim, base_dim).setZero();
      
      // 清零下肢与上肢的耦合项
      M.block(base_dim, base_dim + lowJoint_nums_, lowJoint_nums_, arm_nums_).setZero();
      M.block(base_dim + lowJoint_nums_, base_dim, arm_nums_, lowJoint_nums_).setZero();
      
      a.block(0, 0, info_.stateDim, info_.stateDim) = M;  // ddq的系数
      a.block(0, info_.stateDim, info_.stateDim, contact_force_size_) = -j_contact; // f_contact的系数
      a.block(0, info_.stateDim + contact_force_size_, info_.stateDim, info_.armDim) = -s; // tau的系数

      vector_t b = -data.nle;

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask()
    {
      matrix_t d(2 * info_.armDim, numDecisionVars_);
      d.setZero();
      matrix_t i = matrix_t::Identity(info_.armDim, info_.armDim);
      // 力矩在决策变量中的位置：stateDim + contact_force_size 之后
      size_t torque_start_idx = info_.stateDim + contact_force_size_;
      d.block(0, torque_start_idx, info_.armDim, info_.armDim) = i;
      d.block(info_.armDim, torque_start_idx, info_.armDim, info_.armDim) = -i;
      vector_t f(2 * info_.armDim);
      vector_t all_joint_limits(info_.armDim);
      all_joint_limits << torqueLimits_.head(lowJoint_nums_), torqueLimits_.tail(arm_nums_/2), torqueLimits_.tail(arm_nums_/2);
      f << all_joint_limits, all_joint_limits;

      return {matrix_t(), vector_t(), d, f};
    }

    Task WbcBase::formulateLowJointAccelTask()
    {
      matrix_t a(lowJoint_nums_, numDecisionVars_);
      vector_t b(a.rows());
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.stateDim-info_.armDim, lowJoint_nums_, lowJoint_nums_) = matrix_t::Identity(lowJoint_nums_, lowJoint_nums_);
      
      // 计算位置和速度误差并进行故障检测和限幅处理
      vector_t pos_error = qDesired_.tail(info_.armDim).head(lowJoint_nums_) - qMeasured_.tail(info_.armDim).head(lowJoint_nums_);
      vector_t vel_error = vDesired_.tail(info_.armDim).head(lowJoint_nums_) - vMeasured_.tail(info_.armDim).head(lowJoint_nums_);
      if(is_real_) processLowJointErrorsWithSafe(pos_error, vel_error);   // 真实环境下进行故障检测和限幅处理
      
      // 计算 PD 控制量并限幅
      b = lowJointKp_.cwiseProduct(pos_error) + lowJointKd_.cwiseProduct(vel_error);
      b = b.cwiseMax(-20.0).cwiseMin(20.0);  // 加速度限幅 [-20, 20]

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateArmJointAccelTask()
    {
      matrix_t a(arm_nums_, numDecisionVars_);
      vector_t b(a.rows());
      const bool useVrArmPd = useVrArmAccelTask_ && hasVrArmAccelTask_;
      const vector_t& armKp = useVrArmPd ? vrArmJointKp_ : armJointKp_;
      const vector_t& armKd = useVrArmPd ? vrArmJointKd_ : armJointKd_;
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.stateDim-arm_nums_, arm_nums_, arm_nums_) = matrix_t::Identity(arm_nums_, arm_nums_);
      
      // 计算位置和速度误差并进行故障检测和限幅处理
      vector_t pos_error = qDesired_.tail(arm_nums_) - qMeasured_.tail(arm_nums_);
      vector_t vel_error = vDesired_.tail(arm_nums_) - vMeasured_.tail(arm_nums_);
      if(is_real_) processArmJointErrorsWithSafe(pos_error, vel_error);   // 真实环境下进行故障检测和限幅处理
      
      // 计算 PD 控制量并限幅
      b = armKp.cwiseProduct(pos_error) + armKd.cwiseProduct(vel_error);
      b = b.cwiseMax(-300.0).cwiseMin(300.0);  // 加速度限幅 [-300, 300]

      return {a, b, matrix_t(), vector_t()};
    }
    
    void WbcBase::processArmJointErrorsWithSafe(vector_t& pos_error, vector_t& vel_error)
    {
      const double max_pos_jump = 0.5;   // 最大允许位置跳变 [rad]
      const double max_pos_error = 0.017453293;  // 最大位置误差 [rad], 1度, 减少位置的主要作用
      const double max_vel_error = 3.0; // 最大速度误差 [rad/s]
      
      static vector_t qMeasured_prev;
      static vector_t vMeasured_prev;
      static std::vector<bool> sensor_fault_flags(arm_nums_, false);
      
      // 初始化历史数据
      if (qMeasured_prev.size() != arm_nums_) {
        qMeasured_prev.resize(arm_nums_);
        qMeasured_prev = qMeasured_.tail(arm_nums_);
        vMeasured_prev.resize(arm_nums_);
        vMeasured_prev = vMeasured_.tail(arm_nums_);
        sensor_fault_flags.resize(arm_nums_, false);
      }
      
      // 每个关节独立检测和处理
      for (int i = 0; i < arm_nums_; ++i) {
        // 位置跳变检测
        double pos_jump_i = std::abs(qMeasured_.tail(arm_nums_)[i] - qMeasured_prev[i]);
        sensor_fault_flags[i] = (pos_jump_i > max_pos_jump);
        
        if (sensor_fault_flags[i]) {
          // 故障关节使用历史数据
          pos_error[i] = qDesired_.tail(arm_nums_)[i] - qMeasured_prev[i];
          vel_error[i] = vDesired_.tail(arm_nums_)[i] - vMeasured_prev[i];
          qMeasured_.tail(arm_nums_)[i] = qMeasured_prev[i];
          vMeasured_.tail(arm_nums_)[i] = 0.0;
          std::cout << "[WbcBase] 手臂关节 " << i << " 位置跳变故障！跳变值：" << pos_jump_i << std::endl;
          std::cout << "[WbcBase] 手臂关节 " << i << " 速度跳变故障！跳变值：" << vel_error[i] << std::endl;
        }
        
        // 误差限幅
        pos_error[i] = std::max(-max_pos_error, std::min(max_pos_error, pos_error[i]));
        vel_error[i] = std::max(-max_vel_error, std::min(max_vel_error, vel_error[i]));
      }
      
      // 更新历史数据
      qMeasured_prev = qMeasured_.tail(arm_nums_);
      vMeasured_prev = vMeasured_.tail(arm_nums_);
    }

    void WbcBase::processLowJointErrorsWithSafe(vector_t& pos_error, vector_t& vel_error)
    {
      const double max_pos_jump = 0.5;   // 最大允许位置跳变 [rad]
      const double max_pos_error = 0.017453293;  // 最大位置误差 [rad], 1度, 减少位置的主要作用
      const double max_vel_error = 0.1; // 最大速度误差 [rad/s]
      
      static vector_t qMeasured_prev_low;
      static vector_t vMeasured_prev_low;
      static std::vector<bool> sensor_fault_flags_low(lowJoint_nums_, false);
      
      // 初始化历史数据
      if (qMeasured_prev_low.size() != lowJoint_nums_) {
        qMeasured_prev_low.resize(lowJoint_nums_);
        qMeasured_prev_low = qMeasured_.tail(info_.armDim).head(lowJoint_nums_);
        vMeasured_prev_low.resize(lowJoint_nums_);
        vMeasured_prev_low = vMeasured_.tail(info_.armDim).head(lowJoint_nums_);
        sensor_fault_flags_low.resize(lowJoint_nums_, false);
      }
      
      // 每个关节独立检测和处理
      for (int i = 0; i < lowJoint_nums_; ++i) {
        // 位置跳变检测
        double pos_jump_i = std::abs(qMeasured_.tail(info_.armDim).head(lowJoint_nums_)[i] - qMeasured_prev_low[i]);
        sensor_fault_flags_low[i] = (pos_jump_i > max_pos_jump);
        
        if (sensor_fault_flags_low[i]) {
          // 故障关节使用历史数据
          pos_error[i] = qDesired_.tail(info_.armDim).head(lowJoint_nums_)[i] - qMeasured_prev_low[i];
          vel_error[i] = vDesired_.tail(info_.armDim).head(lowJoint_nums_)[i] - vMeasured_prev_low[i];
          qMeasured_.tail(info_.armDim).head(lowJoint_nums_)[i] = qMeasured_prev_low[i];
          vMeasured_.tail(info_.armDim).head(lowJoint_nums_)[i] = 0.0;
          std::cout << "[WbcBase] 下肢关节 " << i << " 位置跳变故障！跳变值：" << pos_jump_i << std::endl;
        }
        
        // 误差限幅
        pos_error[i] = std::max(-max_pos_error, std::min(max_pos_error, pos_error[i]));
        vel_error[i] = std::max(-max_vel_error, std::min(max_vel_error, vel_error[i]));
      }
      
      // 更新历史数据
      qMeasured_prev_low = qMeasured_.tail(info_.armDim).head(lowJoint_nums_);
      vMeasured_prev_low = vMeasured_.tail(info_.armDim).head(lowJoint_nums_);
    }

    Task WbcBase::formulateBaseAccTask()
    {
      matrix_t a(info_.stateDim-info_.armDim, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();

      a.block(0, 0, info_.stateDim-info_.armDim, info_.stateDim-info_.armDim) = matrix_t::Identity(info_.stateDim-info_.armDim, info_.stateDim-info_.armDim);

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorsoZeroAccTask()
    {
      matrix_t a(6, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();

      a.block(0, 0, 6, info_.stateDim) = j_torso_;
      b = -dj_torso_ * vMeasured_;

      // std::cout << "j_torso_\n" << j_torso_.transpose() << std::endl;

      return {a, b, matrix_t(), vector_t()};
    }

    void WbcBase::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      // Load task file
      torqueLimits_ = vector_t(arm_nums_ / 2 + lowJoint_nums_);
      is_real_ = is_real;
      loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
      if (verbose)
      {
        std::cerr << "\n #### Torque Limits Task:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### each motor: " << torqueLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
      }

      std::string prefix = "lowJointAccelTask.";
      if(verbose)
      {
        std::cerr << "\n #### Low Joint Accel Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      lowJointKp_.resize(lowJoint_nums_);
      lowJointKd_.resize(lowJoint_nums_);

      if(!is_real)
      {
        loadData::loadEigenMatrix(taskFile, prefix + "kp_sim", lowJointKp_);
        loadData::loadEigenMatrix(taskFile, prefix + "kd_sim", lowJointKd_);
      }
      else
      {
        loadData::loadEigenMatrix(taskFile, prefix + "kp", lowJointKp_);
        loadData::loadEigenMatrix(taskFile, prefix + "kd", lowJointKd_);
      }

      if(arm_nums_ != 0){
        prefix = "armAccelTask.";
        if(verbose)
        {
          std::cerr << "\n #### arm Accel Task:";
          std::cerr << "\n #### =============================================================================\n";
        }
        armJointKp_.resize(arm_nums_);
        armJointKd_.resize(arm_nums_);
        if(!is_real)
        {
          loadData::loadEigenMatrix(taskFile, prefix + "kp_sim", armJointKp_);
          loadData::loadEigenMatrix(taskFile, prefix + "kd_sim", armJointKd_);
        }
        else
        {
          loadData::loadEigenMatrix(taskFile, prefix + "kp", armJointKp_);
          loadData::loadEigenMatrix(taskFile, prefix + "kd", armJointKd_);
        }

        vrArmJointKp_ = armJointKp_;
        vrArmJointKd_ = armJointKd_;
        hasVrArmAccelTask_ = false;
        bool hasVrKp = false;
        bool hasVrKd = false;
        try
        {
          prefix = "vrArmAccelTask.";
          vector_t vrKp(arm_nums_);
          loadData::loadEigenMatrix(taskFile, prefix + "kp", vrKp);
          vrArmJointKp_ = vrKp;
          hasVrKp = true;
        }
        catch (const std::exception&)
        {
        }
        try
        {
          prefix = "vrArmAccelTask.";
          vector_t vrKd(arm_nums_);
          loadData::loadEigenMatrix(taskFile, prefix + "kd", vrKd);
          vrArmJointKd_ = vrKd;
          hasVrKd = true;
        }
        catch (const std::exception&)
        {
        }
        hasVrArmAccelTask_ = hasVrKp && hasVrKd;
        if (verbose)
        {
          std::cerr << "\n #### vrArm Accel Task loaded: " << (hasVrArmAccelTask_ ? "true" : "false") << "\n";
        }
      }
    }

    Eigen::Vector3d WbcBase::bodyToWorldVelocity(double v_body, double vyaw_body, double yaw)
    {
      // 创建旋转矩阵（绕z轴旋转）
      Eigen::Matrix2d R_yaw;
      R_yaw << std::cos(yaw), -std::sin(yaw),
               std::sin(yaw),  std::cos(yaw);
      // 本体系线速度（假设侧向速度为0）
      Eigen::Vector2d v_body_2d(v_body, 0.0);
      // 转换到世界系
      Eigen::Vector2d v_world_2d = R_yaw * v_body_2d;

      // 角速度保持不变
      return Eigen::Vector3d(v_world_2d.x(), v_world_2d.y(), vyaw_body);
    }

  } // namespace mobile_manipulator
} // namespace ocs2
