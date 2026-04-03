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
    // 先完善，输入 (x, y, yaw, q_des), 前面不管，只处理后面的关节跟踪
    WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, const ManipulatorModelInfo& info)
        : pinocchioInterfaceMeasured_(pinocchioInterface), pinocchioInterfaceDesired_(pinocchioInterface),
          info_(info)
    {
      Eigen::setNbThreads(1);  // No multithreading within Eigen.
      Eigen::initParallel();
      
      // 决策变量， 3*base_acc + (4+7*2)*joint_acc + (4+7*2)*torque
      std::cout << "[wbcBase] info_.armDim: " << info_.armDim << std::endl;
      numDecisionVars_ = info_.stateDim + info_.armDim;
      
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

      std::cout << "[wbcBase] model.nq: " << model.nq << std::endl;
      std::cout << "[wbcBase] model.nv: " << model.nv << std::endl;
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

      j_base = matrix_t::Zero(6, info_.stateDim);
      pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
      dj_torso_ = matrix_t::Zero(6, info_.stateDim);
      pinocchio::getFrameJacobianTimeVariation(model, data, torso_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, dj_torso_);
      pinocchio::getFrameJacobianTimeVariation(model, data, base_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, j_base);
      dj_torso_ = dj_torso_ - j_base;
    }

    void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
    {
      const auto &model = pinocchioInterfaceDesired_.getModel();
      auto &data = pinocchioInterfaceDesired_.getData();
      // std::cout << "stateDesired: " << stateDesired.size() << std::endl;
      // std::cout << "inputDesired: " << inputDesired.size() << std::endl;

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

      matrix_t a(info_.stateDim, numDecisionVars_);
      a.setZero();
      // A = [M, -S]
      a.block(0, 0, info_.stateDim, info_.stateDim) = data.M;          // ddq 的系数
      a.block(0, info_.stateDim, info_.stateDim, info_.armDim) = -s;         // tau_arm 的系数

      // matrix_t a = (matrix_t(info_.stateDim, numDecisionVars_) << data.M, -s.transpose())
      //                  .finished();
      vector_t b = -data.nle;

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask()
    {
      matrix_t d(2 * info_.armDim, numDecisionVars_);
      d.setZero();
      matrix_t i = matrix_t::Identity(info_.armDim, info_.armDim);
      d.block(0, info_.stateDim, info_.armDim,
              info_.armDim) = i;
      d.block(info_.armDim, info_.stateDim, info_.armDim,
              info_.armDim) = -i;
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
      b = lowJointKp_.cwiseProduct(qDesired_.tail(info_.armDim).head(lowJoint_nums_) - qMeasured_.tail(info_.armDim).head(lowJoint_nums_)) + 
          lowJointKd_.cwiseProduct(vDesired_.tail(info_.armDim).head(lowJoint_nums_) - vMeasured_.tail(info_.armDim).head(lowJoint_nums_));

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
      b = armKp.cwiseProduct(qDesired_.tail(arm_nums_) - qMeasured_.tail(arm_nums_)) +
          armKd.cwiseProduct(vDesired_.tail(arm_nums_) - vMeasured_.tail(arm_nums_));

      // qMeasured_; vMeasured_; info_.generalizedCoordinatesNum; 
      return {a, b, matrix_t(), vector_t()};
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
      loadData::loadEigenMatrix(taskFile, prefix + "kp", lowJointKp_);
      loadData::loadEigenMatrix(taskFile, prefix + "kd", lowJointKd_);

      if(arm_nums_ != 0){
        prefix = "armAccelTask.";
        if(verbose)
        {
          std::cerr << "\n #### arm Accel Task:";
          std::cerr << "\n #### =============================================================================\n";
        }
        armJointKp_.resize(arm_nums_);
        armJointKd_.resize(arm_nums_);
        loadData::loadEigenMatrix(taskFile, prefix + "kp", armJointKp_);
        loadData::loadEigenMatrix(taskFile, prefix + "kd", armJointKd_);

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
