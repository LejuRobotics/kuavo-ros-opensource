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

#include "wheel_arm_abs_vel/humanoid_wheel_wbc/absolute/AbsoluteWbcBase.h"

#include "humanoid_wheel_interface/AccessHelperFunctions.h"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <utility>
#include <nlohmann/json.hpp>
#include "wheel_arm_abs_vel/motion_capture_ik/runtime/E2eTrace.h"
namespace { kuavo_e2e::Trace<256> wbcAudit("wbc"); }
namespace { kuavo_e2e::Trace<384> wbcTaskAudit("wbc_task_v2"); }


namespace ocs2
{
  namespace mobile_manipulator
  {
    // stateDesired/inputDesired 与 MPC 向量同布局：head(3)=base(x,y,yaw)，[3:7]=下肢4关节，tail(14)=双臂
    AbsoluteWbcBase::AbsoluteWbcBase(const PinocchioInterface &pinocchioInterface, const ManipulatorModelInfo& info)
        : WbcBase(pinocchioInterface, info)
    {
      // Shared WbcBase constructs the model and owns the original limit state.
      ros::param::param("/vr_ik/wbc_retain_lower_arm_coupling", retainLowerArmCoupling_, false);
      const auto& model = pinocchioInterfaceMeasured_.getModel();
      if(wbcAudit.enabled()) {
        nlohmann::json snapshot;snapshot["binding"]="AbsoluteContactForceWbc actual constructor model; measured crba(q), nonLinearEffects(q,v)";
        snapshot["nq"]=model.nq;snapshot["nv"]=model.nv;snapshot["base"]=info_.baseFrame;snapshot["torso"]=info_.torsoFrame;snapshot["eeFrames"]=info_.eeFrames;
        for(size_t j=1;j<model.joints.size();++j){auto I=model.inertias[j];std::vector<double> c(I.lever().data(),I.lever().data()+3);Eigen::Matrix3d im=I.inertia().matrix();std::vector<double> v(im.data(),im.data()+9);snapshot["joints"].push_back({{"name",model.names[j]},{"idx_q",model.joints[j].idx_q()},{"nq",model.joints[j].nq()},{"mass",I.mass()},{"com",c},{"inertia",v}});}
        std::ofstream(std::string(std::getenv("KUAVO_E2E_TRACE_DIR"))+"/wbc_model.json")<<snapshot.dump(2);
      }

      std::cout << "[wbcBase] model.nq: " << model.nq << std::endl;
      std::cout << "[wbcBase] model.nv: " << model.nv << std::endl;
      std::cout << "[wbcBase] contact_force_size_: " << contact_force_size_ << std::endl;
    }

    vector_t AbsoluteWbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const SystemObservation& observation)
    {
      taskAuditReady_=false;
      ++taskAuditCycle_;
      updateMeasured(observation.state, observation.input);
      updateDesired(stateDesired, inputDesired);

      return {};
    }

    void AbsoluteWbcBase::updateMeasured(const vector_t &stateMeasured, const vector_t &inputMeasured)
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

    void AbsoluteWbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
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
      pinocchio::crba(model, data, qDesired_);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      pinocchio::nonLinearEffects(model, data, qDesired_, vDesired_);
    }


    void AbsoluteWbcBase::writeE2eAudit(const vector_t& x,int status,int iterations,double solveTime) {
      if(wbcTaskAudit.enabled() && taskAuditReady_ && x.size()>=info_.armDim) {
        auto& r=taskAuditRow_;
        r[4]=status;r[5]=iterations;r[6]=solveTime;r[7]=wbcTaskAudit.dropped();
        r[30]=kuavo_e2e::ns();
        for(int j=0;j<14;++j) r[64+21*j+20]=x[x.size()-14+j];
        wbcTaskAudit.push(r);taskAuditReady_=false;
      }
      if(!wbcAudit.enabled()||info_.stateDim!=21||info_.armDim!=18||x.size()<51)return;
      auto& d=pinocchioInterfaceMeasured_.getData();kuavo_e2e::Trace<256>::Row r{};
      static uint64_t seq=0;r[0]=1;r[1]=++seq;r[2]=kuavo_e2e::ns();r[3]=ros::Time::now().toSec();r[4]=21;r[5]=18;r[6]=jointAccelerationFeedforwardValid_ ? 1.0 : 0.0;r[7]=(useVrArmAccelTask_&&hasVrArmAccelTask_) ? 1.0 : 0.0;r[8]=iterations;r[9]=status;r[10]=solveTime;r[11]=wbcAudit.dropped();
      for(int j=0;j<21;++j){r[16+j]=qMeasured_[j];r[37+j]=vMeasured_[j];r[58+j]=qDesired_[j];r[79+j]=vDesired_[j];r[136+j]=x[j];r[187+j]=d.nle[j];r[208+j]=d.M(j,j);}
      const auto& kp=(useVrArmAccelTask_&&hasVrArmAccelTask_)?vrArmJointKp_:armJointKp_;
      const auto& kd=(useVrArmAccelTask_&&hasVrArmAccelTask_)?vrArmJointKd_:armJointKd_;
      for(int j=0;j<18;++j){double ff=j<4?(jointAccelerationFeedforwardValid_?lowJointAccelerationFeedforward_[j]:0):(armAccelerationFeedforwardValid_?armAccelerationFeedforward_[j-4]:0);double k=j<4?lowJointKp_[j]:kp[j-4],v=j<4?lowJointKd_[j]:kd[j-4];r[100+j]=ff;r[118+j]=ff+k*(qDesired_[j+3]-qMeasured_[j+3])+v*(vDesired_[j+3]-vMeasured_[j+3]);r[157+j]=x[x.size()-18+j];double removed=0;for(int c=0;c<21;++c)if(c<3||(!retainLowerArmCoupling_&&((j<4&&c>=7)||(j>=4&&c>=3&&c<7))))removed+=d.M(j+3,c)*x[c];r[229+j]=removed;}
      for(int j=0;j<12;++j)r[175+j]=x[21+j];wbcAudit.push(r);
    }
    Task AbsoluteWbcBase::formulateFloatingBaseEomTask()
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
      // Fixed base reactions remain separate. Moving torso and arms share one
      // inertia matrix in the physical plant; retain those terms when enabled.
      if (!retainLowerArmCoupling_) {
        M.block(base_dim, base_dim + lowJoint_nums_, lowJoint_nums_, arm_nums_).setZero();
        M.block(base_dim + lowJoint_nums_, base_dim, arm_nums_, lowJoint_nums_).setZero();
      }

      a.block(0, 0, info_.stateDim, info_.stateDim) = M;  // ddq的系数
      a.block(0, info_.stateDim, info_.stateDim, contact_force_size_) = -j_contact; // f_contact的系数
      a.block(0, info_.stateDim + contact_force_size_, info_.stateDim, info_.armDim) = -s; // tau的系数

      vector_t b = -data.nle;

      return {a, b, matrix_t(), vector_t()};
    }



    Task AbsoluteWbcBase::formulateLowJointAccelTask()
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
      // 增量式 VR 时跳过误差限幅，恢复为直接 PD（与 1.4.4 及更早一致）
      const bool applyErrorClamp = is_real_ && !useVrArmAccelTask_;
      if(applyErrorClamp) processLowJointErrorsWithSafe(pos_error, vel_error);   // 真实环境下进行故障检测和限幅处理

      // 计算 PD 控制量并限幅
      const vector_t acceleration_feedforward =
          jointAccelerationFeedforwardValid_
              ? lowJointAccelerationFeedforward_
              : vector_t::Zero(lowJoint_nums_);
      b = acceleration_feedforward + lowJointKp_.cwiseProduct(pos_error) +
          lowJointKd_.cwiseProduct(vel_error);
      b = b.cwiseMax(-20.0).cwiseMin(20.0);  // 加速度限幅 [-20, 20]

      return {a, b, matrix_t(), vector_t()};
    }

    Task AbsoluteWbcBase::formulateArmJointAccelTask()
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
      taskAuditJump_.fill(false);
      if(wbcTaskAudit.enabled() && arm_nums_==14) {
        taskAuditRow_.fill(0.0);
        for(int j=0;j<14;++j) {
          taskAuditRow_[64+21*j+6]=pos_error[j];
          taskAuditRow_[64+21*j+7]=vel_error[j];
          taskAuditRow_[64+21*j+18]=qMeasured_.tail(14)[j];
          taskAuditRow_[64+21*j+19]=vMeasured_.tail(14)[j];
        }
      }
      // 增量式 VR 时跳过误差限幅，恢复为直接 PD（与 1.4.4 及更早一致）
      const bool applyErrorClamp = is_real_ && !useVrArmAccelTask_;
      if(applyErrorClamp) processArmJointErrorsWithSafe(pos_error, vel_error);   // 真实环境下进行故障检测和限幅处理

      // q/dq/ddq are accepted atomically. Feedback remains active for model and
      // execution error; legacy references use zero acceleration feedforward.
      const vector_t acceleration_feedforward =
          armAccelerationFeedforwardValid_
              ? armAccelerationFeedforward_
              : vector_t::Zero(arm_nums_);
      b = acceleration_feedforward + armKp.cwiseProduct(pos_error) +
          armKd.cwiseProduct(vel_error);
      if(wbcTaskAudit.enabled() && arm_nums_==14) {
        auto& r=taskAuditRow_;r[0]=2;r[1]=taskAuditCycle_;
        r[2]=kuavo_e2e::ns();r[3]=ros::Time::now().toSec();
        r[8]=useVrArmPd?1:(taskAuditForcePd_?2:0);
        r[9]=applyErrorClamp;r[10]=armAccelerationFeedforwardValid_;
        r[11]=taskAuditArmWeight_;
        for(int i=0;i<10;++i)r[16+i]=taskAuditReference_[i];
        for(int j=0;j<14;++j) {
          const int o=64+21*j;
          r[o]=qMeasured_.tail(14)[j];r[o+1]=vMeasured_.tail(14)[j];
          r[o+2]=qDesired_.tail(14)[j];r[o+3]=vDesired_.tail(14)[j];
          r[o+4]=armKp[j];r[o+5]=armKd[j];
          r[o+8]=pos_error[j];r[o+9]=vel_error[j];
          r[o+10]=acceleration_feedforward[j];
          r[o+11]=armKp[j]*pos_error[j];r[o+12]=armKd[j]*vel_error[j];
          r[o+13]=b[j];r[o+15]=taskAuditJump_[j];
          r[o+16]=taskAuditForcePd_;r[o+17]=taskAuditReference_[9];
        }
      }
      b = b.cwiseMax(-300.0).cwiseMin(300.0);  // 加速度限幅 [-300, 300]
      if(wbcTaskAudit.enabled() && arm_nums_==14) {
        for(int j=0;j<14;++j)taskAuditRow_[64+21*j+14]=b[j];
        taskAuditReady_=true;
      }

      return {a, b, matrix_t(), vector_t()};
    }

    void AbsoluteWbcBase::processArmJointErrorsWithSafe(vector_t& pos_error, vector_t& vel_error)
    {
      const vector_t before = qMeasured_.tail(arm_nums_);
      WbcBase::processArmJointErrorsWithSafe(pos_error, vel_error);
      // Observe the shared safety correction; no duplicated limits or fault policy.
      for (size_t i = 0; i < std::min<size_t>(arm_nums_, taskAuditJump_.size()); ++i)
        taskAuditJump_[i] = before[i] != qMeasured_.tail(arm_nums_)[i];
    }



    Task AbsoluteWbcBase::formulateBaseAccTask()
    {
      matrix_t a(info_.stateDim-info_.armDim, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();

      a.block(0, 0, info_.stateDim-info_.armDim, info_.stateDim-info_.armDim) = matrix_t::Identity(info_.stateDim-info_.armDim, info_.stateDim-info_.armDim);

      return {a, b, matrix_t(), vector_t()};
    }

    Task AbsoluteWbcBase::formulateTorsoZeroAccTask()
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

    void AbsoluteWbcBase::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      WbcBase::loadTasksSetting(taskFile, verbose, is_real);
    }

    Eigen::Vector3d AbsoluteWbcBase::bodyToWorldVelocity(double v_body, double vyaw_body, double yaw)
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
