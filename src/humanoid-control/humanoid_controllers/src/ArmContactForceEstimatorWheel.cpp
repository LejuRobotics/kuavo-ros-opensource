#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/ArmContactForceEstimatorWheel.h"

#include <algorithm>
#include <cmath>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace humanoidController_wheel_wbc {

ArmContactForceEstimatorWheel::ArmContactForceEstimatorWheel(
    std::shared_ptr<PinocchioInterface> pinocchioInterface,
    const mobile_manipulator::ManipulatorModelInfo& manipulatorModelInfo, size_t lowJointNum,
    humanoid::TopicLogger* rosLogger)
    : pinocchioInterface_ptr_(std::move(pinocchioInterface)),
      manipulatorModelInfo_(manipulatorModelInfo),
      lowJointNum_(lowJointNum),
      ros_logger_(rosLogger) {
  estArmContactforce_.resize(6 * 2);
  estArmContactforce_.setZero();
  estArmContactforceLast_.resize(6 * 2);
  estArmContactforceLast_.setZero();

  if (manipulatorModelInfo_.armDim > 0) {
    cmdTorque_.resize(manipulatorModelInfo_.armDim);
    cmdTorque_.setZero();
  } else {
    cmdTorque_.resize(0);
  }

  if (manipulatorModelInfo_.armDim > lowJointNum_ && ((manipulatorModelInfo_.armDim - lowJointNum_) % 2 == 0)) {
    armDofPerSide_ = (manipulatorModelInfo_.armDim - lowJointNum_) / 2;
  }
}

void ArmContactForceEstimatorWheel::setCmdTorque(const vector_t& cmdTorque) { cmdTorque_ = cmdTorque; }

const vector_t& ArmContactForceEstimatorWheel::getEstimatedArmContactForce() const { return estArmContactforce_; }

vector6_t ArmContactForceEstimatorWheel::dlsSolve6(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                                    double lambda) {
  // A: (m x 6), b: (m)
  Eigen::Matrix<scalar_t, 6, 6> AtA = A.transpose() * A;
  const scalar_t l2 = static_cast<scalar_t>(lambda * lambda);
  AtA.diagonal().array() += l2;
  vector6_t Atb = A.transpose() * b;
  // Use LDLT for symmetric positive definite / semidefinite + λ²I
  return AtA.ldlt().solve(Atb);
}

void ArmContactForceEstimatorWheel::update(const vector_t& state, const vector_t& input,
                                           const ros::Duration& period) {
  estArmContactForce_wheel(state, input, period);
}

// 轮臂机器人手臂末端力估计
void ArmContactForceEstimatorWheel::estArmContactForce_wheel(const vector_t& state, const vector_t& input,
                                                             const ros::Duration& period) {
  // 安全检查：确保pinocchioInterface_ptr_已初始化
  if (!pinocchioInterface_ptr_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[estArmContactForce_wheel] pinocchioInterface_ptr_ is null, skipping");
    return;
  }

  if (estArmContactforce_.size() == 0) {
    estArmContactforce_.resize(6 * 2);
    estArmContactforce_.setZero();
    estArmContactforceLast_.resize(6 * 2);
    estArmContactforceLast_.setZero();
  }

  scalar_t dt = period.toSec();
  if (dt > 0.010 || dt < 1e-5) {
    dt = 0.002;
  }

  // 动量微分法系数
  const scalar_t lamda = cutoffFrequency_;
  const scalar_t gama = exp(-lamda * dt);
  const scalar_t beta = (1 - gama) / (gama * dt);
  // 检查beta是否为有效值（避免除以0）
  if (!std::isfinite(beta) || beta < 0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[estArmContactForce_wheel] Invalid beta value, skipping");
    return;
  }

  const auto& model = pinocchioInterface_ptr_->getModel();
  auto& data = pinocchioInterface_ptr_->getData();

  // 使用Pinocchio模型的真实维度（manipulatorModelInfo_.stateDim = model.nq）
  // 对于轮臂机器人，Pinocchio模型结构：3DOF基座(x, y, yaw) + 所有关节(4+7*2=18) = 21
  size_t generalizedCoordinatesNum = manipulatorModelInfo_.stateDim;  // 应该等于 model.nq = 21
  size_t actuatedDofNum = manipulatorModelInfo_.armDim;               // 包含所有关节（下肢+手臂）= 18
  size_t baseDof = 3;                                                  // 轮臂机器人基座只有3个DOF（x, y, yaw）

  // 维度检查（添加更严格的检查，避免内存错误）
  if (generalizedCoordinatesNum == 0 || actuatedDofNum == 0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[estArmContactForce_wheel] 维度为0，跳过力估计");
    return;
  }

  if (static_cast<int>(model.nq) != static_cast<int>(generalizedCoordinatesNum) ||
      static_cast<int>(model.nv) != static_cast<int>(generalizedCoordinatesNum) ||
      state.size() != baseDof + actuatedDofNum || cmdTorque_.size() != actuatedDofNum) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "[estArmContactForce_wheel] 维度不匹配，跳过本次手臂接触力估计："
                                 << " model(nq,nv)=(" << model.nq << "," << model.nv << ")"
                                 << " stateDim=" << generalizedCoordinatesNum << " armDim=" << actuatedDofNum
                                 << " observation.state.size=" << state.size()
                                 << " cmdTorque.size=" << cmdTorque_.size());
    return;
  }

  // 状态转换：轮臂机器人的Pinocchio模型直接使用observation_wheel_.state
  // qMeasured: [x, y, yaw, 关节角度] - 与observation_wheel_.state结构一致
  auto qMeasured = state;  // 直接使用，无需转换
  auto vMeasured = input;  // 直接使用，无需转换

  // 计算手臂起始索引
  // 轮臂机器人：基座3DOF + 下肢lowJointNum_ = 3 + 4 = 7，手臂从7开始
  const size_t waistNum = 0;                         // 轮臂机器人没有腰部
  const size_t legDof = lowJointNum_;               // 下肢DOF数 = 4
  const size_t totalArmDof = actuatedDofNum - legDof;  // 手臂总DOF = 14
  armDofPerSide_ = totalArmDof / 2;                    // 每侧手臂DOF = 7
  const size_t leftArmStartIdx = baseDof + waistNum + legDof;  // 3(基座) + 0(腰部) + 4(下肢) = 7
  const size_t rightArmStartIdx = leftArmStartIdx + armDofPerSide_;  // 7 + 7 = 14

  // 动力学计算
  // 选择矩阵s：前baseDof列（基座）设为0，从baseDof列开始设置单位矩阵（关节部分）
  matrix_t s(actuatedDofNum, generalizedCoordinatesNum);
  s.block(0, 0, actuatedDofNum, baseDof).setZero();                   // 前3列（基座）设为0
  s.block(0, baseDof, actuatedDofNum, actuatedDofNum).setIdentity();  // 从第3列开始设置18x18单位矩阵

  pinocchio::forwardKinematics(model, data, qMeasured, vMeasured);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);

  pinocchio::crba(model, data, qMeasured);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::getCoriolisMatrix(model, data);
  pinocchio::computeGeneralizedGravity(model, data, qMeasured);

  // 动量微分法计算惯性项
  vector_t p = data.M * vMeasured;
  vector_t pSCg = beta * p + s.transpose() * cmdTorque_ + data.C.transpose() * vMeasured - data.g;

  // 动力学项pSCg滑窗平滑
  pSCgArmSmoothWindow_.push_back(pSCg);
  if (pSCgArmSmoothWindow_.size() > pSCgSmoothWindowSize_) {
    pSCgArmSmoothWindow_.pop_front();
  }

  Eigen::VectorXd pSCg_smooth = Eigen::VectorXd::Zero(pSCg.size());
  if (!pSCgArmSmoothWindow_.empty()) {
    for (const auto& pSCg_val : pSCgArmSmoothWindow_) {
      pSCg_smooth += pSCg_val;
    }
    pSCg_smooth /= pSCgArmSmoothWindow_.size();
  } else {
    pSCg_smooth = pSCg;  // 如果窗口为空，直接使用当前值
  }

  // 低通滤波
  if (pSCgArmZinvlast_.size() != generalizedCoordinatesNum) {
    pSCgArmZinvlast_.resize(generalizedCoordinatesNum);
    pSCgArmZinvlast_.setZero();
  }
  vector_t pSCg_z_inv = (1 - gama) * pSCg_smooth + gama * pSCgArmZinvlast_;
  pSCgArmZinvlast_ = pSCg_z_inv;

  // 计算扰动力矩
  vector_t estDisturbancetorque = beta * p - pSCg_z_inv;

  // 调试输出
  if (ros_logger_) {
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/qMeasured", qMeasured);
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/vMeasured", vMeasured);
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/tauCmd", cmdTorque_);
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/estDisturbancetorque", estDisturbancetorque);
  }

  // 手臂末端力估计主循环（左右臂）
  std::vector<std::string> eeNames = {"zarm_l7_end_effector", "zarm_r7_end_effector"};
  std::vector<size_t> armStartIdxs = {leftArmStartIdx, rightArmStartIdx};
  std::vector<double> jacobian_conditions(2, 1.0);

  for (size_t i = 0; i < 2; ++i) {
    // 末端Frame合法性校验
    const auto armFrameId = model.getFrameId(eeNames[i]);
    if (armFrameId < 0) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[estArmContactForce_wheel] Frame not found: " << eeNames[i]);
      continue;
    }
    size_t armStartIdx = armStartIdxs[i];
    if (armStartIdx + armDofPerSide_ > generalizedCoordinatesNum) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[estArmContactForce_wheel] Arm start index out of range: " << armStartIdx);
      continue;
    }

    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, armFrameId, pinocchio::LOCAL_WORLD_ALIGNED, jac);

    // 构造选择矩阵（只选择对应手臂的关节）
    matrix_t S_li_arm = matrix_t::Zero(armDofPerSide_, generalizedCoordinatesNum);
    S_li_arm.block(0, armStartIdx, armDofPerSide_, armDofPerSide_) =
        matrix_t::Identity(armDofPerSide_, armDofPerSide_);

    // S_JT = S_li * J^T
    Eigen::Matrix<scalar_t, Eigen::Dynamic, 6> S_JT = S_li_arm * jac.transpose();
    vector_t S_tau = S_li_arm * estDisturbancetorque;

    // 调试输出
    if (ros_logger_) {
      std::string arm_side = (i == 0) ? "left" : "right";
      ros_logger_->publishValue("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/armStartIdx",
                                static_cast<double>(armStartIdx));
      ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/S_tau", S_tau);
    }

    // SVD奇异值/条件数
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_sjt(S_JT, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto s = svd_sjt.singularValues();
    const double maxS = (s.size() > 0) ? s(0) : 0.0;
    const double minS = (s.size() > 0) ? s(s.size() - 1) : 0.0;
    const double condNum = (minS > 1e-12) ? (maxS / minS) : 1e12;
    jacobian_conditions[i] = condNum;

    if (ros_logger_) {
      std::string arm_side = (i == 0) ? "left" : "right";
      ros_logger_->publishValue("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/jacobian_condition",
                                condNum);
      ros_logger_->publishValue("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/max_singular_value", maxS);
      ros_logger_->publishValue("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/min_singular_value", minS);
    }

    // 自适应阻尼计算
    const bool cond_trigger = (condNum > armJacCondThresh_);
    const bool minS_trigger = (minS < armJacMinSingularThresh_);
    const bool in_singular_damping = (cond_trigger || minS_trigger);

    double lambda = 0.0;
    if (in_singular_damping) {
      // 非线性阻尼增长
      double singularRatio = std::max(1e-3, minS / armJacMinSingularThresh_);
      lambda = armDampingLambda0_ * exp(armDampingGain_ * (1.0 / singularRatio - 1.0));
      lambda = std::min(lambda, 0.08);  // 阻尼上限
    }

    // 求解接触力
    vector6_t F_estimated = in_singular_damping ? dlsSolve6(S_JT, S_tau, lambda) : svd_sjt.solve(S_tau);

    if (ros_logger_) {
      std::string arm_side = (i == 0) ? "left" : "right";
      ros_logger_->publishValue("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/damping_lambda", lambda);
      ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/" + arm_side + "/F_estimated_raw",
                                 F_estimated);
    }

    // 末端力硬限幅
    F_estimated.segment<3>(0) = F_estimated.segment<3>(0).cwiseMax(-armForceMax_).cwiseMin(armForceMax_);    // 力分量
    F_estimated.segment<3>(3) = F_estimated.segment<3>(3).cwiseMax(-armMomentMax_).cwiseMin(armMomentMax_);  // 力矩分量
    estArmContactforce_.segment<6>(6 * i) = F_estimated;
  }

  // 末端力3窗长中值滤波（添加安全检查）
  if (estArmContactforce_.size() == 12) {
    armForceMedianWindow_.push_back(estArmContactforce_);
    if (armForceMedianWindow_.size() > 3) {
      armForceMedianWindow_.pop_front();
    }
    if (armForceMedianWindow_.size() == 3) {
      // 检查所有窗口元素的尺寸是否一致
      bool all_same_size = true;
      for (const auto& vec : armForceMedianWindow_) {
        if (vec.size() != 12) {
          all_same_size = false;
          break;
        }
      }
      if (all_same_size) {
        Eigen::VectorXd F_median = Eigen::VectorXd::Zero(12);
        for (int j = 0; j < 12; ++j) {
          std::vector<double> vals = {armForceMedianWindow_[0](j), armForceMedianWindow_[1](j),
                                      armForceMedianWindow_[2](j)};
          std::sort(vals.begin(), vals.end());
          F_median(j) = vals[1];  // 3窗长取中值
        }
        estArmContactforce_ = F_median;
      }
    }
  }

  // 低通滤波
  const scalar_t cutoff = 5.0;
  const scalar_t alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff));
  estArmContactforce_ = alpha * estArmContactforce_ + (1.0 - alpha) * estArmContactforceLast_;
  estArmContactforceLast_ = estArmContactforce_;

  // 调试输出：发布最终的接触力估计结果
  if (ros_logger_) {
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/estArmContactforce_final",
                               estArmContactforce_);
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/left_arm_force",
                               estArmContactforce_.segment<6>(0));
    ros_logger_->publishVector("/humanoid_wheel/arm_contact_force_debug/right_arm_force",
                               estArmContactforce_.segment<6>(6));
  }
}

}  // namespace humanoidController_wheel_wbc
