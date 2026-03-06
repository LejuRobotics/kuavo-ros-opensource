/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <humanoid_wheel_interface/MobileManipulatorPreComputation.h>
#include "humanoid_wheel_interface/cost/EndEffectorLocalMultiPointBoxSoftCost.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLocalMultiPointBoxSoftCost::EndEffectorLocalMultiPointBoxSoftCost(int pointNums, 
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 PinocchioInterface pinocchioInterface, 
                                                 const ManipulatorModelInfo& info,
                                                 const vector_t& pos_lower, 
                                                 const vector_t& pos_upper,
                                                 std::vector<RelaxedBarrierPenalty::Config> settingsFocus,
                                                 std::vector<RelaxedBarrierPenalty::Config> settingsUnFocus,
                                                 const int eefInx)
    : pointNums_(pointNums), 
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManager_(referenceManager),
      pinocchioInterface_(pinocchioInterface), 
      info_(info),
      pos_lower_(pos_lower), pos_upper_(pos_upper),
      settingsFocus_(settingsFocus),
      settingsUnFocus_(settingsUnFocus),
      eef_Idx_(eefInx),
      eef_num_(info.eeFrames.size())
{
  // 检查 pos_lower 和 pos_upper 的维度是否为 pointNums * 3
  if(pos_lower_.size() != pos_upper_.size() || pos_lower_.size() != pointNums * 3)
  {
    throw std::invalid_argument(
        "[EndEffectorLocalMultiPointBoxSoftCost] pos_lower or pos_upper have incorrect dimensions");
  }

  // 检查是否每个维度都是合理的（lower <= upper）
  for (int i = 0; i < pos_lower_.size(); ++i) {
    if (pos_lower_[i] > pos_upper_[i]) {
      throw std::invalid_argument(
          "[EndEffectorLocalMultiPointBoxSoftCost] pos_lower must be less than or equal to pos_upper! "
          "Dimension " + std::to_string(i) + ": " + 
          "lower = " + std::to_string(pos_lower_[i]) + 
          ", upper = " + std::to_string(pos_upper_[i]));
    }
  }

  // 检查关键对象的数据索引是否正确
  if (endEffectorKinematics.getIds().size() != pointNums) {
    throw std::runtime_error("[EndEffectorLocalMultiPointBoxSoftCost] endEffectorKinematics has wrong number of end effector IDs.");
  }

  for (int i = 0; i < settingsFocus.size(); i++)
  {
    auto penaltyPtr = std::make_unique<ocs2::RelaxedBarrierPenalty>((settingsFocus[i]));
    penaltyVecFocusPtr_.push_back(std::move(penaltyPtr));
  }

  for (int i = 0; i < settingsUnFocus.size(); i++)
  {
    auto penaltyPtr = std::make_unique<ocs2::RelaxedBarrierPenalty>((settingsUnFocus[i]));
    penaltyVecUnFocusPtr_.push_back(std::move(penaltyPtr));
  }

  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
  start_index_ = eef_Idx_ * 7;

  // ids 第一个为主点，获取零位时，主点和副点的相关信息
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  const auto q = Eigen::VectorXd::Zero(info_.stateDim);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // 获取主点的位置
  bool verbose{false};
  const pinocchio::FrameIndex frameIdMain = model.getFrameId(endEffectorKinematics.getIds()[0]);
  Eigen::Vector3d posMain = data.oMf[frameIdMain].translation();
  Eigen::Quaterniond quatMain(data.oMf[frameIdMain].rotation());

  if(verbose)
  {
    std::cout << "pos_lower_: " << pos_lower_.transpose() << std::endl;
    std::cout << "pos_upper_: " << pos_upper_.transpose() << std::endl;
    std::cout << "posMain: " << posMain.transpose() << " quatMain: " << quatMain.coeffs().transpose() << std::endl;
  }

  // 获取副点位置, 并更新主点指向副点的 offset
  for(int i = 1; i < endEffectorKinematics.getIds().size(); i++)
  {
    const pinocchio::FrameIndex frameId = model.getFrameId(endEffectorKinematics.getIds()[i]);
    const pinocchio::SE3& pose = data.oMf[frameId];

    Eigen::Vector3d posLocal = data.oMf[frameId].translation() - posMain;
    localOffset_.push_back(posLocal);

    if(verbose)
    {
      std::cout << "ids: " << i << " posLocal: " << posLocal.transpose() << std::endl;
    }
  }
}

bool EndEffectorLocalMultiPointBoxSoftCost::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableEeTargetLocalTrajectoriesForArm(eef_Idx_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t EndEffectorLocalMultiPointBoxSoftCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                            const ocs2::PreComputation& preComp) const {
   // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  const auto& desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);
  const vector_t multiTargetPos = targetPoseToMultiPointPos(desiredPositionOrientationWorld, localOffset_);

  // 位置上下界
  scalar_t cost = 0.0;
  std::vector<Eigen::Vector3d> actualPos = endEffectorKinematicsPtr_->getPosition(state);

  for(int i = 0; i < pointNums_; i++)
  {
    matrix_t A_tmp = matrix_t::Identity(6, 6);

    vector_t h_tmp = vector_t::Zero(6);
    h_tmp << actualPos[i] - multiTargetPos.segment<3>(i * 3) - pos_lower_.segment<3>(i * 3),  // 下界
            -actualPos[i] + multiTargetPos.segment<3>(i * 3) + pos_upper_.segment<3>(i * 3);  // 上界
    
    vector_t f_tmp = vector_t::Zero(6);
    f_tmp << actualPos[i], -actualPos[i];

    LinearStateInequalitySoftConstraint constraint;
    if(referenceManager_.getIsFocusEeStatus())
    {
      constraint.penalty = penaltyVecFocusPtr_[i].get();
    }
    else
    {
      constraint.penalty = penaltyVecUnFocusPtr_[i].get();
    }
    constraint.A = A_tmp;
    constraint.h = h_tmp;

    cost += ocs2::mobile_manipulator::getValue(constraint, f_tmp);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation EndEffectorLocalMultiPointBoxSoftCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const ocs2::TargetTrajectories& targetTrajectories,
                                                                                         const ocs2::PreComputation& preComp) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  const auto& desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);
  const vector_t multiTargetPos = targetPoseToMultiPointPos(desiredPositionOrientationWorld, localOffset_);

  // 位置上下界
  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(state.size());
  cost.dfdxx = matrix_t::Zero(state.size(), state.size());

  const std::vector<VectorFunctionLinearApproximation> eePosition = 
        endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
  
  for(int i = 0; i < pointNums_; i++)
  {
    matrix_t A_tmp = matrix_t::Identity(6, 6);

    vector_t h_tmp = vector_t::Zero(6);
    h_tmp <<  eePosition[i].f - multiTargetPos.segment<3>(i * 3) - pos_lower_.segment<3>(i * 3),  // 下界
             -eePosition[i].f + multiTargetPos.segment<3>(i * 3) + pos_upper_.segment<3>(i * 3);  // 上界

    vector_t f_tmp = vector_t::Zero(6);
    f_tmp << eePosition[i].f, -eePosition[i].f;

    matrix_t dfdx_tmp = matrix_t::Zero(6, state.rows());
    dfdx_tmp << eePosition[i].dfdx, -eePosition[i].dfdx;

    LinearStateInequalitySoftConstraint constraint;
    if(referenceManager_.getIsFocusEeStatus())
    {
      constraint.penalty = penaltyVecFocusPtr_[i].get();
    }
    else
    {
      constraint.penalty = penaltyVecUnFocusPtr_[i].get();
    }
    constraint.A = A_tmp;
    constraint.h = h_tmp;

    const auto targetCost = ocs2::mobile_manipulator::getQuadraticApproximation(constraint, f_tmp, dfdx_tmp);
    cost.f += targetCost.f;
    cost.dfdx += targetCost.dfdx;
    cost.dfdxx += targetCost.dfdxx;
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorLocalMultiPointBoxSoftCost::interpolateEndEffectorPose(scalar_t time) const -> std::pair<Eigen::Vector3d, quaternion_t> {
  const auto& targetTrajectories = referenceManager_.getEeTargetTrajectories(eef_Idx_);
  const auto& targetEeState = targetTrajectories.getDesiredState(time);

  Eigen::Vector3d position = targetEeState.segment(0, 3);
  Eigen::Vector3d zyx = targetEeState.segment(3, 3);
  quaternion_t orientation = getQuaternionFromEulerAnglesZyx(zyx);

  return {position, orientation};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorLocalMultiPointBoxSoftCost::targetBaseToWorld(const vector_t& state, 
              const std::pair<Eigen::Vector3d, quaternion_t>& targetBase) const -> std::pair<Eigen::Vector3d, quaternion_t> 
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  quaternion_t orientation;

  // 提取当前基座状态 [x, y, yaw]
  Eigen::Vector2d basePos = state.head(2);  // 世界系位置 [x, y]
  scalar_t baseYaw = state(2);              // 世界系偏航角

  // 构建基座姿态的旋转矩阵 (2D)
  Eigen::Matrix2d R_base = Eigen::Rotation2D<scalar_t>(baseYaw).toRotationMatrix();

  // 位置转换：世界系位置 = 基座位置 + 旋转矩阵 * 本体系相对位置
  Eigen::Vector2d bodyPosition = targetBase.first.head(2);
  Eigen::Vector2d worldPosition = basePos + R_base * bodyPosition;

  // 姿态转换：世界系姿态 = 基座姿态 * 本体系相对姿态
  quaternion_t baseQuat(Eigen::AngleAxisd(baseYaw, Eigen::Vector3d::UnitZ()));
  quaternion_t worldQuat = baseQuat * targetBase.second;
  worldQuat.normalize();

  // 设置世界系位姿
  position.head(2) = worldPosition;
  position(2) = targetBase.first(2);
  orientation = worldQuat;

  return {position, orientation};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLocalMultiPointBoxSoftCost::targetPoseToMultiPointPos(const std::pair<Eigen::Vector3d, quaternion_t>& targetPose, 
                                                                    const std::vector<Eigen::Vector3d>& localOffset) const 
{
  const Eigen::Vector3d& posMain = targetPose.first;
  const Eigen::Matrix3d rotMain = targetPose.second.toRotationMatrix();

  vector_t multiPointPos = vector_t::Zero(3 * pointNums_);

  // 设置主点位置 (前3个元素)
  multiPointPos.segment<3>(0) = posMain;

  // 计算并设置每个副点的位置
  for(int i=0; i < localOffset.size(); i++)
  {
    // 副点位置 = 主点位置 + 旋转后的局部偏移
    multiPointPos.segment<3>(3 * (i + 1)) = posMain + rotMain * localOffset[i];
  }

  return multiPointPos;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
