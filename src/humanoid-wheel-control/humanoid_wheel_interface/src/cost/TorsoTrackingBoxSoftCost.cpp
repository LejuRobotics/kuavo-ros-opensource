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
#include "humanoid_wheel_interface/cost/TorsoTrackingBoxSoftCost.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TorsoTrackingBoxSoftCost::TorsoTrackingBoxSoftCost(const EndEffectorKinematics<scalar_t>& endEffectorKinematicsTorso,
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 const Vector6d& pose_lower, 
                                                 const Vector6d& pose_upper,
                                                 RelaxedBarrierPenalty::Config settingsFocus, 
                                                 RelaxedBarrierPenalty::Config settingsUnFocus)
    : endEffectorKinematicsTorsoPtr_(endEffectorKinematicsTorso.clone()),
      referenceManager_(referenceManager),
      pose_lower_(pose_lower), pose_upper_(pose_upper),
      settingsFocus_(settingsFocus),
      settingsUnFocus_(settingsUnFocus),
      penaltyFocusPtr_(new ocs2::RelaxedBarrierPenalty(settingsFocus)),
      penaltyUnFocusPtr_(new ocs2::RelaxedBarrierPenalty(settingsUnFocus))
{
  // 检查是否每个维度都是合理的（lower <= upper）
  for (int i = 0; i < pose_lower_.size(); ++i) {
    if (pose_lower_[i] > pose_upper_[i]) {
      throw std::invalid_argument(
          "[TorsoTrackingBoxSoftCost] pose_lower must be less than or equal to pose_upper! "
          "Dimension " + std::to_string(i) + ": " + 
          "lower = " + std::to_string(pose_lower_[i]) + 
          ", upper = " + std::to_string(pose_upper_[i]));
    }
  }

  pinocchioEEKinTorsoPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsTorsoPtr_.get());
}

bool TorsoTrackingBoxSoftCost::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableTorsoPoseTargetTrajectories();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t TorsoTrackingBoxSoftCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                            const ocs2::PreComputation& preComp) const {
   // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  // const auto& currentPositionOrientation = preCompMM.getTorsoPose();
  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);

  // 位置上下界
  scalar_t cost(0.0);
  Eigen::Vector3d actualPos = endEffectorKinematicsTorsoPtr_->getPosition(state).front();

  matrix_t A_tmp = matrix_t::Identity(6, 6);

  vector_t h_tmp = vector_t::Zero(6);
  h_tmp << actualPos - desiredPositionOrientationWorld.first - pose_lower_.head<3>(),   // 下界
          -actualPos + desiredPositionOrientationWorld.first + pose_upper_.head<3>();  // 上界

  vector_t f_tmp = vector_t::Zero(6);
  f_tmp << actualPos, -actualPos;

  LinearStateInequalitySoftConstraint constraint;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraint.penalty = penaltyUnFocusPtr_.get();
  }
  else
  {
    constraint.penalty = penaltyFocusPtr_.get();
  }
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getValue(constraint, f_tmp);

  // 姿态上下界
  scalar_t costOri(0.0);
  Eigen::Vector3d OriErr = endEffectorKinematicsTorsoPtr_->getOrientationError(state, {desiredPositionOrientationWorld.second}).front();
  matrix_t A_ori_tmp = matrix_t::Identity(6, 6);

  vector_t h_ori_tmp = vector_t::Zero(6);
  h_ori_tmp << OriErr - pose_lower_.tail<3>(), // 下界
              -OriErr + pose_upper_.tail<3>(); // 上界
  
  vector_t f_ori_tmp = vector_t::Zero(6);
  f_ori_tmp << OriErr, -OriErr;

  LinearStateInequalitySoftConstraint constraintOri;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraintOri.penalty = penaltyUnFocusPtr_.get();
  }
  else
  {
    constraintOri.penalty = penaltyFocusPtr_.get();
  }
  constraintOri.A = A_tmp;
  constraintOri.h = h_ori_tmp;
  
  costOri = ocs2::mobile_manipulator::getValue(constraintOri, f_ori_tmp);

  return cost + costOri;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation TorsoTrackingBoxSoftCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const ocs2::TargetTrajectories& targetTrajectories,
                                                                                         const ocs2::PreComputation& preComp) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);

  // 位置上下界
  ScalarFunctionQuadraticApproximation cost;
  const auto eePositionTorso = endEffectorKinematicsTorsoPtr_->getPositionLinearApproximation(state).front();

  matrix_t A_tmp = matrix_t::Identity(6, 6);

  vector_t h_tmp = vector_t::Zero(6);
  h_tmp <<  eePositionTorso.f - desiredPositionOrientationWorld.first - pose_lower_.head<3>(),   // 下界
           -eePositionTorso.f + desiredPositionOrientationWorld.first + pose_upper_.head<3>();  // 上界

  vector_t f_tmp = vector_t::Zero(6);
  f_tmp << eePositionTorso.f, -eePositionTorso.f;

  matrix_t dfdx_tmp = matrix_t::Zero(6, state.rows());
  dfdx_tmp << eePositionTorso.dfdx, -eePositionTorso.dfdx;

  LinearStateInequalitySoftConstraint constraint;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraint.penalty = penaltyUnFocusPtr_.get();
  }
  else
  {
    constraint.penalty = penaltyFocusPtr_.get();
  }
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getQuadraticApproximation(constraint, f_tmp, dfdx_tmp);

  // 姿态上下界
  ScalarFunctionQuadraticApproximation costOri;
  const auto eeOrientationError = 
      endEffectorKinematicsTorsoPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientationWorld.second}).front();

  vector_t h_ori_tmp = vector_t::Zero(6);
  h_ori_tmp << eeOrientationError.f - pose_lower_.tail<3>(),   // 下界
              -eeOrientationError.f + pose_upper_.tail<3>();  // 上界

  vector_t f_ori_tmp = vector_t::Zero(6);
  f_ori_tmp << eeOrientationError.f, -eeOrientationError.f;

  matrix_t dfdx_ori_tmp = matrix_t::Zero(6, state.rows());
  dfdx_ori_tmp << eeOrientationError.dfdx, -eeOrientationError.dfdx;

  LinearStateInequalitySoftConstraint constraintOri;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraintOri.penalty = penaltyUnFocusPtr_.get();
  }
  else
  {
    constraintOri.penalty = penaltyFocusPtr_.get();
  }
  constraintOri.A = A_tmp;
  constraintOri.h = h_ori_tmp;

  costOri = ocs2::mobile_manipulator::getQuadraticApproximation(constraintOri, f_ori_tmp, dfdx_ori_tmp);

  cost.f += costOri.f;
  cost.dfdx += costOri.dfdx;
  cost.dfdxx += costOri.dfdxx;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TorsoTrackingBoxSoftCost::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManager_.getTorsoTargetTrajectories();
  const auto& targetTorsoState = targetTrajectories.getDesiredState(time);

  vector_t position = targetTorsoState.head(3);
  Eigen::Vector3d zyx = targetTorsoState.segment(3, 3);
  quaternion_t orientation =   // 轮臂躯干采用 pitch-yaw-roll 欧拉角, 可以节省一个角度来表达姿态
         Eigen::AngleAxis<scalar_t>(zyx(1), Eigen::Matrix<scalar_t, 3, 1>::UnitY()) *
         Eigen::AngleAxis<scalar_t>(zyx(0), Eigen::Matrix<scalar_t, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<scalar_t>(zyx(2), Eigen::Matrix<scalar_t, 3, 1>::UnitX());

  return {position, orientation};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TorsoTrackingBoxSoftCost::targetBaseToWorld(const vector_t& state, 
              const std::pair<vector_t, quaternion_t>& targetBase) const -> std::pair<vector_t, quaternion_t> 
{
  vector_t position = vector_t::Zero(3);
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

}  // namespace mobile_manipulator
}  // namespace ocs2
