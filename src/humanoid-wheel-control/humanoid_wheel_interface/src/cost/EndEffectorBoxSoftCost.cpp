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
#include "humanoid_wheel_interface/cost/EndEffectorBoxSoftCost.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorBoxSoftCost::EndEffectorBoxSoftCost(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 const ManipulatorModelInfo& info,
                                                 const Vector6d& pose_lower, 
                                                 const Vector6d& pose_upper,
                                                 RelaxedBarrierPenalty::Config settingsFocus,
                                                 RelaxedBarrierPenalty::Config settingsUnFocus,
                                                 const int eefInx)
    : endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManager_(referenceManager),
      info_(info),
      pose_lower_(pose_lower), pose_upper_(pose_upper),
      settingsFocus_(settingsFocus),
      settingsUnFocus_(settingsUnFocus),
      penaltyFocusPtr_(new ocs2::RelaxedBarrierPenalty(settingsFocus)),
      penaltyUnFocusPtr_(new ocs2::RelaxedBarrierPenalty(settingsUnFocus)),
      eef_Idx_(eefInx),
      eef_num_(info.eeFrames.size())
{
  // 检查是否每个维度都是合理的（lower <= upper）
  for (int i = 0; i < pose_lower_.size(); ++i) {
    if (pose_lower_[i] > pose_upper_[i]) {
      throw std::invalid_argument(
          "[EndEffectorBoxSoftCost] pose_lower must be less than or equal to pose_upper! "
          "Dimension " + std::to_string(i) + ": " + 
          "lower = " + std::to_string(pose_lower_[i]) + 
          ", upper = " + std::to_string(pose_upper_[i]));
    }
  }

  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
  start_index_ = eef_Idx_ * 7;
}

bool EndEffectorBoxSoftCost::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableEeTargetTrajectoriesForArm(eef_Idx_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t EndEffectorBoxSoftCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                            const ocs2::PreComputation& preComp) const {
   // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);

  // 位置上下界
  scalar_t cost(0.0);
  Eigen::Vector3d actualPos = endEffectorKinematicsPtr_->getPosition(state).front();

  matrix_t A_tmp = matrix_t::Identity(6, 6);

  vector_t h_tmp = vector_t::Zero(6);
  h_tmp << actualPos - desiredPositionOrientation.first - pose_lower_.head<3>(),   // 下界
          -actualPos + desiredPositionOrientation.first + pose_upper_.head<3>();  // 上界

  vector_t f_tmp = vector_t::Zero(6);
  f_tmp << actualPos, -actualPos;

  LinearStateInequalitySoftConstraint constraint;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraint.penalty = penaltyFocusPtr_.get();
  }
  else
  {
    constraint.penalty = penaltyUnFocusPtr_.get();
  }
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getValue(constraint, f_tmp);

  // 姿态上下界
  scalar_t costOri(0.0);
  Eigen::Vector3d OriErr = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
  matrix_t A_ori_tmp = matrix_t::Identity(6, 6);

  vector_t h_ori_tmp = vector_t::Zero(6);
  h_ori_tmp << OriErr - pose_lower_.tail<3>(), // 下界
              -OriErr + pose_upper_.tail<3>(); // 上界
  
  vector_t f_ori_tmp = vector_t::Zero(6);
  f_ori_tmp << OriErr, -OriErr;

  LinearStateInequalitySoftConstraint constraintOri;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraintOri.penalty = penaltyFocusPtr_.get();
  }
  else
  {
    constraintOri.penalty = penaltyUnFocusPtr_.get();
  }
  constraintOri.A = A_tmp;
  constraintOri.h = h_ori_tmp;
  
  costOri = ocs2::mobile_manipulator::getValue(constraintOri, f_ori_tmp);

  return cost + costOri;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation EndEffectorBoxSoftCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const ocs2::TargetTrajectories& targetTrajectories,
                                                                                         const ocs2::PreComputation& preComp) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);

  // 位置上下界
  ScalarFunctionQuadraticApproximation cost;
  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  matrix_t A_tmp = matrix_t::Identity(6, 6);

  vector_t h_tmp = vector_t::Zero(6);
  h_tmp <<  eePosition.f - desiredPositionOrientation.first - pose_lower_.head<3>(),   // 下界
           -eePosition.f + desiredPositionOrientation.first + pose_upper_.head<3>();  // 上界

  vector_t f_tmp = vector_t::Zero(6);
  f_tmp << eePosition.f, -eePosition.f;

  matrix_t dfdx_tmp = matrix_t::Zero(6, state.rows());
  dfdx_tmp << eePosition.dfdx, -eePosition.dfdx;

  LinearStateInequalitySoftConstraint constraint;
  if(referenceManager_.getIsFocusEeStatus())
  {
    constraint.penalty = penaltyFocusPtr_.get();
  }
  else
  {
    constraint.penalty = penaltyUnFocusPtr_.get();
  }
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getQuadraticApproximation(constraint, f_tmp, dfdx_tmp);

  // 姿态上下界
  ScalarFunctionQuadraticApproximation costOri;
  const auto eeOrientationError = 
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();

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
    constraintOri.penalty = penaltyFocusPtr_.get();
  }
  else
  {
    constraintOri.penalty = penaltyUnFocusPtr_.get();
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
auto EndEffectorBoxSoftCost::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManager_.getEeTargetTrajectories(eef_Idx_);
  const auto& targetEeState = targetTrajectories.getDesiredState(time);

  vector_t position = targetEeState.segment(0, 3);
  Eigen::Vector3d zyx = targetEeState.segment(3, 3);
  quaternion_t orientation = getQuaternionFromEulerAnglesZyx(zyx);

  return {position, orientation};
}

}  // namespace mobile_manipulator
}  // namespace ocs2
