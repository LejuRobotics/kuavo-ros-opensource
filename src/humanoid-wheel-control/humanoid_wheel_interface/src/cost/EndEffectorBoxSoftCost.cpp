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
                                                 std::vector<RelaxedBarrierPenalty::Config> settingsFocus,
                                                 std::vector<RelaxedBarrierPenalty::Config> settingsUnFocus,
                                                 const int eefInx)
    : endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManager_(referenceManager),
      info_(info),
      pose_lower_(pose_lower), pose_upper_(pose_upper),
      settingsFocus_(settingsFocus),
      settingsUnFocus_(settingsUnFocus),
      eef_Idx_(eefInx)
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

  // 更新各轴屏障函数
  for(int i = 0; i < 3; i++)
  {
    penaltyFocusPtr6D_.emplace_back(new ocs2::RelaxedBarrierPenalty(settingsFocus_[0]));
    penaltyUnFocusPtr6D_.emplace_back(new ocs2::RelaxedBarrierPenalty(settingsUnFocus_[0]));
  }
  for(int i = 0; i < 3; i++)
  {
    penaltyFocusPtr6D_.emplace_back(new ocs2::RelaxedBarrierPenalty(settingsFocus_[1]));
    penaltyUnFocusPtr6D_.emplace_back(new ocs2::RelaxedBarrierPenalty(settingsUnFocus_[1]));
  }
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

  for(int i=0; i<3; i++)
  {
    // 下界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);

      h_one << actualPos.segment(i, 1) - desiredPositionOrientation.first.segment(i, 1) - pose_lower_.segment(i, 1);   // 下界
      f_one << actualPos.segment(i, 1);

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost += ocs2::mobile_manipulator::getValue(constraint_one, f_one);
    }

    // 上界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);

      h_one << -actualPos.segment(i, 1) + desiredPositionOrientation.first.segment(i, 1) + pose_upper_.segment(i, 1);   // 上界
      f_one << -actualPos.segment(i, 1);

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost += ocs2::mobile_manipulator::getValue(constraint_one, f_one);
    }
  }

  // 姿态上下界
  Eigen::Vector3d OriErr = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();

  for(int i=0; i<3; i++)
  {
    // 下界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);

      h_one << OriErr.segment(i, 1) - pose_lower_.tail<3>().segment(i, 1);   // 下界
      f_one << OriErr.segment(i, 1);

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i+3].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i+3].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost += ocs2::mobile_manipulator::getValue(constraint_one, f_one);
    }

    // 上界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);

      h_one << -OriErr.segment(i, 1) + pose_upper_.tail<3>().segment(i, 1);   // 上界
      f_one << -OriErr.segment(i, 1);

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i+3].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i+3].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost += ocs2::mobile_manipulator::getValue(constraint_one, f_one);
    }
  }

  return cost;
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
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(state.size());
  cost.dfdxx = matrix_t::Zero(state.size(), state.size());
  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  for(int i=0; i<3; i++)
  {
    // 下界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      ScalarFunctionQuadraticApproximation cost_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);
      matrix_t dfdx_one = matrix_t::Zero(1, state.rows());

      h_one << eePosition.f.segment(i, 1) - desiredPositionOrientation.first.segment(i, 1) - pose_lower_.segment(i, 1);   // 下界
      f_one << eePosition.f.segment(i, 1);
      dfdx_one << eePosition.dfdx.row(i).eval();

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost_one = ocs2::mobile_manipulator::getQuadraticApproximation(constraint_one, f_one, dfdx_one);
      cost += cost_one;
    }

    // 上界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      ScalarFunctionQuadraticApproximation cost_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);
      matrix_t dfdx_one = matrix_t::Zero(1, state.rows());

      h_one << -eePosition.f.segment(i, 1) + desiredPositionOrientation.first.segment(i, 1) + pose_upper_.segment(i, 1);   // 上界
      f_one << -eePosition.f.segment(i, 1);
      dfdx_one << -eePosition.dfdx.row(i).eval();

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost_one = ocs2::mobile_manipulator::getQuadraticApproximation(constraint_one, f_one, dfdx_one);
      cost += cost_one;
    }
  }

  // 姿态上下界
  ScalarFunctionQuadraticApproximation costOri;
  const auto eeOrientationError = 
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  
  for(int i=0; i<3; i++)
  {
    // 下界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      ScalarFunctionQuadraticApproximation cost_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);
      matrix_t dfdx_one = matrix_t::Zero(1, state.rows());

      h_one << eeOrientationError.f.segment(i, 1) - pose_lower_.tail<3>().segment(i, 1);   // 下界
      f_one << eeOrientationError.f.segment(i, 1);
      dfdx_one << eeOrientationError.dfdx.row(i).eval();

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i+3].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i+3].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost_one = ocs2::mobile_manipulator::getQuadraticApproximation(constraint_one, f_one, dfdx_one);
      cost += cost_one;
    }

    // 上界
    {
      LinearStateInequalitySoftConstraint constraint_one;
      ScalarFunctionQuadraticApproximation cost_one;
      vector_t h_one = vector_t::Zero(1);
      vector_t f_one = vector_t::Zero(1);
      matrix_t dfdx_one = matrix_t::Zero(1, state.rows());

      h_one << -eeOrientationError.f.segment(i, 1) + pose_upper_.tail<3>().segment(i, 1);   // 上界
      f_one << -eeOrientationError.f.segment(i, 1);
      dfdx_one << -eeOrientationError.dfdx.row(i).eval();

      if(referenceManager_.getIsFocusEeStatus())
      {
        constraint_one.penalty = penaltyFocusPtr6D_[i+3].get();
      }
      else
      {
        constraint_one.penalty = penaltyUnFocusPtr6D_[i+3].get();
      }
      constraint_one.A = matrix_t::Identity(1, 1);
      constraint_one.h = h_one;

      cost_one = ocs2::mobile_manipulator::getQuadraticApproximation(constraint_one, f_one, dfdx_one);
      cost += cost_one;
    }
  }

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
