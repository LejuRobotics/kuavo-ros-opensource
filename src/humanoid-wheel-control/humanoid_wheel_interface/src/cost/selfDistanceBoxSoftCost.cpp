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
#include "humanoid_wheel_interface/cost/selfDistanceBoxSoftCost.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
selfDistanceBoxSoftCost::selfDistanceBoxSoftCost(const EndEffectorKinematics<scalar_t>& endEffectorKinematicsPair,
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 RelaxedBarrierPenalty::Config settings,
                                                 const double distance)
    : endEffectorKinematicsPairPtr_(endEffectorKinematicsPair.clone()),
      distance_(distance),
      distanceSq_(distance*distance), 
      settings_(settings),
      referenceManager_(referenceManager),
      penaltyPtr_(new ocs2::RelaxedBarrierPenalty(settings))
{
  if (endEffectorKinematicsPair.getIds().size() != 2) {
    throw std::runtime_error("[selfDistanceBoxSoftCost] endEffectorKinematics has wrong number without 2.");
  }

  pinocchioEEKinPairPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPairPtr_.get());
}

bool selfDistanceBoxSoftCost::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableEeTargetLocalTrajectories() || referenceManager_.getEnableEeTargetTrajectories() ;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t selfDistanceBoxSoftCost::getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                                            const ocs2::PreComputation& preComp) const 
{
  if (pinocchioEEKinPairPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPairPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  scalar_t cost(0.0);

  std::vector<Eigen::Vector3d> actualPos = endEffectorKinematicsPairPtr_->getPosition(state);

  matrix_t A_tmp = matrix_t::Identity(1, 1);
  vector_t h_tmp = vector_t::Zero(1);
  vector_t f_tmp = vector_t::Zero(1);

  // 计算两点差值,
  Eigen::Vector3d diff_pos = actualPos[0] - actualPos[1];
  scalar_t distanceSq = diff_pos.squaredNorm();

  h_tmp << -distanceSq + distanceSq_;
  
  f_tmp << -distanceSq;

  LinearStateInequalitySoftConstraint constraint;
  constraint.penalty = penaltyPtr_.get();
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getValue(constraint, f_tmp);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation selfDistanceBoxSoftCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const ocs2::TargetTrajectories& targetTrajectories,
                                                                                         const ocs2::PreComputation& preComp) const 
{
  if (pinocchioEEKinPairPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComp);
    pinocchioEEKinPairPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  
  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(state.size());
  cost.dfdxx = matrix_t::Zero(state.size(), state.size());

  const std::vector<VectorFunctionLinearApproximation> eePosition = endEffectorKinematicsPairPtr_->getPositionLinearApproximation(state);

  // 提取两个末端执行器的位置和雅可比
  const Eigen::Vector3d& pos1 = eePosition[0].f;
  const Eigen::Vector3d& pos2 = eePosition[1].f;
  const matrix_t& J1 = eePosition[0].dfdx;  // 3 x state_dim
  const matrix_t& J2 = eePosition[1].dfdx;  // 3 x state_dim

  // 计算距离和导数
  Eigen::Vector3d diff_pos = pos1 - pos2;
  scalar_t distanceSq = diff_pos.squaredNorm();

  // 计算两点差值和解析导数
  matrix_t A_tmp = matrix_t::Identity(1, 1);
  vector_t h_tmp = vector_t::Zero(1);
  vector_t f_tmp = vector_t::Zero(1);
  matrix_t dfdx_tmp = matrix_t::Zero(1, state.size());

  // 计算距离关于状态的导数
  Eigen::RowVectorXd ddistanceSq_dx = 2.0 * diff_pos.transpose() * (J1 - J2);
  
  // 假设距离 <= distanceSq_ 的约束
  dfdx_tmp = -ddistanceSq_dx;
  h_tmp << -distanceSq + distanceSq_;  // 负的最小距离
  f_tmp << -distanceSq;    // 实际距离

  LinearStateInequalitySoftConstraint constraint;
  constraint.penalty = penaltyPtr_.get();
  constraint.A = A_tmp;
  constraint.h = h_tmp;

  cost = ocs2::mobile_manipulator::getQuadraticApproximation(constraint, f_tmp, dfdx_tmp);

  return cost;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
