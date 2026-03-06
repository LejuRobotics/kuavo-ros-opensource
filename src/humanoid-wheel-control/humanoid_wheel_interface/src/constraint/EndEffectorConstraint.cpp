/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <humanoid_wheel_interface/MobileManipulatorPreComputation.h>
#include <humanoid_wheel_interface/constraint/EndEffectorConstraint.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const MobileManipulatorReferenceManager& referenceManager, 
                                             const ManipulatorModelInfo& info,
                                             const int eefInx)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager), 
      info_(info),
      eef_Idx_(eefInx),
      eef_num_(info.eeFrames.size()) {
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
  start_index_ = eef_Idx_ * 7;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
  return 6;
}

bool EndEffectorConstraint::isActive(scalar_t time) const 
{
  return referenceManagerPtr_->getEnableEeTargetTrajectoriesForArm(eef_Idx_);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
  constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;

  const auto eeOrientationError =
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  approximation.f.tail<3>() = eeOrientationError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManagerPtr_->getEeTargetTrajectories(eef_Idx_);
  const auto& targetEeState = targetTrajectories.getDesiredState(time);

  vector_t position = targetEeState.segment(0, 3);
  Eigen::Vector3d zyx = targetEeState.segment(3, 3);
  quaternion_t orientation = getQuaternionFromEulerAnglesZyx(zyx);

  return {position, orientation};
}

}  // namespace mobile_manipulator
}  // namespace ocs2
