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
#include "humanoid_wheel_interface/constraint/TorsoTrackingConstraint.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TorsoTrackingConstraint::TorsoTrackingConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematicsTorso,
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematicsBase,
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 const ManipulatorModelInfo& info)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsTorsoPtr_(endEffectorKinematicsTorso.clone()),
      endEffectorKinematicsBasePtr_(endEffectorKinematicsBase.clone()),
      referenceManager_(referenceManager),
      info_(info) 
{
  pinocchioEEKinTorsoPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsTorsoPtr_.get());
  pinocchioEEKinBasePtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsBasePtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t TorsoTrackingConstraint::getNumConstraints(scalar_t time) const {
  return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TorsoTrackingConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
   // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr || pinocchioEEKinBasePtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    pinocchioEEKinBasePtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  // const auto& currentPositionOrientation = preCompMM.getTorsoPose();
  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);

  vector_t constraint(6);
  constraint.head<3>() = (endEffectorKinematicsTorsoPtr_->getPosition(state).front() - desiredPositionOrientation.first)
                         - endEffectorKinematicsBasePtr_->getPosition(state).front();
  constraint.tail<3>() = endEffectorKinematicsTorsoPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front()
                        - endEffectorKinematicsTorsoPtr_->getOrientationError(state, {quaternion_t(1, 0, 0, 0)}).front();
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation TorsoTrackingConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr || pinocchioEEKinBasePtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    pinocchioEEKinBasePtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);

  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePositionTorso = endEffectorKinematicsTorsoPtr_->getPositionLinearApproximation(state).front();
  const auto eePositionBase = endEffectorKinematicsBasePtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = (eePositionTorso.f - desiredPositionOrientation.first) - eePositionBase.f;  // 本体系的torso期望
  approximation.dfdx.topRows<3>() = eePositionTorso.dfdx - eePositionBase.dfdx;

  const auto eeOrientationTorsoError =
      endEffectorKinematicsTorsoPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  const auto eeOrientationBaseError =
      endEffectorKinematicsBasePtr_->getOrientationErrorLinearApproximation(state, {quaternion_t(1, 0, 0, 0)}).front();
  approximation.f.tail<3>() = eeOrientationTorsoError.f - eeOrientationBaseError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationTorsoError.dfdx - eeOrientationBaseError.dfdx;

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TorsoTrackingConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManager_.getTorsoTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index].head(7);
    const auto& rhs = stateTrajectory[index + 1].head(7);
    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.head(3) + (1.0 - alpha) * rhs.head(3);
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // stateTrajectory.size() == 1
    position = stateTrajectory.front().head(7).head(3);
    orientation = quaternion_t(stateTrajectory.front().head(7).tail<4>());
  }

  return {position, orientation};
}

}  // namespace mobile_manipulator
}  // namespace ocs2
