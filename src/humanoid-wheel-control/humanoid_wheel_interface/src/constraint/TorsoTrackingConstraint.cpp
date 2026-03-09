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
                                                 const MobileManipulatorReferenceManager& referenceManager,
                                                 const ManipulatorModelInfo& info)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsTorsoPtr_(endEffectorKinematicsTorso.clone()),
      referenceManager_(referenceManager),
      info_(info) 
{
  pinocchioEEKinTorsoPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsTorsoPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t TorsoTrackingConstraint::getNumConstraints(scalar_t time) const {
  return 6;
}

bool TorsoTrackingConstraint::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableTorsoPoseTargetTrajectories();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TorsoTrackingConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
   // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  // const auto& currentPositionOrientation = preCompMM.getTorsoPose();
  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);

  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsTorsoPtr_->getPosition(state).front() - desiredPositionOrientationWorld.first;
  constraint.tail<3>() = endEffectorKinematicsTorsoPtr_->getOrientationError(state, {desiredPositionOrientationWorld.second}).front();
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation TorsoTrackingConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinTorsoPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinTorsoPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);

  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePositionTorso = endEffectorKinematicsTorsoPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePositionTorso.f - desiredPositionOrientationWorld.first;  // 本体系的torso期望
  approximation.dfdx.topRows<3>() = eePositionTorso.dfdx;

  const auto eeOrientationTorsoError =
      endEffectorKinematicsTorsoPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientationWorld.second}).front();
  approximation.f.tail<3>() = eeOrientationTorsoError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationTorsoError.dfdx;

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TorsoTrackingConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManager_.getTorsoTargetTrajectories();
  const auto& targetTorsoState = targetTrajectories.getDesiredState(time);

  vector_t position = targetTorsoState.head(3);
  Eigen::Vector3d zyx = targetTorsoState.segment(3, 3);
  quaternion_t orientation =  // 轮臂躯干采用 pitch-yaw-roll 欧拉角, 可以节省一个角度来表达姿态
         Eigen::AngleAxis<scalar_t>(zyx(1), Eigen::Matrix<scalar_t, 3, 1>::UnitY()) *
         Eigen::AngleAxis<scalar_t>(zyx(0), Eigen::Matrix<scalar_t, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<scalar_t>(zyx(2), Eigen::Matrix<scalar_t, 3, 1>::UnitX());

  return {position, orientation};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TorsoTrackingConstraint::targetBaseToWorld(const vector_t& state, 
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
