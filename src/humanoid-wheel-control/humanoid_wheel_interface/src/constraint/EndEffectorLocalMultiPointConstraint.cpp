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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <humanoid_wheel_interface/MobileManipulatorPreComputation.h>
#include <humanoid_wheel_interface/constraint/EndEffectorLocalMultiPointConstraint.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLocalMultiPointConstraint::EndEffectorLocalMultiPointConstraint(int pointNums, 
                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const MobileManipulatorReferenceManager& referenceManager, 
                                             PinocchioInterface pinocchioInterface, 
                                             const ManipulatorModelInfo& info,
                                             const int eefInx)
    : StateConstraint(ConstraintOrder::Linear),
      pointNums_(pointNums), 
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManager_(referenceManager), 
      pinocchioInterface_(pinocchioInterface), 
      info_(info),
      eef_Idx_(eefInx),
      eef_num_(info.eeFrames.size()) {
  if (endEffectorKinematics.getIds().size() != pointNums) {
    throw std::runtime_error("[EndEffectorLocalMultiPointConstraint] endEffectorKinematics has wrong number of end effector IDs.");
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorLocalMultiPointConstraint::getNumConstraints(scalar_t time) const {
  return 3 * pointNums_;
}

bool EndEffectorLocalMultiPointConstraint::isActive(scalar_t time) const 
{
  return referenceManager_.getEnableEeTargetLocalTrajectoriesForArm(eef_Idx_);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLocalMultiPointConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);
  const vector_t multiTargetPos = targetPoseToMultiPointPos(desiredPositionOrientationWorld, localOffset_);

  std::vector<vector3_t> eePosState = endEffectorKinematicsPtr_->getPosition(state);

  vector_t constraint = vector_t::Zero(3*pointNums_);

  for(int i = 0; i < eePosState.size(); i++)
  {
    constraint.segment<3>(i * 3) = eePosState[i] - multiTargetPos.segment<3>(i * 3);
  }

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorLocalMultiPointConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto& desiredPositionOrientation = interpolateEndEffectorPose(time);
  auto desiredPositionOrientationWorld = targetBaseToWorld(state, desiredPositionOrientation);
  const vector_t multiTargetPos = targetPoseToMultiPointPos(desiredPositionOrientationWorld, localOffset_);

  auto approximation = VectorFunctionLinearApproximation(3*pointNums_, state.rows(), 0);

  const std::vector<VectorFunctionLinearApproximation> eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);

  for(int i = 0; i < eePosition.size(); i++)
  {
    approximation.f.segment<3>(i * 3) = eePosition[i].f - multiTargetPos.segment<3>(i * 3);
    approximation.dfdx.middleRows(i * 3, 3) = eePosition[i].dfdx;
  }

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorLocalMultiPointConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<Eigen::Vector3d, quaternion_t> {
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
auto EndEffectorLocalMultiPointConstraint::targetBaseToWorld(const vector_t& state, 
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLocalMultiPointConstraint::targetPoseToMultiPointPos(std::pair<Eigen::Vector3d, quaternion_t> targetPose, 
                                                                    std::vector<Eigen::Vector3d> localOffset) const 
{
  Eigen::Vector3d& posMain = targetPose.first;
  Eigen::Matrix3d rotMain = targetPose.second.toRotationMatrix();

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
