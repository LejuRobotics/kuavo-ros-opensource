/******************************************************************************
 * Implementation of PinocchioEndEffectorKinematicsNoBase.
 * See header for rationale.
 *****************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_wheel_interface/PinocchioEndEffectorKinematicsNoBase.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioEndEffectorKinematicsNoBase::zeroBase(const vector_t& state) const {
  vector_t s = state;
  if (s.size() >= 3) {
    s.head(3).setZero();
  }
  return s;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<PinocchioEndEffectorKinematicsNoBase::vector3_t> PinocchioEndEffectorKinematicsNoBase::getPosition(
    const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematicsNoBase] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // 本地数据副本，用 stateNoBase 重算 FK，使 frame 位姿不依赖 base x/y/yaw
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());
  const vector_t q = mappingPtr_->getPinocchioJointPosition(zeroBase(state));
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  std::vector<vector3_t> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    positions.emplace_back(data.oMf[frameId].translation());
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<PinocchioEndEffectorKinematicsNoBase::vector3_t> PinocchioEndEffectorKinematicsNoBase::getOrientationError(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematicsNoBase] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());
  const vector_t q = mappingPtr_->getPinocchioJointPosition(zeroBase(state));
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  std::vector<vector3_t> errors;
  for (int i = 0; i < static_cast<int>(endEffectorFrameIds_.size()); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    errors.emplace_back(quaternionDistance(matrixToQuaternion(data.oMf[frameId].rotation()), referenceOrientations[i]));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsNoBase::getPositionLinearApproximation(
    const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematicsNoBase] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  const vector_t stateNoBase = zeroBase(state);
  const vector_t q = mappingPtr_->getPinocchioJointPosition(stateNoBase);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  std::vector<VectorFunctionLinearApproximation> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    VectorFunctionLinearApproximation pos;
    pos.f = data.oMf[frameId].translation();
    std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(stateNoBase, J.topRows<3>(), matrix_t::Zero(3, model.nv));
    // 浮动机座 x, y, yaw 对应前 3 列，置 0 使约束线性化不依赖 base
    if (pos.dfdx.cols() >= 3) {
      pos.dfdx.leftCols(3).setZero();
    }
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsNoBase::getOrientationErrorLinearApproximation(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematicsNoBase] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  const vector_t stateNoBase = zeroBase(state);
  const vector_t q = mappingPtr_->getPinocchioJointPosition(stateNoBase);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < static_cast<int>(endEffectorFrameIds_.size()); i++) {
    VectorFunctionLinearApproximation err;
    const size_t frameId = endEffectorFrameIds_[i];
    const quaternion_t quat = matrixToQuaternion(data.oMf[frameId].rotation());
    err.f = quaternionDistance(quat, referenceOrientations[i]);
    matrix_t J = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    const matrix_t Jqdist =
        (quaternionDistanceJacobian(quat, referenceOrientations[i]) * angularVelocityToQuaternionTimeDerivative(quat)) *
        J.bottomRows<3>();
    std::tie(err.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(stateNoBase, Jqdist, matrix_t::Zero(3, model.nv));
    if (err.dfdx.cols() >= 3) {
      err.dfdx.leftCols(3).setZero();
    }
    errors.emplace_back(std::move(err));
  }
  return errors;
}

}  // namespace ocs2
