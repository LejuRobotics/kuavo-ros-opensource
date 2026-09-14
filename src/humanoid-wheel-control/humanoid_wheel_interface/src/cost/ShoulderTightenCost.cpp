#include "humanoid_wheel_interface/cost/ShoulderTightenCost.h"

namespace ocs2 {

namespace mobile_manipulator {

shoulderTightenCost::shoulderTightenCost(std::vector<size_t> shoulderStateIndices,
                                         std::vector<int> armIndices,
                                         const vector_t& shoulderWeights,
                                         const MobileManipulatorReferenceManager& referenceManager)
    : shoulderStateIndices_(std::move(shoulderStateIndices)),
      armIndices_(std::move(armIndices)),
      shoulderWeights_(shoulderWeights),
      referenceManager_(referenceManager) {}

bool shoulderTightenCost::isActive(scalar_t time) const {
  return referenceManager_.getEnableShoulderTight();
}

scalar_t shoulderTightenCost::getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                                       const PreComputation&) const {
  const auto desiredState = referenceManager_.getStateInputTargetTrajectories().getDesiredState(time);
  // 收紧期间以"冻结的当前肩部姿态"为参考, 而非陈旧的参考轨迹(初始/上次 reset 姿态), 消除接合瞬间的"回拽"
  const auto& anchorState = referenceManager_.getShoulderRefAnchorState();
  const bool anchorReady = (anchorState.size() == desiredState.size());

  scalar_t f = scalar_t(0.0);
  for (size_t i = 0; i < shoulderStateIndices_.size(); ++i) {
    const int armIdx = armIndices_[i];
    // 按臂门控: 仅对开启收紧的手臂施加拉力 (避免单臂收紧时另一臂被陈旧锚定/权重误拉)
    if (!referenceManager_.getEnableShoulderTightForArm(armIdx)) {
      continue;
    }
    const scalar_t alpha = referenceManager_.getShoulderTightAlpha(armIdx);
    const scalar_t weight = shoulderWeights_(i) * alpha;
    if (weight <= 0.0) {
      continue;
    }
    const auto idx = shoulderStateIndices_[i];
    const scalar_t qRef = (anchorReady && referenceManager_.getShoulderRefAnchorInit(armIdx)) ? anchorState(idx) : desiredState(idx);
    const scalar_t deviation = state(idx) - qRef;
    f += weight * deviation * deviation;
  }
  return f;
}

ScalarFunctionQuadraticApproximation shoulderTightenCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                     const TargetTrajectories&,
                                                                                     const PreComputation&) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.size());
  const auto desiredState = referenceManager_.getStateInputTargetTrajectories().getDesiredState(time);
  // 方案D: 锚定参考 (与 getValue 保持一致)
  const auto& anchorState = referenceManager_.getShoulderRefAnchorState();
  const bool anchorReady = (anchorState.size() == desiredState.size());

  for (size_t i = 0; i < shoulderStateIndices_.size(); ++i) {
    const int armIdx = armIndices_[i];
    // 按臂门控: 与 getValue 保持一致
    if (!referenceManager_.getEnableShoulderTightForArm(armIdx)) {
      continue;
    }
    const scalar_t alpha = referenceManager_.getShoulderTightAlpha(armIdx);
    const scalar_t weight = shoulderWeights_(i) * alpha;
    if (weight <= 0.0) {
      continue;
    }
    const auto idx = shoulderStateIndices_[i];
    const scalar_t qRef = (anchorReady && referenceManager_.getShoulderRefAnchorInit(armIdx)) ? anchorState(idx) : desiredState(idx);
    const scalar_t deviation = state(idx) - qRef;

    cost.f += weight * deviation * deviation;
    cost.dfdx(idx) += 2.0 * weight * deviation;
    cost.dfdxx(idx, idx) += 2.0 * weight;
  }
  return cost;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
