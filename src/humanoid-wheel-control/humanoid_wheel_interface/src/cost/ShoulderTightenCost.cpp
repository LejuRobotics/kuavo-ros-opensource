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

  scalar_t f = scalar_t(0.0);
  for (size_t i = 0; i < shoulderStateIndices_.size(); ++i) {
    const scalar_t alpha = referenceManager_.getShoulderTightAlpha(armIndices_[i]);
    const scalar_t weight = shoulderWeights_(i) * alpha;
    if (weight <= 0.0) {
      continue;
    }
    const auto idx = shoulderStateIndices_[i];
    const scalar_t deviation = state(idx) - desiredState(idx);
    f += weight * deviation * deviation;
  }
  return f;
}

ScalarFunctionQuadraticApproximation shoulderTightenCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                     const TargetTrajectories&,
                                                                                     const PreComputation&) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.size());
  const auto desiredState = referenceManager_.getStateInputTargetTrajectories().getDesiredState(time);

  for (size_t i = 0; i < shoulderStateIndices_.size(); ++i) {
    const scalar_t alpha = referenceManager_.getShoulderTightAlpha(armIndices_[i]);
    const scalar_t weight = shoulderWeights_(i) * alpha;
    if (weight <= 0.0) {
      continue;
    }
    const auto idx = shoulderStateIndices_[i];
    const scalar_t deviation = state(idx) - desiredState(idx);

    cost.f += weight * deviation * deviation;
    cost.dfdx(idx) += 2.0 * weight * deviation;
    cost.dfdxx(idx, idx) += 2.0 * weight;
  }
  return cost;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
