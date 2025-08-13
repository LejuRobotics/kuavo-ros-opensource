/******************************************************************************
Copyright (c) 2024, KUAVO. All rights reserved.
******************************************************************************/

#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

namespace ocs2 {
namespace mobile_manipulator {

class BaseArmRegularizationCost final : public QuadraticStateInputCost {
 public:
  BaseArmRegularizationCost(const mobile_manipulator::ManipulatorModelInfo& info, matrix_t Q)
      : QuadraticStateInputCost(std::move(Q), matrix_t::Zero(info.inputDim, info.inputDim)), info_(info) {}
  ~BaseArmRegularizationCost() override = default;

  BaseArmRegularizationCost(const BaseArmRegularizationCost& rhs) = default;
  BaseArmRegularizationCost* clone() const override { return new BaseArmRegularizationCost(*this); }

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories,
                                                       const PreComputation& preComp) const override {
    // For regularization, the target is zero for the state, so the deviation is the state itself.
    // The input deviation is zero.
    return {state, vector_t::Zero(info_.inputDim)};
  }

 private:
  const mobile_manipulator::ManipulatorModelInfo& info_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2 