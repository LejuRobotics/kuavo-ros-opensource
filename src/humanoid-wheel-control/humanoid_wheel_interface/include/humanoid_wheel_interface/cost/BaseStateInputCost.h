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

#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

class BaseStateInputCost final : public QuadraticStateInputCost {
 public:
  BaseStateInputCost(matrix_t Q, matrix_t R, 
                     const ManipulatorModelInfo& info, 
                     const MobileManipulatorReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)), 
        info_(info), referenceManager_(referenceManager) 
  {
    baseNums_ = info_.stateDim - info_.armDim;
  }

  ~BaseStateInputCost() override = default;

  BaseStateInputCost(const BaseStateInputCost& rhs) = default;
  BaseStateInputCost* clone() const override { return new BaseStateInputCost(*this); }

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override {
    vector_t stateDeviation = vector_t::Zero(state.size());
    vector_t inputDeviation = vector_t::Zero(input.size());

    if(referenceManager_.getEnableBaseTrack())   // 使能底盘跟踪
    {
      stateDeviation.head(baseNums_) = state.head(baseNums_) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredState(time).head(baseNums_);
      inputDeviation.head(baseNums_) = input.head(baseNums_) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredInput(time).head(baseNums_);
    }
    if(referenceManager_.getEnableLegJointTrack()) // 使能下肢关节跟踪
    {
      stateDeviation.segment(baseNums_, 4) = state.segment(baseNums_, 4) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredState(time).segment(baseNums_, 4);
      inputDeviation.segment(baseNums_, 4) = input.segment(baseNums_, 4) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredInput(time).segment(baseNums_, 4);
    }
    if(referenceManager_.getEnableArmJointTrackForArm(0)) // 使能左臂关节跟踪
    {
      stateDeviation.tail(info_.armDim - 4).head(7) = state.tail(info_.armDim - 4).head(7) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredState(time).tail(info_.armDim - 4).head(7);
      inputDeviation.tail(info_.armDim - 4).head(7) = input.tail(info_.armDim - 4).head(7) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredInput(time).tail(info_.armDim - 4).head(7);
    }
    if(referenceManager_.getEnableArmJointTrackForArm(1)) // 使能右臂关节跟踪
    {
      stateDeviation.tail(info_.armDim - 4).tail(7) = state.tail(info_.armDim - 4).tail(7) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredState(time).tail(info_.armDim - 4).tail(7);
      inputDeviation.tail(info_.armDim - 4).tail(7) = input.tail(info_.armDim - 4).tail(7) - 
            referenceManager_.getStateInputTargetTrajectories().getDesiredInput(time).tail(info_.armDim - 4).tail(7);
    }

    return {stateDeviation, inputDeviation};
  }

 private:
  size_t baseNums_;
  const MobileManipulatorReferenceManager& referenceManager_;
  const ManipulatorModelInfo& info_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
