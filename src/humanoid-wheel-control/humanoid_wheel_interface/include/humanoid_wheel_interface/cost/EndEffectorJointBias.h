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

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/PenaltyBase.h>

#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"

namespace ocs2 {
namespace mobile_manipulator {
/**
 *   Implements the cost penalty for state box constraints
 */
class EndEffectorJointBias final : public StateCost {
 public:
  struct BoxConstraint {
    //! Index of the constraint in the state
    size_t index = 0;

    //! Lower bound of the box constraint (default is low, but not numeric::lowest to prevent underflow)
    scalar_t lowerBound = -1e3;

    //! Upper bound of the box constraint (default is high, but not numeric::max to prevent overflow)
    scalar_t upperBound = 1e3;

    //! Penalty function
    std::unique_ptr<PenaltyBase> penaltyPtr;

    /* Constructors and assignment operators */
    BoxConstraint() = default;
    ~BoxConstraint() = default;
    BoxConstraint(const BoxConstraint& other);
    BoxConstraint& operator=(const BoxConstraint& other);
    BoxConstraint(BoxConstraint&& other) noexcept = default;
    BoxConstraint& operator=(BoxConstraint&& other) noexcept = default;
  };

  /**
   * Constructor.
   * @param stateBoxConstraints : box constraint specification for states
   */
  EndEffectorJointBias(std::vector<BoxConstraint> stateBoxConstraints, 
                       const MobileManipulatorReferenceManager& referenceManager);

  ~EndEffectorJointBias() override = default;

  EndEffectorJointBias* clone() const override;

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& /* targetTrajectories */,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& /* targetTrajectories */,
                                                                 const PreComputation& preComp) const override;

 private:
  EndEffectorJointBias(const EndEffectorJointBias& other) = default;

  void sortByIndex(std::vector<BoxConstraint>& boxConstraints) const;

  scalar_t getValue(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints) const;

  void fillQuadraticApproximation(scalar_t t, const vector_t& h, const std::vector<BoxConstraint>& boxConstraints, scalar_t& value,
                                  vector_t& firstDerivative, matrix_t& secondDerivative) const;

  std::vector<BoxConstraint> stateBoxConstraints_;
  const MobileManipulatorReferenceManager& referenceManager_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
