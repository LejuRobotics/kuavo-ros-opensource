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

#pragma once

#include <memory>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"

#include "humanoid_wheel_interface/cost/LinearStateInequalitySoftconstraint.h"

namespace ocs2 {

namespace mobile_manipulator {

class EndEffectorLocalMultiPointBoxSoftCost final : public StateCost {
 public:

  using quaternion_t = Eigen::Quaternion<scalar_t>;
  
  EndEffectorLocalMultiPointBoxSoftCost(int pointNums, 
                          const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                          const MobileManipulatorReferenceManager& referenceManager,
                          PinocchioInterface pinocchioInterface, 
                          const ManipulatorModelInfo& info,
                          const vector_t& pos_lower, 
                          const vector_t& pos_upper,
                          std::vector<RelaxedBarrierPenalty::Config> settingsFocus,
                          std::vector<RelaxedBarrierPenalty::Config> settingsUnFocus,
                          const int eefInx);

  ~EndEffectorLocalMultiPointBoxSoftCost() override = default;
  EndEffectorLocalMultiPointBoxSoftCost* clone() const override { return new EndEffectorLocalMultiPointBoxSoftCost(
                                                                                       pointNums_, 
                                                                                       *endEffectorKinematicsPtr_, 
                                                                                       referenceManager_,
                                                                                       pinocchioInterface_, 
                                                                                       info_,
                                                                                       pos_lower_, pos_upper_, 
                                                                                       settingsFocus_, 
                                                                                       settingsUnFocus_, 
                                                                                       eef_Idx_); }
  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                            const PreComputation& preComp) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  EndEffectorLocalMultiPointBoxSoftCost(const EndEffectorLocalMultiPointBoxSoftCost& other) = default;                                                   
  std::pair<Eigen::Vector3d, quaternion_t> interpolateEndEffectorPose(scalar_t time) const;
  std::pair<Eigen::Vector3d, quaternion_t> targetBaseToWorld(const vector_t& state, 
                              const std::pair<Eigen::Vector3d, quaternion_t>& targetBase) const;
  vector_t targetPoseToMultiPointPos(const std::pair<Eigen::Vector3d, quaternion_t>& targetPose, 
                                     const std::vector<Eigen::Vector3d>& localOffset) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const MobileManipulatorReferenceManager& referenceManager_;
  PinocchioInterface pinocchioInterface_; 

  vector_t pos_lower_;
  vector_t pos_upper_;

  std::vector<RelaxedBarrierPenalty::Config> settingsFocus_;
  std::vector<RelaxedBarrierPenalty::Config> settingsUnFocus_;
  std::vector<std::unique_ptr<ocs2::RelaxedBarrierPenalty>> penaltyVecFocusPtr_;
  std::vector<std::unique_ptr<ocs2::RelaxedBarrierPenalty>> penaltyVecUnFocusPtr_;

  const ManipulatorModelInfo& info_;
  int eef_Idx_;
  const int eef_num_ = 0;  
  int start_index_ = 0;

  int pointNums_ = 0;
  std::vector<Eigen::Vector3d> localOffset_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
