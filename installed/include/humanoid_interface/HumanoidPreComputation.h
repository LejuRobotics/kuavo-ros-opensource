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
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "humanoid_interface/common/ModelSettings.h"
#include "humanoid_interface/constraint/EndEffectorLinearConstraint.h"
#include "humanoid_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace humanoid {

/** Callback for caching and reference update */
class HumanoidPreComputation : public PreComputation {
 public:
  HumanoidPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
  ~HumanoidPreComputation() override = default;

  HumanoidPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeXYReferenceConstraintConfigs() const { return eeXYRefConConfigs_; }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeZeroVelocityConstraintConfigs() const { return eeZeroVelConConfigs_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }
  vector_t getArmWrench() const { return armWrench_; }

  // const vector_t getArm1JointPosTarget() const { return armJointPos_; }
  // const vector_t getArm1JointVelTarget() const { return armJointVel_; }

 private:
  HumanoidPreComputation(const HumanoidPreComputation& other);

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
  std::vector<EndEffectorLinearConstraint::Config> armSwingPosConConfigs_;
  std::vector<EndEffectorLinearConstraint::Config> eeZeroVelConConfigs_;

  std::unique_ptr<PinocchioEndEffectorKinematics> feetKinematicsPtr_;

  vector_t armJointPos_;
  vector_t armJointVel_;
  std::vector<EndEffectorLinearConstraint::Config> eeXYRefConConfigs_;
  vector_t armWrench_ = vector_t::Zero(12);
};

}  // namespace humanoid
}  // namespace ocs2
