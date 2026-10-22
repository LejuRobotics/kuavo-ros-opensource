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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "humanoid_interface/common/utils.h"
#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_interface/common/TopicLogger.h"

namespace ocs2 {
namespace humanoid {

/**
 * State-input tracking cost used for intermediate times
 */
class HumanoidStateInputQuadraticCost final : public QuadraticStateInputCost {
 public:
  HumanoidStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                     const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~HumanoidStateInputQuadraticCost() override = default;
  HumanoidStateInputQuadraticCost* clone() const override { return new HumanoidStateInputQuadraticCost(*this); }

 private:
  HumanoidStateInputQuadraticCost(const HumanoidStateInputQuadraticCost& rhs) = default;
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  /** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
  inline vector_t weightCompensatingInputWithTime(scalar_t time, const CentroidalModelInfoTpl<scalar_t>& info, const contact_flag_t& contactFlags) const {
    const std::shared_ptr<GaitSchedule>&  gaitSchedule = referenceManagerPtr_->getGaitSchedule();
    const ModeSchedule& modeSchedule = gaitSchedule->getModeSchedule();
    const auto current_mode = modeSchedule.modeAtTime(time);
    const auto last_mode_id = modeSchedule.modeBeforeId(time);
    const auto next_mode_id = modeSchedule.modeNextId(time);
    const auto current_contact_flags = modeNumber2StanceLeg(current_mode);
    auto hasFullySwingLeg = [](const contact_flag_t& contactFlags) {
      bool isLFSwingLeg = true;
      bool isRFSwingLeg = true;
      size_t num_contact_points = contactFlags.size()/2;
      for (size_t i = 0; i < num_contact_points; i++) {
        isLFSwingLeg = isLFSwingLeg && !contactFlags[i];
        isRFSwingLeg = isRFSwingLeg && !contactFlags[num_contact_points + i];
      }
      return isLFSwingLeg || isRFSwingLeg;
    };

    if (hasFullySwingLeg(current_contact_flags) || last_mode_id == -1 || next_mode_id == -1)
    {
      return weightCompensatingInput(info, contactFlags);
    }
    const auto numStanceLegs = numberOfClosedContacts(contactFlags);

    const auto before_mode = modeSchedule.modeSequence[last_mode_id];
    const auto next_mode = modeSchedule.modeSequence[next_mode_id];
    const auto before_mode_time = modeSchedule.eventTimes[last_mode_id];
    const auto next_mode_time = modeSchedule.eventTimes[next_mode_id];
    const auto current_mode_time = modeSchedule.eventTimes[last_mode_id+1];
    // std::cout << "current time: " << time  << " mode: " << current_mode << std::endl;
    // std::cout << "last mode: " << before_mode << " t: " << before_mode_time << std::endl;
    // std::cout << "next mode: " << next_mode << " t: " << next_mode_time << std::endl;
    const auto before_contact_flags = modeNumber2StanceLeg(before_mode);
    const auto next_contact_flags = modeNumber2StanceLeg(next_mode);

    // for (size_t i = 0; i < modeSchedule.modeSequence.size(); i++) 
    // {
      
    //   std::cout << "M: [" << i << "] " << modeSchedule.modeSequence[i] ;
    //   std::cout << " t: " << modeSchedule.eventTimes[i] << std::endl;
    //   auto standleg = modeNumber2StanceLeg(modeSchedule.modeSequence[i]);
    //   // for (size_t j = 0; j < standleg.size(); j++) 
    //   // {
    //   //   std::cout << "Stance Leg: [" << j << "] " << standleg[j] << std::endl;
    //   // }
    // }

    const vector_t last_input = weightCompensatingInput(info, before_contact_flags);
    const vector_t current_input = weightCompensatingInput(info, contactFlags);
    const vector_t next_input = (hasFullySwingLeg(next_contact_flags))?   weightCompensatingInput(info, next_contact_flags) : current_input;

    // std::cout << "last_input: " << last_input[2] << " "<<last_input[5*3-1] << std::endl;
    // std::cout << "current_input: " << current_input[2] << " "<<current_input[5*3-1] << std::endl;
    // std::cout << "next_input: " << next_input[2] << " "<<next_input[5*3-1] << std::endl;

    vector_t input;
    double total_time = current_mode_time - before_mode_time;
    double mid_time = (current_mode_time + before_mode_time)/2;

    if (time < mid_time)
        input = (current_input - last_input)*(time - before_mode_time)/(total_time/2) + last_input;
    else
        input = (next_input - current_input)*(time - mid_time)/(total_time/2) + current_input;
    // std::cout << "input: " << input[2] << " "<<input[5*3-1] << std::endl;
    // std::cout <<  "input sum: " << input.head(contactFlags.size()*3).sum() << std::endl;
    return input;
  }
  

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const auto& preCompHumanoid = cast<HumanoidPreComputation>(preComp);
    // const auto& armJointPos = preCompHumanoid.getArm1JointPosTarget();
    // const auto& armJointVel = preCompHumanoid.getArm1JointVelTarget();

    vector_t xNominal(info_.generalizedCoordinatesNum + 6);
    const scalar_t armMode = targetTrajectories.getDesiredState(time)(info_.generalizedCoordinatesNum + 6);
    // if(numerics::almost_eq(armMode, 1.0)) {
    xNominal << targetTrajectories.getDesiredState(time).segment(0, info_.generalizedCoordinatesNum + 6);
    // } else {
    //   xNominal << targetTrajectories.getDesiredState(time).segment(0, 12 + jointNums), armJointPos[0], vector_t::Zero(6), armJointPos[1], vector_t::Zero(6);
    // }
    // const vector_t xNominal = targetTrajectories.getDesiredState(time).segment(0, info_.generalizedCoordinatesNum + 6);
    // const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
    vector_t uNominal = weightCompensatingInputWithTime(time, info_, contactFlags);

    vector_t armWrench = preCompHumanoid.getArmWrench();
    if (armWrench.size() == 12)
    {
      uNominal.segment(info_.numThreeDofContacts * 3, 12) = armWrench;
      // 计算手臂外力在z方向的合力
      vector3_t leftArmForce = armWrench.segment<3>(0);
      vector3_t rightArmForce = armWrench.segment<3>(6);
      scalar_t totalArmForceZ = leftArmForce.z() + rightArmForce.z();

      // 计算当前所有接触点的z方向力之和
      scalar_t totalContactForceZ = 0.0;
      std::vector<scalar_t> contactPointForcesZ;
      contactPointForcesZ.reserve(contactFlags.size());
      
      for (size_t i = 0; i < contactFlags.size(); ++i) {
        if (contactFlags[i]) {
          scalar_t forceZ = uNominal[i * 3 + 2];
          totalContactForceZ += forceZ;
          contactPointForcesZ.push_back(forceZ);
        } else {
          contactPointForcesZ.push_back(0.0);
        }
      }
      
      if (totalContactForceZ > 1e-6) {  // 避免除零
        // 根据原有接触力比例分配补偿力
        size_t contactIndex = 0;
        for (size_t i = 0; i < contactFlags.size(); ++i) {
          if (contactFlags[i]) {
            scalar_t forceRatio = contactPointForcesZ[i] / totalContactForceZ;
            scalar_t compensationForce = -totalArmForceZ * forceRatio;
            uNominal[i * 3 + 2] += compensationForce;
          }
        }
      }
    }
    else
    {
      std::cout << "[HumanoidStateInputQuadraticCost] armWrench size is not 12" << std::endl;
      uNominal.segment(info_.numThreeDofContacts * 3, 12) = vector_t::Zero(12);
    }

    // if(!numerics::almost_eq(armMode, 1.0)){
    //   uNominal[3*8 + 6*2 + jointNums] = armJointVel[0];
    //   uNominal[3*8 + 6*2 + jointNums + 7] = armJointVel[1];
    // }

    return {state - xNominal, input - uNominal};
  }
  
  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;

  const int jointNums = 12;
};

/**
 * State tracking cost used for the final time
 */
class HumanoidStateQuadraticCost final : public QuadraticStateCost {
 public:
  HumanoidStateQuadraticCost(matrix_t Q, CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~HumanoidStateQuadraticCost() override = default;
  HumanoidStateQuadraticCost* clone() const override { return new HumanoidStateQuadraticCost(*this); }

 private:
  HumanoidStateQuadraticCost(const HumanoidStateQuadraticCost& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time).segment(0, info_.generalizedCoordinatesNum + 6);
    return state - xNominal;
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

}  // namespace humanoid
}  // namespace ocs2
