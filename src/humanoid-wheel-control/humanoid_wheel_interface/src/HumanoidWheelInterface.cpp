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

#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "humanoid_wheel_interface/HumanoidWheelInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "humanoid_wheel_interface/MobileManipulatorPreComputation.h"
#include "humanoid_wheel_interface/constraint/TorsoTrackingConstraint.h"
#include "humanoid_wheel_interface/constraint/EndEffectorConstraint.h"
#include "humanoid_wheel_interface/constraint/EndEffectorLocalConstraint.h"
#include "humanoid_wheel_interface/constraint/EndEffectorMultiPointConstraint.h"
#include "humanoid_wheel_interface/constraint/EndEffectorLocalMultiPointConstraint.h"
#include "humanoid_wheel_interface/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "humanoid_wheel_interface/cost/BaseStateInputCost.h"
#include "humanoid_wheel_interface/cost/EndEffectorBoxSoftCost.h"
#include "humanoid_wheel_interface/cost/EndEffectorLocalBoxSoftCost.h"
#include "humanoid_wheel_interface/cost/EndEffectorMultiPointBoxSoftCost.h"
#include "humanoid_wheel_interface/cost/EndEffectorLocalMultiPointBoxSoftCost.h"
#include "humanoid_wheel_interface/cost/TorsoTrackingBoxSoftCost.h"
#include "humanoid_wheel_interface/cost/EndEffectorJointBias.h"
#include "humanoid_wheel_interface/cost/selfDistanceBoxSoftCost.h"
#include "humanoid_wheel_interface/dynamics/WheelBasedMobileManipulatorDynamics.h"
#include "humanoid_wheel_interface/dynamics/WheelWorldBasedMobileManipulatorDynamics.h"
#include "humanoid_wheel_interface/dynamics/DefaultManipulatorDynamics.h"
#include "humanoid_wheel_interface/dynamics/FloatingArmManipulatorDynamics.h"
#include "humanoid_wheel_interface/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidWheelInterface::HumanoidWheelInterface(const std::string& taskFile, const std::string& libraryFolder,
                                                       const std::string& urdfFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[HumanoidWheelInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[HumanoidWheelInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[HumanoidWheelInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[HumanoidWheelInterface] URDF file not found: " + urdfFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[HumanoidWheelInterface] Generated library path: " << libraryFolderPath << std::endl;

  // read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  // resolve meta-information about the model
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
  // read the frame names
  std::string baseFrame, torsoFrame;
  std::vector<std::string> eeFrames;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, torsoFrame, "model_information.torsoFrame", false);
  loadData::loadStdVector<std::string>(taskFile, "model_information.eeFrames", eeFrames, false);

  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
  std::cerr << "\n #### model_information.torsoFrame: \"" << torsoFrame << "\"";
  std::cerr << "\n #### model_information.eeFrames: " << std::endl;
  for (const auto& name : eeFrames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << " #### =============================================================================" << std::endl;

  // create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile, modelType, removeJointNames)));
  std::cerr << *pinocchioInterfacePtr_;

  // ManipulatorModelInfo
  manipulatorModelInfo_ = mobile_manipulator::createManipulatorModelInfo(*pinocchioInterfacePtr_, modelType, baseFrame, torsoFrame, eeFrames);

  bool usePreComputation = true;
  bool recompileLibraries = true;
  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
  loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  std::cerr << " #### =============================================================================\n";

  // Default initial state
  initialState_.setZero(manipulatorModelInfo_.stateDim);
  // const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  // const int armStateDim = manipulatorModelInfo_.armDim;

  // arm base DOFs initial state
  // if (baseStateDim > 0) {
  //   vector_t initialBaseState = vector_t::Zero(baseStateDim);
  //   loadData::loadEigenMatrix(taskFile, "initialState.base." + modelTypeEnumToString(modelType), initialBaseState);
  //   initialState_.head(baseStateDim) = initialBaseState;
  // }

  // arm joints DOFs velocity limits
  // vector_t initialArmState = vector_t::Zero(armStateDim);
  // loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
  // initialState_.tail(armStateDim) = initialArmState;

  // std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

  // Reference Manager
  referenceManagerPtr_ =
      std::make_shared<MobileManipulatorReferenceManager>(manipulatorModelInfo_, *pinocchioInterfacePtr_, taskFile);

  /*
   * Optimal control problem
   */
  // Cost
  problem_.costPtr->add("baseStateInputCost", getBaseStateInputCost(taskFile));

  // Constraints
  // torso tracking constraint
  problem_.stateSoftConstraintPtr->add("torsoTracking", getTorsoTrackingSoftConstraint(*pinocchioInterfacePtr_, manipulatorModelInfo_, taskFile));
  
  // torso tracking box soft cost
  bool useTorsoBoxCost = false;
  loadData::loadPtreeValue(pt, useTorsoBoxCost, "torsoBoxSoftCost.activate", true);

  if(useTorsoBoxCost)
  {
    problem_.stateCostPtr->add("torsoTrackingBoxCost", getTorsoTrackingBoxSoftCost(*pinocchioInterfacePtr_, taskFile));
  }
  
  // joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));

  bool useEeMultiPoint = false;
  loadData::loadPtreeValue(pt, useEeMultiPoint, "multiPointEeConstraint.activate", true);
  if(useEeMultiPoint)
  {
    bool useEeBoxCost = false;
    loadData::loadPtreeValue(pt, useEeBoxCost, "multiPointEeConstraint.endEffectorBox.activate", true);
    for(int eef_idx = 0; eef_idx < manipulatorModelInfo_.eeFrames.size(); eef_idx++)
    {
      problem_.stateSoftConstraintPtr->add("endEffector_" + std::to_string(eef_idx), getEndEffectorMultiPointConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      problem_.finalSoftConstraintPtr->add("finalEndEffector_" + std::to_string(eef_idx), getEndEffectorMultiPointConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      if(useEeBoxCost)
      {
        problem_.stateCostPtr->add("endEffectorBox_" + std::to_string(eef_idx), getEndEffectorMultiPointBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
        problem_.finalCostPtr->add("finalEndEffectorBox_" + std::to_string(eef_idx), getEndEffectorMultiPointBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));  
      }
    }
    for(int eef_idx = 0; eef_idx < manipulatorModelInfo_.eeFrames.size(); eef_idx++)
    {
      problem_.stateSoftConstraintPtr->add("endEffectorLocal_" + std::to_string(eef_idx), getEndEffectorLocalMultiPointConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      problem_.finalSoftConstraintPtr->add("finalEndEffectorLocal_" + std::to_string(eef_idx), getEndEffectorLocalMultiPointConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      if(useEeBoxCost)
      {
        problem_.stateCostPtr->add("endEffectorLocalBox_" + std::to_string(eef_idx), getEndEffectorLocalMultiPointBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
        problem_.finalCostPtr->add("finalEndEffectorLocalBox_" + std::to_string(eef_idx), getEndEffectorLocalMultiPointBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));  
      }
    }
  }
  else
  {
    // end-effector state constraint
    // end effector box soft cost
    bool useEeBoxCost = false;
    loadData::loadPtreeValue(pt, useEeBoxCost, "endEffectorBox.activate", true);
    for(int eef_idx = 0; eef_idx < manipulatorModelInfo_.eeFrames.size(); eef_idx++)
    {
      problem_.stateSoftConstraintPtr->add("endEffector_" + std::to_string(eef_idx), getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      problem_.finalSoftConstraintPtr->add("finalEndEffector_" + std::to_string(eef_idx), getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                          usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      if(useEeBoxCost)
      {
        problem_.stateCostPtr->add("endEffectorBox_" + std::to_string(eef_idx), getEndEffectorBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
        problem_.finalCostPtr->add("finalEndEffectorBox_" + std::to_string(eef_idx), getEndEffectorBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      }
    }
    // end-effector local state constraint
    for(int eef_idx = 0; eef_idx < manipulatorModelInfo_.eeFrames.size(); eef_idx++)
    {
      problem_.stateSoftConstraintPtr->add("endEffectorLocal_" + std::to_string(eef_idx), getEndEffectorLocalConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      problem_.finalSoftConstraintPtr->add("finalEndEffectorLocal_" + std::to_string(eef_idx), getEndEffectorLocalConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                          usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      if(useEeBoxCost)
      {
        problem_.stateCostPtr->add("endEffectorLocalBox_" + std::to_string(eef_idx), getEndEffectorLocalBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
        problem_.finalCostPtr->add("finalEndEffectorLocalBox_" + std::to_string(eef_idx), getEndEffectorLocalBoxSoftCost(*pinocchioInterfacePtr_, taskFile, 
                                                                                   usePreComputation, libraryFolder, recompileLibraries, eef_idx));
      }
    }
  }
  // end-effector joint bias cost
  bool useEeJointBias = false;
  loadData::loadPtreeValue(pt, useEeJointBias, "eeJointBias.activate", true);
  if(useEeJointBias)
  {
    problem_.stateCostPtr->add("endEffectorJointBias", getEndEffectorJointBias(*pinocchioInterfacePtr_, taskFile));
  }

  // self-collision avoidance constraint
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  if (activateSelfCollision) {
    problem_.stateSoftConstraintPtr->add(
        "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
  }

  // self-distance constraint
  bool activateSelfDistance = true;
  loadData::loadPtreeValue(pt, activateSelfDistance, "selfDistanceConstraint.activate", true);
  if (activateSelfDistance) {
    std::vector<std::pair<std::string, std::string>> distanceLinkPair;
    loadData::loadStdVectorOfPair(taskFile, "selfDistanceConstraint.linkPairs", distanceLinkPair, true);
    for(int i = 0; i < distanceLinkPair.size(); i++)
    {
      problem_.stateSoftConstraintPtr->add(
          "selfDistance_" + std::to_string(i), getSelfDistanceConstraint(i, *pinocchioInterfacePtr_, distanceLinkPair[i], 
                                                                         taskFile, true));
    }
  }

  // Dynamics
  switch (manipulatorModelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      problem_.dynamicsPtr.reset(
          new DefaultManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      problem_.dynamicsPtr.reset(
          new FloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      problem_.dynamicsPtr.reset(
          new FullyActuatedFloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      problem_.dynamicsPtr.reset(
          new WheelBasedMobileManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::WheelWorldBasedMobileManipulator: {
      problem_.dynamicsPtr.reset(
          new WheelWorldBasedMobileManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }

  /*
   * Pre-computation
   */
  if (usePreComputation) {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
  }

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> HumanoidWheelInterface::getBaseStateInputCost(const std::string& taskFile) {
  matrix_t Q = matrix_t::Zero(manipulatorModelInfo_.stateDim, manipulatorModelInfo_.stateDim);
  matrix_t R = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;

  std::cout << "baseStateDim: " << baseStateDim << std::endl;
  std::cout << "baseInputDim: " << baseInputDim << std::endl;
  std::cout << "armStateDim: " << armStateDim << std::endl;

  // arm base DOFs input costs
  if (baseStateDim > 0) {
    matrix_t Q_base = matrix_t::Zero(baseStateDim, baseStateDim);
    loadData::loadEigenMatrix(taskFile, "baseCost.Q.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), Q_base);
    Q.topLeftCorner(baseStateDim, baseStateDim) = Q_base;
  }

  // arm base DOFs input costs
  if (baseInputDim > 0) {
    matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
    loadData::loadEigenMatrix(taskFile, "baseCost.R.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  // arm joints DOFs state costs
  matrix_t Q_arm = matrix_t::Zero(armStateDim, armStateDim);
  loadData::loadEigenMatrix(taskFile, "baseCost.Q.arm", Q_arm);
  Q.bottomRightCorner(armStateDim, armStateDim) = Q_arm;

  // arm joints DOFs input costs
  matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
  loadData::loadEigenMatrix(taskFile, "baseCost.R.arm", R_arm);
  R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

  std::cerr << "\n #### State Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "stateCost.Q:  \n" << Q << '\n';
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::make_unique<BaseStateInputCost>(std::move(Q), std::move(R), manipulatorModelInfo_, *referenceManagerPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries, int eefIdx) {
  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorConstraint] eefIdx is out of range.");
  }
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  // const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrames[eefIdx]});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrames[eefIdx]},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, eefIdx));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorBoxSoftCost(const PinocchioInterface& pinocchioInterface,
                                                                             const std::string& taskFile, bool usePreComputation, 
                                                                             const std::string& libraryFolder, bool recompileLibraries, 
                                                                             int eefIdx) {
  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorBoxSoftCost] eefIdx is out of range.");
  }
  scalar_t mu_Focus = 0.5;
  scalar_t delta_Focus = 0.01;
  scalar_t mu_unFocus = 0.5;
  scalar_t delta_unFocus = 0.01;

  vector6_t pose_lower = vector6_t::Ones() * -99;   // default a big num
  vector6_t pose_upper = vector6_t::Ones() * 99;

  const std::string prefix = "endEffectorBox.";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### endEffectorBox Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  /***********************************************************************************************************/
  std::string barrierName = "focus_barrier.";
  loadData::loadPtreeValue(pt, mu_Focus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_Focus, prefix + barrierName + "delta", true);
  barrierName = "unFocus_barrier.";
  loadData::loadPtreeValue(pt, mu_unFocus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_unFocus, prefix + barrierName + "delta", true);
  /***********************************************************************************************************/
  loadData::loadEigenMatrix(taskFile, prefix + "pose_lower", pose_lower);
  std::cout << "pose_lower: " << pose_lower.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "pose_upper", pose_upper);
  std::cout << "pose_upper: " << pose_upper.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorBoxSoftCost] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateCost> constraint;

  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrames[eefIdx]});
    constraint.reset(new EndEffectorBoxSoftCost(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, 
                                                pose_lower, pose_upper, 
                                                ocs2::RelaxedBarrierPenalty::Config(mu_Focus, delta_Focus), 
                                                ocs2::RelaxedBarrierPenalty::Config(mu_unFocus, delta_unFocus), eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrames[eefIdx]},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorBoxSoftCost(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, 
                                                pose_lower, pose_upper, 
                                                ocs2::RelaxedBarrierPenalty::Config(mu_Focus, delta_Focus), 
                                                ocs2::RelaxedBarrierPenalty::Config(mu_unFocus, delta_unFocus), eefIdx));
  }

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorMultiPointBoxSoftCost(const PinocchioInterface& pinocchioInterface,
                                                                             const std::string& taskFile, bool usePreComputation, 
                                                                             const std::string& libraryFolder, bool recompileLibraries, 
                                                                             int eefIdx) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "multiPointEeConstraint.";
  std::string point_prefix;
  switch(eefIdx)
  {
    case 0: point_prefix = "leftArm";  break;
    case 1: point_prefix = "rightArm"; break;
    default: std::cout << "[getEndEffectorMultiPointBoxSoftCost] eefIdx do not support !!! " << std::endl; break;
  }

  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointBoxSoftCost] eefIdx is out of range.");
  }

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorMultiPointBoxSoftCost] referenceManagerPtr_ should be set first!");
  }

  std::vector<std::string> pointNames;
  std::vector<scalar_t> eeMu_focus;
  std::vector<scalar_t> eeDelta_focus;
  std::vector<scalar_t> eeMu_unFocus;
  std::vector<scalar_t> eeDelta_unFocus;

  std::cerr << "\n #### endEffector Multi Point Box Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadStdVector<std::string>(taskFile, prefix + point_prefix, pointNames, true);
  /***************************************************************************************************************/
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.focus_penalty.mu", eeMu_focus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.focus_penalty.delta", eeDelta_focus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.unFocus_penalty.mu", eeMu_unFocus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.unFocus_penalty.delta", eeDelta_unFocus, true);
  /***************************************************************************************************************/

  if(eeMu_focus.size() != pointNames.size() || eeMu_unFocus.size() != pointNames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointBoxSoftCost] eeMu.size() is not equal to pointNames.size().");
  }
  if(eeDelta_focus.size() != pointNames.size() || eeDelta_unFocus.size() != pointNames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointBoxSoftCost] eeDelta.size() is not equal to pointNames.size().");
  }
  vector_t pos_lower = vector_t::Ones(pointNames.size() * 3) * -99;   // default a big num
  vector_t pos_upper = vector_t::Ones(pointNames.size() * 3) * 99;
  loadData::loadEigenMatrix(taskFile, prefix + "endEffectorBox.lower", pos_lower);
  std::cout << "pos_lower: " << pos_lower.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "endEffectorBox.upper", pos_upper);
  std::cout << "pos_upper: " << pos_upper.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  std::vector<RelaxedBarrierPenalty::Config> settings_focus;
  std::vector<RelaxedBarrierPenalty::Config> settings_unFocus;
  for(int i=0; i < pointNames.size(); i++)
  {
    settings_focus.push_back(RelaxedBarrierPenalty::Config(eeMu_focus[i], eeDelta_focus[i]));
    settings_unFocus.push_back(RelaxedBarrierPenalty::Config(eeMu_unFocus[i], eeDelta_unFocus[i]));
  }

  std::unique_ptr<StateCost> constraint;


  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, pointNames);
    constraint.reset(new EndEffectorMultiPointBoxSoftCost(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                          pinocchioInterface, manipulatorModelInfo_, 
                                                          pos_lower, pos_upper, settings_focus, settings_unFocus, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, pointNames,
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics_box", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorMultiPointBoxSoftCost(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                          pinocchioInterface, manipulatorModelInfo_, 
                                                          pos_lower, pos_upper, settings_focus, settings_unFocus, eefIdx));
  }

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorLocalConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                 const std::string& taskFile, const std::string& prefix,
                                                                                 bool usePreComputation, const std::string& libraryFolder,
                                                                                 bool recompileLibraries, int eefIdx) {
  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorLocalConstraint] eefIdx is out of range.");
  }
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  // const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorLocalConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrames[eefIdx]});
    constraint.reset(new EndEffectorLocalConstraint(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrames[eefIdx]},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorLocalConstraint(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, eefIdx));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorLocalBoxSoftCost(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, bool usePreComputation, 
                                                                                  const std::string& libraryFolder, bool recompileLibraries, 
                                                                                  int eefIdx) 
{
  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorLocalBoxSoftCost] eefIdx is out of range.");
  }
  scalar_t mu_Focus = 0.5;
  scalar_t delta_Focus = 0.01;
  scalar_t mu_unFocus = 0.5;
  scalar_t delta_unFocus = 0.01;

  vector6_t pose_lower = vector6_t::Ones() * -99;   // default a big num
  vector6_t pose_upper = vector6_t::Ones() * 99;

  const std::string prefix = "endEffectorBox.";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### endEffectorLocalBox Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  /***********************************************************************************************************/
  std::string barrierName = "focus_barrier.";
  loadData::loadPtreeValue(pt, mu_Focus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_Focus, prefix + barrierName + "delta", true);
  barrierName = "unFocus_barrier.";
  loadData::loadPtreeValue(pt, mu_unFocus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_unFocus, prefix + barrierName + "delta", true);
  /***********************************************************************************************************/
  loadData::loadEigenMatrix(taskFile, prefix + "pose_lower", pose_lower);
  std::cout << "pose_lower: " << pose_lower.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "pose_upper", pose_upper);
  std::cout << "pose_upper: " << pose_upper.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorLocalBoxSoftCost] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateCost> constraint;

  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrames[eefIdx]});
    constraint.reset(new EndEffectorLocalBoxSoftCost(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, 
                                                     pose_lower, pose_upper, 
                                                     ocs2::RelaxedBarrierPenalty::Config(mu_Focus, delta_Focus), 
                                                     ocs2::RelaxedBarrierPenalty::Config(mu_unFocus, delta_unFocus), eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrames[eefIdx]},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorLocalBoxSoftCost(eeKinematics, *referenceManagerPtr_, manipulatorModelInfo_, 
                                                     pose_lower, pose_upper, 
                                                     ocs2::RelaxedBarrierPenalty::Config(mu_Focus, delta_Focus), 
                                                     ocs2::RelaxedBarrierPenalty::Config(mu_unFocus, delta_unFocus), eefIdx));
  }

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorLocalMultiPointBoxSoftCost(const PinocchioInterface& pinocchioInterface,
                                                                             const std::string& taskFile, bool usePreComputation, 
                                                                             const std::string& libraryFolder, bool recompileLibraries, 
                                                                             int eefIdx) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "multiPointEeConstraint.";
  std::string point_prefix;
  switch(eefIdx)
  {
    case 0: point_prefix = "leftArm";  break;
    case 1: point_prefix = "rightArm"; break;
    default: std::cout << "[getEndEffectorLocalMultiPointBoxSoftCost] eefIdx do not support !!! " << std::endl; break;
  }

  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorLocalMultiPointBoxSoftCost] eefIdx is out of range.");
  }

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorLocalMultiPointBoxSoftCost] referenceManagerPtr_ should be set first!");
  }

  std::vector<std::string> pointNames;
  std::vector<scalar_t> eeMu_focus;
  std::vector<scalar_t> eeDelta_focus;
  std::vector<scalar_t> eeMu_unFocus;
  std::vector<scalar_t> eeDelta_unFocus;

  std::cerr << "\n #### endEffector Local Multi Point Box Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadStdVector<std::string>(taskFile, prefix + point_prefix, pointNames, true);
  /***************************************************************************************************************/
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.focus_penalty.mu", eeMu_focus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.focus_penalty.delta", eeDelta_focus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.unFocus_penalty.mu", eeMu_unFocus, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffectorBox.unFocus_penalty.delta", eeDelta_unFocus, true);
  /***************************************************************************************************************/

  if(eeMu_focus.size() != pointNames.size() || eeMu_unFocus.size() != pointNames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointBoxSoftCost] eeMu.size() is not equal to pointNames.size().");
  }
  if(eeDelta_focus.size() != pointNames.size() || eeDelta_unFocus.size() != pointNames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointBoxSoftCost] eeDelta.size() is not equal to pointNames.size().");
  }
  vector_t pos_lower = vector_t::Ones(pointNames.size() * 3) * -99;   // default a big num
  vector_t pos_upper = vector_t::Ones(pointNames.size() * 3) * 99;
  loadData::loadEigenMatrix(taskFile, prefix + "endEffectorBox.lower", pos_lower);
  std::cout << "pos_lower: " << pos_lower.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "endEffectorBox.upper", pos_upper);
  std::cout << "pos_upper: " << pos_upper.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  std::vector<RelaxedBarrierPenalty::Config> settings_focus;
  std::vector<RelaxedBarrierPenalty::Config> settings_unFocus;
  for(int i=0; i < pointNames.size(); i++)
  {
    settings_focus.push_back(RelaxedBarrierPenalty::Config(eeMu_focus[i], eeDelta_focus[i]));
    settings_unFocus.push_back(RelaxedBarrierPenalty::Config(eeMu_unFocus[i], eeDelta_unFocus[i]));
  }

  std::unique_ptr<StateCost> constraint;


  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, pointNames);
    constraint.reset(new EndEffectorLocalMultiPointBoxSoftCost(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                            pinocchioInterface, manipulatorModelInfo_, 
                                            pos_lower, pos_upper, settings_focus, settings_unFocus, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, pointNames,
                                            manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                            "end_effector_kinematics_local_box", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorLocalMultiPointBoxSoftCost(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                          pinocchioInterface, manipulatorModelInfo_, 
                                                          pos_lower, pos_upper, settings_focus, settings_unFocus, eefIdx));
  }

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, const std::string& urdfFile,
                                                                                  const std::string& prefix, bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                                                                            std::move(geometryInterface), minimumDistance);
  } else {
    constraint = std::make_unique<SelfCollisionConstraintCppAd>(
        pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_), std::move(geometryInterface), minimumDistance,
        "self_collision", libraryFolder, recompileLibraries, false);
  }

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> HumanoidWheelInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armInputDim = manipulatorModelInfo_.armDim;
  const auto& model = pinocchioInterface.getModel();

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) {
    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
    const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

    std::cerr << "\n #### JointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = baseStateDim + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }

  // load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    vector_t upperBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    // Base DOFs velocity limits
    if (baseInputDim > 0) {
      vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
      vector_t upperBoundBase = vector_t::Zero(baseInputDim);
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.lowerBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                lowerBoundBase);
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.upperBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                upperBoundBase);
      lowerBound.head(baseInputDim) = lowerBoundBase;
      upperBound.head(baseInputDim) = upperBoundBase;
    }

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(manipulatorModelInfo_.inputDim);
    for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
  return boxConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getTorsoTrackingSoftConstraint(const PinocchioInterface& pinocchioInterface, 
                                                                                  const ManipulatorModelInfo& info, 
                                                                                  const std::string& taskFile) 
{
  // 默认权重值：位置(x,y,z) + 姿态(roll,pitch,yaw)
  vector_t torso_tracking_mu = vector_t::Zero(6);

  const std::string prefix = "torsoTracking.";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### TorsoTracking Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadEigenMatrix(taskFile, prefix + "muWeights", torso_tracking_mu);
  std::cout << "muWeights: " << torso_tracking_mu.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;

  MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
  PinocchioEndEffectorKinematics eeKinematicTorso(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.torsoFrame});

  constraint.reset(new TorsoTrackingConstraint(eeKinematicTorso, *referenceManagerPtr_, info));

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  for (int i = 0; i < 6; ++i) {
    penaltyArray[i] = std::make_unique<QuadraticPenalty>(torso_tracking_mu[i]);
  }

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> HumanoidWheelInterface::getTorsoTrackingBoxSoftCost(const PinocchioInterface& pinocchioInterface,
                                                                               const std::string& taskFile) 
{
  scalar_t mu_Focus = 0.5;      // scaling
  scalar_t delta_Focus = 0.01;

  scalar_t mu_unFocus = 0.5;      // scaling
  scalar_t delta_unFocus = 0.01;

  vector6_t pose_lower_ = vector6_t::Ones() * -99;   // default a big num
  vector6_t pose_upper_ = vector6_t::Ones() * 99;

  const std::string prefix = "torsoBoxSoftCost.";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### torsoBoxSoftCost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  /***********************************************************************************************************/
  std::string barrierName = "focus_barrier.";
  loadData::loadPtreeValue(pt, mu_Focus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_Focus, prefix + barrierName + "delta", true);
  barrierName = "unFocus_barrier.";
  loadData::loadPtreeValue(pt, mu_unFocus, prefix + barrierName + "mu", true);
  loadData::loadPtreeValue(pt, delta_unFocus, prefix + barrierName + "delta", true);
  /***********************************************************************************************************/
  loadData::loadEigenMatrix(taskFile, prefix + "pose_lower", pose_lower_);
  std::cout << "pose_lower: " << pose_lower_.transpose() << std::endl;
  loadData::loadEigenMatrix(taskFile, prefix + "pose_upper", pose_upper_);
  std::cout << "pose_upper: " << pose_upper_.transpose() << std::endl;
  std::cerr << "\n #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateCost> constraint;

  MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
  PinocchioEndEffectorKinematics eeKinematicTorso(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.torsoFrame});

  constraint.reset(new TorsoTrackingBoxSoftCost(eeKinematicTorso, *referenceManagerPtr_, 
                                               pose_lower_, pose_upper_, 
                                               ocs2::RelaxedBarrierPenalty::Config(mu_Focus, delta_Focus), 
                                               ocs2::RelaxedBarrierPenalty::Config(mu_unFocus, delta_unFocus)));

  return constraint;
}

std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorJointBias(const PinocchioInterface& pinocchioInterface, 
                                                                           const std::string& taskFile)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "eeJointBias.";

  std::vector<double> biasCenter;
  std::vector<double> biasMu;
  std::vector<double> biasDelta;
  std::vector<std::string> biasJointNames;
  std::vector<std::string> biasDir;

  std::cerr << "\n #### EndEffector Joint Bias Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadStdVector<std::string>(taskFile, prefix + "biasJointNames", biasJointNames, true);
  loadData::loadStdVector<std::string>(taskFile, prefix + "biasDir", biasDir, true);
  loadData::loadStdVector<double>(taskFile, prefix + "biasCenter", biasCenter, true);
  loadData::loadStdVector<double>(taskFile, prefix + "biasMu", biasMu, true);
  loadData::loadStdVector<double>(taskFile, prefix + "biasDelta", biasDelta, true);

  // 检查所有 vector 大小是否一致，如果不一致则抛出异常
  if (biasJointNames.size() != biasDir.size() || 
       biasDir.size() != biasCenter.size() || 
       biasCenter.size() != biasMu.size() || 
       biasMu.size() != biasDelta.size())
  {
      std::stringstream errorMsg;
      errorMsg << "\n#### ERROR: Mismatched vector sizes in eeJointBias settings!\n"
               << "#### biasJointNames size: " << biasJointNames.size() << "\n"
               << "#### biasDir size: " << biasDir.size() << "\n"
               << "#### biasCenter size: " << biasCenter.size() << "\n"
               << "#### biasMu size: " << biasMu.size() << "\n"
               << "#### biasDelta size: " << biasDelta.size() << "\n"
               << "#### All vectors must have the same size!";
        
      // 输出到标准错误
      std::cerr << errorMsg.str() << std::endl;
        
      // 抛出异常
      throw std::runtime_error(errorMsg.str());
  }

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorJointBias] referenceManagerPtr_ should be set first!");
  }

  std::vector<EndEffectorJointBias::BoxConstraint> biasLimits;

  // 获取手臂数量，假设最后几个维度是手臂
  int armDof = 0;
  loadData::loadPtreeValue(pt, armDof, "model_settings.mpcArmsDof", true);
  int armStartIndex = manipulatorModelInfo_.stateDim - armDof;
  std::cout << "armStartIndex: " << armStartIndex << std::endl;

  // 查询 "zarm_l1_joint" 在 mujoco 的索引,
  int armIndexInPin = 0;
  const auto& model = pinocchioInterface.getModel();
  auto& jointName = model.names;
  for(armIndexInPin = 0; armIndexInPin < jointName.size(); armIndexInPin++)
  {
    if(jointName[armIndexInPin] == "zarm_l1_joint")
    {
      break;
    }
  }
  std::cout << "armIndexInPin: " << armIndexInPin << std::endl;

  biasLimits.reserve(biasJointNames.size());
  for(int i = 0; i < biasJointNames.size(); i++)
  {
    EndEffectorJointBias::BoxConstraint biasConstraint;
    // 对比手臂名称在 state 中的索引
    for(int idx = armIndexInPin; idx < jointName.size(); idx++)
    {
      if(jointName[idx] == biasJointNames[i])
      {
        biasConstraint.index = armStartIndex + idx - armIndexInPin;
        std::cout << "idx: " << i << " biasConstraint.index: " << biasConstraint.index << std::endl;
        break;
      }
    }
    // 通过 dir 判断偏好方向
    if (biasDir[i] == "True") { biasConstraint.lowerBound = biasCenter[i]; }
    else if (biasDir[i] == "False") { biasConstraint.upperBound = biasCenter[i]; }

    biasConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({biasMu[i], biasDelta[i]}));
    biasLimits.push_back(std::move(biasConstraint));
  }
  std::unique_ptr<StateCost> constraint;

  constraint.reset(new EndEffectorJointBias(biasLimits, *referenceManagerPtr_));
  
  return constraint;

}

std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorMultiPointConstraint(const PinocchioInterface& pinocchioInterface, 
                                                                                      const std::string& taskFile, bool usePreComp, 
                                                                                      const std::string& libraryFolder,
                                                                                      bool recompileLibraries, int eefIdx)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "multiPointEeConstraint.";

  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorMultiPointConstraint] eefIdx is out of range.");
  }

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorMultiPointConstraint] referenceManagerPtr_ should be set first!");
  }

  std::vector<std::string> pointNames;
  std::vector<double> eeMu;
  std::string point_prefix;
  switch(eefIdx)
  {
    case 0: point_prefix = "leftArm";  break;
    case 1: point_prefix = "rightArm"; break;
    default: std::cout << "[getEndEffectorMultiPointConstraint] eefIdx do not support !!! " << std::endl; break;
  }
  std::cerr << "\n #### EndEffector Multi Point Constraint Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadStdVector<std::string>(taskFile, prefix + point_prefix, pointNames, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffector.mu", eeMu, true);

  std::unique_ptr<StateConstraint> constraint;

  if (usePreComp) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, pointNames);
    constraint.reset(new EndEffectorMultiPointConstraint(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                         pinocchioInterface, manipulatorModelInfo_, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, pointNames, 
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorMultiPointConstraint(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                         pinocchioInterface, manipulatorModelInfo_, eefIdx));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(pointNames.size() * 3);

  if(eeMu.size() != pointNames.size() * 3)
  {
    throw std::invalid_argument("[getEndEffectorMultiPointConstraint] eeMu.size() is not equal to pointNames.size() * 3.");
  }

  for (int i = 0; i < pointNames.size(); ++i) 
  {
    for (int j = 0; j < 3; j++)
    {
      penaltyArray[i*3 + j] = std::make_unique<QuadraticPenalty>(eeMu[i*3 + j]);
    }
  }

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

std::unique_ptr<StateCost> HumanoidWheelInterface::getEndEffectorLocalMultiPointConstraint(const PinocchioInterface& pinocchioInterface, 
                                                                                           const std::string& taskFile, bool usePreComp, 
                                                                                           const std::string& libraryFolder,
                                                                                           bool recompileLibraries, int eefIdx)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "multiPointEeConstraint.";

  if(eefIdx >= manipulatorModelInfo_.eeFrames.size())
  {
    throw std::invalid_argument("[getEndEffectorLocalMultiPointConstraint] eefIdx is out of range.");
  }

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorLocalMultiPointConstraint] referenceManagerPtr_ should be set first!");
  }

  std::vector<std::string> pointNames;
  std::vector<double> eeMu;
  std::string point_prefix;
  switch(eefIdx)
  {
    case 0: point_prefix = "leftArm";  break;
    case 1: point_prefix = "rightArm"; break;
    default: std::cout << "[getEndEffectorLocalMultiPointConstraint] eefIdx do not support !!! " << std::endl; break;
  }
  std::cerr << "\n #### EndEffector Local Multi Point Constraint Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadStdVector<std::string>(taskFile, prefix + point_prefix, pointNames, true);
  loadData::loadStdVector<double>(taskFile, prefix + "endEffector.mu", eeMu, true);

  std::unique_ptr<StateConstraint> constraint;

  if (usePreComp) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, pointNames);
    constraint.reset(new EndEffectorLocalMultiPointConstraint(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                              pinocchioInterface, manipulatorModelInfo_, eefIdx));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, pointNames, 
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics_local", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorLocalMultiPointConstraint(pointNames.size(), eeKinematics, *referenceManagerPtr_, 
                                                              pinocchioInterface, manipulatorModelInfo_, eefIdx));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(pointNames.size() * 3);

  if(eeMu.size() != pointNames.size() * 3)
  {
    throw std::invalid_argument("[getEndEffectorMultiPointConstraint] eeMu.size() is not equal to pointNames.size() * 3.");
  }

  for (int i = 0; i < pointNames.size(); ++i) 
  {
    for (int j = 0; j < 3; j++)
    {
      penaltyArray[i*3 + j] = std::make_unique<QuadraticPenalty>(eeMu[i*3 + j]);
    }
  }

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));

}

std::unique_ptr<StateCost> HumanoidWheelInterface::getSelfDistanceConstraint(int index, const PinocchioInterface& pinocchioInterface, 
                                                                             std::pair<std::string, std::string> linkPair, 
                                                                             const std::string& taskFile, bool verbose)
{
  std::string prefix = "selfDistanceConstraint.";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getSelfDistanceConstraint] referenceManagerPtr_ should be set first!");
  }

  std::vector<std::string> linkPairNames;   // 获取连杆名称
  linkPairNames.push_back(linkPair.first);
  linkPairNames.push_back(linkPair.second);

  std::vector<double> mu;   //获取屏障函数配置
  std::vector<double> delta;
  loadData::loadStdVector<double>(taskFile, prefix + "mu", mu, true);
  loadData::loadStdVector<double>(taskFile, prefix + "delta", delta, true);
  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu[index], delta[index]});

  std::vector<double> maxDistance;  // 获取最大距离
  loadData::loadStdVector<double>(taskFile, prefix + "maxDistance", maxDistance, true);

  std::unique_ptr<StateCost> constraint;

  MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
  PinocchioEndEffectorKinematics eeKinematicsPair(pinocchioInterface, pinocchioMapping, linkPairNames);
  constraint.reset(new selfDistanceBoxSoftCost(eeKinematicsPair, *referenceManagerPtr_, 
                                               ocs2::RelaxedBarrierPenalty::Config(mu[index], delta[index]), 
                                               maxDistance[index]));

  return constraint;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
