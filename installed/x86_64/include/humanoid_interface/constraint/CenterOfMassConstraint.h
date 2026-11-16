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

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2
{
namespace humanoid
{

/**
 * This class implements a constraint to ensure the center of mass stays within 
 * the bounds defined by the foot positions. The constraint enforces that the 
 * center of mass X and Y positions remain between the left and right foot centers.
 * 
 * IMPORTANT: This implementation follows WbcBase.cpp's COM calculation pattern,
 * which considers both position (q) and velocity (v) when computing center of mass.
 * This ensures consistency with the whole-body controller's COM calculations.
 * 
 * Design Notes:
 * - Uses pinocchio::centerOfMass(model, data, q, v) instead of just q
 * - Maintains consistency with WBC's physics calculations
 * - Supports dynamic motion scenarios where velocity affects COM position
 */
class CenterOfMassConstraint final : public StateConstraint
{
public:
  /**
   * Constructor
   * @param [in] pinocchioInterface: The pinocchio interface for kinematics computation
   * @param [in] info: The centroidal model information
   * @param [in] referenceManager: Reference manager for foot positions
   * @param [in] contactFrameNames: Names of all contact frames corresponding to indices
   */
  CenterOfMassConstraint(const PinocchioInterface& pinocchioInterface,
                        const CentroidalModelInfo& info,
                        const SwitchedModelReferenceManager& referenceManager,
                        const std::vector<std::string>& contactFrameNames);

  ~CenterOfMassConstraint() override = default;
  CenterOfMassConstraint* clone() const override
  {
    return new CenterOfMassConstraint(*this);
  }

  size_t getNumConstraints(scalar_t time) const override
  {
    return 4;  // 4 boundaries: minX, maxX, minY, maxY
  }

  bool isActive(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

private:
  CenterOfMassConstraint(const CenterOfMassConstraint& rhs);
  
  void publishSupportPolygonVisualization(const std::vector<vector3_t>& supportPoints, const vector3_t& comPosition) const;
  void publishSupportPolygonBounds(scalar_t minX, scalar_t maxX, scalar_t minY, scalar_t maxY, const vector3_t& comPosition) const;
  void publishConstraintValue(const vector_t& constraint) const;

  const PinocchioInterface* pinocchioInterfacePtr_;
  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::vector<std::string> contactFrameNames_;
  
  // ROS visualization
  mutable ros::Publisher supportPolygonPub_;
  mutable ros::Publisher comPositionPub_;
  mutable ros::Publisher boundaryPub_;
  mutable ros::Publisher constraintValuePub_;
  mutable bool rosInitialized_;
};

}  // namespace humanoid
}  // namespace ocs2
