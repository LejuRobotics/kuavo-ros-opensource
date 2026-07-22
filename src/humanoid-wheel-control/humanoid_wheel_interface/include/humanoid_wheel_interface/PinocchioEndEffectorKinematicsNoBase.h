/******************************************************************************
 * LejuRobot extension: end-effector kinematics with floating base (x, y, yaw) neutralized.
 *
 * Rationale: PinocchioEndEffectorKinematics reads the frame pose from the pinocchio::Data
 * that MobileManipulatorPreComputation pre-computed with the FULL state (including base x, y, yaw).
 * Therefore simply passing a zeroed state to the getters does NOT change the frame pose value,
 * only the Jacobian. This derived class overrides the getters so that:
 *   - the base x, y, yaw are zeroed before use,
 *   - forward kinematics (and joint Jacobians) are recomputed on a local data copy with the
 *     zeroed state, so the frame pose value no longer depends on x, y, yaw,
 *   - the first 3 columns (x, y, yaw) of the resulting Jacobian are zeroed, so the linearization
 *     has no dependence on the floating base either.
 *
 * This makes the torso tracking constraint / box soft cost invariant to the floating base pose.
 *****************************************************************************/

#pragma once

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2 {

/**
 * End-effector kinematics variant that makes the floating base (first 3 state dims: x, y, yaw)
 * have no effect on the computed position / orientation error and their linear approximations.
 */
class PinocchioEndEffectorKinematicsNoBase : public PinocchioEndEffectorKinematics {
 public:
  PinocchioEndEffectorKinematicsNoBase(const PinocchioInterface& pinocchioInterface,
                                       const PinocchioStateInputMapping<scalar_t>& mapping,
                                       std::vector<std::string> endEffectorIds)
      : PinocchioEndEffectorKinematics(pinocchioInterface, mapping, std::move(endEffectorIds)) {}

  ~PinocchioEndEffectorKinematicsNoBase() override = default;

  PinocchioEndEffectorKinematicsNoBase* clone() const override {
    return new PinocchioEndEffectorKinematicsNoBase(*this);
  }

  std::vector<vector3_t> getPosition(const vector_t& state) const override;
  std::vector<vector3_t> getOrientationError(const vector_t& state,
                                             const std::vector<quaternion_t>& referenceOrientations) const override;

  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override;
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

 private:
  PinocchioEndEffectorKinematicsNoBase(const PinocchioEndEffectorKinematicsNoBase& rhs) = default;

  /** @return a copy of state with the first 3 dims (base x, y, yaw) set to zero. */
  vector_t zeroBase(const vector_t& state) const;
};

}  // namespace ocs2
