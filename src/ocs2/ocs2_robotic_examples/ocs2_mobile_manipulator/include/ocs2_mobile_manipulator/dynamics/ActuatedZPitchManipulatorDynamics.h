#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Implementation of a Z-Pitch-base mobile manipulator.
 *
 * The z-pitch manipulator is simulated z-pitch model for the base. The state
 * of the robot is full.
 *
 * The robot is assumed to be velocity controlled with the base commands as the
 * z velocity and the pitch velocity. The arm is assumed to be velocity controlled
 * with the arm joints.
 */
class ActuatedZPitchManipulatorDynamics final : public SystemDynamicsBaseAD {
 public:
  /**
   * Constructor
   *
   * @param [in] modelInfo : The manipulator information.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose : Display information.
   */
  ActuatedZPitchManipulatorDynamics(ManipulatorModelInfo modelInfo, const std::string& modelName,
                                      const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  ~ActuatedZPitchManipulatorDynamics() override = default;
  ActuatedZPitchManipulatorDynamics* clone() const override { return new ActuatedZPitchManipulatorDynamics(*this); }

 private:
  ActuatedZPitchManipulatorDynamics(const ActuatedZPitchManipulatorDynamics& rhs) = default;

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;

  const ManipulatorModelInfo info_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
