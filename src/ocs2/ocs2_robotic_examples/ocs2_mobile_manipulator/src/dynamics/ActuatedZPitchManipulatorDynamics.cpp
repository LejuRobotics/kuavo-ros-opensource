#include "ocs2_mobile_manipulator/dynamics/ActuatedZPitchManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ActuatedZPitchManipulatorDynamics::ActuatedZPitchManipulatorDynamics(ManipulatorModelInfo info, const std::string& modelName,
                                                                         const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                                         bool recompileLibraries /*= true*/, bool verbose /*= true*/)
    : info_(std::move(info)) {
  this->initialize(info_.stateDim, info_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ActuatedZPitchManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                               const ad_vector_t&) const {
  ad_vector_t dxdt(info_.stateDim);
  const auto v_z = input(0);  // z velocity in base frame
  const auto d_pitch = input(1);  // pitch velocity in base frame
  dxdt << ad_scalar_t(0), ad_scalar_t(0), v_z, ad_scalar_t(0), d_pitch, ad_scalar_t(0), input.tail(info_.armDim);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
