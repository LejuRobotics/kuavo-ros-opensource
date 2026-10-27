#pragma once

#include "wheel_arm_abs_vel/VelocityIkMapperStatus.h"
#include "wheel_arm_abs_vel/VelocityIkShadow.h"

namespace wheel_arm_abs_vel {

// Derive the mapper feedback from the exact diagnostic snapshot. No new
// authority, lifecycle, or freshness decisions are made by this projection.
inline VelocityIkMapperStatus makeMapperStatus(const VelocityIkShadow& source) {
  VelocityIkMapperStatus status;
  status.header = source.header;
  status.session_id = source.session_id;
  status.generation = source.generation;
  status.control_state = source.control_state;
  status.valid = source.valid;
  status.takeover_active = source.takeover_active;
  status.preparation_complete = source.preparation_complete;
  return status;
}

}  // namespace wheel_arm_abs_vel
