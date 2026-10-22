#pragma once
#include <cstdint>
namespace velocity_ik {
// Observation only: these bits must never be fed back into control admission.
enum HoldReason : uint32_t {
  LowConfidence=1, SourceUnavailable=2, TrackingFault=4, DynamicFault=8,
  PrepareComplete=16, OwnerLease=32, ModeBlocked=64, RobotStale=128,
  ContractInactive=256
};
inline uint32_t holdReasons(bool confidence, bool fresh, bool tracking_fault,
    bool dynamic_fault, bool prepare_complete, bool owner_waiting,
    bool mode_ok, bool robot_fresh, bool active_or_prepare) {
  return (!confidence?LowConfidence:0u) | (!fresh?SourceUnavailable:0u) |
      (tracking_fault?TrackingFault:0u) | (dynamic_fault?DynamicFault:0u) |
      (prepare_complete?PrepareComplete:0u) | (owner_waiting?OwnerLease:0u) |
      (!mode_ok?ModeBlocked:0u) | (!robot_fresh?RobotStale:0u) |
      (!active_or_prepare?ContractInactive:0u);
}
inline uint32_t recoveryReasons(bool arm_ok,bool lower_ok,bool reference_stopped,
    bool measurement_stopped,bool source_ok,bool session_owner_ok) {
  return (!arm_ok?1u:0u)|(!lower_ok?2u:0u)|(!reference_stopped?4u:0u)|
      (!measurement_stopped?8u:0u)|(!source_ok?16u:0u)|(!session_owner_ok?32u:0u);
}
}
