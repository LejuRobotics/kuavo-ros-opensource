// Pinocchio must be included before Boost headers.
#include <pinocchio/fwd.hpp>
#include <cmath>
#include "humanoid_controllers/rl/RLControllerManager.h"
#include "humanoid_controllers/rl/AmpAssembleController.h"

namespace humanoid_controller
{
  // AMP family integration with the outer manager. Locked methods share the
  // manager's recursive mutex; no separate state or lock domain is introduced.
  std::string RLControllerManager::resolveControllerNameLocked(const std::string& name) const
  {
    // Preserve flat configurations on other robot versions.
    if (controllers_.find(name) != controllers_.end())
      return name;
    for (const auto& entry : controllers_)
    {
      const auto* family = dynamic_cast<AmpAssembleController*>(entry.second.get());
      if (family && family->ownsMember(name))
        return entry.first;
    }
    return name;
  }

  AmpAssembleController* RLControllerManager::currentAmpFamilyLocked() const
  {
    const auto it = controllers_.find(current_controller_name_);
    return it == controllers_.end() ? nullptr :
        dynamic_cast<AmpAssembleController*>(it->second.get());
  }

  void RLControllerManager::clearPendingAmpFamilyExitLocked(bool cancel_exit)
  {
    if (cancel_exit)
    {
      const auto it = controllers_.find(pending_amp_family_exit_source_name_);
      if (it != controllers_.end())
        if (auto* family = dynamic_cast<AmpAssembleController*>(it->second.get()))
          family->cancelFamilyExit();
    }
    pending_amp_family_exit_ = false;
    pending_direct_mpc_exit_ = false;
    pending_amp_family_exit_source_name_.clear();
    pending_amp_family_exit_target_name_.clear();
  }

  std::optional<bool> RLControllerManager::routeAmpFamilySwitchLocked(const std::string& name)
  {
    const std::string resolved_name = resolveControllerNameLocked(name);
    if (resolved_name != name)
    {
      auto* target_family = dynamic_cast<AmpAssembleController*>(controllers_.at(resolved_name).get());
      const bool direct_entry = current_controller_name_.empty() && target_family &&
          target_family->supportsDirectMpcManipulateSwitch(name);
      if (direct_entry && !directMpcManipulateEntryReadyLocked())
      {
        logSwitchBlocked("MPC->MANIPULATE requires fresh stationary stance, zero command, and settled automatic arms");
        return false;
      }
      // A stationary MPC request can activate MANIPULATE at the outer handoff.
      // Other requests activate the family root before routing the member.
      if (current_controller_name_ != resolved_name)
      {
        const bool activated = switchControllerWithEntryMember(resolved_name, direct_entry ? name : std::string());
        if (!activated)
          return false;
      }
      if (current_controller_name_ != resolved_name)
      {
        pending_amp_member_request_ = name;
        pending_amp_member_target_name_ = resolved_name;
        return true;
      }
      if (direct_entry)
        return target_family->activeMemberName() == name;
      auto* family = currentAmpFamilyLocked();
      if (!family)
        return false;
      clearPendingAmpFamilyExitLocked(true);
      pending_mpc_switch_ = false;
      pending_amp_member_request_.clear();
      pending_amp_member_target_name_.clear();
      clearPendingWalkingSwitchRequest("explicit AMP member request");
      return family->requestMember(name);
    }

    if (auto* family = currentAmpFamilyLocked())
    {
      if (name == current_controller_name_)
      {
        clearPendingAmpFamilyExitLocked(true);
        family->cancelFamilyExit();
        pending_mpc_switch_ = false;
        pending_amp_member_request_.clear();
        pending_amp_member_target_name_.clear();
        clearPendingWalkingSwitchRequest("explicit AMP family re-selection");
        return true;
      }
      if (name != current_controller_name_)
      {
        const auto target = controllers_.find(name);
        if (!name.empty() && (target == controllers_.end() || !target->second))
        {
          logSwitchBlocked("target controller '" + name + "' is not loaded");
          return false;
        }
        const bool emergency = target != controllers_.end() && target->second &&
            target->second->getType() == RLControllerType::FALL_STAND_CONTROLLER;
        if (emergency)
        {
          clearPendingAmpFamilyExitLocked(false);
          pending_mpc_switch_ = false;
          pending_amp_member_request_.clear();
          pending_amp_member_target_name_.clear();
        }
        else
        {
          // MPC never inherits the per-member permission for direct RL exits.
          if (name.empty() && !ampMpcSwitchReadyLocked())
          {
            logSwitchBlocked("AMP->MPC blocked: stop commands and wait for physical standstill, valid state and blend completion");
            return false;
          }
          // Reject atomically before creating pending state or closing command
          // gates. Covers next/previous, named and navigation requests. Emergency
          // FallStand above and family-internal member routing remain available.
          const bool direct_mpc_exit = name.empty() &&
              family->supportsDirectMpcManipulateSwitch(family->activeMemberName());
          if (completing_amp_family_exit_ && pending_direct_mpc_exit_ && !direct_mpc_exit)
          {
            clearPendingAmpFamilyExitLocked(true);
            return false;
          }
          const auto ready = family->requestFamilyExit(!name.empty(), direct_mpc_exit);
          if (!ready.has_value())
          {
            logSwitchBlocked(direct_mpc_exit
                ? "AMP->MPC direct exit requires MANIPULATE with no active walking, arm action, or external control"
                : "AMP RUN/retirement blend blocks ordinary family exit; retire to WALK first, then request again");
            return false;
          }
          if (pending_amp_family_exit_ && pending_amp_family_exit_target_name_ != name)
          {
            pending_mpc_switch_ = false;
            clearPendingWalkingSwitchRequest("superseded AMP family exit");
          }
          pending_amp_family_exit_ = true;
          pending_direct_mpc_exit_ = direct_mpc_exit;
          pending_amp_family_exit_source_name_ = current_controller_name_;
          pending_amp_family_exit_target_name_ = name;
          // Keep the target until member routing and arm preparation complete.
          // Direct RL exits retain the current member; MPC keeps its own gate.
          if (!completing_amp_family_exit_ || !*ready || outer_blend_active_)
            return true;
          if (direct_mpc_exit && !family->familyExitReady())
            return true;
          if (!allowWalkingPhaseSyncSwitchRequest(name) && !isTorsoVelocityStable())
            return true;
        }
      }
    }

    return std::nullopt;
  }

  bool RLControllerManager::processAmpFamilySwitch(bool outer_blend_active)
  {
    AmpAssembleController* family = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      outer_blend_active_ = outer_blend_active;
      family = currentAmpFamilyLocked();
    }
    // advance can invoke child command gates that call back into the manager.
    // Do not hold mutex_ while advancing the family.
    if (family)
    {
      family->advance(outer_blend_active);
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (currentAmpFamilyLocked() != family)
        return true;
      if (pending_direct_mpc_exit_ &&
          (!ampMpcSwitchReadyLocked() || !family->directMpcExitIntentStillSafe()))
      {
        clearPendingAmpFamilyExitLocked(true);
        pending_mpc_switch_ = false;
        return true;
      }
      if (pending_amp_family_exit_ && !pending_mpc_switch_ && !outer_blend_active &&
          pending_amp_family_exit_source_name_ == current_controller_name_ &&
          family->familyExitReady())
      {
        // switchController preserves the request if a legacy outer gate or
        // arm return still blocks it; clear only after an actual activation.
        const std::string target_name = pending_amp_family_exit_target_name_;
        completing_amp_family_exit_ = true;
        switchController(target_name);
        completing_amp_family_exit_ = false;
      }
      return true;
    }
    return false;
  }

  bool RLControllerManager::notifyAmpFamilyExternalControlLocked()
  {
    if (auto* family = currentAmpFamilyLocked())
    {
      family->notifyExternalControl();
      return true;
    }
    return false;
  }

  void RLControllerManager::applyPendingAmpMemberRequestLocked()
  {
    if (auto* family = currentAmpFamilyLocked())
    {
      if (pending_amp_member_target_name_ == current_controller_name_ && !pending_amp_member_request_.empty())
        family->requestMember(pending_amp_member_request_);
    }
    pending_amp_member_request_.clear();
    pending_amp_member_target_name_.clear();
  }

  void RLControllerManager::updateAmpMpcSafety(bool stationary, const ros::Time& stamp)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    amp_mpc_stationary_ = stationary;
    // Repeated stale sensor data must not refresh the wall-clock watchdog.
    if (stamp != amp_mpc_sensor_stamp_)
    {
      amp_mpc_sensor_stamp_ = stamp;
      amp_mpc_sample_wall_time_ = ros::WallTime::now();
    }
  }

  bool RLControllerManager::ampMpcStationarySampleReadyLocked() const
  {
    // sensor_time is an open-loop 1 ms integrator and drifts away from ros::Time.
    // A changed stamp refreshes the wall clock, so this still rejects a frozen sample.
    const bool fresh = !amp_mpc_sensor_stamp_.isZero() && !amp_mpc_sample_wall_time_.isZero() &&
        (ros::WallTime::now() - amp_mpc_sample_wall_time_).toSec() <= 0.1;
    return fresh && amp_mpc_stationary_;
  }

  bool RLControllerManager::directMpcManipulateEntryReadyLocked() const
  {
    return current_controller_name_.empty() && !outer_blend_active_ &&
        !pending_vmp_to_amp_switch_ && !pending_arm_prepared_switch_ &&
        !pending_mpc_switch_ && !pending_amp_family_exit_ &&
        mpc_is_stance_mode_ && mpc_current_gait_name_ == "stance" &&
        !direct_mpc_walking_intent_ &&
        direct_mpc_arm_auto_ready_ && direct_mpc_stopped_command_ &&
        !isRobotActionActiveForAutoSwitchLocked(ros::Time::now()) &&
        ampMpcStationarySampleReadyLocked();
  }

  bool RLControllerManager::ampMpcSwitchReadyLocked() const
  {
    const auto* family = currentAmpFamilyLocked();
    return family && !outer_blend_active_ && family->canSwitchToMpc() &&
        ampMpcStationarySampleReadyLocked();
  }

  bool RLControllerManager::logAmpFamilyExitRetry(const std::string& message) const
  {
    // Some service callers log outside mutex_; synchronize the retry flag read.
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!completing_amp_family_exit_)
      return false;
    // ROS emits this diagnostic to its console as well: do not duplicate an
    // unthrottled copy on std::cerr for every pending control-cycle attempt.
    ROS_WARN_THROTTLE(1.0, "%s", message.c_str());
    return true;
  }

} // namespace humanoid_controller
