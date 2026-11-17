// Pinocchio must be included before Boost headers.
#include <pinocchio/fwd.hpp>
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
    pending_amp_family_exit_source_name_.clear();
    pending_amp_family_exit_target_name_.clear();
  }

  std::optional<bool> RLControllerManager::routeAmpFamilySwitchLocked(const std::string& name)
  {
    const std::string resolved_name = resolveControllerNameLocked(name);
    if (resolved_name != name)
    {
      // Activate the family root first, then request its member. A deferred
      // VMP handoff keeps the requested member until that root is activated.
      if (current_controller_name_ != resolved_name && !switchController(resolved_name))
        return false;
      if (current_controller_name_ != resolved_name)
      {
        pending_amp_member_request_ = name;
        pending_amp_member_target_name_ = resolved_name;
        return true;
      }
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
          const auto ready = family->requestFamilyExit(!name.empty());
          if (!ready.has_value())
          {
            logSwitchBlocked("AMP RUN/retirement blend blocks ordinary family exit; retire to WALK first, then request again");
            return false;
          }
          if (pending_amp_family_exit_ && pending_amp_family_exit_target_name_ != name)
          {
            pending_mpc_switch_ = false;
            clearPendingWalkingSwitchRequest("superseded AMP family exit");
          }
          pending_amp_family_exit_ = true;
          pending_amp_family_exit_source_name_ = current_controller_name_;
          pending_amp_family_exit_target_name_ = name;
          // Keep the target until member routing and arm preparation complete.
          // Direct RL exits retain the current member; MPC keeps its own gate.
          if (!completing_amp_family_exit_ || !*ready || outer_blend_active_)
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

  bool RLControllerManager::ampMpcSwitchReadyLocked() const
  {
    const auto* family = currentAmpFamilyLocked();
    const auto now = ros::Time::now();
    const bool fresh = !amp_mpc_sensor_stamp_.isZero() && now >= amp_mpc_sensor_stamp_ &&
        (now - amp_mpc_sensor_stamp_).toSec() <= 0.1 && !amp_mpc_sample_wall_time_.isZero() &&
        (ros::WallTime::now() - amp_mpc_sample_wall_time_).toSec() <= 0.1;
    return family && !outer_blend_active_ && family->canSwitchToMpc() && fresh && amp_mpc_stationary_;
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
