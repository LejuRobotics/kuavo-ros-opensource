#pragma once
#include "humanoid_controllers/rl/AmpFamilyPolicy.h"
#include <yaml-cpp/yaml.h>
#include <set>
#include "humanoid_controllers/rl/RunThermalGuard.h"
#include "humanoid_controllers/rl/RunToWalkElbowInterpolation.h"
#include <string>
#include <type_traits>

namespace humanoid_controller {
// Parse/validate the entire family before creating any model or ROS endpoint.
struct AmpFamilyConfig {
  using Role = AmpFamilyPolicy::Role;
  inline static constexpr const char* roles[] = {"manipulate", "walk", "run", "soft", "walk_waq"};
  struct Member { bool enabled = false; bool allow_direct_rl_switch = true; std::string name, config_file; };
  std::array<Member, 5> members;
  std::string anchor;
  AmpFamilyPolicy::Config switching;
  double command_timeout = 0.5, external_timeout = 0.5, blend_duration = 0.5;
  double run_decel_tau = 0.0, run_accel_tau = 0.0;
  RunToWalkElbowInterpolation::Config elbow_interpolation;

  RunThermalGuard::Config thermal;
  std::vector<int> thermal_indices{4,5,10,11};
  std::string thermal_topic="/sensor_data_motor/motor_ntc_temperature";
  double thermal_walk_speed=0.6, thermal_exit_timeout=5;

  static AmpFamilyConfig parse(const YAML::Node& yaml, const std::string& family_name) {
    AmpFamilyConfig result;
    const auto entries = yaml["members"];
    if (!entries || !entries.IsMap()) throw std::invalid_argument("AMP members must be a map");
    for (const auto& entry : entries) {
      const auto key = entry.first.as<std::string>();
      if (std::none_of(std::begin(roles), std::end(roles), [&](const char* role) { return key == role; }))
        throw std::invalid_argument("Unknown AMP role: " + key);
    }
    std::set<std::string> names;
    for (size_t i = 0; i < result.members.size(); ++i) {
      auto& member = result.members[i];
      const auto spec = entries[roles[i]];
      if (spec && !spec.IsMap()) throw std::invalid_argument(std::string("AMP member must be a map: ") + roles[i]);
      member.enabled = spec && (!spec["enabled"] || spec["enabled"].as<bool>());
      result.switching.enabled[i] = member.enabled;
      if (spec && spec["anchor"])
        throw std::invalid_argument("Use family-level anchor: <member name>, not members.<role>.anchor");
      member.allow_direct_rl_switch = spec && spec["allow_direct_rl_switch"]
          ? spec["allow_direct_rl_switch"].as<bool>() : i != static_cast<size_t>(Role::Run);
      result.switching.allow_direct_rl_switch[i] = member.allow_direct_rl_switch;
      if (!member.enabled) continue;
      member.name = spec["name"].as<std::string>();
      member.config_file = spec["config_file"].as<std::string>();
      if (member.name.empty() || member.name == family_name || !names.insert(member.name).second)
        throw std::invalid_argument("Enabled AMP member names must be unique and different from the family name");
      if (member.config_file.empty()) throw std::invalid_argument("Empty AMP config_file: " + member.name);
    }
    if (!result.members[static_cast<size_t>(Role::Walk)].enabled)
      throw std::invalid_argument("AMP WALK anchor must be present and enabled");
    const auto& walk = result.members[static_cast<size_t>(Role::Walk)];
    // Preserve existing family files that omit anchor. Explicit names must match
    // the enabled WALK member; changing routing roles is a separate operation.
    result.anchor = yaml["anchor"] ? yaml["anchor"].as<std::string>() : walk.name;
    if (result.anchor != walk.name)
      throw std::invalid_argument("AMP anchor must name the enabled WALK member: " + walk.name);
    const auto sw = yaml["switching"];
    if (sw && !sw.IsMap()) throw std::invalid_argument("AMP switching must be a map");
    // Reject obsolete family keys instead of silently using default thresholds.
    if (sw && (sw["runEnter"] || sw["runExit"]))
      throw std::invalid_argument("Rename AMP switching runEnter/runExit to walkTorun/runTowalk");
    const auto read = [&](const char* key, double& destination) {
      if (sw && sw[key]) destination = sw[key].as<double>();
    };
    auto& cfg = result.switching;
    if (sw && sw["commandThresholdUnits"]) {
      const auto units=sw["commandThresholdUnits"].as<std::string>();
      if(units!="ratio" && units!="velocity")
        throw std::invalid_argument("commandThresholdUnits must be ratio or velocity");
      cfg.command_threshold_ratios=units=="ratio";
    }
    read("walkTorun", cfg.walk_to_run); read("runTowalk", cfg.run_to_walk);
    read("runToWalkSpeedLimit", cfg.run_to_walk_speed_limit);
    read("runMaxAng", cfg.run_max_angular); read("runMinBodySpeed", cfg.run_min_body_speed);
    read("manipMax", cfg.manipulation_max_speed); read("walkEnter", cfg.walk_enter);
    read("intentHold", cfg.intent_hold); read("minDwellTime", cfg.min_dwell);
    read("commandTimeout", result.command_timeout); read("externalHoldTime", result.external_timeout);
    read("blendDuration", result.blend_duration);
    read("runDecelTau", result.run_decel_tau);
    if (!std::isfinite(result.run_decel_tau) || result.run_decel_tau < 0)
      throw std::invalid_argument("AMP runDecelTau must be finite and nonnegative");
    read("runAccelTau", result.run_accel_tau);
    if (!std::isfinite(result.run_accel_tau) || result.run_accel_tau < 0)
      throw std::invalid_argument("AMP runAccelTau must be finite and nonnegative");
    const auto elbow = sw ? sw["runToWalkElbowGuard"] : YAML::Node();
    if (elbow && !elbow.IsNull()) {
      if (!elbow.IsMap()) throw std::invalid_argument("runToWalkElbowGuard must be a map");
      const std::set<std::string> keys{"enabled", "jointIndices", "minPosition", "maxPosition", "holdTime", "maxSampleAge"};
      for (const auto& entry : elbow)
        if (!keys.count(entry.first.as<std::string>()))
          throw std::invalid_argument("Unknown runToWalkElbowGuard setting: " + entry.first.as<std::string>());
      if (elbow["enabled"]) cfg.run_to_walk_elbow_guard = elbow["enabled"].as<bool>();
      const auto pair = [&](const char* key, auto& values) {
        if (!elbow[key]) return;
        if (!elbow[key].IsSequence() || elbow[key].size() != 2)
          throw std::invalid_argument(std::string("Elbow guard requires [left, right]: ") + key);
        using Value = std::decay_t<decltype(values[0])>;
        for (size_t i = 0; i < 2; ++i) values[i] = elbow[key][i].as<Value>();
      };
      pair("jointIndices", cfg.elbow_joint_indices); pair("minPosition", cfg.elbow_min); pair("maxPosition", cfg.elbow_max);
      if (elbow["holdTime"]) cfg.elbow_hold_time = elbow["holdTime"].as<double>();
      if (elbow["maxSampleAge"]) cfg.elbow_sample_timeout = elbow["maxSampleAge"].as<double>();
    }
    for (double time : {result.command_timeout, result.external_timeout, result.blend_duration})
      if (!std::isfinite(time) || time <= 0)
        throw std::invalid_argument("AMP timeouts and blendDuration must be positive");
    const auto th = yaml["runThermalProtection"];
    if (th) {
      if (!th.IsMap()) throw std::invalid_argument("runThermalProtection must be a map");
      const std::set<std::string> keys{"enabled","ntcTopic","motorIndices","entryTemperature","exitTemperature",
        "recoveryHold","sampleTimeout","maxSampleDrop",
        "walkSpeedLimit","exitTimeout"};
      for(const auto& item:th) if(!keys.count(item.first.as<std::string>()))
        throw std::invalid_argument("Unknown thermal parameter: "+item.first.as<std::string>());
      if(th["enabled"]) result.thermal.enabled=th["enabled"].as<bool>();
      if(th["ntcTopic"]) result.thermal_topic=th["ntcTopic"].as<std::string>();
      if(th["motorIndices"]) result.thermal_indices=th["motorIndices"].as<std::vector<int>>();
      const auto number=[&](const char* key,double& value){if(th[key]) value=th[key].as<double>();};
      number("entryTemperature",result.thermal.entry); number("exitTemperature",result.thermal.exit);
      number("recoveryHold",result.thermal.recovery_hold);
      number("sampleTimeout",result.thermal.sample_timeout); number("maxSampleDrop",result.thermal.max_sample_drop);
      number("walkSpeedLimit",result.thermal_walk_speed); number("exitTimeout",result.thermal_exit_timeout);
    }
    (void)RunThermalGuard(result.thermal);
    std::set<int> indices(result.thermal_indices.begin(),result.thermal_indices.end());
    if(indices.empty() || indices.size()!=result.thermal_indices.size() || *indices.begin()<0 || *indices.rbegin()>=32)
      throw std::invalid_argument("Invalid/duplicate thermal motorIndices");
    if(result.thermal_topic.empty() || result.thermal_topic.front()!='/' || !std::isfinite(result.thermal_walk_speed) || result.thermal_walk_speed<=0 ||
       !std::isfinite(result.thermal_exit_timeout) || result.thermal_exit_timeout<=0)
      throw std::invalid_argument("Invalid thermal topic/walkSpeedLimit/exitTimeout");
    const auto interpolation = sw ? sw["runToWalkElbowInterpolation"] : YAML::Node();
    if (interpolation && !interpolation.IsNull()) {
      if (!interpolation.IsMap()) throw std::invalid_argument("runToWalkElbowInterpolation must be a map");
      const std::set<std::string> keys{"enabled", "targetPosition", "duration", "releaseDuration"};
      for (const auto& entry : interpolation)
        if (!keys.count(entry.first.as<std::string>()))
          throw std::invalid_argument("Unknown RUN elbow interpolation setting: " + entry.first.as<std::string>());
      auto& c = result.elbow_interpolation;
      if (interpolation["enabled"]) c.enabled = interpolation["enabled"].as<bool>();
      if (interpolation["targetPosition"]) c.target = interpolation["targetPosition"].as<std::array<double,2>>();
      if (interpolation["duration"]) c.duration = interpolation["duration"].as<double>();
      if (interpolation["releaseDuration"]) c.release_duration = interpolation["releaseDuration"].as<double>();
      RunToWalkElbowInterpolation validator; validator.configure(c);
      if (c.enabled) {
        if (!cfg.run_to_walk_elbow_guard || !cfg.enabled[static_cast<size_t>(Role::Run)])
          throw std::invalid_argument("RUN elbow interpolation requires RUN and elbow guard enabled");
        for (size_t i=0;i<2;++i)
          if (c.target[i] <= cfg.elbow_min[i] || c.target[i] >= cfg.elbow_max[i])
            throw std::invalid_argument("RUN elbow target must be strictly inside the measured elbow guard range");
      }
    }
    (void)AmpFamilyPolicy(cfg); // Validate the same thresholds used at runtime.
    return result;
  }
};
} // namespace humanoid_controller
