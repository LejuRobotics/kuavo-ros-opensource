// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerManager.h"
#include "humanoid_controllers/rl/AutoControllerSwitchPolicy.h"
#include "humanoid_controllers/rl/FallStandController.h"
#include "humanoid_controllers/rl/AmpWalkController.h"
#include "humanoid_controllers/rl/DepthWalkController.h"
#include "humanoid_controllers/rl/VMPController.h"
#include "humanoid_controllers/rl/MoREController.h"
#include "humanoid_controllers/rl/DanceController.h"
#include <algorithm>
#include <ros/master.h>
#include <ros/topic.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include "kuavo_msgs/changeArmCtrlMode.h"
#include <thread>
#include <cctype>
#include <iostream>

namespace humanoid_controller
{
  namespace
  {
    bool isStanceInterpolatedControllerClass(ControllerClass controller_class)
    {
      return controller_class == ControllerClass::BASE_CONTROLLER ||
             controller_class == ControllerClass::DANCE_CONTROLLER;
    }

    const char* switchMotionStateName(RLControllerManager::SwitchMotionState state)
    {
      switch (state)
      {
        case RLControllerManager::SwitchMotionState::STANCE:
          return "STANCE";
        case RLControllerManager::SwitchMotionState::STATIONARY:
          return "STATIONARY";
        case RLControllerManager::SwitchMotionState::WALKING:
          return "WALKING";
      }
      return "UNKNOWN";
    }

    constexpr char kDepthHistoryTopic[] = "/camera/depth/depth_history_array";
  }  // namespace

  RLControllerManager::RLControllerManager() : current_controller_name_(""), 
                                                nh_ptr_(nullptr)
  {
    // 初始化BASE_CONTROLLER列表，MPC控制器在索引0
    walk_controllers_.push_back("mpc");
  }

  //waao: RL切换请求等待处理
  void RLControllerManager::queuePendingWalkingSwitchRequest(const std::string& target_name, const std::string& reason)
  {
    has_pending_walking_switch_request_ = true;
    pending_walking_switch_source_name_ = current_controller_name_;
    pending_walking_switch_target_name_ = target_name;
    pending_walking_switch_reason_ = reason;

    ROS_WARN("[RLControllerManager] Queue pending walking RL->RL switch request: %s -> %s. reason: %s",
             pending_walking_switch_source_name_.c_str(),
             pending_walking_switch_target_name_.c_str(),
             pending_walking_switch_reason_.c_str());
  }

  void RLControllerManager::clearPendingWalkingSwitchRequest(const std::string& reason)
  {
    if (has_pending_walking_switch_request_)
    {
      if (!reason.empty())
      {
        ROS_INFO("[RLControllerManager] Clear pending walking RL->RL switch request: %s -> %s. reason: %s",
                 pending_walking_switch_source_name_.c_str(),
                 pending_walking_switch_target_name_.c_str(),
                 reason.c_str());
      }
      else
      {
        ROS_INFO("[RLControllerManager] Clear pending walking RL->RL switch request: %s -> %s",
                 pending_walking_switch_source_name_.c_str(),
                 pending_walking_switch_target_name_.c_str());
      }
    }

    has_pending_walking_switch_request_ = false;
    pending_walking_switch_source_name_.clear();
    pending_walking_switch_target_name_.clear();
    pending_walking_switch_reason_.clear();
  }

  RLControllerManager::~RLControllerManager()
  {
    depth_history_monitor_.stop();
  }

  void RLControllerManager::startDepthHistoryMonitor()
  {
    if (!nh_ptr_)
    {
      return;
    }

    depth_history_monitor_.setMaxSamples(depth_history_required_samples_);
    depth_history_monitor_.start<std_msgs::Float64MultiArray>(*nh_ptr_, kDepthHistoryTopic, 200);
  }

  bool RLControllerManager::addController(const std::string& name, std::unique_ptr<RLControllerBase> controller)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (name.empty())
    {
      ROS_ERROR("[RLControllerManager] Controller name cannot be empty");
      return false;
    }

    if (controllers_.find(name) != controllers_.end())
    {
      ROS_WARN("[RLControllerManager] Controller '%s' already exists, replacing it", name.c_str());
    }

    // 启动控制器推理线程（创建线程并设置为 PAUSED 状态）
    controller->start();

    controller->setCommandBufferCallback([this]() {
      std::function<bool()> walking_command_block_callback;
      {
        std::lock_guard<std::recursive_mutex> lock(this->mutex_);
        walking_command_block_callback = this->walking_command_block_callback_;
      }
      return !this->isWalkingCommandExecutionAllowed() ||
             this->shouldBufferWalkingCommand() ||
             (walking_command_block_callback && walking_command_block_callback());
    });
    controller->setExternalCommandBufferCallback([this]() {
      return !this->isExternalControlCommandExecutionAllowed() || this->shouldBufferExternalControlCommand();
    });

    controllers_[name] = std::move(controller);
    
    // 如果名称不在列表中，添加到列表
    if (std::find(controller_names_.begin(), controller_names_.end(), name) == controller_names_.end())
    {
      controller_names_.push_back(name);
    }

    // 更新按类型分组的列表
    updateControllerListsByType();

    ROS_INFO("[RLControllerManager] Added controller '%s' (type: %d) in paused state", name.c_str(), 
             static_cast<int>(controllers_[name]->getType()));
    return true;
  }

  RLControllerBase* RLControllerManager::getControllerByName(const std::string& name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    auto it = controllers_.find(name);
    if (it != controllers_.end())
    {
      return it->second.get();
    }
    return nullptr;
  }

  RLControllerBase* RLControllerManager::getControllerByIndex(size_t index)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (index >= controller_names_.size())
    {
      return nullptr;
    }

    std::string name = controller_names_[index];
    return getControllerByName(name);
  }

  RLControllerBase* RLControllerManager::getControllerByType(RLControllerType type)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    for (const auto& pair : controllers_)
    {
      if (pair.second->getType() == type)
      {
        return pair.second.get();
      }
    }
    return nullptr;
  }

  RLControllerType RLControllerManager::getControllerTypeByName(const std::string& name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    auto it = controllers_.find(name);
    if (it != controllers_.end())
    {
      return it->second->getType();
    }
    return RLControllerType::MPC;
  }

  RLControllerType RLControllerManager::getControllerTypeByIndex(size_t index)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (index >= controller_names_.size())
    {
      return RLControllerType::MPC;
    }

    std::string name = controller_names_[index];
    return getControllerTypeByName(name);
  }

  void RLControllerManager::updateSwitchMotionStateLocked()
  {
    if (current_controller_name_.empty())
    {
      switch_motion_state_ = SwitchMotionState::STANCE;
      stationary_candidate_active_ = false;
      return;
    }

    const auto current_it = controllers_.find(current_controller_name_);
    if (current_it == controllers_.end() || current_it->second == nullptr)
    {
      switch_motion_state_ = SwitchMotionState::WALKING;
      stationary_candidate_active_ = false;
      return;
    }

    auto* current_controller = current_it->second.get();
    if (current_controller->isAllowToExit())
    {
      switch_motion_state_ = SwitchMotionState::STANCE;
      stationary_candidate_active_ = false;
      return;
    }

    const bool command_is_near_zero = current_controller->hasNearZeroGaitCommand(
        RL_SWITCH_STATIONARY_COMMAND_LINEAR_THRESHOLD_DEFAULT,
        RL_SWITCH_STATIONARY_COMMAND_ANGULAR_THRESHOLD_DEFAULT);
    const bool physical_state_is_stationary =
        stationary_physical_state_callback_ && stationary_physical_state_callback_();
    if (!command_is_near_zero || !physical_state_is_stationary)
    {
      switch_motion_state_ = SwitchMotionState::WALKING;
      stationary_candidate_active_ = false;
      return;
    }

    const double hold_time = std::max(0.0, static_cast<double>(RL_SWITCH_STATIONARY_HOLD_TIME_DEFAULT));
    if (hold_time <= 0.0)
    {
      switch_motion_state_ = SwitchMotionState::STATIONARY;
      stationary_candidate_active_ = true;
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!stationary_candidate_active_ || now < stationary_candidate_start_time_)
    {
      stationary_candidate_start_time_ = now;
      stationary_candidate_active_ = true;
      switch_motion_state_ = SwitchMotionState::WALKING;
      return;
    }

    switch_motion_state_ =
        (now - stationary_candidate_start_time_).toSec() >= hold_time
            ? SwitchMotionState::STATIONARY
            : SwitchMotionState::WALKING;
  }

  void RLControllerManager::updateSwitchMotionState()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    updateSwitchMotionStateLocked();
  }

  RLControllerManager::SwitchMotionState RLControllerManager::getLastSwitchMotionState() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return last_switch_motion_state_;
  }

  bool RLControllerManager::checkWalkingPhaseSyncSwitchGuard(const std::string& target_name, std::string& message)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!allowWalkingPhaseSyncSwitchRequest(target_name))
    {
      return true;
    }

    if (!walking_phase_sync_switch_guard_callback_)
    {
      return true;
    }

    const std::string current_name = current_controller_name_.empty() ? "mpc" : current_controller_name_;
    return walking_phase_sync_switch_guard_callback_(current_name, target_name, message);
  }

  //waao：控制器切换逻辑
  bool RLControllerManager::switchController(const std::string& name)
  {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    const std::string current_before = current_controller_name_.empty() ? "mpc" : current_controller_name_;

    ocs2::humanoid::CommandDataRL source_gait_command;
    bool has_source_gait_command = false;
    SwitchMotionState source_motion_state = SwitchMotionState::STANCE;
    bool source_is_stance_or_stationary = false;
    bool allow_walking_phase_sync_switch = false;
    
    // mimic 类控制器（倒地起身/舞蹈）：仅在未完成（isAllowToExit==false）时禁止切出
    if (!current_controller_name_.empty())
    {
      auto* current_controller = controllers_[current_controller_name_].get();
      const bool current_is_mimic_controller =
          current_controller->getType() == RLControllerType::FALL_STAND_CONTROLLER ||
          current_controller->getType() == RLControllerType::DANCE_CONTROLLER;
      if (!current_controller->isAllowToExit() && current_is_mimic_controller)
      {
        logSwitchBlocked("current mimic controller '" + current_controller_name_ + "' is not ready to exit");
        return false;
      }
      

      has_source_gait_command = current_controller->getGaitCommandState(source_gait_command);
      updateSwitchMotionStateLocked();
      source_motion_state = switch_motion_state_;
      source_is_stance_or_stationary = source_motion_state != SwitchMotionState::WALKING;
    }

    // 切换到MPC控制器
    if (name.empty())
    {
      // 保护逻辑：RL->MPC 切换时，如果 RL 控制器不在 stance 且躯干仍在运动，不允许切换
      // （对应 V1.1 SwitchMotionState：STANCE/STATIONARY 放行，WALKING 拒绝。
      //   HEAD 已移除 SwitchMotionState 基础设施，用 isTorsoVelocityStable 替代 STATIONARY 判断。）
      if (!current_controller_name_.empty())
      {
        auto* current_controller = controllers_[current_controller_name_].get();
        if (current_controller && source_motion_state == SwitchMotionState::WALKING)
        {
          // 躯干已稳定（物理静止）→ 允许 RL→MPC，对应 V1.1 的 STATIONARY 状态
          if (!isTorsoVelocityStable())
          {
            ROS_WARN_THROTTLE(1.0, "[RLControllerManager] RL not in stance and torso not stable, switch to MPC blocked! Stop walking first.");
            return false;
          }
          ROS_INFO("[RLControllerManager] Allowing RL->MPC switch with stable torso (stationary)");
        }
        if (current_controller && source_motion_state == SwitchMotionState::STATIONARY)
        {
          ROS_INFO("[RLControllerManager] Allow RL->MPC switch in STATIONARY state: %s",
                   current_controller_name_.c_str());
        }

        if (current_controller){
          // RL→MPC 手臂保护：外部控制时先切回 AUTO_SWING，保持 RL 运行直到归位
          auto* arm_ctrl = current_controller->getArmController();
          if (arm_ctrl && arm_ctrl->getMode() != 1)
          {
            changeArmCtrlModeAsync(1);
            arm_ctrl->changeMode(1);
            pending_mpc_switch_ = true;
            ROS_INFO("[RLControllerManager] RL→MPC: arm in external mode, deferring switch until interpolation completes");
            return true;  // RL 保持运行，不切
          }
          current_controller->pause();

          if (nh_ptr_){
            current_controller->RLControllerBase::updateVelocityLimitsParam(*nh_ptr_);
          }

          last_controller_ptr_ = current_controller;
        }
      }
      
      current_controller_name_ = "";
      last_switch_motion_state_ = source_motion_state;
      clearPendingWalkingSwitchRequest("switched to MPC");

      const std::string to_controller = "mpc";
      ROS_INFO("[RLControllerManager] Switched to BASE controller");
      // 切换到 MPC 控制器时也异步切换手臂模式到 1
      changeArmCtrlModeAsync(1);
      if (current_before != to_controller)
      {
        publishControllerSwitchEvent(current_before, to_controller);
      }
      return true;
    }

    // 检查控制器是否存在
    if (controllers_.find(name) == controllers_.end())
    {
      logSwitchBlocked("target controller '" + name + "' is not loaded");
      return false;
    }

    // 如果切换的是同一个控制器，直接返回成功
    if (current_controller_name_ == name)
    {
      clearPendingWalkingSwitchRequest("target controller already active");
      return true;
    }

    // 保护逻辑：MPC->RL 切换时，如果机器人不在 stance 状态，不允许切换
    if (current_controller_name_.empty())
    {
      auto* next_controller = controllers_[name].get();
      const bool desired_switch_to_falldown = next_controller->getType() == RLControllerType::FALL_STAND_CONTROLLER;
      
      const bool is_current_stance = (mpc_current_gait_name_ == "stance") || mpc_is_stance_mode_;
      if (!desired_switch_to_falldown && !is_current_stance)
      {
        logSwitchBlocked("MPC is not in stance (gait=" + mpc_current_gait_name_ + "). Stop walking first.");
        return false;
      }
    }
    else   //waao
    {
      auto* current_controller = controllers_[current_controller_name_].get();
      allow_walking_phase_sync_switch = allowWalkingPhaseSyncSwitchRequest(name);
      // 倒地应急切换(AMP→FallStand)不受行走保护限制——倒地必须无条件切入
      // (与上方 MPC→RL 分支的 desired_switch_to_falldown 同类例外一致)
      bool target_is_falldown = false;
      auto target_it = controllers_.find(name);
      if (target_it != controllers_.end() && target_it->second)
        target_is_falldown = target_it->second->getType() == RLControllerType::FALL_STAND_CONTROLLER;

      if (current_controller && source_motion_state == SwitchMotionState::WALKING &&
          !target_is_falldown && !allow_walking_phase_sync_switch)
      {
        logSwitchBlocked("RL->RL switch blocked because controller '" + current_controller_name_ + "' is not in stance");
        return false;
      }
      //waao：当前为行走状态
      if (current_controller && source_motion_state == SwitchMotionState::WALKING &&
          allow_walking_phase_sync_switch)
      {
        std::string switch_guard_message;
        if (!checkWalkingPhaseSyncSwitchGuard(name, switch_guard_message))
        {
          if (switch_guard_message.empty())
          {
            switch_guard_message = "walking phase sync switch guard rejected the request";
          }
          queuePendingWalkingSwitchRequest(name, switch_guard_message);
          return false;
        }
        clearPendingWalkingSwitchRequest("walking switch guard passed");
        ROS_INFO("[RLControllerManager] Allow walking RL->RL switch without stance: %s -> %s",
                 current_controller_name_.c_str(), name.c_str());
      }
      else
      {
        clearPendingWalkingSwitchRequest();
      }
    }

    auto it = controllers_.find(name);
    if (it != controllers_.end() && it->second &&
        it->second->getType() == RLControllerType::DEPTH_LOCO_CONTROLLER)
    {
      if (!canSwitchToDepthWalkController())
      {
        isDepthHistoryTopicAvailable(/*log=*/true);
        return false;
      }
    }

    // 暂停当前控制器（如果存在）
    if (!current_controller_name_.empty())
    {
      auto* current_controller = controllers_[current_controller_name_].get();
      if (current_controller)
      {
        current_controller->pause();
        // 仅记录上一个控制器的裸指针，所有权仍由 controllers_ 管理
        last_controller_ptr_ = current_controller;
      }
    }

    // 恢复新控制器（使用resume而不是start）
    auto* new_controller = controllers_[name].get();
    if (new_controller)
    {
      if (current_controller_name_.empty())
      {
        new_controller->resetGaitCommandState(true);
      }
#if RL_TO_RL_USE_WARM_RESUME
      bool use_warm_resume = false;
      if (!current_controller_name_.empty())
      {
        const auto current_class_it = controller_classes_.find(current_controller_name_);
        const auto target_class_it = controller_classes_.find(name);
        const bool current_supports_stance_interpolation =
            current_class_it != controller_classes_.end() &&
            isStanceInterpolatedControllerClass(current_class_it->second);
        const bool target_supports_stance_interpolation =
            target_class_it != controller_classes_.end() &&
            isStanceInterpolatedControllerClass(target_class_it->second);
        const bool use_stance_interpolated_resume =
            source_is_stance_or_stationary &&
            current_supports_stance_interpolation &&
            target_supports_stance_interpolation;
        use_warm_resume =
            (source_motion_state == SwitchMotionState::WALKING && allow_walking_phase_sync_switch) ||
            use_stance_interpolated_resume;
      }
#else
      const bool use_warm_resume = false;
#endif
      if (!current_controller_name_.empty())
      {
        if (source_is_stance_or_stationary)
        {
          new_controller->resetGaitCommandState(true);
          ROS_INFO("[RLControllerManager] Reset target gait command to stance during RL->RL switch: %s -> %s",
                   current_controller_name_.c_str(), name.c_str());
        }
        else if (auto_switch_config_.enabled &&
                 name == auto_switch_config_.manipulation_controller)
        {
          new_controller->resetGaitCommandState(true);
          ROS_INFO("[RLControllerManager] Reset manipulation controller gait command to stance during RL->RL switch: %s -> %s",
                   current_controller_name_.c_str(), name.c_str());
        }
        else if (has_source_gait_command)
        {
          new_controller->setGaitCommandState(source_gait_command);
          ROS_INFO("[RLControllerManager] Sync gait command state during RL->RL switch: %s -> %s",
                   current_controller_name_.c_str(), name.c_str());
        }
      }
      if (use_warm_resume)
      {
        new_controller->resumeWarm();
      }
      else
      {
        new_controller->resume();
      }
    }

    current_controller_name_ = name;
    last_switch_motion_state_ = source_motion_state;
    stationary_candidate_active_ = false;
    clearPendingWalkingSwitchRequest("switch completed");
    const std::string to_controller = current_controller_name_;

    ROS_INFO("[RLControllerManager] Switched to controller '%s' (type: %d, source motion state: %s)",
             name.c_str(),
             static_cast<int>(new_controller->getType()),
             switchMotionStateName(source_motion_state));

    // 调用控制器的更新速度限制接口
    if (nh_ptr_) {
      new_controller->updateVelocityLimitsParam(*nh_ptr_);
    }
    // 切换到非 MPC 控制器后，异步切换手臂模式到 1
    if (new_controller && new_controller->getType() != RLControllerType::MPC)
    {
      changeArmCtrlModeAsync(1);
    }
    if (current_before != to_controller)
    {
      publishControllerSwitchEvent(current_before, to_controller);
    }
    return true;
  }

  bool RLControllerManager::isDepthHistoryTopicAvailable(bool log)
  {
    if (!nh_ptr_)
    {
      if (log)
      {
        const std::string reason = "NodeHandle is null while checking topic '" + std::string(kDepthHistoryTopic) + "'.";
        logSwitchBlocked("[DepthLocoSwitch] " + reason);
      }
      return false;
    }

    // 仅读取后台缓存的统计数据，避免切换路径上阻塞等待消息（并把判据/原因下沉到 TopicMonitor）
    TopicMonitor::Requirements req;
    req.must_be_published = true;
    req.min_hz = depth_history_min_frequency_hz_;
    req.max_age_sec = depth_history_wait_timeout_sec_;
    req.min_samples = 2;

    TopicMonitor::CheckReport report;
    const auto res = depth_history_monitor_.check(req, &report);
    publishDepthHistoryStatus(res);
    if (res != TopicMonitor::CheckResult::Ok)
    {
      if (log)
        logSwitchBlocked("[DepthLocoSwitch] " + report.reason +
                         " Refuse to switch to depth_loco_controller.");
      return false;
    }

    if (log)
    {
      const std::string message = "[DepthLocoSwitch] " + report.reason;
      ROS_INFO("%s", message.c_str());
    }
    return true;
  }

  void RLControllerManager::updateDepthHistorySwitchFlag(const ros::TimerEvent& /*event*/)
  {
    const bool depth_history_ok = isDepthHistoryTopicAvailable(/*log=*/false);
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    can_switch_to_depth_walk_controller_ = depth_history_ok;
  }

  bool RLControllerManager::canSwitchToDepthWalkController() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return can_switch_to_depth_walk_controller_;
  }

  bool RLControllerManager::canSwitchTo(const std::string& name, std::string* reason)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    const auto blocked = [reason](const std::string& message) {
      if (reason)
        *reason = message;
      return false;
    };

    // 切到 MPC：除"当前控制器能否退出"（由调用方判）外没有目标侧前置
    if (name.empty())
      return true;

    // 目标控制器必须已加载
    auto it = controllers_.find(name);
    if (it == controllers_.end() || !it->second)
      return blocked("Target controller '" + name + "' is not loaded.");

    // 切到自身视为可行（真正执行时 switchController 会直接返回成功）
    if (current_controller_name_ == name)
      return true;

    auto* target = it->second.get();
    const bool target_is_falldown = target->getType() == RLControllerType::FALL_STAND_CONTROLLER;

    // 当前在 MPC 时：切到非倒地控制器要求 MPC 处于 stance（与 switchController 中的保护一致）
    if (current_controller_name_.empty() && !target_is_falldown)
    {
      const bool is_current_stance = (mpc_current_gait_name_ == "stance") || mpc_is_stance_mode_;
      if (!is_current_stance)
        return blocked("MPC is not in stance (gait=" + mpc_current_gait_name_ + "). Stop walking first.");
    }

    // depth_loco_controller：要求深度历史话题此刻可用（静默探测，不打日志）
    if (target->getType() == RLControllerType::DEPTH_LOCO_CONTROLLER)
    {
      if (!canSwitchToDepthWalkController())
      {
        TopicMonitor::Requirements req;
        req.must_be_published = true;
        req.min_hz = depth_history_min_frequency_hz_;
        req.max_age_sec = depth_history_wait_timeout_sec_;
        req.min_samples = 2;
        TopicMonitor::CheckReport report;
        depth_history_monitor_.check(req, &report);
        return blocked(report.reason.empty() ?
                       "Depth history topic is not ready." : report.reason);
      }
    }

    return true;
  }

  int RLControllerManager::findNextSwitchableIndex(int current_index, int dir,
                                                    std::vector<std::string>* skipped_reasons)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const int n = static_cast<int>(walk_controllers_.size());
    if (n <= 1)
      return -1;
    for (int step = 1; step <= n - 1; ++step)
    {
      const int idx = ((current_index + dir * step) % n + n) % n;
      const std::string probe = (idx == 0) ? std::string() : walk_controllers_[idx];  // 索引 0 固定为 MPC
      std::string reason;
      if (canSwitchTo(probe, &reason))
        return idx;
      if (skipped_reasons)
      {
        skipped_reasons->push_back((probe.empty() ? "mpc" : probe) + ": " + reason);
      }
    }
    return -1;
  }

  void RLControllerManager::logSwitchBlocked(const std::string& reason) const
  {
    const std::string message = "[RLControllerManager] Controller switch blocked: " + reason;
    ROS_WARN("%s", message.c_str());
    std::cerr << message << std::endl;
  }

  void RLControllerManager::loadAutoControllerSwitchConfig(const std::string& config_file)
  {
    AutoControllerSwitchConfig config;
    try
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(config_file, pt);

      const auto child = pt.get_child_optional("autoControllerSwitch");
      if (!child)
      {
        return;
      }

      const auto& node = child.get();
      config.enabled = node.get<bool>("enabled", false);
      config.manipulation_controller = node.get<std::string>("manipulationController", "");
      config.walking_controller = node.get<std::string>("walkingController", "");
      config.cmd_vel_linear_threshold = std::max(0.0, node.get<double>("cmdVelLinearThreshold", 0.02));
      config.cmd_vel_angular_threshold = std::max(0.0, node.get<double>("cmdVelAngularThreshold", 0.02));
      config.cmd_vel_command_hold_time = std::max(0.0, node.get<double>("cmdVelCommandHoldTime", 0.5));
      config.external_command_hold_time = std::max(0.0, node.get<double>("externalCommandHoldTime", 0.5));
      config.min_switch_interval = std::max(0.0, node.get<double>("minSwitchInterval", 1.0));
      config.walking_switch_require_stance = node.get<bool>("walkingSwitchRequireStance", true);
      config.switch_command_buffer_time = std::max(0.0, node.get<double>("switchCommandBufferTime", 0.0));

      if (config.enabled)
      {
        if (config.manipulation_controller.empty() || config.walking_controller.empty())
        {
          ROS_WARN("[RLControllerManager] autoControllerSwitch disabled because target controller name is empty in %s",
                   config_file.c_str());
          config.enabled = false;
        }
      }

      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto_switch_config_ = config;
        auto_switch_config_loaded_ = true;
      }

      if (config.enabled)
      {
        ROS_INFO("[RLControllerManager] autoControllerSwitch enabled: manipulation=%s walking=%s walkingRequireStance=%s commandBuffer=%.3fs",
                 config.manipulation_controller.c_str(),
                 config.walking_controller.c_str(),
                 config.walking_switch_require_stance ? "true" : "false",
                 config.switch_command_buffer_time);
      }
    }
    catch (const std::exception& e)
    {
      ROS_WARN("[RLControllerManager] Failed to load autoControllerSwitch from %s: %s",
               config_file.c_str(), e.what());
    }
  }

  void RLControllerManager::loadDepthHistoryCheckParams(ros::NodeHandle& nh)
  {
    constexpr double kDefaultMinFrequencyHz = 55.0;
    constexpr double kDefaultWaitTimeoutSec = 0.2;
    constexpr int kDefaultRequiredSamples = 10;
    constexpr double kDefaultSampleTimeoutSec = 0.2;
    constexpr const char* kLogTag = "[RLControllerManager]";

    nh.param("depth_history_min_frequency_hz", depth_history_min_frequency_hz_, depth_history_min_frequency_hz_);
    nh.param("depth_history_wait_timeout_sec", depth_history_wait_timeout_sec_, depth_history_wait_timeout_sec_);
    nh.param("depth_history_required_samples", depth_history_required_samples_, depth_history_required_samples_);
    nh.param("depth_history_sample_timeout_sec", depth_history_sample_timeout_sec_, depth_history_sample_timeout_sec_);

    TopicMonitor::ensurePositiveParam(kLogTag, "depth_history_min_frequency_hz", depth_history_min_frequency_hz_, kDefaultMinFrequencyHz);
    TopicMonitor::ensurePositiveParam(kLogTag, "depth_history_wait_timeout_sec", depth_history_wait_timeout_sec_, kDefaultWaitTimeoutSec);
    TopicMonitor::ensureMinIntParam(kLogTag, "depth_history_required_samples", depth_history_required_samples_, 2, kDefaultRequiredSamples);
    TopicMonitor::ensurePositiveParam(kLogTag, "depth_history_sample_timeout_sec", depth_history_sample_timeout_sec_, kDefaultSampleTimeoutSec);

    ROS_INFO("[RLControllerManager] Depth history check params: min_frequency=%.1f Hz, wait_timeout=%.3f s, required_samples=%d, sample_timeout=%.3f s",
             depth_history_min_frequency_hz_,
             depth_history_wait_timeout_sec_,
             depth_history_required_samples_,
             depth_history_sample_timeout_sec_);

    depth_history_monitor_.setMaxSamples(depth_history_required_samples_);
  }

  bool RLControllerManager::switchController(RLControllerType type)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // 切换到MPC控制器
    if (type == RLControllerType::MPC)
    {
      return switchController("");
    }

    // 查找指定类型的控制器
    for (const auto& pair : controllers_)
    {
      if (pair.second->getType() == type)
      {
        return switchController(pair.first);
      }
    }

    ROS_ERROR("[RLControllerManager] Controller with type %d not found", static_cast<int>(type));
    return false;
  }

  void RLControllerManager::changeArmCtrlModeAsync(int mode)
  {
    // 如果还没有初始化 NodeHandle，则无法调用服务
    if (!nh_ptr_)
    {
      ROS_WARN("[RLControllerManager] nh_ptr_ is null, cannot change arm control mode");
      return;
    }

    std::thread([this, mode]()
    {
      try
      {
        ros::NodeHandle nh = *nh_ptr_;
        kuavo_msgs::changeArmCtrlMode srv;
        srv.request.control_mode = mode;

        ros::ServiceClient client =
            nh.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");

        const ros::Duration timeout(2.0);
        if (!client.waitForExistence(timeout))
        {
          ROS_WARN_THROTTLE(1.0,
                            "[RLControllerManager] Arm ctrl mode service '/humanoid_change_arm_ctrl_mode' "
                            "not available (timeout: 2s)");
          return;
        }

        if (client.call(srv))
        {
          if (srv.response.result)
          {
            ROS_INFO("[RLControllerManager] Successfully changed arm control mode to %d", mode);
          }
          else
          {
            ROS_WARN("[RLControllerManager] Failed to change arm control mode to %d, current mode: %d",
                     mode, srv.response.mode);
          }
        }
        else
        {
          ROS_WARN("[RLControllerManager] Failed to call arm ctrl mode service '/humanoid_change_arm_ctrl_mode'");
        }
      }
      catch (const ros::Exception& e)
      {
        ROS_ERROR("[RLControllerManager] ROS exception in changeArmCtrlModeAsync: %s", e.what());
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[RLControllerManager] Exception in changeArmCtrlModeAsync: %s", e.what());
      }
    }).detach();
  }

  RLControllerBase* RLControllerManager::getCurrentController()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (current_controller_name_.empty())
    {
      return nullptr;  // MPC控制器
    }
    auto it = controllers_.find(current_controller_name_);
    if (it != controllers_.end())
    {
      return it->second.get();
    }
    return nullptr;
  }

  RLControllerType RLControllerManager::getCurrentControllerType()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (current_controller_name_.empty())
    {
      return RLControllerType::MPC;
    }
    auto* controller = getCurrentController();
    if (controller)
    {
      return controller->getType();
    }

    return RLControllerType::MPC;
  }

  std::string RLControllerManager::getCurrentControllerName()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_controller_name_;
  }


  bool RLControllerManager::hasController(const std::string& name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return controllers_.find(name) != controllers_.end();
  }

  bool RLControllerManager::hasController(RLControllerType type)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (type == RLControllerType::MPC)
    {
      return true;  // MPC控制器始终存在
    }

    for (const auto& pair : controllers_)
    {
      if (pair.second->getType() == type)
      {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> RLControllerManager::getControllerNames()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return controller_names_;
  }

  size_t RLControllerManager::getControllerCount()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return controller_names_.size();
  }

  void RLControllerManager::switchToBaseController()
  {
    switchController("");
  }

  bool RLControllerManager::isBaseController()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_controller_name_.empty();
  }
  bool RLControllerManager::isBaseControllerActive()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_controller_name_.empty() && getCurrentController() == nullptr;
  }

  void RLControllerManager::setMpcStanceState(bool is_stance, const std::string& gait_name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    mpc_is_stance_mode_ = is_stance;
    mpc_current_gait_name_ = gait_name;
  }

  bool RLControllerManager::loadControllersFromConfig(const std::string& config_file, 
                                                      const std::string& version_config_dir,
                                                      ros::NodeHandle& nh,
                                                      ocs2::humanoid::TopicLogger* ros_logger)
  {
    // 检查配置文件是否存在
    if (!boost::filesystem::exists(config_file))
    {
      ROS_WARN("[RLControllerManager] Controller config file not found: %s", config_file.c_str());
      return false;
    }

    try
    {
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto_switch_config_ = AutoControllerSwitchConfig{};
        auto_switch_config_loaded_ = false;
      }

      // 加载YAML配置文件
      YAML::Node config = YAML::LoadFile(config_file);
      
      if (!config["controllers"])
      {
        ROS_ERROR("[RLControllerManager] No 'controllers' key found in config file: %s", config_file.c_str());
        return false;
      }

      const YAML::Node& controllers = config["controllers"];
      if (!controllers.IsSequence())
      {
        ROS_ERROR("[RLControllerManager] 'controllers' must be a sequence in config file: %s", config_file.c_str());
        return false;
      }

      int loaded_count = 0;
      int enabled_count = 0;

      // 遍历控制器配置
      for (const auto& controller_node : controllers)
      {
        if (!controller_node["name"] || !controller_node["type"] || !controller_node["config_file"])
        {
          ROS_WARN("[RLControllerManager] Skipping invalid controller config (missing name, type, or config_file)");
          continue;
        }

        std::string name = controller_node["name"].as<std::string>();
        std::string type_str = controller_node["type"].as<std::string>();
        std::string config_file_rel = controller_node["config_file"].as<std::string>();
        bool enabled = controller_node["enabled"] ? controller_node["enabled"].as<bool>() : true;
        
        // 解析控制器大类（可选字段，用于分类）
        ControllerClass controller_class = ControllerClass::BASE_CONTROLLER;
        if (controller_node["class"])
        {
          std::string class_str = controller_node["class"].as<std::string>();
          if (!stringToControllerClass(class_str, controller_class))
          {
            ROS_WARN("[RLControllerManager] Unknown controller class '%s' for controller '%s', using default BASE_CONTROLLER", 
                     class_str.c_str(), name.c_str());
          }
        }

        // 跳过未启用的控制器
        if (!enabled)
        {
          ROS_INFO("[RLControllerManager] Controller '%s' is disabled, skipping", name.c_str());
          continue;
        }

        // 解析控制器类型
        RLControllerType type;
        if (!stringToRLControllerType(type_str, type))
        {
          ROS_WARN("[RLControllerManager] Unknown controller type '%s' for controller '%s', skipping", 
                   type_str.c_str(), name.c_str());
          continue;
        }

        // 构建配置文件绝对路径
        std::string config_file_abs;
        if (boost::filesystem::path(config_file_rel).is_absolute())
        {
          config_file_abs = config_file_rel;
        }
        else
        {
          config_file_abs = version_config_dir + "/" + config_file_rel;
        }

        // 检查配置文件是否存在
        if (!boost::filesystem::exists(config_file_abs))
        {
          ROS_ERROR("[RLControllerManager] Controller config file not found for '%s': %s", 
                    name.c_str(), config_file_abs.c_str());
          continue;
        }

        loadAutoControllerSwitchConfig(config_file_abs);

        // 根据类型创建控制器
        std::unique_ptr<RLControllerBase> controller;
        if (type == RLControllerType::FALL_STAND_CONTROLLER)
        {
          controller = std::make_unique<FallStandController>(name, config_file_abs, nh, ros_logger);
        }
        else if (type == RLControllerType::AMP_CONTROLLER)
        {
          // AMP 走路控制器
          controller = std::make_unique<AmpWalkController>(name, config_file_abs, nh, ros_logger);
        }
        else if (type == RLControllerType::DEPTH_LOCO_CONTROLLER)
        {
          controller = std::make_unique<DepthWalkController>(name, config_file_abs, nh, ros_logger);
        }
        else if (type == RLControllerType::VMP_CONTROLLER)
        {
          // VMP 控制器
          controller = std::make_unique<VMPController>(name, config_file_abs, nh, ros_logger);
        }
        else if (type == RLControllerType::MORE_CONTROLLER)
        {
          controller = std::make_unique<MoREController>(name, config_file_abs, nh, ros_logger);
        }
        else if (type == RLControllerType::DANCE_CONTROLLER)
        {
          // Dance 控制器
          auto dance_controller = std::make_unique<DanceController>(name, config_file_abs, nh, ros_logger);
          // [根因修复] 注入 manager 持有的共享 dance_trajectory_state publisher,
          // 避免每个 DanceController 自行 advertise 制造启动期 race
          dance_controller->setDanceTrajectoryStatePublisher(dance_trajectory_state_pub_);
          controller = std::move(dance_controller);
        }
        else
        {
          ROS_WARN("[RLControllerManager] Controller type '%s' not yet implemented, skipping", type_str.c_str());
          continue;
        }

        // 初始化控制器
        if (!controller->initialize())
        {
          ROS_ERROR("[RLControllerManager] Failed to initialize controller '%s'", name.c_str());
          continue;
        }

        // 在刷新可切换列表之前先记录控制器大类
        controller_classes_[name] = controller_class;

        // 添加到管理器（会自动设置为暂停状态）
        if (addController(name, std::move(controller)))
        {
          enabled_count++;
          ROS_INFO("[RLControllerManager] Successfully loaded controller '%s' (type: %s)", 
                   name.c_str(), type_str.c_str());
        }
        else
        {
          controller_classes_.erase(name);
          ROS_ERROR("[RLControllerManager] Failed to add controller '%s'", name.c_str());
        }

        loaded_count++;
      }

      ROS_INFO("[RLControllerManager] Loaded %d controllers from config file (%d enabled)", 
               loaded_count, enabled_count);
      return enabled_count > 0;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR("[RLControllerManager] YAML parsing error in config file '%s': %s", 
                config_file.c_str(), e.what());
      return false;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[RLControllerManager] Error loading controllers from config file '%s': %s", 
                config_file.c_str(), e.what());
      return false;
    }
  }

  bool RLControllerManager::initializeRosServices(ros::NodeHandle& nh)
  {
    nh_ptr_ = &nh;
    loadDepthHistoryCheckParams(nh);
    startDepthHistoryMonitor();
    
    switch_controller_srv_ = nh.advertiseService("/humanoid_controller/switch_controller", 
                                                  &RLControllerManager::switchControllerCallback, this);
    get_controller_list_srv_ = nh.advertiseService("/humanoid_controller/get_controller_list", 
                                                     &RLControllerManager::getControllerListCallback, this);
    switch_to_next_controller_srv_ = nh.advertiseService("/humanoid_controller/switch_to_next_controller", 
                                                          &RLControllerManager::switchToNextControllerCallback, this);
    switch_to_previous_controller_srv_ = nh.advertiseService("/humanoid_controller/switch_to_previous_controller", 
                                                              &RLControllerManager::switchToPreviousControllerCallback, this);
    set_rl_switch_mode_srv_ = nh.advertiseService("/humanoid_controller/set_rl_switch_mode",
                                                  &RLControllerManager::setRLSwitchModeCallback, this);
    set_fall_down_state_srv_ = nh.advertiseService("/humanoid_controller/set_fall_down_state",
                                                   &RLControllerManager::setFallDownStateCallback, this);
    switch_to_vmp_controller_srv_ = nh.advertiseService("/humanoid_controller/switch_to_vmp_controller",
                                                        &RLControllerManager::switchToVMPControllerCallback, this);
    switch_to_dance_controller_srv_ = nh.advertiseService("/humanoid_controller/switch_to_dance_controller",
                                                           &RLControllerManager::switchDanceControllerByStringCallback, this);
    get_dance_controller_list_srv_ = nh.advertiseService("/humanoid_controller/get_dance_controller_list",
                                                       &RLControllerManager::getDanceControllerListCallback, this);
    nav_switch_controller_sub_ = nh.subscribe<std_msgs::String>("/humanoid_controller/nav_switch_rl_controller_by_name", 1,
                                                              &RLControllerManager::navSwitchControllerByNameCallback,this);                                            
    auto_switch_cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, &RLControllerManager::cmdVelAutoSwitchCallback, this);
    auto_switch_gait_name_sub_ = nh.subscribe<std_msgs::String>(
        "/humanoid_mpc_gait_name_request", 10,
        &RLControllerManager::gaitNameAutoSwitchCallback, this);
    auto_switch_arm_control_mode_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
        "/humanoid/mpc/arm_control_mode", 10,
        &RLControllerManager::armControlModeAutoSwitchCallback, this);
    auto_switch_waist_sub_ = nh.subscribe<kuavo_msgs::robotWaistControl>(
        "/robot_waist_motion_data", 10, &RLControllerManager::waistAutoSwitchCallback, this);
    auto_switch_waist_enable_sub_ = nh.subscribe<std_msgs::Bool>(
        "/humanoid_controller/enable_waist_control", 10,
        &RLControllerManager::waistEnableAutoSwitchCallback, this);
    auto_switch_robot_action_state_sub_ = nh.subscribe<humanoid_plan_arm_trajectory::RobotActionState>(
        "/robot_action_state", 10,
        &RLControllerManager::robotActionStateAutoSwitchCallback, this);
    nh.param<double>("/rl_gait_receiver/robot_action_active_timeout",
                     robot_action_active_timeout_, robot_action_active_timeout_);
    robot_action_active_timeout_ = std::max(0.1, robot_action_active_timeout_);
    controller_switch_event_pub_ = nh.advertise<kuavo_msgs::ControllerSwitchEvent>("/humanoid_controller/controller_switch_event", 1, true);
    depth_history_status_pub_ = nh.advertise<std_msgs::Int32>("/humanoid_controller/depth_history_status", 1, true);

    // [根因修复] 所有 DanceController 共用一份 dance_trajectory_state publisher
    // 历史问题: 5 个 DanceController 各自 advertise(latch=true) 同一 topic, 启动期
    //   会让订阅者握手偶发触发 "received a connection for a nonexistent topic", rospy
    //   重试 1 次后 give up, 跳舞时无音乐播放. 现统一在此 advertise 一次, 通过 setter
    //   注入到每个 DanceController. 同时把 latch=true 改为 false: 该场景下 latched 反
    //   而会让"上一次 finished"被新 subscriber 当成"刚 started"误命中 Python 侧逻辑.
    dance_trajectory_state_pub_ = nh.advertise<kuavo_msgs::DanceTrajectoryState>(
        "/humanoid_controller/dance_trajectory_state", 10, /*latch=*/false);
        
    ROS_INFO("[RLControllerManager] Topic registered: /humanoid_controller/dance_trajectory_state");

    can_switch_to_depth_walk_controller_ = isDepthHistoryTopicAvailable(/*log=*/false);
    const double depth_history_check_period_sec = std::min(0.1, depth_history_wait_timeout_sec_ * 0.5);
    depth_history_check_timer_ = nh.createTimer(ros::Duration(depth_history_check_period_sec),
                                                &RLControllerManager::updateDepthHistorySwitchFlag,
                                                this);

    ROS_INFO("[RLControllerManager] ROS services initialized");
    return true;
  }

  void RLControllerManager::publishControllerSwitchEvent(const std::string& from_controller,
                                                         const std::string& to_controller)
  {
    if (!controller_switch_event_pub_)
    {
      return;
    }

    kuavo_msgs::ControllerSwitchEvent msg;
    msg.header.stamp = ros::Time::now();
    msg.from_controller = from_controller;
    msg.to_controller = to_controller;
    controller_switch_event_pub_.publish(msg);
  }

  void RLControllerManager::publishDepthHistoryStatus(TopicMonitor::CheckResult result)
  {
    if (!depth_history_status_pub_)
    {
      return;
    }

    std_msgs::Int32 msg;
    msg.data = TopicMonitor::statusCode(result);
    depth_history_status_pub_.publish(msg);
  }

  std::vector<std::string> RLControllerManager::getWalkControllerList()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return walk_controllers_;
  }

  std::vector<std::string> RLControllerManager::getDanceControllerList()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return dance_controllers_;
  }

  bool RLControllerManager::switchToDanceControllerByName(const std::string& name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (name.empty())
    {
      ROS_WARN("[RLControllerManager] switchToDanceControllerByName: empty name");
      return false;
    }
    if (std::find(dance_controllers_.begin(), dance_controllers_.end(), name) == dance_controllers_.end())
    {
      ROS_WARN("[RLControllerManager] '%s' is not a registered dance controller", name.c_str());
      return false;
    }
    return switchController(name);
  }

  bool RLControllerManager::switchToDanceControllerByIndex(size_t index)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (index >= dance_controllers_.size())
    {
      ROS_WARN("[RLControllerManager] switchToDanceControllerByIndex: index %zu out of range (size %zu)", index,
               dance_controllers_.size());
      return false;
    }
    return switchController(dance_controllers_[index]);
  }

  int RLControllerManager::getCurrentDanceControllerIndex() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (current_controller_name_.empty())
    {
      return -1;
    }
    auto it = controllers_.find(current_controller_name_);
    if (it == controllers_.end() || it->second->getType() != RLControllerType::DANCE_CONTROLLER)
    {
      return -1;
    }
    for (size_t i = 0; i < dance_controllers_.size(); ++i)
    {
      if (dance_controllers_[i] == current_controller_name_)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  void RLControllerManager::updateControllerListsByType()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // 清空按类型分组的列表
    controllers_by_type_.clear();
    walk_controllers_.clear();
    dance_controllers_.clear();
    
      // MPC控制器始终在BASE_CONTROLLER列表的索引0
      walk_controllers_.push_back("mpc");
    
    // 遍历所有控制器，按添加顺序分组
    for (const auto& name : controller_names_)
    {
      auto it = controllers_.find(name);
      if (it == controllers_.end()) continue;
      RLControllerType type = it->second->getType();
      
      // 添加到按类型分组的列表（不包括MPC）
      if (type != RLControllerType::MPC)
      {
        controllers_by_type_[type].push_back(name);
      }
      
      auto class_it = controller_classes_.find(name);
      const bool is_base_controller =
          class_it != controller_classes_.end() && class_it->second == ControllerClass::BASE_CONTROLLER;

      // 只有 BASE_CONTROLLER 参与 walking controller 列表
      if (is_base_controller)
      {
        walk_controllers_.push_back(name);
      }
      if (type == RLControllerType::DANCE_CONTROLLER)
      {
        dance_controllers_.push_back(name);
      }

    }
  }

  bool RLControllerManager::handleSwitchControllerByNameRequest(const std::string& target_name,
                                                                bool only_rl_to_rl,
                                                                std::string& message)
  {
    const std::string switch_name = (target_name == "mpc") ? "" : target_name;
    std::string current_name;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      current_name = current_controller_name_;
    }

    if (target_name.empty())
    {
      message = only_rl_to_rl ? "RL->RL switch only request does not allow empty controller name." :
                                "Controller name cannot be empty.";
      ROS_WARN("[RLControllerManager] %s", message.c_str());
      return false;
    }

    if (only_rl_to_rl)
    {
      if (current_name.empty())
      {
        message = "Navigation RL switch request only supports RL->RL switching. Current controller is MPC.";
        ROS_WARN("[RLControllerManager] %s", message.c_str());
        return false;
      }

      if (!hasController(switch_name))
      {
        message = "Target RL controller not found: " + target_name;
        ROS_WARN("[RLControllerManager] %s", message.c_str());
        return false;
      }

      auto* target_controller = getControllerByName(switch_name);
      if (target_controller == nullptr || target_controller->getType() == RLControllerType::MPC)
      {
        message = "Navigation RL switch request only supports switching to an RL controller: " + target_name;
        ROS_WARN("[RLControllerManager] %s", message.c_str());
        return false;
      }
    }

    const bool allow_walking_phase_sync_switch = allowWalkingPhaseSyncSwitchRequest(switch_name);
    if (!isTorsoVelocityStable() && !allow_walking_phase_sync_switch)
    {
      message = "Torso velocity is not stable. Please wait until the torso velocity is stable.";
      logSwitchBlocked(message);
      return false;
    }
    if (allow_walking_phase_sync_switch)
    {
      ROS_INFO("[RLControllerManager] Bypass torso stability check for walking RL->RL switch: %s -> %s",
               current_name.c_str(), target_name.c_str());
    }

    if (!only_rl_to_rl)
    {
      std::vector<std::string> walk_list;
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        walk_list = walk_controllers_;
      }

      bool found = false;
      for (const auto& controller_name : walk_list)
      {
        if (controller_name == target_name)
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        message = "Unsupported controller type: " + target_name + ". Available controllers: ";
        for (size_t i = 0; i < walk_list.size(); ++i)
        {
          message += walk_list[i];
          if (i + 1 < walk_list.size())
          {
            message += ", ";
          }
        }
        logSwitchBlocked(message);
        return false;
      }
    }

    const bool switch_ok = switchController(switch_name);
    if (!switch_ok)
    {
      message = "Failed to switch to controller: " + target_name;
      logSwitchBlocked(message);
      return false;
    }

    message = "Successfully switched to controller: " + target_name;
    ROS_INFO("[RLControllerManager] %s", message.c_str());
    return true;
  }

  bool RLControllerManager::switchControllerCallback(kuavo_msgs::switchController::Request &req,
                                                      kuavo_msgs::switchController::Response &res)
  {
    ROS_INFO("[RLControllerManager] Received controller switch request: %s", req.controller_name.c_str());

    // 手动切换冷却检查：防止快速连续切换导致机身跳动（对标 G12 的 SWITCH_CONTROLLER_COOLDOWN = 3.0s）
    {
      const ros::Time now = ros::Time::now();
      if (!last_manual_switch_time_.isZero() &&
          (now - last_manual_switch_time_).toSec() < manual_switch_min_interval_)
      {
        res.success = false;
        res.message = "Manual switch cooldown active, please wait.";
        ROS_WARN_THROTTLE(1.0, "[RLControllerManager] Manual switch blocked: cooldown (%.1fs remaining)",
                          manual_switch_min_interval_ - (now - last_manual_switch_time_).toSec());
        return true;
      }
    }

    std::vector<std::string> walk_list;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      walk_list = walk_controllers_;
    }

    int new_index = -1;
    for (size_t i = 0; i < walk_list.size(); ++i)
    {
      if (walk_list[i] == req.controller_name)
      {
        new_index = static_cast<int>(i);
        break;
      }
    }

    res.success = handleSwitchControllerByNameRequest(req.controller_name, false, res.message);
    if (res.success)
    {
      res.message += " (index: " + std::to_string(new_index) + ")";
      last_manual_switch_time_ = ros::Time::now();
    }
    return true;
  }

  void RLControllerManager::navSwitchControllerByNameCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (msg == nullptr)
    {
      ROS_WARN("[RLControllerManager] Received null navigation RL switch request");
      return;
    }

    const std::string target_name = msg->data;
    ROS_INFO("[RLControllerManager] Received navigation RL switch request: %s", target_name.c_str());

    std::string message;
    const bool success = handleSwitchControllerByNameRequest(target_name, true, message);
    if (success)
    {
      ROS_INFO("[RLControllerManager] Navigation RL switch request succeeded: %s", target_name.c_str());
    }
    else
    {
      ROS_WARN("[RLControllerManager] Navigation RL switch request failed: %s", message.c_str());
    }
  }

  void RLControllerManager::recordExternalArmControlActivityLocked(const ros::Time& now)
  {
    last_external_arm_control_time_ = now;
  }

  void RLControllerManager::recordExternalWaistControlActivityLocked(const ros::Time& now)
  {
    last_external_waist_control_time_ = now;
  }

  void RLControllerManager::notifyExternalArmControlActivity()
  {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      recordExternalArmControlActivityLocked(ros::Time::now());
    }
    evaluateAutoControllerSwitch("external arm control");
  }

  void RLControllerManager::cmdVelAutoSwitchCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    const double linear_magnitude = std::sqrt(
        msg->linear.x * msg->linear.x +
        msg->linear.y * msg->linear.y +
        msg->linear.z * msg->linear.z);
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      if (isRobotActionActiveForAutoSwitchLocked(ros::Time::now()))
      {
        latest_cmd_vel_ = geometry_msgs::Twist();
        last_cmd_vel_time_ = ros::Time();
        ROS_DEBUG_THROTTLE(1.0,
                           "[RLControllerManager] Ignore autoSwitch cmd_vel while robot action is active");
        return;
      }
      latest_cmd_vel_ = *msg;
      last_cmd_vel_time_ = ros::Time::now();
    }
    if (linear_magnitude > 1e-4 || std::abs(msg->angular.z) > 1e-4)
    {
      ROS_DEBUG_THROTTLE(1.0,
                         "[RLControllerManager] autoSwitch cmd_vel received: linear=(%.3f, %.3f, %.3f) |linear|=%.3f angular_z=%.3f",
                         msg->linear.x, msg->linear.y, msg->linear.z, linear_magnitude, msg->angular.z);
    }
    evaluateAutoControllerSwitch("cmd_vel");
  }

  void RLControllerManager::robotActionStateAutoSwitchCallback(
      const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr& msg)
  {
    if (msg == nullptr)
    {
      return;
    }

    const bool active = msg->state == 1;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (active)
    {
      last_robot_action_active_time_ = ros::Time::now();
      latest_cmd_vel_ = geometry_msgs::Twist();
      last_cmd_vel_time_ = ros::Time();
      latest_gait_name_.clear();
      last_gait_name_time_ = ros::Time();
    }

    if (robot_action_active_for_auto_switch_ == active)
    {
      return;
    }

    robot_action_active_for_auto_switch_ = active;
    if (active)
    {
      ROS_INFO("[RLControllerManager] Robot action active: ignore walking auto-switch intent");
    }
    else
    {
      ROS_INFO("[RLControllerManager] Robot action inactive: walking auto-switch intent accepted");
    }
  }

  bool RLControllerManager::isRobotActionActiveForAutoSwitchLocked(const ros::Time& now) const
  {
    if (!robot_action_active_for_auto_switch_)
    {
      return false;
    }
    if (!last_robot_action_active_time_.isValid())
    {
      return true;
    }
    return (now - last_robot_action_active_time_).toSec() <= robot_action_active_timeout_;
  }

  void RLControllerManager::gaitNameAutoSwitchCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (msg == nullptr)
    {
      return;
    }

    const std::string gait_name = msg->data;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      latest_gait_name_ = gait_name;
      last_gait_name_time_ = ros::Time::now();
    }

    if (gait_name == "walk" || gait_name == "trot")
    {
      ROS_DEBUG_THROTTLE(1.0,
                         "[RLControllerManager] autoSwitch gait name received: %s",
                         gait_name.c_str());
      evaluateAutoControllerSwitch("gait_name");
    }
  }

  void RLControllerManager::armControlModeAutoSwitchCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if (msg == nullptr || msg->data.empty())
    {
      return;
    }

    constexpr int kExternalArmControlMode = 2;
    const int current_mode = static_cast<int>(std::lround(msg->data.front()));
    const int desired_mode = msg->data.size() > 1
                                 ? static_cast<int>(std::lround(msg->data[1]))
                                 : current_mode;
    if (current_mode != kExternalArmControlMode && desired_mode != kExternalArmControlMode)
    {
      return;
    }

    ROS_DEBUG_THROTTLE(1.0,
                       "[RLControllerManager] autoSwitch arm control mode received: current=%d desired=%d",
                       current_mode, desired_mode);

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      recordExternalArmControlActivityLocked(ros::Time::now());
    }
    evaluateAutoControllerSwitch("arm control mode");
  }

  void RLControllerManager::waistAutoSwitchCallback(const kuavo_msgs::robotWaistControl::ConstPtr& msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      recordExternalWaistControlActivityLocked(ros::Time::now());
    }
    evaluateAutoControllerSwitch("waist command");
  }

  void RLControllerManager::waistEnableAutoSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg == nullptr || !msg->data)
    {
      return;
    }
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }
      recordExternalWaistControlActivityLocked(ros::Time::now());
    }
    evaluateAutoControllerSwitch("waist enable");
  }

  void RLControllerManager::processAutoControllerSwitch()
  {
    evaluateAutoControllerSwitch("control loop");
  }

  bool RLControllerManager::hasRecentExternalControlActivityLocked(const ros::Time& now) const
  {
    const double hold_time = auto_switch_config_.external_command_hold_time;
    if (last_external_arm_control_time_.isValid() &&
        (now - last_external_arm_control_time_).toSec() <= hold_time)
    {
      return true;
    }
    if (last_external_waist_control_time_.isValid() &&
        (now - last_external_waist_control_time_).toSec() <= hold_time)
    {
      return true;
    }
    return false;
  }

  bool RLControllerManager::latestCmdVelRequestsWalkingLocked() const
  {
    if (!last_cmd_vel_time_.isValid())
    {
      return false;
    }
    const ros::Time now = ros::Time::now();
    if ((now - last_cmd_vel_time_).toSec() > auto_switch_config_.cmd_vel_command_hold_time)
    {
      return false;
    }

    const double linear_magnitude = std::sqrt(
        latest_cmd_vel_.linear.x * latest_cmd_vel_.linear.x +
        latest_cmd_vel_.linear.y * latest_cmd_vel_.linear.y +
        latest_cmd_vel_.linear.z * latest_cmd_vel_.linear.z);

    return linear_magnitude > auto_switch_config_.cmd_vel_linear_threshold ||
           std::abs(latest_cmd_vel_.angular.z) > auto_switch_config_.cmd_vel_angular_threshold;
  }

  bool RLControllerManager::latestGaitNameRequestsWalkingLocked() const
  {
    if (!last_gait_name_time_.isValid())
    {
      return false;
    }
    const ros::Time now = ros::Time::now();
    if ((now - last_gait_name_time_).toSec() > auto_switch_config_.cmd_vel_command_hold_time)
    {
      return false;
    }
    return latest_gait_name_ == "walk" || latest_gait_name_ == "trot";
  }

  bool RLControllerManager::shouldBufferCommandLocked(AutoSwitchCommandBufferType type, const ros::Time& now) const
  {
    if (!auto_switch_config_.enabled || type == AutoSwitchCommandBufferType::NONE)
    {
      return false;
    }
    if (auto_switch_command_buffer_type_ != type)
    {
      return false;
    }
    if (!auto_switch_command_buffer_until_.isValid())
    {
      return false;
    }
    return now < auto_switch_command_buffer_until_;
  }

  void RLControllerManager::armAutoSwitchCommandBufferLocked(AutoSwitchCommandBufferType type, const ros::Time& now)
  {
    if (type == AutoSwitchCommandBufferType::NONE || auto_switch_config_.switch_command_buffer_time <= 0.0)
    {
      auto_switch_command_buffer_type_ = AutoSwitchCommandBufferType::NONE;
      auto_switch_command_buffer_until_ = ros::Time();
      return;
    }

    auto_switch_command_buffer_type_ = type;
    auto_switch_command_buffer_until_ = now + ros::Duration(auto_switch_config_.switch_command_buffer_time);
  }

  bool RLControllerManager::shouldBufferWalkingCommand() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return shouldBufferCommandLocked(AutoSwitchCommandBufferType::WALKING, ros::Time::now());
  }

  bool RLControllerManager::shouldBufferExternalControlCommand() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return shouldBufferCommandLocked(AutoSwitchCommandBufferType::EXTERNAL_CONTROL, ros::Time::now());
  }

  bool RLControllerManager::isWalkingCommandExecutionAllowed() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!auto_switch_config_.enabled)
    {
      return true;
    }
    return isWalkingCommandExecutionAllowedByAutoSwitch(current_controller_name_,
                                                        auto_switch_config_.manipulation_controller,
                                                        auto_switch_config_.walking_controller);
  }

  bool RLControllerManager::isExternalControlCommandExecutionAllowed() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!auto_switch_config_.enabled)
    {
      return true;
    }
    return isExternalControlCommandExecutionAllowedByAutoSwitch(current_controller_name_,
                                                                auto_switch_config_.manipulation_controller,
                                                                auto_switch_config_.walking_controller);
  }

  void RLControllerManager::evaluateAutoControllerSwitch(const std::string& reason)
  {
    std::string target_name;
    std::string current_name;
    AutoSwitchCommandBufferType buffer_type = AutoSwitchCommandBufferType::NONE;
    std::function<bool()> walking_command_block_callback;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      walking_command_block_callback = walking_command_block_callback_;
    }
    const bool walking_command_blocked =
        walking_command_block_callback && walking_command_block_callback();
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!auto_switch_config_.enabled)
      {
        return;
      }

      if (current_controller_name_.empty())
      {
        return;
      }

      updateSwitchMotionStateLocked();
      const ros::Time now = ros::Time::now();

      if (hasRecentExternalControlActivityLocked(now) || walking_command_blocked)
      {
        target_name = auto_switch_config_.manipulation_controller;
        buffer_type = AutoSwitchCommandBufferType::EXTERNAL_CONTROL;
        if (!isAutoManipulationSwitchSourceAllowed(current_controller_name_,
                                                   auto_switch_config_.manipulation_controller,
                                                   auto_switch_config_.walking_controller))
        {
          ROS_DEBUG_THROTTLE(2.0,
                             "[RLControllerManager] autoControllerSwitch skipped: manipulation target=%s only allowed from %s, current=%s, reason=%s",
                             target_name.c_str(),
                             auto_switch_config_.walking_controller.c_str(),
                             current_controller_name_.c_str(),
                             reason.c_str());
          return;
        }
        if (walking_command_blocked)
        {
          ROS_DEBUG_THROTTLE(2.0,
                             "[RLControllerManager] autoControllerSwitch keeps manipulation controller because walking command is blocked by external arm target");
        }
      }
      else if (latestCmdVelRequestsWalkingLocked() || latestGaitNameRequestsWalkingLocked())
      {
        target_name = auto_switch_config_.walking_controller;
        buffer_type = AutoSwitchCommandBufferType::WALKING;
        if (!isAutoWalkingSwitchSourceAllowed(current_controller_name_,
                                              auto_switch_config_.manipulation_controller,
                                              target_name))
        {
          ROS_DEBUG_THROTTLE(2.0,
                             "[RLControllerManager] autoControllerSwitch skipped: walking target=%s only allowed from %s, current=%s, reason=%s",
                             target_name.c_str(),
                             auto_switch_config_.manipulation_controller.c_str(),
                             current_controller_name_.c_str(),
                             reason.c_str());
          return;
        }
        if (!last_auto_switch_attempt_time_.isZero() &&
            (now - last_auto_switch_attempt_time_).toSec() < auto_switch_config_.min_switch_interval)
        {
          return;
        }
      }
      else
      {
        if (last_cmd_vel_time_.isValid())
        {
          const double cmd_age = (now - last_cmd_vel_time_).toSec();
          const double linear_magnitude = std::sqrt(
              latest_cmd_vel_.linear.x * latest_cmd_vel_.linear.x +
              latest_cmd_vel_.linear.y * latest_cmd_vel_.linear.y +
              latest_cmd_vel_.linear.z * latest_cmd_vel_.linear.z);
          ROS_DEBUG_THROTTLE(2.0,
                             "[RLControllerManager] autoControllerSwitch skipped: no active walking intent, cmd_age=%.3f, linear=(%.3f, %.3f, %.3f), |linear|=%.3f/%.3f, angular_z=%.3f/%.3f, reason=%s",
                             cmd_age,
                             latest_cmd_vel_.linear.x,
                             latest_cmd_vel_.linear.y,
                             latest_cmd_vel_.linear.z,
                             linear_magnitude,
                             auto_switch_config_.cmd_vel_linear_threshold,
                             latest_cmd_vel_.angular.z,
                             auto_switch_config_.cmd_vel_angular_threshold,
                             reason.c_str());
        }
        return;
      }

      const bool target_is_walking_controller = target_name == auto_switch_config_.walking_controller;
      const bool requires_stance =
          !target_is_walking_controller || auto_switch_config_.walking_switch_require_stance;
      if (requires_stance && switch_motion_state_ != SwitchMotionState::STANCE)
      {
        ROS_DEBUG_THROTTLE(2.0,
                           "[RLControllerManager] autoControllerSwitch skipped: motion_state=%s, target=%s, reason=%s",
                           switchMotionStateName(switch_motion_state_),
                           target_name.c_str(),
                           reason.c_str());
        return;
      }
      if (!requires_stance && switch_motion_state_ != SwitchMotionState::STANCE)
      {
        ROS_DEBUG_THROTTLE(2.0,
                           "[RLControllerManager] autoControllerSwitch allowing walking target without STANCE: motion_state=%s, target=%s, reason=%s",
                           switchMotionStateName(switch_motion_state_),
                           target_name.c_str(),
                           reason.c_str());
      }

      if (target_name.empty() || target_name == current_controller_name_)
      {
        ROS_DEBUG_THROTTLE(2.0,
                           "[RLControllerManager] autoControllerSwitch skipped: target=%s current=%s, reason=%s",
                           target_name.c_str(), current_controller_name_.c_str(), reason.c_str());
        return;
      }

      auto target_it = controllers_.find(target_name);
      if (target_it == controllers_.end() || target_it->second == nullptr)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[RLControllerManager] autoControllerSwitch target controller not found: %s",
                          target_name.c_str());
        return;
      }

      if (target_it->second->getType() == RLControllerType::MPC)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[RLControllerManager] autoControllerSwitch target is not RL: %s",
                          target_name.c_str());
        return;
      }

      current_name = current_controller_name_;
      last_auto_switch_attempt_time_ = now;
    }

    std::string message;
    const bool success = handleSwitchControllerByNameRequest(target_name, true, message);
    if (success)
    {
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        armAutoSwitchCommandBufferLocked(buffer_type, ros::Time::now());
      }
      ROS_INFO("[RLControllerManager] autoControllerSwitch succeeded: %s -> %s, reason: %s",
               current_name.c_str(), target_name.c_str(), reason.c_str());
    }
    else
    {
      ROS_WARN("[RLControllerManager] autoControllerSwitch failed: %s -> %s, reason: %s, message: %s",
               current_name.c_str(), target_name.c_str(), reason.c_str(), message.c_str());
    }
  }

  bool RLControllerManager::getControllerListCallback(kuavo_msgs::getControllerList::Request &req, 
                                                       kuavo_msgs::getControllerList::Response &res)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    res.controller_names.clear();
    res.controller_names = walk_controllers_;
    res.count = static_cast<int>(walk_controllers_.size());
    
    // 计算当前索引
    int current_index = -1;
    if (current_controller_name_.empty())
    {
      current_index = 0;  // MPC控制器在索引0
    }
    else
    {
      for (size_t i = 0; i < walk_controllers_.size(); ++i)
      {
        if (walk_controllers_[i] == current_controller_name_)
        {
          current_index = static_cast<int>(i);
          break;
        }
      }
    }
    
    res.current_index = current_index;
    res.current_controller = current_controller_name_.empty() ? "mpc" : current_controller_name_;
    res.success = true;
    res.message = "Successfully retrieved controller list, total " + std::to_string(res.count) +
                  " controllers, current: " + res.current_controller + " (index: " + std::to_string(res.current_index) + ")";

    ROS_DEBUG("[RLControllerManager] current controller: %s (index: %d, total %d)",
             res.current_controller.c_str(), res.current_index, res.count);

    return true;
  }

  bool RLControllerManager::isTorsoVelocityStable()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // 使用状态估计器提供的稳定性状态
    if (torso_stability_callback_)
    {
      return torso_stability_callback_();
    }
    
    // 如果没有注册稳定性回调，返回true（允许切换）
    ROS_INFO_THROTTLE(1.0, "[RLControllerManager] No torso stability callback registered, allowing switch");
    return true;
  }

  //waao：允许控制器切换标志
  bool RLControllerManager::allowWalkingPhaseSyncSwitchRequest(const std::string& target_name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (current_controller_name_.empty())
    {
      return false;
    }

    auto current_it = controllers_.find(current_controller_name_);
    auto target_it = controllers_.find(target_name);
    if (current_it == controllers_.end() || target_it == controllers_.end())
    {
      return false;
    }

    auto* current_controller = current_it->second.get();
    auto* target_controller = target_it->second.get();
    if (current_controller == nullptr || target_controller == nullptr)
    {
      return false;
    }

    auto current_class_it = controller_classes_.find(current_controller_name_);
    auto target_class_it = controller_classes_.find(target_name);
    if (current_class_it == controller_classes_.end() || target_class_it == controller_classes_.end())
    {
      return false;
    }

    return current_class_it->second == ControllerClass::BASE_CONTROLLER &&
           target_class_it->second == ControllerClass::BASE_CONTROLLER &&
           current_controller->supportsWalkingPhaseSyncSwitch() &&
           target_controller->supportsWalkingPhaseSyncSwitch();
  }

  void RLControllerManager::processPendingWalkingSwitchRequest()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!has_pending_walking_switch_request_)
    {
      return;
    }

    if (pending_walking_switch_target_name_.empty())
    {
      clearPendingWalkingSwitchRequest("empty target");
      return;
    }

    if (current_controller_name_.empty())
    {
      clearPendingWalkingSwitchRequest("current controller is MPC");
      return;
    }

    if (current_controller_name_ != pending_walking_switch_source_name_)
    {
      clearPendingWalkingSwitchRequest("current walking controller changed");
      return;
    }

    auto current_it = controllers_.find(current_controller_name_);
    auto target_it = controllers_.find(pending_walking_switch_target_name_);
    if (current_it == controllers_.end() || target_it == controllers_.end())
    {
      clearPendingWalkingSwitchRequest("controller no longer exists");
      return;
    }

    auto* current_controller = current_it->second.get();
    auto* target_controller = target_it->second.get();
    if (current_controller == nullptr || target_controller == nullptr)
    {
      clearPendingWalkingSwitchRequest("controller pointer is null");
      return;
    }

    if (!allowWalkingPhaseSyncSwitchRequest(pending_walking_switch_target_name_))
    {
      clearPendingWalkingSwitchRequest("request is no longer a walking RL->RL switch");
      return;
    }

    if (current_controller->isAllowToExit())
    {
      clearPendingWalkingSwitchRequest("source controller entered stance before guard passed");
      return;
    }

    std::string guard_message;
    if (!checkWalkingPhaseSyncSwitchGuard(pending_walking_switch_target_name_, guard_message))
    {
      pending_walking_switch_reason_ = guard_message;
      return;
    }

    ROS_INFO("[RLControllerManager] Pending walking RL->RL switch guard passed, auto switching: %s -> %s",
             current_controller_name_.c_str(),
             pending_walking_switch_target_name_.c_str());

    const std::string target_name = pending_walking_switch_target_name_;
    clearPendingWalkingSwitchRequest("auto switch start");
    switchController(target_name);
  }

  bool RLControllerManager::hasPendingWalkingSwitchRequest() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return has_pending_walking_switch_request_;
  }

  std::string RLControllerManager::getPendingWalkingSwitchTargetName() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return pending_walking_switch_target_name_;
  }

  bool RLControllerManager::tryPendingMpcSwitch()
  {
    if (!pending_mpc_switch_)
      return false;

    ArmController* arm_ctrl = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (current_controller_name_.empty())
      {
        pending_mpc_switch_ = false;  // 已经切到 MPC 了，清标记
        return false;
      }
      auto it = controllers_.find(current_controller_name_);
      if (it != controllers_.end() && it->second)
        arm_ctrl = it->second->getArmController();
    }

    if (arm_ctrl && arm_ctrl->getMode() != 1)
      return false;  // 还在归位中

    ROS_INFO("[RLControllerManager] Arm returned to AUTO_SWING, triggering deferred MPC switch");
    pending_mpc_switch_ = false;
    switchController("");
    return true;
  }

  bool RLControllerManager::switchToNextControllerCallback(kuavo_msgs::switchToNextController::Request &req,
                                                           kuavo_msgs::switchToNextController::Response &res)
  {
    // 手动切换冷却检查：防止快速连续切换导致机身跳动（对标 G12 的 SWITCH_CONTROLLER_COOLDOWN = 3.0s）
    {
      const ros::Time now = ros::Time::now();
      if (!last_manual_switch_time_.isZero() &&
          (now - last_manual_switch_time_).toSec() < manual_switch_min_interval_)
      {
        res.success = false;
        res.message = "Manual switch cooldown active, please wait.";
        res.current_controller = "";
        res.next_controller = "";
        res.current_index = -1;
        res.next_index = -1;
        ROS_WARN_THROTTLE(1.0, "[RLControllerManager] Manual switch blocked: cooldown (%.1fs remaining)",
                          manual_switch_min_interval_ - (now - last_manual_switch_time_).toSec());
        return true;
      }
    }

    // 获取当前状态
    std::vector<std::string> walk_list;
    std::string current_name;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      walk_list = walk_controllers_;
      current_name = current_controller_name_;
    }
    
    if (walk_list.empty())
    {
      res.success = false;
      res.message = "No controllers available";
      res.current_controller = "";
      res.next_controller = "";
      res.current_index = -1;
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] No controllers available for switching");
      return true;
    }
    
    // 保存当前控制器信息
    int current_index = -1;
    if (current_name.empty())
    {
      current_index = 0;  // MPC控制器在索引0
    }
    else
    {
      for (size_t i = 0; i < walk_list.size(); ++i)
      {
        if (walk_list[i] == current_name)
        {
          current_index = static_cast<int>(i);
          break;
        }
      }
    }
    
    res.current_controller = current_name.empty() ? "mpc" : current_name;
    res.current_index = current_index;

    // 沿环找下一个"此刻可切换"的控制器（跳过不可用的，如深度话题未就绪的 depth_loco_controller）
    std::vector<std::string> skipped_reasons;
    int target_index = findNextSwitchableIndex(current_index, +1, &skipped_reasons);
    for (const auto& skipped : skipped_reasons)
    {
      if (skipped.find("depth_loco_controller:") == 0)
      {
        logSwitchBlocked("[DepthLocoSwitch] " + skipped +
                         " Refuse to switch to depth_loco_controller.");
      }
    }
    if (target_index < 0)
    {
      res.success = false;
      res.message = "No switchable controller available in the cycle.";
      for (const auto& skipped : skipped_reasons)
        res.message += " " + skipped;
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }

    // 索引 0 固定为 MPC；直接用 switchController("")（而非 switchToBaseController()），以便检查切回 MPC 是否成功
    const std::string target_name = (target_index == 0) ? std::string() : walk_list[target_index];

    // 检查躯干速度是否稳定（在切换前检查）
    const bool allow_walking_phase_sync_switch = !target_name.empty() && allowWalkingPhaseSyncSwitchRequest(target_name);

    // 一次性硬前置：当前控制器若不允许退出（如 AMP 还在行走）或是倒地起身控制器，直接拒绝，
    // 避免下面"沿环找可切候选"时绕一圈空试。
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto it = controllers_.find(current_controller_name_);
      if (!current_controller_name_.empty() && it != controllers_.end() && it->second){
        const bool current_is_fall_stand = it->second->getType() == RLControllerType::FALL_STAND_CONTROLLER;
        if (current_is_fall_stand || (!it->second->isAllowToExit() && !allow_walking_phase_sync_switch)){
          res.success = false;
          res.message = "Current controller is not allowed to exit.";
          res.next_controller = "";
          res.next_index = -1;
          logSwitchBlocked(res.message);
          return true;
        }
      }
    }

    if (!isTorsoVelocityStable() && !allow_walking_phase_sync_switch)
    {
      res.success = false;
      res.message = "Torso velocity is not stable. Please wait until the torso velocity is stable.";
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }
    
    if (allow_walking_phase_sync_switch)
    {
      ROS_INFO("[RLControllerManager] Bypass torso stability check for walking RL->RL switch: %s -> %s",
              res.current_controller.c_str(),
              target_name.c_str());
    }
    
    if (!switchController(target_name))
    {
      res.success = false;
      res.message = "Failed to switch to controller: " + (target_name.empty() ? std::string("mpc") : target_name);
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }
    
    // 设置响应信息
    res.next_controller = target_name.empty() ? "mpc" : target_name;
    res.next_index = target_index;
    res.success = true;
    res.message = "Successfully switched from " + res.current_controller + " (index: " + std::to_string(res.current_index) + 
                  ") to " + res.next_controller + " (index: " + std::to_string(res.next_index) + ")";
    
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());

    last_manual_switch_time_ = ros::Time::now();

    return true;
  }

  bool RLControllerManager::switchToPreviousControllerCallback(kuavo_msgs::switchToNextController::Request &req,
                                                               kuavo_msgs::switchToNextController::Response &res)
  {
    // 手动切换冷却检查：防止快速连续切换导致机身跳动（对标 G12 的 SWITCH_CONTROLLER_COOLDOWN = 3.0s）
    {
      const ros::Time now = ros::Time::now();
      if (!last_manual_switch_time_.isZero() &&
          (now - last_manual_switch_time_).toSec() < manual_switch_min_interval_)
      {
        res.success = false;
        res.message = "Manual switch cooldown active, please wait.";
        res.current_controller = "";
        res.next_controller = "";
        res.current_index = -1;
        res.next_index = -1;
        ROS_WARN_THROTTLE(1.0, "[RLControllerManager] Manual switch blocked: cooldown (%.1fs remaining)",
                          manual_switch_min_interval_ - (now - last_manual_switch_time_).toSec());
        return true;
      }
    }

    // 获取当前状态
    std::vector<std::string> walk_list;
    std::string current_name;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      walk_list = walk_controllers_;
      current_name = current_controller_name_;
    }
    
    if (walk_list.empty())
    {
      res.success = false;
      res.message = "No controllers available";
      res.current_controller = "";
      res.next_controller = "";
      res.current_index = -1;
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] No controllers available for switching");
      return true;
    }
    
    // 保存当前控制器信息
    int current_index = -1;
    if (current_name.empty())
    {
      current_index = 0;  // MPC控制器在索引0
    }
    else
    {
      for (size_t i = 0; i < walk_list.size(); ++i)
      {
        if (walk_list[i] == current_name)
        {
          current_index = static_cast<int>(i);
          break;
        }
      }
    }
    
    res.current_controller = current_name.empty() ? "mpc" : current_name;
    res.current_index = current_index;

    // 沿环（反方向）找上一个"此刻可切换"的控制器（跳过不可用的）。
    std::vector<std::string> skipped_reasons;
    int target_index = findNextSwitchableIndex(current_index, -1, &skipped_reasons);
    for (const auto& skipped : skipped_reasons)
    {
      if (skipped.find("depth_loco_controller:") == 0)
      {
        logSwitchBlocked("[DepthLocoSwitch] " + skipped +
                         " Refuse to switch to depth_loco_controller.");
      }
    }
    if (target_index < 0)
    {
      res.success = false;
      res.message = "No switchable controller available in the cycle.";
      for (const auto& skipped : skipped_reasons)
        res.message += " " + skipped;
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }

    // 索引 0 固定为 MPC；直接用 switchController("")（而非 switchToBaseController()），以便检查切回 MPC 是否成功
    const std::string target_name = (target_index == 0) ? std::string() : walk_list[target_index];
    const bool allow_walking_phase_sync_switch =
        !target_name.empty() && allowWalkingPhaseSyncSwitchRequest(target_name);

    // Walking RL->RL switches are handled by phase synchronization in switchController().
    // Other switches still require the current controller to be ready to exit.
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto it = controllers_.find(current_controller_name_);
      if (!current_controller_name_.empty() && it != controllers_.end() && it->second)
      {
        const bool current_is_fall_stand =
            it->second->getType() == RLControllerType::FALL_STAND_CONTROLLER;
        if (current_is_fall_stand ||
            (!it->second->isAllowToExit() && !allow_walking_phase_sync_switch))
        {
          res.success = false;
          res.message = "Current controller is not allowed to exit.";
          res.next_controller = "";
          res.next_index = -1;
          logSwitchBlocked(res.message);
          return true;
        }
      }
    }

    if (!isTorsoVelocityStable() && !allow_walking_phase_sync_switch)
    {
      res.success = false;
      res.message = "Torso velocity is not stable. Please wait until the torso velocity is stable.";
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }

    if (allow_walking_phase_sync_switch)
    {
      ROS_INFO("[RLControllerManager] Bypass torso stability check for walking RL->RL switch: %s -> %s",
               res.current_controller.c_str(), target_name.c_str());
    }

    if (!switchController(target_name))
    {
      res.success = false;
      res.message = "Failed to switch to controller: " + (target_name.empty() ? std::string("mpc") : target_name);
      res.next_controller = "";
      res.next_index = -1;
      logSwitchBlocked(res.message);
      return true;
    }
    
    // 设置响应信息
    res.next_controller = target_name.empty() ? "mpc" : target_name;
    res.next_index = target_index;
    res.success = true;
    res.message = "Successfully switched from " + res.current_controller + " (index: " + std::to_string(res.current_index) + 
                  ") to " + res.next_controller + " (index: " + std::to_string(res.next_index) + ")";
    
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());

    last_manual_switch_time_ = ros::Time::now();

    return true;
  }

  bool RLControllerManager::setRLSwitchModeCallback(std_srvs::SetBool::Request &req,
                                                    std_srvs::SetBool::Response &res)
  {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      direct_switch_to_rl_ = req.data;
    }

    res.success = true;
    res.message = std::string("Set RL switch mode to ") +
                  (req.data ? "direct" : "interpolated (via MPC)");

    ROS_INFO("[RLControllerManager] %s", res.message.c_str());
    return true;
  }

  bool RLControllerManager::setFallDownStateCallback(std_srvs::SetBool::Request &req,
                                                     std_srvs::SetBool::Response &res)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // 调用回调函数设置倒地状态（如果已注册）
    if (fall_down_state_callback_)
    {
      int state = req.data ? 1 : 0;  // 1=FALL_DOWN, 0=STANDING
      fall_down_state_callback_(state);
    }
    
    // 如果设置为倒地状态，检查是否存在倒地起身控制器并自动切换
    if (req.data)
    {
      res.message = "fall_down_state_ set to FALL_DOWN (1)";
      // 检查控制器列表是否存在倒地起身控制器，自动切换过去
      if (hasController(RLControllerType::FALL_STAND_CONTROLLER))
      {
        ROS_INFO("[RLControllerManager] Fall down detected, switching to fall stand controller");
        bool switch_ok = switchController(RLControllerType::FALL_STAND_CONTROLLER);
        if (!switch_ok)
        {
          ROS_ERROR("[RLControllerManager] Failed to switch to fall stand controller");
          res.success = false;
          res.message += " (but failed to switch to fall stand controller)";
          return true;
        }
      }
      else
      {
        ROS_WARN("[RLControllerManager] No fall stand controller available");
        res.message += " (but no fall stand controller found)";
      }
    }
    else
    {
      res.message = "fall_down_state_ set to STANDING (0)";
    }
    
    res.success = true;
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());
    return true;
  }

  bool RLControllerManager::switchToVMPControllerCallback(std_srvs::Trigger::Request &req,
                                                          std_srvs::Trigger::Response &res)
  {
    ROS_INFO("[RLControllerManager] Received switch to VMP controller request");

    // 查找VMP控制器
    std::string vmp_controller_name;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      for (const auto& pair : controllers_)
      {
        if (pair.second->getType() == RLControllerType::VMP_CONTROLLER)
        {
          vmp_controller_name = pair.first;
          break;
        }
      }
    }

    if (vmp_controller_name.empty())
    {
      res.success = false;
      res.message = "VMP controller not found. Please check if VMP controller is enabled in rl_controllers.yaml";
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }

    // 执行控制器切换
    bool switch_ok = switchController(vmp_controller_name);
    if (!switch_ok)
    {
      res.success = false;
      res.message = "Failed to switch to VMP controller: " + vmp_controller_name;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }

    res.success = true;
    res.message = "Successfully switched to VMP controller: " + vmp_controller_name; 
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());

    return true;
  }
  
  
  bool RLControllerManager::switchDanceControllerByStringCallback(kuavo_msgs::SetString::Request &req,
                                                                 kuavo_msgs::SetString::Response &res)
  {
    const std::string &d = req.data;
    if (d.empty())
    {
      if (!switchToDanceControllerByIndex(0))
      {
        res.success = false;
        res.message = "No dance controller to switch to (list empty or switch failed)";
        return true;
      }
      res.success = true;
      res.message = "Switched to first dance in list (empty request = index 0)";
      return true;
    }
    if (d[0] == '#')
    {
      const std::string idx_str = d.substr(1);
      if (idx_str.empty())
      {
        res.success = false;
        res.message = "Invalid index, use e.g. #0 or #1";
        return true;
      }
      for (char c : idx_str)
      {
        if (!std::isdigit(static_cast<unsigned char>(c)))
        {
          res.success = false;
          res.message = "Invalid #index, digits only after #";
          return true;
        }
      }
      const size_t idx = static_cast<size_t>(std::stoul(idx_str));
      if (!switchToDanceControllerByIndex(idx))
      {
        res.success = false;
        res.message = "Failed to switch to dance at index " + std::to_string(idx);
        return true;
      }
      res.success = true;
      res.message = "Switched to dance index " + std::to_string(idx);
      return true;
    }
    if (!switchToDanceControllerByName(d))
    {
      res.success = false;
      res.message = "Failed to switch to dance name: " + d;
      return true;
    }
    res.success = true;
    res.message = "Switched to dance: " + d;
    return true;
  }

  bool RLControllerManager::getDanceControllerListCallback(kuavo_msgs::GetStringList::Request &req,
                                                           kuavo_msgs::GetStringList::Response &res)
  {
    (void)req;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    res.data = dance_controllers_;
    res.success = true;
    res.message = "Dance count: " + std::to_string(dance_controllers_.size());
    return true;
  }


} // namespace humanoid_controller
