// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerManager.h"
#include "humanoid_controllers/rl/FallStandController.h"
#include "humanoid_controllers/rl/AmpWalkController.h"
#include "humanoid_controllers/rl/DepthWalkController.h"
#include "humanoid_controllers/rl/VMPController.h"
#include "humanoid_controllers/rl/DanceController.h"
#include <algorithm>
#include <ros/master.h>
#include <ros/topic.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <ocs2_core/misc/LoadData.h>
#include "kuavo_msgs/changeArmCtrlMode.h"
#include <thread>
#include <cctype>

namespace humanoid_controller
{
  namespace
  {
    constexpr char kDepthHistoryTopic[] = "/camera/depth/depth_history_array";
  }  // namespace

  RLControllerManager::RLControllerManager() : current_controller_name_(""), 
                                                nh_ptr_(nullptr)
  {
    // 初始化BASE_CONTROLLER列表，MPC控制器在索引0
    walk_controllers_.push_back("mpc");
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

  bool RLControllerManager::switchController(const std::string& name)
  {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    const std::string current_before = current_controller_name_.empty() ? "mpc" : current_controller_name_;
    // mimic 类控制器（倒地起身/舞蹈）：仅在未完成（isAllowToExit==false）时禁止切出
    if (!current_controller_name_.empty())
    {
      auto* current_controller = controllers_[current_controller_name_].get();
      const bool current_is_mimic_controller =
          current_controller->getType() == RLControllerType::FALL_STAND_CONTROLLER ||
          current_controller->getType() == RLControllerType::DANCE_CONTROLLER;
      if (!current_controller->isAllowToExit() && current_is_mimic_controller)
      {
        ROS_WARN("[RLControllerManager] Current mimic controller is not ready to exit, switch blocked!");
        return false;
      }
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
        if (current_controller && !current_controller->isAllowToExit())
        {
          // 躯干已稳定（物理静止）→ 允许 RL→MPC，对应 V1.1 的 STATIONARY 状态
          if (!isTorsoVelocityStable())
          {
            ROS_WARN_THROTTLE(1.0, "[RLControllerManager] RL not in stance and torso not stable, switch to MPC blocked! Stop walking first.");
            return false;
          }
          ROS_INFO("[RLControllerManager] Allowing RL->MPC switch with stable torso (stationary)");
        }

        if (current_controller)
        {
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
          if (nh_ptr_)
          {
            // Restore the shared velocity limit param to MPC defaults when leaving RL.
            current_controller->RLControllerBase::updateVelocityLimitsParam(*nh_ptr_);
          }
          // 仅记录上一个控制器的裸指针，所有权仍由 controllers_ 管理
          last_controller_ptr_ = current_controller;
        }
      }
      current_controller_name_ = "";
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
      ROS_ERROR("[RLControllerManager] Controller '%s' not found", name.c_str());
      return false;
    }

    // 如果切换的是同一个控制器，直接返回成功
    if (current_controller_name_ == name)
    {
      return true;
    }

    // 保护逻辑：MPC->RL 切换时，如果机器人不在 stance 状态，不允许切换
    if (current_controller_name_.empty())
    {
      auto* next_controller = controllers_[name].get();
      bool desired_switch_to_falldown = next_controller->getType() == RLControllerType::FALL_STAND_CONTROLLER;
      
      bool is_current_stance = (mpc_current_gait_name_ == "stance") || mpc_is_stance_mode_;
      if (!desired_switch_to_falldown && !is_current_stance)
      {
        ROS_WARN("[RLControllerManager] MPC not in stance (gait=%s), switch to RL blocked! Stop walking first.", 
                 mpc_current_gait_name_.c_str());
        return false;
      }
    }

    auto it = controllers_.find(name);
    if (it != controllers_.end() && it->second &&
        it->second->getType() == RLControllerType::DEPTH_LOCO_CONTROLLER)
    {
      if (!canSwitchToDepthWalkController())
      {
        ROS_WARN("[RLControllerManager] Refuse to switch to depth_loco_controller because depth history topic check failed.");
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
      new_controller->resume();
    }

    current_controller_name_ = name;
    const std::string to_controller = current_controller_name_;
    ROS_INFO("[RLControllerManager] Switched to controller '%s' (type: %d)", 
             name.c_str(), static_cast<int>(new_controller->getType()));
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
        ROS_WARN("[DepthLocoSwitch] NodeHandle is null while checking topic '%s'.", kDepthHistoryTopic);
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
        ROS_WARN("[DepthLocoSwitch] %s Refuse to switch to depth_loco_controller.", report.reason.c_str());
      return false;
    }

    if (log)
      ROS_INFO("[DepthLocoSwitch] %s", report.reason.c_str());
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

  bool RLControllerManager::canSwitchTo(const std::string& name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // 切到 MPC：除"当前控制器能否退出"（由调用方判）外没有目标侧前置
    if (name.empty())
      return true;

    // 目标控制器必须已加载
    auto it = controllers_.find(name);
    if (it == controllers_.end() || !it->second)
      return false;

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
        return false;
    }

    // depth_loco_controller：要求深度历史话题此刻可用（静默探测，不打日志）
    if (target->getType() == RLControllerType::DEPTH_LOCO_CONTROLLER)
      return canSwitchToDepthWalkController();

    return true;
  }

  int RLControllerManager::findNextSwitchableIndex(int current_index, int dir)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const int n = static_cast<int>(walk_controllers_.size());
    if (n <= 1)
      return -1;
    for (int step = 1; step <= n - 1; ++step)
    {
      const int idx = ((current_index + dir * step) % n + n) % n;
      const std::string probe = (idx == 0) ? std::string() : walk_controllers_[idx];  // 索引 0 固定为 MPC
      if (canSwitchTo(probe))
        return idx;
    }
    return -1;
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

        // 添加到管理器（会自动设置为暂停状态）
        if (addController(name, std::move(controller)))
        {
          enabled_count++;
          ROS_INFO("[RLControllerManager] Successfully loaded controller '%s' (type: %s)", 
                   name.c_str(), type_str.c_str());
        }
        else
        {
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
    controller_switch_event_pub_ = nh.advertise<kuavo_msgs::ControllerSwitchEvent>(
        "/humanoid_controller/controller_switch_event", 1, true);
    depth_history_status_pub_ = nh.advertise<std_msgs::Int32>(
        "/humanoid_controller/depth_history_status", 1, true);

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
      
      // 如果是AMP_CONTROLLER类型，添加到walk_controllers_列表
      if (type == RLControllerType::AMP_CONTROLLER || type == RLControllerType::DEPTH_LOCO_CONTROLLER)
      {
        walk_controllers_.push_back(name);
      }
      if (type == RLControllerType::DANCE_CONTROLLER)
      {
        dance_controllers_.push_back(name);
      }

    }
  }

  bool RLControllerManager::switchControllerCallback(kuavo_msgs::switchController::Request &req, 
                                                      kuavo_msgs::switchController::Response &res)
  {
    ROS_INFO("[RLControllerManager] Received controller switch request: %s", req.controller_name.c_str());
    
    // 检查请求的控制器是否在BASE_CONTROLLER列表中
    std::vector<std::string> walk_list;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      walk_list = walk_controllers_;
    }
    
    bool found = false;
    int new_index = -1;
    for (size_t i = 0; i < walk_list.size(); ++i)
    {
      if (walk_list[i] == req.controller_name)
      {
        found = true;
        new_index = static_cast<int>(i);
        break;
      }
    }
    
    if (!found)
    {
      res.success = false;
      res.message = "Unsupported controller type: " + req.controller_name + ". Available controllers: ";
      for (size_t i = 0; i < walk_list.size(); ++i)
      {
        res.message += walk_list[i];
        if (i < walk_list.size() - 1) res.message += ", ";
      }
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }
    
    // 检查躯干速度是否稳定
    if (!isTorsoVelocityStable())
    {
      res.success = false;
      res.message = "Torso velocity is not stable. Please wait until the torso velocity is stable.";
      ROS_WARN("[RLControllerManager] Controller switch blocked: %s", res.message.c_str());
      return true;
    }
    
    // 执行实际控制器切换
    bool switch_ok = true;
    if (new_index == 0)
    {
      // 切回 MPC 控制器；直接用 switchController("") 以检查返回值
      switch_ok = switchController("");
    }
    else
    {
      // 切换到指定 RL 控制器
      if (!hasController(req.controller_name))
      {
        ROS_WARN("[RLControllerManager] RL controller '%s' not found", req.controller_name.c_str());
        switch_ok = false;
      }
      else
      {
        switch_ok = switchController(req.controller_name);
      }
    }
    
    if (!switch_ok)
    {
      res.success = false;
      res.message = "Failed to switch to controller: " + req.controller_name;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }
    
    res.success = true;
    res.message = "Successfully switched to controller: " + req.controller_name + " (index: " + std::to_string(new_index) + ")";
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());
    
    return true;
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

    // 一次性硬前置：当前控制器若不允许退出（如 AMP 还在行走）或是倒地起身控制器，直接拒绝，
    // 避免下面"沿环找可切候选"时绕一圈空试。
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto it = controllers_.find(current_controller_name_);
      if (!current_controller_name_.empty() && it != controllers_.end() && it->second &&
          (!it->second->isAllowToExit() || it->second->getType() == RLControllerType::FALL_STAND_CONTROLLER))
      {
        res.success = false;
        res.message = "Current controller is not allowed to exit.";
        res.next_controller = "";
        res.next_index = -1;
        ROS_WARN("[RLControllerManager] %s", res.message.c_str());
        return true;
      }
    }

    // 检查躯干速度是否稳定（在切换前检查）
    if (!isTorsoVelocityStable())
    {
      res.success = false;
      res.message = "Torso velocity is not stable. Please wait until the torso velocity is stable.";
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] Controller switch blocked: %s", res.message.c_str());
      return true;
    }

    // 沿环找下一个"此刻可切换"的控制器（跳过不可用的，如深度话题未就绪的 depth_loco_controller）
    int target_index = findNextSwitchableIndex(current_index, +1);
    if (target_index < 0)
    {
      res.success = false;
      res.message = "No switchable controller available in the cycle.";
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }

    // 索引 0 固定为 MPC；直接用 switchController("")（而非 switchToBaseController()），以便检查切回 MPC 是否成功
    const std::string target_name = (target_index == 0) ? std::string() : walk_list[target_index];
    if (!switchController(target_name))
    {
      res.success = false;
      res.message = "Failed to switch to controller: " + (target_name.empty() ? std::string("mpc") : target_name);
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }
    
    // 设置响应信息
    res.next_controller = target_name.empty() ? "mpc" : target_name;
    res.next_index = target_index;
    res.success = true;
    res.message = "Successfully switched from " + res.current_controller + " (index: " + std::to_string(res.current_index) + 
                  ") to " + res.next_controller + " (index: " + std::to_string(res.next_index) + ")";
    
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());
    
    return true;
  }

  bool RLControllerManager::switchToPreviousControllerCallback(kuavo_msgs::switchToNextController::Request &req, 
                                                               kuavo_msgs::switchToNextController::Response &res)
  {
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

    // 一次性硬前置：当前控制器若不允许退出（如 AMP 还在行走）或是倒地起身控制器，直接拒绝。
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto it = controllers_.find(current_controller_name_);
      if (!current_controller_name_.empty() && it != controllers_.end() && it->second &&
          (!it->second->isAllowToExit() || it->second->getType() == RLControllerType::FALL_STAND_CONTROLLER))
      {
        res.success = false;
        res.message = "Current controller is not allowed to exit.";
        res.next_controller = "";
        res.next_index = -1;
        ROS_WARN("[RLControllerManager] %s", res.message.c_str());
        return true;
      }
    }

    // 沿环（反方向）找上一个"此刻可切换"的控制器（跳过不可用的）。
    int target_index = findNextSwitchableIndex(current_index, -1);
    if (target_index < 0)
    {
      res.success = false;
      res.message = "No switchable controller available in the cycle.";
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }

    // 索引 0 固定为 MPC；直接用 switchController("")（而非 switchToBaseController()），以便检查切回 MPC 是否成功
    const std::string target_name = (target_index == 0) ? std::string() : walk_list[target_index];
    if (!switchController(target_name))
    {
      res.success = false;
      res.message = "Failed to switch to controller: " + (target_name.empty() ? std::string("mpc") : target_name);
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[RLControllerManager] %s", res.message.c_str());
      return true;
    }
    
    // 设置响应信息
    res.next_controller = target_name.empty() ? "mpc" : target_name;
    res.next_index = target_index;
    res.success = true;
    res.message = "Successfully switched from " + res.current_controller + " (index: " + std::to_string(res.current_index) + 
                  ") to " + res.next_controller + " (index: " + std::to_string(res.next_index) + ")";
    
    ROS_INFO("[RLControllerManager] %s", res.message.c_str());
    
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
