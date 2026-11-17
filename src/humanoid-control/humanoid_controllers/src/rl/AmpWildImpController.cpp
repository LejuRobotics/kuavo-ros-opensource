// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/AmpWildImpController.h"
#include <kuavo_msgs/ExecuteArmAction.h>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <thread>

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace ocs2::humanoid;

  namespace
  {
    /// 自动进阻抗的重试窗口：激活瞬间手臂可能在归位/插值，ArmController 会拒绝模式切换
    constexpr double kArmEnterRetryWindowSec = 2.0;
  }

  AmpWildImpController::AmpWildImpController(const std::string& name,
                                             const std::string& config_file,
                                             ros::NodeHandle& nh,
                                             TopicLogger* ros_logger)
    : AmpFamilyMemberController(name, RLControllerType::AMP_WILD_IMP_CONTROLLER, config_file, nh, ros_logger)
  {
    // 除手臂阻抗部分（本类 override / 新增）外，行走与观测全部复用 AmpFamilyMemberController。
  }

  AmpWildImpController::~AmpWildImpController()
  {
    arm_mode_service_.shutdown();
    arm_mode_get_service_.shutdown();
    stop();
    if (arm_controller_)
    {
      arm_controller_->setModeChangeCallback({});
    }
    {
      std::lock_guard<std::mutex> lock(motor_param_mutex_);
      motor_param_stop_ = true;
      pending_motor_param_.clear();
    }
    motor_param_cv_.notify_one();
    if (motor_param_worker_.joinable())
    {
      motor_param_worker_.join();
    }
  }

  bool AmpWildImpController::initialize()
  {
    if (!AmpFamilyMemberController::initialize())
    {
      return false;
    }
    // 速度指令屏蔽模式：ignoreTwistCmd=true 已在 AmpFamilyMemberController::loadConfig() 中读入，
    // 并由基类 initialize() 应用到 gait_receiver_（不接收任何 twist 与行走步态请求），
    // 这里无需额外处理。

    // 广告手臂阻抗模式切换服务（柔顺交互 Phase 1/2）
    // 服务名：/humanoid_controllers/amp_wild_imp_controller/change_arm_ctrl_mode（与控制器命名空间保持一致）
    arm_mode_service_ = nh_.advertiseService(
        "/humanoid_controllers/" + name_ + "/change_arm_ctrl_mode",
        &AmpWildImpController::changeArmModeCallback, this);
    arm_mode_get_service_ = nh_.advertiseService(
        "/humanoid_controllers/" + name_ + "/get_arm_ctrl_mode",
        &AmpWildImpController::getArmModeCallback, this);

    // RUIWO 驱动级增益切换（仅实机生效）：读配置文件 use_amp_ruiwo_kpkd
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(config_file_, pt);
    loadData::loadPtreeValue(pt, use_amp_ruiwo_kpkd_, "use_amp_ruiwo_kpkd", false);

    if (isRealRobot() && use_amp_ruiwo_kpkd_)
    {
      srv_change_motor_param_ =
          nh_.serviceClient<kuavo_msgs::ExecuteArmAction>("/hardware/change_ruiwo_motor_param");
      motor_param_worker_ = std::thread(&AmpWildImpController::motorParamWorker, this);
      ROS_INFO("[%s] RUIWO motor param switching enabled (service: /hardware/change_ruiwo_motor_param)",
               name_.c_str());
    }
    else
    {
      ROS_INFO("[%s] RUIWO motor param switching disabled (is_real=%s, use_amp_ruiwo_kpkd=%s)",
               name_.c_str(), isRealRobot() ? "true" : "false", use_amp_ruiwo_kpkd_ ? "true" : "false");
    }

    // 注册手臂模式变化回调：切到阻抗模式(3/4)时把 RUIWO 驱动级增益调低（柔顺），
    // 退出阻抗模式（含超速自动回退 Locked）时恢复 normal 增益
    if (arm_controller_)
    {
      arm_controller_->setModeChangeCallback(
          [this](int old_mode, int new_mode) {
            if (new_mode == 3 || new_mode == 4)
            {
              changeRuiwoMotorParamAsync("impedance_kpkd");
            }
            else
            {
              changeRuiwoMotorParamAsync("normal_kpkd");
            }
          });
    }

    // 自动进阻抗：进入本控制器时自动切到的手臂模式（0=不自动；3=关节阻抗；4=笛卡尔阻抗）。
    // 默认：配置了任一阻抗能力时取 3（关节阻抗），配置了对应段但 disabled 则不自动。
    arm_enter_mode_ = (armImpedanceEnabled() || armCartesianImpedanceEnabled()) ? 3 : 0;
    loadData::loadPtreeValue(pt, arm_enter_mode_, "armImpedance.enterOnActivate", false);
    if (arm_enter_mode_ == 3 && !armImpedanceEnabled())
    {
      ROS_WARN("[%s] armImpedance.enterOnActivate=3 but armImpedance.enabled=false, auto-enter disabled", name_.c_str());
      arm_enter_mode_ = 0;
    }
    else if (arm_enter_mode_ == 4 && !armCartesianImpedanceEnabled())
    {
      ROS_WARN("[%s] armImpedance.enterOnActivate=4 but armCartesianImpedance.enabled=false, auto-enter disabled",
               name_.c_str());
      arm_enter_mode_ = 0;
    }
    if (arm_enter_mode_ > 0)
    {
      ROS_INFO("[%s] Auto-enter arm mode %d on controller activation", name_.c_str(), arm_enter_mode_);
    }

    // 记录阻抗配置状态
    ROS_INFO("[%s] Arm impedance services advertised: %s/change_arm_ctrl_mode (mode 3=joint impedance, 4=cartesian impedance)",
             name_.c_str(), name_.c_str());
    if (arm_controller_)
    {
      ROS_INFO("[%s] Arm controller available: impedance_enabled=%s, cartesian_impedance_enabled=%s",
               name_.c_str(),
               armImpedanceEnabled() ? "true" : "false",
               armCartesianImpedanceEnabled() ? "true" : "false");
    }
    else
    {
      ROS_WARN("[%s] Arm controller NOT available: impedance modes 3/4 will not work (check use_external_arm_controller)", name_.c_str());
    }
    return true;
  }

  void AmpWildImpController::pause()
  {
    AmpFamilyMemberController::pause();
    arm_enter_pending_ = false;
    {
      std::lock_guard<std::mutex> lock(motor_param_mutex_);
      const bool restore_gains = motor_param_active_;
      motor_param_active_ = false;
      if (restore_gains && !motor_param_stop_ && isRealRobot() && use_amp_ruiwo_kpkd_)
      {
        // Atomically invalidate late impedance callbacks and replace queued
        // gains. An in-flight service call finishes before this normal request.
        pending_motor_param_ = "normal_kpkd";
      }
    }
    motor_param_cv_.notify_one();
  }

  void AmpWildImpController::resumeMotorParamSwitching()
  {
    {
      std::lock_guard<std::mutex> lock(motor_param_mutex_);
      motor_param_active_ = isActive();
    }
    if (arm_controller_ && (arm_controller_->getMode() == 3 || arm_controller_->getMode() == 4))
    {
      // Warm activation can retain the arm mode while pause restored normal
      // gains, so do not rely solely on a new mode-change callback.
      changeRuiwoMotorParamAsync("impedance_kpkd");
    }
  }

  void AmpWildImpController::resume()
  {
    AmpFamilyMemberController::resume();
    resumeMotorParamSwitching();
    arm_enter_pending_ = (arm_enter_mode_ > 0);
    arm_enter_deadline_ = ros::Time::now() + ros::Duration(kArmEnterRetryWindowSec);
    tryEnterArmImpedance();
  }

  void AmpWildImpController::resumeWarm()
  {
    AmpFamilyMemberController::resumeWarm();
    resumeMotorParamSwitching();
    arm_enter_pending_ = (arm_enter_mode_ > 0);
    arm_enter_deadline_ = ros::Time::now() + ros::Duration(kArmEnterRetryWindowSec);
    tryEnterArmImpedance();
  }

  bool AmpWildImpController::updateImpl(const ros::Time& time,
                                        const SensorData& sensor_data,
                                        const Eigen::VectorXd& measuredRbdState,
                                        kuavo_msgs::jointCmd& joint_cmd)
  {
    const bool ok = AmpFamilyMemberController::updateImpl(time, sensor_data, measuredRbdState, joint_cmd);
    tryEnterArmImpedance();
    return ok;
  }

  void AmpWildImpController::tryEnterArmImpedance()
  {
    if (!arm_enter_pending_ || arm_enter_mode_ <= 0)
    {
      return;
    }
    if (!isActive())
    {
      return; // 尚未真正激活，等下一周期
    }
    if (arm_controller_ && arm_controller_->getMode() == arm_enter_mode_)
    {
      arm_enter_pending_ = false;
      ROS_INFO("[%s] Auto-entered arm impedance mode %d on activation", name_.c_str(), arm_enter_mode_);
      return;
    }
    if (arm_controller_ && !arm_controller_->isHomingToDefault() &&
        arm_controller_->getRequestedMode() != arm_enter_mode_)
    {
      arm_controller_->requestModeChange(arm_enter_mode_);
    }
    if (ros::Time::now() >= arm_enter_deadline_)
    {
      arm_enter_pending_ = false;
      ROS_WARN("[%s] Auto-enter arm mode %d failed within %.1fs (arm returning to default pose or mode not enabled); "
               "switch manually via /humanoid_controllers/%s/change_arm_ctrl_mode",
               name_.c_str(), arm_enter_mode_, kArmEnterRetryWindowSec, name_.c_str());
    }
  }

  bool AmpWildImpController::changeArmModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                                                   kuavo_msgs::changeArmCtrlMode::Response& res)
  {
    if (!isActive())
    {
      res.result = false;
      res.mode = arm_controller_ ? arm_controller_->getMode() : -1;
      res.message = "AmpWildImp controller is not active";
      ROS_WARN("[%s] Rejected arm mode change: controller not active", name_.c_str());
      return true;
    }

    if (!arm_controller_)
    {
      res.result = false;
      res.mode = -1;
      res.message = "Arm controller not initialized";
      ROS_WARN("[%s] Rejected arm mode change: arm controller not initialized", name_.c_str());
      return true;
    }

    const int target = req.control_mode;
    // 模式 3/4 需要对应的阻抗配置
    if (target == 3 && !armImpedanceEnabled())
    {
      res.result = false;
      res.mode = arm_controller_->getMode();
      res.message = "Joint impedance (mode 3) not configured (armImpedance.enabled=false)";
      ROS_WARN("[%s] Rejected arm mode 3: armImpedance not enabled", name_.c_str());
      return true;
    }
    if (target == 4 && !armCartesianImpedanceEnabled())
    {
      res.result = false;
      res.mode = arm_controller_->getMode();
      res.message = "Cartesian impedance (mode 4) not configured (armCartesianImpedance.enabled=false)";
      ROS_WARN("[%s] Rejected arm mode 4: armCartesianImpedance not enabled", name_.c_str());
      return true;
    }

    const bool success = arm_controller_->requestModeChange(target);
    res.result = success;
    res.mode = arm_controller_->getMode();
    res.message = success ? "Mode change queued for control update" : "Mode change rejected";
    ROS_INFO("[%s] Arm mode change requested: %d -> success=%d, current=%d",
             name_.c_str(), target, success ? 1 : 0, arm_controller_->getMode());
    return true;
  }

  bool AmpWildImpController::getArmModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                                                kuavo_msgs::changeArmCtrlMode::Response& res)
  {
    res.result = true;
    res.mode = arm_controller_ ? arm_controller_->getMode() : -1;
    res.message = arm_controller_ ? "OK" : "Arm controller not initialized";
    return true;
  }

  void AmpWildImpController::changeRuiwoMotorParamAsync(const std::string& param_name)
  {
    if (!isRealRobot() || !use_amp_ruiwo_kpkd_)
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(motor_param_mutex_);
      if (motor_param_stop_ ||
          (param_name == "impedance_kpkd" && (!motor_param_active_ || !isActive())))
      {
        return;
      }
      // Coalesce queued changes and serialize service calls so an older
      // impedance request cannot finish after a newer normal request.
      pending_motor_param_ = param_name;
    }
    motor_param_cv_.notify_one();
  }

  void AmpWildImpController::motorParamWorker()
  {
    for (;;)
    {
      std::string param_name;
      {
        std::unique_lock<std::mutex> lock(motor_param_mutex_);
        motor_param_cv_.wait(lock, [this] {
          return motor_param_stop_ || !pending_motor_param_.empty();
        });
        if (motor_param_stop_)
        {
          return;
        }
        param_name.swap(pending_motor_param_);
      }
      try
      {
        if (!srv_change_motor_param_.waitForExistence(ros::Duration(2.0)))
        {
          ROS_WARN("[%s] Ruiwo motor param service not available", name_.c_str());
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(motor_param_mutex_);
          if (motor_param_stop_)
          {
            return;
          }
          if (!pending_motor_param_.empty())
          {
            param_name.swap(pending_motor_param_);
            pending_motor_param_.clear();
          }
        }
        kuavo_msgs::ExecuteArmAction srv;
        srv.request.action_name = param_name;
        if (srv_change_motor_param_.call(srv) && srv.response.success)
        {
          ROS_INFO("[%s] Ruiwo motor param switched to %s", name_.c_str(), param_name.c_str());
        }
        else
        {
          ROS_WARN("[%s] Ruiwo motor param switch to %s failed", name_.c_str(), param_name.c_str());
        }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[%s] changeRuiwoMotorParamAsync failed: %s", name_.c_str(), e.what());
      }
    }
  }

} // namespace humanoid_controller
