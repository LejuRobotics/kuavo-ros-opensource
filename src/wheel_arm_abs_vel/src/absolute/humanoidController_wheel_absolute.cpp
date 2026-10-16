#include <wheel_arm_abs_vel/humanoid_controllers/arm_inertia_feedforward.h>
#include "wheel_arm_abs_vel/motion_capture_ik/runtime/E2eTrace.h"
namespace { kuavo_e2e::Trace<64> commandAudit("command"); }
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include "wheel_arm_abs_vel/humanoid_controllers/absolute/humanoidController_wheel_absolute.h"
#include "wheel_arm_abs_vel/humanoid_controllers/MobileBaseLock.h"
#include "wheel_arm_abs_vel/humanoid_wheel_wbc/WbcBackend.h"
#include "wheel_arm_abs_vel/humanoid_wheel_wbc/RneaBackend.h"
#include "kuavo_msgs/setContactForceInterpParams.h"
#include "leju_mobile_base_msgs/SetDispatchMode.h"
#include <kuavo_common/common/common.h>
#include "humanoid_interface/common/TopicLogger.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <leju_mobile_base_msgs/BaseCmdVelStatus.h>

#include "humanoid_wheel_interface/estimators/ContinuousEulerAnglesFromMatrix.h"


namespace wheel_absolute_control
{
  using namespace ocs2;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;

  namespace {

  template <typename T>
  void loadOptionalTaskParam(const std::string& taskFile, const std::string& key, T& value) {
    try {
      loadData::loadCppDataType(taskFile, key, value);
    } catch (const std::exception&) {
    }
  }

  void loadArmTrajInterpConfig(const std::string& taskFile, double controlCycleSec,
                               ArmTrajectoryInterpolator::Config& config, bool& enableInterpolator) {
    bool hasKalmanLimitConfig = true;
    try {
      loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.kalman_v_limit", config.kalmanVLimit);
    } catch (const std::exception&) {
      hasKalmanLimitConfig = false;
    }

    config.kalmanProcessAccCap = 80.0;
    config.kalmanProcessQCap = 400.0;
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_r_q", config.kalmanMeasurementQNoise);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_r_dq", config.kalmanMeasurementDqNoise);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_p0_pos", config.kalmanInitialPosVar);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_p0_vel", config.kalmanInitialVelVar);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.fast_update_r_scale", config.fastUpdateRScale);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.target_v_alpha", config.targetVAlpha);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_process_acc_cap",
                          config.kalmanProcessAccCap);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_process_q_cap",
                          config.kalmanProcessQCap);

    int immediate = config.immediateUpdateOnNewTarget ? 1 : 0;
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.immediate_update_on_new_target", immediate);
    config.immediateUpdateOnNewTarget = (immediate != 0);

    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.timeout_sec", config.timeoutSec);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.reference_update_period",
                          config.referenceUpdatePeriodSec);
    config.controlCycleSec = controlCycleSec;

    try {
      loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.enable", enableInterpolator);
    } catch (const std::exception&) {
      enableInterpolator = false;
    }

    const bool kalmanLimitValid = hasKalmanLimitConfig && std::isfinite(config.kalmanVLimit) && config.kalmanVLimit > 0.0;
    if (!kalmanLimitValid) {
      ROS_ERROR_STREAM("[humanoidControllerWheelWbc] Invalid armTrajInterpKinematicLimit.kalman_v_limit, "
                       "arm trajectory interpolator disabled.");
      enableInterpolator = false;
    }
  }

  }  // namespace

  static bool callSimStartSrv(ros::NodeHandle& nh,
                              const std::atomic<bool>* running)
  {
    const auto keep_running = [running] {
      return ros::ok() && (!running || running->load(std::memory_order_acquire));
    };
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("sim_start");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(100);
    while (keep_running() && !client.exists()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        ROS_ERROR("sim_start Service not available");
        return false;
      }
      ROS_WARN_THROTTLE(1.0, "[callSimStartSrv] Waiting for sim_start service...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!keep_running()) return false;
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (!client.call(srv) || !srv.response.success) {
      ROS_ERROR("sim_start Service call failed");
      return false;
    }
    ROS_INFO("sim_start Service call succeeded with message: %s", srv.response.message.c_str());
    return keep_running();
  }

  bool humanoidControllerWheelWbc::init(ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    std::cout << "humanoidControllerWheelWbc init..." << std::endl;
    controllerNh_ = controller_nh;
    // Startup-only selection: live rosparam changes must never unexpectedly
    // release the base. Omitted/invalid configuration retains the lock.
    controllerNh_.param<bool>("/vr_ik/use_velocity_ik_reference", fixed_velocity_ik_pipeline_, false);
    if (!fixed_velocity_ik_pipeline_) { ROS_ERROR("Absolute controller requires explicit absolute mode"); return false; }
    controllerNh_.param<bool>("/base_locked", base_locked_, fixed_velocity_ik_pipeline_);
    ROS_INFO("[MobileBaseLock] base_locked=%s (startup-only, final command boundary)",
             base_locked_ ? "true" : "false");
    controllerNh_.param<std::string>("/dynamics_backend", dynamics_backend_name_, "wbc");
    if (dynamics_backend_name_ != "wbc" && dynamics_backend_name_ != "rnea") {
      ROS_FATAL("dynamics_backend must be wbc or rnea");
      return false;
    }
    ros_logger_ = new humanoid::TopicLogger(controllerNh_);
    /************** Initialize OCS2 *********************/
    std::string taskFile;
    std::string libFolder;
    std::string urdfFile;
    bool verbose = true;

    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/libFolder", libFolder);
    controllerNh_.getParam("/urdfFile", urdfFile);

    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
    }
    std::cout << "is_real: " << is_real_ << std::endl;

    setupHumanoidWheelInterface(taskFile, libFolder, urdfFile);

    observation_wheel_.state.setZero(manipulatorModelInfo_.stateDim);
    observation_wheel_.input.setZero(manipulatorModelInfo_.inputDim);
    observation_wheel_.time = 0;
    observation_wheel_.mode = 0;
    /****************************************************/
    /************load param from task.info***************/
    loadData::loadCppDataType(taskFile, "model_settings.verbose", verbose);
    loadData::loadCppDataType(taskFile, "model_settings.mpcArmsDof", armNum_);
    lowJointNum_ = manipulatorModelInfo_.armDim - armNum_;
    baseDim_ = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
    controllerNh_.param("/vr_ik/wbc_arm_ddq_feedforward_gain",
                        arm_ddq_feedforward_gain_, 0.25);
    if (dynamics_backend_name_ == "rnea")
      controllerNh_.param("/dynamics/rnea/acceleration_gain", arm_ddq_feedforward_gain_, 0.25);
    if (!std::isfinite(arm_ddq_feedforward_gain_)) {
      ROS_FATAL("Nonfinite dynamics acceleration gain"); return false;
    }
    arm_ddq_feedforward_gain_ =
        std::clamp(arm_ddq_feedforward_gain_, 0.0, 1.0);
    ROS_INFO("[humanoidControllerWheelWbc] arm ddq feedforward gain=%.3f",
             arm_ddq_feedforward_gain_);
    if (controllerNh_.hasParam("/vr_ik/arm_inertia_feedforward_scale") &&
        !controllerNh_.getParam("/vr_ik/arm_inertia_feedforward_scale",arm_inertia_feedforward_scale_)) {
      ROS_FATAL("arm_inertia_feedforward_scale must be numeric"); return false;
    }
    if (!std::isfinite(arm_inertia_feedforward_scale_) ||
        arm_inertia_feedforward_scale_<0. || arm_inertia_feedforward_scale_>1.) {
      ROS_FATAL("arm_inertia_feedforward_scale must be in [0,1]"); return false;
    }
    ROS_INFO("[ArmInertiaFeedforward] scale=%.3f, effective gain=%.3f; q/dq and lower body unchanged",
        arm_inertia_feedforward_scale_,arm_ddq_feedforward_gain_*arm_inertia_feedforward_scale_);

    // 从 task.info 加载 /move_base/base_cmd_vel 专用速度上下界
    {
      base_cmd_vel_limit_enable_ = false;
      loadOptionalTaskParam(taskFile, "baseCmdVelLimit.activate", base_cmd_vel_limit_enable_);
      vector_t lowerBound = vector_t::Zero(3);
      vector_t upperBound = vector_t::Zero(3);
      try
      {
        loadData::loadEigenMatrix(taskFile, "baseCmdVelLimit.lowerBound", lowerBound);
        loadData::loadEigenMatrix(taskFile, "baseCmdVelLimit.upperBound", upperBound);
        base_cmd_vel_min_ << lowerBound[0], lowerBound[1], lowerBound[2];
        base_cmd_vel_max_ << upperBound[0], upperBound[1], upperBound[2];
      }
      catch (const std::exception& e)
      {
        ROS_WARN("[humanoidControllerWheelWbc] baseCmdVelLimit bounds not found in task.info, using default +/-1.2: %s",
                 e.what());
      }
      ROS_INFO("[humanoidControllerWheelWbc] base_cmd_vel limit enable=%s, min=(%.3f, %.3f, %.3f), max=(%.3f, %.3f, %.3f)",
               base_cmd_vel_limit_enable_ ? "true" : "false",
               base_cmd_vel_min_[0], base_cmd_vel_min_[1], base_cmd_vel_min_[2],
               base_cmd_vel_max_[0], base_cmd_vel_max_[1], base_cmd_vel_max_[2]);
      // 限频发布：urobot 按 50Hz 消费底盘命令，500Hz 灌入会使其内部队列堆积（见 底盘命令执行延迟分析记录.md）
      loadOptionalTaskParam(taskFile, "baseCmdVelLimit.publish_rate", base_cmd_vel_publish_rate_);
      base_cmd_vel_publish_rate_ = std::max(1.0, base_cmd_vel_publish_rate_);
      ROS_INFO("[humanoidControllerWheelWbc] base_cmd_vel publish rate=%.1f Hz", base_cmd_vel_publish_rate_);
    }
    optimizedState_mrt_.setZero(manipulatorModelInfo_.stateDim);
    optimizedInput_mrt_.setZero(manipulatorModelInfo_.inputDim);
    loadData::loadCppDataType(taskFile, "mpc.mpcDesiredFrequency", mpcFreq_);
    mpcDt_ = 1 / mpcFreq_;
    /****************************************************/
    /************load param from kuavo.json**************/
    RobotVersion rb_version(6, 0);
    if (controllerNh_.hasParam("/robot_version"))
    {
        int rb_version_int;
        controllerNh_.getParam("/robot_version", rb_version_int);
        rb_version = RobotVersion::create(rb_version_int);
    }
    drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
    robot_config_ = drake_interface_->getRobotConfig();
    kuavo_settings_ = drake_interface_->getKuavoSettings();

    const auto& endEffectorTypes = kuavo_settings_.hardware_settings.end_effector_type;
    const bool hasQibeng = std::any_of(endEffectorTypes.begin(), endEffectorTypes.end(),
        [](const auto& eef_type) { return eef_type == EndEffectorType::qibeng; });
    /************** Initialize WBC **********************/
    if (dynamics_backend_name_ == "wbc") {
    wheel_wbc_ = std::make_shared<mobile_manipulator::AbsoluteContactForceWbc>(*pinocchioInterface_ptr_, manipulatorModelInfo_);
    wheel_wbc_->setArmNums(armNum_);
    bool useVrArmAccelTask = false;
    try
    {
      loadData::loadCppDataType(taskFile, "vrArmAccelTask.useVrArmAccelTask", useVrArmAccelTask);
    }
    catch (const std::exception&)
    {
      useVrArmAccelTask = false;
    }
    wheel_wbc_->setUseVrArmAccelTask(useVrArmAccelTask);
    ROS_INFO_STREAM("[humanoidControllerWheelWbc] useVrArmAccelTask=" << (useVrArmAccelTask ? "true" : "false")
                    << ", arm accel task=" << (useVrArmAccelTask ? "vrArmAccelTask" : "armAccelTask"));
    wheel_wbc_->loadTasksSetting(taskFile, verbose, is_real_);
    dynamics_backend_ = std::make_unique<mobile_manipulator::WbcBackend>(wheel_wbc_, manipulatorModelInfo_.armDim, armNum_);
    }
    bool shadowRnea = false;
    controllerNh_.param("/dynamics/rnea_shadow", shadowRnea, false);
    if (shadowRnea && dynamics_backend_name_ != "wbc") {
      ROS_FATAL("rnea_shadow requires dynamics_backend=wbc"); return false;
    }
    if (dynamics_backend_name_ == "rnea" || shadowRnea) {
      try {
        mobile_manipulator::RneaSettings settings;
        ros::NodeHandle config("/dynamics/rnea");
        config.param<std::string>("mode", settings.mode, "full");
        config.param("velocity_cutoff_hz", settings.velocityCutoffHz, 30.0);
        config.param("sensor_timeout", settings.maxMeasurementAge, 0.05);
        config.param("max_period", settings.maxPeriod, 0.05);
        config.param("dynamic_ramp_seconds", settings.dynamicRampSeconds, 0.2);
        dynamics_sensor_timeout_ = settings.maxMeasurementAge;
        vector_t limits = vector_t::Zero(lowJointNum_ + armNum_/2);
        loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", limits);
        settings.torqueLimit.resize(manipulatorModelInfo_.armDim);
        settings.torqueLimit << limits.head(lowJointNum_), limits.tail(armNum_/2), limits.tail(armNum_/2);
        double ratio = 0.8;
        config.param("torque_limit_ratio", ratio, 0.8);
        if (!std::isfinite(ratio) || ratio <= 0 || ratio > 1)
          throw std::invalid_argument("torque_limit_ratio must be in (0,1]");
        settings.torqueLimit *= ratio;
        const auto loadGroupLimit = [&](const std::string& key, double lower, double arm) {
          config.param(key+"_lower", lower, lower);
          config.param(key+"_arm", arm, arm);
          vector_t result(manipulatorModelInfo_.armDim);
          result.head(lowJointNum_).setConstant(lower);
          result.tail(armNum_).setConstant(arm);
          std::vector<double> overrides;
          if (config.hasParam(key)) {
            if (!config.getParam(key, overrides) || overrides.size()!=static_cast<size_t>(result.size()))
              throw std::invalid_argument(key+" must contain one value per driven joint");
            for (size_t j=0;j<overrides.size();++j) result[j]=overrides[j];
          }
          return result;
        };
        settings.torqueRateLimit=loadGroupLimit("torque_rate",3000.0,1000.0);
        settings.accelerationLimit=loadGroupLimit("acceleration_limit",20.0,100.0);
        settings.jerkLimit=loadGroupLimit("jerk_limit",200.0,500.0);
        auto rnea=std::make_unique<mobile_manipulator::RneaBackend>(*pinocchioInterface_ptr_,manipulatorModelInfo_,settings);
        if(shadowRnea) shadow_rnea_=std::move(rnea);
        else dynamics_backend_=std::move(rnea);
        ROS_INFO_STREAM("RNEA mode=" << settings.mode << ", FROZEN base, full torso/arm coupling, torque limits=" << settings.torqueLimit.transpose());
      } catch (const std::exception& e) {
        ROS_FATAL("RNEA configuration failed: %s",e.what()); return false;
      }
    }
    ROS_INFO("Dynamics backend=%s, RNEA shadow=%s (startup selection only)", dynamics_backend_->name(), shadowRnea?"true":"false");
    /****************************************************/

    if(controllerNh_.hasParam("/robot_version"))
    {
      int raw_version = 0;
      controllerNh_.getParam("/robot_version", raw_version);
      robotVersion_ = RobotVersion::create(raw_version).version_number();
    }
    std::cout << "robotVersion_: " << robotVersion_ << std::endl;
    double visualization_rate = 30.0;
    controllerNh_.param("/vr_ik/visualization_rate_hz", visualization_rate, 30.0);
    if (!std::isfinite(visualization_rate)) visualization_rate = 30.0;
    visualization_period_ = visualization_rate <= 0.0 ? 0.0 : 1.0 / visualization_rate;
    bool j3_profile = false;
    controllerNh_.param("/j3_overshoot_profile/enabled", j3_profile, false);
    if (j3_profile) {
      double kp = 250., kd = 18.;
      controllerNh_.param("/j3_overshoot_profile/kp", kp, 250.);
      controllerNh_.param("/j3_overshoot_profile/j3_kd", kd, 18.);
      if (robotVersion_ != 62 || !std::isfinite(kp) || kp <= 0. || kp > 250. ||
          !std::isfinite(kd) || kd <= 0. || kd > 50.)
        throw std::runtime_error("Invalid S62 J3 gain profile");
      auto apply = [&](auto& p, auto& d) {
        if (p.size() < 14 || p.size() != d.size())
          throw std::runtime_error("S62 J3 profile requires 14 ordered arm gains");
        for (int offset : {0, 7}) {
          p[offset] = kp; p[offset + 2] = kp; d[offset + 2] = kd;
        }
      };
      auto& settings = kuavo_settings_.running_settings;
      apply(settings.ruiwo_kp, settings.ruiwo_kd);
      if (!settings.vr_ruiwo_kp.empty() || !settings.vr_ruiwo_kd.empty())
        apply(settings.vr_ruiwo_kp, settings.vr_ruiwo_kd);
    }

    if(controllerNh_.hasParam("/use_vr_control"))
    {
      controllerNh_.getParam("/use_vr_control", use_vr_control_);
      std::cout << "use_vr_control: " << use_vr_control_ << std::endl;
    }
    if (controllerNh_.hasParam("/arm_move_spd"))
    {
      controllerNh_.getParam("/arm_move_spd", arm_move_spd_);
    }
    // This plugin has no MPC owner or incremental fallback.
    bool external_mpc = false;
    controllerNh_.param("/use_external_mpc", external_mpc, false);
    if (external_mpc) {
      ROS_ERROR("wheel_arm_abs_vel requires use_external_mpc=false");
      return false;
    }
    enable_mpc_ = false;
    if (controllerNh_.hasParam("/play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);
      std::cout << "play_back: " << is_play_back_mode_ << std::endl;
    }

    double controlFrequency = 500.0; // 500Hz
    controllerNh_.getParam("/wbc_frequency", controlFrequency);
    std::cout << "wbc_frequency: " << controlFrequency << std::endl;
    dt_ = 1.0 / controlFrequency;

    /*************底盘插补参数设置**********************/
    int vel_num = 3;
    velLimiter_ = std::make_shared<mobile_manipulator::VelocityLimiter>(vel_num);
    Eigen::VectorXd max_acceleration, max_deceleration;
    max_acceleration.setZero(vel_num);
    max_deceleration.setZero(vel_num);
    max_acceleration << 1.2, 1.2, 1.2;  //x, y, yaw 顺序加速度
    max_deceleration << 1.2, 1.2, 1.2;  // 减速度
    velLimiter_->setAccelerationLimits(max_acceleration, max_deceleration);
    velLimiter_->setAccelerationDt(dt_);
    /****************************************************/

    /*****************载入运动学限制相关********************/
    bool obsLimitEnable = false;
    loadData::loadCppDataType(taskFile, "observationKinematicLimit.activate", obsLimitEnable);
    bool mrtLimitEnable = false;
    loadData::loadCppDataType(taskFile, "optimizedTrajKinematicLimit.activate", mrtLimitEnable);

    obsStateLimitFilterPtr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(manipulatorModelInfo_.stateDim, dt_);
    obsInputLimitFilterPtr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(manipulatorModelInfo_.inputDim, dt_);
    mrtStateLimitFilterPtr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(manipulatorModelInfo_.stateDim, dt_);
    mrtInputLimitFilterPtr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(manipulatorModelInfo_.inputDim, dt_);

    observationMaxVel_.setZero(manipulatorModelInfo_.stateDim);
    observationMaxAcc_.setZero(manipulatorModelInfo_.stateDim);
    observationMaxJerk_.setZero(manipulatorModelInfo_.stateDim);
    optimizedTrajMaxVel_.setZero(manipulatorModelInfo_.stateDim);
    optimizedTrajMaxAcc_.setZero(manipulatorModelInfo_.stateDim);
    optimizedTrajMaxJerk_.setZero(manipulatorModelInfo_.stateDim);

    loadData::loadEigenMatrix(taskFile, "observationKinematicLimit.max_vel", observationMaxVel_);
    loadData::loadEigenMatrix(taskFile, "observationKinematicLimit.max_acc", observationMaxAcc_);
    loadData::loadEigenMatrix(taskFile, "observationKinematicLimit.max_jerk", observationMaxJerk_);
    loadData::loadEigenMatrix(taskFile, "optimizedTrajKinematicLimit.max_vel", optimizedTrajMaxVel_);
    loadData::loadEigenMatrix(taskFile, "optimizedTrajKinematicLimit.max_acc", optimizedTrajMaxAcc_);
    loadData::loadEigenMatrix(taskFile, "optimizedTrajKinematicLimit.max_jerk", optimizedTrajMaxJerk_);
    {
      ArmTrajectoryInterpolator::Config config;
      loadArmTrajInterpConfig(taskFile, dt_, config, enable_arm_traj_interpolator_);
      armTrajectoryInterpolator_.configure(config);
      wbc_arm_raw_q_ = vector_t::Zero(armNum_);
      wbc_arm_raw_v_ = vector_t::Zero(armNum_);
      controllerNh_.param("/vr_ik/consistent_arm_reference_timeout_sec",
                          consistent_arm_reference_timeout_sec_, 0.05);
      consistent_arm_reference_timeout_sec_ =
          std::max(0.01, consistent_arm_reference_timeout_sec_);
      ROS_INFO_STREAM("[humanoidControllerWheelWbc] arm trajectory interpolator enable="
                      << (enable_arm_traj_interpolator_ ? "true" : "false")
                      << " kalman_v_limit=" << config.kalmanVLimit
                      << " r_dq=" << config.kalmanMeasurementDqNoise
                      << " process_acc_cap=" << config.kalmanProcessAccCap
                      << " process_q_cap=" << config.kalmanProcessQCap
                      << ", consistent q/dq timeout="
                      << consistent_arm_reference_timeout_sec_ << " s");
    }

    if(obsLimitEnable)
    {
      std::cout << "[humanoidControllerWheelWbc] 启动 observationLimitFilter! " << std::endl;
      // obs.State 支持三阶限制, obs.Input 支持两阶限制
      obsStateLimitFilterPtr_->setFirstOrderDerivativeLimit(observationMaxVel_);
      obsStateLimitFilterPtr_->setSecondOrderDerivativeLimit(observationMaxAcc_);
      // obsStateLimitFilterPtr_->setThirdOrderDerivativeLimit(observationMaxJerk_);
      obsInputLimitFilterPtr_->setFirstOrderDerivativeLimit(observationMaxAcc_);
      obsInputLimitFilterPtr_->setSecondOrderDerivativeLimit(observationMaxJerk_);
    }
    if(mrtLimitEnable)
    {
      std::cout << "[humanoidControllerWheelWbc] 启动 mrtTrajLimitFilter! " << std::endl;
      // mrtState 支持三阶限制, mrtInput 支持两阶限制
      mrtStateLimitFilterPtr_->setFirstOrderDerivativeLimit(optimizedTrajMaxVel_);
      mrtStateLimitFilterPtr_->setSecondOrderDerivativeLimit(optimizedTrajMaxAcc_);
      // mrtStateLimitFilterPtr_->setThirdOrderDerivativeLimit(optimizedTrajMaxJerk_);
      mrtInputLimitFilterPtr_->setFirstOrderDerivativeLimit(optimizedTrajMaxAcc_);
      mrtInputLimitFilterPtr_->setSecondOrderDerivativeLimit(optimizedTrajMaxJerk_);
    }

    // 关节输出限制
    jointCmdLimiterPtr_ = std::make_shared<mobile_manipulator::jointCmdLimiter>(manipulatorModelInfo_.armDim,
                                                            *pinocchioInterface_ptr_,
                                                            taskFile, manipulatorModelInfo_, dt_, armNum_);
    /****************************************************/

    // 浮动基 7 + 底盘下肢电机 4 + 双臂 7*2 + 头部
    ros::param::set("/armRealDof",  static_cast<int>(armNum_));
    ros::param::set("/legRealDof",  static_cast<int>(lowJointNum_));
    ros::param::set("/headRealDof",  2);
    ros::param::set("/waistRealDof",  0);
    vector_t mujoco_q = vector_t::Zero(7 + 4 + 7*2 + 2);
    // 轮臂（major==6）统一初始化：mujoco_q[2] 置 0（60/61/62/63/200062/300062/400062/400063 等价）
    if (rb_version.major() == 6)
    {
      mujoco_q[2] = 0.0;
    }
    mujoco_q[3] = 1.0;
    // 62/63 夹爪气泵版（短版本 patch==0）的特殊初始臂姿
    if ((rb_version.major() == 6 && rb_version.patch() == 0 && (rb_version.minor() == 2 || rb_version.minor() == 3)) && hasQibeng)
    {
      mujoco_q[11] = 0.5236;
      mujoco_q[14] = -1.57;
      mujoco_q[18] = 0.5236;
      mujoco_q[21] = -1.57;
    }
    else
    {
      mujoco_q[11] = 0.2618;
      mujoco_q[14] = -0.5236;
      mujoco_q[18] = 0.2618;
      mujoco_q[21] = -0.5236;
    }
    // Deterministic simulation regressions may start from an explicit lower
    // body + dual-arm posture instead of the generic nearly-extended default.
    // The override is ignored on hardware and is absent from normal launch
    // files, so production initialization remains unchanged.
    std::vector<double> simulation_initial_lower_arm_joint_state;
    if (!is_real_ && controllerNh_.getParam(
            "/simulation_initial_lower_arm_joint_state",
            simulation_initial_lower_arm_joint_state)) {
      constexpr int kConfiguredJointCount = 4 + 14;
      if (simulation_initial_lower_arm_joint_state.size() ==
          kConfiguredJointCount) {
        for (int index = 0; index < kConfiguredJointCount; ++index) {
          mujoco_q[7 + index] =
              simulation_initial_lower_arm_joint_state.at(index);
        }
        ROS_INFO(
            "[humanoidControllerWheelWbc] applied deterministic simulation "
            "lower-body/arm initial state");
      } else {
        ROS_WARN(
            "[humanoidControllerWheelWbc] ignoring "
            "/simulation_initial_lower_arm_joint_state: expected 18 values, "
            "received %zu",
            simulation_initial_lower_arm_joint_state.size());
      }
    }

    // use_custom_arm_prepare:=true 时用 arm_prepare_pose.yaml 覆盖双臂，并按配置做往返摆动。
    // false 或缺参时保持上面的写死臂姿，以及 l5/r5 在 1.5s 内 ±0.5236 的往返。
    const int armStartIndex = 7 + lowJointNum_;
    bool useCustomArmPrepare = false;
    int swingJointIndex = 4;
    double swingRad = 0.5236;
    double preActionDesiredTime = 1.5;
    controllerNh_.getParam("/use_custom_arm_prepare", useCustomArmPrepare);
    if (useCustomArmPrepare)
    {
      std::vector<double> customArmPos;
      const bool hasPose = controllerNh_.getParam("/arm_prepare/joint_pos", customArmPos);
      if (!hasPose || static_cast<int>(customArmPos.size()) != armNum_)
      {
        ROS_ERROR("[humanoidControllerWheelWbc] use_custom_arm_prepare is true, but /arm_prepare/joint_pos is missing or size %zu != %zu. Fallback to built-in pose.",
                  customArmPos.size(), armNum_);
        useCustomArmPrepare = false;
      }
      else
      {
        controllerNh_.param("/arm_prepare/swing_joint_index", swingJointIndex, 4);
        controllerNh_.param("/arm_prepare/swing_rad", swingRad, 0.5236);
        controllerNh_.param("/arm_prepare/duration", preActionDesiredTime, 1.5);
        const int armDofPerSide = armNum_ / 2;
        if (swingJointIndex < 0 || swingJointIndex >= armDofPerSide)
        {
          ROS_ERROR("[humanoidControllerWheelWbc] /arm_prepare/swing_joint_index=%d out of range [0, %d). Fallback to 4 (zarm_*5).",
                    swingJointIndex, armDofPerSide);
          swingJointIndex = 4;
        }
        if (!(preActionDesiredTime > 0.0) || !std::isfinite(preActionDesiredTime) || !std::isfinite(swingRad))
        {
          ROS_ERROR("[humanoidControllerWheelWbc] invalid /arm_prepare duration or swing_rad. Fallback to 1.5s and 0.5236 rad.");
          preActionDesiredTime = 1.5;
          swingRad = 0.5236;
        }
        for (int i = 0; i < armNum_; ++i)
        {
          mujoco_q[armStartIndex + i] = customArmPos[i];
        }
        ROS_INFO("[humanoidControllerWheelWbc] custom arm prepare enabled: swing_joint_index=%d, swing_rad=%.4f, duration=%.2f",
                 swingJointIndex, swingRad, preActionDesiredTime);
      }
    }

    std::vector<double> robot_init_state_param;
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      robot_init_state_param.push_back(mujoco_q(i));
    }

    std::vector<double> stand_arm_joint_state_vector;
    for (int i = 0; i < armNum_; i++)
    {
      stand_arm_joint_state_vector.push_back(mujoco_q(armStartIndex + i));
    }

    /******************************** 双臂初始动作 ****************************************/
    vector_t startAction = mujoco_q.tail(manipulatorModelInfo_.armDim + headNum_).head(manipulatorModelInfo_.armDim);
    vector_t targetAction = startAction;
    if (useCustomArmPrepare)
    {
      // 左臂减去 swing_rad，右臂加上 swing_rad，0.5*duration 到达中点，duration 回到起点。
      targetAction.tail(armNum_)[swingJointIndex] =
          startAction.tail(armNum_)[swingJointIndex] - swingRad;
      targetAction.tail(armNum_ / 2)[swingJointIndex] =
          startAction.tail(armNum_ / 2)[swingJointIndex] + swingRad;
    }
    else
    {
      targetAction.tail(armNum_)[4] = startAction.tail(armNum_)[4] - 0.5236;
      targetAction.tail(armNum_/2)[4] = startAction.tail(armNum_/2)[4] + 0.5236;
      preActionDesiredTime = 1.5;
    }
    initialPreTargetActions(startAction, targetAction, preActionDesiredTime); // 设置机器人启动初始动作
    /************************************************************************************/

    controllerNh_.setParam("/robot_init_state_param", robot_init_state_param);
    controllerNh_.setParam("/standJointState", stand_arm_joint_state_vector);

    // 设置初始状态参数
    std::vector<double> initial_state_vector(robot_init_state_param);
    std::vector<double> squat_initial_state_vector(robot_init_state_param);
    std::vector<double> default_joint_pos_vector(robot_init_state_param);
    controllerNh_.setParam("/initial_state", initial_state_vector);
    controllerNh_.setParam("/squat_initial_state", squat_initial_state_vector);
    controllerNh_.setParam("/default_joint_pos", default_joint_pos_vector);

    // 初始化 MPC 初始期望
    optimizedState_mrt_.tail(manipulatorModelInfo_.armDim) = mujoco_q.segment(7, manipulatorModelInfo_.armDim);
    optimizedState_mrt_limit_ = optimizedState_mrt_;
    optimizedInput_mrt_limit_ = optimizedInput_mrt_;

    // 初始化VR控制相关标志位和参数
    is_transitioning_ = false;
    prev_whole_torso_ctrl_ = false;
    transition_start_time_ = 0.0;

    //TODO
    controllerNh_.setParam("build_cppad_state", 2); // done

    // 初始化发布者
    cmdVelPub_ = controllerNh_.advertise<geometry_msgs::Twist>("/move_base/base_cmd_vel", 10, true);
    baseLockStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/humanoid_wheel/base_locked", 1, true);
    std_msgs::Bool base_lock_state;
    base_lock_state.data = base_locked_;
    baseLockStatePub_.publish(base_lock_state);
    velControlStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/enable_vel_control_state", 1, true);
    enableControlStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/enable_control_state", 1, true);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    waistYawKinematicPublisher_ = controllerNh_.advertise<nav_msgs::Odometry>("/waist_yaw_link_kinematic", 10);
    lbLegTrajPub_ = controllerNh_.advertise<sensor_msgs::JointState>("/lb_leg_traj", 10);
    stopRobotPub_ = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);
    armTrajFilteredPub_ = controllerNh_.advertise<sensor_msgs::JointState>("/vr_incremental/kuavo_arm_traj_filtered", 10);
    resetToStatePub_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/mobile_manipulator_reset_to_state", 1);
    armControlModeStatusPub_ = controllerNh_.advertise<std_msgs::Float64MultiArray>(
        "/humanoid_wheel/arm_control_mode_status", 1, true);

    // 双手末端 FK 话题：/sensors_data_raw/ee_fk/<末端帧名>
    handFkPubs_.resize(manipulatorModelInfo_.eeFrames.size());
    for (size_t i = 0; i < manipulatorModelInfo_.eeFrames.size(); ++i) {
      handFkPubs_[i] = controllerNh_.advertise<geometry_msgs::PoseStamped>(
          "/sensors_data_raw/ee_fk/" + manipulatorModelInfo_.eeFrames[i], 10);
    }

    // 发布初始速度控制开关状态
    {
      std_msgs::Bool msg;
      msg.data = use_vel_control_;
      velControlStatePub_.publish(msg);
    }

    // 发布初始 enable_control 状态 (latched)：启动即 IDLE 可控
    {
      std_msgs::Bool msg;
      msg.data = true;
      enableControlStatePub_.publish(msg);
    }

    // 注册 /enable_control service（回调访问 WBC 成员推进软暂停状态机）
    enableControlServiceServer_ = controllerNh_.advertiseService(
        "/enable_control", &humanoidControllerWheelWbc::enableControlCallback, this);

    // 创建控制数据管理器（替代所有订阅者和服务）
    vector_t leg_initial_state = optimizedState_mrt_.tail(manipulatorModelInfo_.armDim).head(lowJointNum_);
    vector_t arm_initial_state = optimizedState_mrt_.tail(manipulatorModelInfo_.armDim).tail(armNum_);
    control_data_manager_ = std::make_unique<ControlDataManager>(
        controllerNh_, is_real_, armNum_, lowJointNum_, headNum_, leg_initial_state, arm_initial_state);

    // 初始化所有订阅者（包括传感器数据订阅）
    control_data_manager_->initializeSubscribers();

    // 注册服务回调
    registerAllServices();
    // 3791: 双臂 home 取 mujoco_q 初始臂段（[7+lowJointNum_ : 7+lowJointNum_+armNum_)，左臂7+右臂7，
    // 与站姿初始化处提取方式一致（起蹦版在其初始化处同样自动适配）。
    // 此前 firstRun 写死双足遗留值 [-0.0, 0.4, 0.2, -1.5, ...]，关节序不符轮臂，
    // mode 1 会把 WBC 目标拉到错误关节位。
    init_arm_target_qpos_ = mujoco_q.segment(7 + lowJointNum_, armNum_);

    // 初始化手臂目标位置向量
    prev_arm_trajectory_mode_ = arm_trajectory_mode_;

    // 初始化200ms保持期相关变量
    arm_mode_switch_start_time_ = 0.0;
    publishArmControlModeStatus();

    // 初始化腰部运动学计算器
    waistKinematics_ = std::make_shared<humanoid_controller::WaistKinematics>();

    // 设置 Pinocchio 接口，保证模型数据一致性
    if (pinocchioInterface_ptr_)
    {
        waistKinematics_->setPinocchioInterface(pinocchioInterface_ptr_, "waist_yaw_link");
        ROS_INFO("[wheel_absolute_control] WaistKinematics initialized with Pinocchio interface");
    }

    // 初始化中值滤波历史数据缓存
    median_filter_history_.resize(lowJointNum_);  // 4个关节
    for (auto& history : median_filter_history_)
    {
        history.reserve(MEDIAN_FILTER_WINDOW_SIZE);
    }

    // 初始化上一次滤波后的关节位置
    last_filtered_low_joint_pos_ = vector_t::Zero(lowJointNum_);

    // 设置ContactForceWbc指针到DesiredForceManager
    auto contact_force_wbc = std::dynamic_pointer_cast<mobile_manipulator::AbsoluteContactForceWbc>(wheel_wbc_);

    // 初始化期望力管理器（从ContactForceWbc获取插值速度参数）
    double interpolation_speed = contact_force_wbc ? contact_force_wbc->getInterpolationSpeed() : 15.0;
    if (wheel_wbc_)
      desired_force_manager_ = std::make_unique<DesiredForceManager>(controllerNh_, interpolation_speed);

    if (contact_force_wbc && desired_force_manager_) {
      desired_force_manager_->setContactForceWbc(contact_force_wbc);
    }

    // 初始化过渡起点位置
    waist_transition_start_pos_ = vector_t::Zero(lowJointNum_);

    arm_force_estimator_ = std::make_unique<ArmContactForceEstimatorWheel>(
        pinocchioInterface_ptr_, manipulatorModelInfo_, lowJointNum_, ros_logger_);

    // 初始化MPC模式切换服务客户端并设置为ArmOnly模式（仅在启用外部MPC且VR模式时）
    if(enable_mpc_ && use_vr_control_)
    {
      mpc_control_client_ = controllerNh_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");

      // 初始化时设置为BaseArm模式模式，用于VR躯干控制
      kuavo_msgs::changeTorsoCtrlMode srv;
      srv.request.control_mode = 3;
      if(mpc_control_client_.call(srv) && srv.response.result)
      {
        ROS_INFO("[wheel_absolute_control] MPC mode initialized to ArmOnly for VR torso control");
      }
      else
      {
        ROS_WARN("[wheel_absolute_control] Failed to initialize MPC mode to ArmOnly");
      }
    }

    // 初始化重置cmdVel Ruckig规划器服务客户端
    reset_cmd_vel_ruckig_client_ = controllerNh_.serviceClient<std_srvs::SetBool>("/mobile_manipulator_reset_cmd_vel_ruckig");
    reset_cmd_vel_ruckig_srv_.request.data = true;  // 重新规划
    last_reset_cmd_vel_ruckig_time_ = ros::Time::now();  // 初始化重置时间

    // 设置CPU内核隔离（与双足 humanoidController 一致；init 在 controlLoop 线程调用）
    if (is_real_)
    {
      if (!setupCpuIsolation())
      {
        std::cerr << "\033[1;31m"
                  << "==============================\n"
                  << "  错误：未检测到 CPU 内核隔离！\n"
                  << "  请先配置 CPU 内核隔离且配置内核隔离参数 isolated_cpus\n"
                  << "  建议使用脚本 isolate_cores.sh 进行设置。\n"
                  << "  示例：sudo bash ./tools/check_tool/isolate_cores.sh\n"
                  << "=============================="
                  << "\033[0m" << std::endl;
        return false;
      }
    }

    // 初始化底盘调度模式服务客户端
    dispatch_mode_client_ = controllerNh_.serviceClient<leju_mobile_base_msgs::SetDispatchMode>("/move_base/set_dispatch_mode");
    // ========== 底盘急停保护初始化 ==========
    if (controllerNh_.hasParam("/enable_base_emergency_stop"))
    {
      controllerNh_.getParam("/enable_base_emergency_stop", enable_base_emergency_stop_);
    }
    ROS_INFO("[humanoidControllerWheelWbc] enable_base_emergency_stop: %s",
              enable_base_emergency_stop_ ? "true" : "false");

    if (enable_base_emergency_stop_)
    {
      baseCmdVelStatusSub_ = controllerNh_.subscribe<leju_mobile_base_msgs::BaseCmdVelStatus>(
          "/move_base/base_cmd_vel_status", 10,
          &humanoidControllerWheelWbc::baseCmdVelStatusCallback, this);
      ROS_INFO("[humanoidControllerWheelWbc] Subscribed to /move_base/base_cmd_vel_status for base emergency stop protection");
    }

    return true;
  }

  //TODO 此设计over-engineered，后续看情况优化
  void humanoidControllerWheelWbc::registerAllServices()
  {
    // 使用通用接口逐个注册服务

    // 1. 手臂轨迹控制服务
    // control_data_manager_->registerService<kuavo_msgs::changeArmCtrlMode>(
    //     "/enable_wbc_arm_trajectory_control",
    //     [this](auto& req, auto& res) {
    //         return enableArmTrajectoryControlCallback(req, res);
    //     }
    // );

    // 2. 手臂控制模式切换服务
    control_data_manager_->registerService<kuavo_msgs::changeArmCtrlMode>(
        "/change_arm_ctrl_mode",
        [this](auto& req, auto& res) {
            return changeArmCtrlModeCallback(req, res);
        }
    );

    // SetIncrementalArmTrajLink 由 ArmTrajReceiver 在 initializeSubscribers 中 advertise
    // (/humanoid_wheel/set_incremental_arm_traj_link)

    // 3. 腰部逆运动学服务
    control_data_manager_->registerService<kuavo_msgs::lbBaseLinkPoseCmdSrv>(
        "/lb_optimization_ik_service",
        [this](auto& req, auto& res) {
            return handleWaistIkService(req, res);
        }
    );

    // 4. 轮臂MPC, 手臂快慢运动模式切换服务
    control_data_manager_->registerService<kuavo_msgs::changeLbQuickModeSrv>(
        "/enable_lb_arm_quick_mode",
        [this](auto& req, auto& res) {
            return enableLbArmQuickModeCallback(req, res);
        }
    );

    // 5. 轮臂MPC, 关节反馈机制切换服务
    control_data_manager_->registerService<kuavo_msgs::changeLbMpcObsUpdateModeSrv>(
        "/change_lb_mpc_obs_update_mode",
        [this](auto& req, auto& res) {
            return changeLbObsUpdateModeCallback(req, res);
        }
    );

    control_data_manager_->registerService<std_srvs::SetBool>(
      "/enable_vel_control",
      [this](auto& req, auto& res) {
          return enableVelControlCallback(req, res);
      }
  );

    // VR 增量遥操作相关服务
    control_data_manager_->registerService<std_srvs::SetBool>(
        "/enable_vr_arm_accel_task",
        [this](auto& req, auto& res) { return enableVrArmAccelTaskCallback(req, res); }
    );
    control_data_manager_->registerService<std_srvs::SetBool>(
        "/enable_arm_traj_interpolator",
        [this](auto& req, auto& res) { return enableArmTrajInterpCallback(req, res); }
    );
    control_data_manager_->registerService<std_srvs::SetBool>(
        "/enable_vr_arm_kpkd",
        [this](auto& req, auto& res) { return enableVrArmKpKdCallback(req, res); }
    );

    control_data_manager_->registerService<kuavo_msgs::setContactForceInterpParams>(
        "/set_contact_force_params",
        [this](kuavo_msgs::setContactForceInterpParams::Request& req,
               kuavo_msgs::setContactForceInterpParams::Response& res) {
          if (!desired_force_manager_) {
            res.success = false;
            res.message = "DesiredForceManager not ready";
            return true;
          }
          if (req.transition_time <= 0.0 || req.interpolation_speed <= 0.0) {
            res.success = false;
            res.message = "params must be > 0";
            return true;
          }
          desired_force_manager_->applyContactForceInterpParams(req.transition_time, req.interpolation_speed);
          res.success = true;
          res.message = "ok";
          return true;
        });

    ROS_INFO("[humanoidControllerWheelWbc] All ROS services registered through ControlDataManager");
  }

  bool humanoidControllerWheelWbc::enableVelControlCallback(std_srvs::SetBool::Request &req,
                                                         std_srvs::SetBool::Response &res)
  {
    if (fixed_velocity_ik_pipeline_) {
      use_vel_control_ = false;
      res.success = !req.data;
      res.message = "BASE_CONTROL=DISABLED_BY_USER_REQUIREMENT";
      return true;
    }

    std::cout << "[vel_control] 速度控制切换请求: " << (req.data ? "启用" : "禁用") << std::endl;
    use_vel_control_ = req.data;

    // 发布速度控制状态
    {
      std_msgs::Bool msg;
      msg.data = use_vel_control_;
      velControlStatePub_.publish(msg);
    }

    res.success = true;
    res.message = "success change vel control to " + std::to_string(req.data);
    return true;
  }

  bool humanoidControllerWheelWbc::enableControlCallback(std_srvs::SetBool::Request &req,
                                                         std_srvs::SetBool::Response &res)
  {
    ROS_INFO("[enable_control] 控制使能切换请求: %s", req.data ? "启用" : "禁用");

    // 3791: 受理/拒绝矩阵（简单拒绝，不入队）：
    //   IDLE    → 受理 disable（进 PAUSING），拒绝 enable（already）
    //   PAUSING → 拒绝一切（250 帧过渡中）
    //   PAUSED  → 受理 enable（进 RESUMING），拒绝 disable（已暂停）
    //   RESUMING → 拒绝一切（250 帧过渡中）
    // 快速 toggle 被固定周期数吸收：请求只在稳定态被受理，服务不会堆积排队。
    const SoftPauseState st = soft_pause_state_.load();
    const bool accept = (st == SoftPauseState::IDLE && !req.data)
                     || (st == SoftPauseState::PAUSED && req.data);
    if (!accept)
    {
      ROS_WARN_THROTTLE(0.5, "[enable_control] 状态=%d 拒绝本次 %s 请求（仅稳定态受理，简单拒绝不入队）",
                        static_cast<int>(st), req.data ? "启用" : "禁用");
      res.success = false;
      res.message = "soft pause state " + std::to_string(static_cast<int>(st))
                  + " rejects " + (req.data ? "enable" : "disable") + ", retry later";
      return true;
    }

    if (!req.data)
    {
      // ===== disable 受理：立即让整条管线停 =====
      // 受理即发 enable=false（全局同步总开关），不等主循环；
      // 冻结记录刻意不做——由主循环 PAUSING 首拍从上一拍执行值冻结，保证"执行值 == 冻结值"零跳变。
      {
        std_msgs::Bool msg;
        msg.data = false;
        enableControlStatePub_.publish(msg);
      }
      mpc_policy_gate_open_ = false;
      // 3791: pause 转交主循环安全点执行——pauseResumeMpcNode 内部是 roscpp 同步服务
      // 调用（无超时），不能阻塞本回调线程（会卡住整个服务队列）。标志由主循环消费。
      mpc_pause_requested_.store(true);
      // 3791: 每次 disable 重新记录冻结姿态 —— frozen_state_valid_ 置 true 后不会自动复位，
      // 若不清，第二次及以后 disable 时 PAUSING 首拍的冻结记录块（!frozen_state_valid_ 门控）
      // 被跳过，frozen_state_ 停留在第一次 disable 的陈旧姿态（如 home），软暂停时机身
      // 会回陈旧冻结位、enable 后 rst/观测/RM 锚定全部错位导致甩飞。清掉后 PAUSING 首拍
      // 自然重新记录本次 disable 的真实姿态。
      frozen_state_valid_.store(false);
      // 计数器先于状态置位：state 可见时主循环读到的配套值必为当前值（与 enable 分支同序）
      soft_pause_transition_cycles_.store(0);
      soft_pause_state_.store(SoftPauseState::PAUSING);
      ROS_INFO("[enable_control] disable 受理: enable=false 已发布, MPC pause 已请求(主循环执行), 进入 PAUSING");
    }
    else
    {
      // ===== enable 受理：从冻结干净重启（不发 enable=true，到 IDLE 才发） =====
      // resume 立即发（solver 尽快恢复）；resetMpcNode/rst_target/滤波器 reset
      // 由主循环 RESUMING 首拍完成（FK 计算需主线程）；RESUMING 期间 MPC 观测
      // 强制 = 冻结+odom（反向写回），保证 solver 起点与 RM 参考锚一致。
      mpc_policy_gate_open_ = false;
      // resume 同样转交主循环安全点执行（理由同 disable 分支）
      mpc_resume_requested_.store(true);
      // 计数器与 reset_done 必须先于 state 置位：若 state 先可见，主循环可能读到
      // 上一周期的陈旧值（reset_done=true 跳过首拍 reset、counter=250 立即转 IDLE），
      // 导致 RM 未重置就解锁（3791 审查 WBC-T1）
      soft_pause_transition_cycles_.store(0);
      resuming_reset_done_.store(false);
      soft_pause_state_.store(SoftPauseState::RESUMING);
      ROS_INFO("[enable_control] enable 受理: MPC resume 已请求(主循环执行), 进入 RESUMING（IDLE 时发布 enable=true）");
    }

    // 通知底盘调度模式：软急停/恢复。刻意放在本地副作用之后——roscpp call 无超时，
    // 底盘服务挂起不能阻塞本地冻结（安全链不依赖外部服务结果）
    if (dispatch_mode_client_.exists())
    {
      leju_mobile_base_msgs::SetDispatchMode srv;
      srv.request.control_mode = req.data ? 1 : 0;  // 1: 取消软急停, 0: 软急停
      if (dispatch_mode_client_.call(srv))
      {
        ROS_INFO("[enable_control] 底盘 dispatch_mode=%d 调用成功: %s",
                 srv.request.control_mode, srv.response.message.c_str());
      }
      else
      {
        ROS_WARN("[enable_control] 底盘 dispatch_mode=%d 调用失败",
                 srv.request.control_mode);
      }
    }
    else
    {
      ROS_WARN("[enable_control] 底盘 dispatch_mode 服务不可用，跳过");
    }

    res.success = true;
    res.message = "enable_control set to " + std::to_string(req.data);
    return true;
  }

  bool humanoidControllerWheelWbc::enableVrArmAccelTaskCallback(std_srvs::SetBool::Request &req,
                                                                std_srvs::SetBool::Response &res)
  {
    if (!wheel_wbc_) {
      res.success=false; res.message="WBC tasks are unavailable in RNEA mode"; return true;
    }
    wheel_wbc_->setUseVrArmAccelTask(req.data);
    ROS_INFO("[humanoidControllerWheelWbc] useVrArmAccelTask set to %s", req.data ? "true" : "false");
    res.success = true;
    res.message = std::string("useVrArmAccelTask set to ") + (req.data ? "true" : "false");
    return true;
  }

  bool humanoidControllerWheelWbc::enableArmTrajInterpCallback(std_srvs::SetBool::Request &req,
                                                               std_srvs::SetBool::Response &res)
  {
    enable_arm_traj_interpolator_ = req.data;
    ROS_INFO("[humanoidControllerWheelWbc] enable_arm_traj_interpolator set to %s", req.data ? "true" : "false");
    res.success = true;
    res.message = std::string("enable_arm_traj_interpolator set to ") + (req.data ? "true" : "false");
    return true;
  }

  bool humanoidControllerWheelWbc::enableVrArmKpKdCallback(std_srvs::SetBool::Request &req,
                                                           std_srvs::SetBool::Response &res)
  {
    kuavo_settings_.running_settings.use_vr_arm_kpkd = req.data;
    ROS_INFO("[humanoidControllerWheelWbc] use_vr_arm_kpkd set to %s", req.data ? "true" : "false");
    res.success = true;
    res.message = std::string("use_vr_arm_kpkd set to ") + (req.data ? "true" : "false");
    return true;
  }

  bool humanoidControllerWheelWbc::starting(const ros::Time &time,
                                            const std::atomic<bool>* running)
  {
    const auto keep_running = [running] {
      return ros::ok() && (!running || running->load(std::memory_order_acquire));
    };
    if (!keep_running()) return false;
    ROS_WARN_THROTTLE(1.0, "[starting] Waiting for odometry data...");
    // 1. 启动仿真/硬件（回放模式跳过）
    bool play_back = false;
    controllerNh_.getParam("/play_back", play_back);
    if (play_back) {
      ROS_INFO("[starting] Play back mode, skipping sim/hardware startup");
    } else if (!is_real_) {
      if (!callSimStartSrv(controllerNh_, running)) return false;
    } else {
      // 等待硬件就绪
      int isHardwareReady = 0;
      while (keep_running() && isHardwareReady != 1)
      {
        controllerNh_.getParam("/hardware/is_ready", isHardwareReady);
        usleep(10000);  // 10ms
      }
    }

    // 2. 等待数据就绪
    ROS_INFO("Waiting for ControlDataManager data...");
    auto start = std::chrono::steady_clock::now();
    int wait_sec = 0;

    control_data_manager_->setOdomReset();  // 重置里程计

    while (keep_running() && !control_data_manager_->isDataReady(false))
    {
        ros::spinOnce();
        usleep(1000);

        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        double timeout = play_back ? -1.0 : 5.0;  // 回放模式不超时
        if (timeout > 0 && elapsed > timeout) {
            ROS_ERROR("Data timeout! Check: /sensors_data_raw, /odom, /waist_yaw_link_pose");
            return false;
        }

        // 每秒打印一次
        if (static_cast<int>(elapsed) > wait_sec) {
            wait_sec = static_cast<int>(elapsed);
            if (play_back) {
                ROS_WARN("[play_back] Waiting for bag data... %d s", wait_sec);
            } else {
                ROS_WARN("Waiting... %d/5 s", wait_sec);
            }
        }
    }

    // 统一检查 ROS 状态（覆盖所有异常退出情况）
    if (!keep_running())
    {
      ROS_INFO("Controller startup cancelled or ROS shutdown detected");
      return false;
    }

    ROS_INFO("Data ready. Controller starting complete.");

    // control_data_manager_->setOdomReset();  // 重置里程计
    return true;
  }

  bool humanoidControllerWheelWbc::preUpdate(const ros::Time& time)
  {
    if (dynamics_fault_) { stopDynamics("latched dynamics fault"); return false; }
    if (isPreUpdateComplete) return true;
    SensorData sensors_data_new;
    if (!control_data_manager_->getRealtimeSensorData(sensors_data_new)) {
      ROS_WARN_THROTTLE(1.0, "[preUpdate] Waiting for measured joint state");
      return false;
    }
    if (!checkDynamicsSensor(sensors_data_new, true)) return false;
    const Eigen::Index measured_joint_count = manipulatorModelInfo_.armDim + headNum_;
    if (sensors_data_new.jointPos_.size() < measured_joint_count ||
        sensors_data_new.jointVel_.size() < measured_joint_count ||
        !sensors_data_new.jointPos_.allFinite() || !sensors_data_new.jointVel_.allFinite()) {
      stopDynamics("Incomplete measured startup joint state");
      return false;
    }
    vector6_t odom = vector6_t::Zero();
    control_data_manager_->getRealtimeOdomData(odom);
    computeObservationFromSensorData(sensors_data_new, odom);

    // Absolute mode starts from this instance's measured state. No configured
    // prepare pose or pre-owner trajectory is allowed to command the robot.
    // Measured dq remains in the observation; desired dq starts at zero.
    const auto& info = manipulatorModelInfo_;
    if (observation_wheel_.state.size() != info.stateDim ||
        observation_wheel_.input.size() != info.inputDim ||
        !observation_wheel_.state.allFinite() ||
        !observation_wheel_.input.allFinite()) {
      stopDynamics("Invalid measured startup state");
      return false;
    }
    optimizedState_mrt_ = observation_wheel_.state;
    optimizedInput_mrt_ = vector_t::Zero(info.inputDim);
    optimizedState_mrt_limit_ = optimizedState_mrt_;
    optimizedInput_mrt_limit_ = optimizedInput_mrt_;
    fixed_pipeline_last_q = observation_wheel_.state.tail(info.armDim);
    if (headNum_ > 0 && !control_data_manager_->initializeHeadFromMeasured(
            sensors_data_new.jointPos_.tail(headNum_))) {
      stopDynamics("Invalid measured startup head state");
      return false;
    }
    setupMrt();
    initMPC();  // resets observation filters; fixed mode never waits for a policy
    isPreUpdateComplete = true;
    ROS_INFO("[FixedVelocityIK] Measured hold initialized; waiting for formal authority");
    return true;
  }

  void humanoidControllerWheelWbc::update(const ros::Time &time, const ros::Duration &dfd)
  {
    if (dynamics_fault_) { stopDynamics("latched dynamics fault"); return; }
    static bool base_emergency_handled_ = false;
    if (enable_base_emergency_stop_ && base_emergency_triggered_.load() && !base_emergency_handled_)
    {
      base_emergency_handled_ = true;
      publishStopRobot();
      return;
    }

    static const double firstTime = time.toSec();
    double curTime = time.toSec() - firstTime;
    static double lastTime = curTime - dt_;
    double dt = curTime - lastTime;
    // 外环 controlLoop 若用 nanosleep 定频，ros::Time 抖动会使 dt 略小于 dt_。
    // 旧逻辑 if(dt < dt_) return 会漏拍，下一拍 dt≈2*dt_ → 频率在 500/250Hz 跳变。
    // 仅过滤明显的同拍重复（半周期内），并优先采用外环 wall elapsed。
    const double wallElapsed = dfd.toSec();
    if (wallElapsed > 0.5 * dt_ && wallElapsed < 1.8 * dt_) {
      dt = wallElapsed;
    }
    if (dt < 0.5 * dt_) {
      if (ros_logger_) {
        ros_logger_->publishValue("/monitor/wheel/update_early_return", 1.0);
      }
      return;
    }
    lastTime = curTime;
    const auto updateWallStart = std::chrono::steady_clock::now();
    auto stageStart = updateWallStart;
    auto markStageMs = [&](const char* stage) {
      const auto now = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(now - stageStart).count();
      if (ros_logger_) {
        ros_logger_->publishValue(std::string("/monitor/time_cost/wheel/") + stage, ms);
      }
      stageStart = now;
      return ms;
    };

    ros_logger_->publishValue("/humanoid_wheel/freq", 1 / dt);
    ros_logger_->publishValue("/humanoid_wheel/dt_real", dt);
    // 与双足一致的 monitor 话题，便于统一看 WBC 频率/耗时
    ros_logger_->publishValue("/monitor/frequency/wbc", 1.0 / dt);
    static auto timeInit = time.toSec();
    auto& info = manipulatorModelInfo_;
    static int cnt = 0;
    if(cnt % 500 == 0)
    {
      // std::cout << "update is running, time is " << time.toSec() - timeInit << std::endl;
    }
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    auto bIsgetSensorData = control_data_manager_->getRealtimeSensorData(sensors_data_new);
    if (!checkDynamicsSensor(sensors_data_new, bIsgetSensorData)) return;
    if (!bIsgetSensorData)
    {
        ROS_ERROR_THROTTLE(1.0, "[update] Failed to get sensor realtime data");
    }

    // 从控制数据管理器获取里程计数据
    vector6_t odomData_new = vector6_t::Zero();
    auto bIsgetOdomData = control_data_manager_->getRealtimeOdomData(odomData_new);
    if (!bIsgetOdomData)
    {
        ROS_WARN_THROTTLE(1.0, "[update] Failed to get odometry realtime data");
    }

    computeObservationFromSensorData(sensors_data_new, odomData_new);
    markStageMs("sensor_obs");

    // 对 update() 中读回的关节角做双手末端 FK，发布到 /sensors_data_raw/ee_fk/
    publishHandEndEffectorFK(observation_wheel_.state);
    markStageMs("hand_fk");

    /********************************  计算关键点笛卡尔跟踪分析(局部系) ********************************/
    vector_t targetStateTmp = optimizedState_mrt_limit_;
    targetStateTmp.head(3) = Eigen::Vector3d::Zero();
    vector_t currentStateTmp = observation_wheel_.state;
    currentStateTmp.head(3) = Eigen::Vector3d::Zero();
    computeErrorMultiEeFromTargetAndData(targetStateTmp, currentStateTmp);
    /**********************************************************************************************/

    // 更新 mpc 数据
    {
      vector_t optimizedState_mrt, optimizedInput_mrt;
      // Update the current state of the system
      SystemObservation kinemicLimitObs = observation_wheel_;
      kinemicLimitObs.state = obsStateLimitFilterPtr_->update(observation_wheel_.state);
      kinemicLimitObs.input = obsInputLimitFilterPtr_->update(observation_wheel_.input);

      /****************************允许采用mpc输出作为反馈**************************************/
      // 3791 反向写回：RESUMING 期间（solver 已 resume、rollout 未接管）MPC 观测强制 =
      // 冻结关节段 + 当前 odom（== 主循环 rst_state），覆盖模式 1/2/3。250 帧（0.5s）
      // 过渡窗口内观测恒为冻结起点，resume 服务何时生效无关（生效后第一帧观测必然 =
      // rst_state），solver 求解起点与 RM 参考锚一致，第一帧求解不跳。
      if (soft_pause_state_.load() == SoftPauseState::RESUMING)
      {
        std::lock_guard<std::mutex> lock(frozen_state_mutex_);
        if (frozen_state_valid_.load())
        {
          kinemicLimitObs.state = frozen_state_;
          // 底盘段刷新为当前滤波 odom（disable 期间 rollout 块每拍刷新的优化State_mrt_.head）
          kinemicLimitObs.state.head(baseDim_) = optimizedState_mrt_.head(baseDim_);
        }
      }
      else
      {
        if(mpcObsUpdateMode_ == 1 || mpcObsUpdateMode_ == 3)
        {
          kinemicLimitObs.state.segment(baseDim_, lowJointNum_) = optimizedState_mrt_.segment(baseDim_, lowJointNum_);
          kinemicLimitObs.input.segment(baseDim_, lowJointNum_) = optimizedInput_mrt_.segment(baseDim_, lowJointNum_);
        }
        if(mpcObsUpdateMode_ == 2 || mpcObsUpdateMode_ == 3)
        {
          kinemicLimitObs.state.tail(armNum_) = optimizedState_mrt_.tail(armNum_);
          kinemicLimitObs.input.tail(armNum_) = optimizedInput_mrt_.tail(armNum_);
        }
      }
      /**************************************************************************************/

      kinemicLimitObs.time = curTime;
      mrtRosInterface_->setCurrentObservation_directPub(kinemicLimitObs, mpcDt_);

      // Trigger MRT callbacks
      mrtRosInterface_->spinMRT();
      // Update the policy if a new one was received
      bool mrtPolicyUpdated = mrtRosInterface_->updatePolicy();

      // 回放模式下将 MRT 请求时间锚定到 bag 策略时间轴（录制/回放时钟偏移对齐）。
      // bag 中的策略按"录制控制器 curTime"锚定，回放控制器用的是自身 curTime，
      // 两条时钟偏移一旦超过 MPC 时域(0.3s)就报 "currentTime > received plan"。
      // 每次换入新策略时记录 bag 时间锚(=策略初始观测时间)与本地时间锚(=当时
      // curTime)，请求时间 = bag锚 + (curTime - 本地锚)，随 bag 策略推进并吸收偏移。
      if (is_play_back_mode_ && mrtPolicyUpdated && mpc_policy_gate_open_ && mrtRosInterface_->isPolicyUpdated())
      {
        const auto &activeCmd = mrtRosInterface_->getCommand();
        bag_time_anchor_ = activeCmd.mpcInitObservation_.time;
        local_time_anchor_ = curTime;
        policy_time_anchor_valid_ = true;
      }
      if (is_play_back_mode_ && policy_time_anchor_valid_)
      {
        kinemicLimitObs.time = bag_time_anchor_ + (curTime - local_time_anchor_);
      }

      // 3791: disable 下降沿 pauseResumeMpcNode(true)、enable 上升沿 resetMpcNode 均异步调用 reset()，
      // 会清空 activePrimalSolutionPtr_。此时 rolloutPolicy/getPolicy/getCommand 会抛
      // "[MRT_BASE::rolloutPolicy] updatePolicy() should be called first!"。
      // 双足用 resetting_mpc_state_ 状态机在重置未就绪时跳过 MPC 输出；此处用 isPolicyUpdated()
      // 判活：active policy 无效时跳过 rollout，optimizedState_mrt_ 维持缓存值
      // （disable=frozen，enable 上升沿=rst_state），由后续 freeze 覆写/限幅滤波保持稳定。
      // 闸门：MRT reset() 在 detached 线程执行（立即或 pause 服务往返后析构 active policy），
      // getPolicy() 返回裸引用不持有计数——仅靠 isPolicyUpdated() 检查仍存在"检查通过后 reset
      // 析构 → 悬垂访问"的竞态（曾崩于 publishOptimizedTrajectory，GPF）。故边沿处理先置
      // mpc_policy_gate_open_=false，主线程观察到 active 被清空（reset 完成）后才开门，
      // 期间 rollout/可视化均不访问 MRT。与双足 resetting_mpc_state_（非 NORMAL 不碰 policy）
      // 同思想，且边沿处理在本线程串行执行，无跨线程窗口。
      if (!mpc_policy_gate_open_ && !mrtRosInterface_->isPolicyUpdated())
        mpc_policy_gate_open_ = true;
      // 3791: 仅 IDLE（状态机稳定 + enable 已发布 true）才使用 MPC policy 输出；
      // PAUSING/PAUSED/RESUMING 期间 rollout 不跑，optimizedState_mrt_ 维持缓存值
      // （disable=frozen / enable=rst_state），由冻结覆写/限幅滤波保持稳定。
      const bool mpc_policy_ready = soft_pause_state_.load() == SoftPauseState::IDLE
                                    && mpc_policy_gate_open_ && mrtRosInterface_->isPolicyUpdated();
      if (mpc_policy_ready)
      {
        // mrtRosInterface_->evaluatePolicy(kinemicLimitObs.time + dt_, kinemicLimitObs.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
        mrtRosInterface_->rolloutPolicy(kinemicLimitObs.time, kinemicLimitObs.state, dt_, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
        if(enable_mpc_)
        {
          optimizedState_mrt_ = optimizedState_mrt;
          optimizedInput_mrt_ = optimizedInput_mrt;
        }
        if(std::fabs(optimizedInput_mrt_[0]) < 0.05) optimizedInput_mrt_[0] = 0;
        if(std::fabs(optimizedInput_mrt_[1]) < 0.05) optimizedInput_mrt_[1] = 0;
        if(std::fabs(optimizedInput_mrt_[2]) < 0.05) optimizedInput_mrt_[2] = 0;
      }
      else
      {
        // MPC 输出不可用（暂停中 / 重置后等待首帧策略）：跳过 rollout，不写回关节段，
        // 保持 optimizedState_mrt_ 关节段为缓存值（disable=frozen，enable 上升沿=rst_state）。
        // 但底盘段必须每拍从 odom 实时刷新——基线靠 rollout 每拍产出 base≈odom（rollout 输入
        // kinemicLimitObs.state.head 即滤波 odom），跳过 rollout 后需手动补，否则底盘位姿冻在
        // disable 瞬间。用 kinemicLimitObs.state.head 与基线 rollout 输入同源（滤波 odom）。
        optimizedState_mrt_.head(baseDim_) = kinemicLimitObs.state.head(baseDim_);
      }
    }
    // 更新可视化数据：active policy 无效时 getPolicy/getCommand 会抛异常，退化为仅发布观测。
    // 加状态机 + 闸门：disable 期间/重置未就绪时一律不取 policy（getPolicy 返回裸引用，
    // 与 MRT 异步 reset() 析构存在悬垂竞态，详见上方 rollout 块注释）。
    if (last_visualization_time_ < 0.0 || observation_wheel_.time < last_visualization_time_ ||
        observation_wheel_.time - last_visualization_time_ + 1e-9 >= visualization_period_) {
      if (soft_pause_state_.load() == SoftPauseState::IDLE && mpc_policy_gate_open_ && mrtRosInterface_->isPolicyUpdated())
      {
        robotVisualizer_->update(observation_wheel_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());
      }
      else
      {
        // State estimation above remains full rate.
        robotVisualizer_->update_obs(observation_wheel_);
      }
      last_visualization_time_ = observation_wheel_.time;
    }
    // 3791: 消费 enableControlCallback 转交的 MPC pause/resume 请求（安全点）。
    // 本点位于本拍 rollout/可视化之后、下一拍 rollout 之前；受理时已先关
    // mpc_policy_gate_open_（观察到 post-reset policy 才重开），此处无任何
    // in-flight policy 访问，detached reset 落地时无人在飞。
    if (mpc_pause_requested_.exchange(false) && enable_mpc_ && mrtRosInterface_)
      mrtRosInterface_->pauseResumeMpcNode(true);
    if (mpc_resume_requested_.exchange(false) && enable_mpc_ && mrtRosInterface_)
      mrtRosInterface_->pauseResumeMpcNode(false);
    markStageMs("mpc_mrt");

    /******************  用户修改部分  ***********************/
    vector_t target_qpos, target_qvel;
    target_qpos.setZero(info.armDim);
    target_qvel.setZero(info.armDim);

    // Each controller instance retains its own last formal
    // stopped reference if authority is normally released; never return to an
    // old MPC/position target or recapture measured q on every cycle.
    WholeBodyIkReference wholeBodyIkReference;
    bool wholeBodyIkAuthorityOwned = false;
    const bool wholeBodyIkReferenceUsable =
        control_data_manager_->resolveWholeBodyIkReference(
            wholeBodyIkReference, wholeBodyIkAuthorityOwned, dt_,
            !enable_mpc_ && baseCmdVelStatus_ &&
            soft_pause_state_.load() == SoftPauseState::IDLE);
    if (wholeBodyIkAuthorityOwned &&
        (wholeBodyIkReference.q.size() != info.stateDim ||
         wholeBodyIkReference.dq.size() != info.inputDim ||
         !wholeBodyIkReference.q.allFinite() || !wholeBodyIkReference.dq.allFinite())) {
      stopDynamics("Malformed owned absolute reference");
      return;
    }
    const bool wholeBodyIkReferenceFresh = wholeBodyIkReference.source_reference_fresh;
    bool wholeBodyIkReferenceApplied =
        wholeBodyIkAuthorityOwned && wholeBodyIkReferenceUsable &&
        wholeBodyIkReference.takeover_active &&
        wholeBodyIkReference.q.size() ==
            static_cast<Eigen::Index>(info.stateDim) &&
        wholeBodyIkReference.dq.size() ==
            static_cast<Eigen::Index>(info.inputDim);
    if (wholeBodyIkAuthorityOwned && enable_mpc_) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[WholeBodyIkReferenceManager] Formal 21-D IK authority is incompatible with MPC; holding instead of mixing owners");
    }
    if (fixed_velocity_ik_pipeline_ && !wholeBodyIkAuthorityOwned && !enable_mpc_ && fixed_pipeline_last_q.size() == 0) {
      // Defensive initialization from this instance's measured posture.
      // Normal startup already latches this in preUpdate. Never seed a
      // zero-to-home interpolation or follow measured noise each tick.
      fixed_pipeline_last_q = observation_wheel_.state.tail(info.armDim);
    }
    const bool fixedPipelineIdleHold = fixed_velocity_ik_pipeline_ && !enable_mpc_ && !wholeBodyIkAuthorityOwned &&
        fixed_pipeline_last_q.size() == info.armDim;
    if (wholeBodyIkAuthorityOwned && wholeBodyIkReference.q.size() == info.stateDim)
      fixed_pipeline_last_q = wholeBodyIkReference.q.tail(info.armDim);
    if (wholeBodyIkReferenceApplied) {
      target_qpos = wholeBodyIkReference.q.tail(info.armDim);
      target_qvel = wholeBodyIkReference.dq.tail(info.armDim);
    } else if (wholeBodyIkAuthorityOwned) {
      // No per-cycle measured-position reset. An established owner always
      // supplies either its normal tuple or its stopping tuple.
      target_qpos = wholeBodyIkReference.q.tail(info.armDim);
      target_qvel = wholeBodyIkReference.dq.tail(info.armDim);
    } else if (fixedPipelineIdleHold) {
      target_qpos = fixed_pipeline_last_q;
      target_qvel.setZero();  // owner releases only after its stopped endpoint
    } else {
      // With a valid measured state this is unreachable. Do not publish a
      // fallback trajectory; the unchanged HAL handles lost controller output.
      stopDynamics("Absolute reference has no valid initial hold");
      return;
    }

    // Only the formal owner or its latched stopped endpoint can command joints.
    if (wholeBodyIkAuthorityOwned) {
      optimizedState_mrt_ = wholeBodyIkReference.q;
      optimizedInput_mrt_ = wholeBodyIkReference.dq;
    } else {
      optimizedState_mrt_.tail(info.armDim) = target_qpos;
      optimizedInput_mrt_.tail(info.armDim) = target_qvel;
    }
    /*******************************************************/
    ros_logger_->publishVector("/humanoid_wheel/optimizedState_mrt", optimizedState_mrt_);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_mrt", optimizedInput_mrt_);

    // q/dq/ddq are one bounded trajectory transaction. The producer and
    // reference manager validate it before this point. Keep it atomic; original
    // WBC constraints and the original HAL remain downstream safety owners.
    optimizedState_mrt_limit_ = optimizedState_mrt_;
    optimizedInput_mrt_limit_ = optimizedInput_mrt_;
    ros_logger_->publishVector("/humanoid_wheel/optimizedState_mrt_kinemicLimit", optimizedState_mrt_limit_);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_mrt_kinemicLimit", optimizedInput_mrt_limit_);

    // disable 期间：冻结全身关节角度、清零所有速度，底盘不冻结（跟 odom 实时走）
    // 3791: 冻结覆写条件改为状态机 —— enable 受理后状态机仍在 RESUMING，
    // WBC 输出仍需保持冻结（直到 IDLE rollout 接管）
    // 并发协议：frozen_state_ 仅主线程锁内写（PAUSING 首拍）；frozen_state_valid_ 是
    // atomic 可锁外查，服务线程 disable 受理只清 flag 不碰数据，故先查 flag 再加锁安全
    if (!wholeBodyIkAuthorityOwned &&
        soft_pause_state_.load() != SoftPauseState::IDLE && frozen_state_valid_)
    {
      std::lock_guard<std::mutex> lock(frozen_state_mutex_);
      optimizedState_mrt_limit_.segment(baseDim_, info.armDim) = frozen_state_.segment(baseDim_, info.armDim);
      optimizedInput_mrt_limit_.setZero();
    }

    if(enable_mpc_)   // mpc 仅采用硬约束的state作为反馈, 不修改轨迹的动态特性
    {
      optimizedState_mrt_ = optimizedState_mrt_limit_;
      optimizedInput_mrt_ = optimizedInput_mrt_limit_;
    }

    // ===== 软暂停状态机推进（3791 重构：四状态 + 固定周期数）=====
    // 受理/拒绝在 enableControlCallback（服务线程）完成，此处主线程只做推进：
    //   PAUSING  首拍冻结记录 + CDM 写回（主循环消费，取上一拍实际执行值）；
    //            250 帧（0.5s @ 500Hz）到点 → PAUSED
    //   RESUMING 首拍构建 rst_target（FK 需主线程）+ resetMpcNode + 限幅滤波器 reset；
    //            250 帧到点 → IDLE 并发布 enable=true（全局同步总开关）
    // 期间 WBC 输出由冻结覆写（状态 != IDLE）保证为冻结姿态；MPC 观测由观测组装块
    // 强制为冻结+odom（RESUMING，反向写回）。
    // 过渡计时按控制循环帧数（受理时置 0，本块每帧 ++）：计数发生在 update() 主循环
    // 内，与受理记录天然同源，不再有墙钟/仿真时间基准错位问题（3791 状态卡死根因）。
    {
      // --- PAUSING：主循环冻结（首拍）+ 到点转 PAUSED ---
      if (soft_pause_state_.load() == SoftPauseState::PAUSING)
      {
        if (!frozen_state_valid_.load())
        {
          std::lock_guard<std::mutex> lock(frozen_state_mutex_);
          // 取 disable 受理后第一拍的实际执行值（limit filter 后、WBC 输入前的
          // optimizedState_mrt_limit_）：受理后本拍它仍是禁用前最后一拍的输出，即
          // 机器人此刻正在执行的目标位置。取它保证"执行值 == 冻结值"零跳变。
          // 不要取 kinemicLimitObs.state（limit filter 前的 MPC 原始输出，与电机
          // 实际跟踪值可能差一个限幅量）。
          frozen_state_ = optimizedState_mrt_limit_;
          frozen_state_valid_ = true;
          ROS_INFO("[enable_control] frozen state recorded (PAUSING first cycle)");

          // 规划值反向写回 CDM（替代传感器快照），VR/快速模式恢复时与冻结一致
          control_data_manager_->setLbWaistExternalControlState(
              frozen_state_.segment(baseDim_, lowJointNum_));
          control_data_manager_->updateLegExternalControlState(
              frozen_state_.segment(baseDim_, lowJointNum_),
              vector_t::Zero(lowJointNum_), vector_t::Zero(lowJointNum_));
          control_data_manager_->updateArmExternalControlState(
              frozen_state_.tail(armNum_),
              vector_t::Zero(armNum_), vector_t::Zero(armNum_));
        }
        if (soft_pause_transition_cycles_.fetch_add(1) + 1 >= SOFT_PAUSE_TRANSITION_CYCLES)
        {
          soft_pause_state_.store(SoftPauseState::PAUSED);
          ROS_INFO("[enable_control] PAUSING -> PAUSED");
        }
      }
      // --- RESUMING：首拍 reset（从冻结干净重启）+ 到点转 IDLE（发布 enable=true）---
      else if (soft_pause_state_.load() == SoftPauseState::RESUMING)
      {
        // resuming_reset_done_ 首拍保护：resetMpcNode 每次调用都清 active policy，
        // 每帧重复执行会让 solver 永远无法收敛
        if (enable_mpc_ && frozen_state_valid_.load() && !resuming_reset_done_.load())
        {
          std::lock_guard<std::mutex> lock(frozen_state_mutex_);
          // 重置起点：关节段取冻结值（开环规划），底盘段刷新为当前滤波 odom（闭环）。
          // 底盘段取 optimizedState_mrt_.head（禁用期间跳过 rollout 分支每拍从
          // kinemicLimitObs.state.head 刷新的滤波 odom），与 MPC 观测同源，避免起点错位。
          vector_t rst_state = frozen_state_;
          rst_state.head(baseDim_) = optimizedState_mrt_.head(baseDim_);
          vector_t zero_input = vector_t::Zero(optimizedInput_mrt_.size());

          // 目标轨迹必须用 target-pose 格式而非 21 维 MPC state（原因见 buildTargetPoseFromState 注释）
          vector_t rst_target = buildTargetPoseFromState(rst_state, /*zeroBase=*/false);

          mrtRosInterface_->spinMRT();
          mrtRosInterface_->resetMpcNode(
              TargetTrajectories({curTime}, {rst_target}, {zero_input}));

          // 同步限幅滤波器到冻结姿态，消除 Ruckig 滞后跳变
          mrtStateLimitFilterPtr_->reset(rst_state);
          mrtInputLimitFilterPtr_->reset(zero_input);

          // 3791: 把冻结姿态发给 RM（resetAllMpcToState：storage/Ruckig 锚冻结）。
          // 单拍即可：RM pending 锁存至消费，localhost 话题无丢包；多拍曾被跨周期
          // 分批消费导致一次 resume 触发多次 resetAllMpcToState（3791 甩飞诱因之一）
          reset_state_to_publish_ = rst_state;
          reset_to_state_publish_cnt_ = 1;
          resuming_reset_done_.store(true);
          ROS_INFO("[enable_control] MPC reset from frozen state (RESUMING first cycle)");
        }
        if (soft_pause_transition_cycles_.fetch_add(1) + 1 >= SOFT_PAUSE_TRANSITION_CYCLES)
        {
          // 3791: 恢复完成握手——250 帧到点且 MPC policy 已就绪才解锁转 IDLE；
          // 等不到则计数器清零、再顺延一个完整 250 帧窗口（永远不强转），期间
          // WBC 仍冻结输出、enable=true 不发布，不会带着空 policy 解锁。
          // enable_mpc_ 关闭时无 policy 可等，直接转 IDLE。
          if (!enable_mpc_ || (mrtRosInterface_ && mrtRosInterface_->isPolicyUpdated()))
          {
            soft_pause_state_.store(SoftPauseState::IDLE);
            // 全局同步总开关：仅 IDLE 发布 true（enable 受理时未发，延迟到此）
            std_msgs::Bool msg;
            msg.data = true;
            enableControlStatePub_.publish(msg);
            ROS_INFO("[enable_control] RESUMING -> IDLE, enable=true published");
          }
          else
          {
            soft_pause_transition_cycles_.store(0);  // 顺延一个完整窗口再查
            ROS_WARN("[enable_control] RESUMING 250 帧已到但 MPC policy 未就绪，顺延一个窗口（250 帧）再查");
          }
        }
      }

      // 发布冻结姿态给 RM（reset_to_state_publish_cnt_ 在 RESUMING 首拍设置为 1，
      // 发完即止）
      if (reset_to_state_publish_cnt_ > 0)
      {
        std_msgs::Float64MultiArray msg;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "state";
        msg.layout.dim[0].size = reset_state_to_publish_.size();
        msg.layout.dim[0].stride = reset_state_to_publish_.size();
        msg.data.resize(reset_state_to_publish_.size());
        for (size_t i = 0; i < reset_state_to_publish_.size(); ++i)
        {
          msg.data[i] = reset_state_to_publish_[i];
        }
        resetToStatePub_.publish(msg);
        reset_to_state_publish_cnt_--;
      }
    }

    // 更新期望力插值
    if (desired_force_manager_) {
      desired_force_manager_->update(time);
    }

    // 获取期望接触力（基座坐标系）
    vector_t desired_contact_force = getDesiredContactForce();

    // 将期望力传递给WBC
    auto contact_force_wbc = std::dynamic_pointer_cast<mobile_manipulator::AbsoluteContactForceWbc>(wheel_wbc_);
    if (contact_force_wbc) {
      contact_force_wbc->setDesiredContactForce(desired_contact_force);

      // 更新自适应权重（根据期望力大小动态调整手臂任务权重）
      contact_force_wbc->updateAdaptiveWeights(time);

      vector_t joint_position_error = optimizedState_mrt_limit_.tail(armNum_) -
                                      observation_wheel_.state.tail(armNum_);
      bool enable_force_empty_detact = desired_force_manager_ ?
                                       desired_force_manager_->getEnableForceEmptyDetact() : true;
      contact_force_wbc->updateJointPositionError(joint_position_error, enable_force_empty_detact);

      // 发布 force_disabled_ 状态
      bool force_disabled = contact_force_wbc->isForceDisabled();
      ros_logger_->publishValue("/state_estimate/Arm_Contact_Detection/force_empty", static_cast<double>(force_disabled));

      // 如果检测到挥空，且启用了挥空检测失效功能，清除DesiredForceManager中的期望力，执行一次后重置挥空标志
      // 检查期望力是否已经被清除（避免重复清除导致循环打印日志）
      if (force_disabled && desired_force_manager_ && enable_force_empty_detact) {
          // 检查是否还有期望力（避免重复清除）
          bool has_force = desired_force_manager_->hasDesiredForce("left_hand") ||
                          desired_force_manager_->hasDesiredForce("right_hand");
          if (has_force) {
              desired_force_manager_->clearAllForces();
              contact_force_wbc->resetForceDisabled();
          }
      }
    }

    // ===== 下肢 1/2 号电机（knee/leg）锁定：最终输出处硬覆盖 =====
    // 与增量 IK 复用同一 rosparam，锁定语义 = 钉死到开启瞬间捕获的快照（弧度）。
    // 无论 MPC / 快速模式 / 主控回发哪条路，1、2 号都被硬覆盖，waist 关节不受影响。
    {
      // 每 100ms 实时查询一次 rosparam。必须用 get()（无缓存）而非 getCached()：
      // getCached() 带本地缓存，首次读到"参数不存在"后缓存不会随 IK 节点/终端 set 而刷新，
      // 会一直读到旧的 false，导致锁定永远不生效。
      ros::Time now = ros::Time::now();
      if ((now - lastLockParamCheckTime_).toSec() >= 0.1) {
        lastLockParamCheckTime_ = now;
        bool lockParam = false;
        // 参数可能是 bool(true/false) 也可能是 int(1/0)，两种都要兼容：
        // ros::param::get(name, bool&) 只认 XmlRpc boolean 类型，遇到 int(1) 会返回 false，
        // 导致锁定失效。因此 bool 读取失败时再尝试按 int 读取。
        if (ros::param::get("/ik_ros_uni_cpp_node/quest3/lock_knee_leg", lockParam)) {
          lockKneeLegEnabled_.store(lockParam);
        } else {
          int lockParamInt = 0;
          if (ros::param::get("/ik_ros_uni_cpp_node/quest3/lock_knee_leg", lockParamInt)) {
            lockKneeLegEnabled_.store(lockParamInt != 0);
          }
        }
      }
    }
    if (!wholeBodyIkAuthorityOwned && !fixedPipelineIdleHold && lockKneeLegEnabled_.load()) {
      if (!lockKneeLegCaptured_) {
        // 开启首拍：捕获当前实际关节角作为固定目标（knee=idx0, leg=idx1）
        lockKneeQ_ = observation_wheel_.state[baseDim_ + 0];
        lockLegQ_  = observation_wheel_.state[baseDim_ + 1];
        lockKneeLegCaptured_ = true;
        ROS_INFO("[wheel_absolute_control] knee/leg lock: captured knee=%.4f leg=%.4f",
                 lockKneeQ_, lockLegQ_);
      }
      optimizedState_mrt_limit_[baseDim_ + 0] = lockKneeQ_;
      optimizedState_mrt_limit_[baseDim_ + 1] = lockLegQ_;
      optimizedInput_mrt_limit_[baseDim_ + 0] = 0.0;
      optimizedInput_mrt_limit_[baseDim_ + 1] = 0.0;
    } else {
      lockKneeLegCaptured_ = false;  // 解锁后，下次锁定重新捕获快照
    }

    // WBC 目标：optimizedState_wbc=期望位姿/关节角，optimizedInput_wbc=对应速度，维度见头文件注释
    vector_t optimizedState_wbc = optimizedState_mrt_limit_;
    vector_t optimizedInput_wbc = optimizedInput_mrt_limit_;
    // if (enable_arm_traj_interpolator_ && armNum_ > 0)
    // {
    //   ros_logger_->publishVector("/humanoid_wheel/wbc_arm_target_qpos_smooth", optimizedState_wbc.tail(armNum_));
    //   ros_logger_->publishVector("/humanoid_wheel/wbc_arm_target_qvel_smooth", optimizedInput_wbc.tail(armNum_));
    //   // ROS_INFO_THROTTLE(1.0, "[humanoidControllerWheelWbc] WBC arm task uses interpolated arm target.");
    // }
    ros_logger_->publishVector("/humanoid_wheel/optimizedState_wbc_in", optimizedState_wbc);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_wbc_in", optimizedInput_wbc);
    markStageMs("cmd_prepare");

    // Apply ddq only for joints whose q/dq survived every controller-side
    // limiter unchanged. A modified joint remains feedback-only.
    mobile_manipulator::DynamicsReference dynamicsReference;
    dynamicsReference.q=optimizedState_wbc;
    dynamicsReference.v=optimizedInput_wbc;
    dynamicsReference.acceleration=vector_t::Zero(info.armDim);
    arm_acceleration_feedforward_applied_this_cycle_ = false;
    bool wholeBodyAccelerationFeedforwardApplied = false;
    if (arm_ddq_feedforward_gain_ > 0.0 &&
        wholeBodyIkReferenceApplied &&
        wholeBodyIkReference.ddq.size() ==
            static_cast<Eigen::Index>(info.inputDim) &&
        wholeBodyIkReference.ddq.array().isFinite().all())
    {
      constexpr double kReferenceMatchTolerance = 1.0e-7;
      const bool referenceUnchanged =
          (optimizedState_wbc - wholeBodyIkReference.q).cwiseAbs().maxCoeff() <=
              kReferenceMatchTolerance &&
          (optimizedInput_wbc - wholeBodyIkReference.dq).cwiseAbs().maxCoeff() <=
              kReferenceMatchTolerance;
      if (referenceUnchanged) {
        dynamicsReference.acceleration=arm_ddq_feedforward_gain_ * wholeBodyIkReference.ddq.tail(info.armDim);
        wholeBodyAccelerationFeedforwardApplied = true;
        arm_acceleration_feedforward_applied_this_cycle_ = true;
      }
    }
    // Decouple noisy IK ddq feedforward from the atomic position/velocity path.
    // Only the admitted formal reference can supply acceleration.
    if (!humanoid_controller::scaleArmInertiaFeedforward(
          &dynamicsReference.acceleration,armNum_,arm_inertia_feedforward_scale_)) {
      ROS_ERROR_THROTTLE(1.,"Invalid admitted arm inertia feedforward"); return;
    }
    const auto wbc_apply_time=std::chrono::steady_clock::now();
    dynamicsReference.accelerationValid=arm_acceleration_feedforward_applied_this_cycle_;
    dynamicsReference.wholeBodyAcceleration=wholeBodyAccelerationFeedforwardApplied;
    dynamicsReference.session=wholeBodyIkReference.session_id;
    dynamicsReference.generation=wholeBodyIkReference.generation;
    dynamicsReference.sequence=wholeBodyIkReference.reference_seq;
    dynamicsReference.epoch=wholeBodyIkReference.execution_epoch;
    dynamicsReference.atomicReference=wholeBodyIkReferenceApplied;
    dynamicsReference.accelerationGain=arm_ddq_feedforward_gain_;
    vector_t torque;
    if (!computeDynamics(dynamicsReference, dt, torque)) return;
    // Observation only, after the final atomic reference/soft-pause checks.
    vector_t ik_timing(13);
    const double qdiff=wholeBodyIkReference.q.size()==optimizedState_wbc.size()
        ?(wholeBodyIkReference.q-optimizedState_wbc).cwiseAbs().maxCoeff():-1.0;
    const double vdiff=wholeBodyIkReference.dq.size()==optimizedInput_wbc.size()
        ?(wholeBodyIkReference.dq-optimizedInput_wbc).cwiseAbs().maxCoeff():-1.0;
    ik_timing << 1.0,double(wholeBodyIkReference.session_id),double(wholeBodyIkReference.generation),double(wholeBodyIkReference.reference_seq),
        std::chrono::duration<double,std::nano>(wholeBodyIkReference.receive_stamp.time_since_epoch()).count(),
        std::chrono::duration<double,std::nano>(wbc_apply_time.time_since_epoch()).count(),
        wholeBodyIkReference.publish_ros_stamp,
        (wholeBodyIkReferenceApplied&&qdiff>=0&&qdiff<=1e-7&&vdiff>=0&&vdiff<=1e-7)?1.0:0.0,
        wholeBodyIkAuthorityOwned?1.0:0.0,wholeBodyIkReferenceFresh?1.0:0.0,
        double(wholeBodyIkReference.authority_state),qdiff,vdiff;
    ros_logger_->publishVector("/whole_body_velocity_ik/diagnostics/wbc_applied",ik_timing);
    markStageMs(dynamics_backend_->name());
    ros_logger_->publishVector("/humanoid_wheel/torque", torque);
    ros_logger_->publishVector("/humanoid_wheel/target_qpos", target_qpos);

    // 手臂末端力估计（使用当前时间步的周期）
    if (arm_force_estimator_) {
      arm_force_estimator_->setCmdTorque(torque);
      arm_force_estimator_->update(observation_wheel_.state, observation_wheel_.input, dfd);
    }

    // 更新关节指令
    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    for (int i1 = 0; i1 < lowJointNum_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_wbc.tail(info.armDim)[i1]);
      jointCmdMsg.joint_v.push_back(optimizedInput_wbc.tail(info.armDim)[i1]);
      jointCmdMsg.tau.push_back(torque.head(lowJointNum_)[i1]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i2 = 0; i2 < armNum_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_wbc.tail(armNum_)[i2]);
      jointCmdMsg.joint_v.push_back(optimizedInput_wbc.tail(armNum_)[i2]);
      jointCmdMsg.tau.push_back(torque.tail(armNum_)[i2]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[lowJointNum_ + i2]);
      jointCmdMsg.control_modes.push_back(2);
    }

    // 从控制数据管理器计算头部控制
    if (headNum_ > 0)
    {
      vector_t target_pos = control_data_manager_->getHeadExternalControlState();
      vector_t feedback_tau = control_data_manager_->computeHeadControl(target_pos);

      for (int i3 = 0; i3 < headNum_; ++i3)
      {
        jointCmdMsg.joint_q.push_back(target_pos[i3]);
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(feedback_tau[i3]);
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[lowJointNum_ + armNum_ + i3]);
        jointCmdMsg.control_modes.push_back(2);
        jointCmdMsg.joint_kp.push_back(0);
        jointCmdMsg.joint_kd.push_back(0);
      }

      robotVisualizer_->updateHeadJointPositions(sensors_data_new.jointPos_.tail(headNum_));
    }
    replaceDefaultEcMotorPdoGait(jointCmdMsg);  // 统一修改pdo写入的kpkd

    // 测量 WBC 处理延迟：从共享内存读取手臂轨迹数据到发布 /joint_cmd 的时间
    {
      const ros::Time armTrajRecvTime = control_data_manager_->getArmExternalControlStateTimestamp();
      if (armTrajRecvTime.isValid()) {
        const double wbcProcessingLatencyMs = (ros::Time::now() - armTrajRecvTime).toSec() * 1000.0;
        ros_logger_->publishValue("/vr_incremental/wbc_processing_latency_ms", wbcProcessingLatencyMs);
      }
    }

    // 发布滤波后手臂轨迹（JointState 格式, deg），用于互相关测量相位延迟
    // 对比 /vr_incremental/kuavo_arm_traj_shm（滤波前）与此话题（滤波后）即可得到纯相位延迟
    if (armTrajFilteredPub_) {
        sensor_msgs::JointState filteredMsg;
        filteredMsg.header.stamp = time;
        filteredMsg.position.resize(armNum_);
        filteredMsg.velocity.resize(armNum_);
        filteredMsg.name.resize(armNum_);
        for (int i = 0; i < armNum_; ++i) {
            filteredMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
            filteredMsg.position[i] = optimizedState_mrt_limit_.tail(armNum_)[i] * 180.0 / M_PI;
            filteredMsg.velocity[i] = optimizedInput_mrt_limit_.tail(armNum_)[i] * 180.0 / M_PI;
        }
        armTrajFilteredPub_.publish(filteredMsg);
    }

    const double cmd_publish_begin=kuavo_e2e::ns();
    jointCmdPub_.publish(jointCmdMsg);
    if(commandAudit.enabled()) {kuavo_e2e::Trace<64>::Row row{};row[0]=1;row[1]=jointCmdMsg.header.stamp.toSec();row[2]=cmd_publish_begin;row[3]=kuavo_e2e::ns();row[4]=wholeBodyIkReference.session_id;row[5]=wholeBodyIkReference.generation;row[6]=wholeBodyIkReference.reference_seq;row[7]=wholeBodyAccelerationFeedforwardApplied ? 1.0 : 0.0;row[8]=arm_ddq_feedforward_gain_;row[9]=commandAudit.dropped();for(int j=0;j<18;++j){row[16+j]=wholeBodyIkReference.ddq.size()==21?wholeBodyIkReference.ddq[j+3]:0;row[34+j]=torque[j];}commandAudit.push(row);}


    //更新共享内存中的关节命令
    control_data_manager_->publishJointCmdToShm(jointCmdMsg);

    {
      const double updateCostMs =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - updateWallStart).count();
      // update 本体墙钟耗时（不含 controlLoop 睡眠/调度）
      ros_logger_->publishValue("/monitor/time_cost/wbc", updateCostMs);
      markStageMs("publish");
      // 注意：这里是「两次成功 update 的间隔」，含 sleep/被抢占，不是 WBC 计算耗时。
      // WBC 本体耗时见 /monitor/time_cost/wbc；分段见 /monitor/time_cost/wheel/*。
      static double last_ros_time = time.toSec();
      ros_logger_->publishValue("/monitor/time_cost/controller_loop_time", (time.toSec() - last_ros_time) * 1000.0);
      last_ros_time = time.toSec();
    }

    // The formal owner is the only base command source in absolute mode.
    // Before first acquisition and after release, zero stays zero even if a
    // legacy /cmd_vel publisher is active. Owned braking tuples remain intact.
    geometry_msgs::Twist velCmdMsg;
    if (wholeBodyIkAuthorityOwned && wholeBodyIkReferenceApplied && baseCmdVelStatus_) {
      const Eigen::Vector3d desiredVelBody = cmdVelWorldToBody(
          wholeBodyIkReference.dq.head(3), observation_wheel_.state[2]);
      velCmdMsg.linear.x = desiredVelBody[0];
      velCmdMsg.linear.y = desiredVelBody[1];
      velCmdMsg.angular.z = desiredVelBody[2];
    }
    if(true &&
       baseCmdVelStatus_ == true)
    {
      if (base_cmd_vel_limit_enable_)
      {
        clampBaseCmdVel(velCmdMsg);
      }
      // Lock and stop reach the output immediately; normal commands retain dev throttling.
      velCmdMsg = humanoidController_wheel_wbc::applyMobileBaseLock(velCmdMsg, base_locked_);
      const bool stopped = velCmdMsg.linear.x == 0.0 &&
                           velCmdMsg.linear.y == 0.0 && velCmdMsg.angular.z == 0.0;
      if (base_locked_ || stopped || last_cmd_vel_pub_time_.isZero() ||
          (time - last_cmd_vel_pub_time_).toSec() >= 1.0 / base_cmd_vel_publish_rate_) {
        cmdVelPub_.publish(velCmdMsg);
        last_cmd_vel_pub_time_ = time;
      }
    }else{
        ros::Time current_time = ros::Time::now();
        bool should_reset = false;

        // 立即重置
        if(prev_use_vel_control_ != use_vel_control_)
        {
          should_reset = true;
          ROS_INFO("[vel_control] 检测到速度控制模式切换，重置cmdVel Ruckig规划器");
        }
        // 检测时间间隔：超过设定间隔时重置
        else if((current_time - last_reset_cmd_vel_ruckig_time_).toSec() >= RESET_CMD_VEL_RUCKIG_INTERVAL)
        {
          should_reset = true;
        }

        if(should_reset)
        {
          if(reset_cmd_vel_ruckig_client_.call(reset_cmd_vel_ruckig_srv_))
          {
            if(reset_cmd_vel_ruckig_srv_.response.success)
            {
              last_reset_cmd_vel_ruckig_time_ = current_time;  // 更新重置时间
              // ROS_INFO("[vel_control] Successfully reset cmdVel Ruckig planner: %s", reset_cmd_vel_ruckig_srv_.response.message.c_str());
            }
            else
            {
              ROS_WARN("[vel_control] Failed to reset cmdVel Ruckig planner: %s", reset_cmd_vel_ruckig_srv_.response.message.c_str());
            }
          }
          else
          {
            ROS_WARN("[vel_control] Failed to call reset_cmd_vel_ruckig service");
          }
        }
    }
    // 更新上一次的速度控制状态
    prev_use_vel_control_ = use_vel_control_;
    cnt++;
  }

  humanoidControllerWheelWbc::~humanoidControllerWheelWbc()
  {
    // The nodelet has joined the control loop. Drop subscriptions and worker
    // publishers while their nodelet callback queues still exist.
    control_data_manager_.reset();
    robotVisualizer_.reset();
    mrtRosInterface_.reset();
    delete ros_logger_;
    ros_logger_ = nullptr;
  }

  bool humanoidControllerWheelWbc::setupCpuIsolation()
  {
    // 从ROS参数获取隔离的CPU核心索引
    std::vector<int> isolated_cpus;
    std::vector<int> actually_isolated_cpus;

    // 从全局参数服务器获取隔离CPU列表
    if (ros::param::has("/isolated_cpus")) {
      XmlRpc::XmlRpcValue xml_cpus;
      if (ros::param::get("/isolated_cpus", xml_cpus)) {
        if (xml_cpus.getType() == XmlRpc::XmlRpcValue::TypeArray) {
          for (int i = 0; i < xml_cpus.size(); ++i) {
            try {
              if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                std::cerr << "Error: array element " << i << " is invalid" << std::endl;
                continue;
              }

              double value;
              if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                value = static_cast<int>(xml_cpus[i]);
              } else if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                value = static_cast<double>(xml_cpus[i]);
              } else {
                std::cerr << "Error: array element " << i << " is not a number, type: "
                          << xml_cpus[i].getType() << std::endl;
                continue;
              }

              isolated_cpus.push_back(static_cast<int>(value));
            } catch (const std::exception& e) {
              std::cerr << "Error: parameter conversion failed, index " << i << ": " << e.what()
                        << std::endl;
            }
          }
        } else {
          std::cerr << "Error: isolated_cpus is not an array, type: " << xml_cpus.getType()
                    << std::endl;
        }
      } else {
        std::cerr << "Error: failed to get isolated_cpus parameter" << std::endl;
      }
    } else {
      std::cout << "未设置 /isolated_cpus 参数，跳过CPU亲和性设置" << std::endl;
      return false;
    }

    // 检查是否有隔离的核心
    if (isolated_cpus.size() >= 1) {
      bool ruiwo_isolated_core_ = false;
      int max_cpu = sysconf(_SC_NPROCESSORS_ONLN);
      std::cout << "系统CPU核心数: " << max_cpu << std::endl;

      for (size_t i = 0; i < isolated_cpus.size(); ++i) {
        if (isolated_cpus[i] < 0 || isolated_cpus[i] >= max_cpu) {
          std::cerr << "警告: CPU核心 " << isolated_cpus[i] << " 超出有效范围 [0, " << max_cpu - 1
                    << "]" << std::endl;
          return false;
        }
      }

      // 获取 /proc/cmdline 中的 isolcpus 参数列表
      std::vector<int> isolcpus_list;
      std::ifstream cmdline_file("/proc/cmdline");
      if (cmdline_file.is_open()) {
        std::string line;
        std::getline(cmdline_file, line);
        cmdline_file.close();

        size_t isolcpus_pos = line.find("isolcpus=");
        if (isolcpus_pos != std::string::npos) {
          size_t start = isolcpus_pos + 9;  // "isolcpus=" 长度为9
          size_t end = line.find(' ', start);
          if (end == std::string::npos) end = line.length();

          std::string isolcpus_value = line.substr(start, end - start);

          // 解析 isolcpus 参数 (格式如: "1,3-5,7")
          size_t pos = 0;
          while (pos < isolcpus_value.length()) {
            size_t comma_pos = isolcpus_value.find(',', pos);
            std::string range = isolcpus_value.substr(pos, comma_pos - pos);

            size_t dash_pos = range.find('-');
            if (dash_pos != std::string::npos) {
              int start_cpu = std::stoi(range.substr(0, dash_pos));
              int end_cpu = std::stoi(range.substr(dash_pos + 1));
              for (int j = start_cpu; j <= end_cpu; ++j) {
                isolcpus_list.push_back(j);
              }
            } else {
              isolcpus_list.push_back(std::stoi(range));
            }

            if (comma_pos == std::string::npos) break;
            pos = comma_pos + 1;
          }
          std::cout << "已隔离CPU列表: ";
          for (size_t i = 0; i < isolcpus_list.size(); ++i) {
            if (isolcpus_list[i] == 7) {
              ruiwo_isolated_core_ = true;
            }
            std::cout << isolcpus_list[i];
            if (i < isolcpus_list.size() - 1) std::cout << ", ";
          }
          std::cout << std::endl;
        } else {
          std::cout << "未在 /proc/cmdline 中找到 isolcpus 参数" << std::endl;
        }
      } else {
        std::cerr << "警告: 无法打开 /proc/cmdline 文件" << std::endl;
      }

      for (size_t i = 0; i < isolated_cpus.size(); ++i) {
        int cpu_id = isolated_cpus[i];
        if (std::find(isolcpus_list.begin(), isolcpus_list.end(), cpu_id) != isolcpus_list.end()) {
          actually_isolated_cpus.push_back(cpu_id);
          std::cout << "CPU " << cpu_id << " 已隔离" << std::endl;
        } else {
          std::cout << "CPU " << cpu_id << " 未隔离" << std::endl;
        }
      }
      if (!ruiwo_isolated_core_) {  // 7 号核心未隔离，不允许启动
#if !defined(__aarch64__)
        std::cout << "7 号核心未隔离，跳过CPU亲和性设置" << std::endl;
        return false;
#endif
      }
    } else {
      std::cout << "隔离的核心列表为空，跳过CPU亲和性设置" << std::endl;
      return false;
    }

#if defined(__aarch64__)
    // Orin 降本版：WBC 仅绑定隔离核心 2
    constexpr int kWbcCpuAarch64 = 2;
    if (std::find(actually_isolated_cpus.begin(), actually_isolated_cpus.end(), kWbcCpuAarch64) ==
        actually_isolated_cpus.end()) {
      std::cout << "CPU " << kWbcCpuAarch64 << " 未隔离，跳过CPU亲和性设置" << std::endl;
      return false;
    }
    actually_isolated_cpus.assign(1, kWbcCpuAarch64);
#endif

    // 只有在有真正隔离的CPU时才设置亲和性
#if defined(__aarch64__)
    if (actually_isolated_cpus.size() >= 1) {  // aarch64 WBC 单核
#else
    if (actually_isolated_cpus.size() >= 2) {  // x86 至少需要两个核心绑定 WBC
#endif
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);

      for (size_t i = 0; i < actually_isolated_cpus.size(); ++i) {
        CPU_SET(actually_isolated_cpus[i], &cpuset);
      }

      std::cout << "设置WBC线程亲和性到隔离核心: ";
      for (size_t i = 0; i < actually_isolated_cpus.size(); ++i) {
        std::cout << actually_isolated_cpus[i];
        if (i < actually_isolated_cpus.size() - 1) std::cout << ", ";
      }
      std::cout << std::endl;

      int result = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
      if (result != 0) {
        std::cerr << "警告: 设置线程CPU亲和性失败，错误码: " << result << " (" << strerror(result)
                  << ")" << std::endl;
        return false;
      } else {
        std::cout << "成功设置CPU亲和性到隔离核心" << std::endl;
        return true;
      }
    } else {
#if defined(__aarch64__)
      std::cout << "没有真正隔离的 CPU 核心（aarch64 需核心 2），跳过CPU亲和性设置"
#else
      std::cout << "没有真正隔离的CPU核心或隔离的CPU核心数不足（至少需要2个核心，2个核心绑定WBC控制线程），"
                   "跳过CPU亲和性设置"
#endif
                << std::endl;
      return false;
    }
  }

  void humanoidControllerWheelWbc::setupHumanoidWheelInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile)
  {
    HumanoidWheelInterface_ = std::make_shared<mobile_manipulator::HumanoidWheelInterface>(taskFile, libFolder, urdfFile);
    manipulatorModelInfo_ = HumanoidWheelInterface_->getManipulatorModelInfo();
    pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(HumanoidWheelInterface_->getPinocchioInterface());
    robotVisualizer_ = std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(controllerNh_, *HumanoidWheelInterface_);

    std::cout << "info.stateDim " << manipulatorModelInfo_.stateDim << std::endl;
    std::cout << "info.inputDim " << manipulatorModelInfo_.inputDim << std::endl;
    std::cout << "info.armDim " << manipulatorModelInfo_.armDim << std::endl;
    std::cout << "info.baseFrame " << manipulatorModelInfo_.baseFrame << std::endl;
    std::cout << "info.eeFrame: ";
    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      std::cout << manipulatorModelInfo_.eeFrames[eef_inx] << std::endl;
    }
    std::cout << "info.dofNames " << std::endl;
    for(int i=0; i<manipulatorModelInfo_.dofNames.size(); i++)
    {
      std::cout << manipulatorModelInfo_.dofNames[i] << std::endl;
    }
    std::cout << "info.manipulatorModelType " << static_cast<int>(manipulatorModelInfo_.manipulatorModelType) << std::endl;

  }

  void humanoidControllerWheelWbc::computeObservationFromSensorData(const SensorData& sensorData, const vector6_t& odomData)
  {
    // obs 的顺序：
    // state：世界系x, y里程计(2), 机器人的yaw角度(1)，关节角度(下肢，上肢)(4+7*2)
    // input: forward velocity(1)，turning velocity(1)，关节速度(下肢，上肢)(4+7*2)

    current_time_ = sensorData.timeStamp_;
    if(!observation_initialized_){
      last_time_ = current_time_;
      last_yaw_ = odomData[2]; // 初始时的yaw角度
      accumulated_yaw_ = odomData[2]; // 初始化累积yaw角度
      observation_initialized_ = true;
    }
    double diff_time = (current_time_ - last_time_).toSec();
    ros::Duration period = ros::Duration(diff_time);

    last_time_ = current_time_;

    // 使用angles包进行角度累积计算
    double current_yaw = odomData[2];
    accumulated_yaw_ += angles::shortest_angular_distance(last_yaw_, current_yaw);

    // 更新last_yaw_为当前yaw值
    last_yaw_ = current_yaw;

    Eigen::Vector3d velWorld = cmdVelBodyToWorld(Eigen::Vector3d(odomData[3], odomData[4], odomData[5]),
                                                odomData[2]);
    // ROS_WARN_THROTTLE (1.0, "[computeObservationFromSensorData] odomData: %f %f %f %f %f %f", odomData[0], odomData[1], odomData[2], odomData[3], odomData[4], odomData[5]);
    // ROS_WARN_THROTTLE (1.0, "[computeObservationFromSensorData] sensorData.jointPos_: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", sensorData.jointPos_[0], sensorData.jointPos_[1], sensorData.jointPos_[2], sensorData.jointPos_[3], sensorData.jointPos_[4], sensorData.jointPos_[5], sensorData.jointPos_[6], sensorData.jointPos_[7], sensorData.jointPos_[8], sensorData.jointPos_[9], sensorData.jointPos_[10], sensorData.jointPos_[11], sensorData.jointPos_[12], sensorData.jointPos_[13], sensorData.jointPos_[14], sensorData.jointPos_[15], sensorData.jointPos_[16], sensorData.jointPos_[17]);
    observation_wheel_.state.head(2) = odomData.head(2);
    observation_wheel_.state[2] = accumulated_yaw_;
    observation_wheel_.state.tail(4 + 7*2) = sensorData.jointPos_.head(4 + 7*2);
    observation_wheel_.input[0] = velWorld[0];
    observation_wheel_.input[1] = velWorld[1];
    observation_wheel_.input[2] = velWorld[2];
    observation_wheel_.input.tail(4 + 7*2) = sensorData.jointVel_.head(4 + 7*2);
    observation_wheel_.time += period.toSec();

    // 打印末端估计
    // use pinocchio
    std::vector<Eigen::Vector3d> obs_ee_pos(manipulatorModelInfo_.eeFrames.size());
    std::vector<Eigen::Matrix3d> obs_ee_rot(manipulatorModelInfo_.eeFrames.size());
    getEEPose(observation_wheel_.state, obs_ee_pos, obs_ee_rot);

    vector_t eePoses = vector_t::Zero(manipulatorModelInfo_.eeFrames.size() * 6);
    // 初始化连续欧拉角跟踪器
    static std::vector<ocs2::mobile_manipulator::ContinuousEulerAnglesFromMatrix> eeUnwrappers(manipulatorModelInfo_.eeFrames.size());
    for(int i=0; i<manipulatorModelInfo_.eeFrames.size(); i++)
    {
      eePoses.segment(i * 6, 3) = obs_ee_pos[i];
      eePoses.segment(i * 6 + 3, 3) = eeUnwrappers[i].update(obs_ee_rot[i]);
    }
    ros_logger_->publishVector("/humanoid_wheel/eePoses", eePoses);
    ros_logger_->publishVector("/mobile_manipulator_wbc_observation/state", observation_wheel_.state);
    ros_logger_->publishVector("/mobile_manipulator_wbc_observation/input", observation_wheel_.input);
  }

  void humanoidControllerWheelWbc::setupMrt()
  {
    mrtRosInterface_ = std::make_shared<MRT_ROS_Interface>(robotName_);
    mrtRosInterface_->initRollout(&HumanoidWheelInterface_->getRollout());
    mrtRosInterface_->launchNodes(controllerNh_);
  }

  // 由 MPC state 构建 target-pose 格式的单点目标（FK 现场计算）：
  //   [base(3) + torso_pos(3) + torso_quat(4) + ee_pos(3)*N + ee_quat(4)*N]
  // 不能传 21 维 MPC state —— 基类缓冲被 getTargetTrajectories()（约束层）与
  // publishTargetTrajectories（rviz "command" 帧）按目标姿态格式解析，传 state
  // 会把关节角当四元数（invalid quaternion）并污染 RM 目标缓冲。
  vector_t humanoidControllerWheelWbc::buildTargetPoseFromState(const vector_t& state, bool zeroBase)
  {
    std::vector<Eigen::Vector3d> eePos(manipulatorModelInfo_.eeFrames.size());
    std::vector<Eigen::Matrix3d> eeRot(manipulatorModelInfo_.eeFrames.size());
    getEEPose(state, eePos, eeRot);

    Eigen::Vector3d torsoPos;
    Eigen::Matrix3d torsoRot;
    getTorsoPose(state, torsoPos, torsoRot);

    const int base_nums = static_cast<int>(manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim);
    vector_t target(base_nums + 7 + manipulatorModelInfo_.eeFrames.size() * 7);
    if (zeroBase) target.head(base_nums) = vector_t::Zero(base_nums);
    else          target.head(base_nums) = state.head(base_nums);
    target.segment(base_nums, 3) = torsoPos;
    target.segment(base_nums + 3, 4) = Eigen::Quaternion<scalar_t>(torsoRot).coeffs();
    for (int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      target.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx * 7, 3) = eePos[eef_inx];
      target.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx * 7 + 3, 4) = Eigen::Quaternion<scalar_t>(eeRot[eef_inx]).coeffs();
    }
    return target;
  }

  void humanoidControllerWheelWbc::initMPC()
  {
    SystemObservation initial_observation = observation_wheel_;
    initial_observation.time = 0.0;
    observation_wheel_.time = 0.0;
    // initial_observation.state = initial_state;

    // reset kinemic Limit Filters
    obsStateLimitFilterPtr_->reset(initial_observation.state);
    obsInputLimitFilterPtr_->reset(initial_observation.input);
    mrtStateLimitFilterPtr_->reset(initial_observation.state);
    mrtInputLimitFilterPtr_->reset(initial_observation.input);

    // Fixed IK still needs the observation/filter reset above. No external
    // optimizer is launched in this mode, so never reset or wait for a policy.
    if (!enable_mpc_) {
      ROS_INFO("[FixedVelocityIK] MPC disabled: initial policy wait skipped");
      return;
    }

    // initial command（底盘段置零，启动语义）
    vector_t initTarget = buildTargetPoseFromState(initial_observation.state, /*zeroBase=*/true);

    TargetTrajectories initial_target({initial_observation.time},
                                      {initTarget},
                                      {initial_observation.input});

    // Set the first observation and command and wait for optimization to finish
    ROS_INFO_STREAM("Waiting for the initial policy ...");

    // Reset MPC node
    mrtRosInterface_->resetMpcNode(initial_target);
    std::cout << "reset MPC node\n";

    // Wait for the initial policy
    while (!mrtRosInterface_->initialPolicyReceived() && ros::ok() && ros::master::check())
    {
      mrtRosInterface_->spinMRT();
      mrtRosInterface_->setCurrentObservation(initial_observation);
      ros::Rate(HumanoidWheelInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
  }

  void humanoidControllerWheelWbc::getEEPose(const vector_t& init_q, std::vector<Eigen::Vector3d>& ee_pos, std::vector<Eigen::Matrix3d>& ee_rot)
  {
    if(ee_pos.size() > manipulatorModelInfo_.eeFrames.size())
    {
      throw std::invalid_argument("[getEEPose] ee_pos is out of range.");
    }
    if(ee_rot.size() > manipulatorModelInfo_.eeFrames.size())
    {
      throw std::invalid_argument("[getEEPose] ee_rot is out of range.");
    }
    auto model = pinocchioInterface_ptr_->getModel();
    auto data = pinocchioInterface_ptr_->getData();
    pinocchio::framesForwardKinematics(model, data, init_q);

    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      int ee_id = model.getBodyId(manipulatorModelInfo_.eeFrames[eef_inx]);
      ee_pos[eef_inx] = data.oMf[ee_id].translation();
      ee_rot[eef_inx] = data.oMf[ee_id].rotation();
    }
  }

  void humanoidControllerWheelWbc::getTorsoPose(const vector_t& init_q, Eigen::Vector3d& torso_pos, Eigen::Matrix3d& torso_rot)
  {
    auto model = pinocchioInterface_ptr_->getModel();
    auto data = pinocchioInterface_ptr_->getData();
    pinocchio::framesForwardKinematics(model, data, init_q);

    int torso_id = model.getBodyId(manipulatorModelInfo_.torsoFrame);
    torso_pos = data.oMf[torso_id].translation();
    torso_rot = data.oMf[torso_id].rotation();
  }

  // 对 update() 中从 /sensors_data_raw 读回的关节角做双手末端 FK，并发布 PoseStamped：
  //   /sensors_data_raw/ee_fk/zarm_l7_end_effector
  //   /sensors_data_raw/ee_fk/zarm_r7_end_effector
  // state 为 MPC 观测，布局 [base_x, base_y, base_yaw, 下肢4 + 手臂14]。
  // 位姿以 base_link 为参考坐标系，去掉浮动基底的 x/y/yaw，得到纯关节 FK 结果。
  void humanoidControllerWheelWbc::publishHandEndEffectorFK(const vector_t& state)
  {
    auto model = pinocchioInterface_ptr_->getModel();
    auto data = pinocchioInterface_ptr_->getData();
    pinocchio::framesForwardKinematics(model, data, state.head(model.nq));

    const int baseFrameId = model.getFrameId(manipulatorModelInfo_.baseFrame);
    if (baseFrameId == model.frames.size()) {
      ROS_WARN_THROTTLE(1.0, "[publishHandEndEffectorFK] baseFrame %s not found in model",
                        manipulatorModelInfo_.baseFrame.c_str());
      return;
    }
    const pinocchio::SE3 basePose = data.oMf[baseFrameId];
    const ros::Time stamp = ros::Time::now();

    for (size_t i = 0; i < manipulatorModelInfo_.eeFrames.size(); ++i)
    {
      const auto& eeFrame = manipulatorModelInfo_.eeFrames[i];
      const int eeFrameId = model.getFrameId(eeFrame);
      if (eeFrameId == model.frames.size()) {
        ROS_WARN_THROTTLE(1.0, "[publishHandEndEffectorFK] eeFrame %s not found in model", eeFrame.c_str());
        continue;
      }
      // 末端在 base_link 系下的位姿
      const pinocchio::SE3 eeInBase = basePose.inverse() * data.oMf[eeFrameId];

      geometry_msgs::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = manipulatorModelInfo_.baseFrame;
      msg.pose.position.x = eeInBase.translation()[0];
      msg.pose.position.y = eeInBase.translation()[1];
      msg.pose.position.z = eeInBase.translation()[2];
      const Eigen::Quaterniond q(eeInBase.rotation());
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      handFkPubs_[i].publish(msg);
    }
  }

  // 简化的线性插值函数：生成从当前状态到目标状态的轨迹
  vector_t humanoidControllerWheelWbc::interpolateArmTarget(scalar_t currentTime, const vector_t& currentArmState, const vector_t& newDesiredArmState, scalar_t maxSpeed)
  {
    // 计算状态差和距离
    vector_t deltaState = newDesiredArmState - currentArmState;

    // 计算每个关节的角度变化，取最大角度变化作为总距离
    scalar_t totalDistance = 0.0;
    for(int i = 0; i < armNum_; i++)
    {
      totalDistance = std::max(totalDistance, std::abs(deltaState[i]));
    }

    // 如果距离很小，直接返回目标状态
    if (totalDistance < 1e-6)
    {
      return newDesiredArmState;
    }

    // 使用固定步长插值（弧度）
    scalar_t stepDistance = 0.001;  // 每步移动0.1弧度（约5.7度）

    // 如果一步就能到达，直接返回目标状态
    if (stepDistance >= totalDistance)
    {
      std::cout << "[ArmControl] 一步就能到达，直接返回目标状态" << std::endl;
      return newDesiredArmState;
    }

    // 线性插值：朝目标方向移动一步
    vector_t direction = deltaState / totalDistance;  // 单位方向向量
    vector_t interpolatedState = vector_t::Zero(armNum_);
    for(int i = 0; i < armNum_; i++)
    {
      interpolatedState[i] = currentArmState[i] + direction[i] * stepDistance;
    }
    ros_logger_->publishVector("/humanoid_wheel/direction", direction);
    ros_logger_->publishVector("/humanoid_wheel/interpolatedState", interpolatedState);
    return interpolatedState;
  }

  // 手臂控制模式切换状态管理函数
  void humanoidControllerWheelWbc::publishArmControlModeStatus()
  {
    if (!armControlModeStatusPub_)
    {
      return;
    }

    std_msgs::Float64MultiArray msg;
    const int current_mode = isArmControlModeChanged_
                                 ? prev_arm_trajectory_mode_
                                 : arm_trajectory_mode_;
    msg.data = {
        static_cast<double>(current_mode),
        static_cast<double>(arm_trajectory_mode_),
        isArmControlModeChanged_ ? 1.0 : 0.0,
    };
    armControlModeStatusPub_.publish(msg);
  }

  vector_t humanoidControllerWheelWbc::processArmControlModeSwitch(const ros::Time& time, const vector_t& current_qpos, const vector_t& target_qpos)
  {

    vector_t target_arm_state = vector_t::Zero(armNum_);
    // 根据模式确定目标位置
    switch (arm_trajectory_mode_)
    {
      case 0: // keep pose - 保持当前位置
      {
        isArmControlModeChanged_ = false;  // 保持模式立即完成切换
        target_arm_state = current_qpos;
        // std::cout << "[ArmControl] 切换到保持模式，保持当前位置" << std::endl;
        break;
      }
      case 1: // auto swing - 自动摆臂
      {
        target_arm_state = init_arm_target_qpos_;
        // std::cout << "[ArmControl] 切换到自动摆臂模式" << std::endl;
        break;
      }
      case 2: // external control - 外部轨迹控制
      {
        target_arm_state = target_qpos;
        // std::cout << "[ArmControl] 切换到外部控制模式" << std::endl;
        break;
      }
      default:
      {
        target_arm_state = current_qpos;  // 使用入参current_qpos
        isArmControlModeChanged_ = false;
        std::cout << "[ArmControl] 未知的控制模式: " << arm_trajectory_mode_ << "，使用保持模式" << std::endl;
        break;
      }

    }

    if(!isArmControlModeChanged_)
    {
      publishArmControlModeStatus();
      return target_arm_state;
    }

    // 首次进入模式切换时，记录开始时间
    if (!arm_mode_switch_hold_phase_)
    {
      arm_mode_switch_hold_phase_ = true;
      arm_mode_switch_start_time_ = time.toSec();
      arm_start_pos_ = current_qpos;
      std::cout << "[ArmControl] 开始模式切换，进入200ms保持阶段" << std::endl;
    }

    // 前200ms保持当前位置
    double elapsed_time = time.toSec() - arm_mode_switch_start_time_;
    if (elapsed_time < ARM_MODE_SWITCH_HOLD_DURATION)
    {
      std::cout << "[ArmControl] 保持阶段中，剩余时间: " << (ARM_MODE_SWITCH_HOLD_DURATION - elapsed_time) * 1000 << "ms" << std::endl;
      return current_qpos;  // 返回当前关节位置
    }

    // 处理模式切换插值 - 此函数只在isArmControlModeChanged_为true时调用
    // 执行插值 - 使用入参current_qpos作为当前位置
    ros_logger_->publishVector("/humanoid_wheel/current_qpos", current_qpos);
    ros_logger_->publishVector("/humanoid_wheel/start_pos", arm_start_pos_);
    vector_t interpolated_target = interpolateArmTarget(time.toSec(), arm_start_pos_, target_arm_state, arm_move_spd_);
    // 检查是否到达目标
    scalar_t error = (interpolated_target - target_arm_state).norm();

    // 对于外部控制模式(模式2)，由于目标位置可能持续变化，使用更宽松的收敛条件
    scalar_t convergence_threshold = (arm_trajectory_mode_ == 2) ? 0.1 : 0.05;
    if(error < convergence_threshold)
    {
      isArmControlModeChanged_ = false;
      std::cout << "[ArmControl] 模式切换完成，误差: " << error << std::endl;
      publishArmControlModeStatus();
      return target_arm_state;
    }
    arm_start_pos_ = interpolated_target;
    // std::cout << "[ArmControl] 模式切换中，误差: " << error  << ", 目标: " << target_arm_state.transpose() << ", 插值: " << interpolated_target.transpose() << std::endl;
    return interpolated_target;
  }

  bool humanoidControllerWheelWbc::changeArmCtrlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    // 3791: 软暂停期间拒绝（非 IDLE 含 PAUSING/PAUSED/RESUMING）——暂停中翻模式会在
    // 恢复瞬间被消费并触发保持期过渡，"接受但延迟执行"与 RM 侧"拒绝且无副作用"不一致
    if (soft_pause_state_.load() != SoftPauseState::IDLE)
    {
      res.result = false;
      res.mode = arm_trajectory_mode_;
      res.message = "Rejected: control disabled (soft pause).";
      ROS_WARN("[ArmControl] 软暂停期间拒绝模式切换请求: %d", req.control_mode);
      return true;
    }

    int new_mode = req.control_mode;
    if (fixed_velocity_ik_pipeline_ && new_mode != 0 && new_mode != 2) {
      res.result = false;
      res.mode = arm_trajectory_mode_;
      res.message = "Use /quest3/pinocchio_arm_operation for PREPARE; legacy mode-1 writer is disabled";
      return true;
    }

    // 检查模式是否发生变化
    if (new_mode != arm_trajectory_mode_)
    {
      prev_arm_trajectory_mode_ = arm_trajectory_mode_;
      arm_trajectory_mode_ = new_mode;
      // These are consumer admission/hold modes, not legacy interpolation.
      isArmControlModeChanged_ = !fixed_velocity_ik_pipeline_;
      arm_mode_switch_hold_phase_ = false;
      std::cout << "[ArmControl] 模式切换: " << prev_arm_trajectory_mode_ << " -> " << arm_trajectory_mode_ << std::endl;
    }

    use_arm_trajectory_control_ = !fixed_velocity_ik_pipeline_ && (2 == arm_trajectory_mode_);
    if (fixed_velocity_ik_pipeline_) publishArmControlModeStatus();

    res.result = true;
    res.mode = arm_trajectory_mode_;
    res.message = "success change arm ctrl mode";
    return true;
  }

  bool humanoidControllerWheelWbc::enableLbArmQuickModeCallback(kuavo_msgs::changeLbQuickModeSrv::Request &req,
                                                                kuavo_msgs::changeLbQuickModeSrv::Response &res)
  {
    if (fixed_velocity_ik_pipeline_) {
      res.success = false;
      res.message = "LEGACY_NORMAL_QUICK_SWITCH=DISABLED; IK_PIPELINE=PINOCCHIO_VELOCITY";
      return true;
    }

    std::cout << "[ArmControl] 快速模式切换请求, 请求模式为: " << int(req.quickMode) << std::endl;
    quickMode_ = req.quickMode;
    res.success = true;
    res.message = "success change quick ctrl mode to " + std::to_string(req.quickMode);
    return true;
  }

  bool humanoidControllerWheelWbc::changeLbObsUpdateModeCallback(kuavo_msgs::changeLbMpcObsUpdateModeSrv::Request &req,
                                                                kuavo_msgs::changeLbMpcObsUpdateModeSrv::Response &res)
  {
    if (fixed_velocity_ik_pipeline_) {
      res.success = false;
      res.message = "Legacy MPC observation mode disabled; absolute control uses measured feedback";
      return true;
    }
    std::cout << "[ArmControl] mpc反馈机制切换请求: " << int(req.obsUpdateMode) << std::endl;
    mpcObsUpdateMode_ = req.obsUpdateMode;
    res.success = true;
    res.message = "success change obs update mode to " + std::to_string(req.obsUpdateMode);
    return true;
  }

  bool humanoidControllerWheelWbc::enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    if (fixed_velocity_ik_pipeline_) {
      res.result = false;
      res.mode = arm_trajectory_mode_;
      res.message = "Legacy position trajectory writer disabled; use Pinocchio operation service";
      return true;
    }

    if(req.control_mode == arm_trajectory_mode_)
    {
      std::cout << "[ArmControl] 轨迹控制模式已经为: " << (use_arm_trajectory_control_ ? "启用" : "禁用") << std::endl;
      return true;
    }
    arm_trajectory_mode_ = req.control_mode;
    use_arm_trajectory_control_ = (2 == arm_trajectory_mode_);

    // 记录模式切换时的状态
    if (use_arm_trajectory_control_)
    {
      // 将当前状态更新到控制数据管理器中
      control_data_manager_->updateArmExternalControlState(observation_wheel_.state.segment(observation_wheel_.state.size() - armNum_, armNum_), vector_t::Zero(armNum_), vector_t::Zero(armNum_));
    }

    if(1 == arm_trajectory_mode_)
    {
      arm_mode_switch_hold_phase_ = false;
      isArmControlModeChanged_ = true;
    }

    std::cout << "[ArmControl] 轨迹控制模式已切换: " << (use_arm_trajectory_control_ ? "启用" : "禁用") << std::endl;

    res.result = true;
    res.mode = arm_trajectory_mode_;
    res.message = "success change arm ctrl mode";
    return true;
  }

  vector_t humanoidControllerWheelWbc::smoothTransition(const vector_t& current_pos, const vector_t& target_pos, double transition_duration)
  {
    // 从控制数据管理器获取全身控制模式
    bool whole_torso_ctrl = control_data_manager_->getWholeTorsoCtrl();
    // 在全身控制模式下
    if (whole_torso_ctrl)
    {
      prev_whole_torso_ctrl_ = true;
      return target_pos;         // 在全身控制时返回目标位置，保持稳定
    }

    constexpr double kTransitionTime = 2.0;    // 2秒过渡时间
    // 如果不是从全身控制模式切换出来，且不在过渡中，直接返回目标位置
    if (!prev_whole_torso_ctrl_ && !is_transitioning_)
    {
      return target_pos;
    }

    // 检测从全身控制模式切换出来的时刻
    if (prev_whole_torso_ctrl_)
    {
      prev_whole_torso_ctrl_ = false;
      is_transitioning_ = true;
      transition_start_time_ = current_time_.toSec();
      waist_transition_start_pos_ = current_pos;  // 使用当前位置作为腰部过渡起点
      std::cout << "开始过渡: 从 " << current_pos.transpose() << " 到 " << target_pos.transpose() << std::endl;
    }

    // 如果没在过渡中，直接返回目标位置
    if (!is_transitioning_)
    {
      return target_pos;
    }

    // 计算过渡进度
    double current_time = current_time_.toSec();
    double elapsed_time = current_time - transition_start_time_;
    double progress = std::min(elapsed_time / kTransitionTime, 1.0);

    // 如果过渡完成
    if (progress >= 1.0)
    {
      is_transitioning_ = false;
      std::cout << "过渡完成: 到达目标位置 " << target_pos.transpose() << std::endl;
      // 重置VR躯干相对位姿缓存
      control_data_manager_->resetVrTorsoPose();
      return target_pos;
    }

    // 线性插值计算当前位置
    vector_t result = vector_t::Zero(4);
    for(int i = 0; i < 4; ++i)
    {
      result[i] = waist_transition_start_pos_[i] + progress * (target_pos[i] - waist_transition_start_pos_[i]);
    }

    // 打印过渡进度
    if (static_cast<int>(progress * 10) % 2 == 0)  // 每20%打印一次
    {
      std::cout << "过渡进度: " << (progress * 100) << "%, 当前位置: " << result.transpose() << std::endl;
    }

    return result;

  }

  bool humanoidControllerWheelWbc::handleWaistIkService(kuavo_msgs::lbBaseLinkPoseCmdSrv::Request &req, kuavo_msgs::lbBaseLinkPoseCmdSrv::Response &res)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // 获取当前base_link位姿
    vector_t base_pose =createZeroPose();
    if(req.with_chassis)
    {
        // z轴偏移量（真实机器人和仿真不同）
        auto z_offset = is_real_ ? 0.185 : 0.0;

        // 位置: [x, y, z]
        base_pose[0] = req.chassis_info[0];  // x
        base_pose[1] = req.chassis_info[1];  // y
        base_pose[2] = z_offset;             // z（底盘到base_link的高度偏移）

        // 四元数: 从yaw角转换，使用Eigen::AngleAxisd (标准方法)
        double yaw = req.chassis_info[2];
        Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        base_pose[3] = quat.w();  // qw
        base_pose[4] = quat.x();  // qx
        base_pose[5] = quat.y();  // qy
        base_pose[6] = quat.z();  // qz
    }
    else
    {
        // 从控制数据管理器获取当前base_link位姿
        if (!control_data_manager_->getRealtimeBaseLinkPose(base_pose))
        {
            ROS_WARN("[handleWaistIkService] Failed to get base_link pose, using default");
            base_pose[2] = 0.185;  // 设置z轴高度
        }
        // ROS_INFO("Using TF base_link pose: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
        //          base_pose[0], base_pose[1], base_pose[2], base_pose[3], base_pose[4], base_pose[5], base_pose[6]);
    }

    if(base_pose.size() != 7) {
        ROS_ERROR("Base pose size incorrect: %ld", base_pose.size());
        res.success = false;
        res.time_cost = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();
        return true;
    }

    // 构建目标位姿向量
    vector_t target_pose = createZeroPose();
    for(int i = 0; i < 7; ++i) {
        target_pose[i] = req.base_link[i];
    }

    // 获取当前关节角度
    SensorData sensors_data_new;
    if (!control_data_manager_->getRealtimeSensorData(sensors_data_new))
    {
      ROS_ERROR("[handleWaistIkService] Failed to get sensor data");
      return false;
    }
    vector_t current_waist_joints = sensors_data_new.jointPos_.head(lowJointNum_);

    // 计算IK
    auto ik_result = waistKinematics_->computeFastWaistInverseKinematics(base_pose, target_pose, current_waist_joints, false);

    // 填充响应
    res.success = !ik_result.isZero();  // 如果结果是全0向量，说明逆解失败
    for(int i = 0; i < 4; ++i) {
        res.lb_leg[i] = ik_result[i];
    }

    res.time_cost = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();

    ROS_INFO("Waist IK service called: with_chassis=%d, success=%d, time_cost=%.2f ms",
             req.with_chassis, res.success, res.time_cost);

    return true;
  }

  void humanoidControllerWheelWbc::clampBaseCmdVel(geometry_msgs::Twist& cmd) const
  {
    cmd.linear.x = std::max(base_cmd_vel_min_[0], std::min(base_cmd_vel_max_[0], cmd.linear.x));
    cmd.linear.y = std::max(base_cmd_vel_min_[1], std::min(base_cmd_vel_max_[1], cmd.linear.y));
    cmd.angular.z = std::max(base_cmd_vel_min_[2], std::min(base_cmd_vel_max_[2], cmd.angular.z));
  }

  Eigen::Vector3d humanoidControllerWheelWbc::cmdVelWorldToBody(const Eigen::Vector3d& cmd_vel_world, double yaw)
  {
    Eigen::Matrix3d R_world_to_body;
    R_world_to_body << std::cos(-yaw), -std::sin(-yaw), 0,
                       std::sin(-yaw),  std::cos(-yaw), 0,
                       0,               0,              1;
    return R_world_to_body * cmd_vel_world;
  }

  Eigen::Vector3d humanoidControllerWheelWbc::cmdVelBodyToWorld(const Eigen::Vector3d& cmd_vel_body, double yaw)
  {
    Eigen::Matrix3d R_body_to_world;
    R_body_to_world << std::cos(yaw), -std::sin(yaw), 0,
                       std::sin(yaw),  std::cos(yaw), 0,
                       0,               0,              1;
    return R_body_to_world * cmd_vel_body;
  }

  void humanoidControllerWheelWbc::replaceDefaultEcMotorPdoGait(kuavo_msgs::jointCmd& jointCmdMsg)
  {
    // 对于 control_modes == 2 的电机：
    //   EC_MASTER 电机：useVrArmKpKd=true 且 vr_joint_* 有效时使用 vr_joint_kp/kd，否则使用 joint_kp/kd
    //   RUIWO 电机：useVrArmKpKd=true 且 vr_ruiwo_* 有效时使用 vr_ruiwo_kp/kd，否则使用 ruiwo_kp/kd
    // 注意：ec_master_count/ruiwo_count 对应各自驱动器数组中的索引
    const auto &hardware_settings = kuavo_settings_.hardware_settings;
    const auto &running_settings = kuavo_settings_.running_settings;
    const int total_joints = lowJointNum_ + armNum_ + headNum_;

    const bool use_vr_ec = running_settings.use_vr_arm_kpkd &&
                           !running_settings.vr_joint_kp.empty() &&
                           !running_settings.vr_joint_kd.empty() &&
                           running_settings.vr_joint_kp.size() == running_settings.vr_joint_kd.size();
    const bool use_vr_ruiwo = running_settings.use_vr_arm_kpkd &&
                              !running_settings.vr_ruiwo_kp.empty() &&
                              !running_settings.vr_ruiwo_kd.empty() &&
                              running_settings.vr_ruiwo_kp.size() == running_settings.vr_ruiwo_kd.size();

    const auto &selected_ec_kp = use_vr_ec ? running_settings.vr_joint_kp : running_settings.joint_kp;
    const auto &selected_ec_kd = use_vr_ec ? running_settings.vr_joint_kd : running_settings.joint_kd;
    const auto &selected_ruiwo_kp = use_vr_ruiwo ? running_settings.vr_ruiwo_kp : running_settings.ruiwo_kp;
    const auto &selected_ruiwo_kd = use_vr_ruiwo ? running_settings.vr_ruiwo_kd : running_settings.ruiwo_kd;

    // 替换 EC_MASTER 电机 kp/kd
    if (!selected_ec_kp.empty() &&
        !selected_ec_kd.empty() &&
        selected_ec_kp.size() == selected_ec_kd.size())
    {
      const int ec_master_size = static_cast<int>(selected_ec_kp.size());
      int ec_master_count = 0;

      for (int i = 0; i < total_joints && i < static_cast<int>(jointCmdMsg.control_modes.size()); ++i)
      {
        // 检查是否为 EC_MASTER 驱动器
        if (i < static_cast<int>(hardware_settings.driver.size()) &&
            hardware_settings.driver[i] == EC_MASTER)
        {
          // 只有当 control_modes == 2 时才更新 joint_kp 和 joint_kd
          if (jointCmdMsg.control_modes[i] == 2 &&
              ec_master_count < ec_master_size)
          {
            jointCmdMsg.joint_kp[i] = static_cast<double>(selected_ec_kp[ec_master_count]);
            jointCmdMsg.joint_kd[i] = static_cast<double>(selected_ec_kd[ec_master_count]);
          }
          // 无论 control_modes 是 0 还是 2，都要递增 ec_master_count
          // 因为 selected_ec_kp/kd 的索引对应所有 EC_MASTER 驱动器
          ec_master_count++;
        }
      }
    }

    // 替换 RUIWO 电机 kp/kd（手臂默认增益）
    if (!selected_ruiwo_kp.empty() &&
        !selected_ruiwo_kd.empty() &&
        selected_ruiwo_kp.size() == selected_ruiwo_kd.size())
    {
      const int ruiwo_size = static_cast<int>(selected_ruiwo_kp.size());
      int ruiwo_count = 0;

      for (int i = 0; i < total_joints && i < static_cast<int>(jointCmdMsg.control_modes.size()); ++i)
      {
        if (i < static_cast<int>(hardware_settings.driver.size()) &&
            hardware_settings.driver[i] == RUIWO)
        {
          if (jointCmdMsg.control_modes[i] == 2 &&
              ruiwo_count < ruiwo_size)
          {
            jointCmdMsg.joint_kp[i] = static_cast<double>(selected_ruiwo_kp[ruiwo_count]);
            jointCmdMsg.joint_kd[i] = static_cast<double>(selected_ruiwo_kd[ruiwo_count]);
          }
          ruiwo_count++;
        }
      }
    }
  }
  vector_t humanoidControllerWheelWbc::getDesiredContactForce()
  {
    vector_t desired_force = vector_t::Zero(12); // 12维，双臂各6维
    if (!desired_force_manager_) return desired_force;

    // 获取左臂期望力
    if (desired_force_manager_->hasDesiredForce("left_hand")) {
      DesiredForceManager::Vector6d left_force = desired_force_manager_->getDesiredForce("left_hand");
      desired_force.segment<6>(0) = left_force;
    }

    // 获取右臂期望力
    if (desired_force_manager_->hasDesiredForce("right_hand")) {
      DesiredForceManager::Vector6d right_force = desired_force_manager_->getDesiredForce("right_hand");
      desired_force.segment<6>(6) = right_force;
    }

    return desired_force;
  }

  void humanoidControllerWheelWbc::initialPreTargetActions(const vector_t& startActions, const vector_t& preTargetActions, double desiredTime)
  {
    // Check if startActions has the correct dimension
    if (startActions.size() != manipulatorModelInfo_.armDim)
    {
      throw std::invalid_argument("startActions dimension mismatch: expected " +
                                   std::to_string(manipulatorModelInfo_.armDim) +
                                   ", got " + std::to_string(startActions.size()));
    }
    // Check if preTargetActions has the correct dimension
    if (preTargetActions.size() != manipulatorModelInfo_.armDim)
    {
      throw std::invalid_argument("preTargetActions dimension mismatch: expected " +
                                   std::to_string(manipulatorModelInfo_.armDim) +
                                   ", got " + std::to_string(preTargetActions.size()));
    }

    startActions_ = vector_t::Zero(manipulatorModelInfo_.stateDim);
    startActions_.tail(manipulatorModelInfo_.armDim) = startActions;
    preTargetActions_ = vector_t::Zero(manipulatorModelInfo_.stateDim);
    preTargetActions_.tail(manipulatorModelInfo_.armDim) = preTargetActions;
    robotPreActionDesiredTime_ = desiredTime;
  }

  void humanoidControllerWheelWbc::performSimpleActions(const ros::Time &time)
  {
    static vector_t startState = startActions_;
    static vector_t startInput = vector_t::Zero(manipulatorModelInfo_.stateDim);
    static vector_t targetState = preTargetActions_;
    static double startTime = time.toSec();
    static double midTargetTime = startTime + robotPreActionDesiredTime_/2;
    static double endTime = startTime + robotPreActionDesiredTime_;

    scalar_array_t timeTrajectory;
    timeTrajectory.push_back(startTime);
    timeTrajectory.push_back(midTargetTime);
    timeTrajectory.push_back(endTime);
    vector_array_t stateTrajectory;
    stateTrajectory.push_back(startState);
    stateTrajectory.push_back(targetState);
    stateTrajectory.push_back(startState);

    vector_t curTargetState_wbc = LinearInterpolation::interpolate(time.toSec(), timeTrajectory, stateTrajectory);
    static vector_t lastTargetState_wbc = curTargetState_wbc;
    ros_logger_->publishVector("/humanoid_wheel/curTargetState_wbc", curTargetState_wbc);

    vector_t inputVelocity = (curTargetState_wbc - lastTargetState_wbc) / dt_;

    mobile_manipulator::DynamicsReference reference;
    reference.q=curTargetState_wbc;
    reference.v=inputVelocity;
    // Legacy preparation is piecewise linear, with no trustworthy ddq.
    // Do not differentiate its corner or invent acceleration feedforward.
    reference.acceleration=vector_t::Zero(manipulatorModelInfo_.armDim);
    reference.accelerationGain=arm_ddq_feedforward_gain_;
    vector_t torque;
    if (!computeDynamics(reference, dt_, torque)) return;

    lastTargetState_wbc = curTargetState_wbc;

    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    for (int i1 = 0; i1 < lowJointNum_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(curTargetState_wbc.tail(manipulatorModelInfo_.armDim)[i1]);
      jointCmdMsg.joint_v.push_back(0.0);
      jointCmdMsg.tau.push_back(torque.head(lowJointNum_)[i1]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i2 = 0; i2 < armNum_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(curTargetState_wbc.tail(armNum_)[i2]);
      jointCmdMsg.joint_v.push_back(0.0);
      jointCmdMsg.tau.push_back(torque.tail(armNum_)[i2]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[lowJointNum_ + i2]);
      jointCmdMsg.control_modes.push_back(2);
    }

    // 从控制数据管理器计算头部控制（内部自动获取传感器数据）
    if (headNum_ > 0)
    {
      vector_t target_pos = control_data_manager_->getHeadExternalControlState();
      vector_t feedback_tau = control_data_manager_->computeHeadControl(target_pos);

      for (int i3 = 0; i3 < headNum_; ++i3)
      {
        jointCmdMsg.joint_q.push_back(target_pos[i3]);
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(feedback_tau[i3]);
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[lowJointNum_ + armNum_ + i3]);
        jointCmdMsg.control_modes.push_back(2);
        jointCmdMsg.joint_kp.push_back(0);
        jointCmdMsg.joint_kd.push_back(0);
      }
    }

    replaceDefaultEcMotorPdoGait(jointCmdMsg);  // 统一修改pdo写入的kpkd
    jointCmdPub_.publish(jointCmdMsg);
  }

  // 用于更新指令和反馈的关键笛卡尔位姿的误差
  void humanoidControllerWheelWbc::computeErrorMultiEeFromTargetAndData(const vector_t& targetState,
                                                                        const vector_t& currentState)
  {
    std::cout << std::fixed << std::setprecision(4);  // 打印4位小数点

    const auto& model = pinocchioInterface_ptr_->getModel();
    auto& data = pinocchioInterface_ptr_->getData();
    auto& info = manipulatorModelInfo_;

    // 存储当前状态和目标状态的位姿
    pinocchio::SE3 currentTorsoPose, targetTorsoPose;
    std::map<std::string, pinocchio::SE3> currentEePoses, targetEePoses;

    // 1. 先计算当前状态，保存位姿
    pinocchio::forwardKinematics(model, data, currentState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    const int torsoFrameId = model.getFrameId(info.torsoFrame);
    if (torsoFrameId != model.frames.size()) {
      currentTorsoPose = data.oMf[torsoFrameId];
    }

    for (const auto& eeFrame : info.eeFrames) {
      const int eeFrameId = model.getFrameId(eeFrame);
      if (eeFrameId != model.frames.size()) {
        currentEePoses[eeFrame] = data.oMf[eeFrameId];
      }
    }

    // 2. 再计算目标状态，保存位姿
    pinocchio::forwardKinematics(model, data, targetState.head(model.nq));
    pinocchio::updateFramePlacements(model, data);

    if (torsoFrameId != model.frames.size()) {
      targetTorsoPose = data.oMf[torsoFrameId];
    }

    for (const auto& eeFrame : info.eeFrames) {
      const int eeFrameId = model.getFrameId(eeFrame);
      if (eeFrameId != model.frames.size()) {
        targetEePoses[eeFrame] = data.oMf[eeFrameId];
      }
    }

    // 辅助函数：轴角转ZYX欧拉角，输出顺序 [yaw, pitch, roll]
    auto axisAngleToEulerZYX = [](const Eigen::Vector3d& axisAngle) -> Eigen::Vector3d {
      double theta = axisAngle.norm();
      if (theta < 1e-6) return Eigen::Vector3d::Zero();
      Eigen::Matrix3d R = Eigen::AngleAxisd(theta, axisAngle/theta).toRotationMatrix();
      // ZYX欧拉角: yaw (z), pitch (y), roll (x)
      double yaw = std::atan2(R(1,0), R(0,0));
      double pitch = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
      double roll = std::atan2(R(2,1), R(2,2));
      return Eigen::Vector3d(yaw, pitch, roll);
    };

    // 3. 计算torso的误差（直接计算）
    Eigen::Vector3d torsoPosError = (targetTorsoPose.translation() - currentTorsoPose.translation());
    Eigen::Vector3d torsoRotError = pinocchio::log3(targetTorsoPose.rotation() * currentTorsoPose.rotation().transpose());
    Eigen::Vector3d torsoRotErrorEulerZYX = axisAngleToEulerZYX(torsoRotError);

    vector_t torsoError6d = vector_t::Zero(6);
    torsoError6d << torsoPosError, torsoRotErrorEulerZYX;
    ros_logger_->publishVector("/humanoid_wheel/" + info.torsoFrame + "_error_6d", torsoError6d);

    // 4. 计算所有末端执行器的误差（直接计算）
    for (const auto& eeFrame : info.eeFrames) {
      auto itCurrent = currentEePoses.find(eeFrame);
      auto itTarget = targetEePoses.find(eeFrame);

      if (itCurrent != currentEePoses.end() && itTarget != targetEePoses.end()) {
        Eigen::Vector3d posError = (itTarget->second.translation() - itCurrent->second.translation());
        Eigen::Vector3d rotError = pinocchio::log3(itTarget->second.rotation() * itCurrent->second.rotation().transpose());
        Eigen::Vector3d rotErrorEulerZYX = axisAngleToEulerZYX(rotError);

        vector_t eeError6d = vector_t::Zero(6);
        eeError6d << posError, rotErrorEulerZYX;
        ros_logger_->publishVector("/humanoid_wheel/" + eeFrame + "_error_6d", eeError6d);
      }
    }
  }

  void humanoidControllerWheelWbc::baseCmdVelStatusCallback(const leju_mobile_base_msgs::BaseCmdVelStatus::ConstPtr& msg)
  {
    if (!enable_base_emergency_stop_)
    {
      return;
    }

    if (msg->reason == leju_mobile_base_msgs::BaseCmdVelStatus::REASON_DISABLED ||
        msg->reason == leju_mobile_base_msgs::BaseCmdVelStatus::REASON_OTHER)
    {
      if (!base_emergency_triggered_.load())
      {
        ROS_ERROR("[humanoidControllerWheelWbc] Base velocity command ineffective! reason=%d, description=%s",
                  msg->reason, msg->description.c_str());
        std::cerr << "[EMERGENCY STOP] Base velocity command ineffective! reason="
                  << static_cast<int>(msg->reason) << ", description=" << msg->description << std::endl;

        base_emergency_triggered_.store(true);
      }
    }
    else
    {
      base_emergency_triggered_.store(false);
    }

    if(msg->cmd_vel_effective != baseCmdVelStatus_)   // 仅当命令生效时才更新状态
    {
      ROS_INFO_STREAM("[baseCmdVelStatusCallback] baseCmdVelStatus:  [ " << (msg->cmd_vel_effective ? "true" : "false") << " ]");
      baseCmdVelStatus_ = msg->cmd_vel_effective;
    }
  }

  void humanoidControllerWheelWbc::stopDynamics(const char* reason)
  {
    dynamics_fault_=true;
    ROS_ERROR_THROTTLE(1.0,"Dynamics fault (restart required): %s",reason);
    // Non-blocking request through the existing safety channel. Never publish
    // a zero torque replacement or switch to another backend on a fault.
    std_msgs::Bool stop; stop.data=true; stopRobotPub_.publish(stop);
  }

  bool humanoidControllerWheelWbc::checkDynamicsSensor(const SensorData& sensor, bool available)
  {
    if(available) dynamics_sensor_receive_=sensor.receiveStamp_;
    if(dynamics_backend_name_!="rnea") return true;  // WBC baseline unchanged.
    if(!available) {stopDynamics("missing sensor data");return false;}
    const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-dynamics_sensor_receive_).count();
    if(dynamics_sensor_receive_==std::chrono::steady_clock::time_point{} || !std::isfinite(age) || age < 0 || age>dynamics_sensor_timeout_ ||
       sensor.jointPos_.size()<static_cast<Eigen::Index>(manipulatorModelInfo_.armDim) ||
       sensor.jointVel_.size()<static_cast<Eigen::Index>(manipulatorModelInfo_.armDim) ||
       !sensor.jointPos_.allFinite() || !sensor.jointVel_.allFinite()) {
      stopDynamics("invalid/stale sensor data");return false;
    }
    return true;
  }

  bool humanoidControllerWheelWbc::computeDynamics(const mobile_manipulator::DynamicsReference& reference,
                                                  double dt, vector_t& torque)
  {
    if(dynamics_fault_) return false;
    const auto begin=std::chrono::steady_clock::now();
    const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-dynamics_sensor_receive_).count();
    mobile_manipulator::DynamicsResult result;
    try { result=dynamics_backend_->update(reference,observation_wheel_,dt,age); }
    catch(const std::exception& e) {stopDynamics(e.what());return false;}
    const double milliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    ros_logger_->publishValue("/dynamics/backend_id",dynamics_backend_name_=="wbc"?0.0:1.0);
    ros_logger_->publishValue("/dynamics/compute_ms",milliseconds);
    if(!result.valid || result.torque.size()!=static_cast<Eigen::Index>(manipulatorModelInfo_.armDim) || !result.torque.allFinite()) {
      stopDynamics(result.reason);return false;
    }
    torque=result.torque;
    if(result.solvedAcceleration.size()==static_cast<Eigen::Index>(manipulatorModelInfo_.stateDim)) {
      ros_logger_->publishVector("/humanoid_wheel/bodyAcc",result.solvedAcceleration.head(baseDim_));
      ros_logger_->publishVector("/humanoid_wheel/jointAcc",result.solvedAcceleration.tail(manipulatorModelInfo_.armDim));
    }
    ros_logger_->publishVector("/dynamics/raw_torque",result.rawTorque);
    ros_logger_->publishVector("/dynamics/applied_torque",result.torque);
    ros_logger_->publishVector("/dynamics/admitted_acceleration",reference.acceleration);
    ros_logger_->publishValue("/dynamics/acceleration_valid",reference.accelerationValid?1.0:0.0);
    ros_logger_->publishValue("/dynamics/torque_limited",result.torqueLimited?1.0:0.0);
    ros_logger_->publishValue("/dynamics/rate_limited",result.rateLimited?1.0:0.0);
    ros_logger_->publishValue("/dynamics/saturation_seconds",result.saturationSeconds);
    if(result.saturationSeconds>0.1)
      ROS_WARN_THROTTLE(1.0,"RNEA feedforward torque saturation persists; check model, payload and trajectory");
    if(shadow_rnea_) {
      try {
        const auto shadow=shadow_rnea_->update(reference,observation_wheel_,dt,age);
        ros_logger_->publishValue("/dynamics/shadow_valid",shadow.valid?1.0:0.0);
        if(shadow.valid) {
          ros_logger_->publishVector("/dynamics/shadow_rnea_torque",shadow.torque);
          ros_logger_->publishVector("/dynamics/shadow_difference",shadow.torque-result.torque);
        }
      } catch(const std::exception& e) {
        ROS_WARN_THROTTLE(1.0,"RNEA shadow error (no actuator authority): %s",e.what());
        ros_logger_->publishValue("/dynamics/shadow_valid",0.0);
      }
    }
    return true;
  }

  void humanoidControllerWheelWbc::publishStopRobot()
  {
    ROS_ERROR("[humanoidControllerWheelWbc] Publishing stop_robot messages (5 times)");
    for (int i = 0; i < 5; ++i)
    {
      std_msgs::Bool msg;
      msg.data = true;
      stopRobotPub_.publish(msg);
      ros::Duration(0.1).sleep();
    }
    ROS_ERROR("[humanoidControllerWheelWbc] stop_robot messages published");
  }

} // namespace wheel_absolute_control
