#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include "humanoid_controllers/humanoidController_wheel_wbc.h"
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
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <leju_mobile_base_msgs/BaseCmdVelStatus.h>

#include "humanoid_wheel_interface/estimators/ContinuousEulerAnglesFromMatrix.h"


namespace humanoidController_wheel_wbc
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

    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_r_q", config.kalmanMeasurementQNoise);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_r_dq", config.kalmanMeasurementDqNoise);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_p0_pos", config.kalmanInitialPosVar);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.kalman_p0_vel", config.kalmanInitialVelVar);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.fast_update_r_scale", config.fastUpdateRScale);
    loadOptionalTaskParam(taskFile, "armTrajInterpKinematicLimit.target_v_alpha", config.targetVAlpha);

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

  static void callSimStartSrv(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 等待服务可用
    ROS_WARN_THROTTLE(1.0, "[callSimStartSrv] Waiting for sim_start service...");
    bool service_available = ros::service::waitForService("sim_start", ros::Duration(100.0)); // 5秒超时

    if (service_available)
    {
      ros::ServiceClient sim_start_client = nh_.serviceClient<std_srvs::SetBool>("sim_start");
      if (sim_start_client.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("sim_start Service call succeeded with message: %s", srv.response.message.c_str());
          return;
        }
        else
        {
          ROS_ERROR("sim_start Service call failed");
        }
      }
      else
      {
        ROS_ERROR("Failed to call sim_start service");
      }
    }
    else
    {
      ROS_ERROR("sim_start Service not available");
    }
    exit(1);
  }

  bool humanoidControllerWheelWbc::init(ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    std::cout << "humanoidControllerWheelWbc init..." << std::endl;
    controllerNh_ = controller_nh;
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

    bool hasQibeng = false;
    for (const auto& eef_type : kuavo_settings_.hardware_settings.end_effector_type) {
      if (eef_type == EndEffectorType::qibeng) {
        hasQibeng = true;
        break;
      }
    }
    /************** Initialize WBC **********************/
    wheel_wbc_ = std::make_shared<mobile_manipulator::ContactForceWbc>(*pinocchioInterface_ptr_, manipulatorModelInfo_);
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
    /****************************************************/

    if(controllerNh_.hasParam("/robot_version"))
    {
      int raw_version = 0;
      controllerNh_.getParam("/robot_version", raw_version);
      robotVersion_ = RobotVersion::create(raw_version).version_number();
    }
    std::cout << "robotVersion_: " << robotVersion_ << std::endl;
    if(controllerNh_.hasParam("/use_vr_control"))
    {
      controllerNh_.getParam("/use_vr_control", use_vr_control_);
      std::cout << "use_vr_control: " << use_vr_control_ << std::endl;
    }
    if (controllerNh_.hasParam("/arm_move_spd"))
    {
      controllerNh_.getParam("/arm_move_spd", arm_move_spd_);
    }
    if(controllerNh_.hasParam("/use_external_mpc"))
    {
      controllerNh_.getParam("/use_external_mpc", enable_mpc_);
      std::cout << "enable_mpc: " << enable_mpc_ << std::endl;
      // 设置 enable_manipulation_mpc 参数为 true
      controllerNh_.setParam("/enable_manipulation_mpc", true);
      std::cout << "enable_manipulation_mpc: true" << std::endl;
    }
    
    double controlFrequency = 500.0; // 1000Hz
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
      ROS_INFO_STREAM("[humanoidControllerWheelWbc] arm trajectory interpolator enable="
                      << (enable_arm_traj_interpolator_ ? "true" : "false"));
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
    if(robotVersion_ == 60)
    {
      mujoco_q[2] = 0.0;
    }
    else if(robotVersion_ == 61 || robotVersion_ == 62 || robotVersion_ == 63 || robotVersion_ == 200062 || robotVersion_ == 300062)
    {
      mujoco_q[2] = 0.0;
    }
    mujoco_q[3] = 1.0;
    if ((robotVersion_ == 62 || robotVersion_ == 63) && hasQibeng)
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

    std::vector<double> robot_init_state_param;
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      robot_init_state_param.push_back(mujoco_q(i));
    }

    std::vector<double> stand_arm_joint_state_vector;
    int armStartIndex = 7 + lowJointNum_;
    for (int i = 0; i < armNum_; i++)
    {
      stand_arm_joint_state_vector.push_back(mujoco_q(armStartIndex + i));
    }

    /******************************** 双臂初始动作 ****************************************/
    vector_t startAction = mujoco_q.tail(manipulatorModelInfo_.armDim + headNum_).head(manipulatorModelInfo_.armDim);
    vector_t targetAction = startAction;
    targetAction.tail(armNum_)[4] = startAction.tail(armNum_)[4] - 0.5236;
    targetAction.tail(armNum_/2)[4] = startAction.tail(armNum_/2)[4] + 0.5236;
    double preActionDesiredTime = 1.5;
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
    velControlStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/enable_vel_control_state", 1, true);
    enableControlStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/enable_control_state", 1, true);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    waistYawKinematicPublisher_ = controllerNh_.advertise<nav_msgs::Odometry>("/waist_yaw_link_kinematic", 10);
    lbLegTrajPub_ = controllerNh_.advertise<sensor_msgs::JointState>("/lb_leg_traj", 10);
    stopRobotPub_ = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);
    resetToStatePub_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/mobile_manipulator_reset_to_state", 1);

    // 发布初始速度控制开关状态
    {
      std_msgs::Bool msg;
      msg.data = use_vel_control_;
      velControlStatePub_.publish(msg);
    }

    // 发布初始 enable_control 状态 (latched)
    {
      std_msgs::Bool msg;
      msg.data = enable_control_.load();
      enableControlStatePub_.publish(msg);
    }

    // 注册 /enable_control service（直接注册，回调需要访问 WBC 成员做 transition 覆写）
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

    // 初始化腰部运动学计算器
    waistKinematics_ = std::make_shared<humanoid_controller::WaistKinematics>();
    
    // 设置 Pinocchio 接口，保证模型数据一致性
    if (pinocchioInterface_ptr_) 
    {
        waistKinematics_->setPinocchioInterface(pinocchioInterface_ptr_, "waist_yaw_link");
        ROS_INFO("[humanoidController_wheel_wbc] WaistKinematics initialized with Pinocchio interface");
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
    auto contact_force_wbc = std::dynamic_pointer_cast<mobile_manipulator::ContactForceWbc>(wheel_wbc_);
    
    // 初始化期望力管理器（从ContactForceWbc获取插值速度参数）
    double interpolation_speed = contact_force_wbc ? contact_force_wbc->getInterpolationSpeed() : 15.0;
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
        ROS_INFO("[humanoidController_wheel_wbc] MPC mode initialized to ArmOnly for VR torso control");
      }
      else
      {
        ROS_WARN("[humanoidController_wheel_wbc] Failed to initialize MPC mode to ArmOnly");
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
        exit(1);
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

    if (req.data == enable_control_.load())
    {
      res.success = true;
      res.message = "enable_control already " + std::to_string(req.data);
      return true;
    }

    enable_control_.store(req.data);

    // 通知底盘调度模式：软急停/恢复
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
      if (enable_mpc_)
      {
        mrtRosInterface_->pauseResumeMpcNode(true);
      }
      // 3791: 每次 disable 重新记录冻结姿态 —— frozen_state_valid_ 置 true 后不会自动复位，
      // 若不清，第二次及以后 disable 时 PAUSING 首拍的冻结记录块（!frozen_state_valid_ 门控）
      // 被跳过，frozen_state_ 停留在第一次 disable 的陈旧姿态（如 home），软暂停时机身
      // 会回陈旧冻结位、enable 后 rst/观测/RM 锚定全部错位导致甩飞。清掉后 PAUSING 首拍
      // 自然重新记录本次 disable 的真实姿态。
      frozen_state_valid_.store(false);
      soft_pause_state_.store(SoftPauseState::PAUSING);
      soft_pause_transition_cycles_.store(0);
      ROS_INFO("[enable_control] disable 受理: enable=false 已发布, MPC pause 已发, 进入 PAUSING");
    }
    else
    {
      // ===== enable 受理：从冻结干净重启（不发 enable=true，到 IDLE 才发） =====
      // resume 立即发（solver 尽快恢复）；resetMpcNode/rst_target/滤波器 reset
      // 由主循环 RESUMING 首拍完成（FK 计算需主线程）；RESUMING 期间 MPC 观测
      // 强制 = 冻结+odom（反向写回），保证 solver 起点与 RM 参考锚一致。
      mpc_policy_gate_open_ = false;
      if (enable_mpc_)
      {
        mrtRosInterface_->pauseResumeMpcNode(false);
      }
      soft_pause_state_.store(SoftPauseState::RESUMING);
      soft_pause_transition_cycles_.store(0);
      resuming_reset_done_.store(false);
      ROS_INFO("[enable_control] enable 受理: MPC resume 已发, 进入 RESUMING（IDLE 时发布 enable=true）");
    }

    res.success = true;
    res.message = "enable_control set to " + std::to_string(req.data);
    return true;
  }

  bool humanoidControllerWheelWbc::enableVrArmAccelTaskCallback(std_srvs::SetBool::Request &req,
                                                                std_srvs::SetBool::Response &res)
  {
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

  bool humanoidControllerWheelWbc::starting(const ros::Time &time)
  {
    ROS_WARN_THROTTLE(1.0, "[starting] Waiting for odometry data...");
    // 1. 启动仿真/硬件（回放模式跳过）
    bool play_back = false;
    controllerNh_.getParam("/play_back", play_back);
    if (play_back) {
      ROS_INFO("[starting] Play back mode, skipping sim/hardware startup");
    } else if (!is_real_) {
      callSimStartSrv(controllerNh_);
    } else {
      // 等待硬件就绪
      int isHardwareReady = 0;
      while (ros::ok() && isHardwareReady != 1)
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
    
    while (ros::ok() && !control_data_manager_->isDataReady(false)) 
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
    if (!ros::ok()) 
    {
      ROS_ERROR("ROS shutdown detected");
      return false;
    }
    
    ROS_INFO("Data ready. Controller starting complete.");

    // control_data_manager_->setOdomReset();  // 重置里程计
    return true;
  }

  bool humanoidControllerWheelWbc::preUpdate(const ros::Time &time)
  {
    /*********************定时和精确周期调用****************************/
    static double lastTime = time.toSec() - dt_;
    double curTime = time.toSec();
    double dt = curTime - lastTime;
    if(dt < dt_) return true;
    /****************************************************************/

    ROS_INFO_THROTTLE(1.0, "[preUpdate] preUpdate is running !");
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    if (!control_data_manager_->getRealtimeSensorData(sensors_data_new)) {
        ROS_WARN_THROTTLE(1.0, "[preUpdate] Waiting for get sensor data");
        return false;
    }
    
    // 从控制数据管理器获取里程计数据
    vector6_t odomData_new = vector6_t::Zero();
    computeObservationFromSensorData(sensors_data_new, odomData_new);

    static double endTime = time.toSec() + robotPreActionDesiredTime_ + 0.5;
    performSimpleActions(time);   // 执行预设动作
    
    if(time.toSec() > endTime || !is_real_)
    {
      setupMrt();
      initMPC();
      isPreUpdateComplete = true;
      ROS_INFO_THROTTLE(1.0, "[preUpdate] preUpdate is done.");
    }

    return true;
  }

  void humanoidControllerWheelWbc::update(const ros::Time &time, const ros::Duration &dfd)
  {
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
    if(reset_mpc_) // 重置mpc
    {
      // use pinocchio 
      std::vector<Eigen::Vector3d> init_ee_pos(info.eeFrames.size());
      std::vector<Eigen::Matrix3d> init_ee_rot(info.eeFrames.size());
      getEEPose(observation_wheel_.state, init_ee_pos, init_ee_rot);

      Eigen::Vector3d init_torso_pos;
      Eigen::Matrix3d init_torso_rot;
      getTorsoPose(observation_wheel_.state, init_torso_pos, init_torso_rot);

      // initial command
      int base_nums = info.stateDim - info.armDim;
      vector_t initTarget(base_nums + 7 + info.eeFrames.size() * 7);
      initTarget.head(base_nums) = observation_wheel_.state.head(base_nums);
      initTarget.segment(base_nums, 3) = init_torso_pos;
      initTarget.segment(base_nums+3, 4) = Eigen::Quaternion<scalar_t>(init_torso_rot).coeffs();
      for(int eef_inx = 0; eef_inx < info.eeFrames.size(); eef_inx++)
      {
        initTarget.tail(info.eeFrames.size() * 7).segment(eef_inx*7, 3) = init_ee_pos[eef_inx];
        initTarget.tail(info.eeFrames.size() * 7).segment(eef_inx*7+3, 4) = Eigen::Quaternion<scalar_t>(init_ee_rot[eef_inx]).coeffs();
      }
      auto target_trajectories = TargetTrajectories({curTime}, 
                                                    {initTarget}, 
                                                    {observation_wheel_.input});
      mrtRosInterface_->resetMpcNode(target_trajectories);

      reset_mpc_ = false;
      std::cout << "reset MPC node at " << observation_wheel_.time << "\n";

      // reset kinemic Limit Filters
      obsStateLimitFilterPtr_->reset(observation_wheel_.state);
      obsInputLimitFilterPtr_->reset(observation_wheel_.input);
      mrtStateLimitFilterPtr_->reset(observation_wheel_.state);
      mrtInputLimitFilterPtr_->reset(observation_wheel_.input);

    }
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    auto bIsgetSensorData = control_data_manager_->getRealtimeSensorData(sensors_data_new);
    if (!bIsgetSensorData) 
    {
        ROS_ERROR_THROTTLE(1.0, "[update] Failed to get sensor realtime data");
    }
    
    // 从控制数据管理器获取里程计数据
    vector6_t odomData_new;
    auto bIsgetOdomData = control_data_manager_->getRealtimeOdomData(odomData_new);
    if (!bIsgetOdomData) 
    {
        ROS_WARN_THROTTLE(1.0, "[update] Failed to get odometry realtime data");
    }

    computeObservationFromSensorData(sensors_data_new, odomData_new);
    markStageMs("sensor_obs");

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
      mrtRosInterface_->updatePolicy();

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
    // 加 enable_control_ + 闸门：disable 期间/重置未就绪时一律不取 policy（getPolicy 返回裸引用，
    // 与 MRT 异步 reset() 析构存在悬垂竞态，详见上方 rollout 块注释）。
    if (soft_pause_state_.load() == SoftPauseState::IDLE && mpc_policy_gate_open_ && mrtRosInterface_->isPolicyUpdated())
    {
      robotVisualizer_->update(observation_wheel_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());
    }
    else
    {
      robotVisualizer_->update_obs(observation_wheel_);
    }
    markStageMs("mpc_mrt");

    /******************  用户修改部分  ***********************/
    vector_t target_qpos, target_qvel;
    target_qpos.setZero(info.armDim);
    target_qvel.setZero(info.armDim);

    updateUserJointCmd(time, target_qpos, target_qvel);

    int8_t lbMpcMode = control_data_manager_->getLbMpcControlMode(); // 获取当前轮臂MPC控制模式
    if (enable_arm_traj_interpolator_)
    {
      applyArmTrajectoryInterpolation(time, lbMpcMode, sensors_data_new, target_qpos, target_qvel);
    }
    if(!enable_mpc_)
    {
      optimizedState_mrt_.tail(info.armDim) = target_qpos;
      optimizedInput_mrt_.tail(info.armDim) = target_qvel;
    }
    else  // 轮臂MPC模式下的特殊处理
    {
      // 手臂跟踪快模式: 直接从 kuavo_arm_traj 话题获取手臂关节指令
      if (quickMode_ != 0 && (lbMpcMode == 1 || lbMpcMode == 3))  // 设置仅在armOnly和baseArm模式下生效
      {
        vector_t leg_target_qpos = vector_t::Zero(lowJointNum_);
        vector_t leg_target_qvel = vector_t::Zero(lowJointNum_);

        if(quickMode_ == 1 || quickMode_ == 3)
        {
          leg_target_qpos = control_data_manager_->getLegExternalControlState().pos;
          leg_target_qvel = control_data_manager_->getLegExternalControlState().vel;
          optimizedState_mrt_.segment(baseDim_, lowJointNum_) = leg_target_qpos;
          optimizedInput_mrt_.segment(baseDim_, lowJointNum_) = leg_target_qvel;
          ros_logger_->publishVector("/humanoid_wheel/leg_target_qpos_quick_mode", leg_target_qpos);
        }
        if(quickMode_ == 2 || quickMode_ == 3)
        {
          // 兼容旧版：默认直通外部手臂目标；开启插补开关时由插补路径覆盖。
          if (!enable_arm_traj_interpolator_)
          {
            vector_t arm_target_qpos = vector_t::Zero(armNum_);
            vector_t arm_target_qvel = vector_t::Zero(armNum_);
            arm_target_qpos = control_data_manager_->getArmExternalControlState().pos;
            arm_target_qvel = control_data_manager_->getArmExternalControlState().vel;
            optimizedState_mrt_.tail(armNum_) = arm_target_qpos;
            optimizedInput_mrt_.tail(armNum_) = arm_target_qvel;
            ros_logger_->publishVector("/humanoid_wheel/arm_target_qpos_quick_mode", arm_target_qpos);
          }
        }
      }
    }
    /*******************************************************/
    ros_logger_->publishVector("/humanoid_wheel/optimizedState_mrt", optimizedState_mrt_);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_mrt", optimizedInput_mrt_);

    optimizedState_mrt_limit_ = mrtStateLimitFilterPtr_->update(optimizedState_mrt_);
    optimizedInput_mrt_limit_ = mrtInputLimitFilterPtr_->update(optimizedInput_mrt_);

    ros_logger_->publishVector("/humanoid_wheel/optimizedState_mrt_kinemicLimit", optimizedState_mrt_limit_);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_mrt_kinemicLimit", optimizedInput_mrt_limit_);

    static int update_cnt = 0;
    if(update_cnt < (int)(1/dt_))   // 延时1秒钟进mpc，使mpc指令缓冲充分刷新
    {
      static vector_t observation_wheel_state_prev = observation_wheel_.state;
      optimizedState_mrt_limit_.setZero();
      optimizedState_mrt_limit_.tail(info.armDim) = observation_wheel_state_prev.tail(info.armDim);
      optimizedInput_mrt_limit_.setZero();
      update_cnt++;
    }

    // {
    //   vector_t qposLimit = optimizedState_mrt_limit_.tail(info.armDim);
    //   jointCmdLimiterPtr_->clipPositionCommand(qposLimit);
    //   optimizedState_mrt_limit_.tail(info.armDim) = qposLimit;

    //   if(enable_mpc_)   // mpc 仅采用硬约束的state作为反馈, 不修改轨迹的动态特性
    //   {
    //     optimizedState_mrt_ = optimizedState_mrt_limit_;
    //     optimizedInput_mrt_ = optimizedInput_mrt_limit_;
    //   }
    // }

    {
      // 关节段 [baseDim_:stateDim) = 下肢 lowJointNum_ + 手臂 armNum_，见 humanoidController_wheel_wbc.h 维度注释
      static vector_t qposLimit, qvelLimit;

      qposLimit = optimizedState_mrt_limit_.tail(info.armDim);
      qvelLimit = optimizedInput_mrt_limit_.tail(info.armDim);
      jointCmdLimiterPtr_->update(qposLimit, qvelLimit);
      optimizedState_mrt_limit_.tail(info.armDim) = qposLimit;
      static vector_t jointPosTarget_last = optimizedState_mrt_limit_.tail(info.armDim);
      const vector_t jointPosDelta =
          (optimizedState_mrt_limit_.tail(info.armDim) - jointPosTarget_last) / dt_;
      if (enable_arm_traj_interpolator_) {
        // 手臂轨迹插补仅应覆盖手臂段速度；下肢仍用位置差分，与 state 同向。
        // 若对全 armDim 使用 MPC optimizedInput（躯干笛卡尔模式下常为 0 或与 state 不同步），
        // WBC 下肢 PD 的 vel_error 会被 kd 放大，例如 data[3](knee_pitch) 出现大幅负值。
        optimizedInput_mrt_limit_.segment(baseDim_, lowJointNum_) = jointPosDelta.head(lowJointNum_);
        optimizedInput_mrt_limit_.tail(armNum_) = qvelLimit.tail(armNum_);
      } else {
        optimizedInput_mrt_limit_.tail(info.armDim) = jointPosDelta;
      }
      jointPosTarget_last = optimizedState_mrt_limit_.tail(info.armDim);
    }

    // disable 期间：冻结全身关节角度、清零所有速度，底盘不冻结（跟 odom 实时走）
    // 3791: 冻结覆写条件改为状态机 —— enable 受理后 enable_control_ 已 true 但状态机
    // 还在 RESUMING，WBC 输出仍需保持冻结（直到 IDLE rollout 接管）
    // 并发协议：frozen_state_ 仅主线程锁内写（PAUSING 首拍）；frozen_state_valid_ 是
    // atomic 可锁外查，服务线程 disable 受理只清 flag 不碰数据，故先查 flag 再加锁安全
    if (soft_pause_state_.load() != SoftPauseState::IDLE && frozen_state_valid_)
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

          // 目标轨迹必须用 target-pose 格式（仿 initMPC L1792-1815）：
          //   [base(3) + torso_pos(3) + torso_quat(4) + ee_pos(3)*N + ee_quat(4)*N]
          // 不能传 21 维 MPC state —— 基类缓冲被 getTargetTrajectories()（约束层）与
          // publishTargetTrajectories（rviz "command" 帧）按目标姿态格式解析，传 state
          // 会把关节角当四元数（invalid quaternion）并污染 RM 目标缓冲。
          const int base_nums = static_cast<int>(manipulatorModelInfo_.stateDim
                                               - manipulatorModelInfo_.armDim);
          std::vector<Eigen::Vector3d> rst_ee_pos(manipulatorModelInfo_.eeFrames.size());
          std::vector<Eigen::Matrix3d> rst_ee_rot(manipulatorModelInfo_.eeFrames.size());
          getEEPose(rst_state, rst_ee_pos, rst_ee_rot);
          Eigen::Vector3d rst_torso_pos;
          Eigen::Matrix3d rst_torso_rot;
          getTorsoPose(rst_state, rst_torso_pos, rst_torso_rot);

          vector_t rst_target(base_nums + 7 + manipulatorModelInfo_.eeFrames.size() * 7);
          rst_target.head(base_nums) = rst_state.head(base_nums);
          rst_target.segment(base_nums, 3) = rst_torso_pos;
          rst_target.segment(base_nums + 3, 4) = Eigen::Quaternion<scalar_t>(rst_torso_rot).coeffs();
          for (int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
          {
            rst_target.tail(manipulatorModelInfo_.eeFrames.size() * 7)
                     .segment(eef_inx * 7, 3) = rst_ee_pos[eef_inx];
            rst_target.tail(manipulatorModelInfo_.eeFrames.size() * 7)
                     .segment(eef_inx * 7 + 3, 4) = Eigen::Quaternion<scalar_t>(rst_ee_rot[eef_inx]).coeffs();
          }

          mrtRosInterface_->spinMRT();
          mrtRosInterface_->resetMpcNode(
              TargetTrajectories({curTime}, {rst_target}, {zero_input}));

          // 同步限幅滤波器到冻结姿态，消除 Ruckig 滞后跳变
          mrtStateLimitFilterPtr_->reset(rst_state);
          mrtInputLimitFilterPtr_->reset(zero_input);

          // 3791: 把冻结姿态发给 RM（resetAllMpcToState：storage/Ruckig 锚冻结）。
          // 连续发布 3 拍兜底：话题未 latch 且 pub/sub 队列均为 1，单拍可能错位；
          // RM 只消费一次，多余拍无害
          reset_state_to_publish_ = rst_state;
          reset_to_state_publish_cnt_ = 3;
          resuming_reset_done_.store(true);
          ROS_INFO("[enable_control] MPC reset from frozen state (RESUMING first cycle)");
        }
        if (soft_pause_transition_cycles_.fetch_add(1) + 1 >= SOFT_PAUSE_TRANSITION_CYCLES)
        {
          soft_pause_state_.store(SoftPauseState::IDLE);
          // 全局同步总开关：仅 IDLE 发布 true（enable 受理时未发，延迟到此）
          std_msgs::Bool msg;
          msg.data = true;
          enableControlStatePub_.publish(msg);
          ROS_INFO("[enable_control] RESUMING -> IDLE, enable=true published");
        }
      }

      // 连续发布冻结姿态给 RM（reset_to_state_publish_cnt_ 在 RESUMING 首拍设置，
      // 兜底话题丢包）
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
    auto contact_force_wbc = std::dynamic_pointer_cast<mobile_manipulator::ContactForceWbc>(wheel_wbc_);
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
    vector_t x = wheel_wbc_->update(optimizedState_wbc, optimizedInput_wbc, observation_wheel_);
    markStageMs("wbc");

    // 决策变量顺序：x = [ddq_stateDim, f_contact, tau_armDim]
    vector_t bodyAcc = x.head(info.stateDim-info.armDim);
    vector_t jointAcc = x.segment(info.stateDim-info.armDim, info.armDim);
    // 接触力在中间，力矩在最后
    size_t contact_force_size = 6 * manipulatorModelInfo_.eeFrames.size();
    vector_t torque = x.tail(info.armDim); // 力矩在决策变量的最后部分

    ros_logger_->publishVector("/humanoid_wheel/bodyAcc", bodyAcc);
    ros_logger_->publishVector("/humanoid_wheel/jointAcc", jointAcc);
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
    vector_t armJointVelForPublish = optimizedInput_mrt_limit_.tail(armNum_);
    for (int i1 = 0; i1 < lowJointNum_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_mrt_limit_.tail(info.armDim)[i1]);
      jointCmdMsg.joint_v.push_back(optimizedInput_mrt_limit_.tail(info.armDim)[i1]);
      jointCmdMsg.tau.push_back(torque.head(lowJointNum_)[i1]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i2 = 0; i2 < armNum_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_mrt_limit_.tail(armNum_)[i2]);
      jointCmdMsg.joint_v.push_back(armJointVelForPublish[i2]);
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
    jointCmdPub_.publish(jointCmdMsg);

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
    
    // 更新底盘速度（超时自动清零，梯形加减速限制）
    geometry_msgs::Twist cmdVelData;
    control_data_manager_->getRealtimeCmdVel(cmdVelData);  // 失败时cmdVelData保持默认零值
    // 发布速度命令（根据MPC状态选择来源）
    geometry_msgs::Twist velCmdMsg;  // 默认全0
    if (!enable_mpc_)
    {
      // 使用外部速度命令（经过加减速限制）
      Eigen::Vector3d desired_vel(cmdVelData.linear.x, cmdVelData.linear.y, cmdVelData.angular.z);
      Eigen::Vector3d limited_vel = velLimiter_->limitAcceleration(desired_vel);

      velCmdMsg.linear.x = limited_vel[0];
      velCmdMsg.linear.y = limited_vel[1];
      velCmdMsg.angular.z = limited_vel[2];
    }
    else if(baseCmdVelStatus_ == true)  // BaseCmdVelStatus 为 false 时，不设置速度
    {
      Eigen::Vector3d desiredVel = optimizedInput_mrt_limit_.head(3);
      Eigen::Vector3d desiredVelBody = cmdVelWorldToBody(desiredVel,
                                                         observation_wheel_.state[2]);
      // 使用MPC优化的速度
      velCmdMsg.linear.x = desiredVelBody[0];
      velCmdMsg.linear.y = desiredVelBody[1];
      velCmdMsg.angular.z = desiredVelBody[2];
    }
    if(use_vel_control_ && baseCmdVelStatus_ == true)
    {
      if (base_cmd_vel_limit_enable_)
      {
        clampBaseCmdVel(velCmdMsg);
      }
      cmdVelPub_.publish(velCmdMsg);
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
    static bool firstRun = true;
    static double last_yaw_ = 0.0;
    static double accumulated_yaw_ = 0.0;

    if(firstRun){
      last_time_ = current_time_;
      last_yaw_ = odomData[2]; // 初始时的yaw角度
      accumulated_yaw_ = odomData[2]; // 初始化累积yaw角度
      firstRun = false;
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

    // use pinocchio 
    std::vector<Eigen::Vector3d> init_ee_pos(manipulatorModelInfo_.eeFrames.size());
    std::vector<Eigen::Matrix3d> init_ee_rot(manipulatorModelInfo_.eeFrames.size());
    getEEPose(initial_observation.state, init_ee_pos, init_ee_rot);

    Eigen::Vector3d init_torso_pos;
    Eigen::Matrix3d init_torso_rot;
    getTorsoPose(initial_observation.state, init_torso_pos, init_torso_rot);

    // initial command
    int base_nums = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
    vector_t initTarget(base_nums + 7 + manipulatorModelInfo_.eeFrames.size() * 7);
    initTarget.head(base_nums) = vector_t::Zero(base_nums);
    initTarget.segment(base_nums, 3) = init_torso_pos;
    initTarget.segment(base_nums+3, 4) = Eigen::Quaternion<scalar_t>(init_torso_rot).coeffs();
    for(int eef_inx = 0; eef_inx < manipulatorModelInfo_.eeFrames.size(); eef_inx++)
    {
      initTarget.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx*7, 3) = init_ee_pos[eef_inx];
      initTarget.tail(manipulatorModelInfo_.eeFrames.size() * 7).segment(eef_inx*7+3, 4) = Eigen::Quaternion<scalar_t>(init_ee_rot[eef_inx]).coeffs();
    }

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
      return target_arm_state;
    }
    arm_start_pos_ = interpolated_target;
    // std::cout << "[ArmControl] 模式切换中，误差: " << error  << ", 目标: " << target_arm_state.transpose() << ", 插值: " << interpolated_target.transpose() << std::endl;
    return interpolated_target;
  }

  void humanoidControllerWheelWbc::applyArmTrajectoryInterpolation(const ros::Time& time,
                                                                    int8_t lbMpcMode,
                                                                    const SensorData& sensorData,
                                                                    vector_t& target_qpos,
                                                                    vector_t& target_qvel)
  {
    (void)sensorData;
    if (armNum_ == 0) {
      return;
    }

    ArmTrajectoryInterpolator::ModeFlags modeFlags;
    modeFlags.useArmTrajectoryControl = use_arm_trajectory_control_;
    modeFlags.quickMode = quickMode_;
    modeFlags.lbMpcMode = lbMpcMode;
    modeFlags.armCtrlMode = arm_trajectory_mode_;

    const vector_t currentArmQ = observation_wheel_.state.tail(armNum_);

    vector_t armTargetRawQ = currentArmQ;
    vector_t armTargetRawV = vector_t::Zero(armNum_);
    bool hasArmTargetRaw = false;
    ArmJointTrajectory armTrajRaw = control_data_manager_->getArmExternalControlState();
    if (armTrajRaw.pos.size() == static_cast<Eigen::Index>(armNum_)) {
      armTargetRawQ = armTrajRaw.pos;
      hasArmTargetRaw = true;
      if (armTrajRaw.vel.size() == static_cast<Eigen::Index>(armNum_)) {
        armTargetRawV = armTrajRaw.vel;
      }
      armTrajectoryInterpolator_.ingestRawTarget(time, armTrajRaw.pos, armTrajRaw.vel);
    }
    if (hasArmTargetRaw) {
      ros_logger_->publishVector("/humanoid_wheel/arm_target_qpos_raw", armTargetRawQ);
      ros_logger_->publishVector("/humanoid_wheel/arm_target_qvel_raw", armTargetRawV);
    }

    const auto output = armTrajectoryInterpolator_.compute(time, modeFlags, currentArmQ);
    if (!output.valid || output.smoothQ.size() != static_cast<Eigen::Index>(armNum_)) {
      return;
    }
    ros_logger_->publishVector("/humanoid_wheel/arm_target_qpos_smooth", output.smoothQ);
    ros_logger_->publishVector("/humanoid_wheel/arm_target_qvel_smooth", output.smoothV);

    const bool quickArmModeActive = (quickMode_ == 2 || quickMode_ == 3) && (lbMpcMode == 1 || lbMpcMode == 3);
    const bool shouldUseOutput = use_arm_trajectory_control_ || quickArmModeActive;
    if (!shouldUseOutput) {
      return;
    }

    target_qpos.segment(lowJointNum_, armNum_) = output.smoothQ;
    target_qvel.segment(lowJointNum_, armNum_) = output.smoothV;
    if (enable_mpc_ && quickArmModeActive) {
      optimizedState_mrt_.tail(armNum_) = output.smoothQ;
      optimizedInput_mrt_.tail(armNum_) = output.smoothV;
    }
    ros_logger_->publishVector("/humanoid_wheel/arm_target_qpos_interp", output.smoothQ);
    ros_logger_->publishVector("/humanoid_wheel/arm_target_qvel_interp", output.smoothV);
  }

  void humanoidControllerWheelWbc::updateUserJointCmd(const ros::Time &time, vector_t& target_qpos, vector_t& target_qvel)
  {
    if(target_qpos.size() != manipulatorModelInfo_.armDim)
    {
      throw std::invalid_argument("[updateUserJointCmd] target_qpos size is invaild.");
    }
    if(target_qvel.size() != manipulatorModelInfo_.armDim)
    {
      throw std::invalid_argument("[updateUserJointCmd] target_qvel size is invaild");
    }

    // std::cout << "请在 updateUserJointCmd 中加入关节控制指令, 单位: 弧度, 顺序: 下肢+左臂+右臂" << std::endl;
    static bool firstRun = true;

    static double start_time = time.toSec();
    static double last_time = start_time + 2.0;
    static vector_t start_qpos = target_qpos;
    if(firstRun)
    {
      // target_qpos[0] = 0.314;
      // target_qpos[1] = -0.16;
      // target_qpos[2] = -0.157;
      // target_qpos[4] = -0.5;
      // target_qpos[7] = -0.5;
      // target_qpos[8] = -0.5;
      // 双臂 home 复用 init 阶段从 mujoco_q 提取的值（真实站姿，非写死双足遗留值，见 init 处注释）
      for(int i=0; i<init_arm_target_qpos_.size(); i++)
      {
        target_qpos[lowJointNum_+i] = init_arm_target_qpos_[i];
      }
      control_data_manager_->setLbWaistExternalControlState(target_qpos.head(lowJointNum_));
      firstRun = false;
    }
    static vector_t last_qpos = target_qpos;
    scalar_array_t timeTrajectory;
    timeTrajectory.push_back(start_time);
    timeTrajectory.push_back(last_time);
    vector_array_t qposTrajectory;
    qposTrajectory.push_back(start_qpos);
    qposTrajectory.push_back(last_qpos);
    target_qpos = LinearInterpolation::interpolate(time.toSec(), timeTrajectory, qposTrajectory);
    
    SensorData sensors_data_new;
    if (!control_data_manager_->getRealtimeSensorData(sensors_data_new)) 
    {
      ROS_WARN_THROTTLE(1.0, "[updateUserJointCmd] Failed to get sensor data, using observation state");
      // 从 observation 中提取历史关节位置，state 结构: [x, y, yaw, joint_q(4+7*2)] - 只有下肢和上肢，没有头部
      int obs_joints = lowJointNum_ + armNum_;  // 4 + 14 = 18 (不包括头部)
      int total_joints = lowJointNum_ + armNum_ + headNum_;  // 4 + 14 + 2 = 20
      sensors_data_new.resize_joint(total_joints);
      // 提取观测中的关节数据（下肢 + 上肢）
      sensors_data_new.jointPos_.head(obs_joints) = observation_wheel_.state.tail(obs_joints);
      // 头部关节数据不在 observation 中，设为0或保持上一次值
      // sensors_data_new.jointPos_.tail(headNum_).setZero();
    }

    // 从控制数据管理器获取轮臂外部控制状态
    vector_t lb_waist_external_state = control_data_manager_->getLbWaistExternalControlState(); 
    target_qpos.head(lowJointNum_) = lb_waist_external_state;

    // VR控制相关逻辑
    if(use_vr_control_)
    {
      // 从控制数据管理器获取base_link位姿
      vector_t base_pose = createZeroPose();
      if (!control_data_manager_->getRealtimeBaseLinkPose(base_pose)) 
      {
        ROS_WARN_THROTTLE(1.0, "[updateUserJointCmd] Waiting for base_link pose data...");
        base_pose[2] = 0.185;
      }

      vector_t init_joints =  lb_waist_external_state;
      vector_t current_joints = sensors_data_new.jointPos_.head(lowJointNum_);
      auto desire_lbLowJoint_pos_smooth = smoothTransition(current_joints, init_joints);

      // 从控制数据管理器获取VR躯干位姿
      vector_t vr_torso_pose = createZeroPose();
      bool vr_torso_pose_valid = control_data_manager_->getRealtimeVrTorsoPose(vr_torso_pose);
      
      auto torso_pose = waistKinematics_->computeWaistForwardKinematics(base_pose, desire_lbLowJoint_pos_smooth);
      auto target_torso_pose = waistKinematics_->transformPoseWithRelativeOffset(torso_pose, vr_torso_pose);
      auto ik_lb_low_Joint = waistKinematics_->computeFastWaistInverseKinematics(base_pose, target_torso_pose, current_joints);
      auto pinocchio_ik_result = waistKinematics_->computeWaistInverseKinematicsWithPinocchio(base_pose, target_torso_pose, current_joints, false);
      bool whole_torso_ctrl = control_data_manager_->getWholeTorsoCtrl();
      
      
      double filter_alpha = whole_torso_ctrl?0.02:0.99;
      for(int i1 = 0; i1 < 4; ++i1)
      {
        // 添加当前值到历史数据
        median_filter_history_[i1].push_back(ik_lb_low_Joint[i1]);
        
        // 保持窗口大小
        if (median_filter_history_[i1].size() > MEDIAN_FILTER_WINDOW_SIZE) 
        {
          median_filter_history_[i1].erase(median_filter_history_[i1].begin());
        }
        
        // 进行中值滤波
        double median_filtered_value;
        if (median_filter_history_[i1].size() >= 3) 
        {  // 至少需要3个数据点进行中值滤波
          median_filtered_value = medianFilter(median_filter_history_[i1], MEDIAN_FILTER_WINDOW_SIZE)[median_filter_history_[i1].size() - 1];
        } 
        else 
        {
          median_filtered_value = ik_lb_low_Joint[i1];  // 数据不足时直接使用原值
        }
        if(i1 != 3)
        {
          target_qpos[i1] = desire_lbLowJoint_pos_smooth[i1];
        }

        // 从控制数据管理器获取全身控制标志

        if(whole_torso_ctrl)
        {
          target_qpos[i1] = lowpassFilter(median_filtered_value, last_filtered_low_joint_pos_[i1], filter_alpha);
        }
        last_filtered_low_joint_pos_[i1] = target_qpos[i1];
      }
      ros_logger_->publishVector("/humanoid_wheel/base_pose", base_pose);
      ros_logger_->publishValue("/humanoid_wheel/filter_alpha", filter_alpha);
      ros_logger_->publishVector("/humanoid_wheel/vr_torso_pose", vr_torso_pose);
      ros_logger_->publishVector("/humanoid_wheel/ik_lb_low_Joint", ik_lb_low_Joint);
      ros_logger_->publishVector("/humanoid_wheel/filter_lb_low_Joint", target_qpos);
      ros_logger_->publishVector("/humanoid_wheel/target_torso_pose", target_torso_pose);
      ros_logger_->publishVector("/humanoid_wheel/pinocchio_ik_result", pinocchio_ik_result);

      // 当使用外部MPC且启用全身控制时，发布lb_leg_traj话题
      if(enable_mpc_ && whole_torso_ctrl && vr_torso_pose_valid)
      {
        sensor_msgs::JointState leg_traj_msg;
        leg_traj_msg.header.stamp = ros::Time::now();
        leg_traj_msg.name.resize(lowJointNum_);
        leg_traj_msg.position.resize(lowJointNum_);
        leg_traj_msg.velocity.resize(lowJointNum_, 0.0);
        
        // 使用滤波后的关节角度（转换为度）
        for(int i = 0; i < lowJointNum_; ++i)
        {
          leg_traj_msg.name[i] = "leg_joint_" + std::to_string(i+1);
          leg_traj_msg.position[i] = target_qpos[i] * 180.0 / M_PI;  // 弧度转角度
        }
        
        lbLegTrajPub_.publish(leg_traj_msg);
      }
      // 如果当前躯干模式为false，上一躯干模式为true，则下发一次 0 指令作为终止
      static bool pre_torso_ctrl = whole_torso_ctrl;

      if(whole_torso_ctrl == false && pre_torso_ctrl == true)
      {
        sensor_msgs::JointState leg_traj_msg;
        leg_traj_msg.header.stamp = ros::Time::now();
        leg_traj_msg.name.resize(lowJointNum_);
        leg_traj_msg.position.resize(lowJointNum_, 0.0);
        leg_traj_msg.velocity.resize(lowJointNum_, 0.0);
        for(int i = 0; i < lowJointNum_; ++i)
        {
          leg_traj_msg.name[i] = "leg_joint_" + std::to_string(i+1);
        }
        lbLegTrajPub_.publish(leg_traj_msg);
      }
      pre_torso_ctrl = whole_torso_ctrl;
    }

    if(use_arm_trajectory_control_)
    {
      // 从控制数据管理器获取手臂轨迹
      ArmJointTrajectory traj = control_data_manager_->getArmExternalControlState();
      target_qpos.segment(lowJointNum_, armNum_) = traj.pos;
      // target_qvel = traj.vel;
    }

    vector_t target_arm_joints = vector_t::Zero(armNum_);
    vector_t current_arm_joints = vector_t::Zero(armNum_);
    for(int i1 = 0; i1 < armNum_; ++i1)
    {
      target_arm_joints[i1] = target_qpos.segment(lowJointNum_, armNum_)[i1];
      current_arm_joints[i1] = sensors_data_new.jointPos_[lowJointNum_+i1];
    }

    vector_t target_arm_joints_new = vector_t::Zero(armNum_);
    if(isArmControlModeChanged_)
    {
      target_arm_joints_new = processArmControlModeSwitch(time, current_arm_joints, target_arm_joints);
      target_qpos.segment(lowJointNum_, armNum_) = target_arm_joints_new;
      ros_logger_->publishVector("/humanoid_wheel/target_arm_joints_new", target_arm_joints_new);
    }
  }

  bool humanoidControllerWheelWbc::changeArmCtrlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    int new_mode = req.control_mode;
    
    // 检查模式是否发生变化
    if (new_mode != arm_trajectory_mode_) 
    {
      prev_arm_trajectory_mode_ = arm_trajectory_mode_;
      arm_trajectory_mode_ = new_mode;
      isArmControlModeChanged_ = true;
      arm_mode_switch_hold_phase_ = false;
      std::cout << "[ArmControl] 模式切换: " << prev_arm_trajectory_mode_ << " -> " << arm_trajectory_mode_ << std::endl;
    }
    
    use_arm_trajectory_control_ = (2 == arm_trajectory_mode_);
    
    res.result = true;
    res.mode = arm_trajectory_mode_;
    res.message = "success change arm ctrl mode";
    return true;
  }

  bool humanoidControllerWheelWbc::enableLbArmQuickModeCallback(kuavo_msgs::changeLbQuickModeSrv::Request &req, 
                                                                kuavo_msgs::changeLbQuickModeSrv::Response &res)
  {
    std::cout << "[ArmControl] 快速模式切换请求, 请求模式为: " << int(req.quickMode) << std::endl;
    quickMode_ = req.quickMode;
    res.success = true;
    res.message = "success change quick ctrl mode to " + std::to_string(req.quickMode);
    return true;
  }

  bool humanoidControllerWheelWbc::changeLbObsUpdateModeCallback(kuavo_msgs::changeLbMpcObsUpdateModeSrv::Request &req, 
                                                                kuavo_msgs::changeLbMpcObsUpdateModeSrv::Response &res)
  {
    std::cout << "[ArmControl] mpc反馈机制切换请求: " << int(req.obsUpdateMode) << std::endl;
    mpcObsUpdateMode_ = req.obsUpdateMode;
    res.success = true;
    res.message = "success change obs update mode to " + std::to_string(req.obsUpdateMode);
    return true;
  }

  bool humanoidControllerWheelWbc::enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
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

    vector_t x = wheel_wbc_->update(curTargetState_wbc, inputVelocity, observation_wheel_);
    vector_t torque = x.tail(manipulatorModelInfo_.armDim); // 力矩在决策变量的最后部分

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
    bool verbose = false;
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
    Eigen::Vector3d posError = (targetTorsoPose.translation() - currentTorsoPose.translation());
    Eigen::Vector3d rotError = pinocchio::log3(targetTorsoPose.rotation() * currentTorsoPose.rotation().transpose());
    Eigen::Vector3d rotErrorEulerZYX = axisAngleToEulerZYX(rotError);

    vector_t torsoError6d = vector_t::Zero(6);
    torsoError6d << posError, rotErrorEulerZYX;
    ros_logger_->publishVector("/humanoid_wheel/" + info.torsoFrame + "_error_6d", torsoError6d);

    if(verbose)
    {
      std::cout << info.torsoFrame << " position error: " << posError.transpose() << std::endl;
      std::cout << info.torsoFrame << " rotation error (ZYX) [yaw, pitch, roll]: " << rotErrorEulerZYX.transpose() << std::endl;
    }

    // 4. 计算所有末端执行器的误差（直接计算）
    for (const auto& eeFrame : info.eeFrames) {
      auto itCurrent = currentEePoses.find(eeFrame);
      auto itTarget = targetEePoses.find(eeFrame);
      
      if (itCurrent != currentEePoses.end() && itTarget != targetEePoses.end()) {
        Eigen::Vector3d posError = (itTarget->second.translation() - itCurrent->second.translation());
        Eigen::Vector3d rotError = pinocchio::log3(itTarget->second.rotation() * itCurrent->second.rotation().transpose());
        Eigen::Vector3d rotErrorEulerZYX = axisAngleToEulerZYX(rotError);

        if(verbose)
        {
          std::cout << eeFrame << " position error: " << posError.transpose() << std::endl;
          std::cout << eeFrame << " rotation error: " << rotError.transpose() << std::endl;
        }

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

} // namespace humanoidController_wheel_wbc

