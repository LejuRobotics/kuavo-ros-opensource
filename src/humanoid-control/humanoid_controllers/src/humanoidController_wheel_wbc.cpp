#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "humanoid_controllers/humanoidController_wheel_wbc.h"
#include "humanoid_interface/common/TopicLogger.h"
#include <iostream>
#include <cmath>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

#include "humanoid_wheel_interface/estimators/ContinuousEulerAnglesFromMatrix.h"


namespace humanoidController_wheel_wbc
{
  using namespace ocs2;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;

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
    optimizedState_mrt_.setZero(manipulatorModelInfo_.stateDim);
    optimizedInput_mrt_.setZero(manipulatorModelInfo_.inputDim);
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
    /************** Initialize WBC **********************/
    wheel_wbc_ = std::make_shared<mobile_manipulator::WeightedWbc>(*pinocchioInterface_ptr_, manipulatorModelInfo_);
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
      controllerNh_.getParam("/robot_version", robotVersion_);
    }
    std::cout << "robotVersion_: " << robotVersion_ << std::endl;
    if(controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
    }
    std::cout << "is_real: " << is_real_ << std::endl;
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
      config.maxVel = optimizedTrajMaxVel_.tail(armNum_).cwiseAbs();
      config.maxAcc = optimizedTrajMaxAcc_.tail(armNum_).cwiseAbs();
      config.maxJerk = optimizedTrajMaxJerk_.tail(armNum_).cwiseAbs();
      config.dqRemapK = vector_t::Ones(armNum_);
      config.dqRemapOffset = vector_t::Zero(armNum_);
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.q_cutoff_hz", config.qCutoffHz);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.v_cutoff_hz", config.vCutoffHz);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.a_cutoff_hz", config.aCutoffHz);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.timeout_sec", config.timeoutSec);
      } catch (const std::exception&) {}
      config.controlCycleSec = dt_;
      try {
        loadData::loadEigenMatrix(taskFile, "armTrajInterpKinematicLimit.dq_remap_k", config.dqRemapK);
      } catch (const std::exception&) {}
      try {
        loadData::loadEigenMatrix(taskFile, "armTrajInterpKinematicLimit.dq_remap_offset", config.dqRemapOffset);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.fhan_r", config.fhanR);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.fhan_h0_ratio", config.fhanH0Ratio);
      } catch (const std::exception&) {}
      try {
        loadData::loadCppDataType(taskFile, "armTrajInterpKinematicLimit.enable", enable_arm_traj_interpolator_);
      } catch (const std::exception&) {
        enable_arm_traj_interpolator_ = false;
      }
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
                                                            taskFile, manipulatorModelInfo_, dt_);
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
    else if(robotVersion_ == 61 || robotVersion_ == 62 || robotVersion_ == 63)
    {
      mujoco_q[2] = 0.0;
    }
    mujoco_q[3] = 1.0;
    mujoco_q[11] = 0.5236;
    mujoco_q[14] = -1.57;
    mujoco_q[18] = 0.5236;
    mujoco_q[21] = -1.57;

    std::vector<double> robot_init_state_param;
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      robot_init_state_param.push_back(mujoco_q(i));
    }

    controllerNh_.setParam("/robot_init_state_param", robot_init_state_param);

    // 设置初始状态参数
    std::vector<double> initial_state_vector(robot_init_state_param);
    std::vector<double> squat_initial_state_vector(robot_init_state_param);
    std::vector<double> default_joint_pos_vector(robot_init_state_param);
    controllerNh_.setParam("/initial_state", initial_state_vector);
    controllerNh_.setParam("/squat_initial_state", squat_initial_state_vector);
    controllerNh_.setParam("/default_joint_pos", default_joint_pos_vector);

    // 初始化 MPC 初始期望
    optimizedState_mrt_.tail(manipulatorModelInfo_.armDim) = mujoco_q.segment(7, manipulatorModelInfo_.armDim);


    // 初始化VR控制相关标志位和参数
    is_transitioning_ = false;
    prev_whole_torso_ctrl_ = false;
    transition_start_time_ = 0.0;

    //TODO
    controllerNh_.setParam("build_cppad_state", 2); // done 

    // 初始化发布者
    cmdVelPub_ = controllerNh_.advertise<geometry_msgs::Twist>("/move_base/base_cmd_vel", 10, true);
    velControlStatePub_ = controllerNh_.advertise<std_msgs::Bool>("/enable_vel_control_state", 1, true);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    waistYawKinematicPublisher_ = controllerNh_.advertise<nav_msgs::Odometry>("/waist_yaw_link_kinematic", 10);
    lbLegTrajPub_ = controllerNh_.advertise<sensor_msgs::JointState>("/lb_leg_traj", 10);

    // 发布初始速度控制开关状态
    {
      std_msgs::Bool msg;
      msg.data = use_vel_control_;
      velControlStatePub_.publish(msg);
    }

    // 创建控制数据管理器（替代所有订阅者和服务）
    vector_t leg_initial_state = optimizedState_mrt_.tail(manipulatorModelInfo_.armDim).head(lowJointNum_);
    vector_t arm_initial_state = optimizedState_mrt_.tail(manipulatorModelInfo_.armDim).tail(armNum_);
    control_data_manager_ = std::make_unique<ControlDataManager>(
        controllerNh_, is_real_, armNum_, lowJointNum_, headNum_, leg_initial_state, arm_initial_state);
    
    // 初始化所有订阅者（包括传感器数据订阅）
    control_data_manager_->initializeSubscribers();
    
    // 注册服务回调
    registerAllServices();
    init_arm_target_qpos_ = vector_t::Zero(armNum_);
    
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

    // 初始化过渡起点位置
    waist_transition_start_pos_ = vector_t::Zero(lowJointNum_);
    
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

  bool humanoidControllerWheelWbc::starting(const ros::Time &time)
  {
    ROS_WARN_THROTTLE(1.0, "[starting] Waiting for odometry data...");
    // 1. 启动仿真/硬件
    if (!is_real_) 
    {
      callSimStartSrv(controllerNh_);
    } 
    else 
    {
      // 等待硬件就绪
      int isHardwareReady = 0;
      while (ros::ok() && isHardwareReady != 1) 
      {
        controllerNh_.getParam("/hardware/is_ready", isHardwareReady);
        usleep(10000);  // 10ms
      }
    }

    // 2. 等待数据就绪（5秒超时）
    ROS_INFO("Waiting for ControlDataManager data...");
    auto start = std::chrono::steady_clock::now();
    int wait_sec = 0;
    
    control_data_manager_->setOdomReset();  // 重置里程计
    
    while (ros::ok() && !control_data_manager_->isDataReady(false)) 
    {
        ros::spinOnce();
        usleep(1000);
        
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed > 5.0) {
            ROS_ERROR("Data timeout! Check: /sensors_data_raw, /odom, /waist_yaw_link_pose");
            return false;
        }
        
        // 每秒打印一次
        if (static_cast<int>(elapsed) > wait_sec) {
            wait_sec = static_cast<int>(elapsed);
            ROS_WARN("Waiting... %d/5 s", wait_sec);
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
    static int cnt;
    ROS_INFO_THROTTLE(1.0, "[preUpdate] preUpdate is running !");
    cnt++;
    // 获取关节数据，并更新 Observation
    SensorData sensors_data_new;
    if (!control_data_manager_->getRealtimeSensorData(sensors_data_new)) {
        ROS_WARN_THROTTLE(1.0, "[preUpdate] Waiting for get sensor data");
        return false;
    }
    
    // 从控制数据管理器获取里程计数据
    vector6_t odomData_new = vector6_t::Zero();
    computeObservationFromSensorData(sensors_data_new, odomData_new);
    if(cnt == 5)
    {
      setupMrt();
      initMPC();
      isPreUpdateComplete = true;
      cnt = 0;
      ROS_INFO_THROTTLE(1.0, "[preUpdate] preUpdate is done.");
    }

    return true;
  }

  void humanoidControllerWheelWbc::update(const ros::Time &time, const ros::Duration &dfd)
  {
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
      auto target_trajectories = TargetTrajectories({observation_wheel_.time}, 
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

    // 更新 mpc 数据
    {
      vector_t optimizedState_mrt, optimizedInput_mrt;
      // Update the current state of the system
      SystemObservation kinemicLimitObs = observation_wheel_;
      kinemicLimitObs.state = obsStateLimitFilterPtr_->update(observation_wheel_.state);
      kinemicLimitObs.input = obsInputLimitFilterPtr_->update(observation_wheel_.input);

      /****************************允许采用mpc输出作为反馈**************************************/
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
      /**************************************************************************************/
      
      mrtRosInterface_->setCurrentObservation(kinemicLimitObs);

      // Trigger MRT callbacks
      mrtRosInterface_->spinMRT();
      // Update the policy if a new on was received
      if (mrtRosInterface_->updatePolicy())
      {
      }

      mrtRosInterface_->evaluatePolicy(observation_wheel_.time, observation_wheel_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      if(enable_mpc_)
      {
        optimizedState_mrt_ = optimizedState_mrt;
        optimizedInput_mrt_ = optimizedInput_mrt;
      }
      if(std::fabs(optimizedInput_mrt_[0]) < 0.05) optimizedInput_mrt_[0] = 0;
      if(std::fabs(optimizedInput_mrt_[1]) < 0.05) optimizedInput_mrt_[1] = 0;
      if(std::fabs(optimizedInput_mrt_[2]) < 0.05) optimizedInput_mrt_[2] = 0;
    }
    // 更新可视化数据
    // robotVisualizer_->update_obs(observation_wheel_);
    robotVisualizer_->update(observation_wheel_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());

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

    vector_t optimizedState_mrt_limit = optimizedState_mrt_;
    vector_t optimizedInput_mrt_limit = optimizedInput_mrt_;
    optimizedState_mrt_limit = mrtStateLimitFilterPtr_->update(optimizedState_mrt_);
    optimizedInput_mrt_limit = mrtInputLimitFilterPtr_->update(optimizedInput_mrt_);

    ros_logger_->publishVector("/humanoid_wheel/optimizedState_mrt_kinemicLimit", optimizedState_mrt_limit);
    ros_logger_->publishVector("/humanoid_wheel/optimizedInput_mrt_kinemicLimit", optimizedInput_mrt_limit);

    static int update_cnt = 0;
    if(update_cnt < (int)(1/dt_))   // 延时1秒钟进mpc，使mpc指令缓冲充分刷新
    {
      static vector_t observation_wheel_state_prev = observation_wheel_.state;
      optimizedState_mrt_limit.setZero();
      optimizedState_mrt_limit.tail(info.armDim) = observation_wheel_state_prev.tail(info.armDim);
      optimizedInput_mrt_limit.setZero();
      update_cnt++;
    }

    {
      static vector_t qposLimit, qvelLimit;

      qposLimit = optimizedState_mrt_limit.tail(info.armDim);
      qvelLimit = optimizedInput_mrt_limit.tail(info.armDim);
      jointCmdLimiterPtr_->update(qposLimit, qvelLimit);
      optimizedState_mrt_limit.tail(info.armDim) = qposLimit;
      optimizedInput_mrt_limit.tail(info.armDim) = qvelLimit;
    }

    vector_t optimizedState_wbc = optimizedState_mrt_limit;
    vector_t optimizedInput_wbc = optimizedInput_mrt_limit;
    const bool quickArmModeActive = (quickMode_ == 2 || quickMode_ == 3) && (lbMpcMode == 1 || lbMpcMode == 3);
    const bool armTrajOwnedByExternal = use_arm_trajectory_control_ || quickArmModeActive;
    if (enable_arm_traj_interpolator_ && armTrajOwnedByExternal && armNum_ > 0)
    {
      ArmJointTrajectory armTrajRaw = control_data_manager_->getArmExternalControlState();
      if (armTrajRaw.pos.size() == static_cast<Eigen::Index>(armNum_))
      {
        wbc_arm_raw_q_ = armTrajRaw.pos;
        if (armTrajRaw.vel.size() == static_cast<Eigen::Index>(armNum_))
        {
          wbc_arm_raw_v_ = armTrajRaw.vel;
        }
        else
        {
          wbc_arm_raw_v_.setZero(armNum_);
        }
      }
      if (wbc_arm_raw_q_.size() == static_cast<Eigen::Index>(armNum_) &&
          wbc_arm_raw_v_.size() == static_cast<Eigen::Index>(armNum_))
      {
        optimizedState_wbc.tail(armNum_) = wbc_arm_raw_q_;
        optimizedInput_wbc.tail(armNum_) = wbc_arm_raw_v_;
        ros_logger_->publishVector("/humanoid_wheel/wbc_arm_target_qpos_raw", wbc_arm_raw_q_);
        ros_logger_->publishVector("/humanoid_wheel/wbc_arm_target_qvel_raw", wbc_arm_raw_v_);
      }
      ROS_INFO_THROTTLE(1.0, "[humanoidControllerWheelWbc] WBC arm task uses raw kuavo_arm_traj; joint_cmd keeps interpolated q/dq.");
    }

    vector_t x = wheel_wbc_->update(optimizedState_wbc, optimizedInput_wbc, observation_wheel_);

    vector_t bodyAcc = x.head(info.stateDim-info.armDim);
    vector_t jointAcc = x.segment(info.stateDim-info.armDim, info.armDim);
    vector_t torque = x.tail(info.armDim);

    ros_logger_->publishVector("/humanoid_wheel/bodyAcc", bodyAcc);
    ros_logger_->publishVector("/humanoid_wheel/jointAcc", jointAcc);
    ros_logger_->publishVector("/humanoid_wheel/torque", torque);
    ros_logger_->publishVector("/humanoid_wheel/target_qpos", target_qpos);

    // 更新关节指令
    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    for (int i1 = 0; i1 < lowJointNum_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_mrt_limit.tail(info.armDim)[i1]);
      jointCmdMsg.joint_v.push_back(optimizedInput_mrt_limit.tail(info.armDim)[i1]);
      jointCmdMsg.tau.push_back(torque.head(lowJointNum_)[i1]);
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
      jointCmdMsg.control_modes.push_back(2);
    }
    for (int i2 = 0; i2 < armNum_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(optimizedState_mrt_limit.tail(armNum_)[i2]);
      jointCmdMsg.joint_v.push_back(optimizedInput_mrt_limit.tail(armNum_)[i2]);
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

      vector_t head_pos = sensors_data_new.jointPos_.tail(headNum_);
      robotVisualizer_->updateHeadJointPositions(head_pos);
    }
    replaceDefaultEcMotorPdoGait(jointCmdMsg);  // 统一修改pdo写入的kpkd
    jointCmdPub_.publish(jointCmdMsg);

    //更新共享内存中的关节命令
    control_data_manager_->publishJointCmdToShm(jointCmdMsg);
    
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
    else 
    {
      Eigen::Vector3d desiredVel = optimizedInput_mrt_limit.head(3);
      Eigen::Vector3d desiredVelBody = cmdVelWorldToBody(desiredVel, 
                                                         observation_wheel_.state[2]);
      // 使用MPC优化的速度
      velCmdMsg.linear.x = desiredVelBody[0];
      velCmdMsg.linear.y = desiredVelBody[1];
      velCmdMsg.angular.z = desiredVelBody[2];
    }
    if(use_vel_control_)
    {
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
    if (armNum_ == 0) {
      return;
    }

    ArmTrajectoryInterpolator::ModeFlags modeFlags;
    modeFlags.useArmTrajectoryControl = use_arm_trajectory_control_;
    modeFlags.quickMode = quickMode_;
    modeFlags.lbMpcMode = lbMpcMode;
    modeFlags.armCtrlMode = arm_trajectory_mode_;

    const vector_t currentArmQ = observation_wheel_.state.tail(armNum_);
    vector_t measuredDq = vector_t::Zero(armNum_);
    if (sensorData.jointVel_.size() >= static_cast<Eigen::Index>(lowJointNum_ + armNum_)) {
      measuredDq = sensorData.jointVel_.segment(lowJointNum_, armNum_);
    } else if (observation_wheel_.input.size() >= static_cast<Eigen::Index>(3 + lowJointNum_ + armNum_)) {
      measuredDq = observation_wheel_.input.segment(3 + lowJointNum_, armNum_);
    }

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
      armTrajectoryInterpolator_.ingestRawTarget(time, armTrajRaw.pos, armTrajRaw.vel, measuredDq);
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
      vector_t arm_target_qpos(armNum_);
      arm_target_qpos << -0.0, 0.4, 0.2, -1.5, -0.0, -0.0, -0.0, 
                         -0.0, -0.4, -0.2, -1.5, 0.0, -0.0, -0.0;
      init_arm_target_qpos_ = arm_target_qpos;
      for(int i=0; i<arm_target_qpos.size(); i++)
      {
        target_qpos[lowJointNum_+i] = arm_target_qpos[i];
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
    //   EC_MASTER 电机：使用 running_settings.joint_kp/kd（来自 kuavo.json joint_kp/kd）
    //   RUIWO 电机：使用 running_settings.ruiwo_kp/kd（来自 kuavo.json ruiwo_kp/kd）
    // 注意：ec_master_count/ruiwo_count 对应各自驱动器数组中的索引
    const auto &hardware_settings = kuavo_settings_.hardware_settings;
    const auto &running_settings = kuavo_settings_.running_settings;
    const int total_joints = lowJointNum_ + armNum_ + headNum_;

    // 替换 EC_MASTER 电机 kp/kd
    if (!running_settings.joint_kp.empty() && 
        !running_settings.joint_kd.empty() &&
        running_settings.joint_kp.size() == running_settings.joint_kd.size())
    {
      const int ec_master_size = static_cast<int>(running_settings.joint_kp.size());
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
            jointCmdMsg.joint_kp[i] = static_cast<double>(running_settings.joint_kp[ec_master_count]);
            jointCmdMsg.joint_kd[i] = static_cast<double>(running_settings.joint_kd[ec_master_count]);
          }
          // 无论 control_modes 是 0 还是 2，都要递增 ec_master_count
          // 因为 running_settings.joint_kp/kd 的索引对应所有 EC_MASTER 驱动器
          ec_master_count++;
        }
      }
    }

    // 替换 RUIWO 电机 kp/kd（手臂默认增益，来自 kuavo.json ruiwo_kp/kd）
    if (!running_settings.ruiwo_kp.empty() &&
        !running_settings.ruiwo_kd.empty() &&
        running_settings.ruiwo_kp.size() == running_settings.ruiwo_kd.size())
    {
      const int ruiwo_size = static_cast<int>(running_settings.ruiwo_kp.size());
      int ruiwo_count = 0;

      for (int i = 0; i < total_joints && i < static_cast<int>(jointCmdMsg.control_modes.size()); ++i)
      {
        if (i < static_cast<int>(hardware_settings.driver.size()) &&
            hardware_settings.driver[i] == RUIWO)
        {
          if (jointCmdMsg.control_modes[i] == 2 &&
              ruiwo_count < ruiwo_size)
          {
            jointCmdMsg.joint_kp[i] = static_cast<double>(running_settings.ruiwo_kp[ruiwo_count]);
            jointCmdMsg.joint_kd[i] = static_cast<double>(running_settings.ruiwo_kd[ruiwo_count]);
          }
          ruiwo_count++;
        }
      }
    }
  }

} // namespace humanoidController_wheel_wbc

