#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "std_srvs/SetBool.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <fstream>
#include <ros/this_node.h>
#include <cstdlib>

#include "humanoid_controllers/humanoidController.h"
#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "humanoid_controllers/CommonDDS.h"
#endif

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <humanoid_interface_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <std_srvs/Trigger.h>
#include <algorithm> 
#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>
#ifdef KUAVO_CONTROL_LIB_FOUND
#include <kuavo_estimation/base_filter/InEkfBaseFilter.h>
#endif
#include <humanoid_wbc/WeightedWbc.h>
#include <humanoid_wbc/StandUpWbc.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <humanoid_wbc/HierarchicalWbc.h>
#include "kuavo_common/common/sensor_data.h"
#include "kuavo_common/common/utils.h"
#include "humanoid_interface_drake/kuavo_data_buffer.h"
// #include "humanoid_interface_drake/common/utils.h"
// #include "humanoid_interface_drake/common/sensor_data.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"

// RL相关头文件
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <memory>
#include <iomanip>
#include <cmath>

// OpenVINO推理相关头文件
#ifdef USE_OPENVINO
#include <openvino/openvino.hpp>
#endif

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;
  std::mutex head_mtx;

  // 辅助函数：获取完整节点名
  static std::string fullyQualifiedNodeName(const std::string &name)
  {
    if (!name.empty() && name[0] == '/')
      return name;
    std::string ns = ros::this_node::getNamespace();
    if (ns.empty() || ns == "/")
      return "/" + name;
    if (ns.back() == '/')
      return ns + name;
    return ns + "/" + name;
  }

  // 关闭/启动 humanoid_joy_control_auto_gait_with_vel 节点
  static void stopJoyAutoGaitNode()
  {
    const std::string node = fullyQualifiedNodeName("humanoid_joy_control_auto_gait_with_vel");
    const std::string cmd = std::string("rosnode kill ") + node + " >/dev/null 2>&1";
    int ret = std::system(cmd.c_str());
    if (ret == 0)
      ROS_INFO("[JoyAutoGait] stop: success: %s", node.c_str());
    else
      ROS_WARN("[JoyAutoGait] stop: failed: %s", node.c_str());
  }

  static void startJoyAutoGaitNode()
  {
    // 直接通过 rosrun 启动（依赖参数已由 launch 配置到参数服务器）
    const std::string cmd = "rosrun humanoid_interface_ros humanoid_joy_control_auto_gait_with_vel >/dev/null 2>&1 &";
    int ret = std::system(cmd.c_str());
    if (ret == 0)
      ROS_INFO("[JoyAutoGait] start: success");
    else
      ROS_ERROR("[JoyAutoGait] start: failed");
  }

  static void callSimStartSrv(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 等待服务可用
    std::cout << "Waiting for sim_start service..." << std::endl;
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
  void humanoidController::keyboard_thread_func()
  {
    usleep(100000);
    struct sched_param param;
    param.sched_priority = 0;
    auto result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (result != 0)
    {
      std::cerr << "Failed to set keyboard_thread_func's scheduling parameters. Error: " << strerror(result) << std::endl;
    }
    stop_pub_ = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);

    char Walk_Command = '\0';
    while (ros::ok())
    {
      if (hardware_status_ != 1)
      {
        usleep(100000);
        continue;
      }
      if (kbhit())
      {
        Walk_Command = getchar();
        std::cout << "[keyboard command]: " << Walk_Command << std::endl;
        if (Walk_Command == 'x')
        {
          std::cout << "x" << std::endl;
          for (int i = 0; i < 5; i++)
          {
            std::cout << "publish stop message" << std::endl;
            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            stop_pub_.publish(stop_msg);
            ros::Duration(0.1).sleep();
          }
        }
        else if (Walk_Command == 'f')
        {
          wbc_only_ = !wbc_only_;
          std::cout << "start using mpc: " << !wbc_only_ << std::endl;
        }
        else if (Walk_Command == 'r')
        {
          std::cerr << "reset MPC " << std::endl;
          reset_mpc_ = true;
        }
        else if (Walk_Command == 'l')
        {
          if (!rl_available_) {
            std::cerr << "RL controller is not available. RL parameter file not found." << std::endl;
          } else {
            std::cerr << "into  rl " << std::endl;
            is_rl_controller_buffer_.setBuffer(!is_rl_controller_buffer_.get());
            Walkenable_ = false;
          }
        }
        else if (Walk_Command == 'g')
        {
          std::cout << "reset estimator" << std::endl;
          reset_estimator_ = true;
        }
        

        Walk_Command = '\0';
      }
      usleep(50000);
    }
  }

  bool humanoidController::init(HybridJointInterface *robot_hw, ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    RobotVersion rb_version(3, 4);
    if (controllerNh_.hasParam("/robot_version"))
    {
        int rb_version_int;
        controllerNh_.getParam("/robot_version", rb_version_int);
        rb_version = RobotVersion::create(rb_version_int);
    }
    is_nodelet_node_ = is_nodelet_node;
    drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
    kuavo_settings_ = drake_interface_->getKuavoSettings();
    scalar_t comHeight = drake_interface_->getIntialHeight();
    ros::param::set("/com_height", comHeight);
    
    // 初始化控制器列表
    available_controllers_.clear();
    available_controllers_.push_back("mpc");  // 默认MPC控制器
    current_controller_ = "mpc";
    current_controller_index_ = 0;
    is_rl_controller_buffer_.setBuffer(false);

    if (ros::param::has("/timeout_warning_ms"))
    {
      ros::param::get("/timeout_warning_ms", timeout_warning_ms_);
    }
    if (ros::param::has("/pull_up_force_threshold"))
    {
      ros::param::get("/pull_up_force_threshold", pull_up_force_threshold_);
      std::cout << "pull_up_force_threshold: " << pull_up_force_threshold_ << std::endl;
    }
    if (ros::param::has("/enable_pull_up_protect"))
    {
      ros::param::get("/enable_pull_up_protect", enable_pull_up_protect_);
      std::cout << "enable_pull_up_protect: " << enable_pull_up_protect_ << std::endl;
    }
    if (ros::param::has("/torso_interpolation_max_velocity"))
    {
      ros::param::get("/torso_interpolation_max_velocity", torso_interpolation_max_velocity_);
      std::cout << "torso_interpolation_max_velocity: " << torso_interpolation_max_velocity_ << " m/s" << std::endl;
    }
    if (ros::param::has("/arm_interpolation_max_velocity"))
    {
      ros::param::get("/arm_interpolation_max_velocity", arm_interpolation_max_velocity_);
      std::cout << "arm_interpolation_max_velocity: " << arm_interpolation_max_velocity_ << " rad/s" << std::endl;
    }

    auto &motor_info = kuavo_settings_.hardware_settings;
    headNum_ = motor_info.num_head_joints;
    waistNum_ = motor_info.num_waist_joints;
    armNumReal_ = motor_info.num_arm_joints;
    jointNumReal_ = motor_info.num_joints - headNum_ - armNumReal_ - waistNum_;
    is_roban_ = (motor_info.robot_module == "ROBAN2") ? true: false;
    std::string imu_type_str = motor_info.getIMUType(rb_version);
    if (imu_type_str == "xsens")
    {
      imuType_ = 2;
    }
    else if (imu_type_str == "hipnuc") 
    {
      imuType_ = 1;
    }
    else
    {
      imuType_ = 0;
    }
    actuatedDofNumReal_ = jointNumReal_ + armNumReal_ + waistNum_ + headNum_;
    ros::param::set("/armRealDof",  static_cast<int>(armNumReal_));
    ros::param::set("/legRealDof",  static_cast<int>(jointNumReal_));
    ros::param::set("/waistRealDof",  static_cast<int>(waistNum_));
    ros::param::set("/headRealDof",  static_cast<int>(headNum_));
    auto [plant, context] = drake_interface_->getPlantAndContext();
    ros_logger_ = new TopicLogger(controller_nh);
    controllerNh_ = controller_nh;
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    std::string gaitCommandFile;
    std::string rlParamFile;
    controllerNh_.getParam("/network_model_file", networkModelPath_);
    std::cout << "networkModelPath_" << networkModelPath_ ;
    controllerNh_.getParam("/rl_param", rlParamFile);
    controllerNh_.getParam("/urdfFile", urdfFile);
    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/referenceFile", referenceFile);
    controllerNh_.getParam("/gaitCommandFile", gaitCommandFile);
    controllerNh_.getParam("/use_external_mpc", use_external_mpc_);
    double controlFrequency = 500.0; // 1000Hz
    inferenceFrequencyRL_ = 100.0;      // 默认100Hz
    controllerNh_.getParam("/wbc_frequency", controlFrequency);
    if(controllerNh_.hasParam("/visualize_humanoid"))
      controllerNh_.getParam("/visualize_humanoid", visualizeHumanoid_);
    dt_ = 1.0 / controlFrequency;
    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
      // if(!is_real_) waistNum_ = 0; // 仿真环境下, 腰部自由度通过xml设置为fixed
      controllerNh_.getParam("/cali", is_cali_);

    }
    else
    {
      // waistNum_ = 0; // 仿真环境下, mujoco默认未设置 /real 变量
    }
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);

    }
    if (controllerNh_.hasParam("channel_map_path"))
    {
      std::string channel_map_path;
      controllerNh_.getParam("channel_map_path", channel_map_path);
      ROS_INFO_STREAM("Loading joystick mapping from " << channel_map_path);
      loadJoyJsonConfig(channel_map_path);
    }
    else
    {
      ROS_WARN_STREAM("No channel_map_path parameter found, using default joystick mapping.");
    }
    if (controllerNh_.hasParam("joystick_sensitivity"))
    {
      controllerNh_.getParam("joystick_sensitivity", joystickSensitivity);
      ROS_INFO_STREAM("Loading joystick sensitivity: " << joystickSensitivity);
    }
    else
    {
      ROS_WARN_STREAM("No input sensitivity parameter found, using default joystick sensitivity.");
    }
    Eigen::Vector4d joystickFilterCutoffFreq_(joystickSensitivity, joystickSensitivity,
                                              joystickSensitivity, joystickSensitivity);
    joystickFilterRL_.setParams(0.01, joystickFilterCutoffFreq_);
    oldJoyMsg_.axes = std::vector<float>(8, 0.0); // 假设有 8 个轴，默认值为 0.0
    oldJoyMsg_.buttons = std::vector<int32_t>(12, 0);
    size_t buffer_size = (is_play_back_mode_) ? 20 : 5;
    if (controllerNh_.hasParam("use_joint_filter"))
    {
      controllerNh_.getParam("/use_joint_filter", use_joint_filter_);
    }

    controllerNh_.param<bool>("/use_shm_communication", use_shm_communication_, false);
    // 初始化共享内存通讯
    if (use_shm_communication_) {
        shm_manager_ = std::make_unique<gazebo_shm::ShmManager>();
        if (!shm_manager_->initializeSensorsShm() || !shm_manager_->initializeCommandShm()) {
            ROS_ERROR("Failed to initialize shared memory communication");
            return false;
        }
        ROS_INFO("Shared memory communication initialized successfully");
    }

    if (controllerNh_.hasParam("use_estimator_contact"))
    {
      controllerNh_.getParam("/use_estimator_contact", use_estimator_contact_);
    }

    if (controllerNh_.hasParam("/only_half_up_body")) {
      controllerNh_.getParam("/only_half_up_body", only_half_up_body_);
    }
    // 半身模式下初始化插值变量
    if (only_half_up_body_) {
      half_body_arm_interpolation_start_pos_.resize(armNumReal_);
      half_body_arm_interpolation_target_pos_.resize(armNumReal_);
      half_body_arm_interpolation_last_target_pos_.resize(armNumReal_);
      half_body_arm_interpolation_start_pos_.setZero();
      half_body_arm_interpolation_target_pos_.setZero();
      half_body_arm_interpolation_last_target_pos_.setZero();
    }
    if (controllerNh_.hasParam("/stand_up_protect"))
    {
      controllerNh_.getParam("/stand_up_protect", stand_up_protect_);
      std::cout << "get stand up protect param: " << stand_up_protect_ << std::endl;
    }
    // trajectory_publisher_ = new TrajectoryPublisher(controller_nh, 0.001);

    wheel_arm_robot_ = drake_interface_->getKuavoSettings().running_settings.only_half_up_body;
    
    // size_t buffer_size = (is_play_back_mode_) ? 20 + waistNum_ : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);
    gaitManagerPtr_ = new GaitManager(20 + waistNum_);
    gaitManagerPtr_->add(0.0, "stance");
    bool verbose = false;
    loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);
    loadData::loadCppDataType(taskFile, "contact_cst_st", contact_cst_st_);
    loadData::loadCppDataType(taskFile, "contact_cst_et", contact_cst_et_);

#ifdef KUAVO_CONTROL_LIB_FOUND
    joint_filter_ptr_ = new HighlyDynamic::JointFilter(&plant, &kuavo_settings_, 12, dt_, ros_logger_);
#endif
    setupHumanoidInterface(taskFile, urdfFile, referenceFile, gaitCommandFile, verbose, rb_version);
    ros::NodeHandle nh;
    setupMpc();
    setupMrt();
    // Visualization
    CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
    robotMass_ = HumanoidInterface_->getCentroidalModelInfo().robotMass;
    std::cout << "HumanoidInterface_->getCentroidalModelInfo().robotMass:" << HumanoidInterface_->getCentroidalModelInfo().robotMass << std::endl;

    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        HumanoidInterface_->modelSettings().contactNames3DoF);
    eeSpatialKinematicsPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping, 
                                                                                      HumanoidInterface_->modelSettings().contactNames6DoF);
    
    robotVisualizer_ = std::make_shared<HumanoidVisualizer>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), 
                                                            *eeKinematicsPtr_, *eeSpatialKinematicsPtr_, controllerNh_, taskFile);

    pinocchioInterface_ptr_ = new PinocchioInterface(HumanoidInterface_->getPinocchioInterface());
    centroidalModelInfo_ = HumanoidInterface_->getCentroidalModelInfo();
    eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_);

    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    jointNum_ = HumanoidInterface_->modelSettings().mpcLegsDof;
    armNum_ = info.actuatedDofNum - jointNum_ - waistNum_;
    if (armNumReal_ + jointNumReal_ != jointNum_ + armNum_) // mpc维度和实际维度不一致，简化的模型
    {
      is_simplified_model_ = true;
      // std::cout << "[HumanoidController]: using simplified mpc model" << std::endl;
      // std::cout << "jointNumReal_:" << jointNumReal_ << " jointNum_:" << jointNum_ << std::endl;
      // std::cout << "headNum_:" << headNum_ << std::endl;
      // std::cout << "armNumReal_:" << armNumReal_ << " armNum_:" << armNum_<< std::endl;
      armDofMPC_ = armNum_ / 2;
      armDofReal_ = armNumReal_ / 2;
      armDofDiff_ = armDofReal_ - armDofMPC_;
      simplifiedJointPos_ = vector_t::Zero(armDofDiff_*2);
    }
    defalutJointPos_.resize(info.actuatedDofNum);
    sensor_data_head_.resize_joint(headNum_);
    sensor_data_waist_.resize_joint(waistNum_);
    joint_kp_.resize(actuatedDofNumReal_);
    joint_kd_.resize(actuatedDofNumReal_);
    joint_kp_walking_.resize(actuatedDofNumReal_);
    joint_kd_walking_.resize(actuatedDofNumReal_);
    head_kp_.resize(headNum_);
    head_kd_.resize(headNum_); 
    waist_kp_.resize(waistNum_);
    waist_kd_.resize(waistNum_);

    jointArmNum_ = info.actuatedDofNum - jointNum_ - waistNum_;
    jointTorqueCmdRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    jointTorqueCmdRL_.setZero();
    initialStateRL_.resize(12 + jointNumReal_ + waistNum_ + armNumReal_);
    defalutJointPosRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    defalutArmPosMPC_.resize(armNumReal_);
    JointControlModeRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    JointControlModeRL_.setZero();
    JointPDModeRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    JointPDModeRL_.setZero();
    jointKpRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    jointKdRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    torqueLimitsRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    actionScaleTestRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    jointCmdFilterStateRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    sensor_data_headRL_.resize_joint(headNum_);
    head_kpRL_.resize(headNum_);
    head_kdRL_.resize(headNum_);
    output_tauRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
    output_tauRL_.setZero();
    jointPosRL_ = vector_t::Zero(jointNumReal_ + waistNum_ + armNumReal_);
    jointVelRL_ = vector_t::Zero(jointNumReal_ + waistNum_ + armNumReal_);
    jointAccRL_ = vector_t::Zero(jointNumReal_ + waistNum_ + armNumReal_);
    
    // 检查RL参数文件是否存在，只有文件存在时才启用RL功能
    std::ifstream rlParamFileCheck(rlParamFile);
    if (rlParamFileCheck.good()) {
      std::cout << "RL parameter file found: " << rlParamFile << ", enabling RL controller." << std::endl;
      rl_available_ = true;
      loadRLSettings(rlParamFile, verbose, dt_);
      // 初始化RL步态接收器
      rl_gait_receiver_ = std::make_unique<RlGaitReceiver>(controllerNh_, &initialCommandDataRL_);
    } else {
      std::cout << "RL parameter file not found: " << rlParamFile << ", RL controller disabled." << std::endl;
      rl_available_ = false;
    }
    rlParamFileCheck.close();

    joint_control_modes_ = Eigen::VectorXd::Constant(actuatedDofNumReal_, 2);
    output_tau_ = vector_t::Zero(actuatedDofNumReal_);
    output_pos_ = vector_t::Zero(actuatedDofNumReal_);
    output_vel_ = vector_t::Zero(actuatedDofNumReal_);
    Eigen::Vector3d acc_filter_params;
    Eigen::Vector3d gyro_filter_params;
    double arm_joint_pos_filter_cutoff_freq=20,arm_joint_vel_filter_cutoff_freq=20,mrt_joint_vel_filter_cutoff_freq=200;
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
    defalutJointPos_.setZero();
    defalutJointPos_.head(jointNum_) = drake_interface_->getDefaultJointState();
    defalutJointPos_.tail(armNum_) = vector_t::Zero(armNum_);
    currentArmTargetTrajectories_ = {{0.0}, {vector_t::Zero(armNumReal_)}, {vector_t::Zero(info.inputDim)}};

    vector_t drake_q;
    if (is_real_)// 实物从squat姿态开始
      drake_q = drake_interface_->getDrakeSquatState();
    else
      drake_q = drake_interface_->getDrakeState();
    vector_t mujoco_q = vector_t::Zero(drake_q.size());
    mujoco_q << drake_q.segment(4, 3), drake_q.head(4), drake_q.tail(drake_q.size() - 7);
    std::vector<double> robot_init_state_param;
    for (int i = 0; i < drake_q.size(); i++)
    {
      robot_init_state_param.push_back(mujoco_q(i));
    }
    
    auto robot_config = drake_interface_->getRobotConfig();
    AnkleSolverType ankleSolverType = static_cast<AnkleSolverType>(robot_config->getValue<int>("ankle_solver_type"));
    ankleSolver.getconfig(ankleSolverType);
    ros::param::set("robot_init_state_param", robot_init_state_param);
  
    ros::param::set("/humanoid/init_q", robot_init_state_param);

    auto initial_state_ =  drake_interface_->getInitialState();
    auto squat_initial_state_ =  drake_interface_->getSquatInitialState();
    default_state_ = initial_state_;
    std::cout << "controller initial_state_:" << initial_state_.transpose() << std::endl;
    std::cout << "controller squat_initial_state_:" << squat_initial_state_.transpose() << std::endl;
    std::vector<double> initial_state_vector(initial_state_.data(), initial_state_.data() + initial_state_.size());
    std::vector<double> squat_initial_state_vector(squat_initial_state_.data(), squat_initial_state_.data() + squat_initial_state_.size());
    std::vector<double> default_joint_pos_vector(defalutJointPos_.data(), defalutJointPos_.data() + defalutJointPos_.size());
    controllerNh_.setParam("/initial_state", initial_state_vector);
    controllerNh_.setParam("/squat_initial_state", squat_initial_state_vector);
    controllerNh_.setParam("/default_joint_pos", default_joint_pos_vector);

    joint_state_limit_.resize(actuatedDofNumReal_, 2);
    is_swing_arm_ = robot_config->getValue<bool>("swing_arm");
    swing_arm_gain_ = robot_config->getValue<double>("swing_arm_gain");
    swing_elbow_scale_ = robot_config->getValue<double>("swing_elbow_scale");
    ruiwo_motor_velocities_factor_ = robot_config->getValue<double>("motor_velocities_factor");
    gait_map_ = HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getGaitMap();
    std::cout << "gait_map size: " << gait_map_.size() << std::endl;
    defalutArmPosMPC_.setZero();
    loadData::loadEigenMatrix(referenceFile, "joint_kp_", joint_kp_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_", joint_kd_);
    loadData::loadEigenMatrix(referenceFile, "joint_kp_walking_", joint_kp_walking_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_walking_", joint_kd_walking_);
    loadData::loadEigenMatrix(referenceFile, "standJointState", defalutArmPosMPC_);
    std::cout << "defalutArmPosMPC_: " << defalutArmPosMPC_.transpose() << std::endl;
    if (headNum_ > 0)
    {
      loadData::loadEigenMatrix(referenceFile, "head_kp_", head_kp_);
      loadData::loadEigenMatrix(referenceFile, "head_kd_", head_kd_);
      std::vector<std::string> head_joint_names_ = {"zhead_1_joint", "zhead_2_joint"};
      const auto &model = HumanoidInterface_->getPinocchioInterface().getModel();

      for (int i = 0; i < head_joint_names_.size(); i++)
      {
        std::string joint_name = head_joint_names_[i];
        std::pair<double, double> limits = {head_joint_limits_[0].first, head_joint_limits_[0].second};
        if (robotVisualizer_->getJointLimits(joint_name, limits))
        {
          limits.first *= 180.0 / M_PI;
          limits.second *= 180.0 / M_PI;
          head_joint_limits_[i] = limits;
          std::cout << "Head joint " << joint_name << " lower_limit: " << limits.first << " upper_limit: " << limits.second << std::endl;
        }
      }
    }

    loadData::loadEigenMatrix(referenceFile, "acc_filter_cutoff_freq", acc_filter_params);
    loadData::loadEigenMatrix(referenceFile, "gyro_filter_cutoff_freq", gyro_filter_params);
    loadData::loadEigenMatrix(referenceFile, "jointStateLimit", joint_state_limit_);
    loadData::loadCppDataType(referenceFile, "arm_joint_pos_filter_cutoff_freq", arm_joint_pos_filter_cutoff_freq);
    loadData::loadCppDataType(referenceFile, "arm_joint_vel_filter_cutoff_freq", arm_joint_vel_filter_cutoff_freq);
    loadData::loadCppDataType(referenceFile, "mrt_joint_vel_filter_cutoff_freq", mrt_joint_vel_filter_cutoff_freq);
    loadData::loadEigenMatrix(referenceFile, "defaultCotrolMode", joint_control_modes_);


    // Hardware interface
    // TODO: setup hardware controller interface
    
#ifdef USE_DDS
    // Initialize DDS client
    dds_client_ = std::make_unique<HumanoidDDSClientType>();
    
    auto callback = [this](const unitree_hg::msg::dds_::LowState_& data) {
        this->LowStateCallback(data);
    };
    dds_client_->state_listener_->setLowdstateCallback(callback);
    
    // Start the DDS client
    dds_client_->start();
    
    std::cout << "DDS communication initialized" << std::endl;
#elif USE_LEJU_DDS
    // Initialize Leju DDS client
    using LejuDDSClientType = HumanoidControllerDDSClient<leju::msgs::JointCmd, leju::msgs::SensorsData>;
    dds_client_ = std::make_unique<LejuDDSClientType>();

    auto leju_callback = [this](const leju::msgs::SensorsData& data) {
        this->LejuSensorsDataCallback(data);
    };
    dds_client_->state_listener_->setLowdstateCallback(leju_callback);

    // Start the DDS client
    dds_client_->start();

    std::cout << "Leju DDS communication initialized" << std::endl;
#endif

#if !defined(USE_DDS) && !defined(USE_LEJU_DDS)
    std::cout << "DDS communication disabled (compile with -DUSE_DDS or -DUSE_LEJU_DDS to enable)" << std::endl;
#endif
    
    // create a ROS subscriber to receive the joint pos and vel
    jointPos_ = vector_t::Zero(info.actuatedDofNum);
    jointPos_.setZero();
    jointPos_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointPosWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    // jointPosWBC_.setZero();
    jointPosWBC_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointVelWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    jointAccWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    jointCurrentWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);

    jointVel_ = vector_t::Zero(info.actuatedDofNum);
    jointAcc_ = vector_t::Zero(info.actuatedDofNum);
    jointTorque_ = vector_t::Zero(info.actuatedDofNum);
    quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    quat_init = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    arm_joint_pos_cmd_prev_ = vector_t::Zero(armNumReal_);
    arm_joint_pos_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_pos_filter_cutoff_freq));
    arm_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_vel_filter_cutoff_freq));
    mrt_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(info.actuatedDofNum-armNum_, mrt_joint_vel_filter_cutoff_freq));
    acc_filter_.setParams(dt_, acc_filter_params);
    // free_acc_filter_.setParams(dt_, acc_filter_params);
    gyro_filter_.setParams(dt_, gyro_filter_params);
#if !defined(USE_DDS) && !defined(USE_LEJU_DDS)
    // Only subscribe to sensor data via ROS when DDS is not enabled
    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
#endif
    robotLocalizationSub_ = controllerNh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, &humanoidController::robotlocalizationCallback, this);
    mpcStartSub_ = controllerNh_.subscribe<std_msgs::Bool>("/start_mpc", 10, &humanoidController::startMpccallback, this);
    arm_joint_trajectory_.initialize(armNumReal_);
    mm_arm_joint_trajectory_.initialize(armNumReal_);
    arm_joint_traj_sub_ = controllerNh_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, [this](const sensor_msgs::JointState::ConstPtr &msg)
      {
        if (is_rl_controller_ == false)
        {
          if(msg->name.size() != armNumReal_){
            std::cerr << "The dimensin of arm joint pos is NOT equal to the armNumReal_!!" << msg->name.size() << " vs " << armNumReal_ << "\n";
            return;
          }
          for(int i = 0; i < armNumReal_; i++)
          {
            // std::cout << "arm joint pos: " << msg->position[i] << std::endl;
            arm_joint_trajectory_.pos[i] = msg->position[i] * M_PI / 180.0;
            if(msg->velocity.size() == armNumReal_)
              arm_joint_trajectory_.vel[i] = msg->velocity[i] * M_PI / 180.0;
            if(msg->effort.size() == armNumReal_)
              arm_joint_trajectory_.tau[i] = msg->effort[i];
          }
        }
        // std::cout << "arm joint pos: " << arm_joint_trajectory_.pos.size() << std::endl;
      });
      mm_arm_joint_traj_sub_ = controllerNh_.subscribe<sensor_msgs::JointState>("/mm_kuavo_arm_traj", 10, [this](const sensor_msgs::JointState::ConstPtr &msg)
      {
        if(msg->name.size() != armNumReal_){
          std::cerr << "The dimensin of arm joint pos is NOT equal to the armNumReal_!!" << msg->name.size() << " vs " << armNumReal_ << "\n";
          return;
        }
        for(int i = 0; i < armNumReal_; i++)
        {
          // std::cout << "arm joint pos: " << msg->position[i] << std::endl;
          mm_arm_joint_trajectory_.pos[i] = msg->position[i] * M_PI / 180.0;
          if(msg->velocity.size() == armNumReal_)
            mm_arm_joint_trajectory_.vel[i] = msg->velocity[i] * M_PI / 180.0;
          if(msg->effort.size() == armNumReal_)
            mm_arm_joint_trajectory_.tau[i] = msg->effort[i];
        }
        // std::cout << "arm joint pos: " << arm_joint_trajectory_.pos.size() << std::endl;
      });
      // Arm TargetTrajectories
      auto armTargetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg)
      {
        auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

        if (targetTrajectories.stateTrajectory[0].size() != armNumReal_)
        {
          ROS_WARN_STREAM("[humanoidController]:Using simplified model, but arm targetTrajectories size : "
                          << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " != "
                          << std::to_string(armNumReal_) << ", will keep the simplified arm's joints target");
          return;
        }
        currentArmTargetTrajectories_ = targetTrajectories;
      };
      if (is_simplified_model_)// 简化模型需要直接从全部target的topic中去获取被简化关节的target
        arm_target_traj_sub_ =
            controllerNh_.subscribe<ocs2_msgs::mpc_target_trajectories>(robotName_ + "_mpc_arm_commanded", 3, armTargetTrajectoriesCallback);

      gait_scheduler_sub_ = controllerNh_.subscribe<kuavo_msgs::gaitTimeName>(robotName_ + "_mpc_gait_time_name", 10, [this](const kuavo_msgs::gaitTimeName::ConstPtr &msg)
                                                                              {
                                                                              last_gait_ = current_gait_;
            current_gait_.name = msg->gait_name;
            current_gait_.startTime = msg->start_time;
            if (gaitManagerPtr_)
              gaitManagerPtr_->add(current_gait_.startTime, current_gait_.name);
            std::cout << "[controller] receive current gait name: " << current_gait_.name << " start time: " << current_gait_.startTime << std::endl; });
      sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
      head_sub_ = controllerNh_.subscribe("/robot_head_motion_data", 10, &humanoidController::headCmdCallback, this);
      joy_sub_ = controllerNh_.subscribe<sensor_msgs::Joy>("/joy", 10, &humanoidController::joyCallback, this);
      targetTorquePub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetTorque", 10);
      stop_pub = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);
      // rl_control_service_ = controllerNh_.advertiseService("/humanoid_controller/walkenable", &humanoidController::WalkenableCallback, this);
      humanoidStatePublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/humanoid/mm_state", 10);
      cmdPoseWorldPublisher_ = controllerNh_.advertise<geometry_msgs::Twist>("/cmd_pose_world", 10);
      waist_sub_ = controllerNh_.subscribe("/robot_waist_motion_data", 10, &humanoidController::waistCmdCallback, this);
      // Add new subscriber for Float64MultiArray head control
      auto headArrayCallback = [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
          if (msg->data.size() == 2) {
              if (msg->data[0] < head_joint_limits_[0].first || msg->data[0] > head_joint_limits_[0].second 
                  || msg->data[1] < head_joint_limits_[1].first || msg->data[1] > head_joint_limits_[1].second) 
              {
                  std::cout << "\033[1;31m[headArrayCallback] Invalid robot head motion data. Head joints must be in the range [" 
                      << head_joint_limits_[0].first << ", " << head_joint_limits_[0].second << "] and [" 
                      << head_joint_limits_[1].first << ", " << head_joint_limits_[1].second << "].\033[0m" << std::endl;
                  return;
              }
              head_mtx.lock();
              desire_head_pos_[0] = msg->data[0];
              desire_head_pos_[1] = msg->data[1];
              head_mtx.unlock();
          }
          else {
              ROS_WARN("Invalid robot head motion array data. Expected 2 elements, but received %lu elements.", msg->data.size());
          }
      };

      head_array_sub_ = controllerNh_.subscribe<std_msgs::Float64MultiArray>("/robot_head_motion_array", 10, headArrayCallback);
      hand_wrench_sub_ = controllerNh_.subscribe<std_msgs::Float64MultiArray>("/hand_wrench_cmd", 10, [&](const std_msgs::Float64MultiArray::ConstPtr &msg)
        {
          if(msg->data.size() != 12)
            ROS_ERROR("The dimensin of hand wrench cmd is NOT equal to 12!!");
          for(int i = 0; i < 12; i++)
            hand_wrench_cmd_(i) = msg->data[i];
        }
      );
      armJointSynchronizationSrv_ = controllerNh_.advertiseService("/arm_joint_synchronization", &humanoidController::armJointSynchronizationCallback, this); 
      enableArmCtrlSrv_ = controllerNh_.advertiseService("/enable_wbc_arm_trajectory_control", &humanoidController::enableArmTrajectoryControlCallback, this);
      enableMmArmCtrlSrv_ = controllerNh_.advertiseService("/enable_mm_wbc_arm_trajectory_control", &humanoidController::enableMmArmTrajectoryControlCallback, this);
      getMmArmCtrlSrv_ = controllerNh_.advertiseService("/get_mm_wbc_arm_trajectory_control", &humanoidController::getMmArmCtrlCallback, this);
      jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
      imuPub_ = controllerNh_.advertise<sensor_msgs::Imu>("/imu_data", 10);
      kinematicPub_ = controllerNh_.advertise<nav_msgs::Odometry>("/kinematic_data", 10);
      mpcPolicyPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);
      feettargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("/humanoid_controller/feet_target_policys", 10, true);

      wbcFrequencyPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/frequency/wbc", 10);
      wbcTimeCostPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/time_cost/wbc", 10);
      wbc_observation_publisher_ = controllerNh_.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_wbc_observation", 1);
      sensor_data_raw_pub_ = controllerNh_.advertise<kuavo_msgs::sensorsData>("/share_memory/sensor_data_raw", 10);
      lHandWrenchPub_ = controllerNh_.advertise<geometry_msgs::WrenchStamped>("/hand_wrench/left_hand", 10);
      rHandWrenchPub_ = controllerNh_.advertise<geometry_msgs::WrenchStamped>("/hand_wrench/right_hand", 10);
      currentGaitNameSrv_ = controllerNh_.advertiseService(robotName_ + "_get_current_gait_name", 
        &humanoidController::getCurrentGaitNameCallback, this);
      switchControllerSrv_ = controllerNh_.advertiseService("/humanoid_controller/switch_controller", 
        &humanoidController::switchControllerCallback, this);
      getControllerListSrv_ = controllerNh_.advertiseService("/humanoid_controller/get_controller_list", 
        &humanoidController::getControllerListCallback, this);
      switchToNextControllerSrv_ = controllerNh_.advertiseService("/humanoid_controller/switch_to_next_controller", 
        &humanoidController::switchToNextControllerCallback, this);
      arm_control_mode_sub_ = controllerNh_.subscribe<std_msgs::Float64MultiArray>("/humanoid/mpc/arm_control_mode", 10,[&](const std_msgs::Float64MultiArray::ConstPtr &msg)
      {
        if(msg->data.size() == 0)
        {
          ROS_ERROR("The dimensin of arm control mode is 0!!");
          return;
        }
        if (msg->data[0] != mpcArmControlMode_)
        {
          mpcArmControlMode_ = static_cast<ArmControlMode>(msg->data[0]);
          std::cout << "[controller] mpc arm control mode changed to: " << mpcArmControlMode_ << std::endl;
        }
        if (msg->data[1] != mpcArmControlMode_desired_)
        {
          mpcArmControlMode_desired_ = static_cast<ArmControlMode>(msg->data[1]);
          std::cout << "[controller] mpc arm control mode desired changed to: " << mpcArmControlMode_desired_ << std::endl;
        }
      });
      
      armEefWbcPosePublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/humanoid_controller/wbc_arm_eef_pose", 10, true);
      // dexhand state
      dexhand_state_sub_ = controllerNh_.subscribe("/dexhand/state", 10, &humanoidController::dexhandStateCallback, this);

      standUpCompletePub_ = controllerNh_.advertise<std_msgs::Int8>("/bot_stand_up_complete", 10);
      
      enable_mpc_sub_ = controllerNh_.subscribe("/enable_mpc_flag", 10, &humanoidController::getEnableMpcFlagCallback, this);
      enable_wbc_sub_ = controllerNh_.subscribe("/enable_wbc_flag", 10, &humanoidController::getEnableWbcFlagCallback, this);

      // State estimation
      setupStateEstimate(taskFile, verbose);
      if (use_shm_communication_)
      {
        while (!sensors_data_buffer_ptr_->isReady())
        {
          updateSensorDataFromShm();
          usleep(1000);
          // std::cout << "update for sensors data from shm" << std::endl;
        }
        
      }
      else
        sensors_data_buffer_ptr_->waitForReady();
      // std::cout << "waitForReady estimate ready" << std::endl;
      // Whole body control/HierarchicalWbc/WeightedWbc
      // wbc 中 eeKinematicsPtr_ 可能需要修改
      wbc_ = std::make_shared<WeightedWbc>(*pinocchioInterfaceWBCPtr_, centroidalModelInfoWBC_,
                                           *eeKinematicsWBCPtr_);
      wbc_->setArmNums(armNumReal_);
      wbc_->setWaistNums(waistNum_);
      if (motor_info.robot_module == "ROBAN2")
        wbc_->setRobanMode(true);
      wbc_->loadTasksSetting(taskFile, verbose, is_real_);
      if (only_half_up_body_) {
        wbc_->setHalfBodyMode(true);
      }

      standUpWbc_ = std::make_shared<StandUpWbc>(*pinocchioInterfaceWBCPtr_, centroidalModelInfoWBC_,
                                                 *eeKinematicsWBCPtr_);
      standUpWbc_->setArmNums(armNumReal_);
      standUpWbc_->setWaistNums(waistNum_);
      standUpWbc_->loadTasksSetting(taskFile, verbose, is_real_);

      // preupdate
      curRobotLegState_ = vector_t::Zero(centroidalModelInfoWBC_.stateDim);

      // Safety Checker
      safetyChecker_ = std::make_shared<SafetyChecker>(HumanoidInterface_->getCentroidalModelInfo());
      keyboardThread_ = std::thread(&humanoidController::keyboard_thread_func, this);
      if (!keyboardThread_.joinable())
      {
        std::cerr << "Failed to start keyboard thread" << std::endl;
        exit(1);
      }

    
      singleInputDataRL_.resize(numSingleObsRL_);
      networkInputDataRL_.resize(numSingleObsRL_ * frameStackRL_);
      commandPhaseRL_.resize(2);
      actionsRL_.resize(jointNumReal_ + waistNum_ + armNumReal_);
      singleInputDataRL_.setZero();
      networkInputDataRL_.setZero();
      actionsRL_.setZero();
      humanoidState_.resize(6 + jointArmNum_); // base + arm, for kmpc
      for (int i = 0; i < frameStackRL_; i++)
      {
        input_deque.push_back(singleInputDataRL_);
      }
      if (rl_available_)
      {  
        compiled_model_ =
            core_.compile_model(networkModelPath_, "CPU"); // 创建编译模型
        std::string package_path = ros::package::getPath("kuavo_assets");
        std::string version_str = "biped_s" + rb_version.to_string();
        std::string urdf_path = package_path + "/models/" + version_str + "/urdf/" + version_str + ".urdf";
        arm_torque_controller_.reset(new ArmTorqueController(urdf_path, jointKpRL_.segment(jointNumReal_ + waistNum_, armNumReal_), jointKdRL_.segment(jointNumReal_ + waistNum_, armNumReal_))); // 使用智能指针初始化，只传入手臂关节参数
      }
      
      desire_arm_q_ = defalutJointPosRL_.segment(jointNumReal_+ waistNum_, armNumReal_);
      desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);
      // 初始化手臂插值相关变量
      initArmInterpolation();
      // 初始化速度平滑系统
      initializeVelocitySmoothing();
  
      // 初始化原地踏步系统
      in_place_step_velocity_.linear.x = 0.0;
      in_place_step_velocity_.linear.y = 0.0;
      in_place_step_velocity_.linear.z = 0.0;
      in_place_step_velocity_.angular.x = 0.0;
      in_place_step_velocity_.angular.y = 0.0;
      in_place_step_velocity_.angular.z = 0.0;
  
      // 从参数服务器读取原地踏步参数
      if (controllerNh_.hasParam("in_place_step_duration"))
      {
        controllerNh_.getParam("in_place_step_duration", in_place_step_duration_);
        ROS_INFO_STREAM("[InPlaceStepping] 原地踏步持续时间: " << in_place_step_duration_ << "秒");
      }
        
      if (controllerNh_.hasParam("enable_in_place_stepping"))
      {
        controllerNh_.getParam("enable_in_place_stepping", enable_in_place_stepping_);
        ROS_INFO_STREAM("[InPlaceStepping] 原地踏步功能: " << (enable_in_place_stepping_ ? "启用" : "禁用"));
      }
        
      if (controllerNh_.hasParam("stance_transition_duration"))
      {
        controllerNh_.getParam("stance_transition_duration", stance_transition_duration_);
        ROS_INFO_STREAM("[StanceTransition] 站立过渡持续时间: " << stance_transition_duration_ << "秒");
      }
  
  
      // 初始化LB解锁保护系统
      lb_just_unlocked_ = false;
      lb_unlock_time_ = ros::Time::now();
      ROS_INFO_STREAM("[LBProtection] LB解锁保护时间: " << lb_unlock_protection_duration_ << "秒");
  
      auto jointStateCallback = [this](const sensor_msgs::JointState::ConstPtr& msg)
      {
        if( is_rl_controller_ == true)
        {
          // std::cout << "jointStateCallback" << std::endl;
          for (size_t i = 0; i < msg->name.size(); ++i)
          {
            // std::cout << "joint name: " << msg->name[i] << " joint position: " << msg->position[i] << std::endl;
            desire_arm_q_(i) = msg->position[i] * M_PI / 180.0;
            if(msg->velocity.size() == armNumReal_)
              desire_arm_v_(i) = 0;
              // desire_arm_v_(i) = msg->velocity[i] * M_PI / 180.0;
          }
        } 
      };
      joint_sub_ = controllerNh_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, jointStateCallback); // 初始化订阅者
  
      // 设置CPU内核隔离
      if (is_real_)
      {
        if (!setupCpuIsolation())
        {
          // 提示用户配置CPU内核隔离
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
    
    return true;
  }

  void humanoidController::loadRLSettings(const std::string &rlParamFile, bool verbose, double dt)
  {
    bool enable_ = false;
    double index_ = 0, startIdx_ = 0, mumIdx_ = 0, obsScale_ = 0;
    int num_ = 0;
    std::string networkModelFile_;
    Eigen::Vector3d accFilterCutoffFreqRL_, freeAccFilterCutoffFreqRL_, gyroFilterCutoffFreqRL_;
    Eigen::VectorXd defaultBaseStateRL_(12), jointCmdFilterCutoffFreqRL_(jointNumReal_ + armNumReal_ + waistNum_);
    CommandDataRL commandDataRL_;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(rlParamFile, pt);
    // 使用新的 helper 函数来简化数据加载
    auto loadEigenMatrix = [&](const std::string &key, auto &matrix)
    {
      loadData::loadEigenMatrix(rlParamFile, key, matrix);
    };
    loadEigenMatrix("defaultJointState", defalutJointPosRL_);
    loadEigenMatrix("defaultBaseState", defaultBaseStateRL_);
    loadEigenMatrix("JointControlMode", JointControlModeRL_);
    loadEigenMatrix("JointPDMode", JointPDModeRL_);
    loadEigenMatrix("jointKp", jointKpRL_);
    loadEigenMatrix("jointKd", jointKdRL_);
    loadEigenMatrix("torqueLimits", torqueLimitsRL_);
    loadEigenMatrix("actionScaleTest", actionScaleTestRL_);
    loadEigenMatrix("accFilterCutoffFreq", accFilterCutoffFreqRL_);
    loadEigenMatrix("freeAccFilterCutoffFreq", freeAccFilterCutoffFreqRL_);
    loadEigenMatrix("gyroFilterCutoffFreq", gyroFilterCutoffFreqRL_);
    loadEigenMatrix("jointCmdFilterCutoffFreq", jointCmdFilterCutoffFreqRL_);
    loadEigenMatrix("jointCmdFilterState", jointCmdFilterStateRL_);
    loadEigenMatrix("accFilterState", accFilterStateRL_);
    loadEigenMatrix("freeAccFilterState", freeAccFilterStateRL_);
    loadEigenMatrix("gyroFilterState", gyroFilterStateRL_);
    loadEigenMatrix("velocityLimits", velocityLimitsRL_);
    ros::param::set("/humanoid_controller/velocityLimits", std::vector<double>(velocityLimitsRL_.data(), velocityLimitsRL_.data() + velocityLimitsRL_.size()));
    // loadEigenMatrix("initalCommand", initalCommand_);
    // loadEigenMatrix("commandScale", commandScale_);
    loadData::loadCppDataType(rlParamFile, "actionScale", actionScaleRL_);
    loadData::loadCppDataType(rlParamFile, "frameStack", frameStackRL_);
    loadData::loadCppDataType(rlParamFile, "numSingleObs", numSingleObsRL_);
    loadData::loadCppDataType(rlParamFile, "cycleTime", cycleTimeRL_);
    loadData::loadCppDataType(rlParamFile, "cycleTime_short", cycleTime_shortRL_);  
    loadData::loadCppDataType(rlParamFile, "switch_ratio", switch_ratioRL_); 
    loadData::loadCppDataType(rlParamFile, "phase", phaseRL_);
    loadData::loadCppDataType(rlParamFile, "episodeLength", episodeLengthRL_);
    loadData::loadCppDataType(rlParamFile, "clipObservations", clipObservationsRL_);
    loadData::loadCppDataType(rlParamFile, "clipActions", clipActionsRL_);
    loadData::loadCppDataType(rlParamFile, "withArm", withArmRL_);
    loadData::loadCppDataType(rlParamFile, "inferenceFrequency", inferenceFrequencyRL_);
    loadData::loadCppDataType(rlParamFile, "networkModelFile", networkModelFile_);
    loadData::loadCppDataType(rlParamFile, "defaultBaseHeightControl", defaultBaseHeightControl_);
    std::cout << "defaultBaseHeightControl: " << defaultBaseHeightControl_ << std::endl;
    // 加载手臂插值配置
    if (pt.find("armInterpolationDuration") != pt.not_found()) {
      loadData::loadCppDataType(rlParamFile, "armInterpolationDuration", interpolation_duration_);
    } else {
      interpolation_duration_ = 1.0; // 默认2秒，更平滑
    }
    // loadData::loadCppDataType(rlParamFile, "ankleSolverType", ankleSolverType_);

    // ankleSolver.getconfig(ankleSolverType_);
    initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;
    networkModelPath_ = networkModelPath_ + networkModelFile_;
    accFilterRL_.setParams(dt, accFilterCutoffFreqRL_);
    freeAccFilterRL_.setParams(dt, freeAccFilterCutoffFreqRL_);
    gyroFilterRL_.setParams(dt, gyroFilterCutoffFreqRL_);
    jointCmdFilterRL_.setParams(dt, jointCmdFilterCutoffFreqRL_);

    // 加载命令数据
    const std::string prefixCommandData_ = "commandData";
    const std::vector<std::pair<std::string, double CommandDataRL::*>> cmdInitalList = {
        {"cmdVelLineX", &CommandDataRL::cmdVelLineX_},
        {"cmdVelLineY", &CommandDataRL::cmdVelLineY_},
        {"cmdVelLineZ", &CommandDataRL::cmdVelLineZ_},
        {"cmdVelAngularX", &CommandDataRL::cmdVelAngularX_},
        {"cmdVelAngularY", &CommandDataRL::cmdVelAngularY_},
        {"cmdVelAngularZ", &CommandDataRL::cmdVelAngularZ_},
        {"cmdStance", &CommandDataRL::cmdStance_},
    };
    const std::vector<std::pair<std::string, double CommandDataRL::*>> cmdScaleList = {
        {"cmdVelLineX", &CommandDataRL::cmdVelScaleLineX_},
        {"cmdVelLineY", &CommandDataRL::cmdVelScaleLineY_},
        {"cmdVelLineZ", &CommandDataRL::cmdVelScaleLineZ_},
        {"cmdVelAngularX", &CommandDataRL::cmdVelScaleAngularX_},
        {"cmdVelAngularY", &CommandDataRL::cmdVelScaleAngularY_},
        {"cmdVelAngularZ", &CommandDataRL::cmdVelScaleAngularZ_},
        {"cmdStance", &CommandDataRL::cmdScaleStance_}};
    for (const auto &[cmdName, cmdMember] : cmdInitalList)
    {
      loadData::loadPtreeValue(pt, commandDataRL_.*cmdMember, prefixCommandData_ + ".inital." + cmdName, verbose);
    }
    for (const auto &[cmdName, cmdMember] : cmdScaleList)
    {
      loadData::loadPtreeValue(pt, commandDataRL_.*cmdMember, prefixCommandData_ + ".scale." + cmdName, verbose);
    }
    initialCommandDataRL_ = commandDataRL_;
    setCommandDataRL(initialCommandDataRL_);
    // 加载单输入数据
    const std::string prefixSingleInputData_ = "singleInputData";
    if (verbose)
    {
      std::cerr << "\n #### singleInputData:";
      std::cerr << "\n #### =============================================================================\n";
    }
    for (const auto &pair : pt)
    {
      if (pair.first == prefixSingleInputData_)
      {
        for (const auto &pair2 : pair.second)
        {
          singleInputDataRLKeys.push_back(pair2.first);
          loadData::loadPtreeValue(pt, startIdx_, prefixSingleInputData_ + "." + pair2.first + ".startIdx", verbose);
          loadData::loadPtreeValue(pt, mumIdx_, prefixSingleInputData_ + "." + pair2.first + ".numIdx", verbose);
          loadData::loadPtreeValue(pt, obsScale_, prefixSingleInputData_ + "." + pair2.first + ".obsScales", verbose);
          num_ += mumIdx_;
          singleInputDataRLID_[pair2.first] = {startIdx_, mumIdx_, obsScale_};
        }
      }
    }
    if (num_ != numSingleObsRL_)
    {
      std::cerr << "Error: singleInputData number is not equal to 'numSingleObsRL_'" << std::endl;
      std::cerr << "Single Obs Number: " << num_ << std::endl;
      std::cerr << "'numSingleObsRL_' Number: " << numSingleObsRL_ << std::endl;
      exit(1);
    }
    
    // Add RL model to controller list
    if (!networkModelFile_.empty()) {
      std::string rl_controller_name = "rl_" + networkModelFile_;
      // Check if already exists to avoid duplicates
      bool exists = false;
      for (const auto& controller : available_controllers_) {
        if (controller == rl_controller_name) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        available_controllers_.push_back(rl_controller_name);
        ROS_INFO("Added RL controller to list: %s", rl_controller_name.c_str());
      }
    }
  }
  void humanoidController::headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg)
  {
      if (msg->joint_data.size() ==2)
      {
          if (msg->joint_data[0] < head_joint_limits_[0].first || msg->joint_data[0] > head_joint_limits_[0].second 
            || msg->joint_data[1] < head_joint_limits_[1].first || msg->joint_data[1] > head_joint_limits_[1].second)
          {
              // std::cout << "\033[1;31m[headCmdCallback] Invalid robot head motion data. Head joints must be in the range [" 
              //   << head_joint_limits_[0].first << ", " << head_joint_limits_[0].second << "] and [" 
              //   << head_joint_limits_[1].first << ", " << head_joint_limits_[1].second << "].\033[0m" << std::endl;
              return;
          }
          head_mtx.lock();
          desire_head_pos_[0] = msg->joint_data[0]*M_PI/180.0;
          desire_head_pos_[1] = msg->joint_data[1]*M_PI/180.0;
          head_mtx.unlock();
      }
      else
      {
          ROS_WARN("Invalid robot head motion data. Expected 2 elements, but received %lu elements.", msg->joint_data.size());
      }
  }
  void humanoidController::startMpccallback(const std_msgs::Bool::ConstPtr &msg)
  {
    ROS_INFO_STREAM("start_mpc: " << msg->data);
    bool start_mpc_ = msg->data;
    wbc_only_ = !start_mpc_;
  }
  void humanoidController::publishFeetTrajectory(const TargetTrajectories &targetTrajectories)
  {
    auto &stateTrajectory = targetTrajectories.stateTrajectory;
    auto &inputTrajectory = targetTrajectories.inputTrajectory;
    auto &timeTrajectory = targetTrajectories.timeTrajectory;
    TargetTrajectories pubFeetTrajectories;
    pubFeetTrajectories.timeTrajectory = timeTrajectory;
    pubFeetTrajectories.stateTrajectory.clear();
    for (size_t j = 0; j < stateTrajectory.size(); j++)
    {
      const auto state = stateTrajectory.at(j);
      // Fill feet msgs
      const auto &model = pinocchioInterface_ptr_->getModel();
      auto &data = pinocchioInterface_ptr_->getData();
      const auto &q = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_);

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      const auto feetPositions = eeKinematicsPtr_->getPosition(state);
      vector_t feetPositions_vec(3 * centroidalModelInfo_.numThreeDofContacts);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        feetPositions_vec.segment(3 * i, 3) = feetPositions[i];
      }
      pubFeetTrajectories.stateTrajectory.push_back(feetPositions_vec);
    }

    const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(pubFeetTrajectories);
    feettargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
  }
  void humanoidController::robotlocalizationCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    nav_msgs::Odometry robot_localization_ = *msg;
    std::lock_guard<std::mutex> lock(robotlocalization_data_mutex_);
    robotlocalizationDataQueue.push(robot_localization_);
  }

#ifdef USE_DDS
  void humanoidController::LowStateCallback(const unitree_hg::msg::dds_::LowState_& data)
  {
    SensorData sensor_data;
    sensor_msgs::Imu imu_msg;
    sensor_data.resize_joint(jointNumReal_+armNumReal_+headNum_);
    // JOINT DATA
    for (size_t i = 0; i < jointNumReal_+armNumReal_+headNum_; ++i)
    {
      sensor_data.jointPos_(i) = data.motor_state()[i].q();
      sensor_data.jointVel_(i) = data.motor_state()[i].dq();
      sensor_data.jointAcc_(i) = data.motor_state()[i].ddq();
      sensor_data.jointTorque_(i) = data.motor_state()[i].tau_est();
    }
    // Convert timestamp from reserve fields: [0]=seconds, [1]=nanoseconds
    uint32_t timestamp_sec = data.reserve()[0];
    uint32_t timestamp_nsec = data.reserve()[1];
    sensor_data.timeStamp_ = ros::Time(timestamp_sec, timestamp_nsec);
    double sensor_time_diff = 0;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);
    // IMU
    sensor_data.quat_.coeffs().w() = data.imu_state().quaternion()[0];
    sensor_data.quat_.coeffs().x() = data.imu_state().quaternion()[1];
    sensor_data.quat_.coeffs().y() = data.imu_state().quaternion()[2];
    sensor_data.quat_.coeffs().z() = data.imu_state().quaternion()[3];
    sensor_data.angularVel_ << data.imu_state().gyroscope()[0], data.imu_state().gyroscope()[1], data.imu_state().gyroscope()[2];
    sensor_data.linearAccel_ << data.imu_state().accelerometer()[0], data.imu_state().accelerometer()[1], data.imu_state().accelerometer()[2];
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    // if(imuType_ == 2)
    {
      sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
      sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
      
    }
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);

    // std::cout << "sensor_data.jointPos_.size()" << sensor_data.jointPos_.size() << std::endl;
    // std::cout << "jointNumReal_+armNumReal_ + headNum_" << jointNumReal_+armNumReal_ + headNum_ << std::endl;
    if (headNum_ > 0 && sensor_data.jointPos_.size() == jointNumReal_+armNumReal_ + headNum_)
    {
      int head_start_index  =sensor_data.jointPos_.size() - headNum_;
      for (size_t i = 0; i < headNum_; ++i)
      {
        sensor_data_head_.jointPos_(i) = data.motor_state()[head_start_index + i].q();
        sensor_data_head_.jointVel_(i) = data.motor_state()[head_start_index + i].dq();
        sensor_data_head_.jointAcc_(i) = data.motor_state()[head_start_index + i].ddq();
        sensor_data_head_.jointTorque_(i) = data.motor_state()[head_start_index + i].tau_est(); 
      }
    }
    
    // Save latest DDS sensor data for comparison
    latest_dds_sensor_data_ = sensor_data;
    has_dds_data_ = true;
   
    
    if (!is_initialized_)
      is_initialized_ = true;
  }
#elif USE_LEJU_DDS
  void humanoidController::LejuSensorsDataCallback(const leju::msgs::SensorsData& data)
  {
    SensorData sensor_data;
    sensor_msgs::Imu imu_msg;
    sensor_data.resize_joint(jointNumReal_+armNumReal_+headNum_);

    // JOINT DATA - extract from leju::msgs::SensorsData
    size_t joint_count = std::min(static_cast<size_t>(jointNumReal_+armNumReal_+headNum_),
                                  static_cast<size_t>(data.joint_data().joint_q().size()));

    for (size_t i = 0; i < joint_count; ++i)
    {
      sensor_data.jointPos_(i) = data.joint_data().joint_q()[i];
      sensor_data.jointVel_(i) = data.joint_data().joint_v()[i];
      sensor_data.jointAcc_(i) = data.joint_data().joint_vd()[i];
      sensor_data.jointTorque_(i) = data.joint_data().joint_torque()[i];
    }

    // Convert timestamp from leju DDS message
    sensor_data.timeStamp_ = ros::Time(data.header_sec(), data.header_nanosec());
    double sensor_time_diff = 0;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);

    // IMU DATA - extract from leju::msgs::SensorsData
    sensor_data.quat_.coeffs().w() = data.imu_data().quat()[0];
    sensor_data.quat_.coeffs().x() = data.imu_data().quat()[1];
    sensor_data.quat_.coeffs().y() = data.imu_data().quat()[2];
    sensor_data.quat_.coeffs().z() = data.imu_data().quat()[3];
    sensor_data.angularVel_ << data.imu_data().gyro()[0], data.imu_data().gyro()[1], data.imu_data().gyro()[2];
    sensor_data.linearAccel_ << data.imu_data().acc()[0], data.imu_data().acc()[1], data.imu_data().acc()[2];
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();

    // Apply filtering if needed
    {
      sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
      sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
    }

    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);

    // Handle head joint data if available
    if (headNum_ > 0 && sensor_data.jointPos_.size() == jointNumReal_+armNumReal_ + headNum_)
    {
      int head_start_index = sensor_data.jointPos_.size() - headNum_;
      for (size_t i = 0; i < headNum_; ++i)
      {
        sensor_data_head_.jointPos_(i) = data.joint_data().joint_q()[head_start_index + i];
        sensor_data_head_.jointVel_(i) = data.joint_data().joint_v()[head_start_index + i];
        sensor_data_head_.jointAcc_(i) = data.joint_data().joint_vd()[head_start_index + i];
        sensor_data_head_.jointTorque_(i) = data.joint_data().joint_torque()[head_start_index + i];
      }
    }

    // Save latest DDS sensor data for comparison
    latest_dds_sensor_data_ = sensor_data;
    has_dds_data_ = true;

    if (!is_initialized_)
      is_initialized_ = true;
  }
#endif

void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    auto &joint_data = msg->joint_data;
    auto &end_effector_data = msg->end_effector_data; // TODO: add end_effector_data to the observation
    SensorData sensor_data;

    sensor_msgs::Imu imu_msg;
    sensor_data.resize_joint(jointNumReal_+armNumReal_ + waistNum_);
    
    // JOINT DATA
   for(size_t i=0;i<jointNumReal_+waistNum_+armNumReal_;i++)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointTorque_(i) = joint_data.joint_torque[i];
    }
    // for (size_t i = 0; i < waistNum_; ++i)    //避开腰部自由度数据的输入
    // {
    //   sensor_data.jointPos_(jointNumReal_+i) = -joint_data.joint_q[i];
    //   sensor_data.jointVel_(jointNumReal_+i) = -joint_data.joint_v[i];
    //   sensor_data.jointAcc_(jointNumReal_+i) = -joint_data.joint_vd[i];
    //   sensor_data.jointTorque_(jointNumReal_+i) = -joint_data.joint_torque[i];
    // }
    // for (size_t i = 0; i < armNumReal_; ++i)    //避开腰部自由度数据的输入
    // {

    //   sensor_data.jointPos_(jointNumReal_+waistNum_+i) = joint_data.joint_q[jointNumReal_+waistNum_+i];
    //   sensor_data.jointVel_(jointNumReal_+waistNum_+i) = joint_data.joint_v[jointNumReal_+waistNum_+i];
    //   sensor_data.jointAcc_(jointNumReal_+waistNum_+i) = joint_data.joint_vd[jointNumReal_+waistNum_+i];
    //   sensor_data.jointTorque_(jointNumReal_+waistNum_+i) = joint_data.joint_torque[jointNumReal_+waistNum_+i];
    // }
    //test
    // for(size_t i=0;i<sensor_data.jointPos_.size();i++)
    // {
    //   std::cout << "sensor_data.jointPos_:" << sensor_data.jointPos_[i] << std::endl; 
    // }
    if (waistNum_ > 0)
    {
      for (size_t i = 0; i < waistNum_; ++i)
      {
        sensor_data_waist_.jointPos_(i) = joint_data.joint_q[i];
        sensor_data_waist_.jointVel_(i) = joint_data.joint_v[i];
        sensor_data_waist_.jointAcc_(i) = joint_data.joint_vd[i];
        sensor_data_waist_.jointTorque_(i) = joint_data.joint_torque[i];
      }
    }
    ros::Time ros_time = msg->header.stamp;
    sensor_data.timeStamp_ = msg->sensor_time;
    double sensor_time_diff = (ros::Time::now() - ros_time).toSec() * 1000;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);


    auto &imu_data = msg->imu_data;

    if(is_roban_)
    {
      double q_waist = sensor_data.jointPos_[jointNumReal_];
      double qd_waist = sensor_data.jointVel_[jointNumReal_];
      Eigen::Quaterniond imu_quat(imu_data.quat.w, imu_data.quat.x, imu_data.quat.y, imu_data.quat.z);
      Eigen::Quaterniond waist_base_quat(std::cos(-q_waist/2), 0, 0, std::sin(-q_waist/2));

      Eigen::Quaterniond waist_world_quat = imu_quat * waist_base_quat;
  
      // 加速度转换
      Eigen::Vector3d base_imu_acc(imu_data.acc.x, imu_data.acc.y, imu_data.acc.z);
      Eigen::Vector3d waist_base_acc = waist_base_quat.conjugate() * base_imu_acc;

      // 角速度转换
      Eigen::Vector3d waist_gyro(0, 0, qd_waist);
      Eigen::Vector3d imu_gyro(imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
      Eigen::Vector3d waist_base_gyro = waist_base_quat.conjugate() * (imu_gyro - waist_gyro); 

      // roban 版本需要转换 imu 数据
      sensor_data.quat_.coeffs().w() = waist_world_quat.coeffs().w();
      sensor_data.quat_.coeffs().x() = waist_world_quat.coeffs().x();
      sensor_data.quat_.coeffs().y() = waist_world_quat.coeffs().y();
      sensor_data.quat_.coeffs().z() = waist_world_quat.coeffs().z();
      sensor_data.angularVel_ << waist_base_gyro[0], waist_base_gyro[1], waist_base_gyro[2];
      sensor_data.linearAccel_ << waist_base_acc[0], waist_base_acc[1], waist_base_acc[2];
    }
    else
    {
      // 如果没有腰部，直接使用原始IMU数据
      sensor_data.quat_.coeffs().w() = imu_data.quat.w;
      sensor_data.quat_.coeffs().x() = imu_data.quat.x;
      sensor_data.quat_.coeffs().y() = imu_data.quat.y;
      sensor_data.quat_.coeffs().z() = imu_data.quat.z;
      sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
      sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    }
    sensor_data.freeLinearAccel_ << imu_data.free_acc.x, imu_data.free_acc.y, imu_data.free_acc.z;
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    if (!rl_available_)
    {
      sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
      sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
    }
    else
    {    
      sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
      Eigen::Vector3d acc_filtered = accFilterRL_.update(sensor_data.linearAccel_);
      Eigen::Vector3d free_acc_filtered = freeAccFilterRL_.update(sensor_data.freeLinearAccel_);
      Eigen::Vector3d gyro_filtered = gyroFilterRL_.update(sensor_data.angularVel_);
      

      for (int i = 0; i < 3; i++)
      {
        sensor_data.linearAccel_(i) = accFilterStateRL_(i) * acc_filtered(i) + (1 - accFilterStateRL_(i)) * sensor_data.linearAccel_(i);
        sensor_data.freeLinearAccel_(i) = freeAccFilterStateRL_(i) * free_acc_filtered(i) + (1 - freeAccFilterStateRL_(i)) * sensor_data.freeLinearAccel_(i);
        sensor_data.angularVel_(i) = gyroFilterStateRL_(i) * gyro_filtered(i) + (1 - gyroFilterStateRL_(i)) * sensor_data.angularVel_(i);
      }
    }
    
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
    // free_acc_filter_.update(sensor_data.linearAccel_);
    // END_EFFECTOR DATA
    // sensor_data_mutex_.lock();
    // sensorDataQueue.push(sensor_data);
    // sensor_data_mutex_.unlock();
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);

    if (headNum_ > 0 && joint_data.joint_q.size() == jointNumReal_ + armNumReal_ + headNum_ + waistNum_)
    {
      int head_start_index  = joint_data.joint_q.size() - headNum_;
      for (size_t i = 0; i < headNum_; ++i)
      {
        
        sensor_data_head_.jointPos_(i) = joint_data.joint_q[i + head_start_index];
        sensor_data_head_.jointVel_(i) = joint_data.joint_v[i + head_start_index];
        sensor_data_head_.jointAcc_(i) = joint_data.joint_vd[i + head_start_index];
        sensor_data_head_.jointTorque_(i) = joint_data.joint_torque[i + head_start_index];
      }
    }
    if (!is_initialized_)
      is_initialized_ = true;
  }
  void humanoidController::updatakinematics(const SensorData &sensor_data, bool is_initialized_)
  {
    SensorData sensor_data_new = sensor_data;
    ros::Time current_sensor_data_time = ros::Time::now();
    if (!is_initialized_)
    {
      last_sensor_data_time_ = current_sensor_data_time - ros::Duration(0.002);
    }
    double diff_time = (current_sensor_data_time - last_sensor_data_time_).toSec();
    ros::Duration period = ros::Duration(diff_time);
    nav_msgs::Odometry kinematics_odom;
    sensor_msgs::Imu imu_msg;
    Eigen::Quaterniond imu_quat(sensor_data_new.quat_.coeffs().w(), 
                                sensor_data_new.quat_.coeffs().x(), 
                                sensor_data_new.quat_.coeffs().y(), 
                                sensor_data_new.quat_.coeffs().z());
    imu_msg.header.stamp = current_sensor_data_time;
    imu_msg.header.frame_id = "dummy_link";
    imu_msg.header.seq = seq_;
    imu_msg.orientation.w = sensor_data_new.quat_.coeffs().w();
    imu_msg.orientation.x = sensor_data_new.quat_.coeffs().x();
    imu_msg.orientation.y = sensor_data_new.quat_.coeffs().y();
    imu_msg.orientation.z = sensor_data_new.quat_.coeffs().z();
    imu_msg.angular_velocity.x = sensor_data_new.angularVel_(0);
    imu_msg.angular_velocity.y = sensor_data_new.angularVel_(1);
    imu_msg.angular_velocity.z = sensor_data_new.angularVel_(2);
    imu_msg.linear_acceleration.x = sensor_data_new.linearAccel_(0);
    imu_msg.linear_acceleration.y = sensor_data_new.linearAccel_(1);
    imu_msg.linear_acceleration.z = sensor_data_new.linearAccel_(2);
    imu_msg.orientation_covariance = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05};
    imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    kinematics_odom = stateEstimate_->updateKinematics(current_sensor_data_time, imu_quat, period);
    kinematics_odom.header.seq  = seq_;
    seq_++;
    imuPub_.publish(imu_msg);
    kinematicPub_.publish(kinematics_odom);
    { // robotlocalization_data_mutex_
      std::lock_guard<std::mutex> lock(robotlocalization_data_mutex_);
      if(!robotlocalizationDataQueue.empty())
      {
      nav_msgs::Odometry robot_localization_ = robotlocalizationDataQueue.front();
      Eigen::Quaterniond robot_quat(robot_localization_.pose.pose.orientation.w, 
                                    robot_localization_.pose.pose.orientation.x, 
                                    robot_localization_.pose.pose.orientation.y, 
                                    robot_localization_.pose.pose.orientation.z);
      Eigen::Quaterniond sensor_quat(sensor_data_new.quat_.coeffs().w(), 
                                    sensor_data_new.quat_.coeffs().x(), 
                                    sensor_data_new.quat_.coeffs().y(), 
                                    sensor_data_new.quat_.coeffs().z());
      Eigen::Vector3d robot_eulerAngles = quatToZyx(robot_quat);
      Eigen::Vector3d sensor_eulerAngles = quatToZyx(sensor_quat);
      Eigen::Vector3d updata_eulerAngles;
      updata_eulerAngles << robot_eulerAngles(0), sensor_eulerAngles(1),sensor_eulerAngles(2);
              robot_quat_state_update_ = Eigen::AngleAxisd(updata_eulerAngles[0], Eigen::Vector3d::UnitZ())*
                                 Eigen::AngleAxisd(updata_eulerAngles[1], Eigen::Vector3d::UnitY())*
                                 Eigen::AngleAxisd(updata_eulerAngles[2], Eigen::Vector3d::UnitX());
      robotlocalizationDataQueue.pop();
      }
    }  // robotlocalization_data_mutex_
    sensor_data_new.quat_.w() = robot_quat_state_update_.w();
    sensor_data_new.quat_.x() = robot_quat_state_update_.x();
    sensor_data_new.quat_.y() = robot_quat_state_update_.y();
    sensor_data_new.quat_.z() = robot_quat_state_update_.z();
    last_sensor_data_time_ = current_sensor_data_time;
    ros_logger_->publishVector("/sensor_data_new/rpy/zyx", quatToZyx(sensor_data_new.quat_).transpose());
    ros_logger_->publishVector("/sensor_data_new/quat/wxyz", quatToZyx(sensor_data_new.quat_).transpose());
    stateEstimate_->updateImu(sensor_data_new.quat_, sensor_data_new.angularVel_, sensor_data_new.linearAccel_, sensor_data_new.orientationCovariance_, sensor_data_new.angularVelCovariance_, sensor_data_new.linearAccelCovariance_);
  }
  
  bool humanoidController::enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
      bool old_mode = use_ros_arm_joint_trajectory_;
      use_ros_arm_joint_trajectory_ = req.control_mode;
      
      // 记录模式切换
      if (old_mode != use_ros_arm_joint_trajectory_) 
      {
          ROS_INFO_STREAM("[ArmControl] ROS arm trajectory control mode changed: " << (use_ros_arm_joint_trajectory_ ? "ENABLED" : "DISABLED"));
      }
      
      res.result = true;
      return true;
  }

  bool humanoidController::enableMmArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
      bool old_mode = use_mm_arm_joint_trajectory_;
      use_mm_arm_joint_trajectory_ = req.control_mode;
      
      {
        mm_arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_+ waistNum_, armNumReal_);
        mm_arm_joint_trajectory_.vel = vector_t::Zero(armNumReal_);
      }

      // 记录模式切换
      if (old_mode != use_mm_arm_joint_trajectory_) 
      {
          ROS_INFO_STREAM("[ArmControl] MM arm trajectory control mode changed: " << (use_mm_arm_joint_trajectory_ ? "ENABLED" : "DISABLED"));
      }
      // arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_, armNumReal_);
      res.result = true;
      return true;
  }

  bool humanoidController::armJointSynchronizationCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    if (req.control_mode)
    {
      // arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_, armNumReal_);
      mm_arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_+ waistNum_, armNumReal_);
      res.result = true;
      res.message = "Successfully synchronize arm joint trajectory";
    }
    else
    {
      res.result = true;
      res.message = "disable arm joint synchronization";
    }
    return true;
  }

  bool humanoidController::getMmArmCtrlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    res.result = true;
    res.mode = static_cast<int>(use_mm_arm_joint_trajectory_);
    res.message = "Successfully get mm arm ctrl mode to " + std::to_string(static_cast<int>(use_mm_arm_joint_trajectory_));
    return true;
  }
  void humanoidController::starting(const ros::Time &time)
  {
    // Initial state
    // set the initial state = {0, 0, 0, 0, 0, 0, 0, 0, 0.976, 0, 0, 0, 0, 0, 0.35, -0.90, -0.55, 0, 0, 0, 0.35, -0.90, -0.55, 0}
    // currentObservation_.state = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().stateDim);
    // currentObservation_.state(8) = 0.78626;
    // currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;
    initial_status_ = HumanoidInterface_->getInitialState();
    initial_statusRL_ = initialStateRL_;
    pull_up_status_ = initial_status_;
    cur_status_ = initial_status_;
    currentObservation_.state = initial_status_;
    std::cout << "intial state:" << currentObservation_.state.transpose() << std::endl;
    std::cout << "waitign for the first sensor data" << std::endl;
    while (!is_initialized_)
    {
      if (!is_nodelet_node_)
        ros::spinOnce();
      usleep(1000);
    }
    std::cout << "sensor data received" << std::endl;
    if (is_real_)
    {
      std::cout << "wait for real robot controller starting\n";
      real_init_wait();
      std::cout << "real_init_wait done\n";
    }
    else
    {
      hardware_status_ = 1;
    }
    // applySensorsData(sensors_data_buffer_ptr_->getLastData());
    currentObservationWBC_.state.setZero(centroidalModelInfoWBC_.stateDim);
    currentObservationWBC_.input.setZero(centroidalModelInfoWBC_.inputDim);
    measuredRbdStateReal_.setZero(centroidalModelInfoWBC_.generalizedCoordinatesNum*2);
    currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);

    last_time_ = current_time_;
    updateStateEstimation(time, true);
    currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
    optimizedState2WBC_mrt_ = vector_t::Zero(centroidalModelInfoWBC_.stateDim);
    optimizedState2WBC_mrt_.head(centroidalModelInfo_.stateDim) = currentObservation_.state;
    std::cout << "initial state(after updateStateEstimation): " << currentObservation_.state.transpose() << std::endl;
    optimizedInput2WBC_mrt_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    optimizedInput2WBC_mrt_.head(centroidalModelInfo_.inputDim) = currentObservation_.input;

    currentObservation_.mode = ModeNumber::SS;
    

    intail_input_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    cur_input_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    for (int i = 0; i < 8; i++)
      intail_input_(3 * i + 2) = centroidalModelInfoWBC_.robotMass * 9.81 / 8; // 48.7*g/8
    optimizedInput2WBC_mrt_ = intail_input_;
    pull_up_input_ = intail_input_;
    if (is_simplified_model_)
    {
      optimizedState2WBC_mrt_.head(centroidalModelInfo_.stateDim) = currentObservation_.state;
      optimizedState2WBC_mrt_.tail(armNumReal_).setZero();

      for (int i = 0; i < 2; i++)
      {
        optimizedState2WBC_mrt_.segment(12 + jointNum_ + waistNum_ + i * armDofReal_, armDofMPC_) = optimizedState2WBC_mrt_.segment(12 + jointNum_ + waistNum_ + i * armDofMPC_, armDofMPC_);
      }
    }
    currentObservationWBC_ = currentObservation_;
    currentObservationWBC_.state = optimizedState2WBC_mrt_;
    initialState2WBC_mrt_ = optimizedState2WBC_mrt_;
    currentObservationWBC_.input = optimizedInput2WBC_mrt_;
    initialInput2WBC_mrt_ = optimizedInput2WBC_mrt_;
    stanceState_mrt_ = optimizedState2WBC_mrt_;
    stanceInput_mrt_ = optimizedInput2WBC_mrt_;
    // else
    // {
    //   mpcMrtInterface_->setCurrentObservation(currentObservation_);
    //   mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    //   while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    //   {
    //     mpcMrtInterface_->advanceMpc();
    //     ros::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    //   }
    // }
    ROS_INFO_STREAM("Initial policy has been received.");
    // usleep(1000); // wait for 1s to ensure that the initial policy is received by the MPC node
    if (!is_real_ && !is_play_back_mode_)
      callSimStartSrv(controllerNh_);
    // if (is_real_)
    // {
    //   std::cout << "real robot controller starting\n";
    //   real_init_wait();
    //   std::cout << "real_init_wait done\n";
    // }
    // current_time_ = ros::Time::now();
    last_time_ = current_time_;
    if (!is_play_back_mode_)
      sensors_data_buffer_ptr_->sync();

    std::cout << "starting the controller" << std::endl;
    mpcRunning_ = true;
    // 网络推理线程
    if (rl_available_)
    {
      inferenceThread_ = std::thread(&humanoidController::inference_thread_func, this);
      std::cout << "inferenceThread_ is start" << std::endl;
      if (!inferenceThread_.joinable())
      {
        ROS_ERROR_STREAM("Failed to start inference thread");
        exit(1);
      }
    }
  }
  
  void humanoidController::real_init_wait()
  {
    while (ros::ok())
    {
      if (ros::param::get("/hardware/is_ready", hardware_status_))
      {
        if (hardware_status_ == 1)
        {
          std::cerr << "real robot is ready\n";
          break;
        }
      }
      usleep(1000);
    }
    
  }

  bool humanoidController::preUpdate(const ros::Time &time)
  {
    // 半身模式下跳过起立过程，直接进入MPC初始化
    if (!only_half_up_body_)
    {
      std::cout << "only_half_up_body_ is false, start stand up process" << std::endl;
      /*******************输入蹲姿和站姿**********************/
      auto &infoWBC = centroidalModelInfoWBC_;
      vector_t squatState = vector_t::Zero(infoWBC.stateDim);
      squatState.head(12 + jointNum_) = drake_interface_->getSquatInitialState();
      vector_t standState = vector_t::Zero(infoWBC.stateDim);
      standState.head(12 + jointNum_) = drake_interface_->getInitialState();
      /*******采用 standUp_controller 从蹲姿运动到站姿*********/
      stateEstimate_->setFixFeetHeights(true);
      updateStateEstimation(time, false);
      // vector_t measuredRbdStateRL_;
      // measuredRbdStateRL_ = getRobotState();
      double startTime;
      double endTime;
      const double motionVel = 0.11;
      if (!isInitStandUpStartTime_)
      {
        isInitStandUpStartTime_ = true;
        robotStartStandTime_ = time.toSec();
        // 站立的结束时间是依据开始时间确定的
        startTime = robotStartStandTime_;
        endTime = startTime + (standState[8] - squatState[8]) / motionVel; // 以 0.11m/s 速度起立
        robotStandUpCompleteTime_ = endTime;
        ROS_INFO_STREAM("Set standUp start time: " << robotStartStandTime_);
      }

      vector_t curState = vector_t::Zero(infoWBC.stateDim);
      vector_t desiredState = vector_t::Zero(infoWBC.stateDim);
      if (is_abnor_StandUp_)
      {
        // 机器人站立异常，恢复到蹲起姿态
        curState = curRobotLegState_;
        desiredState = squatState;
        startTime = robotStartSquatTime_;
        endTime = startTime + (curRobotLegState_[8] - squatState[8]) / motionVel; // 以 0.11m/s 速度挂起
      }
      else
      {
        curState = squatState;
        curRobotLegState_ = standState;
        desiredState = standState;
      }
      scalar_array_t timeTrajectory;
      timeTrajectory.push_back(startTime);
      timeTrajectory.push_back(endTime);
      vector_array_t stateTrajectory;
      stateTrajectory.push_back(curState);
      stateTrajectory.push_back(desiredState);
      vector_t curTargetState_wbc = LinearInterpolation::interpolate(time.toSec(), timeTrajectory, stateTrajectory);
      vector_t torque = standUpWbc_->update(curTargetState_wbc, intail_input_, measuredRbdStateReal_, ModeNumber::SS, dt_, false).tail(infoWBC.actuatedDofNum);

      is_robot_standup_complete_ = fabs(standState[8] - curTargetState_wbc[8]) < 0.002;

      kuavo_msgs::jointCmd jointCmdMsg;
      for (int i1 = 0; i1 < jointNumReal_; ++i1)
      {
        jointCmdMsg.joint_q.push_back(curTargetState_wbc(12 + i1));
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(torque(i1));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.joint_kp.push_back(joint_kp_[i1]);
        jointCmdMsg.joint_kd.push_back(joint_kd_[i1]);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
        jointCmdMsg.control_modes.push_back(2);
      }
      for (int i1 = 0; i1 < waistNum_; ++i1)
      {
        jointCmdMsg.joint_q.push_back(curTargetState_wbc(12 + jointNumReal_ + i1));
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(torque(jointNumReal_+i1));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.joint_kp.push_back(joint_kp_[jointNumReal_+i1]);
        jointCmdMsg.joint_kd.push_back(joint_kd_[jointNumReal_+i1]);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[jointNumReal_+i1]);
        jointCmdMsg.control_modes.push_back(2);
      }
      for (int i2 = 0; i2 < armNumReal_; ++i2)
      {
        jointCmdMsg.joint_q.push_back(curTargetState_wbc(12 + jointNumReal_ + waistNum_ + i2));
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(torque(jointNumReal_+waistNum_+i2));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[jointNumReal_+waistNum_+i2]);
        jointCmdMsg.control_modes.push_back(joint_control_modes_[jointNumReal_+waistNum_+i2]);
        jointCmdMsg.joint_kp.push_back(0);
        jointCmdMsg.joint_kd.push_back(0);
      }
      for (int i3 = 0; i3 < headNum_; ++i3)
      {
        jointCmdMsg.joint_q.push_back(0);
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(0);
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(10);
        jointCmdMsg.control_modes.push_back(2);
        jointCmdMsg.joint_kp.push_back(10);
        jointCmdMsg.joint_kd.push_back(2);
      }
      // 发布控制命令
      publishControlCommands(jointCmdMsg);
      
      // if (use_shm_communication_) 
      //     publishJointCmdToShm(jointCmdMsg);

      if (!wheel_arm_robot_ && stand_up_protect_ && is_real_)
      {
        const double norSingleLegSupport = centroidalModelInfo_.robotMass * 9.8 / 4; // 单脚支撑力只要达到重量的1/4的力即认为已落地成功
        bool bNotLanding = is_robot_standup_complete_ && (contactForce_[2] < norSingleLegSupport || contactForce_[8] < norSingleLegSupport);
        bool bUneventForce = fabs(contactForce_[2] - contactForce_[8]) > (norSingleLegSupport * 2.0); // 左右脚支撑立差值超过重量的1/2即判断为异常/*  */
        if (bNotLanding || bUneventForce)
        {
          if (!is_abnor_StandUp_ && (bUneventForce || (time.toSec() > robotStandUpCompleteTime_ + 0.5)))
          {
            ROS_WARN("Robot standing abnormal...!!");
            if(bNotLanding)
            {
              ROS_WARN("Single-foot contact force that does not reach one-quarter of body weight");
              ROS_INFO_STREAM("left feet force: " << contactForce_[2] << "less than " << norSingleLegSupport);
              ROS_INFO_STREAM("right feet force: " << contactForce_[8] << "less than " << norSingleLegSupport);
            }
            if(bUneventForce)
            {
              ROS_WARN("Abnormal contact force difference between left and right foot");
              ROS_INFO_STREAM("left feet force: " << contactForce_[2]);
              ROS_INFO_STREAM("right feet force: " << contactForce_[8]);
            }
            is_abnor_StandUp_ = true;
            is_robot_standup_complete_ = false;
            curRobotLegState_ = currentObservationWBC_.state;
            robotStartSquatTime_ = time.toSec();
            ROS_INFO_STREAM("Set squat start time: " << robotStartSquatTime_);
          }
        }

        // 等待机器人脚收回
        if (is_abnor_StandUp_)
        {
          bool isReSquatComplete = fabs(squatState[8] - curTargetState_wbc[8]) < 0.002;
          if (isReSquatComplete)
          {
            // 判断机器人的脚是否收回
            ROS_WARN("The robot goes into a squat state, waiting for adjustment...");

            // 将硬件准备状态位设置为0
            ROS_INFO_STREAM("Set hardware/is_ready is 0.");
            ros::param::set("/hardware/is_ready", 0);
            hardware_status_ = 0;
            isInitStandUpStartTime_ = false;
            is_abnor_StandUp_ = false;

            std_msgs::Int8 bot_stand_up_failed;
            bot_stand_up_failed.data = -1;
            standUpCompletePub_.publish(bot_stand_up_failed);
            return false;
          }
          return true;
        }
      }
    } // 结束 only_half_up_body_ 判断的else块

    /*******************超过设置时间，退出******************/
    // 延迟启动, 避免切换不稳定
    // 半身模式下，设置robotStandUpCompleteTime_为过去时间，立即触发MPC初始化
    if (only_half_up_body_ && !isInitStandUpStartTime_)
    {
      isInitStandUpStartTime_ = true;
      robotStandUpCompleteTime_ = time.toSec() - 1.0;
    }
    if (time.toSec() > robotStandUpCompleteTime_ + 0.8 || !is_real_)
    {
      SystemObservation initial_observation = currentObservation_;
      initial_observation.state = initial_status_;
      TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});
      mpc_current_target_trajectories_ = target_trajectories;
      // Set the first observation and command and wait for optimization to finish
      ROS_INFO_STREAM("Waiting for the initial policy ...");
      {
        // Reset MPC node
        mrtRosInterface_->resetMpcNode(target_trajectories);
        std::cout << "reset MPC node\n";
        // Wait for the initial policy
        while (!mrtRosInterface_->initialPolicyReceived() && ros::ok() && ros::master::check())
        {
          mrtRosInterface_->spinMRT();
          mrtRosInterface_->setCurrentObservation(initial_observation);
          ros::Rate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
        mrtRosInterface_->updatePolicy();
        vector_t optimizedState_mrt, optimizedInput_mrt;
        mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      }
      
      stateEstimate_->setFixFeetHeights(false);
      isPreUpdateComplete = true;
      standupTime_ = currentObservation_.time;

      std_msgs::Int8 bot_stand_up_complete;
      bot_stand_up_complete.data = 1;
      standUpCompletePub_.publish(bot_stand_up_complete);
    }
    return true;
  }
  void humanoidController::checkMpcPullUp(double current_time, vector_t & current_state, const TargetTrajectories& planner_target_trajectories)
  {
    if (!is_stance_mode_ || only_half_up_body_)
      return;

    // 检查高度轨迹是否为水平直线的lambda函数
    auto isHeightTrajectoryHorizontal = [](const vector_array_t& stateTrajectory) -> bool {
      if (stateTrajectory.empty()) return true;
      
      // 获取第一个点的高度作为参考值
      const double reference_height = stateTrajectory.front()[8];
      
      // 检查所有点的高度是否与参考高度相同
      return std::all_of(stateTrajectory.begin(), stateTrajectory.end(),
                        [reference_height](const vector_t& state) {
                          return std::abs(state[8] - reference_height) < 1e-3;
                        });
    };

    auto planner_state = planner_target_trajectories.getDesiredState(current_time);
    bool is_fixed_height = isHeightTrajectoryHorizontal(planner_target_trajectories.stateTrajectory);

    if (is_fixed_height && current_state[8] - planner_state[8] > 0.02)// 期望高度差很大
    {
      ROS_WARN("Mpc pull up detected, current height: %f, planner height: %f", current_state[8], planner_state[8]);
      isPullUp_ = true;
    }
  }
  void humanoidController::update(const ros::Time &time, const ros::Duration &dfd)
  {
    is_rl_controller_buffer_.updateFromBuffer();// 使用buffer中的值更新is_rl_controller_,避免多线程更新
    is_rl_controller_ = is_rl_controller_buffer_.get();
    
    // 如果RL不可用，强制设置 is_rl_controller_ 为 false
    if (!rl_available_) {
      is_rl_controller_ = false;
    }
    
    ros_logger_->publishValue("/humanoid_controller/is_rl_controller_", is_rl_controller_);
    ros_logger_->publishValue("/humanoid_controller/resetting_mpc_state_", resetting_mpc_state_);
    if (!last_is_rl_controller_ && is_rl_controller_)
    {
      // 进入 RL 前，先用 MPC 将躯干高度插值到 RL 默认高度
      inference_running_ = true;
      std::cout << "pause MPC" << std::endl;
      Eigen::VectorXd current_arm_pos = Eigen::VectorXd::Zero(armNumReal_);
      Eigen::VectorXd current_arm_vel = Eigen::VectorXd::Zero(armNumReal_);
      current_arm_pos = jointPosWBC_.segment(jointNumReal_+ waistNum_, armNumReal_);
      current_arm_vel = jointVelWBC_.segment(jointNumReal_+ waistNum_, armNumReal_);
      Eigen::VectorXd target_arm_pos = defalutJointPosRL_.segment(jointNumReal_+ waistNum_, armNumReal_);
      Eigen::VectorXd target_arm_vel = Eigen::VectorXd::Zero(armNumReal_);

      // 初始化RL模式的手臂目标位置和速度（避免保留旧的外部控制值）
      desire_arm_q_ = defalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
      desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);
      
      startArmInterpolation(time, current_arm_pos, current_arm_vel, target_arm_pos, target_arm_vel);
      ROS_INFO("[MPC->RL] 触发手臂插值到RL默认位置");

      // 清理 kuavo_arm_traj 话题缓存（MPC->RL时也清理）
      // 避免RL模式下使用MPC遗留的外部控制数据
      arm_joint_trajectory_.pos = current_arm_pos;
      arm_joint_trajectory_.vel = current_arm_vel;
      arm_joint_trajectory_.tau = Eigen::VectorXd::Zero(armNumReal_);
      ROS_INFO("[MPC->RL] 清理手臂轨迹缓存");

      // 启动躯干插值，XY 对齐双脚中心，Z 对齐 RL 默认高度
      // 使用已有的躯干插值系统，并覆盖目标高度为 initialStateRL_(8)
      vector3_t feet_center = currentObservation_.state.segment<3>(6);
      feet_center(2) = defaultBaseHeightControl_;
      vector6_t targetPose = vector6_t::Zero();
      targetPose.segment<3>(0) = feet_center;                  // xyz
      targetPose(3) = 0.0;                                     // roll
      targetPose(4) = 0.0;                      // pitch 维持与原逻辑一致
      targetPose(5) = currentObservation_.state(9);                     // yaw 维持与原逻辑一致
      // 获取当前手臂位置作为目标（保持当前位置）
      stanceState_mrt_ = currentObservation_.state;
      stanceInput_mrt_ = initialInput2WBC_mrt_;
      startMPCRLInterpolation(currentObservation_.time, targetPose, target_arm_pos);
      resetting_mpc_state_ = ResettingMpcState::RESET_BASE;

      // 同步启动RL原地踏步2秒（若启用）
      if (enable_in_place_stepping_)
      {
        temp_in_place_duration_backup_ = in_place_step_duration_;
        in_place_step_duration_ = 2.0;
        temp_in_place_end_time_ = time + ros::Duration(2.0);
        temp_in_place_duration_override_active_ = true;
        startInPlaceStepping(time);
        Walkenable_ = true;
        ROS_INFO("[MPC->RL] 启动原地踏步2秒");
      }

    }
    else if (last_is_rl_controller_ && !is_rl_controller_)
    {
      reset_mpc_ = true;
      inference_running_ = false;
      stanceState_mrt_ = currentObservation_.state;
      stanceInput_mrt_ = initialInput2WBC_mrt_;
      
      // 清理 kuavo_arm_traj 话题缓存，避免使用旧数据
      // 重置为当前实际手臂位置，确保切换平滑
      vector_t current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
      vector_t current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
      arm_joint_trajectory_.pos = current_arm_pos;
      arm_joint_trajectory_.vel = current_arm_vel;
      arm_joint_trajectory_.tau = Eigen::VectorXd::Zero(armNumReal_);
      
      ROS_INFO("[RL->MPC] 清理手臂轨迹缓存，重置为当前位置: [%.3f, %.3f, ...]", 
               current_arm_pos(0), current_arm_pos(1));
    }
    last_is_rl_controller_ = is_rl_controller_;
    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    // 使用共享内存更新传感器数据
    if (use_shm_communication_) {
      updateSensorDataFromShm();
    }
    updateStateEstimation(time, false);
    mrtRosInterface_->setCurrentObservation(currentObservation_);

    // 非RL或躯干插值中，均走MPC流程
    bool mpc_flow = (!is_rl_controller_) || is_torso_interpolation_active_;
    if (mpc_flow){
      is_mpc_controller_ = true;
      if (reset_mpc_) // 重置mpc
      {
        mrtRosInterface_->pauseResumeMpcNode(false);
        std::cout << "resume MPC" << std::endl;
        // Trigger MRT callbacks
        mrtRosInterface_->spinMRT();
        currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
        auto target_trajectories = TargetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
        mrtRosInterface_->resetMpcNode(target_trajectories);
        // 修复：滤波器重置为当前实际手臂位置，避免跳变
        vector_t current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        vector_t current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        arm_joint_pos_filter_.reset(current_arm_pos);
        arm_joint_vel_filter_.reset(current_arm_vel);

        reset_mpc_ = false;
        resetting_mpc_state_ = ResettingMpcState::RESET_INITIAL_POLICY;
        
        std::cout << "reset MPC node at " << currentObservation_.time << "\n";
      }
      // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getNextData();
      // // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
      // applySensorsData(msg);
      // State Estimate
      ros::Duration period = ros::Duration(dt_);

      auto& info = centroidalModelInfo_;
      auto& infoWBC = centroidalModelInfoWBC_;

      vector_t optimizedState_mrt, optimizedInput_mrt;
      bool is_mpc_updated = false;
      {// update mpc policy
        // Only use halfup_body doesn't work well.
        if (!only_half_up_body_) {
          // Update the current state of the system
          mrtRosInterface_->setCurrentObservation(currentObservation_);
          
          // Trigger MRT callbacks
          mrtRosInterface_->spinMRT();
          // Update the policy if a new on was received
          if (resetting_mpc_state_ == ResettingMpcState::RESET_INITIAL_POLICY)// 重置MPC状态1：等待初始策略, 状态2：更新躯干位置
          {
            if (mrtRosInterface_->initialPolicyReceived() && mrtRosInterface_->updatePolicy() && mrtRosInterface_->isPolicyUpdated())// 收到初始策略，更新成功
            {
              std::cout << "resetting_mpc_ and initialPolicyReceived, switching to RESET_BASE" << std::endl;
              resetting_mpc_state_ = ResettingMpcState::RESET_BASE;
              
              // 获取双脚中心位置
              vector3_t targetTorsoPos = stateEstimate_->getFeetCenterPosition();
              targetTorsoPos(2) = default_state_[8];
              vector6_t targetTorsoPose = vector6_t::Zero();
              targetTorsoPose.segment<3>(0) = targetTorsoPos;
              targetTorsoPose(3) = 0.0;
              targetTorsoPose(4) = default_state_(10);
              targetTorsoPose(5) = stanceState_mrt_(9);
              
              // 修复：切换到RL使用当前实际手臂位置作为插值目标，而不是默认位置，避免跳变
              vector_t target_arm_pos = defalutArmPosMPC_;
              if (is_rl_controller_)
              {
                target_arm_pos = defalutJointPosRL_.segment(jointNumReal_+ waistNum_, armNumReal_);
              }

              startMPCRLInterpolation(currentObservation_.time, targetTorsoPose, target_arm_pos);
            }
            std::cout << "waiting for initialPolicy,using default state target..." << std::endl;
            optimizedState_mrt = stanceState_mrt_;
            optimizedInput_mrt = stanceInput_mrt_;
            plannedMode_ = ModeNumber::SS;
          }
          else
          {
            optimizedState_mrt = stanceState_mrt_;
            optimizedInput_mrt = stanceInput_mrt_;
            plannedMode_ = ModeNumber::SS;
            if (resetting_mpc_state_ == ResettingMpcState::RESET_BASE)
            {// 插值阶段
              // 更新躯干插值
              updateMPCRLInterpolation(currentObservation_.time);

              // 检查插值是否完成
              if (!is_torso_interpolation_active_)
              {
                std::cout << "Torso interpolation completed, switching to NORMAL" << std::endl;
                resetting_mpc_state_ = ResettingMpcState::NOMAL;
              }
            }
            if (mrtRosInterface_->updatePolicy())
            {
              is_mpc_updated = true;
              auto &policy = mrtRosInterface_->getPolicy();
              auto &state_trajectory = policy.stateTrajectory_;
              auto &command = mrtRosInterface_->getCommand();
              mpc_current_target_trajectories_ = command.mpcTargetTrajectories_;
              // checkMpcPullUp(currentObservation_.time, currentObservation_.state, command.mpcTargetTrajectories_);
              // trajectory_publisher_->publishTrajectory(state_trajectory);
              TargetTrajectories target_trajectories(policy.timeTrajectory_, policy.stateTrajectory_, policy.inputTrajectory_);

              publishFeetTrajectory(target_trajectories);
            }
            if (mrtRosInterface_->isPolicyUpdated())
            {
              mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
            }

            if (is_torso_interpolation_active_)
            {
              optimizedState_mrt.segment<6>(6) = torso_interpolation_result_;
              // optimizedState_mrt.segment<12>(12) = currentObservation_.state.segment<12>(12);
              // optimizedInput_mrt = stanceInput_mrt_;
              plannedMode_ = ModeNumber::SS;
            }
            // else
            // {
            //   mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
            // }
          }
        }
      }
      // std::cout << "optimizedState_mrt:" << optimizedState_mrt.transpose() << " \noptimizedInput_mrt:" << optimizedInput_mrt.transpose() << " plannedMode_:" << plannedMode_ << std::endl;
      ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_origin", optimizedState_mrt);
      ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt_origin", optimizedInput_mrt);

      bool enable_mpc{true};
      {
        std::lock_guard<std::mutex> lk(disable_mpc_srv_mtx_);
        enable_mpc = !disable_mpc_;
      }

      wbc_->setPullUpState(isPullUp_);
      if (setPullUpState_)
      {
        pull_up_status_ = optimizedState_mrt;
        setPullUpState_ = false;
      }
      if (wbc_only_ || only_half_up_body_)
      {
        optimizedState_mrt = initial_status_;
        optimizedInput_mrt = intail_input_;
      }
      else if (isPullUp_)
      {
        optimizedState_mrt = pull_up_status_;
        optimizedInput_mrt = intail_input_;
        plannedMode_ = ModeNumber::SS;
      }
      else if (!enable_mpc)
      {
        optimizedState_mrt = cur_status_;
        optimizedInput_mrt = cur_input_;
        plannedMode_ = ModeNumber::SS;
      }
      else
      {
        cur_status_ = optimizedState_mrt;
        cur_input_ = optimizedInput_mrt;
      }

      if (is_simplified_model_)
      {
        // 躯干和腿部target
        optimizedState2WBC_mrt_.head(info.stateDim) = optimizedState_mrt;
        optimizedInput2WBC_mrt_.head(info.inputDim) = optimizedInput_mrt;
        optimizedState2WBC_mrt_.tail(armNumReal_).setZero();
        optimizedInput2WBC_mrt_.tail(armNumReal_).setZero();

        // 手臂target前半部分
        for (int i = 0; i < 2; i++)
        {
          optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
              optimizedState_mrt.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
          optimizedInput2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
              optimizedInput_mrt.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
        }

        // 手臂target后半部分，从arm_joint_trajectory_获取

        auto target_arm_pos = currentArmTargetTrajectories_.getDesiredState(currentObservation_.time);
        if (target_arm_pos.size() == armNumReal_)
        {
          // 只使用上半身模式, 此时 MPC 求解未开启, 需要进行插值处理
          if (only_half_up_body_)
          {
            // 检测目标改变并启动插值
            if ((half_body_arm_interpolation_last_target_pos_.size() != target_arm_pos.size()) ||
                ((target_arm_pos - half_body_arm_interpolation_last_target_pos_).norm() > 0.01))
            {
              Eigen::VectorXd current_pos = is_half_body_arm_interpolating_ ? half_body_arm_interpolation_start_pos_ : jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
              is_half_body_arm_interpolating_ = ((target_arm_pos - current_pos).norm() >= 0.05);
              if (is_half_body_arm_interpolating_)
              {
                half_body_arm_interpolation_start_pos_ = current_pos;
                half_body_arm_interpolation_target_pos_ = target_arm_pos;
                half_body_interpolation_start_time_ = currentObservation_.time;
              }
              half_body_arm_interpolation_last_target_pos_ = target_arm_pos;
            }
            
            // 使用LinearInterpolation计算插值位置
            Eigen::VectorXd final_arm_pos = target_arm_pos;
            if (is_half_body_arm_interpolating_ && half_body_interpolation_duration_ > 0.0)
            {
              double end_time = half_body_interpolation_start_time_ + half_body_interpolation_duration_;
              if (currentObservation_.time < end_time)
              {
                scalar_array_t timeTrajectory = {half_body_interpolation_start_time_, end_time};
                vector_array_t stateTrajectory = {half_body_arm_interpolation_start_pos_, half_body_arm_interpolation_target_pos_};
                final_arm_pos = LinearInterpolation::interpolate(currentObservation_.time, timeTrajectory, stateTrajectory);
              }
              else
              {
                is_half_body_arm_interpolating_ = false;
              }
            }
            
            // 应用到WBC
            for (int i = 0; i < 2; i++)
            {
              optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofReal_) =
                  final_arm_pos.segment(i * armDofReal_, armDofReal_);
            }
          }
          else
          {
            for (int i = 0; i < 2; i++)
            {
              optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_ + armDofMPC_, armDofDiff_) =
                  target_arm_pos.segment(i * armDofReal_ + armDofMPC_, armDofDiff_);
            }
          }
        }
      }
      else
      {
        optimizedState2WBC_mrt_ = optimizedState_mrt;
        optimizedInput2WBC_mrt_ = optimizedInput_mrt;
        
      }
      currentObservation_.input = optimizedInput_mrt;// 传什么值都一样, MPC不使用obs.input
      
       
      // *************************** arm joint trajectory **********************************

      if(use_mm_arm_joint_trajectory_)
      {
        // TODO: feedback in planner
        // auto arm_pos = currentObservation_.state.tail(armNum_); 
        // optimizedInput2WBC_mrt_.tail(armNum_) = 0.05 * (arm_joint_trajectory_.pos - arm_pos)/dt_;
        // optimizedState2WBC_mrt_.tail(armNum_) = arm_pos + optimizedInput2WBC_mrt_.tail(armNum_) * dt_;
        if (only_half_up_body_) 
        {
            optimizedState2WBC_mrt_.segment<7>(24) = mm_arm_joint_trajectory_.pos.segment<7>(0);
            optimizedState2WBC_mrt_.segment<7>(24+7) = mm_arm_joint_trajectory_.pos.segment<7>(7);
        }
        else if (mpcArmControlMode_desired_ == ArmControlMode::EXTERN_CONTROL && mpcArmControlMode_ == ArmControlMode::EXTERN_CONTROL){// 只有外部控制模式才直接使用关节target
            // 位置、速度
            optimizedState2WBC_mrt_.tail(armNumReal_) = mm_arm_joint_trajectory_.pos;
        }
      }
      static bool low_latency_first_enter = true;
      if (mpcArmControlMode_desired_ != ArmControlMode::EXTERN_CONTROL)
      {
        low_latency_first_enter = true;
      }
      if (use_ros_arm_joint_trajectory_)
      {
        if (mpcArmControlMode_desired_ == ArmControlMode::EXTERN_CONTROL && mpcArmControlMode_ == ArmControlMode::EXTERN_CONTROL)
        {
          vector_t filtered_pos = arm_joint_pos_filter_.update(arm_joint_trajectory_.pos);
          optimizedState2WBC_mrt_.tail(armNumReal_) = filtered_pos;
    
        // 2. 如果外部轨迹没有提供速度，使用滤波后的位置计算速度
          static vector_t prev_filtered_pos = filtered_pos;
          if (low_latency_first_enter)
          {
            prev_filtered_pos = filtered_pos;
            low_latency_first_enter = false;
          }
          vector_t computed_vel = (filtered_pos - prev_filtered_pos) / dt_;
            
            // 3. 对计算出的速度再次滤波
            optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(computed_vel);

            //ros_logger_->publishVector("/humanoid_controller/arm_joint_computed_vel", computed_vel);
            prev_filtered_pos = filtered_pos;
    
        }
        else if(only_half_up_body_ && mpcArmControlMode_desired_ == ArmControlMode::EXTERN_CONTROL)
        {
          optimizedState2WBC_mrt_.segment<7>(24) = arm_joint_trajectory_.pos.segment<7>(0);
          optimizedState2WBC_mrt_.segment<7>(24+7) = arm_joint_trajectory_.pos.segment<7>(7);
          optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_pos_filter_.update(optimizedState2WBC_mrt_.tail(armNumReal_));
          optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(optimizedInput2WBC_mrt_.tail(armNumReal_));
        }
        else
        {
          // use filter output
          optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_pos_filter_.update(optimizedState2WBC_mrt_.tail(armNumReal_));
          optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(optimizedInput2WBC_mrt_.tail(armNumReal_));
        }

      }
      else
      {
        if (resetting_mpc_state_ == ResettingMpcState::RESET_BASE && is_torso_interpolation_active_)
        { 
          optimizedState2WBC_mrt_.tail(armNumReal_) = arm_interpolation_result_;
        }
        // // use filter output
        optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_pos_filter_.update(optimizedState2WBC_mrt_.tail(armNumReal_));
        optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(optimizedInput2WBC_mrt_.tail(armNumReal_));
        low_latency_first_enter = true;
      }

    // *************************** arm joint trajectory **********************************

      // optimizedInput2WBC_mrt_.segment(optimizedInput_mrt.size() - info.actuatedDofNum, jointNum_) = mrt_joint_vel_filter_.update(optimizedInput_mrt.segment(optimizedInput_mrt.size() - info.actuatedDofNum, jointNum_));
      // ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt_filtered", optimizedInput2WBC_mrt_);
      
      // // use ik output 
      // vector_t filtered_arm_pose = arm_joint_pos_filter_.update(arm_joint_trajectory_.pos);
      // optimizedState2WBC_mrt_.tail(armNum_) = filtered_arm_pose;
      // vector_t filter_input_vel = (filtered_arm_pose- arm_joint_pos_cmd_prev_)/dt_;
      // optimizedInput2WBC_mrt_.tail(armNum_) = arm_joint_vel_filter_.update(filter_input_vel);
      // arm_joint_pos_cmd_prev_ = filtered_arm_pose;
    
    
      // for(int i=0;i<info.actuatedDofNum;i++)
      // {
      //   optimizedState2WBC_mrt_(12+i) = std::max(joint_state_limit_(i, 0), std::min(optimizedState2WBC_mrt_[12+i], joint_state_limit_(i, 1)));
      // }
      
      optimized_mode_ = plannedMode_;
      // currentObservation_.input.tail(info.actuatedDofNum) = measuredRbdState_.tail(info.actuatedDofNum);

      // Whole body control
      // wbc_->setStanceMode(currentObservation_.mode == ModeNumber::SS);

      auto contactFlag_ = modeNumber2StanceLeg(currentObservation_.mode);
      bool lf_contact = std::any_of(contactFlag_.begin(), contactFlag_.begin() + 4, [](int flag)
                                    { return flag; });
      bool rf_contact = std::any_of(contactFlag_.begin() + 4, contactFlag_.end(), [](int flag)
                                    { return flag; });
      if (lf_contact && rf_contact)
      {
        wbc_->setStanceMode(true);
      }
      else
      {
        wbc_->setStanceMode(false);
      }
      wbcTimer_.startTimer();
      for(int i=0;i<infoWBC.numThreeDofContacts;i++)
      {
        ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt/force_" + std::to_string(i+1), optimizedInput2WBC_mrt_.segment(3 * i, 3));
      }
      if (info.numSixDofContacts > 0)
      {
        Eigen::Matrix3d R_ws = ocs2::getRotationMatrixFromZyxEulerAngles(Eigen::Vector3d(optimizedState2WBC_mrt_(9), 0, 0));
        Eigen::VectorXd hand_wrench_cmd_tmp = hand_wrench_cmd_;
        hand_wrench_cmd_tmp.segment<3>(0) = R_ws * hand_wrench_cmd_.segment<3>(0);
        hand_wrench_cmd_tmp.segment<3>(6) = R_ws * hand_wrench_cmd_.segment<3>(6);
        optimizedInput2WBC_mrt_.segment(3 * info.numThreeDofContacts, hand_wrench_cmd_.size()) = hand_wrench_cmd_tmp;
        for(int i=0;i<info.numSixDofContacts;i++)
        {
          Eigen::VectorXd wrench = optimizedInput2WBC_mrt_.segment(3 * info.numThreeDofContacts + 6 * i, 6);
          ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt/wrench_" + std::to_string(i+1), wrench);
          visualizeWrench(wrench, i==0);
        }
      }
      
      ros_logger_->publishValue("/humanoid_controller/optimized_mode", static_cast<double>(optimized_mode_));

      ros_logger_->publishVector("/humanoid_controller/optimizedState_wbc_mrt_origin", optimizedState2WBC_mrt_);
      ros_logger_->publishVector("/humanoid_controller/optimizedInput_wbc_mrt_origin", optimizedInput2WBC_mrt_);
      // *************************** WBC **********************************

      bool enable_wbc{true};
      {
        std::lock_guard<std::mutex> lk(disable_wbc_srv_mtx_);
        enable_wbc = !disable_wbc_;
      }

      publishWbcArmEndEffectorPose();

      // std::chrono::time_point<std::chrono::high_resolution_clock> t4;
      if (enable_wbc)
      {
        vector_t x = wbc_->update(optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, measuredRbdStateReal_, plannedMode_, period.toSec(), is_mpc_updated);

        // wbc_->updateVd(jointAcc_);
        wbcTimer_.endTimer();

        // 决策变量, 6*body_acc + 12*joint_acc + 3x4*contact_force + 12*torque = 42
        vector_t torque = x.tail(infoWBC.actuatedDofNum);
        const vector_t &wbc_planned_joint_acc = x.segment(6, infoWBC.actuatedDofNum);
        const vector_t &wbc_planned_body_acc = x.head(6);
        // std::cout << "wbc_planned_joint_acc:" << wbc_planned_joint_acc.transpose() << std::endl;
        // std::cout << "wbc_planned_body_acc:" << wbc_planned_body_acc.transpose() << std::endl;
        const vector_t &wbc_planned_contact_force = x.segment(6 + infoWBC.actuatedDofNum, wbc_->getContactForceSize());
        // std::cout << "wbc_planned_contact_force:" << wbc_planned_contact_force.transpose() << std::endl;
        // std::cout << "torque:" << torque.transpose() << std::endl;
        ros_logger_->publishVector("/humanoid_controller/torque", torque);
        ros_logger_->publishVector("/humanoid_controller/wbc_planned_joint_acc", wbc_planned_joint_acc);
        ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/linear", wbc_planned_body_acc.head<3>());
        ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/angular", wbc_planned_body_acc.tail<3>());
        ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/left_foot", wbc_planned_contact_force.head<12>());
        ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/right_foot", wbc_planned_contact_force.tail<12>());
        // std::cout << "wbc_planned_contact_force:" << wbc_planned_contact_force.transpose() << std::endl;

        vector_t posDes = centroidal_model::getJointAngles(optimizedState2WBC_mrt_, infoWBC);
        vector_t velDes = centroidal_model::getJointVelocities(optimizedInput2WBC_mrt_, infoWBC);

        scalar_t dt = period.toSec();
        bool is_joint_acc_out_of_range = wbc_planned_joint_acc.array().abs().maxCoeff() > 2000;
        if (is_joint_acc_out_of_range)
        {
          ROS_INFO_STREAM("wbc_planned_joint_acc is out of range!");
          std::cerr << "wbc_planned_joint_acc: " << wbc_planned_joint_acc.transpose() << std::endl;
          torque = output_tau_;
        }
        else
        {
          posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
          velDes = velDes + wbc_planned_joint_acc * dt;
        }
        // ros_logger_->publishVector("/humanoid_controller/posDes", posDes);
        // ros_logger_->publishVector("/humanoid_controller/velDes", velDes);
        // ***************************** WBC END **********************************

        // Safety check, if failed, stop the controller
        if (!safetyChecker_->check(currentObservation_, optimizedState_mrt, optimizedInput_mrt))
        {
          ROS_ERROR_STREAM("[humanoid Controller] Safety check failed, stopping the controller.");
          std_msgs::Bool stop_msg;
          stop_msg.data = true;
          stop_pub_.publish(stop_msg);
          usleep(100000);

          return;
        }

        {
          output_pos_ = posDes;
          output_vel_ = velDes;
          output_tau_ = torque;
        }
      }

      vector_t kp_ = joint_kp_, kd_ = joint_kd_;
      if (currentObservation_.mode != ModeNumber::SS)
      {
        kp_ = joint_kp_walking_;
        kd_ = joint_kd_walking_;
      }



      
      for (int i1 = 0; i1 < jointNumReal_+ waistNum_; ++i1)
      {
        jointCmdMsg.joint_q.push_back(output_pos_(i1));
        jointCmdMsg.joint_v.push_back(output_vel_(i1));
        jointCmdMsg.tau.push_back(output_tau_(i1));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.joint_kp.push_back(joint_kp_[i1]);
        jointCmdMsg.joint_kd.push_back(joint_kd_[i1]);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
        jointCmdMsg.control_modes.push_back(joint_control_modes_[i1]);

        // jointCurrentWBC_(i1) = output_tau_(i1);
      }
      ModeSchedule current_mode_schedule;
      if (resetting_mpc_state_ != ResettingMpcState::RESET_INITIAL_POLICY)
      {
        current_mode_schedule = mrtRosInterface_->getCurrentModeSchedule();
      }
      else
      {
        current_mode_schedule = ModeSchedule({0}, {ModeNumber::SS, ModeNumber::SS});
      }
      auto is_SS_mode_after = [&](const ModeSchedule &mode_schedule) { // 后续都是SS mode
        int start_index = mode_schedule.modeBeforeId(currentObservation_.time);
        for (int i1 = start_index + 1; i1 < mode_schedule.modeSequence.size(); ++i1)
        {
          if (mode_schedule.modeSequence[i1] != ModeNumber::SS)
          {
            return false;
          }
        }
        return true;
      };
      auto is_walking_gait = [&](const std::string &gait_name)
      {
        return gait_name == "walk" || gait_name == "trot";
      };

      is_stance_mode_ = is_SS_mode_after(current_mode_schedule);

      // 膝关节全程力控

      const auto &current_time = currentObservation_.time - dt_;
      size_t current_mode = currentObservation_.mode;
      size_t before_mode = current_mode_schedule.modeBefore(current_time);
      nextMode_ = current_mode_schedule.modeNext(current_time);
      double switch_time = current_mode_schedule.timeSwitch(current_time);
      double start_time = current_mode_schedule.timeBefore(current_time);
      size_t be_before_mode = current_mode_schedule.modeBefore(start_time - dt_); // 前前一个mode

      bool to_double_contact = current_mode == ModeNumber::SS && before_mode != ModeNumber::SS;
      bool lf_heel_off_contact = current_mode == ModeNumber::TS && current_time < start_time + contact_cst_et_ && be_before_mode != ModeNumber::SS;
      bool rf_heel_off_contact = current_mode == ModeNumber::ST && current_time < start_time + contact_cst_et_ && be_before_mode != ModeNumber::SS;

      if (((current_mode == ModeNumber::SF || current_mode == ModeNumber::FS) && current_time >= switch_time - contact_cst_st_) ||
          ((current_mode == ModeNumber::SF || current_mode == ModeNumber::FS) && current_time <= start_time + contact_cst_et_) ||
          current_mode == ModeNumber::SH || current_mode == ModeNumber::TS ||
          current_mode == ModeNumber::HS || current_mode == ModeNumber::ST || to_double_contact)
      {
        jointCmdMsg.joint_kp[3] = joint_kp_walking_[3];
        jointCmdMsg.joint_kp[9] = joint_kp_walking_[9];
        jointCmdMsg.joint_kd[3] = joint_kd_walking_[3];
        jointCmdMsg.joint_kd[9] = joint_kd_walking_[9];
      }

      // 踝关节全程力控+pd
      jointCmdMsg.control_modes[4] = 0;
      jointCmdMsg.control_modes[5] = 0;
      jointCmdMsg.control_modes[10] = 0;
      jointCmdMsg.control_modes[11] = 0;
      if (isPullUp_)
      {
        for (int i = 0; i < jointNumReal_; i++)
        {
          // if (i == 4 || i == 5 || i == 10 || i == 11) // 踝关节
          // {
          //   jointCmdMsg.control_modes[i+waistNum_] = 0;
          //   jointCmdMsg.tau[i+waistNum_] = 0;
          //   jointCmdMsg.joint_kp[i+waistNum_] = 0;
          //   jointCmdMsg.joint_kd[i+waistNum_] = 0;
          // }
          // else
            jointCmdMsg.control_modes[i] = 2;
        }
      }
      if (!is_stance_mode_)
      {
        if (std::any_of(contactFlag_.begin(), contactFlag_.begin() + 4, [](int flag)
                        { return !flag; }))
        {
          jointCmdMsg.joint_kp[4] = joint_kp_walking_[4];
          jointCmdMsg.joint_kp[5] = joint_kp_walking_[5];
          jointCmdMsg.joint_kd[4] = joint_kd_walking_[4];
          jointCmdMsg.joint_kd[5] = joint_kd_walking_[5];
        }

        if (std::any_of(contactFlag_.begin() + 4, contactFlag_.end(), [](int flag)
                        { return !flag; }))
        {
          jointCmdMsg.joint_kp[10] = joint_kp_walking_[10];
          jointCmdMsg.joint_kp[11] = joint_kp_walking_[11];
          jointCmdMsg.joint_kd[10] = joint_kd_walking_[10];
          jointCmdMsg.joint_kd[11] = joint_kd_walking_[11];
        }
      }
      else
      {
        jointCmdMsg.joint_kp[4] = 0.0;
        jointCmdMsg.joint_kp[5] = 0.0;
        jointCmdMsg.joint_kd[4] = 0.0;
        jointCmdMsg.joint_kd[5] = 0.0;
        jointCmdMsg.joint_kp[10] = 0.0;
        jointCmdMsg.joint_kp[11] = 0.0;
        jointCmdMsg.joint_kd[10] = 0.0;
        jointCmdMsg.joint_kd[11] = 0.0;
      }

      // 补全手臂的Cmd维度
      for(int i2 = 0; i2 < armNumReal_; ++i2)
      {
        jointCmdMsg.joint_q.push_back(output_pos_(waistNum_+jointNum_+i2));
        jointCmdMsg.joint_v.push_back(output_vel_(waistNum_+jointNum_+i2));
        jointCmdMsg.tau.push_back(output_tau_(waistNum_+jointNum_+i2));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[waistNum_+jointNum_+i2]);
        jointCmdMsg.control_modes.push_back(joint_control_modes_[waistNum_+jointNum_+i2]);
        jointCmdMsg.joint_kp.push_back(0);
        jointCmdMsg.joint_kd.push_back(0);
        // jointCurrentWBC_(jointNum_+i2) = output_tau_(jointNum_+i2);
      }

      // 补充头部维度
      // 计算头部反馈力
      if (headNum_ > 0)
      {
        vector_t get_head_pos = vector_t::Zero(headNum_);
        head_mtx.lock();
        get_head_pos = desire_head_pos_;
        head_mtx.unlock();
        auto &hardware_settings = kuavo_settings_.hardware_settings;
        vector_t head_feedback_tau = vector_t::Zero(headNum_);
        vector_t head_feedback_vel = vector_t::Zero(headNum_);
        if (!is_real_) // 实物不需要头部反馈力，来自kuavo仓库的移植
          head_feedback_tau = head_kp_.cwiseProduct(get_head_pos - sensor_data_head_.jointPos_) + head_kd_.cwiseProduct(-sensor_data_head_.jointVel_);
        for (int i3 = 0; i3 < headNum_; ++i3)
        {
          auto cur_head_pos = sensor_data_head_.jointPos_ * TO_DEGREE;
          auto vel = (get_head_pos[i3] - sensor_data_head_.jointPos_[i3]) * TO_DEGREE / dt_ * ruiwo_motor_velocities_factor_;
          double head_limit_vel = hardware_settings.joint_velocity_limits[waistNum_+jointNum_ + armNumReal_ + i3];

          vel = std::clamp(vel, -head_limit_vel, head_limit_vel) * TO_RADIAN;
          jointCmdMsg.joint_q.push_back(get_head_pos(i3));
          jointCmdMsg.joint_v.push_back(0);
          jointCmdMsg.tau.push_back(head_feedback_tau(i3));
          jointCmdMsg.tau_ratio.push_back(1);
          jointCmdMsg.tau_max.push_back(10);
          jointCmdMsg.control_modes.push_back(2);
          jointCmdMsg.joint_kp.push_back(0);
          jointCmdMsg.joint_kd.push_back(0);
          // jointCurrentWBC_(jointNum_ + armNumReal_ + i3) = get_head_pos(i3);
        }
        robotVisualizer_->updateHeadJointPositions(sensor_data_head_.jointPos_);
      }


    }
    else{

      vector_t measuredRbdStateRL_;
      measuredRbdStateRL_ = getRobotState();
      if (is_mpc_controller_) // 处理一次从MPC切换的逻辑
      {
        std::cout << "HumanoidController::update: pause MPC" << std::endl;
        mrtRosInterface_->pauseResumeMpcNode(true);
      }
      is_mpc_controller_ = false;

      // // 躯干插值已完成：开启 RL 推理并暂停 MPC
      // if (!inference_running_)
      // {
      //   inference_running_ = true;
      //   mrtRosInterface_->pauseResumeMpcNode(true);
      //   std::cout << "[MPC->RL] torso alignment done, pause MPC and enable RL inference" << std::endl;
      // }

      // 更新RL步态接收器
      if (rl_gait_receiver_) {
        vector_t feetPositions = stateEstimate_->getEndEffectorPositions();
        vector_t baseState = stateEstimate_->getTorsoState();  // 使用状态估计模块提供的接口
        rl_gait_receiver_->update(time, baseState, feetPositions);
      }

      vector_t optimizedState_mrt, optimizedInput_mrt;
      auto &info = centroidalModelInfoWBC_;
      optimizedState_mrt_ = initial_statusRL_;
      optimizedInput_mrt_ = intail_input_;
      // optimized_mode_ = plannedMode_;
      auto current_jointPos = measuredRbdStateRL_.segment(6, info.actuatedDofNum);
      auto current_jointVel = measuredRbdStateRL_.segment(12 + jointNumReal_ + armNumReal_ + waistNum_, info.actuatedDofNum);
       
      Eigen::VectorXd actuation(jointNumReal_ + armNumReal_ + waistNum_);
      actuation.setZero();
      
      // 获取当前RL命令数据并更新CommandDataRL_
      if (rl_gait_receiver_) {
        CommandDataRL rlCmd = rl_gait_receiver_->getCurrentCommand();
        setCommandDataRL(rlCmd);
      }
      
      actuation = updateRLcmd(measuredRbdStateRL_);
      
      if (!is_real_)
      {
        for (int i1 = 0; i1 < jointNumReal_ + armNumReal_+ waistNum_; ++i1)
        {
          jointCmdMsg.joint_q.push_back(0.0);
          jointCmdMsg.joint_v.push_back(0.0);
          jointCmdMsg.joint_kp.push_back(jointKpRL_[i1]);
          jointCmdMsg.joint_kd.push_back(jointKdRL_[i1]);
          jointCmdMsg.tau.push_back(actuation(i1));
          jointCmdMsg.tau_ratio.push_back(1);
          jointCmdMsg.tau_max.push_back(torqueLimitsRL_[i1]);
          jointCmdMsg.control_modes.push_back(JointControlModeRL_(i1));
          // std::cout << "joint_kp: " << jointKp_[i1] << " joint_kd: " << jointKd_[i1] << std::endl;
        }
      }
      else
      {
        for (int i1 = 0; i1 < jointNumReal_ + armNumReal_ + waistNum_; ++i1)
        {
          if (JointControlModeRL_(i1) == 0)
          {
            if (JointPDModeRL_(i1) == 0)
            {
              jointCmdMsg.joint_q.push_back(0.0);
              jointCmdMsg.joint_v.push_back(0.0);
              jointCmdMsg.joint_kp.push_back(0);
              jointCmdMsg.joint_kd.push_back(0);
              jointCmdMsg.tau.push_back(actuation(i1));
              jointCmdMsg.tau_ratio.push_back(1);
              jointCmdMsg.tau_max.push_back(torqueLimitsRL_[i1]);
              jointCmdMsg.control_modes.push_back(JointControlModeRL_(i1));
            }
            else
            {
              jointCmdMsg.joint_q.push_back(actuation(i1));
              jointCmdMsg.joint_v.push_back(0.0);
              jointCmdMsg.joint_kp.push_back(jointKpRL_[i1]);
              jointCmdMsg.joint_kd.push_back(jointKdRL_[i1]);
              jointCmdMsg.tau.push_back(0.0);
              jointCmdMsg.tau_ratio.push_back(1);
              jointCmdMsg.tau_max.push_back(torqueLimitsRL_[i1]);
              jointCmdMsg.control_modes.push_back(JointControlModeRL_(i1));
            }
          }
          else
          {
            jointCmdMsg.joint_q.push_back(current_jointPos(i1));
            jointCmdMsg.joint_v.push_back(0.0);
            jointCmdMsg.joint_kp.push_back(jointKpRL_[i1]);
            jointCmdMsg.joint_kd.push_back(jointKdRL_[i1]);
            jointCmdMsg.tau.push_back(actuation(i1));
            jointCmdMsg.tau_ratio.push_back(1);
            jointCmdMsg.tau_max.push_back(torqueLimitsRL_[i1]);
            jointCmdMsg.control_modes.push_back(JointControlModeRL_(i1));
          }
        }
      }

      // 补充头部维度
      // 计算头部反馈力
      if (headNum_ > 0)
      {
        vector_t get_head_pos = vector_t::Zero(headNum_);
        head_mtx.lock();
        get_head_pos = desire_head_pos_;
        head_mtx.unlock();
        auto &hardware_settings = kuavo_settings_.hardware_settings;
        vector_t head_feedback_tau = vector_t::Zero(headNum_);
        vector_t head_feedback_vel = vector_t::Zero(headNum_);
        if (!is_real_) // 实物不需要头部反馈力，来自kuavo仓库的移植
          head_feedback_tau = head_kp_.cwiseProduct(get_head_pos - sensor_data_head_.jointPos_) + head_kd_.cwiseProduct(-sensor_data_head_.jointVel_);
        for (int i3 = 0; i3 < headNum_; ++i3)
        {
          auto cur_head_pos = sensor_data_head_.jointPos_ * TO_DEGREE;
          auto vel = (get_head_pos[i3] - sensor_data_head_.jointPos_[i3]) * TO_DEGREE / dt_ * ruiwo_motor_velocities_factor_;
          double head_limit_vel = hardware_settings.joint_velocity_limits[jointNumReal_ + waistNum_ + armNumReal_ + i3];

          vel = std::clamp(vel, -head_limit_vel, head_limit_vel) * TO_RADIAN;
          jointCmdMsg.joint_q.push_back(get_head_pos(i3));
          jointCmdMsg.joint_v.push_back(0);
          jointCmdMsg.joint_kp.push_back(0);
          jointCmdMsg.joint_kd.push_back(0);  
          jointCmdMsg.tau.push_back(head_feedback_tau(i3));
          jointCmdMsg.tau_ratio.push_back(1);
          jointCmdMsg.tau_max.push_back(10);
          jointCmdMsg.control_modes.push_back(2);
        }
      }

      // arm - 使用平滑插值控制//
      updateArmInterpolation(time, jointCmdMsg); 



      // 规范化jointCmd尺寸，确保与硬件/仿真期望一致
      {
        auto adjust_double = [&](std::vector<double>& v, size_t n, double fill){
          if (v.size() < n) v.resize(n, fill);
          else if (v.size() > n) v.resize(n);
        };
        auto adjust_int = [&](std::vector<int>& v, size_t n, int fill){
          if (v.size() < n) v.resize(n, fill);
          else if (v.size() > n) v.resize(n);
        };
        const size_t expected_size = static_cast<size_t>(jointNumReal_ + armNumReal_ + waistNum_ + headNum_);
        adjust_double(jointCmdMsg.joint_q, expected_size, 0.0);
        adjust_double(jointCmdMsg.joint_v, expected_size, 0.0);
        adjust_double(jointCmdMsg.tau, expected_size, 0.0);
        adjust_double(jointCmdMsg.tau_ratio, expected_size, 1.0);
        adjust_double(jointCmdMsg.tau_max, expected_size, 0.0);
        adjust_double(jointCmdMsg.joint_kp, expected_size, 0.0);
        adjust_double(jointCmdMsg.joint_kd, expected_size, 0.0);
        adjust_int(jointCmdMsg.control_modes, expected_size, 2);
      }

      {
        std::lock_guard<std::mutex> lock(joint_cmd_mutex_);
        for (int i1 = 0; i1 < jointNumReal_ + armNumReal_ + waistNum_; ++i1)
        {
          jointTorqueCmdRL_(i1) = jointCmdMsg.tau[i1];
        }
      }
      
    }

    // 发布控制命令
    publishControlCommands(jointCmdMsg);

    // Visualization
    if (visualizeHumanoid_ && resetting_mpc_state_ != ResettingMpcState::RESET_INITIAL_POLICY)
    {
      robotVisualizer_->updateSimplifiedArmPositions(simplifiedJointPos_);
      if (mrtRosInterface_->isPolicyUpdated())
        robotVisualizer_->update(currentObservation_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());

      // 更新灵巧手可视化
      robotVisualizer_->updateHandJointPositions(dexhand_joint_pos_);
    }

    // publish time cost
    std_msgs::Float64 msg;
    msg.data = wbcTimer_.getFrequencyInHz();
    wbcFrequencyPub_.publish(msg);
    msg.data = wbcTimer_.getLastIntervalInMilliseconds();
    wbcTimeCostPub_.publish(msg);

    static double last_ros_time = ros::Time::now().toSec();
    ros_logger_->publishValue("/monitor/time_cost/controller_loop_time", (ros::Time::now().toSec() - last_ros_time) * 1000);
    last_ros_time = ros::Time::now().toSec();
  
  }

  void humanoidController::applySensorData()
  {
    if (!sensorDataQueue.empty())
    {
      sensor_data_mutex_.lock();
      while (sensorDataQueue.size() > 10)
      {
        sensorDataQueue.pop();
        // ROS_WARN_STREAM("Sensor data queue size exceeds 10, pop one element");
      }
      SensorData data = sensorDataQueue.front();
      sensorDataQueue.pop();
      sensor_data_mutex_.unlock();

      applySensorData(data);
    }
  }
  
  void humanoidController::applySensorData(const SensorData &data)
  {
    if (is_simplified_model_)// 简化模型, 需要将实物维度转为MPC维度
    {

      jointPos_.head(jointNum_ + waistNum_) = data.jointPos_.head(jointNum_+ waistNum_);
      jointVel_.head(jointNum_ + waistNum_) = data.jointVel_.head(jointNum_+ waistNum_);
      jointAcc_.head(jointNum_ + waistNum_) = data.jointAcc_.head(jointNum_+ waistNum_);
      jointTorque_.head(jointNum_ + waistNum_) = data.jointTorque_.head(jointNum_+ waistNum_);

      for (int i = 0; i < 2; i++)
      {
        jointPos_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointPos_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        jointVel_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointVel_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        jointAcc_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointAcc_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        jointTorque_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointTorque_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        simplifiedJointPos_.segment(armDofDiff_ * i, armDofDiff_) = data.jointPos_.segment(jointNum_ + waistNum_ + armDofReal_ * i + armDofMPC_, armDofDiff_);

      }
    }
    else
    {
      jointPos_ = data.jointPos_;
      jointVel_ = data.jointVel_;
      jointAcc_ = data.jointAcc_;
      jointTorque_ = data.jointTorque_;
    }

    jointPosWBC_ = data.jointPos_;
    jointVelWBC_ = data.jointVel_;
    jointAccWBC_ = data.jointAcc_;
    jointCurrentWBC_ = data.jointTorque_;

    quat_ = quat_init.inverse() * data.quat_;
    angularVel_ = data.angularVel_;
    linearAccel_ = data.linearAccel_;
    orientationCovariance_ = data.orientationCovariance_;
    angularVelCovariance_ = data.angularVelCovariance_;
    linearAccelCovariance_ = data.linearAccelCovariance_;
    current_time_ = data.timeStamp_;
    // stateEstimate_->updateJointStates(jointPos_, jointVel_);
    stateEstimate_->updateImu(quat_, angularVel_, linearAccel_, orientationCovariance_, angularVelCovariance_, linearAccelCovariance_);
    // apply sensor data to rl
    auto sensor_data_copy = data.copy();
    applySensorDataRL(sensor_data_copy);
  }

  void humanoidController::applySensorDataRL(const SensorData &data)
  {
    SensorData sensor_data_copy = data;
    sensor_data_copy.jointPos_ = data.jointPos_;
    sensor_data_copy.jointVel_ = data.jointVel_;
    sensor_data_copy.jointAcc_ = data.jointAcc_;
    sensor_data_copy.jointCurrent_ = data.jointCurrent_;
    sensor_data_copy.quat_ = data.quat_;
    sensor_data_copy.angularVel_ = data.angularVel_;
    sensor_data_copy.linearAccel_ = data.linearAccel_;
    sensor_data_copy.freeLinearAccel_ = data.freeLinearAccel_;
    sensor_data_copy.quat_offset_ = stateEstimate_->getImuOrientation();
    setRobotSensorData(sensor_data_copy);
  }

  void humanoidController::updateStateEstimation(const ros::Time &time, bool is_init)
  {
    {
      if (reset_estimator_)
      {
        stateEstimate_->reset();
        reset_estimator_ = false;
      }

      contact_flag_t contactFlag;
      // vector_t measuredRbdStateRL_;
      // measuredRbdStateRL_ = getRobotState();
      // vector3_t angularVel, linearAccel;
      // matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
      SensorData sensors_data;
      if (is_init)
        sensors_data = sensors_data_buffer_ptr_->getLastData();
      else
        sensors_data = sensors_data_buffer_ptr_->getLastData();
      // SensorData &sensors_data = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
      applySensorData(sensors_data);

      if (is_init)
      {
        last_time_ = current_time_ - ros::Duration(0.001);
        stateEstimate_->updateJointStates(jointPos_, jointVel_);
        quat_init = stateEstimate_->updateIntialEulerAngles(quat_);
        applySensorData(sensors_data);
        stateEstimate_->set_intial_state(currentObservation_.state);
        measuredRbdState_ = stateEstimate_->getRbdState();

      }
      double diff_time = (current_time_ - last_time_).toSec();

      last_time_ = current_time_;
      ros::Duration period = ros::Duration(diff_time);


      vector_t activeTorque_ = jointTorque_;
      vector_t activeTorqueWBC_ =  jointCurrentWBC_;
      stateEstimate_->setCmdTorque(activeTorque_);
      stateEstimate_->estContactForce(period);
      auto est_contact_force = stateEstimate_->getEstContactForce();
      contactForce_ = est_contact_force;
      ros_logger_->publishVector("/state_estimate/Contact_Detection/contactForce", est_contact_force);
      if (is_rl_controller_)
      {
        plannedMode_ = rl_plannedMode_;
        ros_logger_->publishValue("/rl_controller/rl_optimized_mode_", static_cast<double>(plannedMode_));
      }
      auto est_mode = stateEstimate_->ContactDetection(nextMode_, is_stance_mode_, plannedMode_, robotMass_, est_contact_force(2), est_contact_force(8), diff_time);
      ros_logger_->publishValue("/state_estimate/Contact_Detection/mode", static_cast<double>(est_mode));
      if (!use_estimator_contact_)
      {
        est_mode = plannedMode_;
      }
      stateEstimate_->updateMode(est_mode);
      stateEstimate_->updateGait(gaitManagerPtr_->getGaitName(currentObservation_.time));
      // rbdState_: Angular(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
      if (diff_time > 0.00005 || is_init)
      {
        Eigen::VectorXd updated_joint_pos = jointPos_;
        Eigen::VectorXd updated_joint_vel = jointVel_;
        Eigen::VectorXd updated_joint_torque = jointTorque_;
  #ifdef KUAVO_CONTROL_LIB_FOUND
        if (use_joint_filter_)
        {
          joint_filter_ptr_->update(measuredRbdState_, updated_joint_pos, updated_joint_vel, updated_joint_torque, output_tau_, est_mode);
        }
  #endif
        stateEstimate_->updateJointStates(updated_joint_pos, updated_joint_vel); // 使用关节滤波之后的jointPos和jointVel更新状态估计器
        // stateEstimate_->updateKinematics(period);
        updatakinematics(sensors_data, is_initialized_);
        measuredRbdState_ = stateEstimate_->update(time, period);                // angle(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
        currentObservation_.time += period.toSec();
      }
      // 只有非半身轮臂模式站立状态&&站起来稳定之后进行保护, 并且手臂不是外部遥操作模式才可触发拉起保护
      bool enable_pull_up = enable_pull_up_protect_ &&  !is_rl_controller_ && isPreUpdateComplete && is_stance_mode_ && 
        !only_half_up_body_ && currentObservation_.time - standupTime_ > 4 
        && mpcArmControlMode_ != ArmControlMode::EXTERN_CONTROL && resetting_mpc_state_ == ResettingMpcState::NOMAL;

      bool new_pull_up_state = false;
      if (enable_pull_up)
      {
        new_pull_up_state = stateEstimate_->checkPullUp(pull_up_force_threshold_);
      }
      ros_logger_->publishValue("/state_estimate/pull_up_state", isPullUp_);
      if (enable_pull_up &&  new_pull_up_state && !isPullUp_)
      {
        ROS_WARN_STREAM("Pull up detected");
        isPullUp_ = true;
        setPullUpState_=true;
        pull_up_trigger_time_ = currentObservation_.time;  // 记录触发时间
      }
      else if (isPullUp_ && (currentObservation_.time - pull_up_trigger_time_ > 2.0))
      {
        ROS_WARN_STREAM("Pull up protection triggered - publishing stop_robot message");
        isPullUp_ = false;
        
        // 发布stop_robot话题
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        stop_pub_.publish(stop_msg);
        ROS_WARN_STREAM("stop_robot message published");
      }
      ros_logger_->publishVector("/state_estimate/measuredRbdState", measuredRbdState_);
      auto &info = HumanoidInterface_->getCentroidalModelInfo();


      scalar_t yawLast = currentObservation_.state(9);
      currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
      currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
      std_msgs::Float32MultiArray state;
      for (int i1 = 0; i1 < currentObservation_.state.rows(); ++i1)
      {
        state.data.push_back(currentObservation_.state(i1));
      }
      // RbdStatePub_.publish(state);
      // std::cout << "currentObservation_.state:" << currentObservation_.state.transpose() << std::endl;
      // currentObservation_.mode = stateEstimate_->getMode();
      // std::cout << "currentObservation_.mode:" << currentObservation_.mode << std::endl;
      // TODO: 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
      // currentObservation_.mode = plannedMode_;
      currentObservation_.mode = plannedMode_;
      if (is_simplified_model_)
      {

        for (int i = 0; i < 2; i++)// qv
        {
          // 躯干+腿部自由度
          measuredRbdStateReal_.segment(centroidalModelInfoWBC_.generalizedCoordinatesNum * i, 6 + jointNum_ + waistNum_) =
              measuredRbdState_.segment(info.generalizedCoordinatesNum * i, 6 + jointNum_ + waistNum_);

          // 共有的手臂关节
          int arm_start_index = centroidalModelInfoWBC_.generalizedCoordinatesNum * i + 6 + jointNum_ + waistNum_;
          int arm_start_index_mpc = info.generalizedCoordinatesNum * i + 6 + jointNum_ + waistNum_;
          for (int j = 0; j < 2; j++) // 左右手
          {
            measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j, armDofMPC_) =
                measuredRbdState_.segment(arm_start_index_mpc + armDofMPC_ * j, armDofMPC_);

            // 简化的手臂关节部分从传感器数据获取
            vector_t joint_qv(sensors_data.jointPos_.size() * 2);
            joint_qv << sensors_data.jointPos_, sensors_data.jointVel_;
            int sensors_joint_num = sensors_data.jointPos_.size();
            measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j + armDofMPC_, armDofDiff_) =
              joint_qv.segment(sensors_joint_num * i + jointNum_ + waistNum_ + armDofReal_ * j + armDofMPC_, armDofDiff_);
          }
        }

        // obs
        currentObservationWBC_.state.head(info.stateDim) = currentObservation_.state;
        currentObservationWBC_.input.head(info.inputDim) = currentObservation_.input;
        currentObservationWBC_.state.tail(armNumReal_).setZero();
        currentObservationWBC_.input.tail(armNumReal_).setZero();
        // 共有部分
        for (int i = 0; i < 2; i++)
        {
          currentObservationWBC_.state.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
              currentObservation_.state.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
          currentObservationWBC_.input.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
              currentObservation_.input.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
        }
        // 手臂target后半简化部分
      int arm_start_index = 6 + jointNum_ + waistNum_;
        for (int i = 0; i < 2; i++)
        {
          currentObservationWBC_.state.tail(armNumReal_).segment(i * armDofReal_ + armDofMPC_, armDofDiff_) =
              measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * i + armDofMPC_, armDofDiff_);
        }

        currentObservationWBC_.time = currentObservation_.time;
        currentObservationWBC_.mode = currentObservation_.mode;

      }
      else
      {
        currentObservationWBC_ = currentObservation_;
        measuredRbdStateReal_ = measuredRbdState_;
      }
      wbc_observation_publisher_.publish(ros_msg_conversions::createObservationMsg(currentObservationWBC_));
      setRobotState(measuredRbdStateReal_);

      // std::cout << "jointPosWBC_:" << jointPosWBC_.transpose() << std::endl;

      auto est_arm_contact_force = stateEstimate_->getEstArmContactForce(jointPosWBC_, jointVelWBC_, activeTorqueWBC_, period);
      ros_logger_->publishVector("/state_estimate/est_arm_contact_force", est_arm_contact_force);
    }
  }

  void humanoidController::publishHumanoidState(const vector_t& rbdState)
  {
    humanoidState_ << rbdState.segment(3, 3), rbdState.segment(0, 3), rbdState.segment(6 + jointNum_, jointArmNum_);
    humanoidState_.head(2) *= -1; // 坐标系转换为跟kuavo-ros-control仓库一致
    humanoidState_(2) -= initialStateRL_(8); // 减去默认的高度
    std_msgs::Float64MultiArray state_msg;
    state_msg.data.resize(humanoidState_.size());
    for (int i = 0; i < humanoidState_.size(); ++i)
    {
      state_msg.data[i] = humanoidState_(i);
    }
    humanoidStatePublisher_.publish(state_msg);
  }

  humanoidController::~humanoidController()
  {
    controllerRunning_ = false;
    if (mpcThread_.joinable())
    {
      mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
  }

  void humanoidController::setupHumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitCommandFile,
                                                  bool verbose,  RobotVersion rb_version)
  {
    HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile, gaitCommandFile, rb_version);
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                      HumanoidInterface_->getCentroidalModelInfo());
    // **************** create the centroidal model for WBC ***********
    // PinocchioInterface
    auto &modelSettings_ = HumanoidInterface_->modelSettings();
    pinocchioInterfaceWBCPtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNamesReal)));
    pinocchioInterfaceEstimatePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNamesReal)));

    vector_t defaultJointState(pinocchioInterfaceWBCPtr_->getModel().nq);
    defaultJointState.setZero();
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
    defaultJointState.head(6 + jointNum_) = drake_interface_->getInitialState().head(6 + jointNum_);

    // CentroidalModelInfo
    centroidalModelInfoWBC_ = centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfaceWBCPtr_, centroidal_model::loadCentroidalType(taskFile), defaultJointState, modelSettings_.contactNames3DoF,
        modelSettings_.contactNames6DoF);
    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfoWBC_);

    eeKinematicsWBCPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(*pinocchioInterfaceWBCPtr_, pinocchioMapping,
                                                                    modelSettings_.contactNames3DoF);
    eeKinematicsWBCPtr_->setPinocchioInterface(*pinocchioInterfaceWBCPtr_);
  
    eeSpatialKinematicsWBCPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(*pinocchioInterfaceWBCPtr_, pinocchioMapping,
                                                                                         modelSettings_.contactNames6DoF);
    eeSpatialKinematicsWBCPtr_->setPinocchioInterface(*pinocchioInterfaceWBCPtr_);

    centroidalModelInfoEstimate_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfaceEstimatePtr_, centroidal_model::loadCentroidalType(taskFile), defaultJointState, modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);



  }

  void humanoidController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    return;
    
  }

  
  void humanoidController::setupMpc()
  {
    std::cout << "use_external_mpc_:" << use_external_mpc_ << std::endl;
    if (use_external_mpc_)
      return;
    // mpc_ = std::make_shared<SqpMpc>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->sqpSettings(),
    //                                 HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());
    mpc_ = std::make_shared<GaussNewtonDDP_MPC>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->ddpSettings(), HumanoidInterface_->getRollout(),
                                                HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());


    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(controllerNh_, HumanoidInterface_->getSwitchedModelReferenceManagerPtr(), robotName_);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName_, HumanoidInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(controllerNh_);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);
  }

  void humanoidController::setupMrt()
  {

    mrtRosInterface_ = std::make_shared<MRT_ROS_Interface>(robotName_);
    mrtRosInterface_->launchNodes(controllerNh_);
    return;
  }

  ocs2_msgs::mpc_flattened_controller humanoidController::createMpcPolicyMsg(const PrimalSolution &primalSolution,
                                                                             const CommandData &commandData,
                                                                             const PerformanceIndex &performanceIndices)
  {
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

    mpcPolicyMsg.initObservation = ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);
    mpcPolicyMsg.planTargetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
    mpcPolicyMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
    mpcPolicyMsg.performanceIndices =
        ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

    switch (primalSolution.controllerPtr_->getType())
    {
    case ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    case ControllerType::LINEAR:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
    }

    // maximum length of the message
    const size_t N = primalSolution.timeTrajectory_.size();

    mpcPolicyMsg.timeTrajectory.clear();
    mpcPolicyMsg.timeTrajectory.reserve(N);
    mpcPolicyMsg.stateTrajectory.clear();
    mpcPolicyMsg.stateTrajectory.reserve(N);
    mpcPolicyMsg.data.clear();
    mpcPolicyMsg.data.reserve(N);
    mpcPolicyMsg.postEventIndices.clear();
    mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

    // time
    for (auto t : primalSolution.timeTrajectory_)
    {
      mpcPolicyMsg.timeTrajectory.emplace_back(t);
    }

    // post-event indices
    for (auto ind : primalSolution.postEventIndices_)
    {
      mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
    }

    // state
    for (size_t k = 0; k < N; k++)
    {
      ocs2_msgs::mpc_state mpcState;
      mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
      for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++)
      {
        mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
      }
      mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
    } // end of k loop

    // input
    for (size_t k = 0; k < N; k++)
    {
      ocs2_msgs::mpc_input mpcInput;
      mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
      for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++)
      {
        mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
      }
      mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
    } // end of k loop

    // controller
    scalar_array_t timeTrajectoryTruncated;
    std::vector<std::vector<float> *> policyMsgDataPointers;
    policyMsgDataPointers.reserve(N);
    for (auto t : primalSolution.timeTrajectory_)
    {
      mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

      policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
      timeTrajectoryTruncated.push_back(t);
    } // end of k loop

    // serialize controller into data buffer
    primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

    return mpcPolicyMsg;
  }

  void humanoidController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
    // 这部分只有下肢，可能需要修改。
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);

    currentObservation_.time = 0;
    stateEstimate_->initializeEstArmContactForce(*pinocchioInterfaceEstimatePtr_, centroidalModelInfoEstimate_);
  }

  void humanoidCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                              HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
  }

  void humanoidKuavoController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
#ifdef KUAVO_CONTROL_LIB_FOUND
    // auto [plant, context] = drake_interface_->getPlantAndContext();
    stateEstimate_ = std::make_shared<InEkfBaseFilter>(HumanoidInterface_->getPinocchioInterface(),
                                                       HumanoidInterface_->getCentroidalModelInfo(),
                                                       *eeKinematicsPtr_,
                                                       drake_interface_,
                                                       dt_,
                                                       ros_logger_);
    std::cout << "InEkfBaseFilter stateEstimate_ initialized" << std::endl;
#endif
  }

  bool humanoidController::updateSensorDataFromShm()
  {
    if (!use_shm_communication_ || !shm_manager_) {
        return false;
    }

    gazebo_shm::SensorsData sensors_data;
    if (shm_manager_->readSensorsData(sensors_data)) {
        // std::cout << "sensors_data.sensor_time: "<< sensors_data.sensor_time << std::endl;
        if (waistNum_ > 0)
        {
          for (size_t i = 0; i < waistNum_; ++i)
          {
            sensor_data_waist_.jointPos_(i) = sensors_data.joint_data[i].position;
            sensor_data_waist_.jointVel_(i) = sensors_data.joint_data[i].velocity;
            sensor_data_waist_.jointAcc_(i) = 0.0;
            sensor_data_waist_.jointTorque_(i) = sensors_data.joint_data[i].effort;
          }
        }

        SensorData sensor_data;
        sensor_data.resize_joint(jointNumReal_+ armNumReal_ + waistNum_);
        
        // 关节数据
        for (size_t i = 0; i < waistNum_+jointNumReal_+armNumReal_; ++i) {
            sensor_data.jointPos_(i) = sensors_data.joint_data[i].position;
            sensor_data.jointVel_(i) = sensors_data.joint_data[i].velocity;
            sensor_data.jointAcc_(i) = 0.0;  // 加速度在共享内存中未提供
            sensor_data.jointTorque_(i) = sensors_data.joint_data[i].effort;
        }
        
        // IMU数据
        sensor_data.quat_.coeffs() << sensors_data.imu_data.orientation[0],
                                    sensors_data.imu_data.orientation[1],
                                    sensors_data.imu_data.orientation[2],
                                    sensors_data.imu_data.orientation[3];
                                    
        sensor_data.angularVel_ << sensors_data.imu_data.angular_velocity[0],
                                  sensors_data.imu_data.angular_velocity[1],
                                  sensors_data.imu_data.angular_velocity[2];
                                  
        sensor_data.linearAccel_ << sensors_data.imu_data.linear_acceleration[0],
                                   sensors_data.imu_data.linear_acceleration[1],
                                   sensors_data.imu_data.linear_acceleration[2];

        // 填充IMU数据到ROS消息
        kuavo_msgs::sensorsData msg;
        msg.header.stamp = ros::Time(sensors_data.sensor_time);
        msg.header.frame_id = "base_link";
        
        // 关节数据
        for (size_t i = 0; i < waistNum_+jointNumReal_+armNumReal_; ++i) {
            msg.joint_data.joint_q.push_back(sensors_data.joint_data[i].position);
            msg.joint_data.joint_v.push_back(sensors_data.joint_data[i].velocity);
            msg.joint_data.joint_vd.push_back(0.0);
            msg.joint_data.joint_torque.push_back(sensors_data.joint_data[i].effort);
        }
        
        // IMU数据
        msg.imu_data.quat.w = sensors_data.imu_data.orientation[3];
        msg.imu_data.quat.x = sensors_data.imu_data.orientation[0];
        msg.imu_data.quat.y = sensors_data.imu_data.orientation[1];
        msg.imu_data.quat.z = sensors_data.imu_data.orientation[2];
        
        msg.imu_data.gyro.x = sensors_data.imu_data.angular_velocity[0];
        msg.imu_data.gyro.y = sensors_data.imu_data.angular_velocity[1];
        msg.imu_data.gyro.z = sensors_data.imu_data.angular_velocity[2];
        
        msg.imu_data.acc.x = sensors_data.imu_data.linear_acceleration[0];
        msg.imu_data.acc.y = sensors_data.imu_data.linear_acceleration[1];
        msg.imu_data.acc.z = sensors_data.imu_data.linear_acceleration[2];

        // 设置协方差矩阵为零
        sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
        sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
        sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
        
        // 更新时间戳
        sensor_data.timeStamp_ = ros::Time(sensors_data.sensor_time);
        ros_logger_->publishVector("/state_estimate/imu_data_ori/linearAccel", sensor_data.linearAccel_);
        ros_logger_->publishVector("/state_estimate/imu_data_ori/angularVel", sensor_data.angularVel_);
        
        // 应用滤波器
        sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
        sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
        
        // 记录数据
        ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
        ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
        
        // 添加到数据缓冲区
        sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);
        
        // 处理头部关节数据（如果有）
        if (headNum_ > 0 && sensors_data.num_joints == jointNumReal_+armNumReal_+headNum_+waistNum_) {
            int head_start_index = sensors_data.num_joints - headNum_;
            for (size_t i = 0; i < headNum_; ++i) {
                sensor_data_head_.jointPos_(i) = sensors_data.joint_data[i + head_start_index].position;
                sensor_data_head_.jointVel_(i) = sensors_data.joint_data[i + head_start_index].velocity;
                sensor_data_head_.jointAcc_(i) = 0.0;
                sensor_data_head_.jointTorque_(i) = sensors_data.joint_data[i + head_start_index].effort;
                
                // 添加头部关节数据到ROS消息
                msg.joint_data.joint_q.push_back(sensors_data.joint_data[i + head_start_index].position);
                msg.joint_data.joint_v.push_back(sensors_data.joint_data[i + head_start_index].velocity);
                msg.joint_data.joint_vd.push_back(0.0);
                msg.joint_data.joint_torque.push_back(sensors_data.joint_data[i + head_start_index].effort);
            }
        }
        
        if (!is_initialized_) {
            is_initialized_ = true;
        }
        sensor_data_raw_pub_.publish(msg);
        return true;
    }
    return false;
  }

  void humanoidController::publishJointCmdToShm(const kuavo_msgs::jointCmd& jointCmdMsg)
  {
    if (!use_shm_communication_ || !shm_manager_) {
        return;
    }

    gazebo_shm::JointCommand joint_cmd;
    joint_cmd.num_joints = waistNum_ + jointNumReal_ + armNumReal_ + headNum_;

    // 从jointCmdMsg中复制数据到共享内存结构
    for (size_t i = 0; i < joint_cmd.num_joints; ++i) {
        joint_cmd.joint_q[i] = jointCmdMsg.joint_q[i];
        joint_cmd.joint_v[i] = jointCmdMsg.joint_v[i];
        joint_cmd.tau[i] = jointCmdMsg.tau[i];
        joint_cmd.tau_max[i] = jointCmdMsg.tau_max[i];
        joint_cmd.joint_kp[i] = jointCmdMsg.joint_kp[i];
        joint_cmd.joint_kd[i] = jointCmdMsg.joint_kd[i];
        joint_cmd.control_modes[i] = jointCmdMsg.control_modes[i];
    }

    // 写入共享内存
    shm_manager_->writeJointCommandNext(joint_cmd);
  }

  void humanoidController::publishControlCommands(const kuavo_msgs::jointCmd& jointCmdMsg)
  {
    // 发布控制命令
#ifdef USE_DDS
    // Publish via DDS when DDS is enabled
    if (dds_client_) {
        // Convert jointCmdMsg to DDS LowCmd_
        unitree_hg::msg::dds_::LowCmd_ low_cmd;
        
        // Initialize motor commands array (35 motors)
        for (size_t i = 0; i < 35; ++i) {
            auto& motor_cmd = low_cmd.motor_cmd()[i];
            if (i < jointCmdMsg.joint_q.size()) {
                // Map data from jointCmdMsg to DDS motor command
                motor_cmd.mode(static_cast<uint8_t>(jointCmdMsg.control_modes[i]));
                motor_cmd.q(static_cast<float>(jointCmdMsg.joint_q[i]));
                motor_cmd.dq(static_cast<float>(jointCmdMsg.joint_v[i]));
                motor_cmd.tau(static_cast<float>(jointCmdMsg.tau[i]));
                motor_cmd.kp(static_cast<float>(jointCmdMsg.joint_kp[i]));
                motor_cmd.kd(static_cast<float>(jointCmdMsg.joint_kd[i]));
                motor_cmd.reserve(0);
            } else {
                // Zero out unused motors
                motor_cmd.mode(0);
                motor_cmd.q(0.0f);
                motor_cmd.dq(0.0f);
                motor_cmd.tau(0.0f);
                motor_cmd.kp(0.0f);
                motor_cmd.kd(0.0f);
                motor_cmd.reserve(0);
            }
        }
        
        // Set mode machine and mode_pr
        low_cmd.mode_machine(1);  // Default mode
        low_cmd.mode_pr(1);       // Default mode
        
        // Calculate and set CRC
        uint32_t crc = Crc32Core((uint32_t*)&low_cmd, (sizeof(low_cmd) >> 2) - 1);
        low_cmd.crc(crc);
        
        dds_client_->publishLowCmd(low_cmd);
    }
#else
    // Use ROS and SHM publishing when DDS is disabled
    if (use_shm_communication_){
      publishJointCmdToShm(jointCmdMsg);
    } 
    jointCmdPub_.publish(jointCmdMsg);
    
    // // 发布到共享内存
    // if (use_shm_communication_) {
    //     publishJointCmdToShm(jointCmdMsg);
    // }
#endif
  }

  void humanoidController::visualizeWrench(const Eigen::VectorXd &wrench, bool is_left)
  {
    if(wrench.size() != 6)
      ROS_ERROR_STREAM("wrench size is not 6");
    // 创建并填充 WrenchStamped 消息
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time::now();  // 设置时间戳
    wrench_msg.header.frame_id = "zarm_r7_end_effector";
    if(is_left)
      wrench_msg.header.frame_id = "zarm_l7_end_effector";

    // TODO: 转换到局部坐标系
    // 将优化输入分割到力和力矩字段中
    wrench_msg.wrench.force.x = wrench(0); // 力 x
    wrench_msg.wrench.force.y = wrench(1); // 力 y
    wrench_msg.wrench.force.z = wrench(2); // 力 z

    wrench_msg.wrench.torque.x = wrench(3); // 力矩 x
    wrench_msg.wrench.torque.y = wrench(4); // 力矩 y
    wrench_msg.wrench.torque.z = wrench(5); // 力矩 z

    if(is_left)
      lHandWrenchPub_.publish(wrench_msg);
    else
      rHandWrenchPub_.publish(wrench_msg);
  }

  bool humanoidController::getCurrentGaitNameCallback(kuavo_msgs::getCurrentGaitName::Request &req, kuavo_msgs::getCurrentGaitName::Response &res) {
    if(gaitManagerPtr_) {
      res.gait_name = gaitManagerPtr_->getGaitName(currentObservation_.time);
      res.success = true;
    }
    else{
      res.gait_name = "none";
      res.success = false;
    }
    return true;
  }

  bool humanoidController::switchControllerCallback(kuavo_msgs::switchController::Request &req, kuavo_msgs::switchController::Response &res)
  {
    ROS_INFO("Received controller switch request: %s", req.controller_name.c_str());
    
    // Check if requested controller is in available list
    bool found = false;
    int new_index = -1;
    for (size_t i = 0; i < available_controllers_.size(); ++i) {
      if (available_controllers_[i] == req.controller_name) {
        found = true;
        new_index = i;
        break;
      }
    }
    
    if (found) {
      // 检查是否尝试切换到RL控制器，以及RL是否可用
      if (new_index != 0 && !rl_available_) {
        res.success = false;
        res.message = "RL controller is not available. RL parameter file not found.";
        ROS_WARN("[HumanoidController] %s", res.message.c_str());
        return true;
      }
      
      // Update current controller state
      current_controller_ = req.controller_name;
      current_controller_index_ = new_index;
      
      res.success = true;
      res.message = "Successfully switched to controller: " + req.controller_name + " (index: " + std::to_string(new_index) + ")";
      // switch to rl controller
      is_rl_controller_buffer_.setBuffer(!(current_controller_index_ == 0));
      // Here you can implement specific controller switching logic
      // e.g., stop current controller, start new controller, etc.
    } else {
      res.success = false;
      res.message = "Unsupported controller type: " + req.controller_name + ". Available controllers: ";
      for (size_t i = 0; i < available_controllers_.size(); ++i) {
        res.message += available_controllers_[i];
        if (i < available_controllers_.size() - 1) res.message += ", ";
      }
      ROS_WARN("[HumanoidController] %s", res.message.c_str());
    }
    
    return true;
  }

  bool humanoidController::getControllerListCallback(kuavo_msgs::getControllerList::Request &req, kuavo_msgs::getControllerList::Response &res)
  {
    res.controller_names.clear();
    res.controller_names = available_controllers_;
    res.count = available_controllers_.size();
    res.current_index = current_controller_index_;
    res.current_controller = current_controller_;
    res.success = true;
    res.message = "Successfully retrieved controller list, total " + std::to_string(res.count) + " controllers, current: " + current_controller_ + " (index: " + std::to_string(current_controller_index_) + ")";
    
    ROS_INFO("[HumanoidController] current controller: %s (index: %d, total %d),", current_controller_.c_str(), current_controller_index_, res.count);
    
    return true;
  }

  bool humanoidController::switchToNextControllerCallback(kuavo_msgs::switchToNextController::Request &req, kuavo_msgs::switchToNextController::Response &res)
  {
    if (available_controllers_.empty()) {
      res.success = false;
      res.message = "No controllers available";
      res.current_controller = "";
      res.next_controller = "";
      res.current_index = -1;
      res.next_index = -1;
      ROS_WARN("[HumanoidController] No controllers available for switching");
      return true;
    }

    // 保存当前控制器信息
    res.current_controller = current_controller_;
    res.current_index = current_controller_index_;

    // 计算下一个控制器的索引（循环切换），跳过不可用的RL控制器
    int next_index = current_controller_index_;
    int search_count = 0;
    do {
      next_index = (next_index + 1) % available_controllers_.size();
      search_count++;
      
      // 如果是RL控制器且RL不可用，跳过
      if (next_index != 0 && !rl_available_) {
        continue;
      }
      
      break;
    } while (search_count < available_controllers_.size());
    
    // 如果找不到可用的控制器（应该不会发生，因为至少MPC可用）
    if (search_count >= available_controllers_.size()) {
      res.success = false;
      res.message = "No available controllers to switch to";
      res.next_controller = "";
      res.next_index = -1;
      ROS_WARN("[HumanoidController] %s", res.message.c_str());
      return true;
    }

    std::string next_controller = available_controllers_[next_index];

    // 更新当前控制器状态
    current_controller_ = next_controller;
    current_controller_index_ = next_index;
    is_rl_controller_buffer_.setBuffer(!(current_controller_index_ == 0));
    // 设置响应信息
    res.next_controller = next_controller;
    res.next_index = next_index;
    res.success = true;
    res.message = "Successfully switched from " + res.current_controller + " (index: " + std::to_string(res.current_index) + 
                  ") to " + res.next_controller + " (index: " + std::to_string(res.next_index) + ")";

    ROS_INFO("[HumanoidController] %s", res.message.c_str());
    
    return true;
  }


  void humanoidController::waistCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    return;
  }

  void humanoidController::dexhandStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    if(msg->name.size() != dexhand_joint_pos_.size())
      return;
    for(size_t i = 0; i < dexhand_joint_pos_.size(); ++i)  
      dexhand_joint_pos_(i) = msg->position[i];
  }

  void humanoidController::getEnableMpcFlagCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if(msg->data == disable_mpc_)
    {
      ROS_INFO("Received enable mpc value: %s", msg->data ? "true" : "false");
      disable_mpc_ = !msg->data;
    }
    
    if(false == disable_mpc_)
    {
      ROS_INFO("reset Mpc controller");
      reset_mpc_ = true;
    }
  }

  void humanoidController::getEnableWbcFlagCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if(msg->data == disable_wbc_)
    {
      ROS_INFO("Received enable wbc value: %s", msg->data ? "true" : "false");
      disable_wbc_ = !msg->data;
    }
  }

  void humanoidController::publishWbcArmEndEffectorPose()
  {
    auto& infoWBC = centroidalModelInfoWBC_;
    // publish arm eef pose from WBC
    if (infoWBC.numSixDofContacts > 0 && eeSpatialKinematicsWBCPtr_)
    {
        // Manually update the pinocchio data for the WBC interface, similar to how WBC does it internally.
        auto& wbc_pinocchio_model = pinocchioInterfaceWBCPtr_->getModel();
        auto& wbc_pinocchio_data = pinocchioInterfaceWBCPtr_->getData();
        const auto q_wbc = CentroidalModelPinocchioMapping(centroidalModelInfoWBC_).getPinocchioJointPosition(currentObservationWBC_.state);
        pinocchio::forwardKinematics(wbc_pinocchio_model, wbc_pinocchio_data, q_wbc);
        pinocchio::updateFramePlacements(wbc_pinocchio_model, wbc_pinocchio_data);

        // Call with empty vector to use the pre-updated data
        const auto armPositions = eeSpatialKinematicsWBCPtr_->getPosition(vector_t());
        const auto armOrientations = eeSpatialKinematicsWBCPtr_->getOrientation(vector_t());

        if (armPositions.size() == infoWBC.numSixDofContacts)
        {
            std_msgs::Float64MultiArray pose_msg;
            pose_msg.data.resize(armPositions.size() * 7);
            for (size_t i = 0; i < armPositions.size(); ++i)
            {
                pose_msg.data[i * 7 + 0] = armPositions[i].x();
                pose_msg.data[i * 7 + 1] = armPositions[i].y();
                pose_msg.data[i * 7 + 2] = armPositions[i].z();
                pose_msg.data[i * 7 + 3] = armOrientations[i].x();
                pose_msg.data[i * 7 + 4] = armOrientations[i].y();
                pose_msg.data[i * 7 + 5] = armOrientations[i].z();
                pose_msg.data[i * 7 + 6] = armOrientations[i].w();
            }
            armEefWbcPosePublisher_.publish(pose_msg);
        }
    }
  }

  bool humanoidController::setupCpuIsolation()
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
              // 检查数组元素是否存在
              if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                std::cerr << "Error: array element " << i << " is invalid" << std::endl;
                continue;
              }
              
              // 尝试转换为double
              double value;
              if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                value = static_cast<int>(xml_cpus[i]);
              } else if (xml_cpus[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                value = static_cast<double>(xml_cpus[i]);
              } else {
                std::cerr << "Error: array element " << i << " is not a number, type: " << xml_cpus[i].getType() << std::endl;
                continue;
              }
              
              isolated_cpus.push_back(static_cast<int>(value));
            } catch (const std::exception& e) {
              std::cerr << "Error: parameter conversion failed, index " << i << ": " << e.what() << std::endl;
            }
          }
        }
        else{
          std::cerr << "Error: isolated_cpus is not an array, type: " << xml_cpus.getType() << std::endl;
        }
      }
      else{
        std::cerr << "Error: failed to get isolated_cpus parameter" << std::endl;
      }
    } else {
      std::cout << "未设置 /isolated_cpus 参数，跳过CPU亲和性设置" << std::endl;
      return false;
    }
    
    // 检查是否有隔离的核心
    if (isolated_cpus.size() >= 1) {
      bool ruiwo_isolated_core_ = false;
      // 检查CPU核心编号是否有效
      int max_cpu = sysconf(_SC_NPROCESSORS_ONLN);
      std::cout << "系统CPU核心数: " << max_cpu << std::endl;
      
      for (size_t i = 0; i < isolated_cpus.size(); ++i) {
        if (isolated_cpus[i] < 0 || isolated_cpus[i] >= max_cpu) {
          std::cerr << "警告: CPU核心 " << isolated_cpus[i] << " 超出有效范围 [0, " << max_cpu-1 << "]" << std::endl;
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
        
        // 查找 isolcpus 参数
        size_t isolcpus_pos = line.find("isolcpus=");
        if (isolcpus_pos != std::string::npos) {
          size_t start = isolcpus_pos + 9; // "isolcpus=" 长度为9
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
              // 处理范围
              int start_cpu = std::stoi(range.substr(0, dash_pos));
              int end_cpu = std::stoi(range.substr(dash_pos + 1));
              for (int j = start_cpu; j <= end_cpu; ++j) {
                isolcpus_list.push_back(j);
              }
            } else {
              // 处理单个CPU
              isolcpus_list.push_back(std::stoi(range));
            }
            
            if (comma_pos == std::string::npos) break;
            pos = comma_pos + 1;
          }
          std::cout << "已隔离CPU列表: ";
          for (size_t i = 0; i < isolcpus_list.size(); ++i) {
            if (isolcpus_list[i] == 7){
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
      

      // 判断每个CPU是否在隔离列表中
      for (size_t i = 0; i < isolated_cpus.size(); ++i) {
        int cpu_id = isolated_cpus[i];
        // 检查是否在 isolcpus 列表中
        if (std::find(isolcpus_list.begin(), isolcpus_list.end(), cpu_id) != isolcpus_list.end()) {
          actually_isolated_cpus.push_back(cpu_id);
          std::cout << "CPU " << cpu_id << " 已隔离" << std::endl;
        } else {
          std::cout << "CPU " << cpu_id << " 未隔离" << std::endl;
        }
      }
      if (!ruiwo_isolated_core_){    // 7 号核心未隔离，不允许启动
        std::cout << "7 号核心未隔离，跳过CPU亲和性设置" << std::endl;
        return false;
      }
    } else {
      std::cout << "隔离的核心列表为空，跳过CPU亲和性设置" << std::endl;
      return false;
    }

    // 只有在有真正隔离的CPU时才设置亲和性
    if (actually_isolated_cpus.size() >= 2) {           // 新版本至少需要两个核心： 2 个核心绑定 WBC
      // 设置CPU亲和性到隔离的核心
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      
      // 将所有真正隔离的核心添加到CPU集合中
      for (size_t i = 0; i < actually_isolated_cpus.size(); ++i) {
        CPU_SET(actually_isolated_cpus[i], &cpuset);
      }
      
      std::cout << "设置WBC线程亲和性到隔离核心: ";
      for (size_t i = 0; i < actually_isolated_cpus.size(); ++i) {
        std::cout << actually_isolated_cpus[i];
        if (i < actually_isolated_cpus.size() - 1) std::cout << ", ";
      }
      std::cout << std::endl;
      
      // 设置当前线程的CPU亲和性
      int result = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
      if (result != 0) {
        std::cerr << "警告: 设置线程CPU亲和性失败，错误码: " << result << " (" << strerror(result) << ")" << std::endl;
        return false;
      } else {
        std::cout << "成功设置CPU亲和性到隔离核心" << std::endl;
        return true;
      }
    } else {
      std::cout << "没有真正隔离的CPU核心或隔离的CPU核心数不足（至少需要2个核心，2个核心绑定WBC控制线程），跳过CPU亲和性设置" << std::endl;
      return false;
    }
  }
  void humanoidController::loadJoyJsonConfig(const std::string &config_file)
  {
    std::ifstream ifs(config_file);
    if (!ifs.is_open())
    {
      std::cerr << "Failed to open config file: " << config_file << std::endl;
      return;
    }
    nlohmann::json data_;
    ifs >> data_;
    auto updateMap = [](auto &map, const auto &items)
    {
      for (const auto &[key, value] : items)
      {
        std::cout << "button:" << key << " value:" << value << std::endl;
        map[key] = value;
      }
    };
    updateMap(joyButtonMap, data_["JoyButton"].items());
    updateMap(joyAxisMap, data_["JoyAxis"].items());
  }

  // ==================== 站立键平滑过渡系统函数实现 ====================
  
  void humanoidController::startStanceTransition(const ros::Time& current_time)
  {
    is_stance_transition_ = true;
    stance_transition_start_time_ = current_time;
    
    stance_transition_velocity_.linear.x = smoothed_cmd_vel_.linear.x;
    stance_transition_velocity_.linear.y = smoothed_cmd_vel_.linear.y;
    stance_transition_velocity_.linear.z = smoothed_cmd_vel_.linear.z;
    stance_transition_velocity_.angular.x = smoothed_cmd_vel_.angular.x;
    stance_transition_velocity_.angular.y = smoothed_cmd_vel_.angular.y;
    stance_transition_velocity_.angular.z = smoothed_cmd_vel_.angular.z;
    
    // ROS_INFO("[StanceTransition] 开始站立过渡，持续时间: %.1f秒", stance_transition_duration_);
    // ROS_INFO("[StanceTransition] 起始速度: lin(%.3f, %.3f, %.3f) ang(%.3f, %.3f, %.3f)", 
    //          stance_transition_velocity_.linear.x, stance_transition_velocity_.linear.y, stance_transition_velocity_.linear.z,
    //          stance_transition_velocity_.angular.x, stance_transition_velocity_.angular.y, stance_transition_velocity_.angular.z);
  }
  
  void humanoidController::stopStanceTransition()
  {
    is_stance_transition_ = false;
    // Walkenable_ = false;
    ROS_INFO("[StanceTransition] 站立过渡结束，Walkenable设置为False");
  }
  
  void humanoidController::updateStanceTransition(const ros::Time& current_time)
  {
    if (!is_stance_transition_)
      return;
    
    double elapsed_time = (current_time - stance_transition_start_time_).toSec();
    double alpha = elapsed_time / stance_transition_duration_;
    
    if (alpha >= 1.0)
    {
      stopStanceTransition();
      return;
    }
    
    double smooth_factor = 0.5 * (1.0 + cos(M_PI * alpha)); 
    stance_transition_velocity_.linear.x *= smooth_factor;
    stance_transition_velocity_.linear.y *= smooth_factor;
    stance_transition_velocity_.linear.z *= smooth_factor;
    stance_transition_velocity_.angular.x *= smooth_factor;
    stance_transition_velocity_.angular.y *= smooth_factor;
    stance_transition_velocity_.angular.z *= smooth_factor;
    
    ROS_DEBUG_STREAM("[StanceTransition] 过渡进度: " << (alpha * 100) << "%, 当前速度: lin(" 
                     << stance_transition_velocity_.linear.x << ", " << stance_transition_velocity_.linear.y << ", " << stance_transition_velocity_.linear.z 
                     << ") ang(" << stance_transition_velocity_.angular.x << ", " << stance_transition_velocity_.angular.y << ", " << stance_transition_velocity_.angular.z << ")");
  }

  // ==================== RL相关函数实现 ====================
  
  void humanoidController::printRLparam()
  {
    std::cout << "[RL param]:Inital Command: " << initialCommandDataRL_.getCommandRL().transpose() << std::endl;
    std::cout << "[RL param]:Start Using RL: " << is_rl_controller_ << std::endl;
    std::cout << "[RL param]:Joint Control Mode(0:CST, 1:CSV, 2:CSP):" << JointControlModeRL_.transpose() << std::endl;
    std::cout << "[RL param]:Joint PD mode:" << JointPDModeRL_.transpose() << std::endl;
    std::cout << "[RL param]:Joint Kp:" << jointKpRL_.transpose() << std::endl;
    std::cout << "[RL param]:Joint Kd:" << jointKdRL_.transpose() << std::endl;
    std::cout << "[RL param]:Initial State:" << initialStateRL_.transpose() << std::endl;
    std::cout << "[RL param]:Torque Limits:" << torqueLimitsRL_.transpose() << std::endl;
    std::cout << "[RL param]:Action Scale Test:" << actionScaleTestRL_.transpose() << std::endl;
    std::cout << "=============================================================================" << std::endl;
  }

  std::vector<bool> humanoidController::commandLineToTargetTrajectories(const vector_t &joystick_origin_axis, geometry_msgs::Twist &cmdVel)
  {
    std::vector<bool> updated(6, false);
    Eigen::Vector4d limit_vector;
    limit_vector = velocityLimitsRL_;
    limit_vector(0) *= 0.5;
    double dead_zone = 0.05;
    if (joystick_origin_axis.cwiseAbs().maxCoeff() < dead_zone)
      return updated; // command line is zero, do nothing
    if (abs(joystick_origin_axis(0)) > 0.5)
    {
      if (joystick_origin_axis(0) > 0)
      {
        limit_vector(0) *= 2;
      }
    }
    commadLineTarget_.head(4) = joystick_origin_axis.head(4).cwiseProduct(limit_vector);
    if (joystick_origin_axis.head(2).cwiseAbs().maxCoeff() > dead_zone)
    {
      cmdVel.linear.x = commadLineTarget_(0);
      cmdVel.linear.y = commadLineTarget_(1);
      updated[0] = true;
      updated[1] = true;
    }
    if (std::abs(joystick_origin_axis(2)) > dead_zone)
    {
      updated[2] = true;
      cmdVel.linear.z = commadLineTarget_(2);
    }
    else
    {
      cmdVel.linear.z = 0.0;
    }
    if (std::abs(commadLineTarget_(3)) > dead_zone)
    {
      updated[3] = true;
      cmdVel.angular.z = commadLineTarget_(3);
    }
    return updated;
  }

  // ==================== 速度平滑系统函数实现 ====================
  
  void humanoidController::initializeVelocitySmoothing()
  {
    // 已禁用速度平滑，无需初始化
  }
  
  geometry_msgs::Twist humanoidController::smoothVelocityCommand(const geometry_msgs::Twist& target_vel, const ros::Time& current_time)
  {
    geometry_msgs::Twist smoothed_vel = smoothed_cmd_vel_;
    
    // 计算时间差
    double dt = (current_time - last_velocity_update_time_).toSec();
    if (dt <= 0.0) dt = 0.001; // 避免除零错误
    
    // 各轴独立平滑，避免不同轴之间相互影响
    auto apply_axis_smooth = [&](double target, double current) -> double {
      double diff = target - current;
      double abs_target = std::abs(target);
      double factor = velocity_smooth_factor_;
      // 目标接近零时更快收敛
      if (abs_target < 0.01) {
        factor = velocity_smooth_factor_;
      } else if (std::abs(diff) > max_velocity_change_) {
        // 单轴变化较大时，仅该轴降低平滑因子
        factor = velocity_smooth_factor_ * 0.5;
      }
      return current + diff * factor;
    };

    smoothed_vel.linear.x = apply_axis_smooth(target_vel.linear.x, smoothed_vel.linear.x);
    smoothed_vel.linear.y = apply_axis_smooth(target_vel.linear.y, smoothed_vel.linear.y);
    smoothed_vel.linear.z = apply_axis_smooth(target_vel.linear.z, smoothed_vel.linear.z);
    smoothed_vel.angular.x = apply_axis_smooth(target_vel.angular.x, smoothed_vel.angular.x);
    smoothed_vel.angular.y = apply_axis_smooth(target_vel.angular.y, smoothed_vel.angular.y);
    smoothed_vel.angular.z = apply_axis_smooth(target_vel.angular.z, smoothed_vel.angular.z);
    
    // 更新时间和保存当前值
    last_velocity_update_time_ = current_time;
    previous_cmd_vel_ = smoothed_cmd_vel_;
    smoothed_cmd_vel_ = smoothed_vel;
    
    return smoothed_vel;
  }
  
  double humanoidController::calculateVelocityMagnitude(const geometry_msgs::Twist& vel)
  {
    return std::sqrt(vel.linear.x * vel.linear.x + 
                     vel.linear.y * vel.linear.y + 
                     vel.linear.z * vel.linear.z +
                     vel.angular.x * vel.angular.x + 
                     vel.angular.y * vel.angular.y + 
                     vel.angular.z * vel.angular.z);
  }
  
  // ==================== 原地踏步系统函数实现 ====================
  
  void humanoidController::startInPlaceStepping(const ros::Time& current_time)
  {
    if (!enable_in_place_stepping_)
    {
      return;
    }
    
    is_in_place_stepping_ = true;
    in_place_step_start_time_ = current_time;
    
    in_place_step_velocity_.linear.x = -0.2;
    in_place_step_velocity_.linear.y = 0.0;
    in_place_step_velocity_.linear.z = 0.0;
    in_place_step_velocity_.angular.x = 0.0;
    in_place_step_velocity_.angular.y = 0.0;
    in_place_step_velocity_.angular.z = 0.025;
  }
  
  void humanoidController::updateInPlaceStepping(const ros::Time& current_time)
  {
    if (!is_in_place_stepping_)
      return;
    
    double elapsed_time = (current_time - in_place_step_start_time_).toSec();
    
    if (elapsed_time >= in_place_step_duration_)
    {
      is_in_place_stepping_ = false;
    }
  }
  
  void humanoidController::stopInPlaceStepping()
  {
    is_in_place_stepping_ = false;
  }
  
  // ==================== 持续原地踏步系统函数实现 ====================
  
  void humanoidController::startContinuousInPlaceStepping(const ros::Time& current_time)
  {
    is_continuous_in_place_stepping_ = true;
    
    // 设置持续原地踏步的速度命令（很小的速度，让机器人原地踏步）
    continuous_in_place_step_velocity_.linear.x = -0.21;  // 很小的前进速度
    continuous_in_place_step_velocity_.linear.y = 0.0;
    continuous_in_place_step_velocity_.linear.z = 0.0;
    continuous_in_place_step_velocity_.angular.x = 0.0;
    continuous_in_place_step_velocity_.angular.y = 0.0;
    continuous_in_place_step_velocity_.angular.z = 0.025;
    
    // ROS_INFO("[ContinuousInPlaceStepping] 开始持续原地踏步");
  }
  
  void humanoidController::stopContinuousInPlaceStepping()
  {
    is_continuous_in_place_stepping_ = false;
    // ROS_INFO("[ContinuousInPlaceStepping] 停止持续原地踏步");
  }
  
  void humanoidController::updateContinuousInPlaceStepping(const ros::Time& current_time)
  {
    if (!is_continuous_in_place_stepping_)
      return;
    
    // 持续原地踏步不需要时间限制，会一直进行直到手动停止
    // ROS_DEBUG_STREAM("[ContinuousInPlaceStepping] 持续原地踏步中...");
  }



  // 手臂动作平滑插值相关函数实现
  void humanoidController::initArmInterpolation()
  {
    arm_interpolation_start_pos_.resize(armNumReal_);
    arm_interpolation_start_vel_.resize(armNumReal_);
    arm_interpolation_target_pos_.resize(armNumReal_);
    arm_interpolation_target_vel_.resize(armNumReal_);
    
    arm_interpolation_start_pos_.setZero();
    arm_interpolation_start_vel_.setZero();
    arm_interpolation_target_pos_.setZero();
    arm_interpolation_target_vel_.setZero();
    
    last_cmdStance_ = -1;
    is_arm_interpolating_ = false;
    interpolation_start_time_ = 0.0;
    
    // 添加紧急停止标志
    emergency_stop_interpolation_ = false;
    
    std::cout << "[ArmInterpolation] 手臂双向插值系统初始化完成" << std::endl;
  }

  void humanoidController::updateActionsForInterpolation(const ros::Time &time)
  {
    // 检查紧急停止标志
    if (emergency_stop_interpolation_)
    {
      std::cout << "[ArmInterpolation] 检测到紧急停止标志，停止所有插值" << std::endl;
      is_arm_interpolating_ = false;
      emergency_stop_interpolation_ = false;
      return;
    }
    
    // 只在行走模式插值时修改actions_
    if (is_arm_interpolating_)
    {
      CommandDataRL cmdData = getCommandDataRL();
      if (cmdData.cmdStance_ == 0)  // 行走模式
      {
        // 使用传入的time参数，确保时间同步
        double elapsed_time = time.toSec() - interpolation_start_time_;
        
        // 添加时间同步检查
        double current_time_diff = std::abs(time.toSec() - ros::Time::now().toSec());
        if (current_time_diff > 0.1) // 如果时间差异超过0.1秒
        {
          std::cout << "[ArmInterpolation] 警告：时间差异较大(" << current_time_diff 
                    << "秒)，可能存在时间同步问题" << std::endl;
        }
        
        // 添加安全检查：防止负值和除零错误
        if (elapsed_time < 0.0)
        {
          std::cout << "[ArmInterpolation] 警告：检测到负的经过时间(" << elapsed_time 
                    << ")，重置插值状态" << std::endl;
          is_arm_interpolating_ = false;
          return;
        }
        
        if (interpolation_duration_ <= 0.0)
        {
          std::cout << "[ArmInterpolation] 错误：插值持续时间为0或负数(" << interpolation_duration_ 
                    << ")，跳过插值" << std::endl;
          is_arm_interpolating_ = false;
          return;
        }
        
        double alpha = elapsed_time / interpolation_duration_;
        
        if (alpha >= 1.0)
        {
          alpha = 1.0;
          is_arm_interpolating_ = false;
        }
        
        // 计算插值位置
        Eigen::VectorXd interpolated_pos = getInterpolatedArmPos(alpha);
        
        // 添加NaN检查
        bool has_nan = false;
        for(int i = 0; i < interpolated_pos.size(); ++i)
        {
          if (std::isnan(interpolated_pos(i)))
          {
            has_nan = true;
            break;
          }
        }
        
        if (has_nan)
        {
          std::cout << "[ArmInterpolation] 错误：插值结果包含NaN值，停止插值" << std::endl;
          is_arm_interpolating_ = false;
          emergency_stop_interpolation_ = true; // 设置紧急停止标志
          return;
        }
        
        // 修改actions_的手臂部分
        {
          std::lock_guard<std::mutex> lock(action_mtx_);
          for(int arm_idx = 0; arm_idx < armNumReal_; ++arm_idx)
          {
            int global_idx = jointNumReal_ + waistNum_ + arm_idx;
            double target_pos = interpolated_pos(arm_idx);
            double default_pos = defalutJointPosRL_[global_idx];
            
            // 添加安全检查
            if (std::isnan(target_pos) || std::isnan(default_pos) || 
                std::isnan(actionScaleRL_) || std::isnan(actionScaleTestRL_[global_idx]))
            {
              std::cout << "[ArmInterpolation] 错误：计算action时检测到NaN值，跳过此关节" << std::endl;
              continue;
            }
            
            if (actionScaleRL_ * actionScaleTestRL_[global_idx] == 0.0)
            {
              std::cout << "[ArmInterpolation] 错误：actionScale为0，跳过此关节" << std::endl;
              continue;
            }
            
            double action_value = (target_pos - default_pos) / (actionScaleRL_ * actionScaleTestRL_[global_idx]);
            actionsRL_[global_idx] = action_value;
          }
        }
        
        ros_logger_->publishValue("/arm_interpolation/updateActions_alpha", alpha);
        ros_logger_->publishVector("/arm_interpolation/updateActions_interpolated_pos", interpolated_pos);
      }
    }
  }

  void humanoidController::startArmInterpolation(const ros::Time &time, 
                                                const Eigen::VectorXd &current_pos, 
                                                const Eigen::VectorXd &current_vel,
                                                const Eigen::VectorXd &target_pos, 
                                                const Eigen::VectorXd &target_vel)
  {
    // 添加输入参数有效性检查
    if (current_pos.size() != target_pos.size())
    {
      std::cout << "[ArmInterpolation] 错误：当前位置和目标位置维度不匹配(" 
                << current_pos.size() << " vs " << target_pos.size() << ")" << std::endl;
      return;
    }
    
    if (current_vel.size() != target_vel.size())
    {
      std::cout << "[ArmInterpolation] 错误：当前速度和目标速度维度不匹配(" 
                << current_vel.size() << " vs " << target_vel.size() << ")" << std::endl;
      return;
    }
    
    // 检查输入是否包含NaN
    bool has_nan = false;
    for(int i = 0; i < current_pos.size(); ++i)
    {
      if (std::isnan(current_pos(i)) || std::isnan(target_pos(i)))
      {
        has_nan = true;
        break;
      }
    }
    for(int i = 0; i < current_vel.size(); ++i)
    {
      if (std::isnan(current_vel(i)) || std::isnan(target_vel(i)))
      {
        has_nan = true;
        break;
      }
    }
    
    if (has_nan)
    {
      std::cout << "[ArmInterpolation] 错误：输入参数包含NaN值，跳过插值" << std::endl;
      return;
    }
    
    // 检查位置差异，如果差异很小就跳过插值
    double position_diff = (target_pos - current_pos).norm();
    const double min_interpolation_threshold = 0.05; // 0.05弧度 ≈ 3度
    
    if (position_diff < min_interpolation_threshold)
    {
      std::cout << "[ArmInterpolation] 位置差异很小(" << position_diff 
                << " rad)，跳过插值直接切换" << std::endl;
      is_arm_interpolating_ = false;
      return;
    }
    
    // 检查插值持续时间是否有效
    if (interpolation_duration_ <= 0.0)
    {
      std::cout << "[ArmInterpolation] 错误：插值持续时间无效(" << interpolation_duration_ 
                << ")，跳过插值" << std::endl;
      return;
    }
    
    arm_interpolation_start_pos_ = current_pos;
    arm_interpolation_start_vel_ = current_vel;
    arm_interpolation_target_pos_ = target_pos;
    arm_interpolation_target_vel_ = target_vel;
    
    interpolation_start_time_ = time.toSec();
    is_arm_interpolating_ = true;
    
    std::cout << "[ArmInterpolation] 开始手臂动作插值，持续时间: " << interpolation_duration_ << "秒" << std::endl;
    std::cout << "[ArmInterpolation] 起始位置: " << arm_interpolation_start_pos_.transpose() << std::endl;
    std::cout << "[ArmInterpolation] 目标位置: " << arm_interpolation_target_pos_.transpose() << std::endl;
    std::cout << "[ArmInterpolation] 总位置变化: " << position_diff << " rad" << std::endl;
  }

  Eigen::VectorXd humanoidController::getInterpolatedArmPos(double alpha)
  {
    // 添加输入参数检查
    if (std::isnan(alpha))
    {
      std::cout << "[ArmInterpolation] 错误：alpha参数为NaN，返回起始位置" << std::endl;
      return arm_interpolation_start_pos_;
    }
    
    // 限制alpha在[0,1]范围内
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    // 检查起始和目标位置是否有效
    if (arm_interpolation_start_pos_.size() != arm_interpolation_target_pos_.size())
    {
      std::cout << "[ArmInterpolation] 错误：起始位置和目标位置维度不匹配" << std::endl;
      return arm_interpolation_start_pos_;
    }
    
    // 使用五次多项式插值，提供更平滑的过渡
    // 5次函数: 6t⁵ - 15t⁴ + 10t³，在起点和终点的一阶和二阶导数都为0
    double alpha_smooth = alpha * alpha * alpha * (6.0 * alpha * alpha - 15.0 * alpha + 10.0);
    
    // 限制alpha_smooth在[0,1]范围内
    alpha_smooth = std::max(0.0, std::min(1.0, alpha_smooth));
    
    Eigen::VectorXd result = (1.0 - alpha_smooth) * arm_interpolation_start_pos_ + alpha_smooth * arm_interpolation_target_pos_;
    
    // 检查结果是否包含NaN
    for(int i = 0; i < result.size(); ++i)
    {
      if (std::isnan(result(i)))
      {
        std::cout << "[ArmInterpolation] 错误：插值结果包含NaN，返回起始位置" << std::endl;
        return arm_interpolation_start_pos_;
      }
    }
    
    return result;
  }

  Eigen::VectorXd humanoidController::getInterpolatedArmVel(double alpha)
  {
    // 添加输入参数检查
    if (std::isnan(alpha))
    {
      std::cout << "[ArmInterpolation] 错误：alpha参数为NaN，返回起始速度" << std::endl;
      return arm_interpolation_start_vel_;
    }
    
    // 限制alpha在[0,1]范围内
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    // 检查起始和目标速度是否有效
    if (arm_interpolation_start_vel_.size() != arm_interpolation_target_vel_.size())
    {
      std::cout << "[ArmInterpolation] 错误：起始速度和目标速度维度不匹配" << std::endl;
      return arm_interpolation_start_vel_;
    }
    
    // 对于速度，使用相同的五次多项式插值
    double alpha_smooth = alpha * alpha * alpha * (6.0 * alpha * alpha - 15.0 * alpha + 10.0);
    
    // 限制alpha_smooth在[0,1]范围内
    alpha_smooth = std::max(0.0, std::min(1.0, alpha_smooth));
    
    Eigen::VectorXd result = (1.0 - alpha_smooth) * arm_interpolation_start_vel_ + alpha_smooth * arm_interpolation_target_vel_;
    
    // 检查结果是否包含NaN
    for(int i = 0; i < result.size(); ++i)
    {
      if (std::isnan(result(i)))
      {
        std::cout << "[ArmInterpolation] 错误：插值速度结果包含NaN，返回起始速度" << std::endl;
        return arm_interpolation_start_vel_;
      }
    }
    
    return result;
  }

  void humanoidController::updateArmInterpolation(const ros::Time &time, kuavo_msgs::jointCmd &jointCmdMsg)
  {
    // 检查紧急停止标志
    if (emergency_stop_interpolation_)
    {
      std::cout << "[ArmInterpolation] 检测到紧急停止标志，停止所有插值" << std::endl;
      is_arm_interpolating_ = false;
      emergency_stop_interpolation_ = false;
      return;
    }
    
    CommandDataRL armCommandData = getCommandDataRL();
    
    // 检测模式切换 - 支持双向插值
    if (last_cmdStance_ != -1 && last_cmdStance_ != armCommandData.cmdStance_)
    {
      if (last_cmdStance_ == 0 && armCommandData.cmdStance_ == 1)
      {
        // 从行走模式切换到站立模式，开始插值
        Eigen::VectorXd current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        Eigen::VectorXd current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        desire_arm_q_ = defalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
        desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);
        Eigen::VectorXd target_arm_pos = desire_arm_q_;
        Eigen::VectorXd target_arm_vel = desire_arm_v_;
        
        startArmInterpolation(time, current_arm_pos, current_arm_vel, target_arm_pos, target_arm_vel);
        std::cout << "[ArmInterpolation] 模式切换: 行走 -> 站立，开始插值到VR第一个动作" << std::endl;
        std::cout << "[ArmInterpolation] 当前手臂位置: " << current_arm_pos.transpose() << std::endl;
        std::cout << "[ArmInterpolation] VR目标位置: " << target_arm_pos.transpose() << std::endl;
        std::cout << "[ArmInterpolation] 位置差异幅度: " << (target_arm_pos - current_arm_pos).norm() << " rad" << std::endl;
      }
      else if (last_cmdStance_ == 1 && armCommandData.cmdStance_ == 0)
      {
        // 从站立模式切换到行走模式，开始插值
        Eigen::VectorXd current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        Eigen::VectorXd current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        
        // 改进：使用默认手臂位置作为目标，而不是不稳定的RL输出
        Eigen::VectorXd target_arm_pos = defalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
        Eigen::VectorXd target_arm_vel = Eigen::VectorXd::Zero(armNumReal_);
        
        startArmInterpolation(time, current_arm_pos, current_arm_vel, target_arm_pos, target_arm_vel);
        std::cout << "[ArmInterpolation] 模式切换: 站立 -> 行走，开始插值到默认手臂位置" << std::endl;
        std::cout << "[ArmInterpolation] 当前手臂位置: " << current_arm_pos.transpose() << std::endl;
        std::cout << "[ArmInterpolation] 默认目标位置: " << target_arm_pos.transpose() << std::endl;
        std::cout << "[ArmInterpolation] 位置差异幅度: " << (target_arm_pos - current_arm_pos).norm() << " rad" << std::endl;
      }
    }
    
    last_cmdStance_ = armCommandData.cmdStance_;
    
    // 处理插值（适用于两种模式）
    if (is_arm_interpolating_)
    {
      double elapsed_time = time.toSec() - interpolation_start_time_;
      
      // 添加安全检查：防止负值和除零错误
      if (elapsed_time < 0.0)
      {
        std::cout << "[ArmInterpolation] 警告：检测到负的经过时间(" << elapsed_time 
                  << ")，重置插值状态" << std::endl;
        is_arm_interpolating_ = false;
        return;
      }
      
      if (interpolation_duration_ <= 0.0)
      {
        std::cout << "[ArmInterpolation] 错误：插值持续时间为0或负数(" << interpolation_duration_ 
                  << ")，跳过插值" << std::endl;
        is_arm_interpolating_ = false;
        return;
      }
      
      double alpha = elapsed_time / interpolation_duration_;
      
      if (alpha >= 1.0)
      {
        // 插值完成
        alpha = 1.0;
        is_arm_interpolating_ = false;
        if (armCommandData.cmdStance_ == 1)
        {
          std::cout << "[ArmInterpolation] 插值完成，切换到正常VR控制" << std::endl;
        }
        else
        {
          std::cout << "[ArmInterpolation] 插值完成，切换到正常RL控制" << std::endl;
        }
      }
      
      // 计算插值后的位置和速度
      Eigen::VectorXd interpolated_pos = getInterpolatedArmPos(alpha);
      Eigen::VectorXd interpolated_vel = getInterpolatedArmVel(alpha);
      
      // 添加NaN检查
      bool has_nan_pos = false;
      bool has_nan_vel = false;
      for(int i = 0; i < interpolated_pos.size(); ++i)
      {
        if (std::isnan(interpolated_pos(i)))
        {
          has_nan_pos = true;
          break;
        }
      }
      for(int i = 0; i < interpolated_vel.size(); ++i)
      {
        if (std::isnan(interpolated_vel(i)))
        {
          has_nan_vel = true;
          break;
        }
      }
      
      if (has_nan_pos || has_nan_vel)
      {
        std::cout << "[ArmInterpolation] 错误：插值结果包含NaN值，停止插值" << std::endl;
        is_arm_interpolating_ = false;
        emergency_stop_interpolation_ = true; // 设置紧急停止标志
        return;
      }
      
      // 添加手臂位置和速度的合理性检查
      const double max_arm_position = 3.14; // 约180度
      const double max_arm_velocity = 10.0; // 弧度/秒
      
      bool position_out_of_range = false;
      bool velocity_out_of_range = false;
      
      for(int i = 0; i < interpolated_pos.size(); ++i)
      {
        if (std::abs(interpolated_pos(i)) > max_arm_position)
        {
          position_out_of_range = true;
          std::cout << "[ArmInterpolation] 警告：手臂位置超出合理范围(" << interpolated_pos(i) 
                    << ")，关节索引: " << i << std::endl;
        }
      }
      
      for(int i = 0; i < interpolated_vel.size(); ++i)
      {
        if (std::abs(interpolated_vel(i)) > max_arm_velocity)
        {
          velocity_out_of_range = true;
          std::cout << "[ArmInterpolation] 警告：手臂速度超出合理范围(" << interpolated_vel(i) 
                    << ")，关节索引: " << i << std::endl;
        }
      }
      
      if (position_out_of_range || velocity_out_of_range)
      {
        std::cout << "[ArmInterpolation] 错误：检测到异常的手臂位置或速度，停止插值" << std::endl;
        is_arm_interpolating_ = false;
        emergency_stop_interpolation_ = true; // 设置紧急停止标志
        return;
      }
      
      if (armCommandData.cmdStance_ == 1)
      {
        // 站立模式：使用插值结果直接控制手臂
        arm_torque_controller_->setMeasuredState(jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_), jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_));
        vector_t arm_tau_desired = arm_torque_controller_->computeTorque(interpolated_pos, interpolated_vel, vector_t::Zero(armNumReal_));
        
        for(int i = jointNumReal_ + waistNum_; i < jointNumReal_ + waistNum_ + armNumReal_; ++i)
        {
          int arm_idx = i - jointNumReal_ - waistNum_;
          jointCmdMsg.joint_q[i] = interpolated_pos(arm_idx);
          jointCmdMsg.joint_v[i] = interpolated_vel(arm_idx);
          jointCmdMsg.joint_kp[i] = jointKpRL_[i];
          jointCmdMsg.joint_kd[i] = jointKdRL_[i];
          jointCmdMsg.tau[i] = arm_tau_desired(arm_idx);
          jointCmdMsg.tau_ratio[i] = 1;
          jointCmdMsg.tau_max[i] = torqueLimitsRL_[i];
          jointCmdMsg.control_modes[i] = JointControlModeRL_(i);
        }
      }
      else
      {
        std::cout << "[ArmInterpolation] 行走模式插值中" << std::endl;
      }
      
      // 发布插值状态
      ros_logger_->publishValue("/arm_interpolation/alpha", alpha);
      ros_logger_->publishVector("/arm_interpolation/interpolated_pos", interpolated_pos);
      ros_logger_->publishVector("/arm_interpolation/interpolated_vel", interpolated_vel);
      
      // 每隔0.1秒输出一次插值进度（避免过多日志）
      static double last_debug_time = 0.0;
      if (time.toSec() - last_debug_time > 0.1)
      {
        std::cout << "[ArmInterpolation] 插值进度: " << std::fixed << std::setprecision(1) 
                  << (alpha * 100.0) << "%, 当前位置: " << interpolated_pos.transpose() << std::endl;
        last_debug_time = time.toSec();
      }
    }
    // else
    // {
    //   // 不在插值状态，按正常逻辑处理
    //   if (armCommandData.cmdStance_ == 1)
    //   {
    //     // 站立模式：正常使用VR控制 jointNumReal_ + armNumReal_
    //     arm_torque_controller_->setMeasuredState(jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_), jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_));
    //     vector_t arm_tau_desired = arm_torque_controller_->computeTorque(desire_arm_q_, desire_arm_v_, vector_t::Zero(armNumReal_));
        
    //     for(int i = jointNumReal_ + waistNum_; i < jointNumReal_ + waistNum_ + armNumReal_; ++i)
    //     {
    //       int arm_idx = i - jointNumReal_ - waistNum_;/// 
    //       jointCmdMsg.joint_q[i] = desire_arm_q_(arm_idx);
    //       jointCmdMsg.joint_v[i] = desire_arm_v_(arm_idx);
    //       jointCmdMsg.joint_kp[i] = jointKpRL_[i];
    //       jointCmdMsg.joint_kd[i] = jointKdRL_[i];
    //       jointCmdMsg.tau[i] = arm_tau_desired(arm_idx);
    //       jointCmdMsg.tau_ratio[i] = 1;
    //       jointCmdMsg.tau_max[i] = torqueLimitsRL_[i];
    //       jointCmdMsg.control_modes[i] = JointControlModeRL_(i);
    //     }
    //   }
    //   // 行走模式下不在插值时，完全由RL控制器处理，无需特殊处理
    // }
  }

  void humanoidController::inference_thread_func()
  {
    
    ros::Rate inference_rate(inferenceFrequencyRL_);
    while (ros::ok())
    {
      if (!inference_running_)
      {
        inference_rate.sleep();
        continue;
      }
      SensorData sensors_data = getRobotSensorData();
      const auto measuredRbdStateRL_ = getRobotState();
      Eigen::VectorXd local_measure_state = measuredRbdStateRL_;
      updateObservation(local_measure_state, sensors_data);
      inference();
      inference_rate.sleep();
    }
  }
  void humanoidController::inference()
  {
    infer_request_ = compiled_model_.create_infer_request();
    const auto input_port = compiled_model_.input();
    Eigen::VectorXf float_network_input = networkInputDataRL_.cast<float>();
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), float_network_input.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.start_async();
    infer_request_.wait();
    const auto output_tensor = infer_request_.get_output_tensor();
    const size_t output_buf_length = output_tensor.get_size();
    const auto output_buf = output_tensor.data<float>();
    const size_t expected_output_length = withArmRL_ ? jointNumReal_ + waistNum_ + armNumReal_ : jointNumReal_ + waistNum_;
    if (output_buf_length != expected_output_length)
    {
      std::cout << "神经网络输出维度错误！！维度等于： " << output_buf_length << std::endl;
      return;
    }
    Eigen::VectorXd actionTargetPosRL_(output_buf_length);
    {
      std::lock_guard<std::mutex> lock(action_mtx_);
      for (int i = 0; i < output_buf_length; ++i)
      {
        actionsRL_[i] = output_buf[i];
        actionTargetPosRL_[i] = output_buf[i] * actionScaleRL_ + defalutJointPosRL_[i];
      }
      clip(actionsRL_, jointNumReal_ + waistNum_ + armNumReal_, clipActionsRL_);
      ros_logger_->publishVector("/rl_controller/actions", actionsRL_);
      ros_logger_->publishVector("/rl_controller/actionTargetPos", actionTargetPosRL_);
    }
  }
  void humanoidController::updatePhase(const CommandDataRL &CommandDataRL_)
  {
    float ratio = CommandDataRL_.cmdVelLineX_ / velocityLimitsRL_(0);
    // double nearest_int_phase = std::round(phaseRL_);
    double targetCycleTimeRL = (ratio > switch_ratioRL_) ? cycleTime_shortRL_ : cycleTimeRL_;
    if (targetCycleTimeRL != currentCycleTimeRL_)
    {
      float temp_phase = episodeLengthRL_ * dt_ / currentCycleTimeRL_;
      episodeLengthRL_ = int((targetCycleTimeRL/dt_)*(temp_phase - int(temp_phase)));
      // episodeLength_ = targetCycleTime / dt_ / 2;   //将周期置为0.5的位置
    }
    double alpha = 1;
    currentCycleTimeRL_ = (1.0 - alpha) * currentCycleTimeRL_ + alpha * targetCycleTimeRL;
    
    // 基于 episode 进度计算 phase
    // phaseRL_ = CommandDataRL_.cmdStance_ == 1 ? 0.0 : episodeLengthRL_ * dt_ / currentCycleTimeRL_;
    phaseRL_ = CommandDataRL_.cmdStance_ == 1 ? 0.0 : episodeLengthRL_ * dt_ / currentCycleTimeRL_;

    commandPhaseRL_(0) = sin(2 * M_PI * phaseRL_);
    commandPhaseRL_(1) = cos(2 * M_PI * phaseRL_);
    rl_plannedMode_ = (commandPhaseRL_(0) < 0) ? ModeNumber::SF : (commandPhaseRL_(0) > 0) ? ModeNumber::FS
                                                                                       : ModeNumber::SS;
  }

   void humanoidController::clip(Eigen::VectorXd &a, int num, double limit)
   {
     a = a.cwiseMax(-limit).cwiseMin(limit);
   }

   void humanoidController::updateObservation(const Eigen::VectorXd &state_est, const SensorData &sensor_data)
   {
     CommandDataRL CommandDataRL_;
     CommandDataRL_ = getCommandDataRL();
     updatePhase(CommandDataRL_);
     // Extract state data
    //  std::cout << "[DEBUG] 进入 updateObservation" << std::endl;
    //  std::cout << "[DEBUG] state_est: " << state_est << std::endl;
     const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
     const Eigen::Vector3d baseAngVel(state_est(6 + jointNumReal_ + waistNum_ + armNumReal_ ),
                                      state_est(6 + jointNumReal_ + waistNum_ + armNumReal_ + 1),
                                      state_est(6 + jointNumReal_ + waistNum_ + armNumReal_ + 2));
     const Eigen::Vector3d baseLineVel = state_est.segment(9 + jointNumReal_ + waistNum_ + armNumReal_, 3);
     const Eigen::Vector3d basePos = state_est.segment(3, 3);
     // Extract and process sensor data
     Eigen::VectorXd jointPos = sensor_data.jointPos_ - defalutJointPosRL_;
     const Eigen::VectorXd &jointVel = sensor_data.jointVel_;
     Eigen::VectorXd jointTorque = sensor_data.jointCurrent_;
     const Eigen::Vector3d &bodyAngVel = sensor_data.angularVel_;
     const Eigen::Vector3d &bodyLineAcc = sensor_data.linearAccel_;
     const Eigen::Vector3d &bodyLineFreeAcc = sensor_data.freeLinearAccel_;
     // Normalize joint torques
     for (int i = 0; i < jointNumReal_ + waistNum_ + armNumReal_; ++i)
     {
       jointTorque[i] /= torqueLimitsRL_[i];
     }
     // Transform base linear velocity
     const Eigen::Matrix3d R = sensor_data.quat_offset_.matrix();
     const Eigen::Vector3d bodyLineVel = R.transpose() * baseLineVel;
     // Get local action
     Eigen::VectorXd local_action;
     {
       std::lock_guard<std::mutex> lock(action_mtx_);
       local_action = actionsRL_;
       if(!is_rl_controller_)
       {
         local_action.setZero();
       }
     }
     if (!withArmRL_)
     {
       local_action.tail(armNumReal_).setZero();
     }
    // Normalize command
    CommandDataRL_.scale();
    Eigen::VectorXd tempCommand_ = CommandDataRL_.getCommandRL();
    
    // 发布tempCommand数据
    ros_logger_->publishVector("/rl_controller/tempCommand", tempCommand_);
    
    // Populate singleInputDataMap_
     const std::map<std::string, Eigen::VectorXd> singleInputDataMap_ = {
         {"baseEuler", baseEuler},
         {"baseAngVel", baseAngVel},
         {"baseLineVel", baseLineVel},
         {"basePos", basePos},
         {"jointPos", jointPos},
         {"jointVel", jointVel},
         {"jointTorque", jointTorque},
         {"bodyAngVel", bodyAngVel},
         {"bodyLineAcc", bodyLineAcc},
         {"bodyLineFreeAcc", bodyLineFreeAcc},
         {"bodyLineVel", bodyLineVel},
         {"commandPhase", commandPhaseRL_},
         {"command", tempCommand_},
         {"action", local_action}};
     // Fill singleInputData
     int index = 0;
     for (const auto &key : singleInputDataRLKeys)
     {
       const auto &value = singleInputDataRLID_[key];
       singleInputDataRL_.segment(index, value[1]) = singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2];
       index += value[1];
       ros_logger_->publishVector("/rl_controller/InputData/" + key, singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2]);
     }
     // Clip and update input_deque
     clip(singleInputDataRL_, numSingleObsRL_, clipObservationsRL_);
     input_deque.push_back(singleInputDataRL_);
     input_deque.pop_front();
     // Update networkInputData_
     for (int i = 0; i < frameStackRL_; ++i)
     {
       networkInputDataRL_.segment(i * numSingleObsRL_, numSingleObsRL_) = input_deque[i];
     }
     // Publish data
    //  ros_logger_->publishVector("/rl_controller/singleInputData", singleInputDataRL_);
   }
  
  
  Eigen::VectorXd humanoidController::updateRLcmd(const vector_t &state)
  {
    // 在RL控制计算前检查并更新手臂插值状态
    updateActionsForInterpolation(ros::Time::now());
    
    Eigen::VectorXd actuation(jointNumReal_ + waistNum_ + armNumReal_);
    Eigen::VectorXd cmd_out(jointNumReal_ + waistNum_ + armNumReal_);
    Eigen::VectorXd cmd(jointNumReal_ + waistNum_ + armNumReal_);
    Eigen::VectorXd cmd_filter(jointNumReal_ + waistNum_ + armNumReal_);
    Eigen::VectorXd torque(jointNumReal_ + waistNum_ + armNumReal_);
    // std::cout << "state: " << state.size() << std::endl;
    const Eigen::VectorXd jointPos_ = state.segment(6, jointNumReal_ + waistNum_ + armNumReal_);
    const Eigen::VectorXd jointVel_ = state.tail(jointNumReal_ + waistNum_ + armNumReal_);
    Eigen::VectorXd motorPos_ = jointPos_;
    Eigen::VectorXd motorVel_ = jointVel_;
    Eigen::VectorXd local_action;
    {
      std::lock_guard<std::mutex> lock(action_mtx_);
      local_action = actionsRL_;
    }
    if (!withArmRL_)
    {
      local_action.tail(armNumReal_+ waistNum_).setZero();
    }
    motorPos_.head(jointNumReal_) = ankleSolver.joint_to_motor_position(jointPos_.head(jointNumReal_));
    motorVel_.head(jointNumReal_) = ankleSolver.joint_to_motor_velocity(jointPos_.head(jointNumReal_), motorPos_, jointVel_.head(jointNumReal_));
    Eigen::VectorXd jointTor_ = -(jointKdRL_.cwiseProduct(motorVel_));
    jointTor_.head(jointNumReal_) = ankleSolver.motor_to_joint_torque(jointPos_.head(jointNumReal_), motorPos_, jointTor_.head(jointNumReal_));
    for (int i = 0; i < jointNumReal_ + waistNum_ + armNumReal_; i++)
    {
      jointTor_(i) = jointTor_(i) + jointKpRL_(i) * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
    }
    if (is_real_)
    {
      for (int i = 0; i < jointNumReal_ + waistNum_ + armNumReal_; i++)
      {
        if (JointControlModeRL_(i) == 0)
        {
          if (JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpRL_[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel_[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] + defalutJointPosRL_[i]);
            torque[i] = jointKpRL_[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel_[i];
          }
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
          torque[i] = jointTor_[i];
        }
      }
    }
    else
    {
      for (int i = 0; i < jointNumReal_ + waistNum_ + armNumReal_; i++)
      {
        if (JointControlModeRL_(i) == 0)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel_[i];
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor_[i];
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
        torque[i] = cmd[i];
      }
    }
    cmd_filter = jointCmdFilterRL_.update(cmd);
    cmd_out = cmd_filter.cwiseProduct(jointCmdFilterStateRL_) + cmd.cwiseProduct(Eigen::VectorXd::Ones(jointNumReal_ + waistNum_ + armNumReal_) - jointCmdFilterStateRL_);
    actuation = cmd_out;

    // actuation = cmd;// kuavo-RL仓库五代去掉了滤波
    episodeLengthRL_++;
    ros_logger_->publishVector("/rl_controller/cmd_out", cmd_out);
    ros_logger_->publishVector("/rl_controller/torque", torque);
    ros_logger_->publishVector("/rl_controller/cmd_filter", cmd_filter);
    ros_logger_->publishVector("/rl_controller/cmd", cmd);
    return actuation;
  }

  Eigen::VectorXd humanoidController::computeInterpolation(const InterpolationRequest& req, double alpha)
  {
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    switch (req.interpolation_type)
    {
      case 0: // 线性插值
      {
        return (1.0 - alpha) * req.start_pos + alpha * req.target_pos;
      }
      
      case 1: // 五次多项式插值（默认）
      {
        // 5次函数: 6t⁵ - 15t⁴ + 10t³，在起点和终点的一阶和二阶导数都为0
        double alpha_smooth = alpha * alpha * alpha * (6.0 * alpha * alpha - 15.0 * alpha + 10.0);
        return (1.0 - alpha_smooth) * req.start_pos + alpha_smooth * req.target_pos;
      }
      
      case 2: // 三次样条插值（考虑速度）
      {
        // 三次Hermite插值
        double t = alpha;
        double t2 = t * t;
        double t3 = t2 * t;
        
        double h00 = 2*t3 - 3*t2 + 1;
        double h10 = t3 - 2*t2 + t;
        double h01 = -2*t3 + 3*t2;
        double h11 = t3 - t2;
        
        return h00 * req.start_pos + h10 * req.duration * req.start_vel + 
               h01 * req.target_pos + h11 * req.duration * req.target_vel;
      }
      
      default:
        std::cerr << "[GeneralInterpolation] 警告：未知插值类型 " << req.interpolation_type 
                  << "，使用五次多项式插值" << std::endl;
        double alpha_smooth = alpha * alpha * alpha * (6.0 * alpha * alpha - 15.0 * alpha + 10.0);
        return (1.0 - alpha_smooth) * req.start_pos + alpha_smooth * req.target_pos;
    }
  }
  
  Eigen::VectorXd humanoidController::computeInterpolationVelocity(const InterpolationRequest& req, double alpha)
  {
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    switch (req.interpolation_type)
    {
      case 0: // 线性插值
      {
        return (req.target_pos - req.start_pos) / req.duration;
      }
      
      case 1: // 五次多项式插值
      {
        // 五次多项式的导数: 30t⁴ - 60t³ + 30t²
        double t = alpha;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double dt_dalpha = (30.0 * t4 - 60.0 * t3 + 30.0 * t2) / req.duration;
        return dt_dalpha * (req.target_pos - req.start_pos);
      }
      
      case 2: // 三次样条插值
      {
        double t = alpha;
        double t2 = t * t;
        
        double dh00_dt = (6*t2 - 6*t) / req.duration;
        double dh10_dt = (3*t2 - 4*t + 1) / req.duration;
        double dh01_dt = (-6*t2 + 6*t) / req.duration;
        double dh11_dt = (3*t2 - 2*t) / req.duration;
        
        return dh00_dt * req.start_pos + dh10_dt * req.duration * req.start_vel + 
               dh01_dt * req.target_pos + dh11_dt * req.duration * req.target_vel;
      }
      
      default:
        double t = alpha;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double dt_dalpha = (30.0 * t4 - 60.0 * t3 + 30.0 * t2) / req.duration;
        return dt_dalpha * (req.target_pos - req.start_pos);
    }
  }

  // ==================== MPC-RL插值系统实现 ====================
  void humanoidController::startMPCRLInterpolation(double current_time, const vector6_t& target_torso_pose, const vector_t& target_arm_pos)
  {
    // 获取当前躯干姿态（xyz+rpy）
    vector6_t current_torso_pose = vector6_t::Zero();
    current_torso_pose.segment<3>(0) = currentObservation_.state.segment<3>(6); // 位置 xyz
    // 假定 currentObservation_ 中姿态的 rpy 来源：roll=default_state_(10), yaw=stanceState_mrt_(9)，pitch 先保持 0 或从状态中获取
    // 为保持行为一致，沿用原 update 发布中的来源：
    current_torso_pose(3) = 0.0;                 // roll
    current_torso_pose(4) = currentObservation_.state(10);  // pitch
    current_torso_pose(5) = currentObservation_.state(9); // yaw
    torso_interpolation_result_ = current_torso_pose;// rpy->ypr
    torso_interpolation_result_(3) = currentObservation_.state(9);
    torso_interpolation_result_(5) = 0.0;

    // 获取当前手臂位置
    vector_t current_arm_pos = vector_t::Zero(armNumReal_);
    current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
    arm_interpolation_result_ = current_arm_pos;

    // 计算躯干位移距离（仅xyz用于限速）
    vector3_t target_position_torso = target_torso_pose.segment<3>(0);
    vector3_t current_torso_pos = current_torso_pose.segment<3>(0);
    double torso_distance = (target_position_torso - current_torso_pos).norm();

    // 计算手臂位移距离
    double arm_distance = 0.0;
    if (target_arm_pos.size() == current_arm_pos.size())
    {
      arm_distance = (target_arm_pos - current_arm_pos).norm();
    }
    else
    {
      std::cout << "[MPCRLInterpolation] 错误：手臂位置维度不匹配(" 
                << target_arm_pos.size() << " vs " << current_arm_pos.size() << ")" << std::endl;
    }

    // 设置插值参数
    is_torso_interpolation_active_ = true;
    torso_interpolation_start_pose_ = current_torso_pose;
    torso_interpolation_target_pose_ = target_torso_pose;
    torso_interpolation_start_time_ = current_time;
    
    // 计算总期望插值时间（基于躯干和手臂的最大距离和最大速度）
    torso_interpolation_duration_ = torso_distance / torso_interpolation_max_velocity_;
    double arm_interpolation_duration = arm_distance / arm_interpolation_max_velocity_;
    torso_interpolation_duration_ = std::max(arm_interpolation_duration, torso_interpolation_duration_);

    // 初始化手臂插值参数
    arm_interpolation_start_pos_ = current_arm_pos;
    arm_interpolation_target_pos_ = target_arm_pos;

    // 初始化插值状态变量
    last_interpolated_pose_ = current_torso_pose;
    last_interpolation_time_ = current_time;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Starting MPC-RL interpolation:" << std::endl;
    std::cout << "  Torso from [" << current_torso_pose.transpose() 
              << "] to [" << target_torso_pose.transpose() << "] (distance: " << torso_distance << "m)" << std::endl;
    std::cout << "  Arm from [" << current_arm_pos.transpose() 
              << "] to [" << target_arm_pos.transpose() << "] (distance: " << arm_distance << "rad)" << std::endl;
    std::cout << "  Max velocity: " << torso_interpolation_max_velocity_ 
              << " m/s, duration: " << torso_interpolation_duration_ << "s" << std::endl;
  }

  void humanoidController::updateMPCRLInterpolation(double current_time)
  {
    if (!is_torso_interpolation_active_)
      return;
    
    double dt = current_time - last_interpolation_time_;
    if (dt <= 0.001) // 避免除零和过于频繁的更新
      return;
    
    // 计算目标方向向量（xyz）
    // auto mpc_target_pose = mpc_current_target_trajectories_.getDesiredState(current_time).segment<6>(6);
    
    // 基于6D位姿计算距离和方向
    vector6_t direction = torso_interpolation_target_pose_ - currentObservation_.state.segment<6>(6);
    vector3_t pos_direction = direction.segment<3>(0);
    double distance_to_target = std::abs(pos_direction[2]);
    
    // 如果已经到达目标位置
    if (distance_to_target < 0.003 || current_time - torso_interpolation_start_time_ > torso_interpolation_duration_*2)
    {
      is_torso_interpolation_active_ = false;
      is_arm_interpolating_ = false;
      std::cout << "MPC-RL interpolation completed" << std::endl;
      return;
    }
    
    // 计算基于时间的插值进度
    double elapsed_time = current_time - torso_interpolation_start_time_;
    double alpha = std::min(1.0, elapsed_time / torso_interpolation_duration_);
    
    // 使用线性插值计算当前躯干位姿
    vector6_t interpolated_pose = torso_interpolation_start_pose_ + alpha * (torso_interpolation_target_pose_ - torso_interpolation_start_pose_);
    
    // 计算手臂插值
    arm_interpolation_result_ = arm_interpolation_start_pos_ + alpha * (arm_interpolation_target_pos_ - arm_interpolation_start_pos_);
    // 更新位姿和时间
    last_interpolated_pose_ = interpolated_pose;
    last_interpolation_time_ = current_time;
    torso_interpolation_result_ = interpolated_pose;
    torso_interpolation_result_(3) = stanceState_mrt_(9);
    torso_interpolation_result_(5) = 0.0;
    
    // 发布/cmd_pose_world话题 (geometry_msgs::Twist)
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = interpolated_pose(0);  // x位置
    twist_msg.linear.y = interpolated_pose(1);  // y位置
    twist_msg.linear.z = interpolated_pose(2) - default_state_[8];  // z位置与基准高度的差值
    // 使用插值后的rpy
    twist_msg.angular.x = torso_interpolation_target_pose_(3); // roll
    twist_msg.angular.y = torso_interpolation_target_pose_(4); // pitch
    twist_msg.angular.z = torso_interpolation_target_pose_(5); // yaw
    
    cmdPoseWorldPublisher_.publish(twist_msg);
    
    // 每0.1秒输出一次进度信息
    static double last_debug_time = current_time;
    if (current_time - last_debug_time > 0.05)
    {
      double elapsed_time = current_time - torso_interpolation_start_time_;
      //double progress = (elapsed_time / torso_interpolation_duration_) * 100.0;
      double z_diff = interpolated_pose(2) - default_state_[8];
      std::cout << "MPC-RL: TO "<< (is_rl_controller_ ? "RL" : "MPC") << ", elapsed_time: " 
                << elapsed_time << "s, expected duration: " << torso_interpolation_duration_ 
                << "s, z_diff: " << z_diff << "m" << std::endl;
      std::cout << "torso_interpolation_target_pose_: " << torso_interpolation_target_pose_.transpose() << std::endl;
      std::cout << "arm_interpolated_pos: " << arm_interpolation_result_.transpose() << std::endl;
      last_debug_time = current_time;
    }
  }

  void humanoidController::stopTorsoInterpolation()
  {
    if (is_torso_interpolation_active_)
    {
      is_torso_interpolation_active_ = false;
      std::cout << "Torso interpolation stopped" << std::endl;
    }
  }

  double humanoidController::getTorsoInterpolationProgress() const
  {
    if (!is_torso_interpolation_active_)
      return 0.0;
    
    // 基于时间计算进度
    double elapsed_time = last_interpolation_time_ - torso_interpolation_start_time_;
    double progress = (elapsed_time / torso_interpolation_duration_) * 100.0;
    
    return std::max(0.0, std::min(100.0, progress));
  }

 } // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)

