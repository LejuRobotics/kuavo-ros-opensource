#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "std_srvs/SetBool.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <fstream>
#include <ros/this_node.h>
#include <cstdlib>
#include <filesystem>
#include <boost/filesystem.hpp>
#include <sstream>

#include "humanoid_controllers/humanoidController.h"
#include "humanoid_controllers/rl/armController.h"
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

    if(is_roban_)
    {
      if (ros::param::has("/pull_up_force_threshold_roban"))
      {
        ros::param::get("/pull_up_force_threshold_roban", pull_up_force_threshold_);
        std::cout << "pull_up_force_threshold_roban: " << pull_up_force_threshold_ << std::endl;
      }
      else
      {
        ROS_WARN_STREAM("Param '/pull_up_force_threshold_roban' not found, Using default value");
      }
    }

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

    // 存储并初始化 WBC 控制频率
    wbc_frequency_ = controlFrequency;
    wbc_rate_ = std::make_unique<ros::Rate>(wbc_frequency_);
    ROS_INFO_STREAM("[humanoidController] WBC rate initialized at " << wbc_frequency_ << " Hz");

    // 获取传感器频率参数
    controllerNh_.param("/sensor_frequency", sensor_frequency_, 1000.0);
    sensor_dt_ = 1.0 / sensor_frequency_;
    ROS_INFO_STREAM("[humanoidController] Sensor frequency: " << sensor_frequency_ << " Hz, sensor_dt: " << sensor_dt_ << " s");
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
    ros::param::set("/is_real", is_real_);
    ros::param::set("/is_roban", is_roban_);
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);

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

    if (controllerNh_.hasParam("/stand_up_protect"))
    {
      controllerNh_.getParam("/stand_up_protect", stand_up_protect_);
      std::cout << "get stand up protect param: " << stand_up_protect_ << std::endl;
    }
    if (controllerNh_.hasParam("/init_fall_down_state"))
    {
      controllerNh_.getParam("/init_fall_down_state", init_fall_down_state_);
      if (init_fall_down_state_)
      {
        fall_down_state_==FallStandState::FALL_DOWN;
      }
    }

    // 检测is_rl_start参数，如果为true则绕过MPC控制器，直接使用RL控制器
    controllerNh_.param<bool>("/is_rl_start", is_rl_start_, false);

    
    // trajectory_publisher_ = new TrajectoryPublisher(controller_nh, 0.001);

    wheel_arm_robot_ = drake_interface_->getKuavoSettings().running_settings.only_half_up_body;
    
    // size_t buffer_size = (is_play_back_mode_) ? 20 + waistNum_ : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, sensor_dt_);
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

    torso_position_interpolator_ptr_ = std::make_shared<FloatInterpolation>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo());

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
    jointTorqueCmdRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    jointTorqueCmdRL_.setZero();
    initialStateRL_.resize(12 + jointNumReal_ + armNumReal_ + waistNum_);
    currentDefalutJointPosRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    defalutArmPosMPC_.resize(armNumReal_);
    JointControlModeRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    JointControlModeRL_.setZero();
    JointPDModeRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    JointPDModeRL_.setZero();
    jointKpRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    jointKdRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    torqueLimitsRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    actionScaleTestRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    jointCmdFilterStateRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    sensor_data_headRL_.resize_joint(headNum_);
    head_kpRL_.resize(headNum_);
    head_kdRL_.resize(headNum_);
    output_tauRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
    output_tauRL_.setZero();
    jointPosRL_ = vector_t::Zero(jointNumReal_ + armNumReal_ + waistNum_);
    jointVelRL_ = vector_t::Zero(jointNumReal_ + armNumReal_ + waistNum_);
    jointAccRL_ = vector_t::Zero(jointNumReal_ + armNumReal_ + waistNum_);
    default_state_.resize(12+actuatedDofNumReal_);
    default_state_.setZero();
    arm_mode_sync_time_ = ros::Time::now().toSec();
    
    // 检查RL参数文件是否存在，只有文件存在时才启用RL功能
    // std::ifstream rlParamFileCheck(rlParamFile);
    // if (rlParamFileCheck.good()) {
    //   std::cout << "RL parameter file found: " << rlParamFile << ", enabling RL controller." << std::endl;
    //   rl_available_ = true;
    //   loadRLSettings(rlParamFile, verbose, dt_);
    //   // 初始化RL步态接收器
    //   rl_gait_receiver_ = std::make_unique<RlGaitReceiver>(controllerNh_, &initialCommandDataRL_);
    // } else {
    //   std::cout << "RL parameter file not found: " << rlParamFile << ", RL controller disabled." << std::endl;
    //   rl_available_ = false;
    // }
    // rlParamFileCheck.close();

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
    // 同步脚踝解算类型到 ROS 参数，供倒地起身等 RL 控制器使用
    ros::param::set("/ankle_solver_type", static_cast<int>(ankleSolverType));
    if (!init_fall_down_state_) // 初始倒地时不在这里设置初始状态
    {
      ros::param::set("robot_init_state_param", robot_init_state_param);
    }
  
    ros::param::set("/humanoid/init_q", robot_init_state_param);

    auto initial_state_ =  drake_interface_->getInitialState();// 里面不包含手臂和腰部
    auto squat_initial_state_ =  drake_interface_->getSquatInitialState();
    default_state_.head(12+12) = initial_state_.head(12+12);
    std::cout << "controller initial_state_:" << initial_state_.transpose() << std::endl;
    std::cout << "controller squat_initial_state_:" << squat_initial_state_.transpose() << std::endl;
    std::vector<double> initial_state_vector(initial_state_.data(), initial_state_.data() + initial_state_.size());
    std::vector<double> squat_initial_state_vector(squat_initial_state_.data(), squat_initial_state_.data() + squat_initial_state_.size());
    std::vector<double> default_joint_pos_vector(defalutJointPos_.data(), defalutJointPos_.data() + defalutJointPos_.size());
    controllerNh_.setParam("/initial_state", initial_state_vector);
    if (!init_fall_down_state_) // 初始倒地时不在这里设置实物初始状态
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
    std::vector<double> stand_joint_state_vector(defalutArmPosMPC_.data(), defalutArmPosMPC_.data() + defalutArmPosMPC_.size());
    controllerNh_.setParam("/standJointState", stand_joint_state_vector);
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
    joint_pos_ = vector_t::Zero(info.actuatedDofNum);
    joint_pos_.setZero();
    joint_pos_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointPosWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    // jointPosWBC_.setZero();
    jointPosWBC_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointVelWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    jointAccWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);
    jointCurrentWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ + waistNum_);

    joint_vel_ = vector_t::Zero(info.actuatedDofNum);
    joint_acc_ = vector_t::Zero(info.actuatedDofNum);
    joint_torque_ = vector_t::Zero(info.actuatedDofNum);
    quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    quat_init = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    arm_joint_pos_cmd_prev_ = vector_t::Zero(armNumReal_);
    arm_joint_pos_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_pos_filter_cutoff_freq));
    arm_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_vel_filter_cutoff_freq));
    mrt_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(info.actuatedDofNum-armNum_, mrt_joint_vel_filter_cutoff_freq));

    // 使用传感器频率对应的 dt 初始化滤波器，确保滤波器采样周期与实际回调频率匹配
    acc_filter_.setParams(sensor_dt_, acc_filter_params);
    // free_acc_filter_.setParams(sensor_dt_, acc_filter_params);
    gyro_filter_.setParams(sensor_dt_, gyro_filter_params);
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
      changeRuiwoMotorParamSrv_ = controllerNh_.advertiseService("humanoid_controller/change_ruiwo_motor_param",
        &humanoidController::changeRuiwoMotorParamCallback, this);
      // 初始化 RL 控制器管理系统
      controller_manager_ = std::make_unique<RLControllerManager>();
      
      // 注册倒地状态回调函数
      controller_manager_->registerFallDownStateCallback([this](int state) {
        fall_down_state_ = static_cast<FallStandState>(state);
        ROS_INFO("[HumanoidController] fall_down_state_ set to %d via callback", fall_down_state_);
      });
      
      // 注册躯干稳定性状态回调函数（从状态估计器获取）
      controller_manager_->registerTorsoStabilityCallback([this]() -> bool {
        if (stateEstimate_)
        {
          return stateEstimate_->isTorsoVelocityStable();
        }
        else
        {
          // 如果状态估计器未初始化，返回true（允许切换）
          return true;
        }
      });
      
      // 初始化 ROS 服务（由 RLControllerManager 管理）
      controller_manager_->initializeRosServices(controllerNh_);
      
      
      // 只有在 RL 可用时才初始化控制器（需要配置文件）
      {
        // 从 rlParamFile 路径推断版本配置目录
        boost::filesystem::path rl_param_path(rlParamFile);
        boost::filesystem::path version_config_dir = rl_param_path.parent_path().parent_path();
        std::string controller_list_config = version_config_dir.string() + "/rl_controllers.yaml";
        
        // 尝试从配置文件加载控制器列表
        if (boost::filesystem::exists(controller_list_config))
        {
          ROS_INFO("[HumanoidController] Loading RL controllers from config file: %s", controller_list_config.c_str());
          if (controller_manager_->loadControllersFromConfig(controller_list_config, version_config_dir.string(), 
                                                           controllerNh_, ros_logger_))
          {
            // 从 RLControllerManager 获取 WALK_CONTROLLER 列表（包括 BASE）
            available_controllers_ = controller_manager_->getWalkControllerList();
            
            // is_rl_start模式：从所有已加载控制器中找到第一个非MPC、非FALL_STAND的控制器
            // 不限于 walk_controllers_ 列表，dance等控制器也可以作为 rl_start 目标
            if (is_rl_start_)
            {
              std::string first_rl_controller = "";
              auto all_controller_names = controller_manager_->getControllerNames();
              for (const auto& name : all_controller_names)
              {
                if (name != "mpc" && 
                    controller_manager_->getControllerTypeByName(name) != RLControllerType::FALL_STAND_CONTROLLER)
                {
                  first_rl_controller = name;
                  break;
                }
              }
              
              if (!first_rl_controller.empty())
              {
                current_controller_ = first_rl_controller;
                // 如果在 walk 列表中则设置索引，否则设为 -1
                auto it = std::find(available_controllers_.begin(), available_controllers_.end(), first_rl_controller);
                current_controller_index_ = (it != available_controllers_.end()) 
                    ? static_cast<int>(it - available_controllers_.begin()) : -1;
                
                // 获取RL控制器的默认姿态，供preUpdate起立使用
                auto* target_ptr = controller_manager_->getControllerByName(first_rl_controller);
                if (target_ptr)
                {
                  currentDefalutJointPosRL_ = target_ptr->getDefaultJointPos();
                  defaultBaseHeightControl_ = target_ptr->getDefaultBaseHeightControl();
                  desire_arm_q_ = currentDefalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
                  desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);
                }
                ROS_INFO("[HumanoidController] is_rl_start=true, target RL controller: %s", first_rl_controller.c_str());
              }
              else
              {
                ROS_WARN("[HumanoidController] is_rl_start=true but no suitable RL controller found, falling back to MPC");
                is_rl_start_ = false;
              }
            }
            
            // 默认使用MPC（is_rl_start失败或未设置时）
            if (!is_rl_start_)
            {
              current_controller_ = "mpc";
              current_controller_index_ = 0;
            }
            
            ROS_INFO("[HumanoidController] Successfully loaded controllers from config, total %zu controllers", 
                     available_controllers_.size());
            
            
          }
          else
          {
            ROS_WARN("[HumanoidController] Failed to load controllers from config file");
          }
          has_fall_stand_controller_ = controller_manager_->hasController(RLControllerType::FALL_STAND_CONTROLLER);
        }
        else
        {
          ROS_WARN("[HumanoidController] RL not available, RL controllers will not be initialized");
        }

          // 如果 init_fall_down_state_=true，初始切换到倒地起身控制器
        if (init_fall_down_state_)
        {
          if (has_fall_stand_controller_)
          {
            std::cout << "[humanoid Controller]init_fall_down_state_, switch to fall down controller" << std::endl;
            controller_manager_->switchController(RLControllerType::FALL_STAND_CONTROLLER);
          }
        }
      }
      
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
          
          // 模式切换时重置拉起保护滤波器（避免模式切换时的接触力变化导致误触发）
          if (stateEstimate_)
          {
            stateEstimate_->resetPullUpFilter();
            ROS_INFO("[HumanoidController] Reset pullup filter due to mode switch (from %d to %d)", 
                     static_cast<int>(mpcArmControlMode_));
          }
          
          // 检查模式是否已同步，如果同步则记录时间
          arm_mode_sync_time_ = ros::Time::now().toSec();
          
        }
        if (msg->data[1] != mpcArmControlMode_desired_)
        {
          mpcArmControlMode_desired_ = static_cast<ArmControlMode>(msg->data[1]);
          std::cout << "[controller] mpc arm control mode desired changed to: " << mpcArmControlMode_desired_ << std::endl;
          
          // 如果当前是 RL 控制器，设置 arm_controller_ 的模式
          if (controller_manager_)
          {
            auto* current_controller = controller_manager_->getCurrentController();
            if (current_controller)
            {
              auto* arm_controller = current_controller->getArmController();
              if (arm_controller)
              {
                // 获取当前关节位置和速度（用于模式切换）
                // 优先从 jointPosWBC_ 和 jointVelWBC_ 获取（这些数据在 update 中会更新）
                Eigen::VectorXd joint_pos, joint_vel;
                int total_joints = jointNumReal_ + waistNum_ + armNumReal_;
                
                // 尝试从 jointPosWBC_ 和 jointVelWBC_ 获取（如果已初始化）
                if (jointPosWBC_.size() >= total_joints && jointVelWBC_.size() >= total_joints)
                {
                  joint_pos = jointPosWBC_.head(total_joints);
                  joint_vel = jointVelWBC_.head(total_joints);
                }
                // 否则从 measuredRbdStateReal_ 获取
                else if (measuredRbdStateReal_.size() >= total_joints * 2)
                {
                  joint_pos = measuredRbdStateReal_.head(total_joints);
                  joint_vel = measuredRbdStateReal_.segment(total_joints, total_joints);
                }
                // 如果都没有，使用零向量（不推荐，但可以避免崩溃）
                else
                {
                  joint_pos = Eigen::VectorXd::Zero(total_joints);
                  joint_vel = Eigen::VectorXd::Zero(total_joints);
                  ROS_WARN("[controller] Cannot get joint pos/vel, using zero vectors for mode change");
                }
                
                // 转换 ArmControlMode 到 ArmController 模式
                // ArmControlMode: KEEP=0, AUTO_SWING=1, EXTERN_CONTROL=2
                // ArmController: 0=固定到当前动作, 1=自动摆手, 2=外部控制
                int arm_controller_mode = static_cast<int>(mpcArmControlMode_desired_);
                arm_controller->changeMode(arm_controller_mode);
                ROS_INFO("[controller] Set arm_controller mode to %d (from mpcArmControlMode_desired_=%d)", 
                         arm_controller_mode, static_cast<int>(mpcArmControlMode_desired_));
              }
            }
          }
        }
      });
      
      armEefWbcPosePublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>("/humanoid_controller/wbc_arm_eef_pose", 10, true);
      // dexhand state
      dexhand_state_sub_ = controllerNh_.subscribe("/dexhand/state", 10, &humanoidController::dexhandStateCallback, this);

      standUpCompletePub_ = controllerNh_.advertise<std_msgs::Int8>("/bot_stand_up_complete", 10);
      
      enable_mpc_sub_ = controllerNh_.subscribe("/enable_mpc_flag", 10, &humanoidController::getEnableMpcFlagCallback, this);
      enable_wbc_sub_ = controllerNh_.subscribe("/enable_wbc_flag", 10, &humanoidController::getEnableWbcFlagCallback, this);

      // State estimation
      setupStateEstimate(taskFile, verbose, referenceFile);
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

      taskFile_switchParams_ = taskFile;
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
    actionsRL_.resize(jointNumReal_ + armNumReal_ + waistNum_);
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
      // arm_torque_controller_.reset(new ArmTorqueController(urdf_path, jointKpRL_.segment(jointNumReal_, armNumReal_), jointKdRL_.segment(jointNumReal_, armNumReal_))); // 使用智能指针初始化，只传入手臂关节参数
    }
    
    desire_arm_q_ = currentDefalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
    desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);

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


  void humanoidController::replaceDefaultEcMotorPdoGait(kuavo_msgs::jointCmd& jointCmdMsg)
  {
    // 对于 control_modes == 2 的电机：
    //   EC_MASTER 电机：使用 running_settings.joint_kp/kd（来自 kuavo.json joint_kp/kd）
    //   RUIWO 电机：使用 running_settings.ruiwo_kp/kd（来自 kuavo.json ruiwo_kp/kd）
    // 注意：ec_master_count/ruiwo_count 对应各自驱动器数组中的索引
    const auto &hardware_settings = kuavo_settings_.hardware_settings;
    const auto &running_settings = kuavo_settings_.running_settings;
    const int total_joints = jointNumReal_ + waistNum_ + armNumReal_;

    // 替换 EC_MASTER 电机 kp/kd
    if (!running_settings.joint_kp.empty() && 
        !running_settings.joint_kd.empty() &&
        running_settings.joint_kp.size() == running_settings.joint_kd.size())
    {
      const int ec_master_size = static_cast<int>(running_settings.joint_kp.size());
      int ec_master_count = 0;
      
      for (int i = 0; i < total_joints && i < static_cast<int>(jointCmdMsg.control_modes.size()); ++i)
      {
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

  bool humanoidController::changeRuiwoMotorParamCallback(kuavo_msgs::ExecuteArmActionRequest &req, kuavo_msgs::ExecuteArmActionResponse &res)
  {
    const std::string &param_name = req.action_name;
    auto robot_config = drake_interface_->getRobotConfig();
    if (robot_config == nullptr) {
      res.message = "Robot config is not initialized";
      res.success = false;
      ROS_ERROR("[HumanoidController] changeRuiwoMotorParamCallback: robot config is not initialized");
      return true;
    }

    auto nested_obj = (*robot_config)[param_name];
    if (!nested_obj.contains("ruiwo_kp") || !nested_obj.contains("ruiwo_kd")) {
      res.message = "Nested object '" + param_name + "' does not contain ruiwo_kp or ruiwo_kd";
      res.success = false;
      ROS_ERROR("[HumanoidController] changeRuiwoMotorParamCallback: %s", res.message.c_str());
      return true;
    }

    std::vector<int32_t> kp_values = nested_obj["ruiwo_kp"].get<std::vector<int32_t>>();
    std::vector<int32_t> kd_values = nested_obj["ruiwo_kd"].get<std::vector<int32_t>>();

    if (kp_values.empty() || kd_values.empty()) {
      res.message = "Failed to load ruiwo_kp or ruiwo_kd from '" + param_name + "'";
      res.success = false;
      ROS_ERROR("[HumanoidController] changeRuiwoMotorParamCallback: %s", res.message.c_str());
      return true;
    }

    if (kp_values.size() != kd_values.size()) {
      res.message = "ruiwo_kp and ruiwo_kd size mismatch in '" + param_name + "'";
      res.success = false;
      ROS_ERROR("[HumanoidController] changeRuiwoMotorParamCallback: %s", res.message.c_str());
      return true;
    }

    kuavo_settings_.running_settings.ruiwo_kp = kp_values;
    kuavo_settings_.running_settings.ruiwo_kd = kd_values;

    res.message = "Successfully set Ruiwo motor parameters from config: " + param_name;
    res.success = true;
    ROS_INFO("[HumanoidController] changeRuiwoMotorParamCallback: %s", res.message.c_str());
    return true;
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
    if (headNum_ > 0 && sensor_data.jointPos_.size() == jointNumReal_+armNumReal_ + waistNum_ + headNum_)
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
    sensor_data.resize_joint(jointNumReal_+armNumReal_ + waistNum_+headNum_);

    // JOINT DATA - extract from leju::msgs::SensorsData
    size_t joint_count = std::min(static_cast<size_t>(jointNumReal_+armNumReal_ + waistNum_+headNum_),
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
    if (headNum_ > 0 && sensor_data.jointPos_.size() == jointNumReal_+armNumReal_ + waistNum_ + headNum_)
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
    SensorData sensor_data_rl;

    sensor_msgs::Imu imu_msg;
    sensor_data.resize_joint(jointNumReal_+armNumReal_ + waistNum_);
    sensor_data_rl.resize_joint(jointNumReal_+armNumReal_ + waistNum_);
    
    // JOINT DATA
   for(size_t i=0;i<jointNumReal_+waistNum_+armNumReal_;i++)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointTorque_(i) = joint_data.joint_torque[i];
      sensor_data_rl.jointPos_(i) = joint_data.joint_q[i];
      sensor_data_rl.jointVel_(i) = joint_data.joint_v[i];
      sensor_data_rl.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data_rl.jointTorque_(i) = joint_data.joint_torque[i];
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
    sensor_data_rl.timeStamp_ = msg->sensor_time;
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
      Eigen::Vector3d base_imu_free_acc(imu_data.free_acc.x, imu_data.free_acc.y, imu_data.free_acc.z);
      Eigen::Vector3d waist_base_free_acc = waist_base_quat.conjugate() * base_imu_free_acc;


      // 角速度转换
      Eigen::Vector3d waist_gyro(0, 0, qd_waist);
      Eigen::Vector3d imu_gyro(imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
      Eigen::Vector3d waist_base_gyro = waist_base_quat.conjugate() * imu_gyro - waist_gyro; 

      // roban 版本需要转换 imu 数据
      sensor_data.quat_.coeffs().w() = waist_world_quat.coeffs().w();
      sensor_data.quat_.coeffs().x() = waist_world_quat.coeffs().x();
      sensor_data.quat_.coeffs().y() = waist_world_quat.coeffs().y();
      sensor_data.quat_.coeffs().z() = waist_world_quat.coeffs().z();
      sensor_data.angularVel_ << waist_base_gyro[0], waist_base_gyro[1], waist_base_gyro[2];
      sensor_data.linearAccel_ << waist_base_acc[0], waist_base_acc[1], waist_base_acc[2];
      sensor_data.freeLinearAccel_ << waist_base_free_acc[0], waist_base_free_acc[1], waist_base_free_acc[2];
      
      
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
      sensor_data.freeLinearAccel_ << imu_data.free_acc.x, imu_data.free_acc.y, imu_data.free_acc.z;
    }
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data_rl.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data_rl.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data_rl.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data_rl.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data_rl.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data_rl.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data_rl.freeLinearAccel_ << imu_data.free_acc.x, imu_data.free_acc.y, imu_data.free_acc.z;
    sensor_data_rl.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data_rl.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data_rl.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();

    // if (!rl_available_)
    {
      sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
      sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
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

    applySensorDataRL(sensor_data_rl);
    if (!is_initialized_)
      is_initialized_ = true;
  }
  void humanoidController::updatakinematics(const SensorData &sensor_data, bool is_initialized_)
  {
   
    SensorData sensor_data_new = sensor_data;
    ros::Time current_sensor_data_time = ros::Time::now();
    if (!is_initialized_ || last_fall_down_state_ != fall_down_state_)
    {
      last_sensor_data_time_ = current_sensor_data_time - ros::Duration(0.002);
    }

    last_fall_down_state_ = fall_down_state_;
    
    // 使用现有的last_is_rl_controller_变量来判断上一次是否是MPC模式
    bool last_was_mpc = !last_is_rl_controller_;
    bool current_is_mpc = !is_rl_controller_;
    
     // 在RL模式下，如果不在MPC-RL插值期间，则只更新IMU并返回
     // 如果正在插值（is_torso_interpolation_active_），MPC仍在运行，需要继续更新kinematics
     if (is_rl_controller_ && !is_torso_interpolation_active_)
     {
       double diff_time = (current_sensor_data_time - last_sensor_data_time_).toSec();
       ros::Duration period = ros::Duration(diff_time);
       
       // 使用原始传感器数据更新IMU（RL模式下不使用robot_localization融合）
       stateEstimate_->updateImu(sensor_data_new.quat_, sensor_data_new.angularVel_, sensor_data_new.linearAccel_, 
                                 sensor_data_new.orientationCovariance_, sensor_data_new.angularVelCovariance_, 
                                 sensor_data_new.linearAccelCovariance_);
       last_sensor_data_time_ = current_sensor_data_time;
       return;
     }
    double diff_time = (current_sensor_data_time - last_sensor_data_time_).toSec();
    ros::Duration period = ros::Duration(diff_time);
    nav_msgs::Odometry kinematics_odom;
    sensor_msgs::Imu imu_msg;
    // updateKinematics内部会使用自己的yaw和传入的roll/pitch融合，所以传入原始传感器值
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
        updata_eulerAngles << robot_eulerAngles(0), sensor_eulerAngles(1), sensor_eulerAngles(2);
        robot_quat_state_update_ = Eigen::AngleAxisd(updata_eulerAngles[0], Eigen::Vector3d::UnitZ())*
                                   Eigen::AngleAxisd(updata_eulerAngles[1], Eigen::Vector3d::UnitY())*
                                   Eigen::AngleAxisd(updata_eulerAngles[2], Eigen::Vector3d::UnitX());
        robotlocalizationDataQueue.pop();
      }
      else
      {
        // 如果队列为空，根据上一次和当前的控制器模式决定策略
        // 插值期间：即使current_is_mpc=false，也要保持yaw连续性，避免姿态跳变导致估计器不稳定
        if ((last_was_mpc && current_is_mpc) || is_torso_interpolation_active_)
        {
          // 上一次是MPC，当前也是MPC：保持上一次融合的yaw值，避免yaw跳变
          // 插值期间：同样保持yaw连续性，确保状态估计器稳定
          // 只更新roll和pitch，保持yaw连续性
          Eigen::Vector3d sensor_euler = quatToZyx(sensor_data_new.quat_);
          Eigen::Vector3d last_robot_euler = quatToZyx(robot_quat_state_update_);
          Eigen::Vector3d keep_yaw_euler;
          keep_yaw_euler << last_robot_euler(0), sensor_euler(1), sensor_euler(2);
          robot_quat_state_update_ = Eigen::AngleAxisd(keep_yaw_euler[0], Eigen::Vector3d::UnitZ())*
                                     Eigen::AngleAxisd(keep_yaw_euler[1], Eigen::Vector3d::UnitY())*
                                     Eigen::AngleAxisd(keep_yaw_euler[2], Eigen::Vector3d::UnitX());
        }
        else
        {
          // 从RL切换到MPC或其他情况：使用传感器原始值，保证过渡的稳定性
          robot_quat_state_update_ = sensor_data_new.quat_;
        }
      }
    }  // robotlocalization_data_mutex_
    sensor_data_new.quat_.w() = robot_quat_state_update_.w();
    sensor_data_new.quat_.x() = robot_quat_state_update_.x();
    sensor_data_new.quat_.y() = robot_quat_state_update_.y();
    sensor_data_new.quat_.z() = robot_quat_state_update_.z();
    last_sensor_data_time_ = current_sensor_data_time;
    ros_logger_->publishVector("/sensor_data_new/rpy/zyx", quatToZyx(sensor_data_new.quat_).transpose());
    ros_logger_->publishVector("/sensor_data_new/quat/wxyz", quatToZyx(sensor_data_new.quat_).transpose());
    
    // updateImu使用融合后的值（robot_localization的yaw + 传感器的roll/pitch）
    stateEstimate_->updateImu(sensor_data_new.quat_, sensor_data_new.angularVel_, sensor_data_new.linearAccel_, 
                              sensor_data_new.orientationCovariance_, sensor_data_new.angularVelCovariance_, 
                              sensor_data_new.linearAccelCovariance_);
  }

  void humanoidController::resetKinematicsEstimation()
  {
    // 获取当前传感器数据
    SensorData sensors_data = sensors_data_buffer_ptr_->getLastData();
    
    // 清空robotlocalization队列中的旧数据
    {
      std::lock_guard<std::mutex> lock(robotlocalization_data_mutex_);
      size_t queue_size_before = robotlocalizationDataQueue.size();
      while(!robotlocalizationDataQueue.empty())
      {
        robotlocalizationDataQueue.pop();
      }
      // robot_quat_state_update_会在第一次updatakinematics调用时自动更新为当前传感器值
    }
    
    // 重置状态估计器
    stateEstimate_->reset();
    
    // 重置时间戳，确保第一次更新的period很小
    last_sensor_data_time_ = sensors_data.timeStamp_ - ros::Duration(0.002);
    
    // 更新关节状态
    stateEstimate_->updateJointStates(joint_pos_, joint_vel_);
    
    // 使用当前IMU值更新初始欧拉角
    quat_init = stateEstimate_->updateIntialEulerAngles(sensors_data.quat_);
    
    // 更新IMU数据
    stateEstimate_->updateImu(sensors_data.quat_, sensors_data.angularVel_, 
                            sensors_data.linearAccel_, 
                            sensors_data.orientationCovariance_, 
                            sensors_data.angularVelCovariance_, 
                            sensors_data.linearAccelCovariance_);
    
    // 获取更新后的RBD状态并构建初始centroidal状态
    measuredRbdState_ = stateEstimate_->getRbdState();
    vector_t initial_centroidal_state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);

    
    // 保持yaw的连续性，避免切换时跳变
    // 从传感器数据获取当前yaw值
    Eigen::Vector3d sensor_euler = quatToZyx(sensors_data.quat_);
    scalar_t sensor_yaw = sensor_euler(0);
    
    // 确定使用哪个yaw值作为参考
    scalar_t yawLast = sensor_yaw;  // 默认使用传感器yaw
    bool use_sensor_yaw = true;
    
    if (currentObservation_.state.size() > 9 && std::abs(currentObservation_.state(9)) > 1e-6)
    {
      scalar_t current_yaw = currentObservation_.state(9);
      scalar_t yaw_diff = std::abs(angles::shortest_angular_distance(current_yaw, sensor_yaw));
      
      ROS_INFO("[ResetKinematics] yaw diff: |current_yaw - sensor_yaw| = %.6f", yaw_diff);
      
      // 如果currentObservation_中的yaw与传感器yaw差异小于0.5弧度，使用currentObservation_的yaw
      // 否则说明currentObservation_中的yaw可能已过时，使用传感器yaw
      if (yaw_diff < 0.5)
      {
        yawLast = current_yaw;
        use_sensor_yaw = false;
        ROS_INFO("[ResetKinematics] Using currentObservation_.state(9) as yaw reference: %.6f (diff from sensor: %.6f < 0.5)", 
                 yawLast, yaw_diff);
      }
      else
      {
        ROS_WARN("[ResetKinematics] currentObservation_.state(9)=%.6f differs too much from sensor yaw=%.6f (%.6f >= 0.5), using sensor yaw", 
                 current_yaw, sensor_yaw, yaw_diff);
      }
    }
    else
    {
      ROS_INFO("[ResetKinematics] currentObservation_.state(9) invalid (size=%zu or value=%.6f), using sensor yaw: %.6f", 
               currentObservation_.state.size(), 
               (currentObservation_.state.size() > 9) ? currentObservation_.state(9) : 0.0,
               sensor_yaw);
    }
    
    // 计算yaw连续性保持
    scalar_t newYaw = initial_centroidal_state(9);
    scalar_t yawDiff = angles::shortest_angular_distance(yawLast, newYaw);
    scalar_t yaw_before = initial_centroidal_state(9);
    initial_centroidal_state(9) = yawLast + yawDiff;
    
    // 设置状态估计器的初始状态
    ROS_INFO("[ResetKinematics] initial_centroidal_state(9) before set: %.6f", initial_centroidal_state(9));
    stateEstimate_->set_intial_state(initial_centroidal_state);
    
    // 重要：更新currentObservation_.state为新的初始状态，确保resetMpcNode使用正确的yaw值
    // 这样MPC的参考轨迹会基于正确的yaw值进行校准
    scalar_t currentObs_yaw_before = (currentObservation_.state.size() > 9) ? currentObservation_.state(9) : 0.0;
    currentObservation_.state = initial_centroidal_state;
    
    // 更新stanceState_mrt_为重置后的状态，确保后续使用正确的yaw值
    stanceState_mrt_ = initial_centroidal_state;
    ROS_INFO("[ResetKinematics] stanceState_mrt_.yaw=%.6f", stanceState_mrt_(9));
    
  }
  
  bool humanoidController::enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
      bool old_mode = use_ros_arm_joint_trajectory_;
      use_ros_arm_joint_trajectory_ = req.control_mode;

      ultra_fast_mode_ = req.control_mode;
      if(req.control_mode == kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode)
      {
        last_ultra_fast_mode_ = true;
        ROS_INFO_STREAM("[humanoidController] ultra fast mode");
      }

      if(last_ultra_fast_mode_ && use_ros_arm_joint_trajectory_){
        ultra_fast_mode_ = kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode;
        ROS_INFO_STREAM("[humanoidController] ultra fast mode Enter Again");
      }
      
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
    if (is_roban_) {
      int arm_start = 12 + jointNumReal_ + waistNum_;
      if (static_cast<int>(initial_status_.size()) >= arm_start + static_cast<int>(armNumReal_)) {
        initial_status_.segment(arm_start, armNumReal_) = defalutArmPosMPC_;
        arm_joint_pos_filter_.reset(defalutArmPosMPC_);
      } else {
        ROS_WARN_STREAM("[starting] initial_status_.size()=" << initial_status_.size()
                        << " too small to set arm segment at " << arm_start
                        << " (need " << armNumReal_ << ")");
      }
    }

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
      /*******************输入蹲姿和站姿**********************/
      auto &infoWBC = centroidalModelInfoWBC_;
      vector_t squatState = vector_t::Zero(infoWBC.stateDim);
      squatState.head(12 + jointNum_) = drake_interface_->getSquatInitialState();
      vector_t standState = vector_t::Zero(infoWBC.stateDim);
      standState.head(12 + jointNum_) = drake_interface_->getInitialState();
      standState.tail(armNumReal_) = defalutArmPosMPC_;
      // is_rl_start 模式下，直接起立到 RL 的默认姿态和高度，确保衔接平滑
      if (is_rl_start_ && currentDefalutJointPosRL_.size() == (jointNumReal_ + waistNum_ + armNumReal_))
      {
        standState.segment(12, currentDefalutJointPosRL_.size()) = currentDefalutJointPosRL_;
        standState(8) = defaultBaseHeightControl_;
      }
      /*******采用 standUp_controller 从蹲姿运动到站姿*********/
      stateEstimate_->setFixFeetHeights(true);
      updateStateEstimation(time, false);
      // vector_t measuredRbdStateRL_;
      // measuredRbdStateRL_ = getRobotState();
      double startTime;
      double endTime;
      double motionVel;
      if(is_roban_)
        motionVel = 0.03;  //鲁班站立速度
      else
        motionVel = 0.11;   //其他机器人站立速度
      
      if (!isInitStandUpStartTime_)
      {
        resetKinematicsEstimation();
        initial_status_(9) = currentObservation_.state(9);
        
        isInitStandUpStartTime_ = true;
        robotStartStandTime_ = time.toSec();
        // 站立的结束时间是依据开始时间确定的
        startTime = robotStartStandTime_;
        endTime = startTime + (standState[8] - squatState[8]) / motionVel; // 以 motionVel 速度起立
        robotStandUpCompleteTime_ = endTime;
        // std::cout << "standUp duration: " << robotStandUpCompleteTime_ - startTime << " seconds"  << std::endl;
        ROS_INFO_STREAM("Set standUp start time: " << startTime << " end time: " << robotStandUpCompleteTime_);
      }

      vector_t curState = vector_t::Zero(infoWBC.stateDim);
      vector_t desiredState = vector_t::Zero(infoWBC.stateDim);
      if (is_abnor_StandUp_)
      {
        // 机器人站立异常，恢复到蹲起姿态
        curState = curRobotLegState_;
        desiredState = squatState;
        startTime = robotStartSquatTime_;
        endTime = startTime + (curRobotLegState_[8] - squatState[8]) / motionVel; // 以 motionVel 速度挂起
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
      if (!init_fall_down_state_)
      { 
        replaceDefaultEcMotorPdoGait(jointCmdMsg);
        publishControlCommands(jointCmdMsg);
      }
      
      // if (use_shm_communication_) 
      //     publishJointCmdToShm(jointCmdMsg);

      if (!wheel_arm_robot_ && stand_up_protect_ && is_real_)
      {
        const double norSingleLegSupport = centroidalModelInfo_.robotMass * 9.8 / 4; // 单脚支撑力只要达到重量的1/4的力即认为已落地成功
        bool bNotLanding = is_robot_standup_complete_ && (contactForce_[2] < norSingleLegSupport || contactForce_[8] < norSingleLegSupport);
        bool bUneventForce = fabs(contactForce_[2] - contactForce_[8]) > (norSingleLegSupport * 2.0); // 左右脚支撑立差值超过重量的1/2即判断为异常/*  */
        if (bNotLanding || bUneventForce)
        {
          if (!is_abnor_StandUp_ && (bNotLanding || bUneventForce || (time.toSec() > robotStandUpCompleteTime_ + 0.5)))
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
    if (time.toSec() > robotStandUpCompleteTime_ + 0.8 || !is_real_ || init_fall_down_state_)
    {
      SystemObservation initial_observation = currentObservation_;
      initial_observation.state = initial_status_;
      TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});
      mpc_current_target_trajectories_ = target_trajectories;
      // Set the first observation and command and wait for optimization to finish
      ROS_INFO_STREAM("Waiting for the initial policy ...");
      // Reset MPC node
      mrtRosInterface_->resetMpcNode(target_trajectories);
      std::cout << "reset MPC node\n";
      vector_t current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
      vector_t current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
      arm_joint_pos_filter_.reset(current_arm_pos);
      arm_joint_vel_filter_.reset(current_arm_vel);
      //暂时跳过MPC初始化
      if (!is_rl_start_)
      {
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
      else
      {
        mrtRosInterface_->pauseResumeMpcNode(true);
        ROS_INFO("[HumanoidController] is_rl_start=true, skipping MPC initialization in preUpdate");
      }
      
      stateEstimate_->setFixFeetHeights(false);
      isPreUpdateComplete = true;
      standupTime_ = currentObservation_.time;

      standUpWbc_->loadSwitchParamsSetting(taskFile_switchParams_, true, is_real_);

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
    // 使用共享内存更新传感器数据
    if (use_shm_communication_) {
      updateSensorDataFromShm();
    }
    updateStateEstimation(time, false);
    ros_logger_->publishValue("/humanoid_controller/is_rl_controller_", is_rl_controller_);
    ros_logger_->publishValue("/humanoid_controller/resetting_mpc_state_", resetting_mpc_state_);
    ros_logger_->publishValue("/humanoid_controller/fall_down_state_", fall_down_state_);

    // is_rl_controller_buffer_.updateFromBuffer();// 使用buffer中的值更新is_rl_controller_,避免多线程更新
    is_rl_controller_ = !controller_manager_->isBaseControllerActive();
    current_controller_ptr_ = controller_manager_->getCurrentController();

    // 同步MPC stance状态到RLControllerManager
    controller_manager_->setMpcStanceState(is_stance_mode_, current_gait_.name);

    RLControllerType current_controller_type = controller_manager_->getCurrentControllerType();
    bool is_fall_stand_controller_active = current_controller_type == RLControllerType::FALL_STAND_CONTROLLER;
    if (is_rl_controller_)// 针对倒地起身控制器这种可以主动退出的控制器
    {
      if (current_controller_ptr_->isReadyToExit())
      {
        if(current_controller_type==RLControllerType::DANCE_CONTROLLER)
        {
          ROS_INFO("[HumanoidController] Dance controller finished, switching to AMP walk controller");
          if (controller_manager_->switchController(RLControllerType::AMP_CONTROLLER))
          {
            current_controller_ptr_ = controller_manager_->getCurrentController();
            ROS_INFO("[HumanoidController] Successfully switched to AMP walk controller");
          }
          else
          {
            ROS_ERROR("[HumanoidController] Failed to switch to AMP walk controller");
          }
        }
        else
        {
          ROS_WARN("[HumanoidController] Current controller requests exit, switching to BASE controller");
          controller_manager_->switchToBaseController();
          fall_down_state_ = FallStandState::STANDING;
        }
      }
    }
    // is_rl_start模式：MPC初始化完成后直接切换到RL控制器（不需要MPC插值）
    if (is_rl_start_ && isPreUpdateComplete && !is_rl_controller_)
    {
      // 检查是否有有效的RL控制器
      if (current_controller_ != "mpc" && controller_manager_->switchController(current_controller_))
      {
        current_controller_ptr_ = controller_manager_->getCurrentController();
        is_rl_controller_ = last_is_rl_controller_ = true;
        is_rl_start_ = false;
        ROS_INFO("[HumanoidController] is_rl_start: MPC initialized, switched to RL controller: %s", current_controller_.c_str());
      }
    }
    // 是否直接切换到RL控制器（true：直接切换；false：通过MPC插值过渡）
    // 从当前RL控制器的配置文件中读取 use_interploate_from_mpc 参数
    bool derect_switch_to_rl = true;  // 默认直接切换到RL控制器
    if (is_rl_controller_ && current_controller_ptr_ != nullptr)
    {
      // 从当前控制器获取参数：use_interpolate_from_mpc = true 表示使用插值（derect_switch_to_rl = false）
      // use_interpolate_from_mpc = false 表示直接切换（derect_switch_to_rl = true）
      bool use_interpolate = current_controller_ptr_->getUseInterpolateFromMPC();
      derect_switch_to_rl = !use_interpolate;
    }
    if (!derect_switch_to_rl && !last_is_rl_controller_ && is_rl_controller_)
    {
      // 进入 RL 前，先用 MPC 将躯干高度插值到 RL 默认高度
      inference_running_ = true;
      std::cout << "pause MPC" << std::endl;
      Eigen::VectorXd current_arm_pos = Eigen::VectorXd::Zero(armNumReal_);
      Eigen::VectorXd current_arm_vel = Eigen::VectorXd::Zero(armNumReal_);
      current_arm_pos = jointPosWBC_.segment(jointNumReal_+ waistNum_, armNumReal_);
      current_arm_vel = jointVelWBC_.segment(jointNumReal_+ waistNum_, armNumReal_);

      currentDefalutJointPosRL_ = current_controller_ptr_->getDefaultJointPos();
      defaultBaseHeightControl_ = current_controller_ptr_->getDefaultBaseHeightControl();
      std::cout << "last_controller_ptr->getName(): " << current_controller_ptr_->getName() << std::endl;
      std::cout << "New currentDefalutJointPosRL_: " << currentDefalutJointPosRL_.transpose() << std::endl;

      Eigen::VectorXd target_arm_pos = currentDefalutJointPosRL_.segment(jointNumReal_+ waistNum_, armNumReal_);
      Eigen::VectorXd target_arm_vel = Eigen::VectorXd::Zero(armNumReal_);

      // 初始化RL模式的手臂目标位置和速度（避免保留旧的外部控制值）
      desire_arm_q_ = currentDefalutJointPosRL_.segment(jointNumReal_ + waistNum_, armNumReal_);
      desire_arm_v_ = Eigen::VectorXd::Zero(armNumReal_);

      // 清理 kuavo_arm_traj 话题缓存（MPC->RL时也清理）
      // 避免RL模式下使用MPC遗留的外部控制数据
      arm_joint_trajectory_.pos = current_arm_pos;
      arm_joint_trajectory_.vel = current_arm_vel;
      arm_joint_trajectory_.tau = Eigen::VectorXd::Zero(armNumReal_);
      ROS_INFO("[MPC->RL] 清理手臂轨迹缓存");

      // 启动躯干插值，XY 基于当前双脚中心 + RL 控制器配置的 base X 偏移，Z 对齐 RL 默认高度
      // 使用已有的躯干插值系统，并覆盖目标高度为 RL 默认高度
      vector3_t feet_center = stateEstimate_->getFeetCenterPosition();
      // RL 控制器配置的站立时 base 在 x 方向相对于足端中心(0)的偏移（机器人前向）
      double base_x_offset = 0.0;
      if (current_controller_ptr_)
      {
        base_x_offset = current_controller_ptr_->getDefaultBaseXOffsetControl();
      }
      // 将机体前向的 X 偏移旋转到世界坐标系，并叠加到双脚中心位置
      const double yaw = currentObservation_.state(9);
      feet_center(0) += std::cos(yaw) * base_x_offset;
      feet_center(1) += std::sin(yaw) * base_x_offset;
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
      
      // 从RL切换到MPC时，重置运动学估计（包括状态估计器、时间戳、yaw连续性等）
      resetKinematicsEstimation();
    }
    last_is_rl_controller_ = is_rl_controller_;
    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    
    mrtRosInterface_->setCurrentObservation(currentObservation_);

    // 非RL或躯干插值中，均走MPC流程
    bool mpc_flow = !is_rl_controller_;
    if (!derect_switch_to_rl)
    {
      mpc_flow = (!is_rl_controller_) || is_torso_interpolation_active_;
    }
    if (mpc_flow && !is_fall_stand_controller_active)
    {
      is_mpc_controller_ = true;
      if (reset_mpc_) // 重置mpc
      {
        // Safety check, if failed, stop the controller
        if (!safetyChecker_->check(currentObservation_, vector_t::Zero(currentObservation_.state.size()), vector_t::Zero(currentObservation_.input.size())))
        {
          ROS_ERROR_STREAM("[humanoid Controller] Safety check failed!");
          fall_down_state_ = FallStandState::FALL_DOWN;
          
          // 检查控制器列表是否存在倒地起身控制器，自动切换过去
          if (controller_manager_ && has_fall_stand_controller_)
          {
            std::cout << "[humanoid Controller]fall down detected, switch to fall down controller" << std::endl;
            controller_manager_->switchController(RLControllerType::FALL_STAND_CONTROLLER);
            current_controller_ptr_ = controller_manager_->getCurrentController();
            current_controller_ptr_->reset();
            mrtRosInterface_->pauseResumeMpcNode(true);
            return;
          }
          ROS_ERROR_STREAM("[humanoid Controller] No Fall Stand Controller, stopping all controllers.");
          std_msgs::Bool stop_msg;
          stop_msg.data = true;
          stop_pub_.publish(stop_msg);
          usleep(100000);
          return;
        }

        mrtRosInterface_->pauseResumeMpcNode(false);
        std::cout << "resume MPC" << std::endl;
        // Trigger MRT callbacks
        mrtRosInterface_->spinMRT();
        currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);      
        auto target_trajectories = TargetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
        ROS_INFO("[RL->MPC] target_trajectories.stateTrajectory[0](9) (yaw): %.6f", 
                 target_trajectories.stateTrajectory[0](9));
        ROS_INFO("[RL->MPC] ====== 调用resetMpcNode ======");
        mrtRosInterface_->resetMpcNode(target_trajectories);
        ROS_INFO("[RL->MPC] resetMpcNode完成");
        // 修复：滤波器重置为当前实际手臂位置，避免跳变
        vector_t current_arm_pos = jointPosWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        vector_t current_arm_vel = jointVelWBC_.segment(jointNumReal_ + waistNum_, armNumReal_);
        arm_joint_pos_filter_.reset(current_arm_pos);
        arm_joint_vel_filter_.reset(current_arm_vel);

        reset_mpc_ = false;
        standupTime_ = currentObservation_.time;

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
        
        // 修复：切换到RL使用实际手臂位置作为插值目标，而不是默认位置，避免跳变
        vector_t target_arm_pos = defalutArmPosMPC_;
        if (is_rl_controller_)
        {
          target_arm_pos = currentDefalutJointPosRL_.segment(jointNumReal_+ waistNum_, armNumReal_);
        }

        startMPCRLInterpolation(currentObservation_.time, targetTorsoPose, target_arm_pos);
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
          // mrtRosInterface_->setCurrentObservation(currentObservation_);
          
          // Trigger MRT callbacks
          mrtRosInterface_->spinMRT();
          // Update the policy if a new on was received
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
                bool mpc_ready = mrtRosInterface_->initialPolicyReceived() && 
                                          mrtRosInterface_->updatePolicy() && 
                                          mrtRosInterface_->isPolicyUpdated() &&
                                          mrtRosInterface_->getPolicyReceiveCount() > 1;
                if (is_rl_controller_)// 切换到RL
                {
                  std::cout << "[MPC->RL] Torso interpolation completed, switching to NORMAL" << std::endl;
                  resetting_mpc_state_ = ResettingMpcState::NOMAL;
                }else if (mpc_ready)
                {
                  std::cout << "MPC-RL interpolation completed, policy receive count: " << mrtRosInterface_->getPolicyReceiveCount() << std::endl;
                  standupTime_ = currentObservation_.time;
                  std::cout << "[RL->MPC] Torso interpolation completed, switching to NORMAL" << std::endl;
                  resetting_mpc_state_ = ResettingMpcState::NOMAL;
                }

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
              double height_tol = 0.002;
              auto& check_state_traj = mpc_current_target_trajectories_.stateTrajectory;
              // 鲁班检查是否是摇杆上下拉起
              if (is_roban_ && check_state_traj.size() >= 1 && check_state_traj.front().size() > 8 && check_state_traj.back().size() > 8)
              {
                double first_height = check_state_traj.front()(8);
                double last_height = check_state_traj.back()(8);
                condition_pull_up_mpc_height_ = (std::abs(first_height - default_state_[8]) < height_tol &&
                                        std::abs(last_height - default_state_[8]) < height_tol);
              }
              else
                condition_pull_up_mpc_height_ = true; 
            }
            
            
            if (resetting_mpc_state_ != ResettingMpcState::NOMAL /* && !is_rl_controller_ */) // 当前是从RL切换到MPC, 使用WBC插值防止MPC没有启动
            {
              optimizedState_mrt.segment<6>(6) = torso_interpolation_result_;
              optimizedState_mrt.segment(12, jointNumReal_+ waistNum_) = leg_interpolation_result_.head(jointNumReal_+ waistNum_);
              // optimizedState_mrt.segment(12, jointNumReal_+ waistNum_) = default_state_.segment(12,jointNumReal_+ waistNum_);
              // optimizedInput_mrt = stanceInput_mrt_;
              plannedMode_ = ModeNumber::SS;

            }else if (mrtRosInterface_->isPolicyUpdated())
            {
              mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
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
          for (int i = 0; i < 2; i++)
          {
            // 只使用上半身模式, 此时 MPC 求解未开启, 直接使用 target_arm_pos
            if (only_half_up_body_)
            {
              optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofReal_) =
                  target_arm_pos.segment(i * armDofReal_, armDofDiff_);
            }
            else
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
      if (use_ros_arm_joint_trajectory_ && resetting_mpc_state_ == ResettingMpcState::NOMAL)
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
          // vector_t computed_vel = (filtered_pos - prev_filtered_pos) / dt_;
          vector_t computed_vel = vector_t::Zero(armNumReal_);
            
            // 3. 对计算出的速度再次滤波
          optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(computed_vel);

          //ros_logger_->publishVector("/humanoid_controller/arm_joint_computed_vel", computed_vel);
          prev_filtered_pos = filtered_pos;

          if(ultra_fast_mode_ == kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode)
          {
            optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_trajectory_.pos;          
            optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_trajectory_.vel;
          }
    
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
        if (resetting_mpc_state_ != ResettingMpcState::NOMAL)
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
        static vector_t x;

        if( resetting_mpc_state_ != ResettingMpcState::NOMAL || is_torso_interpolation_active_)
        {
          x = standUpWbc_->update(optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, measuredRbdStateReal_, ModeNumber::SS, period.toSec(), false);
        }
        else
        {
          x = wbc_->update(optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, measuredRbdStateReal_, plannedMode_, period.toSec(), is_mpc_updated);
        }
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
          ROS_ERROR_STREAM("[humanoid Controller] Safety check failed!");
          fall_down_state_ = FallStandState::FALL_DOWN;
          
          // 检查控制器列表是否存在倒地起身控制器，自动切换过去
          if (controller_manager_ && has_fall_stand_controller_)
          {
            std::cout << "[humanoid Controller]fall down detected, switch to fall down controller" << std::endl;
            controller_manager_->switchController(RLControllerType::FALL_STAND_CONTROLLER);
            current_controller_ptr_ = controller_manager_->getCurrentController();
            current_controller_ptr_->reset();
            mrtRosInterface_->pauseResumeMpcNode(true);
            return;
          }
          ROS_ERROR_STREAM("[humanoid Controller] No Fall Stand Controller, stopping all controllers.");
          
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
        if(!is_roban_ && is_torso_interpolation_active_ && i1 < jointNumReal_)
        {
          jointCmdMsg.control_modes.push_back(2); // 躯干插值阶段，腿部全部位置控制
          continue;
        }
        jointCmdMsg.control_modes.push_back(joint_control_modes_[i1]);

      }
      ModeSchedule current_mode_schedule;
      if (resetting_mpc_state_ == ResettingMpcState::NOMAL)
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
        const auto &running_settings = kuavo_settings_.running_settings;
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
          double head_kp_cmd = 0.0, head_kd_cmd = 0.0;
          {
            const int n_ruiwo = static_cast<int>(running_settings.ruiwo_kp.size());
            if (!running_settings.ruiwo_kp.empty() &&
                !running_settings.ruiwo_kd.empty() &&
                running_settings.ruiwo_kp.size() == running_settings.ruiwo_kd.size() &&
                n_ruiwo >= headNum_)
            {
              const int base = n_ruiwo - headNum_;
              head_kp_cmd = static_cast<double>(running_settings.ruiwo_kp[base + i3]);
              head_kd_cmd = static_cast<double>(running_settings.ruiwo_kd[base + i3]);
            }
          }
          jointCmdMsg.joint_q.push_back(get_head_pos(i3));
          jointCmdMsg.joint_v.push_back(0);
          jointCmdMsg.tau.push_back(head_feedback_tau(i3));
          jointCmdMsg.tau_ratio.push_back(1);
          jointCmdMsg.tau_max.push_back(10);
          jointCmdMsg.control_modes.push_back(2);
          jointCmdMsg.joint_kp.push_back(head_kp_cmd);
          jointCmdMsg.joint_kd.push_back(head_kd_cmd);
        }
        robotVisualizer_->updateHeadJointPositions(sensor_data_head_.jointPos_);
      }

      // 对于 control_modes == 2 且 driver == EC_MASTER 的电机，使用 running_settings.joint_kp 和 joint_kd
      // running_settings.joint_kp 和 joint_kd 只包含 EC_MASTER 电机的值，需要建立映射
      replaceDefaultEcMotorPdoGait(jointCmdMsg);

    }
    else
    {
      if (is_mpc_controller_) // 处理一次从MPC切换的逻辑
      {
        std::cout << "HumanoidController::update: pause MPC" << std::endl;
        mrtRosInterface_->pauseResumeMpcNode(true);
      }
      is_mpc_controller_ = false;
      if (!current_controller_ptr_) { is_rl_controller_ = last_is_rl_controller_ = false; return; }

      jointCmdMsg.header.stamp = time;
      vector_t feetPositions = stateEstimate_->getEndEffectorPositions();
      vector_t baseState = stateEstimate_->getTorsoState();
      // 传入额外的估计量
      current_controller_ptr_->applyFeetPositions(feetPositions);
      current_controller_ptr_->applyBaseState(baseState);
      // 更新控制器
      current_controller_ptr_->update(time, getRobotSensorData(), getRobotState(), jointCmdMsg);



      // 补充头部维度
      // 计算头部反馈力
      if (headNum_ > 0)
      {
        vector_t get_head_pos = vector_t::Zero(headNum_);
        head_mtx.lock();
        get_head_pos = desire_head_pos_;
        head_mtx.unlock();
        auto &hardware_settings = kuavo_settings_.hardware_settings;
        const auto &running_settings = kuavo_settings_.running_settings;
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
          auto head_start_index = jointNumReal_ + waistNum_ + armNumReal_;
          double head_kp_cmd = 0.0, head_kd_cmd = 0.0;
          {
            const int n_ruiwo = static_cast<int>(running_settings.ruiwo_kp.size());
            if (!running_settings.ruiwo_kp.empty() &&
                !running_settings.ruiwo_kd.empty() &&
                running_settings.ruiwo_kp.size() == running_settings.ruiwo_kd.size() &&
                n_ruiwo >= headNum_)
            {
              const int base = n_ruiwo - headNum_;
              head_kp_cmd = static_cast<double>(running_settings.ruiwo_kp[base + i3]);
              head_kd_cmd = static_cast<double>(running_settings.ruiwo_kd[base + i3]);
            }
          }
          jointCmdMsg.joint_q[head_start_index + i3] = get_head_pos(i3);
          jointCmdMsg.joint_v[head_start_index + i3] = 0;
          jointCmdMsg.joint_kp[head_start_index + i3] = head_kp_cmd;
          jointCmdMsg.joint_kd[head_start_index + i3] = head_kd_cmd;
          jointCmdMsg.tau[head_start_index + i3] = head_feedback_tau(i3);
          jointCmdMsg.tau_ratio[head_start_index + i3] = 1;
          jointCmdMsg.tau_max[head_start_index + i3] = 10;
          jointCmdMsg.control_modes[head_start_index + i3] = 2;
        }
      }
      // 如果 use_default_motor_csp_kpkd 为 true，使用 running_settings 中的 kp/kd 替换 EC_MASTER 电机的值
      if (current_controller_ptr_->getUseDefaultMotorCspKpkd())
      {
        replaceDefaultEcMotorPdoGait(jointCmdMsg);
      } 

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
    }
    
    // 发布控制命令
    publishControlCommands(jointCmdMsg);

    // Visualization
    if (visualizeHumanoid_)
    {
      robotVisualizer_->updateSimplifiedArmPositions(simplifiedJointPos_);
      if (is_rl_controller_ || resetting_mpc_state_ != ResettingMpcState::NOMAL)
      {
        // RL 模式下，直接使用 publishObservation 发布 tf 树（不需要 MPC 的 policy 和 command）
        robotVisualizer_->publishObservation(ros::Time::now(), currentObservation_);
      }
      // 仅在 MPC 正常工作且 policy 已更新时，才使用 MPC 轨迹进行可视化
      else if (mrtRosInterface_->isPolicyUpdated())
      {
        robotVisualizer_->update(currentObservation_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());
      }
      robotVisualizer_->updateHeadJointPositions(sensor_data_head_.jointPos_);
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

      joint_pos_.head(jointNum_ + waistNum_) = data.jointPos_.head(jointNum_+ waistNum_);
      joint_vel_.head(jointNum_ + waistNum_) = data.jointVel_.head(jointNum_+ waistNum_);
      joint_acc_.head(jointNum_ + waistNum_) = data.jointAcc_.head(jointNum_+ waistNum_);
      joint_torque_.head(jointNum_ + waistNum_) = data.jointTorque_.head(jointNum_+ waistNum_);

      for (int i = 0; i < 2; i++)
      {
        joint_pos_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointPos_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        joint_vel_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointVel_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        joint_acc_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointAcc_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        joint_torque_.segment(jointNum_ + waistNum_ + armDofMPC_ * i, armDofMPC_) = data.jointTorque_.segment(jointNum_ + waistNum_ + armDofReal_ * i, armDofMPC_);
        simplifiedJointPos_.segment(armDofDiff_ * i, armDofDiff_) = data.jointPos_.segment(jointNum_ + waistNum_ + armDofReal_ * i + armDofMPC_, armDofDiff_);

      }
    }
    else
    {
      joint_pos_ = data.jointPos_;
      joint_vel_ = data.jointVel_;
      joint_acc_ = data.jointAcc_;
      joint_torque_ = data.jointTorque_;
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
    // stateEstimate_->updateJointStates(joint_pos_, joint_vel_);
    stateEstimate_->updateImu(quat_, angularVel_, linearAccel_, orientationCovariance_, angularVelCovariance_, linearAccelCovariance_);
    // apply sensor data to rl
    // auto sensor_data_copy = data.copy();
    // applySensorDataRL(sensor_data_copy);
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
    sensor_data_copy.quat_offset_ = data.quat_;
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
        stateEstimate_->updateJointStates(joint_pos_, joint_vel_);
        quat_init = stateEstimate_->updateIntialEulerAngles(quat_);
        applySensorData(sensors_data);
        stateEstimate_->set_intial_state(currentObservation_.state);
        measuredRbdState_ = stateEstimate_->getRbdState();
        // auto mat = getRobotSensorData().quat_.toRotationMatrix();
      
        // // 计算yaw偏移
        // double current_yaw = std::atan2(mat(1, 2), mat(0, 2)); 
        // my_yaw_offset_ = current_yaw - motionTrajectory_.reference_yaw;
        // std::cout << "sensors_data.quat_: [" << sensors_data.quat_.w() << ", " 
        //           << sensors_data.quat_.x() << ", " << sensors_data.quat_.y() << ", " << sensors_data.quat_.z() << "]" << std::endl;
        // std::cout << "Current yaw: " << current_yaw << std::endl;
        // std::cout << "Reference yaw: " << motionTrajectory_.reference_yaw << std::endl;
        // // 归一化到[-π, π]范围
        // while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
        // while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;

      }
      double diff_time = (current_time_ - last_time_).toSec();

      last_time_ = current_time_;
      ros::Duration period = ros::Duration(diff_time);


      vector_t activeTorque = joint_torque_;
      vector_t activeTorqueWBC =  jointCurrentWBC_;
     
      stateEstimate_->setCmdTorque(activeTorque);
      stateEstimate_->estContactForce(period);
      auto est_contact_force = stateEstimate_->getEstContactForce();
      contactForce_ = est_contact_force;
      ros_logger_->publishVector("/state_estimate/Contact_Detection/contactForce", est_contact_force);
      if (is_rl_controller_)
      {
        plannedMode_ = rl_plannedMode_;
        ros_logger_->publishValue("/rl_controller/rl_optimized_mode_", static_cast<double>(plannedMode_));
      }

      if(!is_simplified_model_)
      {
        stateEstimate_->estArmContactForce(period);
        auto arm_force_simple = stateEstimate_->getEstArmContactForce();
        ros_logger_->publishVector("/state_estimate/arm_contact_force", arm_force_simple);
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
        Eigen::VectorXd updated_joint_pos = joint_pos_;
        Eigen::VectorXd updated_joint_vel = joint_vel_;
        Eigen::VectorXd updated_joint_torque = joint_torque_;
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
        
        // 更新躯干稳定性状态（用于控制器切换检测）
        stateEstimate_->updateTorsoStability(time, period);
        
        currentObservation_.time += period.toSec();
      }
      // 只有非半身轮臂模式站立状态&&站起来稳定之后进行保护, 并且手臂不是外部遥操作模式才可触发拉起保护
      // 确保当前模式已切换到期望模式后才启用拉起保护，模式同步后还需要等待1.5秒才能触发拉起保护
      double current_time = ros::Time::now().toSec();
      bool mode_sync_ready = (mpcArmControlMode_ == mpcArmControlMode_desired_) && 
                             (current_time - arm_mode_sync_time_ >= 1.5);
      bool enable_pull_up = enable_pull_up_protect_ &&  !is_rl_controller_ && isPreUpdateComplete && is_stance_mode_ && 
        !only_half_up_body_ && currentObservation_.time - standupTime_ > 4 
        && mpcArmControlMode_ != ArmControlMode::EXTERN_CONTROL && resetting_mpc_state_ == ResettingMpcState::NOMAL
        && mode_sync_ready  // 确保当前模式已切换到期望模式，且已等待1.5秒
        && (!is_roban_ || condition_pull_up_mpc_height_);  // roban 时要求 MPC 规划高度接近站立高度时才进行触发

      // 发布拉起保护启用状态
      ros_logger_->publishValue("/state_estimate/enable_pull_up", enable_pull_up);

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
        ROS_WARN_STREAM("now Contact Force: " << contactForce_[2]+contactForce_[8] <<
                        " desired Contact Force: " << pull_up_force_threshold_ * robotMass_ * 9.81);
      }
      else if (isPullUp_ && (currentObservation_.time - pull_up_trigger_time_ > 2.0))
      {
        if (!has_fall_stand_controller_)
        {
          ROS_WARN_STREAM("Pull up protection triggered - publishing stop_robot message");
          // 发布stop_robot话题
          std_msgs::Bool stop_msg;
          stop_msg.data = true;
          stop_pub_.publish(stop_msg);
          ROS_WARN_STREAM("stop_robot message published");
        }else {
          // 检查控制器列表是否存在倒地起身控制器，自动切换过去
          ROS_WARN_STREAM("Pull up detected, switch to fall down controller");
          controller_manager_->switchController(RLControllerType::FALL_STAND_CONTROLLER);
          current_controller_ptr_ = controller_manager_->getCurrentController();
          current_controller_ptr_->reset();
          mrtRosInterface_->pauseResumeMpcNode(true);

          isPullUp_ = false;
        }
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
     
      // 手臂末端接触力估计，完整模型计算
      if(is_simplified_model_ && !is_roban_)
      {
        stateEstimate_->setArmForceInputs(measuredRbdStateReal_, activeTorqueWBC);
        stateEstimate_->estArmContactForce(period);
        ros_logger_->publishVector("/state_estimate/arm_contact_force", stateEstimate_->getEstArmContactForce());
      }

      setRobotState(measuredRbdStateReal_);
      ros_logger_->publishVector("/state_estimate/measuredRbdStateReal", measuredRbdStateReal_);

      // std::cout << "jointPosWBC_:" << jointPosWBC_.transpose() << std::endl;
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

  void humanoidController::setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &referenceFile)
  {
    // 这部分只有下肢，可能需要修改。
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose, referenceFile);

    currentObservation_.time = 0;

    // 若 MPC 使用简化模型（例如手臂DOF从14降到8），则在估计器内部再配置一套 WBC/full Pinocchio 模型，仅用于手臂末端接触力估计（保持 MPC 估计主链路维度不变）
    if (is_simplified_model_ && pinocchioInterfaceWBCPtr_ && !is_roban_) 
    {
      std::cout << "set Full Arm Force Model." << std::endl;
      stateEstimate_->setFullArmForceModel(*pinocchioInterfaceWBCPtr_, centroidalModelInfoWBC_);
    }
  }

  void humanoidCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/, const std::string & /*referenceFile*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                              HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
  }

  void humanoidKuavoController::setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &referenceFile)
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

/*****************************************倒地起身*****************************************************************/ 
bool humanoidController::loadMotionTrajectory(const std::string& trajectoryFile) {
  std::ifstream file(trajectoryFile);
  if (!file.is_open()) {
      std::cerr << "Failed to open trajectory file: " << trajectoryFile << std::endl;
      return false;
  }
  
  std::vector<std::vector<double>> csvData;
  std::string line;
  bool firstLine = true;
  int lineNum = 0;
  const int expectedCols = 3 + 4 + (jointNum_ + jointArmNum_ + waistNum_) * 2;
  
  while (std::getline(file, line)) {
      lineNum++;
      if (line.empty()) continue;
      
      std::vector<double> row;
      std::stringstream ss(line);
      std::string value;
      bool parseError = false;
      
      while (std::getline(ss, value, '\t')) {
          if (value.empty()) continue;
          try {
              row.push_back(std::stod(value));
          } catch (const std::exception& e) {
              // 解析失败，可能是表头
              parseError = true;
              break;
          }
      }        
      // 如果是第一行且解析失败，认为是表头，跳过
      if (firstLine && parseError) {
          std::cout << "Detected header line, skipping: " << line.substr(0, std::min(80, (int)line.size())) << "..." << std::endl;
          firstLine = false;
          continue;
      }
      firstLine = false;
      
      if (row.size() == expectedCols) {
          csvData.push_back(row);
      } else if (row.size() > 0 && !parseError) {
          std::cerr << "Warning: Line " << lineNum << " has " << row.size() 
                   << " columns, expected " << expectedCols << " (49)" << std::endl;
      }
  }
  file.close();
  
  int timeSteps = csvData.size();
  if (timeSteps == 0) {
      std::cerr << "No valid data found in CSV file" << std::endl;
      return false;
  }
  
  const int numJoints = jointNum_ + jointArmNum_ + waistNum_;
  motionTrajectory_.time_step_total = timeSteps;
  motionTrajectory_.current_time_step = 0;
  
  motionTrajectory_.body_pos_w.resize(timeSteps, 3);
  motionTrajectory_.body_quat_w.resize(timeSteps, 4);
  motionTrajectory_.joint_pos.resize(timeSteps, numJoints);
  motionTrajectory_.joint_vel.resize(timeSteps, numJoints);
  
  for (int i = 0; i < timeSteps; ++i) {
      const auto& row = csvData[i];
      
      motionTrajectory_.body_pos_w(i, 0) = row[0];
      motionTrajectory_.body_pos_w(i, 1) = row[1];
      motionTrajectory_.body_pos_w(i, 2) = row[2];
      
      motionTrajectory_.body_quat_w(i, 0) = row[3];  // w
      motionTrajectory_.body_quat_w(i, 1) = row[4];  // x
      motionTrajectory_.body_quat_w(i, 2) = row[5];  // y
      motionTrajectory_.body_quat_w(i, 3) = row[6];  // z
      
      for (int j = 0; j < numJoints; ++j) {
          motionTrajectory_.joint_pos(i, j) = row[7 + j];
      }        
      for (int j = 0; j < numJoints; ++j) {
          motionTrajectory_.joint_vel(i, j) = row[7 + numJoints + j];
      }
  }
  if (timeSteps > 0) {
      Eigen::Quaterniond first_quat(
          motionTrajectory_.body_quat_w(0, 0),  // w
          motionTrajectory_.body_quat_w(0, 1),  // x
          motionTrajectory_.body_quat_w(0, 2),  // y
          motionTrajectory_.body_quat_w(0, 3)   // z
      );
      
      Eigen::Matrix3d ref_mat = first_quat.toRotationMatrix();        
      motionTrajectory_.reference_yaw = std::atan2(ref_mat(1, 2), ref_mat(0, 2));
  }    
  return true;
}

Eigen::VectorXd humanoidController::getTrajectoryCommand() {
  if (!trajectoryLoaded_) {
    return Eigen::VectorXd::Zero((jointNum_ + jointArmNum_ + waistNum_) * 2);
  }
  return motionTrajectory_.getCurrentCommand();
}
Eigen::VectorXd humanoidController::getTrajectoryAnchorPos() {
  return motionTrajectory_.getTargetAnchorPos();
}
Eigen::Quaterniond humanoidController::getTrajectoryAnchorQuat() {
  return motionTrajectory_.getTargetAnchorQuat();
}



// Get motion anchor position difference in body frame
Eigen::Vector3d humanoidController::getMotionAnchorPosB(const Eigen::Vector3d& currentBasePos, const Eigen::Quaterniond& currentBaseQuat) {
  if (!trajectoryLoaded_) {
    return Eigen::Vector3d::Zero();
  }
  
  // Get target anchor position and orientation from trajectory
  Eigen::Vector3d targetAnchorPos = motionTrajectory_.getTargetAnchorPos();
  
  // Calculate position difference in body frame using subtract_frame_transforms logic
  // T_12 = T_01^(-1) * T_02, where T_01 is current robot pose, T_02 is target pose
  Eigen::Quaterniond currentQuatInv = currentBaseQuat.inverse();
  Eigen::Vector3d positionDiff = currentQuatInv * (targetAnchorPos - currentBasePos);
  return positionDiff;
}

// Get motion anchor orientation difference in body frame  
Eigen::VectorXd humanoidController::getMotionAnchorOriB(const Eigen::Quaterniond& currentBaseQuat) {
  if (!trajectoryLoaded_) {
    return Eigen::VectorXd::Zero(6);
  }
  
  // Get target anchor orientation from trajectory
  Eigen::Quaterniond targetAnchorQuat = motionTrajectory_.getTargetAnchorQuat();
  // q12 = q10 * q02, where q10 = q01^(-1)
  Eigen::Quaterniond currentQuatInv = currentBaseQuat.inverse();
  Eigen::Quaterniond quatDiff = currentQuatInv * targetAnchorQuat;    
  Eigen::Matrix3d rotMat = quatDiff.toRotationMatrix();    
  // Extract first two columns and reshape row-wise (mat[..., :2].reshape(mat.shape[0], -1))
  Eigen::VectorXd oriDiff(6);
  oriDiff << 
      rotMat(0, 0), rotMat(0, 1),
      rotMat(1, 0), rotMat(1, 1),
      rotMat(2, 0), rotMat(2, 1);
  return oriDiff;
}

/*****************************************倒地起身*****************************************************************/ 


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



  void humanoidController::waistCmdCallback(const kuavo_msgs::robotWaistControl::ConstPtr &msg)
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

  
  // ==================== MPC-RL插值系统实现 ====================
  // target_torso_pose顺序：xyz+rpy
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
    target_position_torso.head(2) = current_torso_pose.head(2);

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

    vector_t currentLegJointAngles = currentObservation_.state.segment(12, jointNumReal_);
    vector_t targetLegJointAngles = initial_status_.segment(12, jointNumReal_);

    leg_interpolation_start_pose_ = torso_position_interpolator_ptr_->getlegJointAngles(currentObservation_.state, current_torso_pose, currentLegJointAngles);
    leg_interpolation_target_pose_ = torso_position_interpolator_ptr_->getlegJointAngles(currentObservation_.state, target_torso_pose, targetLegJointAngles);

    leg_interpolation_result_.setZero(waistNum_ + jointNumReal_);
    leg_interpolation_result_.head(jointNumReal_) = leg_interpolation_start_pose_;

    double leg_distance = 0.0;
    if (leg_interpolation_start_pose_.size() == leg_interpolation_target_pose_.size())
    {
      leg_distance = (leg_interpolation_target_pose_ - leg_interpolation_start_pose_).norm();
    }
    else
    {
      std::cout << "[MPCRLInterpolation] 错误：下肢位置维度不匹配(" 
                << leg_interpolation_start_pose_.size() << " vs " << leg_interpolation_target_pose_.size() << ")" << std::endl;
    }

    // torso_interpolation_target_pose_.head(2) = current_torso_pose.head(2);
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
    // last_interpolation_time_ = current_time;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Starting MPC-RL interpolation:" << std::endl;
    std::cout << "  Torso from [" << current_torso_pose.transpose() 
              << "] to [" << target_torso_pose.transpose() << "] (distance: " << torso_distance << "m)" << std::endl;
    std::cout << "  Leg from [" << leg_interpolation_start_pose_.transpose() 
              << "] to [" << leg_interpolation_target_pose_.transpose() << "] (distance: " << leg_distance << "rad)" << std::endl;
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
    // 计算基于时间的插值进度
    double elapsed_time = current_time - torso_interpolation_start_time_;
    double alpha = std::min(1.0, elapsed_time / torso_interpolation_duration_);

    // 如果已经到达目标位置
    if (current_time - torso_interpolation_start_time_ >= torso_interpolation_duration_+0.1 && 
      (distance_to_target < switch_distance_threshold_ || current_time - torso_interpolation_start_time_ > torso_interpolation_duration_ + switch_timeout_base_threshold_))
    {
      is_torso_interpolation_active_ = false;
      is_arm_interpolating_ = false;
      std::cout << "MPC-RL interpolation complete，elapsed_time: " << current_time - torso_interpolation_start_time_  << "s, duration: " << torso_interpolation_duration_ << "s" << std::endl;
      return;
    }
    
    // 使用线性插值计算当前躯干位姿
    vector6_t interpolated_pose = torso_interpolation_start_pose_ + alpha * (torso_interpolation_target_pose_ - torso_interpolation_start_pose_);
    leg_interpolation_result_.head(jointNumReal_) = leg_interpolation_start_pose_ + alpha * (leg_interpolation_target_pose_ - leg_interpolation_start_pose_);

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
    twist_msg.linear.z = torso_interpolation_target_pose_(2) - default_state_[8];  // z位置与基准高度的差值
    // 使用插值后的rpy
    twist_msg.angular.x = torso_interpolation_target_pose_(3); // roll
    twist_msg.angular.y = torso_interpolation_target_pose_(4); // pitch
    twist_msg.angular.z = torso_interpolation_target_pose_(5); // yaw、、
    
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
                << "s, distance_to_target: " << distance_to_target << "m" << std::endl;
      std::cout << "torso_interpolation_target_pose_: " << torso_interpolation_target_pose_.transpose() << std::endl;
      std::cout << "arm_interpolated_pos: " << arm_interpolation_result_.transpose() << std::endl;
      last_debug_time = current_time;
    }
  }

  void humanoidController::waitForNextCycle()
  {
    // 根据当前控制模式选择对应的频率控制
    if (is_rl_controller_ && current_controller_ptr_ != nullptr)
    {
      // RL 模式：委托给当前 RL 控制器的 waitForNextCycle
      current_controller_ptr_->waitForNextCycle();
    }
    else
    {
      // MPC 模式：使用自身的 wbc_rate_
      wbc_rate_->sleep();
    }
  }

  double humanoidController::getControlFrequency() const
  {
    // 根据当前控制模式返回对应的控制频率
    if (is_rl_controller_ && current_controller_ptr_ != nullptr)
    {
      // RL 模式：返回当前 RL 控制器的控制频率
      return current_controller_ptr_->getControlFrequency();
    }
    else
    {
      // MPC 模式：返回 WBC 频率
      return wbc_frequency_;
    }
  }

 } // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)

