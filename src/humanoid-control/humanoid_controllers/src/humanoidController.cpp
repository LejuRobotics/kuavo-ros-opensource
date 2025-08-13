#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "std_srvs/SetBool.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <fstream>

#include "humanoid_controllers/humanoidController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <humanoid_interface_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
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
#include "humanoid_interface_drake/humanoid_interface_drake.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;
  std::mutex head_mtx;


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
    auto &motor_info = kuavo_settings_.hardware_settings;
    headNum_ = motor_info.num_head_joints;
    armNumReal_ = motor_info.num_arm_joints;
    jointNumReal_ = motor_info.num_joints - headNum_ - armNumReal_;
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
    actuatedDofNumReal_ = jointNumReal_ + armNumReal_ + headNum_;
    ros::param::set("/armRealDof",  static_cast<int>(armNumReal_));
    ros::param::set("/legRealDof",  static_cast<int>(jointNumReal_));
    ros::param::set("/headRealDof",  static_cast<int>(headNum_));

    motor_c2t_ = Eigen::Map<Eigen::VectorXd>(motor_info.c2t_coeff.data(), motor_info.c2t_coeff.size());
    auto [plant, context] = drake_interface_->getPlantAndContext();
    ros_logger_ = new TopicLogger(controller_nh);
    controllerNh_ = controller_nh;
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    std::string gaitCommandFile;
    controllerNh_.getParam("/urdfFile", urdfFile);
    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/referenceFile", referenceFile);
    controllerNh_.getParam("/gaitCommandFile", gaitCommandFile);
    controllerNh_.getParam("/use_external_mpc", use_external_mpc_);
    double controlFrequency = 500.0; // 1000Hz
    controllerNh_.getParam("/wbc_frequency", controlFrequency);
    if(controllerNh_.hasParam("/visualize_humanoid"))
      controllerNh_.getParam("/visualize_humanoid", visualizeHumanoid_);
    dt_ = 1.0 / controlFrequency;
    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
      controllerNh_.getParam("/cali", is_cali_);

    }
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);

    }

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
    // trajectory_publisher_ = new TrajectoryPublisher(controller_nh, 0.001);

    wheel_arm_robot_ = drake_interface_->getKuavoSettings().running_settings.only_half_up_body;
    
    size_t buffer_size = (is_play_back_mode_) ? 20 : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);
    gaitManagerPtr_ = new GaitManager(20);
    gaitManagerPtr_->add(0.0, "stance");
    bool verbose = false;
    loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);
    loadData::loadCppDataType(taskFile, "contact_cst_st", contact_cst_st_);
    loadData::loadCppDataType(taskFile, "contact_cst_et", contact_cst_et_);

#ifdef KUAVO_CONTROL_LIB_FOUND
    joint_filter_ptr_ = new HighlyDynamic::JointFilter(&plant, &kuavo_settings_, 12, dt_, ros_logger_);
#endif
    setupHumanoidInterface(taskFile, urdfFile, referenceFile, gaitCommandFile, verbose, rb_version);
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
    armNum_ = info.actuatedDofNum - jointNum_;


    if (armNumReal_ + jointNumReal_ != jointNum_ + armNum_) // mpc维度和实际维度不一致，简化的模型
    {
      is_simplified_model_ = true;
      std::cout << "[HumanoidController]: using simplified mpc model" << std::endl;
      std::cout << "jointNumReal_:" << jointNumReal_ << " jointNum_:" << jointNum_ << std::endl;
      std::cout << "headNum_:" << headNum_ << std::endl;
      std::cout << "armNumReal_:" << armNumReal_ << " armNum_:" << armNum_<< std::endl;
      armDofMPC_ = armNum_ / 2;
      armDofReal_ = armNumReal_ / 2;
      armDofDiff_ = armDofReal_ - armDofMPC_;
      simplifiedJointPos_ = vector_t::Zero(armDofDiff_*2);
    }
    defalutJointPos_.resize(info.actuatedDofNum);
    sensor_data_head_.resize_joint(headNum_);
    joint_kp_.resize(actuatedDofNumReal_);
    joint_kd_.resize(actuatedDofNumReal_);
    joint_kp_walking_.resize(actuatedDofNumReal_);
    joint_kd_walking_.resize(actuatedDofNumReal_);
    head_kp_.resize(headNum_);
    head_kd_.resize(headNum_);
    // quat_offset_ = Eigen::Quaterniond(1, 0, 0, 0);

    joint_control_modes_ = Eigen::VectorXd::Constant(actuatedDofNumReal_, 2);
    output_tau_ = vector_t::Zero(actuatedDofNumReal_);
    output_pos_ = vector_t::Zero(actuatedDofNumReal_);
    output_vel_ = vector_t::Zero(actuatedDofNumReal_);
    Eigen::Vector3d acc_filter_params;
    Eigen::Vector3d gyro_filter_params;
    double arm_joint_pos_filter_cutoff_freq=20,arm_joint_vel_filter_cutoff_freq=20,mrt_joint_vel_filter_cutoff_freq=200;
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
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
    
    ros::param::set("robot_init_state_param", robot_init_state_param);
  
    ros::param::set("/humanoid/init_q", robot_init_state_param);

    auto initial_state_ =  drake_interface_->getInitialState();
    auto squat_initial_state_ =  drake_interface_->getSquatInitialState();
    std::cout << "controller initial_state_:" << initial_state_.transpose() << std::endl;
    std::cout << "controller squat_initial_state_:" << squat_initial_state_.transpose() << std::endl;
    std::vector<double> initial_state_vector(initial_state_.data(), initial_state_.data() + initial_state_.size());
    std::vector<double> squat_initial_state_vector(squat_initial_state_.data(), squat_initial_state_.data() + squat_initial_state_.size());
    std::vector<double> default_joint_pos_vector(defalutJointPos_.data(), defalutJointPos_.data() + defalutJointPos_.size());
    controllerNh_.setParam("/initial_state", initial_state_vector);
    controllerNh_.setParam("/squat_initial_state", squat_initial_state_vector);
    controllerNh_.setParam("/default_joint_pos", default_joint_pos_vector);

    joint_state_limit_.resize(actuatedDofNumReal_, 2);

    auto robot_config = drake_interface_->getRobotConfig();
    is_swing_arm_ = robot_config->getValue<bool>("swing_arm");
    swing_arm_gain_ = robot_config->getValue<double>("swing_arm_gain");
    swing_elbow_scale_ = robot_config->getValue<double>("swing_elbow_scale");
    ruiwo_motor_velocities_factor_ = robot_config->getValue<double>("motor_velocities_factor");
    gait_map_ = HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getGaitMap();
    std::cout << "gait_map size: " << gait_map_.size() << std::endl;

    loadData::loadEigenMatrix(referenceFile, "joint_kp_", joint_kp_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_", joint_kd_);
    loadData::loadEigenMatrix(referenceFile, "joint_kp_walking_", joint_kp_walking_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_walking_", joint_kd_walking_);
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
    // create a ROS subscriber to receive the joint pos and vel
    jointPos_ = vector_t::Zero(info.actuatedDofNum);
    jointPos_.setZero();
    jointPos_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointPosWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_ );
    // jointPosWBC_.setZero();
    jointPosWBC_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointVelWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_);
    jointAccWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_);
    jointCurrentWBC_ = vector_t::Zero(armNumReal_ + jointNumReal_);

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
    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
    robotLocalizationSub_ = controllerNh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, &humanoidController::robotlocalizationCallback, this);
    mpcStartSub_ = controllerNh_.subscribe<std_msgs::Bool>("/start_mpc", 10, &humanoidController::startMpccallback, this);
    arm_joint_trajectory_.initialize(armNumReal_);
    mm_arm_joint_trajectory_.initialize(armNumReal_);
    arm_joint_traj_sub_ = controllerNh_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, [this](const sensor_msgs::JointState::ConstPtr &msg)
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
      head_sub_ = controllerNh_.subscribe("/robot_head_motion_data", 10, &humanoidController::headCmdCallback, this);
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
      wbc_->loadTasksSetting(taskFile, verbose, is_real_);
      if (only_half_up_body_) {
        wbc_->setHalfBodyMode(true);
      }

      standUpWbc_ = std::make_shared<StandUpWbc>(*pinocchioInterfaceWBCPtr_, centroidalModelInfoWBC_,
                                                 *eeKinematicsWBCPtr_);
      standUpWbc_->setArmNums(armNumReal_);
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

    
    // 设置CPU内核隔离
    setupCpuIsolation();
    
    return true;
  }
  void humanoidController::headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg)
  {
      if (msg->joint_data.size() ==2)
      {
          if (msg->joint_data[0] < head_joint_limits_[0].first || msg->joint_data[0] > head_joint_limits_[0].second 
            || msg->joint_data[1] < head_joint_limits_[1].first || msg->joint_data[1] > head_joint_limits_[1].second)
          {
              std::cout << "\033[1;31m[headCmdCallback] Invalid robot head motion data. Head joints must be in the range [" 
                << head_joint_limits_[0].first << ", " << head_joint_limits_[0].second << "] and [" 
                << head_joint_limits_[1].first << ", " << head_joint_limits_[1].second << "].\033[0m" << std::endl;
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

  void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    auto &joint_data = msg->joint_data;
    auto &imu_data = msg->imu_data;
    auto &end_effector_data = msg->end_effector_data; // TODO: add end_effector_data to the observation
    SensorData sensor_data;
    sensor_msgs::Imu imu_msg;
    sensor_data.resize_joint(jointNumReal_+armNumReal_);
    // JOINT DATA
    for (size_t i = 0; i < jointNumReal_+armNumReal_; ++i)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointTorque_(i) = joint_data.joint_torque[i];
    }
    // std::cout << "received joint data: " << jointPos_.transpose() << std::endl;
    ros::Time ros_time = msg->header.stamp;
    sensor_data.timeStamp_ = msg->sensor_time;
    double sensor_time_diff = (ros::Time::now() - ros_time).toSec() * 1000;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);
    // IMU
    sensor_data.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
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
    // free_acc_filter_.update(sensor_data.linearAccel_);
    // END_EFFECTOR DATA
    // sensor_data_mutex_.lock();
    // sensorDataQueue.push(sensor_data);
    // sensor_data_mutex_.unlock();
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);

    if (headNum_ > 0 && joint_data.joint_q.size() == jointNumReal_+armNumReal_ + headNum_)
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
        mm_arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_, armNumReal_);
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
      mm_arm_joint_trajectory_.pos = currentObservationWBC_.state.segment(12 + jointNumReal_, armNumReal_);
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
    std::cout << "initial state: " << currentObservation_.state.transpose() << std::endl;
    optimizedInput2WBC_mrt_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    optimizedInput2WBC_mrt_.head(centroidalModelInfo_.inputDim) = currentObservation_.input;

    currentObservation_.mode = ModeNumber::SS;
    // List: 原先的 mpc 初始化位置修改到 preUpdate
    // SystemObservation initial_observation = currentObservation_;
    // initial_observation.state = initial_status_;
    // TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});

    // // Set the first observation and command and wait for optimization to finish
    // ROS_INFO_STREAM("Waiting for the initial policy ...");
    // if (use_external_mpc_)
    // {
    //   // Reset MPC node
    //   mrtRosInterface_->resetMpcNode(target_trajectories);
    //   std::cout << "reset MPC node\n";
    //   // Wait for the initial policy
    //   while (!mrtRosInterface_->initialPolicyReceived() && ros::ok() && ros::master::check())
    //   {
    //     mrtRosInterface_->spinMRT();
    //     mrtRosInterface_->setCurrentObservation(initial_observation);
    //     ros::Rate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    //   }
    // }
    // else
    // {
    //   mpcMrtInterface_->setCurrentObservation(initial_observation);
    //   mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    //   ROS_INFO_STREAM("Waiting for the initial policy ...");
    //   while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    //   {
    //     mpcMrtInterface_->advanceMpc();
    //     ros::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    //   }
    // }

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
        optimizedState2WBC_mrt_.segment(12 + jointNum_ + i * armDofReal_, armDofMPC_) = optimizedState2WBC_mrt_.segment(12 + jointNum_ + i * armDofMPC_, armDofMPC_);
      }
    }
    currentObservationWBC_ = currentObservation_;
    currentObservationWBC_.state = optimizedState2WBC_mrt_;
    currentObservationWBC_.input = optimizedInput2WBC_mrt_;

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
    /*******************输入蹲姿和站姿**********************/
    auto &infoWBC = centroidalModelInfoWBC_;
    vector_t squatState = vector_t::Zero(infoWBC.stateDim);
    squatState.head(12 + jointNum_) = drake_interface_->getSquatInitialState();
    vector_t standState = vector_t::Zero(infoWBC.stateDim);
    standState.head(12 + jointNum_) = drake_interface_->getInitialState();
    /*******采用 standUp_controller 从蹲姿运动到站姿*********/
    stateEstimate_->setFixFeetHeights(true);
    updateStateEstimation(time, false);

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
    for (int i2 = 0; i2 < armNumReal_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(output_pos_(jointNumReal_ + i2));
      jointCmdMsg.joint_v.push_back(output_vel_(jointNumReal_ + i2));
      jointCmdMsg.tau.push_back(output_tau_(jointNumReal_ + i2));
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[jointNumReal_+i2]);
      jointCmdMsg.control_modes.push_back(joint_control_modes_[jointNumReal_+i2]);
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
      jointCmdMsg.joint_kp.push_back(0);
      jointCmdMsg.joint_kd.push_back(0);
    }
    // if (use_shm_communication_) 
    //     publishJointCmdToShm(jointCmdMsg);
    jointCmdPub_.publish(jointCmdMsg);

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

    /*******************超过设置时间，退出******************/
    // 延迟启动, 避免切换不稳定
    if (time.toSec() > robotStandUpCompleteTime_ + 0.8 || !is_real_)
    {
      SystemObservation initial_observation = currentObservation_;
      initial_observation.state = initial_status_;
      TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});

      // Set the first observation and command and wait for optimization to finish
      ROS_INFO_STREAM("Waiting for the initial policy ...");
      if (use_external_mpc_)
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
      else
      {
        mpcMrtInterface_->setCurrentObservation(initial_observation);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
        ROS_INFO_STREAM("Waiting for the initial policy ...");
        while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
        {
          mpcMrtInterface_->advanceMpc();
          ros::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
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
    const auto t1 = Clock::now();

    // 使用共享内存更新传感器数据
    if (use_shm_communication_) {
        updateSensorDataFromShm();
    }
    if (reset_mpc_) // 重置mpc
    {
      currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
      auto target_trajectories = TargetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
      mrtRosInterface_->resetMpcNode(target_trajectories);
      reset_mpc_ = false;
      std::cout << "reset MPC node at " << currentObservation_.time << "\n";
    }
    // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getNextData();
    // // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
    // applySensorsData(msg);
    // State Estimate
    ros::Duration period = ros::Duration(dt_);
    updateStateEstimation(time, false);
    const auto t2 = Clock::now();

    auto& info = centroidalModelInfo_;
    auto& infoWBC = centroidalModelInfoWBC_;

    vector_t optimizedState_mrt, optimizedInput_mrt;
    bool is_mpc_updated = false;
    if (use_external_mpc_)
    {
      // Only use halfup_body doesn't work well.
      if (!only_half_up_body_) {
      // Update the current state of the system
      mrtRosInterface_->setCurrentObservation(currentObservation_);
      
      // Trigger MRT callbacks
      mrtRosInterface_->spinMRT();
      // Update the policy if a new on was received
      if (mrtRosInterface_->updatePolicy())
      {
        is_mpc_updated = true;
        auto &policy = mrtRosInterface_->getPolicy();
        auto &state_trajectory = policy.stateTrajectory_;
        auto &command = mrtRosInterface_->getCommand();
        // checkMpcPullUp(currentObservation_.time, currentObservation_.state, command.mpcTargetTrajectories_);
        // trajectory_publisher_->publishTrajectory(state_trajectory);
        TargetTrajectories target_trajectories(policy.timeTrajectory_, policy.stateTrajectory_, policy.inputTrajectory_);

        publishFeetTrajectory(target_trajectories);
      }

      mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      }
    }
    else
    {
      // Update the current state of the system
      mpcMrtInterface_->setCurrentObservation(currentObservation_);

      // Load the latest MPC policy
      if (mpcMrtInterface_->updatePolicy())
      {
        is_mpc_updated = true;
        auto &policy = mpcMrtInterface_->getPolicy();
        auto &command = mpcMrtInterface_->getCommand();
        auto &performance_indices = mpcMrtInterface_->getPerformanceIndices();
        auto &state_trajectory = policy.stateTrajectory_;

        ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
            createMpcPolicyMsg(policy, command, performance_indices);

        // publish the message
        mpcPolicyPublisher_.publish(mpcPolicyMsg);

      }

      // Evaluate the current policy
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
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
    if (use_ros_arm_joint_trajectory_)
    {
      // TODO: feedback in planner
      // auto arm_pos = currentObservation_.state.tail(armNum_); 
      // optimizedInput2WBC_mrt_.tail(armNum_) = 0.05 * (arm_joint_trajectory_.pos - arm_pos)/dt_;
      // optimizedState2WBC_mrt_.tail(armNum_) = arm_pos + optimizedInput2WBC_mrt_.tail(armNum_) * dt_;
      if (only_half_up_body_) 
      {
          optimizedState2WBC_mrt_.segment<7>(24) = arm_joint_trajectory_.pos.segment<7>(0);
          optimizedState2WBC_mrt_.segment<7>(24+7) = arm_joint_trajectory_.pos.segment<7>(7);
      }
      else {
          // 位置、速度
          optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_trajectory_.pos;
          // optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_trajectory_.vel;//TODO: 临时去除
          // std::cout << "target_arm_joint_pos: " << arm_joint_trajectory_.pos.transpose() << std::endl;
      }
      // std::cout << "target_arm_joint_pos[0]: " << arm_joint_trajectory_.pos[0] << std::endl;
    }
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
      else {
          // 位置、速度
          optimizedState2WBC_mrt_.tail(armNumReal_) = mm_arm_joint_trajectory_.pos;
      }
    }


    // // use filter output
    optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_pos_filter_.update(optimizedState2WBC_mrt_.tail(armNumReal_));
    optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(optimizedInput2WBC_mrt_.tail(armNumReal_));
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
    const auto t3 = Clock::now();
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
    
    ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt/joint_vel", optimizedInput2WBC_mrt_.segment(3 * infoWBC.numThreeDofContacts, infoWBC.actuatedDofNum));
    
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/com/linear_vel_xyz", optimizedState2WBC_mrt_.head<3>());
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/com/angular_vel_xyz", optimizedState2WBC_mrt_.segment<3>(3));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/base/pos_xyz", optimizedState2WBC_mrt_.segment<3>(6));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/base/angular_zyx", optimizedState2WBC_mrt_.segment<3>(9));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/joint_pos", optimizedState2WBC_mrt_.segment(12, infoWBC.actuatedDofNum));
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

    std::chrono::time_point<std::chrono::high_resolution_clock> t4;
    if (enable_wbc)
    {
      vector_t x = wbc_->update(optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, measuredRbdStateReal_, plannedMode_, period.toSec(), is_mpc_updated);

      // wbc_->updateVd(jointAcc_);
      t4 = Clock::now();
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
        // TODO: send the stop command to hardware interface
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


    const auto t5 = Clock::now();

    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    // std::cout << "jointNum_:  " << jointNum_+armNum_ << "\n\n";
    for (int i1 = 0; i1 < jointNumReal_; ++i1)
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

    ModeSchedule current_mode_schedule = mrtRosInterface_->getCurrentModeSchedule();
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
        if (i == 4 || i == 5 || i == 10 || i == 11) // 踝关节
        {
          jointCmdMsg.control_modes[i] = 0;
          jointCmdMsg.tau[i] = 0;
          jointCmdMsg.joint_kp[i] = 0;
          jointCmdMsg.joint_kd[i] = 0;
        }
        else
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
      jointCmdMsg.joint_q.push_back(output_pos_(jointNum_+i2));
      jointCmdMsg.joint_v.push_back(output_vel_(jointNum_+i2));
      jointCmdMsg.tau.push_back(output_tau_(jointNum_+i2));
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[jointNum_+i2]);
      jointCmdMsg.control_modes.push_back(joint_control_modes_[jointNum_+i2]);
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
        double head_limit_vel = hardware_settings.joint_velocity_limits[jointNum_ + armNumReal_ + i3];

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

    // 发布控制命令
    if (use_shm_communication_) 
        publishJointCmdToShm(jointCmdMsg);
    jointCmdPub_.publish(jointCmdMsg);
    
    // Visualization
    if (visualizeHumanoid_)
    {
      robotVisualizer_->updateSimplifiedArmPositions(simplifiedJointPos_);
      if (use_external_mpc_)
        robotVisualizer_->update(currentObservation_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());
      else
        robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

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
    lastObservation_ = currentObservation_;

     //Publish the observation. Only needed for the command interface
    const auto t6 = Clock::now();
    auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count();
    if (time_cost > timeout_warning_ms_)
    {
      std::cerr << "humanoidController::controllerLoop time cost: "<< time_cost << " ms" << std::endl;
      std::cerr << "t1-t2: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
      std::cerr << "t2-t3: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;
      std::cerr << "t3-t4: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms" << std::endl;
      std::cerr << "t4-t5: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << " ms" << std::endl;
      std::cerr << "t5-t6: " << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count() << " ms" << std::endl;
    }
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
      jointPos_.head(jointNum_) = data.jointPos_.head(jointNum_);
      jointVel_.head(jointNum_) = data.jointVel_.head(jointNum_);
      jointAcc_.head(jointNum_) = data.jointAcc_.head(jointNum_);
      jointTorque_.head(jointNum_) = data.jointTorque_.head(jointNum_);

      for (int i = 0; i < 2; i++)
      {

        jointPos_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointPos_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointVel_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointVel_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointAcc_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointAcc_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointTorque_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointTorque_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        simplifiedJointPos_.segment(armDofDiff_ * i, armDofDiff_) = data.jointPos_.segment(jointNum_ + armDofReal_ * i + armDofMPC_, armDofDiff_);
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
  }
  void humanoidController::updateStateEstimation(const ros::Time &time, bool is_init)
  {
    if (reset_estimator_)
    {
      stateEstimate_->reset();
      reset_estimator_ = false;
    }
    // vector_t jointPos(jointNum_+armNum_), jointVel(jointNum_+armNum_), jointCurrent(jointNum_+armNum_);
    // contact_flag_t contacts;
    // Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag;
    // vector3_t angularVel, linearAccel;
    // matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
    SensorData sensors_data;
    if (is_init)
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    else
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    // SensorData &sensors_data = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
    applySensorData(sensors_data);

    // TODO: get contactFlag from hardware interface
    // 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
    // contactFlag = modeNumber2StanceLeg(plannedMode_);
    if (is_init)
    {
      last_time_ = current_time_ - ros::Duration(0.001);
      stateEstimate_->updateJointStates(jointPos_, jointVel_);
      quat_init = stateEstimate_->updateIntialEulerAngles(quat_);
      applySensorData(sensors_data);
      stateEstimate_->set_intial_state(currentObservation_.state);
      measuredRbdState_ = stateEstimate_->getRbdState();
      // std::cout << "initial measuredRbdState_:" << measuredRbdState_.transpose() << std::endl;
      // clear the sensor data queue
      // sensor_data_mutex_.lock();
      // while (!sensorDataQueue.empty())
      //   sensorDataQueue.pop();
      // sensor_data_mutex_.unlock();
    }
    // last_time_ = current_time_ - ros::Duration(0.002);
    double diff_time = (current_time_ - last_time_).toSec();
    // auto est_mode = stateEstimate_->ContactDetection(plannedMode_, jointVel_, jointTorque_, diff_time);
    // ros_logger_->publishValue("/state_estimate/mode", static_cast<double>(est_mode));
    // est_mode = plannedMode_;
    // contactFlag = modeNumber2StanceLeg(est_mode);
    // std::cout << "mode: " << modeNumber2String(est_mode) << std::endl;
    last_time_ = current_time_;
    ros::Duration period = ros::Duration(diff_time);
    // std::cout << "diff_time: " << diff_time << std::endl;
    // std::cout << "diff_time: " << period.toSec() << std::endl;

    vector_t activeTorque_ = jointTorque_;
    vector_t activeTorqueWBC_ =  jointCurrentWBC_;
    stateEstimate_->setCmdTorque(activeTorque_);
    stateEstimate_->estContactForce(period);
    auto est_contact_force = stateEstimate_->getEstContactForce();
    contactForce_ = est_contact_force;
    ros_logger_->publishVector("/state_estimate/Contact_Detection/contactForce", est_contact_force);
    auto est_mode = stateEstimate_->ContactDetection(nextMode_, is_stance_mode_, plannedMode_, robotMass_, est_contact_force(2), est_contact_force(8), diff_time);
    ros_logger_->publishValue("/state_estimate/Contact_Detection/mode", static_cast<double>(est_mode));
    if (!use_estimator_contact_)
    {
      est_mode = plannedMode_;
    }
    estPlannedMode_ = est_mode;
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
    bool new_pull_up_state = false;
     // 只有非半身轮臂模式站立状态&&站起来稳定之后进行保护, 并且手臂不是外部遥操作模式才可触发拉起保护
    if (enable_pull_up_protect_ && isPreUpdateComplete && is_stance_mode_ && 
    !only_half_up_body_ && currentObservation_.time - standupTime_ > 4 
    && mpcArmControlMode_ != ArmControlMode::EXTERN_CONTROL)
      new_pull_up_state = stateEstimate_->checkPullUp(pull_up_force_threshold_);
    ros_logger_->publishValue("/state_estimate/pull_up_state", new_pull_up_state);
    if (new_pull_up_state && !isPullUp_)
    {
      ROS_WARN_STREAM("Pull up detected");
      isPullUp_ = true;
      setPullUpState_=true;
    }
    // else if (!new_pull_up_state && isPullUp_)// TODO:拉起之后重新站立
    // {
    //   ROS_WARN_STREAM("Pull up end");
    //   isPullUp_ = false;
    // }
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
        measuredRbdStateReal_.segment(centroidalModelInfoWBC_.generalizedCoordinatesNum * i, 6 + jointNum_) =
            measuredRbdState_.segment(info.generalizedCoordinatesNum * i, 6 + jointNum_);

        // 共有的手臂关节
        int arm_start_index = centroidalModelInfoWBC_.generalizedCoordinatesNum * i + 6 + jointNum_;
        int arm_start_index_mpc = info.generalizedCoordinatesNum * i + 6 + jointNum_ ;
        for (int j = 0; j < 2; j++) // 左右手
        {
          measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j, armDofMPC_) =
              measuredRbdState_.segment(arm_start_index_mpc + armDofMPC_ * j, armDofMPC_);

          // 简化的手臂关节部分从传感器数据获取
          vector_t joint_qv(sensors_data.jointPos_.size() * 2);
          joint_qv << sensors_data.jointPos_, sensors_data.jointVel_;
          int sensors_joint_num = sensors_data.jointPos_.size();
          measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j + armDofMPC_, armDofDiff_) =
              joint_qv.segment(sensors_joint_num * i + jointNum_ + armDofReal_ * j + armDofMPC_, armDofDiff_);
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
      int arm_start_index = 6 + jointNum_;
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

    // std::cout << "jointPosWBC_:" << jointPosWBC_.transpose() << std::endl;

    auto est_arm_contact_force = stateEstimate_->getEstArmContactForce(jointPosWBC_, jointVelWBC_, activeTorqueWBC_, period);
    ros_logger_->publishVector("/state_estimate/est_arm_contact_force", est_arm_contact_force);
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

    vector_t defaultJointState(pinocchioInterfaceWBCPtr_->getModel().nq - 6);
    defaultJointState.setZero();
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
    defaultJointState.head(jointNum_) = drake_interface_->getDefaultJointState();

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
    if (use_external_mpc_)
    {
      mrtRosInterface_ = std::make_shared<MRT_ROS_Interface>(robotName_);
      mrtRosInterface_->launchNodes(controllerNh_);
      return;
    }
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&HumanoidInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]()
                             {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            HumanoidInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        //TODO: send the stop command to hardware interface
      }
    } });
    setThreadPriority(HumanoidInterface_->sqpSettings().threadPriority, mpcThread_);
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
        SensorData sensor_data;
        sensor_data.resize_joint(jointNumReal_+armNumReal_);
        
        // 关节数据
        for (size_t i = 0; i < jointNumReal_+armNumReal_; ++i) {
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
        for (size_t i = 0; i < jointNumReal_+armNumReal_; ++i) {
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
        if (headNum_ > 0 && sensors_data.num_joints == jointNumReal_+armNumReal_+headNum_) {
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
    joint_cmd.num_joints = jointNumReal_ + armNumReal_ + headNum_;

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

  void humanoidController::setupCpuIsolation()
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
      return;
    }
    
    // 检查是否有隔离的核心
    if (isolated_cpus.size() >= 1) {
      // 检查CPU核心编号是否有效
      int max_cpu = sysconf(_SC_NPROCESSORS_ONLN);
      std::cout << "系统CPU核心数: " << max_cpu << std::endl;
      
      for (size_t i = 0; i < isolated_cpus.size(); ++i) {
        if (isolated_cpus[i] < 0 || isolated_cpus[i] >= max_cpu) {
          std::cerr << "警告: CPU核心 " << isolated_cpus[i] << " 超出有效范围 [0, " << max_cpu-1 << "]" << std::endl;
          return;
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
    } else {
      std::cout << "隔离的核心列表为空，跳过CPU亲和性设置" << std::endl;
      return;
    }

    // 只有在有真正隔离的CPU时才设置亲和性
    if (actually_isolated_cpus.size() >= 1) {
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
      } else {
        std::cout << "成功设置CPU亲和性到隔离核心" << std::endl;
      }
    } else {
      std::cout << "没有真正隔离的CPU核心，跳过CPU亲和性设置" << std::endl;
    }
  }

} // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)

