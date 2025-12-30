#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "std_srvs/SetBool.h"
#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "humanoid_controllers/humanoidController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
// #include <humanoid_dummy/gait/GaitReceiver.h>
#include "humanoid_interface_ros/gait/GaitReceiver.h"

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <std_srvs/Trigger.h>

#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>

#include <humanoid_wbc/WeightedWbc.h>
#include <humanoid_wbc/HierarchicalWbc.h>
#include "humanoid_interface_drake/kuavo_data_buffer.h"
#include "kuavo_common/common/sensor_data.h"
#include "kuavo_common/common/utils.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;
  std::mutex g_obs_mutex;
  static void mujocoSimStart(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 等待服务可用
    bool service_available = ros::service::waitForService("sim_start", ros::Duration(5.0)); // 5秒超时

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
    ros::NodeHandle nh;
    stop_pub_ = nh.advertise<std_msgs::Bool>("/stop_robot", 10);

    char Walk_Command = '\0';
    while (ros::ok())
    {
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
        else if (Walk_Command == 'v')
        {
          std::cout << "v" << std::endl;
          control_mode_ = (control_mode_ == 1) ? 2 : 1;
          std::cout << "control mode:" << control_mode_ << std::endl;
        }
        else if (Walk_Command == 't')
        {
          control_mode_ = (control_mode_ == 0) ? 2 : 0;
          std::cout << "control mode:" << control_mode_ << std::endl;
        }

        Walk_Command = '\0';
      }
      usleep(50000);
    }
  }

  bool humanoidController::init(HybridJointInterface *robot_hw, ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    int robot_version_int;
    RobotVersion robot_version(3, 4);
    if (controllerNh_.hasParam("/robot_version"))
    {
        controllerNh_.getParam("/robot_version", robot_version_int);
        int major = robot_version_int / 10;
        int minor = robot_version_int % 10;
        robot_version = RobotVersion(major, minor);
    }
    is_nodelet_node_ = is_nodelet_node;
    drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
    kuavo_settings_ = drake_interface_->getKuavoSettings();
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
    dt_ = 1.0 / controlFrequency;
    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
      controllerNh_.getParam("/cali", is_cali_);
      if (is_real_)
      {
        std::cout << "real robot controller" << std::endl;
        ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(controllerNh_);
      }
    }
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
      std::cout << "get param wbc_only: " << wbc_only_ << std::endl;
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);

      std::cout << "get param play_back: " << is_play_back_mode_ << std::endl;
    }

    if (controllerNh_.hasParam("use_joint_filter"))
    {
      controllerNh_.getParam("/use_joint_filter", use_joint_filter_);
    }

    // trajectory_publisher_ = new TrajectoryPublisher(controller_nh, 0.001);
    size_t buffer_size = (is_play_back_mode_) ? 20 : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);
    
    setupHumanoidInterface(taskFile, urdfFile, referenceFile, gaitCommandFile, verbose,robot_version_int);
    setupMpc();
    setupMrt();
    // Visualization
    ros::NodeHandle nh;
    CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
    std::cout << "HumanoidInterface_->getCentroidalModelInfo().robotMass:" << HumanoidInterface_->getCentroidalModelInfo().robotMass << std::endl;
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        HumanoidInterface_->modelSettings().contactNames3DoF);
    eeSpatialKinematicsPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                                      HumanoidInterface_->modelSettings().contactNames6DoF);
    robotVisualizer_ = std::make_shared<HumanoidVisualizer>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(),
                                                            *eeKinematicsPtr_, *eeSpatialKinematicsPtr_, nh, taskFile);

    pinocchioInterface_ptr_ = new PinocchioInterface(HumanoidInterface_->getPinocchioInterface());
    centroidalModelInfo_ = HumanoidInterface_->getCentroidalModelInfo();
    eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_);

    defalutJointPos_.resize(jointNum_);
    joint_kp_.resize(jointNum_);
    joint_kd_.resize(jointNum_);
    joint_kp_walking_.resize(jointNum_);
    joint_kd_walking_.resize(jointNum_);
    output_tau_.resize(jointNum_);
    output_tau_.setZero();
    Eigen::Vector3d acc_filter_params;
    Eigen::Vector3d gyro_filter_params;
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(RobotVersion(3, 4), true, 2e-3);
    defalutJointPos_ = drake_interface_->getDefaultJointState();

    auto robot_config = drake_interface_->getRobotConfig();
    is_swing_arm_ = robot_config->getValue<bool>("swing_arm");
    swing_arm_gain_ = robot_config->getValue<double>("swing_arm_gain");
    swing_elbow_scale_ = robot_config->getValue<double>("swing_elbow_scale");
    gait_map_ = HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getGaitMap();
    std::cout << "gait_map size: " << gait_map_.size() << std::endl;

    loadData::loadEigenMatrix(referenceFile, "joint_kp_", joint_kp_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_", joint_kd_);
    loadData::loadEigenMatrix(referenceFile, "joint_kp_walking_", joint_kp_walking_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_walking_", joint_kd_walking_);
    loadData::loadEigenMatrix(referenceFile, "acc_filter_cutoff_freq", acc_filter_params);
    loadData::loadEigenMatrix(referenceFile, "gyro_filter_cutoff_freq", gyro_filter_params);
    // real_initial_start_service_ = nh.advertiseService("/humanoid_controller/real_initial_start", &humanoidController::realIntialStartCallback, this);
    // Hardware interface
    // TODO: setup hardware controller interface
    // create a ROS subscriber to receive the joint pos and vel
    jointPos_ = vector_t::Zero(jointNum_);
    // set jointPos_ to {0, 0, 0.35, -0.90, -0.55, 0, 0, 0, 0.35, -0.90, -0.55, 0}
    jointPos_ << -0.01867, -0.00196, -0.65815, 0.86691, -0.31346, 0.01878, 0.01868, 0.00197, -0.65815, 0.86692, -0.31347, -0.01878;
    jointVel_ = vector_t::Zero(jointNum_);
    jointAcc_ = vector_t::Zero(jointNum_);
    quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);

    acc_filter_.setParams(dt_, acc_filter_params);
    // free_acc_filter_.setParams(dt_, acc_filter_params);
    gyro_filter_.setParams(dt_, gyro_filter_params);
    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
    mpcStartSub_ = controllerNh_.subscribe<std_msgs::Bool>("/start_mpc", 10, &humanoidController::startMpccallback, this);
    gait_scheduler_sub_ = controllerNh_.subscribe<kuavo_msgs::gaitTimeName>(robotName_ + "_mpc_gait_time_name", 10, [this](const kuavo_msgs::gaitTimeName::ConstPtr &msg)
                                                                            {
                                                                              last_gait_ = current_gait_;
            current_gait_.name = msg->gait_name;
            current_gait_.startTime = msg->start_time;
            std::cout << "current gait name: " << current_gait_.name << " start time: " << current_gait_.startTime << std::endl; });
    // jointPosVelSub_ = controllerNh_.subscribe<std_msgs::Float32MultiArray>("/jointsPosVel", 10, &humanoidController::jointStateCallback, this);
    // jointAccSub_ = controllerNh_.subscribe<std_msgs::Float32MultiArray>("/jointsAcc", 10, &humanoidController::jointAccCallback, this);
    // imuSub_ = controllerNh_.subscribe<sensor_msgs::Imu>("/imu", 10, &humanoidController::ImuCallback, this);
    jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    mpcPolicyPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);
    feettargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("/humanoid_controller/feet_target_policys", 10, true);

    // targetPosPub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetPos", 10);
    // targetVelPub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetVel", 10);
    // targetKpPub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetKp", 10);
    // targetKdPub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/targetKd", 10);
    // RbdStatePub_ = controllerNh_.advertise<std_msgs::Float32MultiArray>("/RbdState", 10);
    wbcFrequencyPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/frequency/wbc", 10);
    wbcTimeCostPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/time_cost/wbc", 10);
    if (is_play_back_mode_)
      observation_sub_ = controllerNh_.subscribe<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 10, [this](const ocs2_msgs::mpc_observation::ConstPtr &msg)
                                                                             {
                                                                              std::lock_guard<std::mutex> lock(g_obs_mutex);
                                                                              currentObservation_ = ros_msg_conversions::readObservationMsg(*msg); });
    // State estimation
    setupStateEstimate(taskFile, verbose);
    std::cout << "waiting for msg, please use rosbag play to replay the data" << std::endl;
    sensors_data_buffer_ptr_->waitForReady();
    // std::cout << "waitForReady estimate ready" << std::endl;
    // Whole body control/HierarchicalWbc/WeightedWbc
    wbc_ = std::make_shared<WeightedWbc>(HumanoidInterface_->getPinocchioInterface(), HumanoidInterface_->getCentroidalModelInfo(),
                                         *eeKinematicsPtr_);
    wbc_->loadTasksSetting(taskFile, verbose, is_real_);

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(HumanoidInterface_->getCentroidalModelInfo());
    keyboardThread_ = std::thread(&humanoidController::keyboard_thread_func, this);
    if (!keyboardThread_.joinable())
    {
      std::cerr << "Failed to start keyboard thread" << std::endl;
      exit(1);
    }

    return true;
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

  void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    auto &joint_data = msg->joint_data;
    auto &imu_data = msg->imu_data;
    auto &end_effector_data = msg->end_effector_data; // TODO: add end_effector_data to the observation
    SensorData sensor_data;
    sensor_data.resize_joint(jointNum_);
    // JOINT DATA
    for (size_t i = 0; i < jointNum_; ++i)
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

    sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
    // free_acc_filter_.update(sensor_data.linearAccel_);
    sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
    // END_EFFECTOR DATA
    // sensor_data_mutex_.lock();
    // sensorDataQueue.push(sensor_data);
    // sensor_data_mutex_.unlock();
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);
    if (!is_initialized_)
      is_initialized_ = true;
  }
  

  void humanoidController::starting(const ros::Time &time)
  {
    // Initial state
    // set the initial state = {0, 0, 0, 0, 0, 0, 0, 0, 0.976, 0, 0, 0, 0, 0, 0.35, -0.90, -0.55, 0, 0, 0, 0.35, -0.90, -0.55, 0}
    // currentObservation_.state = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().stateDim);
    // currentObservation_.state(8) = 0.78626;
    // currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;
    initial_status_ = HumanoidInterface_->getInitialState();
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
      SensorData_t intial_sensor_data;
      std::cout << "real robot controller starting\n";
      real_init_wait();
      std::cout << "real_init_wait done\n";
    }

    // applySensorsData(sensors_data_buffer_ptr_->getLastData());

    last_time_ = current_time_;
    if (!is_play_back_mode_)
      updateStateEstimation(time, true);
    currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
    optimizedState2WBC_mrt_ = currentObservation_.state;
    std::cout << "initial state: " << currentObservation_.state.transpose() << std::endl;
    optimizedInput2WBC_mrt_ = currentObservation_.input;

    currentObservation_.mode = ModeNumber::SS;
    SystemObservation initial_observation = currentObservation_;
    initial_observation.state = initial_status_;
    TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});

    // Set the first observation and command and wait for optimization to finish
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    {

      // Reset MPC node
      // mrtRosInterface_->resetMpcNode(target_trajectories);
      // Wait for the initial policy
      while (!mrtRosInterface_->updatePolicy() && ros::ok() && ros::master::check())
      {
        mrtRosInterface_->spinMRT();
        mrtRosInterface_->setCurrentObservation(initial_observation);
        ros::Rate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
      }
    }

    intail_input_ = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
    for (int i = 0; i < 8; i++)
      intail_input_(3 * i + 2) = HumanoidInterface_->getCentroidalModelInfo().robotMass * 9.81 / 8; // 48.7*g/8
    optimizedInput2WBC_mrt_ = intail_input_;
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
      mujocoSimStart(controllerNh_);
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
  void humanoidController::update(const ros::Time &time, const ros::Duration &dfd)
  {
    std::lock_guard<std::mutex> lock(g_obs_mutex);

    const auto t1 = Clock::now();
    // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getNextData();
    // // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
    // applySensorsData(msg);
    // State Estimate
    ros::Duration period = ros::Duration(dt_);
    const auto t2 = Clock::now();
    Eigen::VectorXd joint_control_modes = Eigen::VectorXd::Constant(jointNum_, control_mode_); // init control mode
    vector_t optimizedState_mrt, optimizedInput_mrt;
    bool is_mpc_updated = false;

    // Update the current state of the system
    // mrtRosInterface_->setCurrentObservation(currentObservation_);
    // Trigger MRT callbacks
    mrtRosInterface_->spinMRT();
    // Update the policy if a new on was received
    if (mrtRosInterface_->updatePolicy())
    {
      is_mpc_updated = true;
      auto &policy = mrtRosInterface_->getPolicy();
      auto &state_trajectory = policy.stateTrajectory_;
      // trajectory_publisher_->publishTrajectory(state_trajectory);
      TargetTrajectories target_trajectories(policy.timeTrajectory_, policy.stateTrajectory_, policy.inputTrajectory_);

      publishFeetTrajectory(target_trajectories);
      // std::cout << "state_trajectory.size :" << state_trajectory.size() << std::endl;
      // std::cout << "state_trajectory.front().size :" << state_trajectory.front().size() << std::endl;
      // std::cout << "<<< New MPC policy starting at " << mrtRosInterface_->getPolicy().timeTrajectory_.front() << "\n";
    }
    mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);

    // std::cout << "optimizedState_mrt:" << optimizedState_mrt.transpose() << " \noptimizedInput_mrt:" << optimizedInput_mrt.transpose() << " plannedMode_:" << plannedMode_ << std::endl;
    auto &info = HumanoidInterface_->getCentroidalModelInfo();

    optimizedState2WBC_mrt_ = optimizedState_mrt;
    optimizedInput2WBC_mrt_ = optimizedInput_mrt;
    if (wbc_only_)
    {
      optimizedState2WBC_mrt_ = initial_status_;
      optimizedInput2WBC_mrt_ = intail_input_;
    }

    optimized_mode_ = plannedMode_;
    currentObservation_.input = optimizedInput2WBC_mrt_;
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
      // TODO:站立也使用mrt获取到的optimizedState
      // optimizedInput2WBC_mrt_.setZero();

      // optimizedState2WBC_mrt_.segment(6, 6) = currentObservation_.state.segment<6>(6);
      // optimizedState2WBC_mrt_.segment(6 + 6, jointNum_) = defalutJointPos_;
      // plannedMode_ = 3;
      wbc_->setStanceMode(true);
    }
    else
    {
      wbc_->setStanceMode(false);
    }
    wbcTimer_.startTimer();

    robotVisualizer_->update(currentObservation_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());

    const auto t6 = Clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count() > 1000)
    {
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
    jointPos_ = data.jointPos_;
    jointVel_ = data.jointVel_;
    jointAcc_ = data.jointAcc_;
    jointTorque_ = data.jointTorque_;
    quat_ = data.quat_;
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
    vector_t jointPos(jointNum_), jointVel(jointNum_), jointCurrent(jointNum_);
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    // contact_flag_t contactFlag;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
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
      stateEstimate_->updateIntialEulerAngles(quat_);
      applySensorData(sensors_data);
      stateEstimate_->set_intial_state(currentObservation_.state);
      measuredRbdState_ = stateEstimate_->getRbdState();
      std::cout << "initial measuredRbdState_:" << measuredRbdState_.transpose() << std::endl;
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
    stateEstimate_->estContactForce(period);
    auto est_contact_force = stateEstimate_->getEstContactForce();
    ros_logger_->publishVector("/state_estimate/contact_force", est_contact_force);
    auto est_mode = stateEstimate_->ContactDetection(plannedMode_, is_stance_mode_, plannedMode_, 50, est_contact_force(2), est_contact_force(8), diff_time);
    ros_logger_->publishValue("/state_estimate/mode", static_cast<double>(est_mode));
    est_mode = plannedMode_;
    stateEstimate_->updateMode(est_mode);

    // rbdState_: Angular(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
    if (diff_time > 0.00005 || is_init)
    {
      Eigen::VectorXd updated_joint_pos = jointPos_;
      Eigen::VectorXd updated_joint_vel = jointVel_;

      // ros_logger_->publishVector("/humanoid_controller/updated_joint_pos", updated_joint_pos);
      // ros_logger_->publishVector("/humanoid_controller/updated_joint_vel", updated_joint_vel);
      stateEstimate_->updateJointStates(updated_joint_pos, updated_joint_vel); // 使用关节滤波之后的jointPos和jointVel更新状态估计器
      measuredRbdState_ = stateEstimate_->update(time, period);                // angle(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
      currentObservation_.time += period.toSec();
    }
    // ros_logger_->publishVector("/humanoid_controller/measuredRbdState", measuredRbdState_);
    ros_logger_->publishVector("/state_estimate/base/angular_zyx", measuredRbdState_.segment(0, 3));
    ros_logger_->publishVector("/state_estimate/base/pos_xyz", measuredRbdState_.segment(3, 3));
    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    ros_logger_->publishVector("/state_estimate/joint/pos", measuredRbdState_.segment(6, info.actuatedDofNum));
    ros_logger_->publishVector("/state_estimate/base/angular_vel_zyx", measuredRbdState_.segment(6 + info.actuatedDofNum, 3));
    ros_logger_->publishVector("/state_estimate/base/linear_vel", measuredRbdState_.segment(9 + info.actuatedDofNum, 3));
    ros_logger_->publishVector("/state_estimate/joint/vel", measuredRbdState_.segment(12 + info.actuatedDofNum, 3));
    // else
    //   std::cout << "diff_time too small, skip update state estimate" << std::endl;
    // std::cout << "measuredRbdState_:" << measuredRbdState_.transpose() << std::endl;

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
    currentObservation_.mode = est_mode;
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
                                                  bool verbose, int robot_version_int)
  {
    HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile, gaitCommandFile, robot_version_int);
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                      HumanoidInterface_->getCentroidalModelInfo());
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

    ros::NodeHandle nh;
    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(nh, HumanoidInterface_->getSwitchedModelReferenceManagerPtr(), robotName_);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName_, HumanoidInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nh);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);
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
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
    currentObservation_.time = 0;
  }

  void humanoidCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                              HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
  }

} // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

class HumanoidPlaybackNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_INFO("Initializing HumanoidPlaybackNodelet nodelet...");
    nh = getNodeHandle();
    robot_hw = new humanoid_controller::HybridJointInterface(); // TODO:useless
    pause_sub = nh.subscribe<std_msgs::Bool>("pauseFlag", 1, &HumanoidPlaybackNodelet::pauseCallback, this);

    control_thread = std::thread(&HumanoidPlaybackNodelet::controlLoop, this);
    NODELET_INFO("HumanoidPlaybackNodelet nodelet initialized.");
  }

private:
  bool pause_flag{false};
  ros::NodeHandle nh;
  humanoid_controller::HybridJointInterface *robot_hw;
  ros::Subscriber pause_sub;
  humanoid_controller::humanoidController *controller_ptr_;
  std::chrono::high_resolution_clock::time_point lastTime;
  std::thread control_thread;
  void pauseCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    pause_flag = msg->data;
    std::cerr << "pause_flag: " << pause_flag << std::endl;
  }

  void controlLoop()
  {
    int estimator_type = 1;
    if (nh.hasParam("/estimator_type"))
    {
      nh.getParam("/estimator_type", estimator_type);
    }
    else
    {
      ROS_INFO("estimator_type not found in parameter server");
    }

    std::cout << "Using nomal estimator" << std::endl;
    controller_ptr_ = new humanoid_controller::humanoidController();

    if (!controller_ptr_->init(robot_hw, nh, true))
    {
      ROS_ERROR("Failed to initialize the humanoid controller!");
      return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    auto startTimeROS = ros::Time::now();
    controller_ptr_->starting(startTimeROS);
    lastTime = startTime;
    double controlFrequency = 500.0; // 1000Hz
    nh.getParam("/wbc_frequency", controlFrequency);
    ROS_INFO_STREAM("Wbc control frequency: " << controlFrequency);
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    ros::Rate rate(controlFrequency);
    uint64_t cycle_count = 0;
    while (ros::ok())
    {
      // std::cout << "\n\nControlLoop: "<<cycle_count++ << std::endl;
      if (!pause_flag)
      {
        // std::cout << "controlLoop: Running control loop" << std::endl;
        const auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = currentTime - lastTime;
        ros::Duration elapsedTime(time_span.count());
        lastTime = currentTime;

        // Control
        controller_ptr_->update(ros::Time::now(), elapsedTime);

        // Sleep
        next_time.tv_sec += (next_time.tv_nsec + 1 / controlFrequency * 1e9) / 1e9;
        next_time.tv_nsec = (int)(next_time.tv_nsec + 1 / controlFrequency * 1e9) % (int)1e9;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        // Add print statement if the cycle time exceeds 1 second
        const auto cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count();
        if (cycleTime > 1000)
        {
          ROS_ERROR_STREAM("WBC Cycle time exceeded 1 second: " << cycleTime << "ms");
        }
      }
      else
      {
        std::cout << "controlLoop: Paused" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100000));
      }
      rate.sleep();
    }
    std::cout << "controlLoop: Exiting control loop" << std::endl;
  }
};

PLUGINLIB_EXPORT_CLASS(HumanoidPlaybackNodelet, nodelet::Nodelet)
