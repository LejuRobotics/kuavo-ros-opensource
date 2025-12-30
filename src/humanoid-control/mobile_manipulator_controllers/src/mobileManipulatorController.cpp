#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "mobile_manipulator_controllers/mobileManipulatorController.h"

#include <std_msgs/Float64MultiArray.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>

#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace mobile_manipulator_controller
{
  std::string controlTypeToString(ControlType controlType)
  {
    switch (controlType)
    {
      case ControlType::None:
        return "None";
      case ControlType::ArmOnly:  
        return "ArmOnly";
      case ControlType::BaseOnly:
        return "BaseOnly";
      case ControlType::BaseArm:
        return "BaseArm";
      default:
        return "Unknown";
    }
  }

  MobileManipulatorController::~MobileManipulatorController()
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
  }

  bool MobileManipulatorController::init(ros::NodeHandle &nh, int freq)
  {
    basePoseCmdUpdatedTime_ = ros::Time::now();
    controllerNh_ = nh;
    freq_ = freq;
    std::string taskFile, libFolder, urdfFile;
    MpcType mpcType = MpcType::SQP;
    controllerNh_.getParam("/mm/taskFile", taskFile);
    controllerNh_.getParam("/mm/libFolder", libFolder);
    controllerNh_.getParam("/mm/urdfFile", urdfFile);

    if(!controllerNh_.hasParam("/dummy_sim"))
      ROS_ERROR_STREAM("dummy_sim parameter is NOT found.");
    else
      controllerNh_.getParam("/dummy_sim", dummySim_);
    if(!controllerNh_.hasParam("/dummy_sim_arm"))
      ROS_ERROR_STREAM("dummy_sim_arm parameter is NOT found.");
    else
      controllerNh_.getParam("/dummy_sim_arm", dummySimArm_);
    std::cout << "Dummy Sim Base: " << dummySim_ << std::endl;
    std::cout << "Dummy Sim Arm: "  << dummySimArm_ << std::endl;
    
    ROS_WARN_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
    while(!controllerNh_.hasParam("/com_height"))
    {
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("\033[32m>>>>>>>>>>>>  com_height parameter is founded. <<<<<<<<<<< \033[0m");
    controllerNh_.getParam("/com_height", comHeight_);
    std::cout << "comHeight: " << comHeight_<<std::endl;
    if(controllerNh_.hasParam("/mm/mpcType"))
    {
      int mpcTypeInt;
      controllerNh_.getParam("/mm/mpcType", mpcTypeInt);
      std::cout << "MPC type: " << mpcTypeInt << std::endl;
      mpcType = static_cast<MpcType>(mpcTypeInt);
    }
    if(controllerNh_.hasParam("/visualize_mm"))
      controllerNh_.getParam("/visualize_mm", visualizeMm_);
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    std::cerr << "MPC type: " << (mpcType == MpcType::SQP? "SQP" : "DDP") << std::endl;
    setupMobileManipulatorInterface(taskFile, libFolder, urdfFile, mpcType);

    // init
    mmObservation_.state.setZero(info_.stateDim);
    mmObservation_.input.setZero(info_.inputDim);


    // auto basePoseCmdCallback = [&](const std_msgs::Float64MultiArray::ConstPtr& msg)
    // {
    //   if(msg->data.size() != 6)
    //   {
    //     ROS_ERROR_STREAM("Invalid base pose command size: " << msg->data.size());
    //     return;
    //   }
    //   basePoseCmd_ = Eigen::Map<const Eigen::VectorXd>(msg->data.data(), msg->data.size());
    //   basePoseCmdUpdatedTime_ = ros::Time::now();
    //   std::cout << "Received base pose command: " << basePoseCmd_.transpose() << std::endl;
    // };

    terrainHeightSubscriber_ = controllerNh_.subscribe<std_msgs::Float64>("/humanoid/mpc/terrainHeight", 1,
                               [&](const std_msgs::Float64::ConstPtr& msg){ terrain_height_ = msg->data; });
    humanoidObservationSub_ = controllerNh_.subscribe("/humanoid_wbc_observation", 1, &MobileManipulatorController::humanoidObservationCallback, this);// contain all arm joint
    // basePoseCmdSubscriber_ = controllerNh_.subscribe<std_msgs::Float64MultiArray>("/base_pose_cmd", 10, basePoseCmdCallback);
    kinematicMpcControlSrv_ = controllerNh_.advertiseService(robotName_ + "_mpc_control", &MobileManipulatorController::controlService, this);
    getKinematicMpcControlModeSrv_ = controllerNh_.advertiseService(robotName_ + "_get_mpc_control_mode", &MobileManipulatorController::getKinematicMpcControlModeService, this);
    resetMpcMrtSrv_ = controllerNh_.advertiseService(robotName_ + "_reset_mpc_mrt", &MobileManipulatorController::resetMpcMrtService, this);
    mpcPolicyPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);
    toggleMpcSrv_ = controllerNh_.advertiseService(robotName_ + "_toggle_mpc", &MobileManipulatorController::toggleMpcService, this);
    humanoidTorsoTargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target_pose", 1);
    humanoidArmTargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target_arm", 1);
    humanoidTargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target", 1);
    humanoidCmdVelPublisher_ = controllerNh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    humanoidCmdPosPublisher_ = controllerNh_.advertise<geometry_msgs::Twist>("/cmd_pose", 10, true);
    mmEefPosesPublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>(robotName_ + "_eef_poses", 10, true);
    mmPlanedTrajPublisher_ = nh.advertise<visualization_msgs::MarkerArray>(robotName_ + "/planed_two_hand_trajectory", 10);
    armTrajPublisher_ = controllerNh_.advertise<sensor_msgs::JointState>("/mm_kuavo_arm_traj", 10);
    targetSubscriber_ = controllerNh_.subscribe(robotName_ + "_mpc_target", 1, &MobileManipulatorController::targetCallback, this);

    resetMpcSrv_ = controllerNh_.advertiseService("reset_mm_mpc", &MobileManipulatorController::resetMpcService, this);

    yaml_cfg_ = YAML::LoadFile(mobile_manipulator_controller::getPath() + "/cfg/cfg.yaml");

    arm_min_ = yaml_cfg_["arm_min"].as<std::vector<double>>();
    arm_max_ = yaml_cfg_["arm_max"].as<std::vector<double>>();
    if(arm_min_.size() != info_.armDim || arm_max_.size() != info_.armDim)
    {
      ROS_ERROR_STREAM("Invalid arm limits size: " << arm_min_.size() << " " << arm_max_.size());
      throw std::runtime_error("Invalid arm limits size.");
    }

    auto base_pose_delta_limit = yaml_cfg_["base_pose_delta_limit"].as<std::vector<double>>();
    // basePoseDeltaLimit_ << 1, 1, 0.2, 1, 0.5, 0.1;
    basePoseDeltaLimit_ << Eigen::Map<const Eigen::VectorXd>(base_pose_delta_limit.data(), base_pose_delta_limit.size());

    setupMpc();
    setupMrt();

    starting();
    return true;
  }

  void MobileManipulatorController::setupMobileManipulatorInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile, MpcType mpcType)
  {
    mpcType_ = mpcType;
    mobileManipulatorInterface_ = std::make_shared<ocs2::mobile_manipulator::MobileManipulatorInterface>(taskFile, libFolder, urdfFile);
    info_ = mobileManipulatorInterface_->getManipulatorModelInfo();
    // Visualization
    visualizationPtr_ = std::make_shared<MobileManipulatorVisualization>(controllerNh_, *mobileManipulatorInterface_);
    pinocchioInterface_ptr_.reset(new PinocchioInterface(mobileManipulatorInterface_->getPinocchioInterface()));
    pinocchioMappingPtr_ = std::make_unique<MobileManipulatorPinocchioMapping>(mobileManipulatorInterface_->getManipulatorModelInfo());
    eeSpatialKinematicsPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(mobileManipulatorInterface_->getPinocchioInterface(), *pinocchioMappingPtr_.get(), 
                                                                                      mobileManipulatorInterface_->getManipulatorModelInfo().eeFrames);
    eeSpatialKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_.get());
  }

  void MobileManipulatorController::setupMpc()
  {
    if(mpcType_ == MpcType::SQP)
      mpc_ = std::make_shared<SqpMpc>(mobileManipulatorInterface_->mpcSettings(), mobileManipulatorInterface_->sqpSettings(),
                                      mobileManipulatorInterface_->getOptimalControlProblem(), mobileManipulatorInterface_->getInitializer());
    else if(mpcType_ == MpcType::DDP)
      mpc_ = std::make_shared<GaussNewtonDDP_MPC>(mobileManipulatorInterface_->mpcSettings(), mobileManipulatorInterface_->ddpSettings(), mobileManipulatorInterface_->getRollout(),
                                                  mobileManipulatorInterface_->getOptimalControlProblem(), mobileManipulatorInterface_->getInitializer());

    else
    {
      ROS_ERROR_STREAM("MPC type: " << static_cast<int>(mpcType_) << " is not supported.");
      throw std::runtime_error("Invalid MPC type.");
    }
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName_, mobileManipulatorInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(controllerNh_);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);
  }

  void MobileManipulatorController::setupMrt()
  {
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
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
                newMPCSolved_ = true;
              }
            },
            mobileManipulatorInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    } });
    setThreadPriority(mobileManipulatorInterface_->sqpSettings().threadPriority, mpcThread_);
  }

  void MobileManipulatorController::starting()
  {
    ROS_WARN("No observation received yet. Keep waiting...");
    while(!recievedObservation_)
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    ROS_INFO_STREAM("\033[32m>>>>>>>>>>>> Observation received. <<<<<<<<<<< \033[0m");

    SystemObservation initial_observation = mmObservation_;
    initial_observation.state.setZero(info_.stateDim);
    initial_observation.input.setZero(info_.inputDim);
    vector_t init_q;
    if(ros::param::has("/humanoid/init_q"))
    {
      std::vector<double> init_q_vec;
      ros::param::get("/humanoid/init_q", init_q_vec);
      init_q.resize(init_q_vec.size());
      init_q = vector_t::Map(init_q_vec.data(), init_q_vec.size());
      init_q(2) = 0.0;
      std::cout << "Init q: " << init_q.transpose() << std::endl;
    }
    else
      ROS_ERROR_STREAM("/humanoid/init_q parameter is NOT found.");
    const size_t baseDim = info_.stateDim - info_.armDim;
    initial_observation.state.head(3) = init_q.head(3); // 暂时不考虑姿态
    initial_observation.state.tail(info_.armDim) = init_q.tail(info_.armDim);
    mmObservationDummy_ = initial_observation;
    std::cout << "Initial state: " << initial_observation.state.transpose() << std::endl;
    std::cout << "Initial state size: " << initial_observation.state.size() << std::endl;
    std::cout << "Initial input size: " << initial_observation.input.size() << std::endl;
    // initial command
    ocs2::vector_t initTarget(baseDim + 7*2);
    initTarget.head(baseDim).setZero();
    ocs2::vector_t hand_pose(7);

    hand_pose.head(3) << 0.03991785153362081, 0.23201663533511738, -0.1;
    auto q = Eigen::Quaternion<scalar_t>(1.0, 0, 0, 0);
    q.normalize();
    hand_pose.tail(4) << q.coeffs();
    initTarget.segment<7>(baseDim) = hand_pose;
    hand_pose(1) *= -1.0;
    initTarget.segment<7>(baseDim+7) = hand_pose;
    const vector_t zeroInput = vector_t::Zero(info_.inputDim);
    const TargetTrajectories target_trajectories({initial_observation.time}, {initTarget}, {zeroInput});
    // TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});

    // Set the first observation and command and wait for optimization to finish
    mpcMrtInterface_->setCurrentObservation(initial_observation);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    mpcRunning_ = false;
    while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    {
      mpcMrtInterface_->advanceMpc();
      ros::WallRate(mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
    ROS_INFO_STREAM("Initial policy received time: " << mmObservation_.time);
  }

  void MobileManipulatorController::update()
  {
    pubHumanoid2MMTf();
    {
      std::lock_guard<std::mutex> lock(targetMutex_);
      if (controlType_ != ControlType::None && !mpcRunning_ && newTargetReceived_) {
          // More robust: check if target is recent
          if ((ros::Time::now() - lastTargetTime_).toSec() < 0.5) {
                restart();
                ROS_INFO("MPC started after receiving new target.");
          }
      }
    }
    
    // 检查是否正在重置中，如果是则跳过更新
    if (isResetting_.load()) {
      return;
    }
    
    if(!recievedObservation_)
    {
      ROS_WARN("No observation received yet. Skipping update.");
      return;
    }
    
    // 检查MRT接口是否有效
    if (!mpcMrtInterface_) {
      ROS_WARN("MRT interface is null. Skipping update.");
      return;
    }
    const size_t baseDim = info_.stateDim - info_.armDim;
    // std::cout << "[MobileManipulatorController] mmObservation_: " << mmObservation_.state.transpose() << std::endl;
    const vector_t eefPoses = getMMEefPose(mmObservation_.state);
    const TargetTrajectories target_trajectories({mmObservation_.time}, {eefPoses}, {vector_t::Zero(info_.inputDim)});
    switch(controlType_)
    {
      case ControlType::None:
        mmObservationDummy_.state = mmObservation_.state;
        // mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
        stop();
        // return;//
        break;
      case ControlType::ArmOnly:
        break;
      case ControlType::BaseOnly:
        break;
      case ControlType::BaseArm:
        break;
      default:
        break;
    }
    // Update the current state of the system
    SystemObservation obs = mmObservation_;
    obs.time = mmObservationDummy_.time;
    obs.input.setZero();
    if(dummySim_)
    {
      obs.state.head(baseDim) = mmObservationDummy_.state.head(baseDim);
      // obs.state.tail(info_.armDim) = mmObservationDummy_.state.tail(info_.armDim);
    }
    if(dummySimArm_)
      obs.state.tail(info_.armDim) = mmObservationDummy_.state.tail(info_.armDim);
    
    // 安全访问MRT接口
    if (!mpcMrtInterface_) return;
    mpcMrtInterface_->setCurrentObservation(obs);
    
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(obs));

    if (mpcRunning_ && newMPCSolved_)
    {

      vector_t optimizedStateMrt, optimizedInputMrt;
      // Load the latest MPC policy
      bool is_mpc_updated = false;
      
      if (mpcMrtInterface_ && mpcMrtInterface_->updatePolicy())
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
        // ROS_INFO_STREAM("MPC policy updated. TIME: " << mmObservation_.time);
        if(visualizeMm_)
          visualizationPtr_->update(obs, policy, command);
      }

      // Evaluate the current policy - 使用异常处理来安全调用
      if (!mpcMrtInterface_) return;
      
      try {
        size_t mode;
        mpcMrtInterface_->evaluatePolicy(obs.time, obs.state, optimizedStateMrt, optimizedInputMrt, mode);
      } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("updatePolicy() should be called first") != std::string::npos) {
          // MRT接口还没有准备好，跳过这个周期
          ROS_WARN_THROTTLE(1.0, "MRT interface requires policy update before evaluation, skipping this cycle");
          return;
        } else {
          // 其他运行时错误，记录但不崩溃
          ROS_ERROR_STREAM_THROTTLE(1.0, "MRT evaluatePolicy error: " << error_msg);
          return;
        }
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Unexpected error in evaluatePolicy: " << e.what());
        return;
      }
      // if(controlling_)
      //   controlHumanoid(optimizedStateMrt, optimizedInputMrt, humanoidObservation_);
      // else
      //   ROS_WARN("Controlling is not enabled.");
      // if(dummySim_)

      mmObservationDummy_ = forwardSimulation(mmObservationDummy_);
      if(controlType_ != ControlType::None)
        controlHumanoid(mmObservationDummy_.state, optimizedInputMrt, humanoidObservation_);
      // publish eef poses
      // const vector_t eefPoses = getMMEefPose(mmObservationDummy_.state);
    }
    else{
      mmObservationDummy_.state = mmObservation_.state;
    }

    // visualize eef poses
    if(mmPlanedTrajQueue_.size() < 200)
      mmPlanedTrajQueue_.push_back(eefPoses);
    else
      mmPlanedTrajQueue_.pop_front();
    auto publishEefPoses = [&](const vector_t& eefPoses)
    {
      std_msgs::Float64MultiArray eefPosesMsg;
      eefPosesMsg.data.resize(eefPoses.size());
      for(size_t i = 0; i < eefPoses.size(); i++)
        eefPosesMsg.data[i] = eefPoses[i];
      mmEefPosesPublisher_.publish(eefPosesMsg);
    };
    publishEefPoses(eefPoses);
    // visualize
    mmPlanedTrajPublisher_.publish(getVisualizeTrajectoryMsg(mmPlanedTrajQueue_, {0.1, 0.9, 0.1, 1.0}));
  }

  void MobileManipulatorController::humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg)
  {
    humanoidObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    if(!recievedObservation_)
    {
      recievedObservation_ = true;
      convertObservationfromHumanoid2MM(humanoidObservation_, mmObservation_);
    }
    if(!dummySim_)
      convertObservationfromHumanoid2MM(humanoidObservation_, mmObservation_);
  }

  ocs2_msgs::mpc_flattened_controller MobileManipulatorController::createMpcPolicyMsg(const PrimalSolution &primalSolution,
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

    return std::move(mpcPolicyMsg);
  }


  void MobileManipulatorController::convertObservationfromHumanoid2MM(const SystemObservation& humanoidObservation, SystemObservation& mmOservation)
  {
    const auto& info = info_;
    const size_t baseDim = info.stateDim - info.armDim;
    //TO-DO: add more types of mapping(12-09 by Matthew)
    switch(info.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        mmOservation.state.segment<2>(0) = humanoidObservation.state.segment<2>(6); // x-y
        mmOservation.state(2) = humanoidObservation.state(9); // yaw
        mmOservation.input(0) = humanoidObservation.state(0); // v_x
        mmOservation.input(1) = humanoidObservation.state(5); // v_yaw
        break;
      case ManipulatorModelType::FloatingArmManipulator:
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        mmOservation.state.segment<6>(0) = humanoidObservation.state.segment<6>(6); // x-y-z-yaw-pitch-roll
        mmOservation.input.segment<6>(0) = humanoidObservation.state.segment<6>(0); // x-y-z-yaw-pitch-roll
        mmOservation.state(2) -= (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        mmOservation.state.segment<5>(0) = humanoidObservation.state.segment<5>(6); // x-y-z-yaw-pitch
        mmOservation.input.segment<5>(0) = humanoidObservation.state.segment<5>(0); // x-y-z-yaw-pitch
        mmOservation.state(2) -= (comHeight_ + terrain_height_);
        break;
      default:
        break;
    }
    mmOservation.state.segment(baseDim, info.armDim) = humanoidObservation.state.segment(24, info.armDim);
    mmOservation.input.tail(info.armDim) = humanoidObservation.input.tail(info.armDim);
    mmOservation.time = humanoidObservation.time;
    mmOservation.mode = humanoidObservation.mode;
  }

  void MobileManipulatorController::convertObservationfromMM2Humanoid(const SystemObservation& mmOservation, const SystemObservation& currentHumanoidObservation, SystemObservation& humanoidObservation)
  {
    const size_t baseDim = info_.stateDim - info_.armDim;
    humanoidObservation = currentHumanoidObservation;
    const size_t humanoidInputDim = humanoidObservation.input.size();
    //TO-DO: add more types of mapping(12-09 by Matthew)
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        humanoidObservation.state.segment<2>(6) = mmOservation.state.segment<2>(0); // x-y
        humanoidObservation.state(9) = mmOservation.state(2);// yaw
        humanoidObservation.state(0) = mmOservation.input(0); // v_x
        humanoidObservation.state(5) = mmOservation.input(1); // v_yaw
        break;
      case ManipulatorModelType::FloatingArmManipulator:
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        humanoidObservation.state.segment<6>(6) = mmOservation.state.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidObservation.state.segment<6>(0) = mmOservation.input.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidObservation.state(8) += (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        humanoidObservation.state.segment<5>(6) = mmOservation.state.segment<5>(0); // x-y-z-yaw-pitch
        humanoidObservation.state.segment<5>(0) = mmOservation.input.segment<5>(0); // x-y-z-yaw-pitch
        humanoidObservation.state(8) += (comHeight_ + terrain_height_);
        break;
      default:
        break;
    }
    humanoidObservation.state.segment(24, info_.armDim) = mmOservation.state.segment(baseDim, info_.armDim);
    humanoidObservation.input.tail(info_.armDim) = mmOservation.input.tail(info_.armDim);
    humanoidObservation.time = mmOservation.time;
    humanoidObservation.mode = mmOservation.mode;
  }

  std::pair<vector_t, vector_t> MobileManipulatorController::convertStateInputfromMM2Humanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation)
  {
    vector_t humanoidState = currentHumanoidObservation.state;
    vector_t humanoidInput = currentHumanoidObservation.input;

    humanoidState(8) = (comHeight_ + terrain_height_);
    //TODO: add more types of mapping
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        humanoidState(6) = mmState(0); // x
        humanoidState(7) = mmState(1); // y
        humanoidState(9) = mmState(2);// yaw
        humanoidState(0) = mmInput(0); // v_x
        humanoidState(5) = mmInput(1); // v_yaw
        break;
      default:
        break;
    }
    humanoidState.tail(info_.armDim) = mmState.tail(info_.armDim);
    humanoidInput.tail(info_.armDim) = mmInput.tail(info_.armDim);
    return std::move(std::make_pair(humanoidState, humanoidInput));
  }

  void MobileManipulatorController::controlHumanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation)
  {
    vector_t desiredState, desiredInput;
    std::tie(desiredState, desiredInput) = convertStateInputfromMM2Humanoid(mmState, mmInput, humanoidObservation_);
    limitHumanoidTargetState(desiredState);

    // // target reaching duration
    // const scalar_t targetReachingTime = currentHumanoidObservation.time + 0.01;//TODO: time
    // // desired time trajectory
    // const scalar_array_t timeTrajectory{targetReachingTime};
    // // desired state trajectory
    // vector_array_t stateTrajectory(1);
    // stateTrajectory[0] = desiredState;
    // // desired input trajectory (just right dimensions, they are not used)
    // vector_array_t inputTrajectory(1);
    // inputTrajectory[0] = desiredInput;
    // TargetTrajectories goalTargetTrajectories(timeTrajectory, stateTrajectory, inputTrajectory);

    // const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(goalTargetTrajectories);
    // humanoidTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);

    const auto& desiredTorsoState = desiredState.segment<6>(6);
    const auto& currentTorsoState = currentHumanoidObservation.state.segment<6>(6);
    ocs2::vector_t desiredArmState = desiredState.tail(info_.armDim);
    ocs2::vector_t desiredArmInput = desiredInput.tail(info_.armDim);
    // limitArmPosition(desiredArmState);
    const auto& currentArmState = currentHumanoidObservation.state.tail(info_.armDim);
    // std::cout << "Desired torso state: " << desiredTorsoState.transpose() << std::endl;
    auto goalTorsoTargetTrajectories = generateTargetTrajectories(currentTorsoState, desiredTorsoState, currentHumanoidObservation);
    auto goalArmTargetTrajectories = generateTargetTrajectories(currentArmState, desiredArmState, currentHumanoidObservation);
    // if(dummySim_)
    //   humanoidTorsoTargetTrajectoriesPublisher_.publish(ros_msg_conversions::createTargetTrajectoriesMsg(goalTorsoTargetTrajectories));
    // else
    // if(velControl_)
    //   controlBase(mmState, mmInput);
    // else
    // std::cout << "[MobileManipulatorController] mmState: " << mmState.transpose() << std::endl;

    // humanoidArmTargetTrajectoriesPublisher_.publish(ros_msg_conversions::createTargetTrajectoriesMsg(goalArmTargetTrajectories));
    auto getJointStatesMsg = [&](const vector_t& q_arm, const vector_t& dq_arm)
    {
      
      if(q_arm.size() != dq_arm.size()) // 关节数不一致
        ROS_ERROR("q_arm, dq_arm size is not equal");
      sensor_msgs::JointState msg;
      msg.name.resize(q_arm.size());
      for (int i = 0; i < q_arm.size(); ++i) {
        msg.name[i] = "arm_joint_" + std::to_string(i + 1);
      }
      msg.header.stamp = ros::Time::now();
      
      // 假设 q_arm 的大小已符合
      msg.position.resize(q_arm.size());
      msg.velocity.resize(q_arm.size());
      for (size_t i = 0; i < q_arm.size(); ++i) {
        msg.position[i] = 180.0 / M_PI * q_arm[i]; // 转换为度      
        msg.velocity[i] = 180.0 / M_PI * dq_arm[i]; // 转换为度/s
      }
    
      return std::move(msg);
    };
    switch(controlType_)
    {
      case ControlType::None:
        break;
      case ControlType::ArmOnly:
        humanoidArmTargetTrajectoriesPublisher_.publish(ros_msg_conversions::createTargetTrajectoriesMsg(goalArmTargetTrajectories));
        armTrajPublisher_.publish(getJointStatesMsg(desiredArmState, desiredArmInput));
        break;
      case ControlType::BaseOnly:
        // controlBasePos(mmState, mmInput);只控制base没有意义
        break;
      case ControlType::BaseArm:
        controlBasePos(mmState, mmInput);
        humanoidArmTargetTrajectoriesPublisher_.publish(ros_msg_conversions::createTargetTrajectoriesMsg(goalArmTargetTrajectories));
        armTrajPublisher_.publish(getJointStatesMsg(desiredArmState, desiredArmInput));
        break;
      default:
        break;
    }
  }

  TargetTrajectories MobileManipulatorController::generateTargetTrajectories(const vector_t& currentState, const vector_t& desiredState, const SystemObservation& currentHumanoidObservation)
  {
    // target reaching duration
    const scalar_t targetReachingTime = currentHumanoidObservation.time + 0.01;//TODO: time
    // desired time trajectory
    const scalar_array_t timeTrajectory{currentHumanoidObservation.time, targetReachingTime};
    // desired state trajectory
    vector_array_t stateTrajectory{currentState, desiredState};
    // desired input trajectory (just right dimensions, they are not used)
    vector_array_t inputTrajectory(2, vector_t::Zero(currentHumanoidObservation.input.size()));
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  void MobileManipulatorController::limitHumanoidTargetState(vector_t& humanoidTargetState)
  {
    humanoidTargetState.head(6).setZero(); // x-y-z-roll-pitch-yaw
    humanoidTargetState(10) = 3*M_PI/180.0; // pitch
    humanoidTargetState(11) = 0.0;          // roll
  }

  void MobileManipulatorController::controlBase(const vector_t& mmState, const vector_t& mmInput)
  {
    geometry_msgs::Twist msg;
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        msg.linear.x = mmInput(0); //v_x
        msg.angular.z = mmInput(1);//yaw
        break;
      case ManipulatorModelType::FloatingArmManipulator:
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        msg.linear.x = mmInput(0);
        msg.linear.y = mmInput(1);
        // msg.linear.z = mmInput(2);
        msg.linear.z = mmState(2);
        msg.angular.z = mmInput(3);//yaw
        msg.angular.y = mmInput(4);//pitch
        msg.angular.x = mmInput(5);//roll
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        msg.linear.x = mmInput(0);
        msg.linear.y = mmInput(1);
        // msg.linear.z = mmInput(2);
        msg.linear.z = mmState(2);
        msg.angular.z = mmInput(3);//yaw
        msg.angular.y = mmInput(4);//pitch
        break;
      default:
        break;
    }
    // limit vel
    {
    //   // TO-DO: 临时做法，后续需要修改(12/12 by Matthew)
    //   msg.linear.x = (msg.linear.x < 0.05) ? 0.0 : msg.linear.x;
    //   msg.linear.y = (msg.linear.y < 0.1) ? 0.0 : msg.linear.y;
    //   msg.angular.z = (msg.angular.z < 0.05) ? 0.0 : msg.angular.z;
      msg.linear.z = std::max(-0.3, std::min(msg.linear.z, 0.05));
    }
    if((msg.linear.x < 0.05) && (msg.linear.y < 0.1))
    {
      ROS_INFO("Cmd_vel is too small, using position control.");
      controlBasePos(mmState, mmInput);
      return;
    }
    humanoidCmdVelPublisher_.publish(msg);
  }

  void MobileManipulatorController::controlBasePos(const vector_t& mmState, const vector_t& mmInput)
  {
    Vector6d pose_now = Vector6d::Zero();
    Vector6d delta_pose = Vector6d::Zero();
    auto limitDeltaPose = [](Vector6d& delta_pose, const Vector6d& basePoseDeltaLimit, int size) {
      for (int i = 0; i < size; i++) {
        if (delta_pose(i) > basePoseDeltaLimit(i)) {
          std::cout << "[MobileManipulatorController] delta_pose(" << i << ") is too large(" << delta_pose(i) << "), limiting to " << basePoseDeltaLimit(i) << std::endl;
          delta_pose(i) = basePoseDeltaLimit(i);
        } else if (delta_pose(i) < -basePoseDeltaLimit(i)) {
          std::cout << "[MobileManipulatorController] delta_pose(" << i << ") is too large(" << delta_pose(i) << "), limiting to " << -basePoseDeltaLimit(i) << std::endl;
          delta_pose(i) = -basePoseDeltaLimit(i);
        }
      }
    };
    geometry_msgs::Twist msg;
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        break;
      case ManipulatorModelType::FloatingArmManipulator:
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        pose_now = mmObservation_.state.head(6);
        delta_pose = mmState.head(6) - pose_now;
        limitDeltaPose(delta_pose, basePoseDeltaLimit_, 6);
        msg.linear.z = pose_now(2) + delta_pose(2);
        msg.angular.y = pose_now(4) + delta_pose(4);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        pose_now.head(5) = mmObservation_.state.head(5);
        delta_pose.head(5) = mmState.head(5) - pose_now;
        limitDeltaPose(delta_pose, basePoseDeltaLimit_, 5);
        msg.linear.z = pose_now(2) + delta_pose(2);
        msg.angular.y = pose_now(4) + delta_pose(4);
        break;
      default:
        break;
    }
    // limit pos
    {
      msg.linear.z = std::max(-0.3, std::min(msg.linear.z, 0.05));
      // msg.angular.y = std::max(0.0, std::min(msg.linear.z, 50.0*M_PI/180.0));
    }
    humanoidCmdPosPublisher_.publish(msg);
  }

  SystemObservation MobileManipulatorController::forwardSimulation(const SystemObservation& currentObservation) {
    const scalar_t dt = 1.0 / static_cast<scalar_t>(freq_);

    SystemObservation nextObservation;
    nextObservation.time = currentObservation.time + dt;
    if (mpcMrtInterface_->isRolloutSet()) {  // If available, use the provided rollout as to integrate the dynamics.
      mpcMrtInterface_->rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                        nextObservation.mode);
    } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
      mpcMrtInterface_->evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
                          nextObservation.mode);
    }

    return std::move(nextObservation);
  }

  bool MobileManipulatorController::controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
      std::cout << "Kinematic MPC control service request received." << std::endl;

      // When transitioning from "None" to an active mode, synchronize the dummy state
      // with the latest real observation. This ensures the MPC starts planning from the robot's actual current state.

      if(controlType_ != ControlType::None && req.control_mode == static_cast<int>(ControlType::None)) {
        std::cout << "Stopping Kinematic MPC control." << std::endl;
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.z = 0;
        // humanoidCmdVelPublisher_.publish(msg);
      }
      if(req.control_mode == static_cast<int>(ControlType::None)) {
        stop();
      }else{
        std::lock_guard<std::mutex> lock(targetMutex_);
        newTargetReceived_ = false;
        stop();
        ROS_INFO("Control mode switched. MPC stopped, waiting for a new target.");
      }

      controlType_ = static_cast<ControlType>(req.control_mode);
      res.result = true;
      res.mode = req.control_mode;
      res.message = "Set controlling to " + controlTypeToString(controlType_) + ".";
      std::cout << res.message << std::endl;
      return true;
  }

  bool MobileManipulatorController::getKinematicMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
    res.result = true;
    res.mode = static_cast<int>(controlType_);
    res.message = "Kinematic MPC control mode is " + controlTypeToString(controlType_) + ".";
    std::cout << res.message << std::endl;
    return true;
  }

  bool MobileManipulatorController::resetMpcService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
    std::cout << "Reset MPC service request received." << std::endl;
    vector_t eefPoses = getMMEefPose(mmObservation_.state);
    const TargetTrajectories target_trajectories({mmObservation_.time}, {eefPoses}, {vector_t::Zero(info_.inputDim)});

    mpcMrtInterface_->resetMpcNode(target_trajectories);
    res.result = true;
    res.message = "MPC reset successfully.";
    return true;
  }

  void MobileManipulatorController::pubHumanoid2MMTf()
  {
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();

    static_transformStamped.header.frame_id = "odom";
    static_transformStamped.child_frame_id = "mm/world";

    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = comHeight_ + terrain_height_;

    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;

    staticBroadcaster_.sendTransform(static_transformStamped);
  }

  vector_t MobileManipulatorController::getMMEefPose(const vector_t& state)
  {
    vector_t eefPoses;// pos(x,y,z) + quat(x,y,z,w)
    const auto& model = pinocchioInterface_ptr_->getModel();
    auto& data = pinocchioInterface_ptr_->getData();
    const auto q = pinocchioMappingPtr_->getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const auto eefPositions = eeSpatialKinematicsPtr_->getPosition(state);
    const auto eefOrientations = eeSpatialKinematicsPtr_->getOrientation(state);

    if(eefPositions.size() != eefOrientations.size())
      std::cerr << "[MobileManipulatorController] eefPositions.size() != eefOrientations.size()" << std::endl;
    eefPoses.resize(7*eefPositions.size());
    for(int i = 0; i < eefPositions.size(); i++)
    {
      eefPoses.segment<7>(7*i).head(3) = eefPositions[i];
      // eefPoses.segment<7>(7*i)(2) += comHeight_;
      eefPoses.segment<7>(7*i).tail(4) = eefOrientations[i].coeffs();
    }
    return std::move(eefPoses);
  }
  // 可视化轨迹
  visualization_msgs::MarkerArray MobileManipulatorController::getVisualizeTrajectoryMsg(const std::deque<Eigen::VectorXd>& twoHandPoseTrajectory, std::vector<double> rgba)
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_l, marker_r;
    marker_l.header.frame_id = "mm/world";
    marker_l.header.stamp = ros::Time::now();
    // marker_l.ns = "l_hand";
    marker_l.id = 0;
    marker_l.type = visualization_msgs::Marker::LINE_STRIP;
    marker_l.action = visualization_msgs::Marker::ADD;
    marker_l.scale.x = 0.01;    // 设置线宽
    marker_l.color.r = rgba[0]; // 设置颜色
    marker_l.color.g = rgba[1]; // 设置颜色
    marker_l.color.b = rgba[2]; // 设置颜色
    marker_l.color.a = rgba[3]; // 设置透明度

    marker_r = marker_l;
    // marker_r.ns = "r_hand";
    marker_r.id = 1;
    auto getTraj = [&](visualization_msgs::Marker &marker, bool isLeft)
    {
      for(const auto& pose : twoHandPoseTrajectory)
      {
        const auto hand_pose = (isLeft ? pose.head(7) : pose.tail(7));
        geometry_msgs::Point p;
        p.x = hand_pose(0);
        p.y = hand_pose(1);
        p.z = hand_pose(2);
        marker.points.push_back(p);
      }
    };
    // left hand
    getTraj(marker_l, true);
    marker_array.markers.push_back(marker_l);
    // right hand
    getTraj(marker_r, false);
    marker_array.markers.push_back(marker_r);

    return std::move(marker_array);
  }

  bool MobileManipulatorController::limitArmPosition(ocs2::vector_t& armPosition)
  {
    bool isLimited = false;
    const vector_t armPositionOld = armPosition;
    for(int i = 0; i < armPosition.size(); i++)
    {
      if(armPosition(i) < arm_min_[i] || armPosition(i) > arm_max_[i])
        isLimited = true;
      armPosition(i) = std::max(arm_min_[i], std::min(arm_max_[i], armPosition(i)));
    }
    if(isLimited)
    {
      std::cout << "[MobileManipulatorController] Arm position is limited from: \n" << armPositionOld.transpose()
                << "\n to: \n" << armPosition.transpose() << std::endl;
    }
    return isLimited;
  }

  void MobileManipulatorController::resetMpc()
  {
    ROS_WARN("Resetting MPC...");
    try {
      // 先清理旧的MPC（智能指针会自动调用析构函数）
      if (mpc_) {
        // 获取旧的solver来清理ReferenceManager（如果需要的话）
        auto* oldSolver = mpc_->getSolverPtr();
        if (oldSolver) {
          // 某些solver可能需要显式清理ReferenceManager
          // 但由于是shared_ptr管理，这里主要是确保引用计数正确
          ROS_INFO("Cleaning old MPC solver references");
        }
        mpc_.reset(); // 显式重置
      }
      
      // 等待确保旧资源完全释放
      ros::Duration(0.1).sleep();
      
      // 重新创建MPC实例
      if(mpcType_ == MpcType::SQP)
        mpc_ = std::make_shared<SqpMpc>(mobileManipulatorInterface_->mpcSettings(), mobileManipulatorInterface_->sqpSettings(),
                                        mobileManipulatorInterface_->getOptimalControlProblem(), mobileManipulatorInterface_->getInitializer());
      else if(mpcType_ == MpcType::DDP)
        mpc_ = std::make_shared<GaussNewtonDDP_MPC>(mobileManipulatorInterface_->mpcSettings(), mobileManipulatorInterface_->ddpSettings(), 
                                                    mobileManipulatorInterface_->getRollout(),
                                                    mobileManipulatorInterface_->getOptimalControlProblem(), mobileManipulatorInterface_->getInitializer());
      else {
        throw std::runtime_error("Unknown MPC type");
      }
      
      // 重新设置ReferenceManager
      auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName_, mobileManipulatorInterface_->getReferenceManagerPtr());
      rosReferenceManagerPtr->subscribe(controllerNh_);
      mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
      
      ROS_INFO("MPC reset successfully");
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to reset MPC: " << e.what());
      throw;
    }
  }

  void MobileManipulatorController::resetMrt()
  {
    ROS_WARN("Resetting MRT interface...");
    try {
      // 停止MPC运行
      mpcRunning_ = false;
      
      // 等待一段时间确保MPC线程停止处理
      ros::Duration(0.1).sleep();
      
      // 等待当前MPC线程完成
      if (mpcThread_.joinable()) {
        controllerRunning_ = false;
        mpcThread_.join();
        ROS_INFO("MPC thread stopped successfully");
      }
      
      // 清理旧的MRT接口
      mpcMrtInterface_.reset();
      
      // 等待一段时间确保资源完全释放
      ros::Duration(0.05).sleep();
      
      // 重新创建MRT接口
      mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
      mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
      mpcTimer_.reset();
      
      // 等待一段时间确保MRT接口完全初始化
      ros::Duration(0.05).sleep();
      
      // 重新启动MPC线程
      controllerRunning_ = true;
      mpcThread_ = std::thread([&]() {
        // 在线程开始时等待一段时间
        ros::Duration(0.1).sleep();
        
        while (controllerRunning_) {
          try {
            executeAndSleep(
                [&]() {
                  if (mpcRunning_ && mpcMrtInterface_) {
                    mpcTimer_.startTimer();
                    mpcMrtInterface_->advanceMpc();
                    mpcTimer_.endTimer();
                    newMPCSolved_ = true;
                  }
                },
                mobileManipulatorInterface_->mpcSettings().mpcDesiredFrequency_);
          } catch (const std::exception& e) {
            controllerRunning_ = false;
            ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
          }
        }
      });
      setThreadPriority(mobileManipulatorInterface_->sqpSettings().threadPriority, mpcThread_);
      
      // 等待线程完全启动后再启动MPC
      ros::Duration(0.2).sleep();
      mpcRunning_ = true;
      
      ROS_INFO("MRT interface reset successfully");
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to reset MRT: " << e.what());
      throw;
    }
  }

  void MobileManipulatorController::resetMpcMrt()
  {
    std::lock_guard<std::mutex> lock(resetMutex_);
    if (isResetting_.load()) {
      ROS_WARN("Reset already in progress, skipping...");
      return;
    }
    
    isResetting_.store(true);
    ROS_WARN("Starting MPC/MRT reset...");
    
    try {
      // 1. 停止MPC运行
      mpcRunning_ = false;
      newMPCSolved_ = false;
      ROS_INFO("MPC running stopped");
      
      // 2. 等待确保停止
      ros::Duration(0.2).sleep();
      
      // 3. 停止线程
      if (mpcThread_.joinable()) {
        controllerRunning_ = false;
        mpcThread_.join();
        ROS_INFO("MPC thread stopped successfully");
      }
      
      // 4. 清理资源
      mpcMrtInterface_.reset();
      mpc_.reset();
      ROS_INFO("Resources cleared");
      
      // 5. 等待资源完全释放
      ros::Duration(0.3).sleep();
      
      // 6. 重新创建MPC
      resetMpc();
      
      // 7. 等待MPC完全初始化
      ros::Duration(0.2).sleep();
      
      // 8. 重新创建MRT接口
      mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
      mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
      mpcTimer_.reset();
      ROS_INFO("MRT interface recreated");
      
      // 9. 等待MRT接口完全初始化
      ros::Duration(0.2).sleep();
      
      // 10. 重新启动线程
      controllerRunning_ = true;
      mpcThread_ = std::thread([&]() {
        ros::Duration(0.2).sleep(); // 线程启动延迟
        
        while (controllerRunning_) {
          try {
            executeAndSleep(
                [&]() {
                  if (mpcRunning_ && mpcMrtInterface_ && controllerRunning_) {
                    mpcTimer_.startTimer();
                    mpcMrtInterface_->advanceMpc();
                    mpcTimer_.endTimer();
                    newMPCSolved_ = true;
                  }
                },
                mobileManipulatorInterface_->mpcSettings().mpcDesiredFrequency_);
          } catch (const std::exception& e) {
            controllerRunning_ = false;
            ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
          }
        }
      });
      setThreadPriority(mobileManipulatorInterface_->sqpSettings().threadPriority, mpcThread_);
      
      // 11. 等待线程完全启动
      ros::Duration(0.3).sleep();
      
      // 12. 最后启动MPC
      mpcRunning_ = true;
      
      // 13. 强制进行一次策略更新以初始化MRT接口
      if (recievedObservation_ && mpcMrtInterface_) {
        try {
          // 使用当前观测设置初始观测
          mpcMrtInterface_->setCurrentObservation(mmObservation_);
          
          // 设置一个简单的目标轨迹
          const vector_t eefPoses = getMMEefPose(mmObservation_.state);
          const TargetTrajectories target_trajectories({mmObservation_.time}, {eefPoses}, {vector_t::Zero(info_.inputDim)});
          mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
          
          // 强制等待一次MPC求解
          ros::Duration(0.5).sleep();
          
          ROS_INFO("MRT interface initialized with current observation");
        } catch (const std::exception& e) {
          ROS_WARN_STREAM("Failed to initialize MRT interface: " << e.what());
        }
      }
      
      ROS_INFO("MPC/MRT reset completed successfully");
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("MPC/MRT reset failed: " << e.what());
      // 确保重置标志被清除
      mpcRunning_ = false;
      controllerRunning_ = false;
    }
    
    isResetting_.store(false);
  }

  bool MobileManipulatorController::resetMpcMrtService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (req.data) {
      try {
        resetMpcMrt();
        res.success = true;
        res.message = "MPC/MRT reset successfully";
        ROS_INFO("MPC/MRT reset requested via service - completed successfully");
        return true;
      } catch (const std::exception& e) {
        res.success = false;
        res.message = std::string("Failed to reset MPC/MRT: ") + e.what();
        ROS_ERROR_STREAM("MPC/MRT reset requested via service - failed: " << e.what());
        return true;  // 服务调用成功，但操作失败
      }
    } else {
      res.success = false;
      res.message = "Reset not requested (data=false)";
      return true;
    }
  }

  bool MobileManipulatorController::toggleMpcService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (!req.data) {
      stop();
      res.success = true;
      res.message = "MPC stopped";
    } else {
      std::lock_guard<std::mutex> lock(targetMutex_);
      newTargetReceived_ = false;
      stop();
      ROS_INFO("MPC enabled via service. Waiting for a new target to start.");
      res.success = true;
      res.message = "MPC enabled, waiting for target.";
    }
    return true;
  }

  void MobileManipulatorController::targetCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg)
  {
      std::lock_guard<std::mutex> lock(targetMutex_);
      newTargetReceived_ = true;
      lastTargetTime_ = ros::Time::now();
      ROS_INFO_ONCE("Received first target, MPC can be started.");
  }

} // namespace mobile_manipulator_controller