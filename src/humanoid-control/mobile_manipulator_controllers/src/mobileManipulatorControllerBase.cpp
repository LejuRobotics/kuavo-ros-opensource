#include <cmath>
#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "mobile_manipulator_controllers/mobileManipulatorControllerBase.h"

#include <std_msgs/Float64MultiArray.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>

#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <thread>

namespace mobile_manipulator_controller
{
  std::string controlTypeToString(ControlType controlType)
  {
    switch (controlType)
    {
      case ControlType::None: return "None";
      case ControlType::ArmOnly: return "ArmOnly";
      case ControlType::BaseOnly: return "BaseOnly";
      case ControlType::BaseArm: return "BaseArm";
      default: return "Unknown";
    }
  }

  MobileManipulatorControllerBase::MobileManipulatorControllerBase(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, 
    const std::string& urdfFile, MpcType mpcType, int freq, 
    ControlType control_type, bool dummySimArm, bool visualizeMm)
  : controllerNh_(nh)
  , mpcType_(mpcType)
  , freq_(freq)
  , controlType_(control_type)
  , dummySimArm_(dummySimArm)
  , visualizeMm_(visualizeMm)
  {
    ros_logger_ = std::make_unique<humanoid::TopicLogger>(nh);
    mpcPolicyPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);
    mmEefPosesPublisher_ = controllerNh_.advertise<std_msgs::Float64MultiArray>(robotName_ + "_eef_poses", 10, true);
    mmPlanedTrajPublisher_ = nh.advertise<visualization_msgs::MarkerArray>(robotName_ + "/planed_two_hand_trajectory", 10);// TODO: 考虑去除
    mmDetectionResultPublisher_ = controllerNh_.advertise<kuavo_msgs::MmDetectionMsg>( "mm/detection_result", 10, true);
    setupMobileManipulatorInterface(taskFile, libFolder, urdfFile, mpcType);
    setupMpc();
    setupMrt();
    starting();
    ROS_INFO("MobileManipulatorControllerBase initialized.");
    ikTargetManager_ = std::make_unique<MobileManipulatorIkTarget>(controllerNh_, robotName_);
  }

  MobileManipulatorControllerBase::~MobileManipulatorControllerBase()
  {
    // 安全地停止MPC线程
    controllerRunning_ = false;
    mpcRunning_ = false;
    updateRunning_ = false;
    
    // 通知线程停止
    {
      std::lock_guard<std::mutex> lock(mpcThreadMutex_);
      mpcThreadPaused_ = false;  // 确保线程不会被暂停在条件变量上
      mpcThreadCondition_.notify_all();
    }
    
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

  void MobileManipulatorControllerBase::setupMobileManipulatorInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile, MpcType mpcType)
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

  void MobileManipulatorControllerBase::setupMpc()
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

  void MobileManipulatorControllerBase::setupMrt()
  {
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]()
                             {
    while (controllerRunning_) {
      try {
        // Check if thread should be paused
        {
          std::unique_lock<std::mutex> lock(mpcThreadMutex_);
          mpcThreadCondition_.wait(lock, [&]() { return !mpcThreadPaused_.load() || !controllerRunning_.load(); });
          
          if (!controllerRunning_) {
            break;
          }
        }
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
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

  ocs2::vector_t MobileManipulatorControllerBase::getTargetFromState(const vector_t& state)
  {
    // initial command
    const size_t baseDim = info_.stateDim - info_.armDim;
    const size_t hand_dim = 7;
    ocs2::vector_t initTarget(baseDim + 2*hand_dim);
    initTarget.head(baseDim) = state.head(baseDim);
    const vector_t eefPoses = getMMEefPose(state);
    if(eefPoses.size() != 2*hand_dim)
      ROS_ERROR_STREAM("eefPoses.size() != 2*hand_dim");
    initTarget.tail(2*hand_dim) = eefPoses;
    return initTarget;
  }

  void MobileManipulatorControllerBase::starting()
  {
    SystemObservation initial_observation;
    initial_observation.time = 0.0;
    initial_observation.state.setZero(info_.stateDim);
    initial_observation.input.setZero(info_.inputDim);

    const size_t baseDim = info_.stateDim - info_.armDim;
    const size_t hand_dim = 7;
    mmObservationDummy_ = initial_observation;
    // Reset observation publishing flag at start
    observationPublishing_ = false;
    std::cout << "Initial time: " << initial_observation.time << std::endl;
    std::cout << "Initial state: " << initial_observation.state.transpose() << std::endl;
    std::cout << "Initial state size: " << initial_observation.state.size() << std::endl;
    std::cout << "Initial input size: " << initial_observation.input.size() << std::endl;
    
    const ocs2::vector_t initTarget = getTargetFromState(initial_observation.state);
    std::cout << "initial target base:       " << initTarget.segment(0, baseDim).transpose() << std::endl;
    std::cout << "initial target hand left:  " << initTarget.segment(baseDim, hand_dim).transpose() << std::endl;
    std::cout << "initial target hand right: " << initTarget.segment(baseDim+hand_dim, hand_dim).transpose() << std::endl;
    const vector_t zeroInput = vector_t::Zero(info_.inputDim);
    const TargetTrajectories target_trajectories({initial_observation.time, initial_observation.time+1.0}, {initTarget, initTarget}, {zeroInput, zeroInput});
    // Set the first observation and command and wait for optimization to finish
    mpcMrtInterface_->setCurrentObservation(initial_observation);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    {
      mpcMrtInterface_->advanceMpc();
      ros::WallRate(mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
    ROS_INFO_STREAM("Initial policy received.");
    mpcRunning_ = true;
    updateRunning_ = true;
  }

  int MobileManipulatorControllerBase::update(const vector_t& externalState, vector_t& nextState)
  {
    vector_t optimizedInput = vector_t::Zero(info_.inputDim);
    return update(externalState, nextState, optimizedInput);
  }

  int MobileManipulatorControllerBase::update(const vector_t& externalState, vector_t& nextState, vector_t& optimizedInput)
  {
    TargetTrajectories target_trajectories;
    if(ikTargetManager_->getTargetTrajectories(target_trajectories))
      mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    if(externalState.size() != info_.stateDim || nextState.size() != info_.stateDim)
    {
      ROS_ERROR_STREAM("externalState.size() != info_.stateDim");
      return -2;
    }
    if(!updateRunning_)
      return -1;
    const size_t baseDim = info_.stateDim - info_.armDim;
    // Update the current state of the system
    SystemObservation obs = mmObservationDummy_;
    obs.time = mmObservationDummy_.time;
    obs.state = externalState;
    obs.input.setZero();//TODO: 验证给mpc的input是否需要清零
    switch(controlType_){
      case ControlType::None:
      case ControlType::ArmOnly:
        break;
      case ControlType::BaseOnly:
      case ControlType::BaseArm:
        // 被控维度使用dummy state
        obs.state(2) = mmObservationDummy_.state(2); // z
        obs.state(4) = mmObservationDummy_.state(4); // pitch
        break;
      default:
        ROS_ERROR("[MobileManipulatorControllerBase] Invalid control type");
        break;
    }
    if(dummySimArm_)
      obs.state.tail(info_.armDim) = mmObservationDummy_.state.tail(info_.armDim);

    // ros_logger_->publishVector("/mm/state_before_mpc", obs.state);
    mpcMrtInterface_->setCurrentObservation(obs);

    const scalar_t dt = 1.0 / static_cast<scalar_t>(freq_);
    vector_t optimizedStateMrt, optimizedInputMrt;
    int result = getOptimizedStateAndInput(obs, optimizedStateMrt, optimizedInputMrt);
    if(result != 0)
      return result;
    ros_logger_->publishVector("/mm/optimized_state", optimizedStateMrt);
    mmObservationDummy_ = forwardSimulation(obs, dt);

    nextState = mmObservationDummy_.state;
    optimizedInput = optimizedInputMrt;
    // ros_logger_->publishVector("/mm/state_after_mpc", nextState);

    const vector_t eefPoses = getMMEefPose(mmObservationDummy_.state);
    publishEefPoses(eefPoses);
    // publish eef poses
    if(mmPlanedTrajQueue_.size() < 200)
      mmPlanedTrajQueue_.push_back(eefPoses);
    else
      mmPlanedTrajQueue_.pop_front();
    // visualize
    mmPlanedTrajPublisher_.publish(getVisualizeTrajectoryMsg(mmPlanedTrajQueue_, {0.1, 0.9, 0.1, 1.0}));
    // filter abnormal state
    double base_err_norm = (nextState - externalState).head(info_.stateDim - info_.armDim).norm();
    double arm_err_norm = (nextState - externalState).tail(info_.armDim).norm();
    ros_logger_->publishValue("/mm/base_err_norm", base_err_norm);
    ros_logger_->publishValue("/mm/arm_err_norm", arm_err_norm);
    
    bool reset_by_count = (updateCount_ < 10);
    bool reset_by_count_and_error = ((updateCount_ < 30) && (arm_err_norm > 0.2 || base_err_norm > 0.2));
    if(reset_by_count || reset_by_count_and_error)
    {
      mmObservationDummy_.state = externalState;
      mmObservationDummy_.input.setZero();
      nextState = externalState;
      optimizedInput = vector_t::Zero(info_.inputDim);
    }
    ++updateCount_;
    ros_logger_->publishVector("/mm/dummy_state", mmObservationDummy_.state);
    ros_logger_->publishValue("/mm/update_count", updateCount_);
    return 0;
  }

  int MobileManipulatorControllerBase::getOptimizedStateAndInput(const SystemObservation& currentObservation, vector_t& optimizedStateMrt, vector_t& optimizedInputMrt)
  {
    // Load the latest MPC policy
    if (mpcMrtInterface_->updatePolicy())
    {
      auto &policy = mpcMrtInterface_->getPolicy();
      auto &command = mpcMrtInterface_->getCommand();
      auto &performance_indices = mpcMrtInterface_->getPerformanceIndices();
      auto &state_trajectory = policy.stateTrajectory_;

      ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
          createMpcPolicyMsg(policy, command, performance_indices);
      anomaly_check(currentObservation, policy, command, performance_indices);
      // if(result != 0)
      //   return result;
      // publish the message
      mpcPolicyPublisher_.publish(mpcPolicyMsg);
      if(visualizeMm_)
        visualizationPtr_->update(currentObservation, policy, command);
    }
    // Evaluate the current policy
    size_t mode;
    mpcMrtInterface_->evaluatePolicy(currentObservation.time, currentObservation.state, optimizedStateMrt, optimizedInputMrt, mode);
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation));
    
    // Set flag to indicate observations are being published
    observationPublishing_ = true;
    
    return 0;
  };

  int MobileManipulatorControllerBase::anomaly_check(const SystemObservation& currentObservation, const PrimalSolution& policy, const CommandData& command, const PerformanceIndex& performanceIndices)
  {
    // 第一类检测：Policy和Target的位置误差（整个轨迹比较）
    std::vector<EefPoseError> policyEefPoseErrors;
    std::vector<scalar_t> validTimes;
    
    if (!policy.timeTrajectory_.empty() && !command.mpcTargetTrajectories_.timeTrajectory.empty())
    {
      const auto& targetTrajectories = command.mpcTargetTrajectories_;
      
      // 遍历整个policy轨迹
      for (size_t i = 0; i < policy.timeTrajectory_.size(); ++i)
      {
        scalar_t currentTime = policy.timeTrajectory_[i];
        validTimes.push_back(currentTime);
        // std::cout << "policy.stateTrajectory_ get" << i << std::endl;
        // std::cout << "policy.stateTrajectory_ size" << policy.stateTrajectory_[i].size() << std::endl;
        // 获取policy在当前时间的末端效应器位姿
        vector_t policyEefPoses = getMMEefPose(policy.stateTrajectory_[i]);

        
        // 获取target在当前时间的末端效应器位姿
        vector_t targetEefPoses = targetTrajectories.getDesiredState(currentTime);
        // std::cout << "policyEefPoses TargetEefPoses get" << i << std::endl;
        // std::cout << "targetEefPoses size" << targetEefPoses.size() << std::endl;
        // std::cout << "policyEefPoses size" << policyEefPoses.size() << std::endl;
                 // 计算位置误差
         EefPoseError eefPoseError = calculateEefPoseErrors(policyEefPoses, targetEefPoses);
         // std::cout << "eefPoseError get" << i << std::endl;
         policyEefPoseErrors.push_back(eefPoseError); // 传递整个结构体而不是totalErrors
        
                 // 发布调试信息
        //  ros_logger_->publishVector("/mm/policy_eef_poses/policy_eef_pose_" + std::to_string(i), policyEefPoses);
        //  ros_logger_->publishVector("/mm/target_eef_poses/target_eef_pose_" + std::to_string(i), targetEefPoses);
        //  ros_logger_->publishVector("/mm/eef_pose_errors/eef_pose_error_" + std::to_string(i), eefPoseError.totalErrors);
        //  ros_logger_->publishVector("/mm/eef_pose_errors/eef_position_error_" + std::to_string(i), eefPoseError.positionErrors);
        //  ros_logger_->publishVector("/mm/eef_pose_errors/eef_orientation_error_" + std::to_string(i), eefPoseError.orientationErrors);
        //  ros_logger_->publishValue("/mm/eef_pose_errors/eef_pose_error_" + std::to_string(i) + "_valid_time", currentTime);
      }
    }
    
    // 第二类检测：Policy的速度检测（直接使用inputTrajectory）
    std::vector<vector_t> policyVelocities;
    std::vector<scalar_t> velocityMagnitudes;
    
    if (!policy.inputTrajectory_.empty())
    {
      for (size_t i = 0; i < policy.inputTrajectory_.size(); ++i)
      {
        const vector_t& inputVector = policy.inputTrajectory_[i];
        vector_t jointVelocities = inputVector.tail(info_.armDim);

        // std::cout << "inputVector get" << i << std::endl;
        policyVelocities.push_back(jointVelocities);
        
        // 计算速度大小（可以是整个input向量的norm，或者提取特定关节的速度）
        scalar_t velocityMagnitude = jointVelocities.norm();
        // std::cout << "velocityMagnitude get"  << i<< std::endl;
        velocityMagnitudes.push_back(velocityMagnitude);
        
        // 发布调试信息
        // ros_logger_->publishVector("/mm/policy_velocities/policy_velocity_" + std::to_string(i), inputVector);
        // ros_logger_->publishValue("/mm/policy_velocities/policy_velocity_magnitude_" + std::to_string(i), velocityMagnitude);
        // if (i < validTimes.size()) {
        //   ros_logger_->publishValue("/mm/policy_velocities/policy_velocity_" + std::to_string(i) + "_valid_time", validTimes[i]);
        // }
      }
    }
    
    // 进行异常检测
    DetectionResult detectionResult = detectEefAnomaliesNew(policyEefPoseErrors, velocityMagnitudes);
    
    publishDetectionResult(detectionResult);
    return 0;
  }

  ocs2_msgs::mpc_flattened_controller MobileManipulatorControllerBase::createMpcPolicyMsg(const PrimalSolution &primalSolution,
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

  SystemObservation MobileManipulatorControllerBase::forwardSimulation(const SystemObservation& currentObservation, scalar_t dt) {
    SystemObservation nextObservation;
    nextObservation.time = currentObservation.time + dt;
    if (mpcMrtInterface_->isRolloutSet()) {  // If available, use the provided rollout as to integrate the dynamics.
      // ROS_INFO_STREAM("Using rollout to integrate the dynamics.");
      mpcMrtInterface_->rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                        nextObservation.mode);
    } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
      // ROS_INFO_STREAM("Using MPC policy to integrate the dynamics.");
      mpcMrtInterface_->evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
                          nextObservation.mode);
    }

    return std::move(nextObservation);
  }


  vector_t MobileManipulatorControllerBase::getMMEefPose(const vector_t& state)
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
      ROS_ERROR("[MobileManipulatorControllerBase] eefPositions.size() != eefOrientations.size()");
    eefPoses.resize(7*eefPositions.size());
    for(int i = 0; i < eefPositions.size(); i++)
    {
      eefPoses.segment<7>(7*i).head(3) = eefPositions[i];
      eefPoses.segment<7>(7*i).tail(4) = eefOrientations[i].coeffs();
    }
    return std::move(eefPoses);
  }

  // 可视化轨迹
  visualization_msgs::MarkerArray MobileManipulatorControllerBase::getVisualizeTrajectoryMsg(const std::deque<Eigen::VectorXd>& twoHandPoseTrajectory, std::vector<double> rgba)
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

  void MobileManipulatorControllerBase::publishEefPoses(const vector_t& eefPoses)
  {
    std_msgs::Float64MultiArray eefPosesMsg;
    eefPosesMsg.data.resize(eefPoses.size());
    for(size_t i = 0; i < eefPoses.size(); i++)
      eefPosesMsg.data[i] = eefPoses[i];
    mmEefPosesPublisher_.publish(eefPosesMsg);
  }

  bool MobileManipulatorControllerBase::isMpcThreadPaused() const {
    std::lock_guard<std::mutex> lock(mpcThreadMutex_);
    // 线程暂停的条件：控制器运行但MPC不运行，且暂停标志为true
    return controllerRunning_ && !mpcRunning_ && mpcThreadPaused_;
  }

  int MobileManipulatorControllerBase::reset(const vector_t& externalState)
  {
    updateCount_ = 0;
    if(externalState.size() != info_.stateDim)
    {
      ROS_ERROR_STREAM("externalState.size() != info_.stateDim");
      return -1;
    }
    
    ROS_INFO("Starting MPC reset process...");
    
    // 暂停MPC线程运行
    mpcRunning_ = false;
    updateRunning_ = false;
    
    // Reset observation publishing flag
    observationPublishing_ = false;
    
    // 使用条件变量暂停MPC线程，避免硬编码等待时间
    {
      std::unique_lock<std::mutex> lock(mpcThreadMutex_);
      mpcThreadPaused_ = true;
      
      // 通知线程暂停
      mpcThreadCondition_.notify_all();
      
      ROS_INFO("MPC thread paused successfully");
    }
    
    // 动态等待线程暂停，基于实际线程状态和配置参数
    auto start_wait = std::chrono::steady_clock::now();
    const auto max_wait_time = std::chrono::milliseconds(maxThreadWaitTimeMs_);
    const auto check_interval = std::chrono::milliseconds(threadCheckIntervalMs_);
    
    bool thread_paused = false;
    int check_count = 0;
    
    while (!thread_paused && std::chrono::steady_clock::now() - start_wait < max_wait_time) {
      thread_paused = isMpcThreadPaused();
      check_count++;
      
      if (!thread_paused) {
        std::this_thread::sleep_for(check_interval);
      }
    }
    
    if (!thread_paused) {
      ROS_WARN("Thread pause timeout after %d checks, proceeding with reset anyway", check_count);
    } else {
      auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_wait);
      ROS_INFO("Thread paused after %ld ms (%d checks)", wait_duration.count(), check_count);
    }
    
    // 安全的重置序列 - 避免内存损坏
    try
    {
      // 第一步：清理所有缓冲区，避免内存冲突
      ROS_INFO("Step 1: Cleaning buffers...");
      mmPlanedTrajQueue_.clear();
      
      // 第二步：重置MRT_BASE，清理所有智能指针
      ROS_INFO("Step 2: Resetting MRT_BASE...");
      mpcMrtInterface_->reset();
      ROS_INFO("MRT_BASE reset completed");
      
      // 第三步：等待一小段时间确保内存清理完成
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      
      // 第四步：重置MPC，确保内存清理
      ROS_INFO("Step 3: Resetting MPC...");
      mpc_->reset();
      ROS_INFO("MPC reset completed");
      
      // 第五步：再次等待确保重置完成
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("Failed to reset MRT_BASE/MPC: " << e.what());
      return -2;
    }

    // 准备新的观测值
    SystemObservation resetObservation;
    resetObservation.time = mmObservationDummy_.time; // 使用当前时间,避免时间跳变
    resetObservation.state = externalState;
    resetObservation.input.setZero(info_.inputDim);
    
    // 安全地重置MPC到给定状态
    try {
      resetMpcToGivenState(resetObservation);
    }
    catch(const std::exception& e) {
      ROS_ERROR_STREAM("Failed to reset MPC to given state: " << e.what());
      return -3;
    }

    // 重新启动MPC线程
    {
      std::lock_guard<std::mutex> lock(mpcThreadMutex_);
      mpcThreadPaused_ = false;
      mpcRunning_ = true;
      updateRunning_ = true;
      mpcThreadCondition_.notify_all();
    }
    
    // 给MPC求解器一些时间进行初始化
    // 这可以减少后续调用getPerformanceIndices时的竞争条件
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 重置observation，保证update函数使用重置后的observation
    mmObservationDummy_ = resetObservation;
    
    ROS_INFO("MPC reset process completed successfully");
    return 0;
  }

  // reset observation
  void MobileManipulatorControllerBase::resetMpcToGivenState(const SystemObservation& resetObservation)
  {
    try {
      ROS_INFO("Step 4: Setting current observation...");
      // 确保观测值已设置
      mpcMrtInterface_->setCurrentObservation(resetObservation);
      
      // 验证观测值设置是否成功
      if (resetObservation.state.size() != info_.stateDim) {
        throw std::runtime_error("Invalid state dimension in reset observation");
      }
      
      ROS_INFO("Step 5: Preparing target trajectories...");
      // reset mpc
      const ocs2::vector_t externalTarget = getTargetFromState(resetObservation.state);
      
      // 验证目标轨迹的有效性
      if (externalTarget.size() == 0) {
        throw std::runtime_error("Invalid target trajectory generated");
      }
      
      const TargetTrajectories target_trajectories({resetObservation.time, resetObservation.time+1.0}, {externalTarget, externalTarget}, {resetObservation.input, resetObservation.input});
      ROS_INFO_STREAM("Resetting MPC to state: " << resetObservation.state.transpose());
      ROS_INFO_STREAM("Target trajectories after reset: " << externalTarget.transpose());
      
      // 重置MPC节点 - 使用更安全的方式
      ROS_INFO("Step 6: Resetting MPC node...");
      try {
        mpcMrtInterface_->resetMpcNode(target_trajectories);
        ROS_INFO("MPC node reset completed");
      }
      catch(const std::exception& e) {
        ROS_ERROR_STREAM("Failed to reset MPC node: " << e.what());
        throw;
      }

      // 等待初始策略 - 使用更保守的方法
      std::cout << "Waiting for the initial policy after reset ..." << std::endl;
      auto start_time = std::chrono::steady_clock::now();
      int max_iterations = 50; // 减少最大迭代次数
      int iteration_count = 0;
      bool policy_received = false;
      
      while (!policy_received && ros::ok() && iteration_count < max_iterations)
      {
        if(std::chrono::steady_clock::now() - start_time > std::chrono::seconds(3)) {
          ROS_ERROR("Timeout while waiting for initial policy after reset.");
          break;
        }
        
        try {
          // 确保使用重置后的观测值
          mpcMrtInterface_->setCurrentObservation(resetObservation);
          mpcMrtInterface_->advanceMpc();
          
          // 检查策略是否已接收
          policy_received = mpcMrtInterface_->initialPolicyReceived();
          
          if (policy_received) {
            ROS_INFO("Initial policy received successfully");
            break;
          }
          
          // 使用更长的睡眠时间，减少CPU使用
          ros::WallRate(mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
          iteration_count++;
        }
        catch(const std::exception& e) {
          ROS_ERROR_STREAM("Error during policy waiting: " << e.what());
          break;
        }
      }
      
      if (!policy_received) {
        ROS_WARN("Initial policy not received within timeout, continuing anyway");
      }
      
      std::cout << "Policy waiting completed after " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << " milliseconds" << std::endl;    
      
      // 更新策略 - 使用更安全的方式
      ROS_INFO("Step 7: Updating policy...");
      try {
        if(mpcMrtInterface_->updatePolicy())
        {
          std::cout << "Policy updated successfully" << std::endl;
        
          auto policy = mpcMrtInterface_->getPolicy();
          std::cout << "Initial policy after reset start at: " << policy.timeTrajectory_[0] << " and end at: " << policy.timeTrajectory_.back() << std::endl;
        }
        else {
          ROS_WARN("Failed to update policy after reset");
        }
      }
      catch(const std::exception& e) {
        ROS_ERROR_STREAM("Error updating policy: " << e.what());
        // 不抛出异常，允许继续执行
      }
      
      ROS_INFO("MPC state reset completed successfully");
    }
    catch(const std::exception& e) {
      ROS_ERROR_STREAM("Exception in resetMpcToGivenState: " << e.what());
      throw; // 重新抛出异常，让上层处理
    }
  };

  int MobileManipulatorControllerBase::setTargetTrajectory(const TargetTrajectories& targetTrajectories)
  {
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
    return 0;
  }


  MobileManipulatorControllerBase::EefPoseError MobileManipulatorControllerBase::calculateEefPoseErrors(const vector_t& optimizedEefPoses, 
    const vector_t& targetEefPoses)
  {
    EefPoseError result;

    if (optimizedEefPoses.size() != targetEefPoses.size()) {
    ROS_WARN_STREAM_THROTTLE(5.0, "EEF poses size mismatch: optimized=" << optimizedEefPoses.size() 
    << ", target=" << targetEefPoses.size());
    return result;
    }

    // 计算总的误差
    result.totalErrors = optimizedEefPoses - targetEefPoses;

    // 分别计算双手的位置误差和姿态误差
    const int numHands = 2;
    const int posePerHand = 7; // 位置(3) + 四元数(4)

    result.positionErrors.resize(numHands * 3);  // 双手位置误差
    result.orientationErrors.resize(numHands);   // 双手姿态误差(角度差)

    for (int hand = 0; hand < numHands; hand++) {
    const int offset = hand * posePerHand;

    // 位置误差 (欧几里得距离)
    Eigen::Vector3d posError = optimizedEefPoses.segment<3>(offset) - targetEefPoses.segment<3>(offset);
    result.positionErrors.segment<3>(hand * 3) = posError;

    // 姿态误差 (四元数角度差)
    Eigen::Quaterniond qOpt(optimizedEefPoses(offset + 6), optimizedEefPoses(offset + 3), 
    optimizedEefPoses(offset + 4), optimizedEefPoses(offset + 5));
    Eigen::Quaterniond qTarget(targetEefPoses(offset + 6), targetEefPoses(offset + 3), 
    targetEefPoses(offset + 4), targetEefPoses(offset + 5));

    qOpt.normalize();
    qTarget.normalize();

    // 确保两个四元数在同一个半球上（避免符号翻转导致的跳变）
    if (qOpt.w() * qTarget.w() < 0) {
      qTarget.coeffs() *= -1.0;
    }

    // 使用四元数点积直接计算角度距离（最稳定的方法）
    // 计算两个四元数的点积（内积）
    double dot = qOpt.w() * qTarget.w() + qOpt.x() * qTarget.x() + qOpt.y() * qTarget.y() + qOpt.z() * qTarget.z();
    
    // 由于已经确保了符号一致性，dot应该是正数
    // 但为了数值稳定性，仍然取绝对值并限制范围
    dot = std::abs(dot);
    dot = std::min(1.0, dot);  // 防止数值误差导致dot > 1
    
    // 使用四元数点积公式计算角度：θ = 2 * acos(|q1 · q2|)
    // 已经确保了符号一致性，可以直接使用：θ = 2 * acos(dot)
    double lieAlgebraDistance = 2.0 * std::acos(dot);

    // 确保角度在[-π/2, π/2]范围内 临时使用，理论上不应该这样 
    if (lieAlgebraDistance > M_PI/2) lieAlgebraDistance = lieAlgebraDistance - M_PI;
    if (lieAlgebraDistance < -M_PI/2) lieAlgebraDistance = lieAlgebraDistance + M_PI;
    
    // std::cout << "lieAlgebraDistance: " << lieAlgebraDistance << std::endl;
    // if (abs(lieAlgebraDistance - errorAnglerEX_) > 1.0)
    // {
    //   std::cout << "lieAlgebraDistance: " << lieAlgebraDistance << std::endl;
    //   std::cout << "errorAnglerEX_: " << errorAnglerEX_ << std::endl;
    //   std::cout << "qOpt: " << qOpt.coeffs().transpose() << " " << qOpt.w() << std::endl;
    //   std::cout << "qTarget: " << qTarget.coeffs().transpose() << " " << qTarget.w() << std::endl;
    //   ros_logger_->publishVector("/mm/eef_pose_errors/qOpt" + std::to_string(hand), qOpt.coeffs());
    //   ros_logger_->publishVector("/mm/eef_pose_errors/qTarget" + std::to_string(hand), qTarget.coeffs());
    // }

    // errorAnglerEX_ = lieAlgebraDistance;

    result.orientationErrors(hand) = abs(lieAlgebraDistance);
    }

    return result;
  }

     MobileManipulatorControllerBase::DetectionResult MobileManipulatorControllerBase::detectEefAnomaliesNew(const std::vector<EefPoseError>& poseErrors,
     const std::vector<scalar_t>& velocityMagnitudes)
   {
     DetectionResult result;
     result.hasAnomaly = false;
     result.anomalyType = "";

     // 初始化所有标志
     result.trajectoryPositionMaxThresholdExceeded = false;
     result.trajectoryPositionAvgThresholdExceeded = false;
     result.trajectoryOrientationMaxThresholdExceeded = false;
     result.trajectoryOrientationAvgThresholdExceeded = false;
     result.velocityAvgThresholdExceeded = false;
     result.velocityMaxThresholdExceeded = false;

     if(poseErrors.empty() && velocityMagnitudes.empty())
     {
       return result;
     }

     try {
       // 第一类检测：整个轨迹上的位置误差
       if (!poseErrors.empty()) {
         std::vector<double> leftHandPosErrors, rightHandPosErrors;
         std::vector<double> leftHandOrientErrors, rightHandOrientErrors;
         
         for (const auto& poseError : poseErrors) {
           if (poseError.positionErrors.size() >= 6 && poseError.orientationErrors.size() >= 2) {
             // 左手位置误差 (前3个元素)
             double leftPosError = poseError.positionErrors.head(3).norm();
             leftHandPosErrors.push_back(leftPosError);
             
             // 右手位置误差 (后3个元素)
             double rightPosError = poseError.positionErrors.tail(3).norm();
             rightHandPosErrors.push_back(rightPosError);
             
             // 左手姿态误差 (已经正确计算的角度误差)
             double leftOrientError = poseError.orientationErrors(0);
             leftHandOrientErrors.push_back(leftOrientError);
             
             // 右手姿态误差 (已经正确计算的角度误差)
             double rightOrientError = poseError.orientationErrors(1);
             rightHandOrientErrors.push_back(rightOrientError);
           }
         }
         
         // 计算位置误差统计
         if (!leftHandPosErrors.empty() && !rightHandPosErrors.empty()) {
           double leftPosAvg = std::accumulate(leftHandPosErrors.begin(), leftHandPosErrors.end(), 0.0) / leftHandPosErrors.size();
           double rightPosAvg = std::accumulate(rightHandPosErrors.begin(), rightHandPosErrors.end(), 0.0) / rightHandPosErrors.size();
                       double leftPosMax = *std::max_element(leftHandPosErrors.begin(), leftHandPosErrors.end());
            double rightPosMax = *std::max_element(rightHandPosErrors.begin(), rightHandPosErrors.end());
            
            result.trajectoryPositionErrorMax = std::max(leftPosMax, rightPosMax);
            result.trajectoryPositionErrorAvg = (leftPosAvg + rightPosAvg) / 2.0;
            
            if (result.trajectoryPositionErrorMax > positionErrorMaxThreshold_) {
              result.hasAnomaly = true;
              result.trajectoryPositionMaxThresholdExceeded = true;
              result.anomalyType += "TrajectoryPositionError ";
            }
            if (result.trajectoryPositionErrorAvg > positionErrorAvgThreshold_) {
              result.hasAnomaly = true;
              result.trajectoryPositionAvgThresholdExceeded = true;
              result.anomalyType += "TrajectoryPositionError ";
            } 
         }
         
         // 计算姿态误差统计
         if (!leftHandOrientErrors.empty() && !rightHandOrientErrors.empty()) {
           double leftOrientAvg = std::accumulate(leftHandOrientErrors.begin(), leftHandOrientErrors.end(), 0.0) / leftHandOrientErrors.size();
           double rightOrientAvg = std::accumulate(rightHandOrientErrors.begin(), rightHandOrientErrors.end(), 0.0) / rightHandOrientErrors.size();
                       double leftOrientMax = *std::max_element(leftHandOrientErrors.begin(), leftHandOrientErrors.end());
            double rightOrientMax = *std::max_element(rightHandOrientErrors.begin(), rightHandOrientErrors.end());
            
            result.trajectoryOrientationErrorMax = std::max(leftOrientMax, rightOrientMax);
            result.trajectoryOrientationErrorAvg = (leftOrientAvg + rightOrientAvg) / 2.0;
            
            if (result.trajectoryOrientationErrorMax > orientationErrorMaxThreshold_) {
              result.hasAnomaly = true;
              result.trajectoryOrientationMaxThresholdExceeded = true;
              result.anomalyType += "TrajectoryOrientationError ";
            } 

            if (result.trajectoryOrientationErrorAvg > orientationErrorAvgThreshold_) {
              result.hasAnomaly = true;
              result.trajectoryOrientationAvgThresholdExceeded = true;
              result.anomalyType += "TrajectoryOrientationError ";
            } 


         }
       }
       
       // 第二类检测：速度阈值检测
       if (!velocityMagnitudes.empty()) {
         // 计算速度统计
                   double velocityAvg = std::accumulate(velocityMagnitudes.begin(), velocityMagnitudes.end(), 0.0) / velocityMagnitudes.size();
          double velocityMax = *std::max_element(velocityMagnitudes.begin(), velocityMagnitudes.end());
          
          result.velocityMagnitudeAvg = velocityAvg;
          result.velocityMagnitudeMax = velocityMax;
          
          // 检查速度阈值 (这里使用线速度阈值，可以根据需要调整)
          if (velocityAvg > VelocityErrorAvgThreshold_) {
            result.hasAnomaly = true;
            result.velocityAvgThresholdExceeded = true;
            result.anomalyType += "VelocityAvgError ";
          }
          
          if (velocityMax > VelocityErrorMaxThreshold_) {
            result.hasAnomaly = true;
            result.velocityMaxThresholdExceeded = true;
            result.anomalyType += "VelocityMaxError ";
          }
       }
       
       // 如果没有异常，设置正常状态
       if (!result.hasAnomaly) {
         result.anomalyType = "Normal";
       }
       
                // 如果检测到异常，打印警告信息
         if (result.hasAnomaly) {
           ROS_WARN_STREAM_THROTTLE(2.0, "ANOMALY DETECTED: " << result.anomalyType 
             << " - TrajPosMaxError: " << result.trajectoryPositionErrorMax 
             << ", TrajPosAvgError: " << result.trajectoryPositionErrorAvg
             << ", TrajOrientMaxError: " << result.trajectoryOrientationErrorMax
             << ", TrajOrientAvgError: " << result.trajectoryOrientationErrorAvg
             << ", VelAvg/Max: " << result.velocityMagnitudeAvg << "/" << result.velocityMagnitudeMax);
         }
       
     } catch (const std::exception& e) {
       ROS_ERROR_STREAM_THROTTLE(5.0, "Error in anomaly detection: " << e.what());
       result.hasAnomaly = false;
       result.anomalyType = "DetectionError";
     }

     return result;
   }
  void MobileManipulatorControllerBase::publishDetectionResult(const DetectionResult& result)
  {
    kuavo_msgs::MmDetectionMsg detectionMsg;
    
    // 添加时间戳
    detectionMsg.header.stamp = ros::Time::now();
    detectionMsg.header.frame_id = "mobile_manipulator";
    
    // 基础异常检测结果
    detectionMsg.hasAnomaly = result.hasAnomaly;
    
    // 第一类检测：轨迹级别的位置误差
    detectionMsg.trajectoryPositionErrorMax = result.trajectoryPositionErrorMax;
    detectionMsg.trajectoryPositionErrorAvg = result.trajectoryPositionErrorAvg;
    detectionMsg.trajectoryPositionMaxThresholdExceeded = result.trajectoryPositionMaxThresholdExceeded;
    detectionMsg.trajectoryPositionAvgThresholdExceeded = result.trajectoryPositionAvgThresholdExceeded;
    
    // 第一类检测：轨迹级别的姿态误差
    detectionMsg.trajectoryOrientationErrorMax = result.trajectoryOrientationErrorMax;
    detectionMsg.trajectoryOrientationErrorAvg = result.trajectoryOrientationErrorAvg;
    detectionMsg.trajectoryOrientationMaxThresholdExceeded = result.trajectoryOrientationMaxThresholdExceeded;
    detectionMsg.trajectoryOrientationAvgThresholdExceeded = result.trajectoryOrientationAvgThresholdExceeded;
    
    // 第二类检测：速度阈值检测
    detectionMsg.velocityMagnitudeAvg = result.velocityMagnitudeAvg;
    detectionMsg.velocityMagnitudeMax = result.velocityMagnitudeMax;
    detectionMsg.velocityAvgThresholdExceeded = result.velocityAvgThresholdExceeded;
    detectionMsg.velocityMaxThresholdExceeded = result.velocityMaxThresholdExceeded;
    
    mmDetectionResultPublisher_.publish(detectionMsg);
  }

} // namespace mobile_manipulator
