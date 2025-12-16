/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h"

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE &mpc, std::string topicPrefix)
      : mpc_(mpc),
        topicPrefix_(std::move(topicPrefix)),
        bufferPrimalSolutionPtr_(new PrimalSolution()),
        publisherPrimalSolutionPtr_(new PrimalSolution()),
        bufferCommandPtr_(new CommandData()),
        publisherCommandPtr_(new CommandData()),
        bufferPerformanceIndicesPtr_(new PerformanceIndex),
        publisherPerformanceIndicesPtr_(new PerformanceIndex)
  {
    // start thread for publishing
#ifdef PUBLISH_THREAD
    publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  MPC_ROS_Interface::~MPC_ROS_Interface()
  {
    shutdownNode();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::resetMpcNode(TargetTrajectories &&initTargetTrajectories)
  {
    std::lock_guard<std::mutex> resetLock(resetMutex_);
    mpc_.reset();
    mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(initTargetTrajectories));
    mpcTimer_.reset();
    resetRequestedEver_ = true;
    terminateThread_ = false;
    readyToPublish_ = false;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::pauseResumeMpcNode(bool pause)
  {
    std::lock_guard<std::mutex> resetLock(resetMutex_);
    mpcPaused_ = pause;
    
    if (pause) {
      ROS_INFO_STREAM("MPC has been paused.");
    } else {
      ROS_INFO_STREAM("MPC has been resumed.");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request &req, ocs2_msgs::reset::Response &res)
  {
    if (static_cast<bool>(req.reset))
    {
      auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories);
      resetMpcNode(std::move(targetTrajectories));
      res.done = static_cast<uint8_t>(true);

      std::cerr << "\n#####################################################"
                << "\n#####################################################"
                << "\n#################  MPC is reset.  ###################"
                << "\n#####################################################"
                << "\n#####################################################\n";
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[MPC_ROS_Interface] Reset request failed!");
      return false;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool MPC_ROS_Interface::pauseResumeMpcCallback(ocs2_msgs::pause_resume::Request &req, ocs2_msgs::pause_resume::Response &res)
  {
    pauseResumeMpcNode(static_cast<bool>(req.pause));
    res.done = static_cast<uint8_t>(true);
    return true;
  }

  

  ocs2_msgs::mpc_solver_data MPC_ROS_Interface::createMPCSolverDataMsg(const SolverData &solverData, const ocs2_msgs::mpc_flattened_controller &mpcPolicyMsg)
  {
    ocs2_msgs::mpc_solver_data mpcSolverDataMsg;
    mpcSolverDataMsg.initTime = solverData.initTime_;
    mpcSolverDataMsg.finalTime = solverData.finalTime_;

    mpcSolverDataMsg.initState.clear();
    for (int i = 0; i < solverData.initState_.size(); i++){
      mpcSolverDataMsg.initState.push_back(solverData.initState_[i]);
    }
    mpcSolverDataMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(solverData.modeSchedule_);

    mpcSolverDataMsg.targetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(solverData.targetTrajectories_);

    mpcSolverDataMsg.mpc_flattened_controller = mpcPolicyMsg;

    mpcSolverDataMsg.swingPlannerMultipliers.clear();
    for(int i = 0; i < solverData.swingPlannerMultipliers_.size(); i++)
    {
      mpcSolverDataMsg.swingPlannerMultipliers.push_back(solverData.swingPlannerMultipliers_[i]);
    }
    return mpcSolverDataMsg;
  }
  
  SolverData MPC_ROS_Interface::readMPCSolverDataMsg(const ocs2_msgs::mpc_solver_data &mpcSolverDataMsg)
  {
    SolverData solverData;
    solverData.initTime_ = mpcSolverDataMsg.initTime;
    solverData.finalTime_ = mpcSolverDataMsg.finalTime;
    solverData.initState_.resize(mpcSolverDataMsg.initState.size());
    for (int i = 0; i < mpcSolverDataMsg.initState.size(); i++){
      solverData.initState_[i] = mpcSolverDataMsg.initState[i];
    }
    solverData.modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(mpcSolverDataMsg.modeSchedule);

    solverData.targetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(mpcSolverDataMsg.targetTrajectories);

    // mpcSolverDataMsg.mpc_flattened_controller = mpcPolicyMsg;
    CommandData commandData;
    PrimalSolution primalSolution;
    PerformanceIndex performanceIndices;
    readPolicyMsg(mpcSolverDataMsg.mpc_flattened_controller, commandData, primalSolution, performanceIndices);
    solverData.primalSolution_ = primalSolution;

    solverData.swingPlannerMultipliers_.resize(mpcSolverDataMsg.swingPlannerMultipliers.size());
    for (int i = 0; i < mpcSolverDataMsg.swingPlannerMultipliers.size(); i++)
    {
      solverData.swingPlannerMultipliers_[i] = mpcSolverDataMsg.swingPlannerMultipliers[i];
    }
    return solverData;
  }

  void MPC_ROS_Interface::readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, CommandData& commandData,
                                      PrimalSolution& primalSolution, PerformanceIndex& performanceIndices) {
  commandData.mpcInitObservation_ = ros_msg_conversions::readObservationMsg(msg.initObservation);
  commandData.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(msg.planTargetTrajectories);
  performanceIndices = ros_msg_conversions::readPerformanceIndicesMsg(msg.performanceIndices);

  const size_t N = msg.timeTrajectory.size();
  if (N == 0) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] controller message is empty!");
  }
  if (msg.stateTrajectory.size() != N && msg.inputTrajectory.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] state and input trajectories must have same length!");
  }
  if (msg.data.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Data has the wrong length!");
  }

  primalSolution.clear();

  primalSolution.modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(msg.modeSchedule);

  size_array_t stateDim(N);
  size_array_t inputDim(N);
  primalSolution.timeTrajectory_.reserve(N);
  primalSolution.stateTrajectory_.reserve(N);
  primalSolution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.stateTrajectory[i].value.size();
    inputDim[i] = msg.inputTrajectory[i].value.size();
    primalSolution.timeTrajectory_.emplace_back(msg.timeTrajectory[i]);
    primalSolution.stateTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXf>(msg.stateTrajectory[i].value.data(), stateDim[i]).cast<scalar_t>());
    primalSolution.inputTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXf>(msg.inputTrajectory[i].value.data(), inputDim[i]).cast<scalar_t>());
  }

  primalSolution.postEventIndices_.reserve(msg.postEventIndices.size());
  for (auto ind : msg.postEventIndices) {
    primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  std::vector<std::vector<float> const*> controllerDataPtrArray(N, nullptr);
  for (int i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg.data[i].data);
  }

  // instantiate the correct controller
  switch (msg.controllerType) {
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: {
      auto controller = FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new FeedforwardController(std::move(controller)));
      break;
    }
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR: {
      auto controller = LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
  }
}


  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2_msgs::mpc_flattened_controller MPC_ROS_Interface::createMpcPolicyMsg(const PrimalSolution &primalSolution,
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

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::publisherWorker()
  {
    while (!terminateThread_)
    {
      std::unique_lock<std::mutex> lk(publisherMutex_);

      msgReady_.wait(lk, [&]
                     { return (readyToPublish_ || terminateThread_); });

      if (terminateThread_)
      {
        std::cerr << "MPC_ROS_Interface::publisherWorker: Terminating thread." << std::endl;
        break;
      }

      {
        std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
        publisherCommandPtr_.swap(bufferCommandPtr_);
        publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
        publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
      }

      ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
          createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_, *publisherPerformanceIndicesPtr_);

      // publish the message
      mpcPolicyPublisher_.publish(mpcPolicyMsg);
      // publish time cost
      std_msgs::Float64 msg;
      msg.data = mpcTimer_.getFrequencyInHz();
      mpcFrequencyPublisher_.publish(msg);
      msg.data = mpcTimer_.getLastIntervalInMilliseconds();
      mpcTimeCostPublisher_.publish(msg);

      readyToPublish_ = false;
      lk.unlock();
      msgReady_.notify_one();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::copyToBuffer(const SystemObservation &mpcInitObservation)
  {
    // buffer policy mutex
    std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);

    // get solution
    scalar_t finalTime = mpcInitObservation.time + mpc_.settings().solutionTimeWindow_;
    if (mpc_.settings().solutionTimeWindow_ < 0)
    {
      finalTime = mpc_.getSolverPtr()->getFinalTime();
    }
    mpc_.getSolverPtr()->getPrimalSolution(finalTime, bufferPrimalSolutionPtr_.get());

    // command
    bufferCommandPtr_->mpcInitObservation_ = mpcInitObservation;
    bufferCommandPtr_->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();

    // performance indices
    *bufferPerformanceIndicesPtr_ = mpc_.getSolverPtr()->getPerformanceIndeces();
  }
  
  void MPC_ROS_Interface::mpcPlaybackCallback(const ocs2_msgs::mpc_solver_data::ConstPtr &msg)
  {
    if (!mpc_.settings().playBackMode_)
      return;
    SolverData solverData = readMPCSolverDataMsg(*msg);

    std::lock_guard<std::mutex> resetLock(resetMutex_);

    // if (!resetRequestedEver_.load())
    // {
    //   ROS_WARN_STREAM("[mpcPlaybackCallback] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
    //   return;
    // }

    // current time, state, input, and subsystem
    SystemObservation currentObservation;
    currentObservation.time = solverData.initTime_;
    currentObservation.state = solverData.initState_;

    // measure the delay in running MPC
    mpcTimer_.startTimer();

    // run MPC
    // bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
    bool controllerIsUpdated = mpc_.playback(solverData);
    if (!controllerIsUpdated)
    {
      return;
    }
    copyToBuffer(currentObservation);

    // measure the delay for sending ROS messages
    mpcTimer_.endTimer();

    // check MPC delay and solution window compatibility
    scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
    if (mpc_.settings().solutionTimeWindow_ < 0)
    {
      timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
    }
    if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3)
    {
      std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    // display
    if (mpc_.settings().debugPrint_)
    {
      std::cerr << '\n';
      std::cerr << "\n### MPC_ROS Benchmarking";
      std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
      std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    }

    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
        createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);

    mpcPolicyPublisher_.publish(mpcPolicyMsg);
    std_msgs::Float64 msgaa;
    msgaa.data = mpcTimer_.getFrequencyInHz();
    mpcFrequencyPublisher_.publish(msgaa);
    msgaa.data = mpcTimer_.getLastIntervalInMilliseconds();
    mpcTimeCostPublisher_.publish(msgaa);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
  {
    if (mpc_.settings().playBackMode_)
    {
      return;
    }
    std::lock_guard<std::mutex> resetLock(resetMutex_);

    if (!resetRequestedEver_.load())
    {
      ROS_WARN_STREAM("MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
      return;
    }

    // Check if MPC is paused
    if (mpcPaused_.load())
    {
      return;
    }

    // current time, state, input, and subsystem
    const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

    // measure the delay in running MPC
    mpcTimer_.startTimer();

    // run MPC
    bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
    if (!controllerIsUpdated)
    {
      return;
    }
    copyToBuffer(currentObservation);

    // measure the delay for sending ROS messages
    mpcTimer_.endTimer();

    // check MPC delay and solution window compatibility
    scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
    if (mpc_.settings().solutionTimeWindow_ < 0)
    {
      timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
    }
    if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3)
    {
      std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    // display
    if (mpc_.settings().debugPrint_)
    {
      std::cerr << '\n';
      std::cerr << "\n### MPC_ROS Benchmarking";
      std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
      std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    }

// #ifdef PUBLISH_THREAD
    // std::unique_lock<std::mutex> lk(publisherMutex_);
    // readyToPublish_ = true;
    // lk.unlock();
    // msgReady_.notify_one();

// #else
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
        createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
    if (mpc_.settings().recordSolverData_)
    {
      auto solverData = mpc_.getSolverPtr()->getSolverData();
      
      ocs2_msgs::mpc_solver_data  mpcSolverDataMsg = createMPCSolverDataMsg(solverData, mpcPolicyMsg);
      mpcSolverDataPublisher_.publish(mpcSolverDataMsg);
    }
    mpcPolicyPublisher_.publish(mpcPolicyMsg);
    std_msgs::Float64 msgaa;
    msgaa.data = mpcTimer_.getFrequencyInHz();
    mpcFrequencyPublisher_.publish(msgaa);
    msgaa.data = mpcTimer_.getLastIntervalInMilliseconds();
    mpcTimeCostPublisher_.publish(msgaa);
// #endif
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::shutdownNode()
  {
#ifdef PUBLISH_THREAD
    ROS_INFO_STREAM("Shutting down workers ...");

    std::unique_lock<std::mutex> lk(publisherMutex_);
    terminateThread_ = true;
    lk.unlock();

    msgReady_.notify_all();

    if (publisherWorker_.joinable())
    {
      publisherWorker_.join();
    }

    ROS_INFO_STREAM("All workers are shut down.");
#endif

    // shutdown publishers
    mpcPolicyPublisher_.shutdown();
    mpcFrequencyPublisher_.shutdown();
    mpcTimeCostPublisher_.shutdown();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::spin()
  {
    ROS_INFO_STREAM("Start spinning now ...");
    // Equivalent to ros::spin() + check if master is alive
    ros::Rate loop_rate(mpc_.settings().mpcDesiredFrequency_);
    while (::ros::ok() && ::ros::master::check())
    {
      // ::ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MPC_ROS_Interface::launchNodes(ros::NodeHandle &nodeHandle)
  {
    ROS_INFO_STREAM("MPC node is setting up ...");

    // Observation subscriber
    mpcObservationSubscriber_ = nodeHandle.subscribe(topicPrefix_ + "_mpc_observation", 1, &MPC_ROS_Interface::mpcObservationCallback, this,
                                                     ::ros::TransportHints().tcpNoDelay());
    if (nodeHandle.hasParam("play_back"))
    {
      bool play_back;
      ros::param::get("play_back", play_back);
      mpc_.settings().playBackMode_ = play_back;
    }
    if (mpc_.settings().playBackMode_)
      mpcPlaybackSubscriber_ = nodeHandle.subscribe(topicPrefix_ + "_mpc_solver_data", 10, &MPC_ROS_Interface::mpcPlaybackCallback, this);

    // MPC publisher
    mpcPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_flattened_controller>(topicPrefix_ + "_mpc_policy", 1, true);
    mpcSolverDataPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_solver_data>(topicPrefix_ + "_mpc_solver_data", 1, true);
    mpcFrequencyPublisher_ = nodeHandle.advertise<std_msgs::Float64>("monitor/frequency/mpc", 10, true);
    mpcTimeCostPublisher_ = nodeHandle.advertise<std_msgs::Float64>("monitor/time_cost/mpc", 10, true);

    // MPC reset service server
    mpcResetServiceServer_ = nodeHandle.advertiseService(topicPrefix_ + "_mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);
    
    // MPC pause/resume service server
    mpcPauseResumeServiceServer_ = nodeHandle.advertiseService(topicPrefix_ + "_mpc_pause_resume", &MPC_ROS_Interface::pauseResumeMpcCallback, this);

    // display
#ifdef PUBLISH_THREAD
    ROS_INFO_STREAM("Publishing SLQ-MPC messages on a separate thread.");
#endif

    ROS_INFO_STREAM("MPC node is ready.");

    // spin
    spin();
  }

} // namespace ocs2
