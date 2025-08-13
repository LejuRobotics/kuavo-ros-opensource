#include "mobile_manipulator_controllers/mobileManipulatorController.h"
#include <std_msgs/Float64MultiArray.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <thread>
#include <sstream>

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

  MobileManipulatorController::MobileManipulatorController(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile, MpcType mpcType, int freq, 
            bool dummySimBase, bool dummySimArm, bool visualizeMm, bool anomalyStopMpc)
    : MobileManipulatorControllerBase(nh, taskFile, libFolder, urdfFile, mpcType, freq, dummySimBase, dummySimArm, visualizeMm)
  {
  }

  MobileManipulatorController::~MobileManipulatorController()
  {
  }

  bool MobileManipulatorController::init(ros::NodeHandle &nh)
  {
    std::cout << "[MobileManipulatorController] get com_height param..." << std::endl;
    // 等待 com_height 参数
    while(!nh.hasParam("/com_height")) { ros::Duration(0.1).sleep(); }
    nh.getParam("/com_height", comHeight_);
    std::cout << "[MobileManipulatorController] com_height: " << comHeight_ << std::endl;

    // 初始化观测状态
    mmObservation_.state.setZero(info_.stateDim);
    mmObservation_.input.setZero(info_.inputDim);

    // ROS通信层
    terrainHeightSubscriber_ = nh.subscribe<std_msgs::Float64>("/humanoid/mpc/terrainHeight", 1,
                               [&](const std_msgs::Float64::ConstPtr& msg){ terrain_height_ = msg->data; });
    humanoidObservationSub_ = nh.subscribe("/humanoid_wbc_observation", 1, &MobileManipulatorController::humanoidObservationCallback, this);
    kinematicMpcControlSrv_ = nh.advertiseService(robotName_ + "_mpc_control", &MobileManipulatorController::controlService, this);
    getKinematicMpcControlModeSrv_ = nh.advertiseService(robotName_ + "_get_mpc_control_mode", &MobileManipulatorController::getKinematicMpcControlModeService, this);

    humanoidTorsoTargetTrajectoriesPublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target_pose", 1);
    humanoidArmTargetTrajectoriesPublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target_arm", 1);
    humanoidTargetTrajectoriesPublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>("humanoid_mpc_target", 1);
    humanoidCmdVelPublisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    humanoidCmdPosPublisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_pose", 10, true);
    armTrajPublisher_ = nh.advertise<sensor_msgs::JointState>("/mm_kuavo_arm_traj", 10);

    yaml_cfg_ = YAML::LoadFile(mobile_manipulator_controller::getPath() + "/cfg/cfg.yaml");
    arm_min_ = yaml_cfg_["arm_min"].as<std::vector<double>>();
    arm_max_ = yaml_cfg_["arm_max"].as<std::vector<double>>();
    auto positionErrorMaxThreshold = yaml_cfg_["detection_thresholds"]["position_error_max"].as<double>();
    auto orientationErrorMaxThreshold = yaml_cfg_["detection_thresholds"]["orientation_error_max"].as<double>();
    auto positionErrorAvgThreshold = yaml_cfg_["detection_thresholds"]["position_error_avg"].as<double>();
    auto orientationErrorAvgThreshold = yaml_cfg_["detection_thresholds"]["orientation_error_avg"].as<double>();
    auto VelocityErrorAvgThreshold = yaml_cfg_["detection_thresholds"]["velocity_error_avg"].as<double>();
    auto VelocityErrorMaxThreshold = yaml_cfg_["detection_thresholds"]["velocity_error_max"].as<double>();
    setAnomalyCheckThreshold(positionErrorMaxThreshold, orientationErrorMaxThreshold, positionErrorAvgThreshold, orientationErrorAvgThreshold, VelocityErrorAvgThreshold, VelocityErrorMaxThreshold);
    auto base_pose_delta_limit = yaml_cfg_["base_pose_delta_limit"].as<std::vector<double>>();
    basePoseDeltaLimit_ << Eigen::Map<const Eigen::VectorXd>(base_pose_delta_limit.data(), base_pose_delta_limit.size());

    mmControlTypePublisher_ = nh.advertise<std_msgs::Int8>("/mm/control_type", 1);
    // play back mode
    if (nh.hasParam("play_back"))
    {
      nh.getParam("/play_back", is_play_back_mode_);
    }
    ROS_INFO("[MobileManipulatorController] Play back mode: %d", is_play_back_mode_);
    if(is_play_back_mode_)
    {
      mmControlTypeSubscriber_ = nh.subscribe("/mm/control_type", 1, &MobileManipulatorController::mmControlTypeCallback, this);
      // mmStateSubscriber_ = nh.subscribe("/mm/external_state", 1, &MobileManipulatorController::mmStateCallback, this);
      ROS_INFO("[MobileManipulatorController] Play back mode is enabled");
    }
    return true;
  }

  void MobileManipulatorController::update()
  {
    // Provide a mpc observation for the first time
    if(!mpcInitialized_) {//important: only initialize once, to provide a mpc observation for the first time
      mpcInitialized_ = true;
      vector_t nextState = vector_t::Zero(info_.stateDim);
      int result = MobileManipulatorControllerBase::update(mmObservation_.state, nextState);
      std::cout << "MPC initialized, result: " << result << std::endl;
    }
    while(!recievedObservation_) { std::cout << "Waiting for observation..." << std::endl; return;}
    if(!is_play_back_mode_){
      std_msgs::Int8 msg;
      msg.data = static_cast<int8_t>(controlType_);
      mmControlTypePublisher_.publish(msg);
    }
    // ros_logger_->publishValue("/mm/control_type", static_cast<int>(controlType_));

    if(controlType_ == ControlType::None || !recievedObservation_) return;
    
    pubHumanoid2MMTf();
    
    vector_t externalState = mmObservation_.state;
    if(is_play_back_mode_)
    {
      ros_logger_->publishVector("/mm/external_state_play_back", externalState);
    }
    else
      ros_logger_->publishVector("/mm/external_state", externalState);
    
    vector_t nextState = vector_t::Zero(info_.stateDim);
    vector_t optimizedInput = vector_t::Zero(info_.inputDim);
    
    int result = MobileManipulatorControllerBase::update(externalState, nextState, optimizedInput);
    ros_logger_->publishVector("/mm/next_state", nextState);
    ros_logger_->publishVector("/mm/optimized_input", optimizedInput);
    if(result == 0 && controlType_ != ControlType::None) {
      controlHumanoid(nextState, optimizedInput, humanoidObservation_);
    }
  }

  void MobileManipulatorController::humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg)
  {
    humanoidObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    if(!recievedObservation_) recievedObservation_ = true;
    convertObservationfromHumanoid2MM(humanoidObservation_, mmObservation_);
  }

  void MobileManipulatorController::convertObservationfromHumanoid2MM(const SystemObservation& humanoidObservation, SystemObservation& mmOservation)
  {
    const size_t baseDim = info_.stateDim - info_.armDim;
    
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::WheelBasedMobileManipulator:
        mmOservation.state.segment<2>(0) = humanoidObservation.state.segment<2>(6);
        mmOservation.state(2) = humanoidObservation.state(9);
        mmOservation.input(0) = humanoidObservation.state(0);
        mmOservation.input(1) = humanoidObservation.state(5);
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        mmOservation.state.segment<6>(0) = humanoidObservation.state.segment<6>(6);
        mmOservation.input.segment<6>(0) = humanoidObservation.state.segment<6>(0);
        mmOservation.state(2) -= (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        mmOservation.state.segment<5>(0) = humanoidObservation.state.segment<5>(6);
        mmOservation.input.segment<5>(0) = humanoidObservation.state.segment<5>(0);
        mmOservation.state(2) -= (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedZPitchManipulator:
        mmOservation.state.segment<6>(0) = humanoidObservation.state.segment<6>(6);
        mmOservation.input(0) = humanoidObservation.state(2); // v_z
        mmOservation.input(1) = humanoidObservation.state(4); // d_pitch
        mmOservation.state(2) -= (comHeight_ + terrain_height_);
        break;
      default:
        break;
    }
    mmOservation.state.segment(baseDim, info_.armDim) = humanoidObservation.state.segment(24, info_.armDim);
    mmOservation.input.tail(info_.armDim) = humanoidObservation.input.tail(info_.armDim);
    mmOservation.time = humanoidObservation.time;
    mmOservation.mode = humanoidObservation.mode;
  }

  void MobileManipulatorController::convertObservationfromMM2Humanoid(const SystemObservation& mmOservation, const SystemObservation& currentHumanoidObservation, SystemObservation& humanoidObservation)
  {
    const size_t baseDim = info_.stateDim - info_.armDim;
    humanoidObservation = currentHumanoidObservation;
    const size_t humanoidInputDim = humanoidObservation.input.size();
    
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
      case ManipulatorModelType::ActuatedZPitchManipulator:
        humanoidObservation.state.segment<6>(6) = mmOservation.state.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidObservation.state(2) = mmOservation.input(0); // v_z
        humanoidObservation.state(4) = mmOservation.input(1); // d_pitch
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
      case ManipulatorModelType::FloatingArmManipulator:
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        humanoidState.segment<6>(6) = mmState.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidState.segment<6>(0) = mmInput.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidState(8) += (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        humanoidState.segment<5>(6) = mmState.segment<5>(0); // x-y-z-yaw-pitch
        humanoidState.segment<5>(0) = mmInput.segment<5>(0); // x-y-z-yaw-pitch
        humanoidState(8) += (comHeight_ + terrain_height_);
        break;
      case ManipulatorModelType::ActuatedZPitchManipulator:
        humanoidState.segment<6>(6) = mmState.segment<6>(0); // x-y-z-yaw-pitch-roll
        humanoidState(2) = mmInput(0); // v_z
        humanoidState(4) = mmInput(1); // d_pitch
        humanoidState(8) += (comHeight_ + terrain_height_);
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

    const auto& desiredTorsoState = desiredState.segment<6>(6);
    const auto& currentTorsoState = currentHumanoidObservation.state.segment<6>(6);
    ocs2::vector_t desiredArmState = desiredState.tail(info_.armDim);
    ocs2::vector_t desiredArmInput = desiredInput.tail(info_.armDim);
    // limitArmPosition(desiredArmState);
    const auto& currentArmState = currentHumanoidObservation.state.tail(info_.armDim);
    
    auto goalTorsoTargetTrajectories = generateTargetTrajectories(currentTorsoState, desiredTorsoState, currentHumanoidObservation);
    auto goalArmTargetTrajectories = generateTargetTrajectories(currentArmState, desiredArmState, currentHumanoidObservation);
    
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
    const scalar_t targetReachingTime = currentHumanoidObservation.time + 0.01;
    const scalar_array_t timeTrajectory{currentHumanoidObservation.time, targetReachingTime};
    vector_array_t stateTrajectory{currentState, desiredState};
    vector_array_t inputTrajectory(2, vector_t::Zero(currentHumanoidObservation.input.size()));
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  void MobileManipulatorController::limitHumanoidTargetState(vector_t& humanoidTargetState)
  {
    humanoidTargetState.head(6).setZero();
    humanoidTargetState(10) = 3*M_PI/180.0;
    humanoidTargetState(11) = 0.0;
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
      case ManipulatorModelType::ActuatedZPitchManipulator:
        msg.linear.z = mmInput(0);
        msg.angular.y = mmInput(1);
        break;
      default:
        break;
    }
    // limit vel
    {
      // TO-DO: 临时做法，后续需要修改(12/12 by Matthew)
      // msg.linear.x = (msg.linear.x < 0.05) ? 0.0 : msg.linear.x;
      // msg.linear.y = (msg.linear.y < 0.1) ? 0.0 : msg.linear.y;
      // msg.angular.z = (msg.angular.z < 0.05) ? 0.0 : msg.angular.z;
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
      case ManipulatorModelType::ActuatedZPitchManipulator:
        pose_now.head(6) = mmObservation_.state.head(6);
        delta_pose.head(6) = mmState.head(6) - pose_now;
        limitDeltaPose(delta_pose, basePoseDeltaLimit_, 6);
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

  bool MobileManipulatorController::controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
    lastControlType_ = controlType_;
    controlType_ = static_cast<ControlType>(req.control_mode);
    if(lastControlType_ != controlType_){
      if(controlType_ == ControlType::None){
        stop();
        ROS_INFO("MPC is stopped, if you want to resume, please set control_mode to ArmOnly or BaseArm.");
      }
      else{
        reset(mmObservation_.state);
        ROS_INFO_STREAM("MPC is reseted, control mode switched to " << controlTypeToString(controlType_) << ".");
        
        ROS_INFO("Waiting for MPC to fully start and begin publishing observations...");
        
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(2); // 2 seconds timeout
        
        while (!observationPublishing_ && ros::ok()) {
          if (std::chrono::steady_clock::now() - start_time > timeout_duration) {
            ROS_ERROR("Timeout waiting for MPC to start publishing observations.");
            res.result = false;
            res.mode = req.control_mode;
            res.message = "Timeout waiting for MPC to start - " + controlTypeToString(controlType_) + ".";
            return true;
          }
          
          // Show progress every 500ms
          auto elapsed = std::chrono::steady_clock::now() - start_time;
          if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() % 500 == 0) {
            ROS_INFO_STREAM("Waiting for MPC to stabilize... " 
                          << (10.0 - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() / 1000.0) 
                          << "s remaining");
          }
          
          // Short sleep to avoid busy waiting
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (observationPublishing_) {
          ROS_INFO("MPC has fully started and observations are being published!");
        }
      }
      lastControlType_ = controlType_;
    }
    res.result = true;
    res.mode = req.control_mode;
    res.message = "Set controlling to " + controlTypeToString(controlType_) + ".";
    return true;
  }

  bool MobileManipulatorController::getKinematicMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
    res.result = true;
    res.mode = static_cast<int>(controlType_);
    res.message = "Kinematic MPC control mode is " + controlTypeToString(controlType_) + ".";
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

  void MobileManipulatorController::mmControlTypeCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    lastControlType_ = controlType_;
    controlType_ = static_cast<ControlType>(msg->data);
    if(lastControlType_ != controlType_){
      if(controlType_ == ControlType::None){
        stop();
        ROS_INFO("MPC is stopped, if you want to resume, please set control_mode to ArmOnly or BaseArm.");
      }
      else{
        reset(mmObservation_.state);
        ROS_INFO("MPC is reseted, now you can control the humanoid.");
      }
    }
  }

} // namespace mobile_manipulator_controller 