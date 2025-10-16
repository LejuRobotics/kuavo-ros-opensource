#include <mobile_manipulator_controllers/mobileManipulatorControllerGeneral.h>

namespace mobile_manipulator_controller
{
  MobileManipulatorControllerGeneral::MobileManipulatorControllerGeneral(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile, MpcType mpcType, int freq, 
    ControlType control_type, bool dummySimArm, bool visualizeMm)
    : MobileManipulatorControllerBase(nh, taskFile, libFolder, urdfFile, mpcType, freq, control_type, dummySimArm, visualizeMm)
    , nh_(nh)
  {
    ikTargetManager_->setEnableHumanoidObservationCallback(false);
  }

  bool MobileManipulatorControllerGeneral::init(double comHeight)
  {
    comHeight_ = comHeight;
    // ros
    humanoidStateSubscriber_ = nh_.subscribe("/humanoid/mm_state", 1, &MobileManipulatorControllerGeneral::humanoidStateCallback, this);
    armTrajPublisher_ = nh_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 10);
    kinematicMpcControlSrv_ = nh_.advertiseService("mobile_manipulator_mpc_control", &MobileManipulatorControllerGeneral::controlService, this);
    humanoidCmdPosPublisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_pose", 10, true);
    humanoidState_ = vector_t::Zero(info_.stateDim);
    ROS_INFO("MobileManipulatorControllerGeneral is initialized, waiting for /humanoid/state is available.");
    return true;
  }

  void MobileManipulatorControllerGeneral::humanoidStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    Eigen::VectorXd mmState(msg->data.size());    
    for(size_t i = 0; i < msg->data.size(); i++)
    {
      mmState(i) = msg->data[i];
    }
    // arm state
    humanoidState_.tail(info_.armDim) = mmState.tail(info_.armDim);
    // base state
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        break;
      case ManipulatorModelType::FloatingArmManipulator:
        humanoidState_.head(6) = mmState.head(6);
        break;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        humanoidState_.head(2) = mmState.head(2);
        humanoidState_(2) = mmState(3);
        break;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        humanoidState_.head(6) = mmState.head(6);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        humanoidState_.head(5) = mmState.head(5);
        break;
      case ManipulatorModelType::ActuatedZPitchManipulator:
        humanoidState_.head(6) = mmState.head(6);
        break;
      default:
        ROS_ERROR("Unknown manipulator model type");
        break;
    }
    if(!recievedObservation_) 
    {
      recievedObservation_ = true;
      ROS_INFO("Recieved first humanoid state");
    }
    ikTargetManager_->setHumanoidObservationByMmState(humanoidState_);
  }

  void MobileManipulatorControllerGeneral::update()
  {
    ros_logger_->publishValue("/mm/control_type", static_cast<int>(controlType_));
    if(lastControlType_ != controlType_){
      if(controlType_ == ControlType::None){
        stop();
        ROS_INFO("MPC is stopped, if you want to resume, please set control_mode to ArmOnly or BaseArm.");
      }
      else if(lastControlType_ == ControlType::None){
        reset(humanoidState_);
        ROS_INFO("MPC is reseted, now you can control the humanoid.");
      }
      lastControlType_ = controlType_;
    }
    if(controlType_ == ControlType::None || !recievedObservation_) return;

    ros_logger_->publishVector("/mm/external_state", humanoidState_);
    vector_t nextState = vector_t::Zero(info_.stateDim);
    vector_t optimizedInput = vector_t::Zero(info_.inputDim);
    bool result = MobileManipulatorControllerBase::update(humanoidState_, nextState, optimizedInput);
    if(result != 0)
    {
      ROS_ERROR("Failed to update MPC");
      return;
    }
    ros_logger_->publishVector("/mm/next_state", nextState);
    ros_logger_->publishVector("/mm/optimized_input", optimizedInput);
    controlHumanoid(nextState, optimizedInput);
  }

  int MobileManipulatorControllerGeneral::controlHumanoid(const vector_t& desiredState, const vector_t& desiredInput)
  {
    ocs2::vector_t desiredArmState = desiredState.tail(info_.armDim);
    ocs2::vector_t desiredArmInput = desiredInput.tail(info_.armDim);

    controlBasePos(desiredState);
    armTrajPublisher_.publish(getJointStatesMsg(desiredArmState, desiredArmInput));
    return 0;
  }

  sensor_msgs::JointState MobileManipulatorControllerGeneral::getJointStatesMsg(const vector_t& q_arm, const vector_t& dq_arm)
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
  }

  bool MobileManipulatorControllerGeneral::controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res) {
    lastControlType_ = controlType_;
    controlType_ = static_cast<ControlType>(req.control_mode);
    res.result = true;
    res.mode = req.control_mode;
    res.message = "Set controlling to " + controlTypeToString(controlType_) + ".";
    return true;
  }

  void MobileManipulatorControllerGeneral::controlBasePos(const vector_t& mmState)
  {
    geometry_msgs::Twist msg;
    switch(info_.manipulatorModelType)
    {
      case ManipulatorModelType::DefaultManipulator:
        return;
      case ManipulatorModelType::WheelBasedMobileManipulator:
        return;
      case ManipulatorModelType::FloatingArmManipulator:
        return;
      case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        msg.linear.z = mmState(2);
        msg.angular.y = mmState(4);
        break;
      case ManipulatorModelType::ActuatedXYZYawPitchManipulator:
        msg.linear.z = mmState(2);
        msg.angular.y = mmState(4);
        break;
      case ManipulatorModelType::ActuatedZPitchManipulator:
        msg.linear.z = mmState(2);
        msg.angular.y = mmState(4);
        break;
      default:
        return;
    }
    humanoidCmdPosPublisher_.publish(msg);
  }
} // namespace mobile_manipulator_controller
