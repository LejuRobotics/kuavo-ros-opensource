#include <ros/init.h>
#include <ros/package.h>
#include <cmath>
#include "std_msgs/Float64.h"
#include <std_srvs/Trigger.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/mpc_observation.h>
#include <sensor_msgs/JointState.h>
#include <kuavo_msgs/endEffectorData.h>
#include <kuavo_msgs/armPoseWithTimeStamp.h>
#include <kuavo_msgs/headBodyPose.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/armTargetPoses.h>
#include <std_srvs/SetBool.h>

#include "humanoid_interface/command/HumanoidHandTarget.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"

using namespace ocs2;
using namespace humanoid;

enum class ArmCtlIdx { Left, Right, Both };
enum class ArmCtlMode { JointSpace, TaskSpace };
enum class ArmControlMode {
    KEEP = 0,
    AUTO_SWING = 1,
    EXTERN_CONTROL = 2,
};

#define ArmToZeroTime 3

namespace
{
  scalar_t targetArmDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t comHeight;
  scalar_t armMode = 1.0;

  vector_t targetTorsoDispalcementVelocity(6);
  vector_t defaultJointState(12);
  scalar_t deg2rad = M_PI / 180.0;

  struct HandPose{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    void addBaseOffset(const Eigen::Vector3d& offset)
    {
      position += offset;
    }
    Eigen::VectorXd toVector() const 
    {
      vector_t target(7);
      target(0) = position(0);
      target(1) = position(1);
      target(2) = position(2);
      target(3) = orientation.w();
      target(4) = orientation.x();
      target(5) = orientation.y();
      target(6) = orientation.z();

      return target;
    }
  };
  struct HeadBodyPose{
    double head_pitch{0};
    double head_yaw{0};
    double body_yaw{0};
    double body_pitch{6*deg2rad}; // 绝对值
    double body_x{0};
    double body_y{0};
    double body_height{0.74};
  };
} // namespace


class VRHandCommandNode {
  public:
    VRHandCommandNode(::ros::NodeHandle nodeHandle, ArmCtlMode armCtlMode, ArmCtlIdx armCtlIdx, std::string robotName="humanoid")
      :nh_(nodeHandle)
      ,armCtlMode_(armCtlMode)
      ,armCtlIdx_(armCtlIdx)
      {
        {
          // wait for parameters
          while (!nh_.hasParam("/mpc/mpcArmsDof") || !nh_.hasParam("/armRealDof"))
          {
            sleep(1);
          }
          ros::param::get("/mpc/mpcArmsDof", num_mpc_arm_joints_);
          ros::param::get("/armRealDof", num_arm_joints_);
          half_num_arm_joints_ = num_arm_joints_/2;
          half_num_mpc_arm_joints_ = num_mpc_arm_joints_/2;
        }
        last_callback_time_ = std::chrono::steady_clock::now();
        // Trajectories publisher     
        lastEeState_.resize(num_arm_joints_);
        targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, robotName));
        ArmTargetTrajectoriesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(robotName + "_mpc_target_arm", 1);
        torsoTargetTrajectoriesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(robotName + "_mpc_target_pose", 1);

        armTargetPosePublisher_ = nodeHandle.advertise<kuavo_msgs::armTargetPoses>("/kuavo_arm_target_poses", 1, true);
        if (nodeHandle.hasParam("/only_half_up_body"))
        {
          nodeHandle.getParam("/only_half_up_body", only_half_up_body_);
        }

        vrCmdSub_ = nh_.subscribe("/kuavo_arm_traj", 10, &VRHandCommandNode::vrCmdCallback, this);
        observationSub_ = nh_.subscribe(robotName + "_wbc_observation", 10, &VRHandCommandNode::observationCallback, this);  
        auto eePoseCallback = [this](const kuavo_msgs::endEffectorData::ConstPtr& msg){
          std::lock_guard<std::mutex> lock(latestObservationMutex_);
          kuavo_msgs::endEffectorData eeState = *msg;
          lastEeState_.resize(eeState.position.size());
          for(size_t i = 0; i < eeState.position.size(); i++){
            lastEeState_(i) = static_cast<scalar_t>(eeState.position[i]);
          }
          if(!get_eepose_) get_eepose_ = true;
          // std::cout << "lastEeState_:  " << lastEeState_.transpose() << "\n\n";
        };
        eePoseSub_ = nh_.subscribe<kuavo_msgs::endEffectorData>("/humanoid_ee_State", 1, eePoseCallback);       
        eePoseTargetSub_ = nh_.subscribe<kuavo_msgs::armPoseWithTimeStamp>("/kuavo_hand_pose", 1, &VRHandCommandNode::eePoseTargetCallback, this);
        headBodyPoseSub_ = nh_.subscribe<kuavo_msgs::headBodyPose>("/kuavo_head_body_orientation", 1, &VRHandCommandNode::headBodyPoseCallback, this);
        changeArmCtrlModeSrv_ = nh_.advertiseService("change_arm_ctrl_mode", &VRHandCommandNode::changeArmCtlModeCallback, this);
        auto terrHeightCallback = [this](const std_msgs::Float64::ConstPtr& msg){
          terrainHeight_ = static_cast<scalar_t>(msg->data);
        };
        terrainHeightSub_ = nh_.subscribe<std_msgs::Float64>("/humanoid/mpc/terrainHeight", 10, terrHeightCallback);
      }
    void run()
    {
      // ros::spin();
      ROS_INFO("[VRHandCommandNode]: Waiting for first observation...");
      while (!get_observation_ && ros::ok())
      {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      ROS_INFO("[VRHandCommandNode]: First observation received.");
      ROS_INFO("[VRHandCommandNode]: Waiting for first ee pose...");
      while (!get_eepose_ && ros::ok())
      {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      ROS_INFO("[VRHandCommandNode]: First ee pose received.");

      auto rate = ros::Rate(100);
      while (ros::ok())
      {
        ros::spinOnce();
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTimeMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - last_callback_time_);
        if (updated_torso_ctrl_mode_ && elapsedTimeMilliseconds.count() > 2e3)// 2s未收到躯干数据则将躯干控制模式切换为0
        {
          if(callSetTorsoModeSrv(0))//6-dof
          {
            ROS_INFO("[VRHandCommandNode]: There is no new command for 2s, Switch to 6-dof TORSO control mode.");
            updated_torso_ctrl_mode_ = false;
          }
        }
        rate.sleep();
      }
      
    }
    // void setArmMode(const scalar_t& mode) { armCtlMode_ = mode; }
    void setArmCtlMode(const ArmCtlMode& mode) { armCtlMode_ = mode; }

  public:
    scalar_t estimateTimeToArmTarget(const vector_t &desiredArmDisplacement)
    {
      const scalar_t &dx = desiredArmDisplacement(0);
      const scalar_t &dy = desiredArmDisplacement(1);
      const scalar_t &dz = desiredArmDisplacement(2);
      const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
      const scalar_t displacementTime = displacement / targetArmDisplacementVelocity;
      return displacementTime;
    }

    void limitTargetPose(vector_t& targetPose)
    {
      targetPose(2) = std::max(0.3, std::min(targetPose(2), 0.9)); // height limit
      targetPose(4) = std::max(-5 * deg2rad, std::min(targetPose(4), 40 * deg2rad)); // pitch limit
      targetPose(5) = 0.0;
    }

    double normalized_yaw(double yaw)
    {
      while (yaw > M_PI)
        yaw -= 2*M_PI;
      while(yaw < -M_PI)
        yaw += 2*M_PI;
      return yaw;
    }
    /**
     * Converts the pose of the interactive marker to LeftHandTargetTrajectories.
    */
    TargetTrajectories goalHandPoseToTargetTrajectories(HandPose leftHandTargetPose, HandPose rightHandTargetPose, SystemObservation& observation
                                                        , const vector_t& lastEeState) {
      
      const vector_t currentPose = observation.state.segment<6>(6);
      vector_t currentArmPose = observation.state.segment(12+12,num_mpc_arm_joints_);

      vector_t targetPose = currentPose;
      // targetPose(0) = headBodyPose_.body_x;
      // targetPose(1) = headBodyPose_.body_y;
      targetPose(2) = headBodyPose_.body_height;
      double delta_yaw = headBodyPose_.body_yaw - normalized_yaw(currentPose(3));
      delta_yaw = normalized_yaw(delta_yaw);
      // targetPose(3) += delta_yaw;
      targetPose(4) = headBodyPose_.body_pitch;
      limitTargetPose(targetPose);
      // targetPose(4) = 6.0*deg2rad;
      leftHandTargetPose.addBaseOffset(targetPose.head<3>());
      rightHandTargetPose.addBaseOffset(targetPose.head<3>());
      // target reaching duration
      scalar_t leftHandTargetReachingTimeCost = estimateTimeToArmTarget(leftHandTargetPose.toVector() - lastEeState.segment<7>(0));
      scalar_t rightHandTargetReachingTimeCost = estimateTimeToArmTarget(rightHandTargetPose.toVector() - lastEeState.segment<7>(7));
      scalar_t targetReachingTimeCost = leftHandTargetReachingTimeCost > rightHandTargetReachingTimeCost ? leftHandTargetReachingTimeCost : rightHandTargetReachingTimeCost;
      scalar_t targetReachingTime = observation.time + targetReachingTimeCost;

      // desired time trajectory
      const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

      // desired state trajectory
      int stateDim = observation.state.size() + 1 + lastEeState.size();
      vector_array_t stateTrajectory(2, vector_t::Zero(stateDim));

      stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState, currentArmPose, armMode, lastEeState;
      switch (armCtlIdx_)
      {
      case ArmCtlIdx::Left:
        stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState, currentArmPose, armMode, leftHandTargetPose.toVector(), lastEeState.segment<7>(7);
        break;
      case ArmCtlIdx::Right:
        stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState, currentArmPose, armMode, lastEeState.segment<7>(0), rightHandTargetPose.toVector();
        break;
      case ArmCtlIdx::Both:
        stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState, currentArmPose, armMode, leftHandTargetPose.toVector(), rightHandTargetPose.toVector();
      default:
        break;
      }
      // desired input trajectory (just right dimensions, they are not used)
      const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

      return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    /**
     * Converts the joint state of the arm to LeftHandTargetTrajectories.
    */
    TargetTrajectories goalHandPoseToTargetTrajectories(const vector_t& armJointState, const SystemObservation& observation) {
      
      const vector_t currentPose = observation.state.segment<6>(6);
      vector_t currentArmPose = observation_.state.segment(12+12,num_arm_joints_);
      vector_t targetPose = currentPose;
      // targetPose(0) = headBodyPose_.body_x;
      // targetPose(1) = headBodyPose_.body_y;
      targetPose(2) = headBodyPose_.body_height;
      double delta_yaw = headBodyPose_.body_yaw - normalized_yaw(currentPose(3));
      delta_yaw = normalized_yaw(delta_yaw);
      // targetPose(3) += delta_yaw;
      targetPose(4) = headBodyPose_.body_pitch;
      limitTargetPose(targetPose);

      // target reaching duration
      const scalar_t targetReachingTime = observation.time + 0.01;

      // desired time trajectory
      const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

      // desired state trajectory
      vector_array_t stateTrajectory(2, vector_t::Zero(num_arm_joints_));
      stateTrajectory[0] << currentArmPose;
      stateTrajectory[1] << armJointState;

      // desired input trajectory (just right dimensions, they are not used)
      const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

      return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    scalar_t computeTorsoChangeTimeCost(const vector_t& currentPose, const vector_t& targetPose)
    {
      if(currentPose.size() != 6 || targetPose.size() != 6)
      {
        ROS_ERROR("currentPose or targetPose is not of size 6.");
        return 0.0;
      }
      const vector_t delta_pose = targetPose - currentPose;
      scalar_t time = 0.0;
      for(int i = 0; i < 6; i++)
      {
        time = std::max(time, std::abs(delta_pose(i)) / targetTorsoDispalcementVelocity(i));
        std::cout << "time [" << i << "]: " << time << "\n";
      }
      return time;
    }
    /**
     * Converts the torso pose state to (Torso)TargetTrajectories.
    */
    TargetTrajectories goalTorsoPoseToTargetTrajectories(const SystemObservation& observation) {
      
      const vector_t &currentPose = observation.state.segment<6>(6);

      vector_t targetPose = currentPose;
      double delta_x = headBodyPose_.body_x - currentPose(0);
      double delta_y = headBodyPose_.body_y - currentPose(1);
      double delta_z = headBodyPose_.body_height;
      double delta_yaw = normalized_yaw(headBodyPose_.body_yaw) - normalized_yaw(currentPose(3));
      delta_yaw = normalized_yaw(delta_yaw);

      // 限制最大位移 
      // targetPose(0) += std::max(-0.05, std::min(0.05, delta_x));
      // targetPose(1) += std::max(-0.05, std::min(0.05, delta_y));
      targetPose(2) = comHeight + std::max(-0.3, std::min(0.2, delta_z));
      // targetPose(3) += std::max(-0.01 * TO_DEGREE, std::min(0.01 * TO_DEGREE, delta_yaw));
      targetPose(4) = headBodyPose_.body_pitch;
      limitTargetPose(targetPose);
      targetPose(2) += terrainHeight_;
      // std::cout << "targetPose: " << targetPose.transpose() << "\n";

      // target reaching duration
      const scalar_t targetReachingTime = observation.time + computeTorsoChangeTimeCost(currentPose, targetPose);

      // desired time trajectory
      const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

      // desired state trajectory
      vector_array_t stateTrajectory(2, vector_t::Zero(6));
      stateTrajectory[0] << currentPose;
      stateTrajectory[1] << targetPose;

      // desired input trajectory (just right dimensions, they are not used)
      const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

      return {timeTrajectory, stateTrajectory, inputTrajectory};
    }


    void setInitArmPos(const Eigen::VectorXd& init_arm_pos){
      init_arm_pos_ = init_arm_pos;
    }

  private:
    void vrCmdCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
      // if(!enable_ctrl_){
      //   ROS_ERROR("Arm control is not enabled, can NOT response to vr command.");
      //   return;
      // }
      if (!get_observation_){
        ROS_ERROR("[VRHandCommandNode]: Can NOT response to vr command before first observation.");
        return;
      }
      if(armCtlMode_ != ArmCtlMode::JointSpace){
        ROS_ERROR("[VRHandCommandNode]: Arm control mode is not joint space, can NOT response to joint state target.");
        return;
      }
      if (num_arm_joints_ != msg->name.size())
      {
        ROS_ERROR("[VRHandCommandNode]:Received joint state target size: %d, number of arm joints: %d", msg->name.size(), num_arm_joints_);
        return;
      }
      vector_t armJointState(num_arm_joints_);
      for(size_t i = 0; i < msg->name.size(); i++){
        armJointState(i) = deg2rad * msg->position[i];
      }
      TargetTrajectories goalTargetTrajectories = goalHandPoseToTargetTrajectories(armJointState, observation_);
      const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(goalTargetTrajectories);

      ArmTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
      
      // targetTrajectoriesPublisherPtr_->publishTargetTrajectories(goalTargetTrajectories);
      last_joint_state_ = *msg;
    }
    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
    {
      observation_ = ros_msg_conversions::readObservationMsg(*observation_msg);
      if(!get_observation_){
        
        if (only_half_up_body_){
          // Control mode:
          // 0: Keep current arm pose
          // 1: Auto swing arms during walking
          // 2: External control through VR/teleoperation
          callSetArmModeSrv(ArmControlMode::EXTERN_CONTROL);
          backArmPoseToZero();
          ros::Duration(ArmToZeroTime).sleep();
          callSetArmModeSrv(ArmControlMode::KEEP);
        }
      }
      get_observation_ = true;
    }
    void headBodyPoseCallback(const kuavo_msgs::headBodyPose::ConstPtr& msg)
    {
      if(!updated_torso_ctrl_mode_)
      {
        if(callSetTorsoModeSrv(2)) // control height and pitch
          updated_torso_ctrl_mode_ = true;
      }
      last_callback_time_ = std::chrono::steady_clock::now();

      headBodyPose_.body_yaw = msg->body_yaw;
      headBodyPose_.body_pitch = msg->body_pitch;
      headBodyPose_.body_x = msg->body_x;
      headBodyPose_.body_y = msg->body_y;
      headBodyPose_.body_height = msg->body_height;

      headBodyPose_.head_pitch = msg->head_pitch;
      headBodyPose_.head_yaw = msg->head_yaw;
      // bodyYaw_ = std::min(60*deg2rad, std::max(-60*deg2rad, bodyYaw_));//limit the body yaw to [-60, 60] degree
      // std::cout << "bodyYaw:  " << headBodyPose_.body_yaw << "\n\n";

      TargetTrajectories goalTargetTrajectories = goalTorsoPoseToTargetTrajectories(observation_);
      const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(goalTargetTrajectories);
      torsoTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
    }

    void backArmPoseToZero(){

      if (init_arm_pos_.size() != num_arm_joints_){
        ROS_ERROR("[VRHandCommandNode]: init_arm_pos_ size: %d, num_arm_joints_: %d", init_arm_pos_.size(), num_arm_joints_);
        return;
      }
      kuavo_msgs::armTargetPoses arm_target_poses;
      arm_target_poses.times.clear();
      arm_target_poses.values.clear();
      
      // 3 seconds has tested in real robot
      arm_target_poses.times.push_back(ArmToZeroTime);
      
      arm_target_poses.values.resize(init_arm_pos_.size());
      for(int i = 0; i < init_arm_pos_.size(); i++) {
        arm_target_poses.values[i] = init_arm_pos_(i);
      }
      while(armTargetPosePublisher_.getNumSubscribers() == 0 && ros::ok()){
        ROS_INFO_THROTTLE(1, "[VRHandCommandNode]: Waiting for /kuavo_arm_target_pose subscribers");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
      armTargetPosePublisher_.publish(arm_target_poses);
      // 半身模式下 /kuavo_arm_target_pose 会替换掉初始数据，所以初始发两次确保发送成功
      armTargetPosePublisher_.publish(arm_target_poses);
    }

    bool changeArmCtlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
    {
        if (!get_observation_){
          ROS_ERROR("[VRHandCommandNode]: Arm control mode can NOT be changed before first observation.");
          return false;
        }
        ArmControlMode control_mode = static_cast<ArmControlMode>(req.control_mode);

        // Control mode:
        // 0: Keep current arm pose
        // 1: Auto swing arms during walking
        // 2: External control through VR/teleoperation
        if(only_half_up_body_ &&
          control_mode == ArmControlMode::AUTO_SWING && last_arm_control_mode_ == ArmControlMode::EXTERN_CONTROL) {

          backArmPoseToZero();
        }

        enable_ctrl_ = (control_mode != ArmControlMode::KEEP);
        last_arm_control_mode_ = control_mode;
        res.result = true;
        std::cout << "Arm control mode changed to " << static_cast<int>(control_mode) << "\n";
        callSetArmModeSrv(control_mode);
        return true;
    }
    void callSetArmModeSrv(ArmControlMode mode)
    {
      kuavo_msgs::changeArmCtrlMode srv;
      srv.request.control_mode = static_cast<int>(mode);
      auto change_arm_mode_service_client_ = nh_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");

      // 调用设置arm_mode_changing的服务
      std_srvs::Trigger trigger_srv;
      
      // Control mode:
      // 0: Keep current arm pose
      // 1: Auto swing arms during walking
      // 2: External control through VR/teleoperation
      if (mode == ArmControlMode::EXTERN_CONTROL && ros::service::exists("/quest3/set_arm_mode_changing", false)) {
        
        auto set_arm_mode_changing_client_ = nh_.serviceClient<std_srvs::Trigger>("/quest3/set_arm_mode_changing");
        if (set_arm_mode_changing_client_.call(trigger_srv))
        {
          ROS_INFO("Set arm mode changing service call successful");
        }
        else
        {
          ROS_ERROR("Failed to call set arm mode changing service");
        }
      }
      else ROS_WARN("Service /quest3/set_arm_mode_changing skipping call");
      
      // 调用服务
      if (change_arm_mode_service_client_.call(srv))
      {
        ROS_INFO("SetArmModeSrv call successful");
      }
      else
      {
        ROS_ERROR("Failed to call SetArmModeSrv");
      }
    }

    bool callSetTorsoModeSrv(int32_t mode)
    {
      kuavo_msgs::changeTorsoCtrlMode srv;
      srv.request.control_mode = mode;
      auto change_torso_mode_service_client_ = nh_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/humanoid_change_torso_ctrl_mode");

      bool success = change_torso_mode_service_client_.call(srv);
      // 调用服务
      if (success)
      {
        ROS_INFO("SetTorsoModeSrv call successful");
      }
      else
      {
        ROS_ERROR("Failed to call SetTorsoModeSrv");
      }
      return success;
    }

    void eePoseTargetCallback(const kuavo_msgs::armPoseWithTimeStamp::ConstPtr& msg)
    {
      if(!enable_ctrl_){
        ROS_ERROR("Arm control is not enabled, can NOT response to vr command.");
        return;
      }
      if(armCtlMode_ != ArmCtlMode::TaskSpace){
        ROS_ERROR("Arm control mode is not task space, can NOT response to ee pose target.");
        return;
      }
      if(msg->left_hand_pose.size() != 7 || msg->right_hand_pose.size() != 7)
      {
        // ROS_ERROR("Invalid left or right hand pose size.");
        return;
      }
      HandPose leftHandTargetPose;
      HandPose rightHandTargetPose;

      leftHandTargetPose.orientation.x() = msg->left_hand_pose[0];
      leftHandTargetPose.orientation.y() = msg->left_hand_pose[1];
      leftHandTargetPose.orientation.z() = msg->left_hand_pose[2];
      leftHandTargetPose.orientation.w() = msg->left_hand_pose[3];
      leftHandTargetPose.position(0) = msg->left_hand_pose[4];
      leftHandTargetPose.position(1) = msg->left_hand_pose[5];
      leftHandTargetPose.position(2) = msg->left_hand_pose[6];
      // std::cout << "left hand target position: " << leftHandTargetPose.position.transpose() << "\n\n";

      rightHandTargetPose.orientation.x() = msg->right_hand_pose[0];
      rightHandTargetPose.orientation.y() = msg->right_hand_pose[1];
      rightHandTargetPose.orientation.z() = msg->right_hand_pose[2];
      rightHandTargetPose.orientation.w() = msg->right_hand_pose[3];
      rightHandTargetPose.position(0) = msg->right_hand_pose[4];
      rightHandTargetPose.position(1) = msg->right_hand_pose[5];
      rightHandTargetPose.position(2) = msg->right_hand_pose[6];
      // std::cout << "right hand target position: " << rightHandTargetPose.position.transpose() << "\n\n";

      TargetTrajectories goalTargetTrajectories = goalHandPoseToTargetTrajectories(leftHandTargetPose, rightHandTargetPose, observation_, lastEeState_);
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(goalTargetTrajectories);
    }

    ros::NodeHandle nh_;
    ArmCtlMode armCtlMode_;
    ArmCtlIdx armCtlIdx_;
    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    ros::Subscriber vrCmdSub_;
    ros::Subscriber observationSub_;
    ros::Subscriber eePoseSub_;
    ros::Subscriber eePoseTargetSub_;
    ros::Subscriber headBodyPoseSub_;
    ros::Subscriber terrainHeightSub_;
    ros::Publisher ArmTargetTrajectoriesPublisher_;
    ros::Publisher torsoTargetTrajectoriesPublisher_;

    ros::ServiceServer changeArmCtrlModeSrv_;
    ocs2::SystemObservation observation_;
    ocs2::SystemObservation lastObservation_;
    int num_arm_joints_ = 14;
    int num_mpc_arm_joints_ = 14; // number of arm joints for mpc
    int half_num_arm_joints_;
    int half_num_mpc_arm_joints_;
    double terrainHeight_ = 0.0;
    vector_t lastEeState_;
    bool get_observation_ = false;
    bool get_eepose_ = false;
    // double bodyYaw_{0.0};
    HeadBodyPose headBodyPose_;
    mutable std::mutex latestObservationMutex_;
    bool enable_ctrl_{false};
    bool updated_torso_ctrl_mode_{false};
    // 添加记录上一次进回调的时间戳变量
    std::chrono::steady_clock::time_point last_callback_time_;

    ros::Publisher armTargetPosePublisher_;
    bool only_half_up_body_{false};
    ArmControlMode last_arm_control_mode_{ArmControlMode::KEEP};
    sensor_msgs::JointState last_joint_state_;
    // 手臂初始位置
    vector_t init_arm_pos_;

    // 手臂碰撞检查控制，如果 true，表示当前发生了手臂碰撞，正在执行手臂归位，不发送手臂目标轨迹
    bool arm_collision_check_control_{false};
};


int main(int argc, char* argv[]) {
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_VR_hand_target");
  ::ros::NodeHandle nodeHandle;

  // Get node parameters
  std::string referenceFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  RobotVersion rb_version(4, 0);
  if (nodeHandle.hasParam("/robot_version"))
  {
      int rb_version_int;
      nodeHandle.getParam("/robot_version", rb_version_int);
      rb_version = RobotVersion::create(rb_version_int);
  }
  auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
  defaultJointState = drake_interface_->getDefaultJointState();
  // comHeight = drake_interface_->getIntialHeight();
  // ros::param::set("/com_height", comHeight);
  // loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadCppDataType(referenceFile, "targetArmDisplacementVelocity", targetArmDisplacementVelocity);
  loadData::loadEigenMatrix(referenceFile, "targetTorsoDispalcementVelocity", targetTorsoDispalcementVelocity);

  VRHandCommandNode node(nodeHandle, ArmCtlMode::JointSpace, ArmCtlIdx::Both);
  node.setInitArmPos(drake_interface_->getKuavoSettings().predefined_arm_pose.init_arm_pos);
  node.run();
  return 0;
}