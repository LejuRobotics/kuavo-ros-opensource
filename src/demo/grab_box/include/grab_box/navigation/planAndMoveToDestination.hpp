#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Core>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <grab_box/utils/customIoPort.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "kuavo_msgs/changeTorsoCtrlMode.h"
#include <kuavo_msgs/gaitTimeName.h>
#include "humanoid_interface/gait/GaitSchedule.h"
#include <humanoid_interface/gait/MotionPhaseDefinition.h>

namespace GrabBox
{
  enum PlanMoveType
  {
    PLAN_ALL_DOF = 0,
    PLAN_HEAD_FORWARD = 1,
    PLAN_NORMAL_POSE = 2,
    PLAN_TURNING_POSE = 3,
  };

  std::string moveTypeToString(PlanMoveType move_type)
  {
    switch(move_type)
    {
      case PLAN_ALL_DOF:
        return "PLAN_ALL_DOF";
      case PLAN_HEAD_FORWARD:
        return "PLAN_HEAD_FORWARD";
      case PLAN_NORMAL_POSE:
        return "PLAN_NORMAL_POSE";
      case PLAN_TURNING_POSE:
        return "PLAN_TURNING_POSE";
      default:
        return "UNKNOWN";
    }
  }

  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  using namespace std::chrono;
  // Example of Asynchronous node that uses StatefulActionNode as base class
  class PlanAndMoveToDestination : public BT::StatefulActionNode
  {
  public:
    PlanAndMoveToDestination(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    {
      ros::NodeHandle nh;
      torsoTargetTrajectoriesPublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>("/humanoid_mpc_target_pose", 1);
      modeSequenceTemplatePublisher_ = nh.advertise<ocs2_msgs::mode_schedule>("/humanoid_mpc_mode_schedule", 10, true);
      gait_time_name_sub_ = nh.subscribe("/humanoid_mpc_gait_time_name", 10, &PlanAndMoveToDestination::gaitTimeNameCallback, this);

      // feet subscriber
      auto feetCallback = [this](const std_msgs::Float64MultiArray::ConstPtr &feet_msg)
      {
        feet_pos_measured_ = Eigen::Map<const Eigen::VectorXd>(feet_msg->data.data(), feet_msg->data.size());
      };
      feet_sub_ = nh.subscribe<std_msgs::Float64MultiArray>("/humanoid_controller/swing_leg/pos_measured", 2, feetCallback);
      

      // gait
      std::string gaitCommandFile;
      nh.getParam("/gaitCommandFile", gaitCommandFile);
      ROS_INFO_STREAM("/humanoid_mpc_mode_schedule node is setting up ...");
      std::vector<std::string> gaitList;
      ocs2::loadData::loadStdVector(gaitCommandFile, "list", gaitList, false);
      gait_map_.clear();
      for (const auto &gaitName : gaitList)
      {
        gait_map_.insert({gaitName, ocs2::humanoid::loadModeSequenceTemplate(gaitCommandFile, gaitName, false)});
      }
      torso_displacement_vel_ = getParamsFromBlackboard<std::vector<double>>(config, "PlanAndMoveToDestination.torso_displacement_vel");
      gait_duration_ = getParamsFromBlackboard<double>(config, "PlanAndMoveToDestination.gait_duration");
      stance_pos_threshold_ = getParamsFromBlackboard<double>(config, "PlanAndMoveToDestination.stance_pos_threshold");
      stance_yaw_threshold_ = getParamsFromBlackboard<double>(config, "PlanAndMoveToDestination.stance_yaw_threshold");
      stance_feet_diff_threshold_ = getParamsFromBlackboard<double>(config, "PlanAndMoveToDestination.stance_feet_diff_threshold");
      gaitManagerPtr_ = new ocs2::humanoid::GaitManager(20);

    }
    void gaitTimeNameCallback(kuavo_msgs::gaitTimeName::ConstPtr msg)
    {
        // current_gait_ = msg->gait_name;
        gaitManagerPtr_->add(msg->start_time, msg->gait_name);
        std::cout << "current gait name : " << current_gait_ << std::endl;
    }

    static BT::PortsList providedPorts()
    {
      // amount of milliseconds that we want to sleep
      return{
        BT::InputPort<Eigen::Vector3d>("box_pos"),
        BT::InputPort<Eigen::Vector4d>("box_quat"),
        BT::InputPort<Eigen::Vector3d>("target_box_offset"),
        BT::InputPort<int>("move_type") };
    }

    BT::NodeStatus onStart() override
    {
      if(!config().blackboard->get("current_time", start_time_))
      {
        ROS_ERROR("Failed to retrieve current_time from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      // 获取 box_pos 和 box_quat 
      Eigen::Vector3d box_pos;
      Eigen::Vector4d box_quat;
      if (!getInput("box_pos", box_pos) || !getInput("box_quat", box_quat) || !getInput("target_box_offset", target_box_offset_)) {
        ROS_ERROR("Failed to get input ports for box_pos or box_quat");
        return BT::NodeStatus::FAILURE;
      }
      box_pose_world_.resize(7);
      box_pose_world_ << box_pos(0), box_pos(1), box_pos(2), box_quat(0), box_quat(1), box_quat(2), box_quat(3);

      computeTargertPose(box_pose_world_, target_box_offset_, target_pose_);


      // std::cout << "start_time: " << start_time_ << "\n";
      end_time_ = start_time_ + 2 * gait_duration_; // make sure end_time_ is after (start_time_ + gait_duration_)
      if(callSetTorsoModeSrv(0))//6-dof
      {
        ROS_INFO("SetTorsoModeSrv call successful, control mode is 6-dof");
      }
      getInput<int>("move_type", move_type_);
      std::cout << "[PlanAndMoveToDestination] move_type: " << moveTypeToString(static_cast<PlanMoveType>(move_type_)) << "\n";
      if(move_type_ != PlanMoveType::PLAN_NORMAL_POSE)
        publishGaitTemplate("walk");
      traj_published_ = false;
      while(!ros::param::has("/com_height"))
      {
        ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      ROS_INFO_STREAM("com_height parameter is founded.");
      ros::param::get("/com_height", com_height_);
      std::cout << "[PlanAndMoveToDestination] comHeight: " << com_height_ <<std::endl;
      // limitArmControlMode();

      return BT::NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus onRunning() override
    {
      if(!config().blackboard->get("current_time", current_time_))
      {
        ROS_ERROR("Failed to retrieve current_time from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      if(!config().blackboard->get("current_mode", current_mode_))
      {
        ROS_ERROR("Failed to retrieve current_mode from blackboard");
        return BT::NodeStatus::FAILURE;
      }

      if(!config().blackboard->get("ocs2_state", ocs2_state_))
      {
        ROS_ERROR("Failed to retrieve ocs2_state from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      if(!config().blackboard->get("tag_pose_world", box_pose_world_))
      {
        ROS_ERROR("Failed to retrieve box_pose_world_ from blackboard");
        return BT::NodeStatus::FAILURE;
      }

      current_gait_ = gaitManagerPtr_->getGaitName(current_time_);
      if (current_gait_ != "walk")
      {
        if (!send_walk_gait_)
        {
          publishGaitTemplate("walk");
          send_walk_gait_ = true;
          std::cout << "send gait template: walk" << std::endl;
        }
        return BT::NodeStatus::RUNNING;
      }
      auto current_pose = ocs2_state_.segment<6>(6);
      bool res = checkFeetContactPos(current_pose, current_mode_);
      if(checkReachTarget() && res)
      {
        if(move_type_ != PlanMoveType::PLAN_NORMAL_POSE)
          publishGaitTemplate("stance");
        return BT::NodeStatus::SUCCESS;
      }
      else if (current_time_ - last_pub_time_ > 1/freq_ && current_time_ > start_time_ + gait_duration_)
      {
        last_pub_time_ = current_time_;
        computeAndPublishTargetTrajectories();
      }

      // ROS_INFO_STREAM("PlanAndMoveToDestination still running, time left: " << end_time_ + gait_duration_ - current_time);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "PlanAndMoveToDestination interrupted" << std::endl;
    }
  private:
    bool checkFeetContactPos(const ocs2::vector_t &currentPose, size_t current_mode)
    {
      std::cerr << "checkFeetContactPos" << std::endl;
      Eigen::Vector3d lf_pos_w = Eigen::Vector3d::Zero();
      Eigen::Vector3d rf_pos_w = Eigen::Vector3d::Zero();
      for (int i = 0; i < 4; i++)
      {
        lf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3) / 4;
        rf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3 + 12) / 4;
      }

      Eigen::Matrix<ocs2::scalar_t, 3, 1> zyx;
      zyx << -currentPose.tail(3)[0], 0, 0;
      Eigen::Vector3d lf = ocs2::getRotationMatrixFromZyxEulerAngles(zyx) * lf_pos_w;
      Eigen::Vector3d rf = ocs2::getRotationMatrixFromZyxEulerAngles(zyx) * rf_pos_w;
      if (current_mode == ocs2::humanoid::ModeNumber::SS && std::abs(lf(0) - rf(0)) < stance_feet_diff_threshold_)
        return true;
      return false;
    }
    bool checkReachTarget()
    {
      Vector4d currentPose = ocs2_state_.segment<4>(6);
      currentPose[3] *= 180.0 / M_PI;

      double distance = std::sqrt(std::pow(target_pose_(0) - currentPose(0), 2) + std::pow(target_pose_(1) - currentPose(1), 2));
      double dyaw = abs(target_pose_(3) - currentPose(3));
      std::cout << "distance: " << distance << ", dyaw: " << dyaw << "\n";
      if(distance < stance_pos_threshold_ && dyaw < stance_yaw_threshold_)
      {
        return true;
      }
      return false;
    }
    /**
     * Converts the torsoPoseTrajectory to (Torso)TargetTrajectories.
    */
    void computeAndPublishTargetTrajectories()
    {
      torsoPoseTraj planed_trajectory;
      computeTargertPose(box_pose_world_, target_box_offset_, target_pose_);
      planed_trajectory.push_back(target_pose_);
      Vector6d currentPose = Vector6d::Zero();
      if (!traj_published_) // 使用一次实际位置
      {
        currentPose.head(6) = ocs2_state_.segment<6>(6);
      }
      else
      {
        auto state = lastGoalTargetTrajectories_.getDesiredState(current_time_);
        currentPose.head(6) = state.segment<6>(0);
      }
      std::cout << "current_time_: " << current_time_ << "\n";
      std::cout << "currentPose: " << currentPose.transpose() << "\n";
      std::cout << "targetPose: " << target_pose_.transpose() << "\n";

      lastGoalTargetTrajectories_ = goalTorsoPoseToTargetTrajectories(currentPose, current_time_, planed_trajectory);
      const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(lastGoalTargetTrajectories_);
      torsoTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
      end_time_ = lastGoalTargetTrajectories_.timeTrajectory.back();
      traj_published_ = true;

    }
    ocs2::TargetTrajectories goalTorsoPoseToTargetTrajectories(const Vector6d& currentPose, double currentTime, const torsoPoseTraj& torsoPoseTrajectory) 
    {
      Vector6d targetPose = currentPose;
      const int num = torsoPoseTrajectory.size();

      // desired time trajectory
      ocs2::scalar_array_t timeTrajectory(num+1, currentTime);
      ocs2::vector_array_t stateTrajectory(num+1, ocs2::vector_t::Zero(6));
      stateTrajectory[0] = currentPose;

      double pitch_ref = M_PI / 180.0 * getParamsFromBlackboard<double>(config(), "normal_torso_pitch");
      
      double terrain_height=0.0;
      config().blackboard->get("terrain_height", terrain_height);
      std::cout << "[PlanAndMoveToDestination] terrain_height: " << terrain_height << "\n";
      for(int i = 1; i < num+1; i++)
      {
        targetPose.head(4) = torsoPoseTrajectory[i-1];// xyz,yaw
        targetPose(2) = com_height_ + terrain_height;       //height
        targetPose(3) *= M_PI / 180.0;// convert to radian
        targetPose(4) = pitch_ref;         //pitch
        targetPose(5) = 0.0;               //roll
        if(move_type_ == PlanMoveType::PLAN_HEAD_FORWARD)
        {
          targetPose.head(4) = torsoPoseTrajectory[i-1];// xyz,yaw
          targetPose(2) = com_height_ + terrain_height;       //height
          targetPose(3) = ocs2_state_(9);    //using current yaw 
          targetPose(4) = pitch_ref;         //pitch
          ROS_INFO_STREAM("targetPose["<< i << "]: " << targetPose.transpose() << "\n");
        }
        if(move_type_ == PlanMoveType::PLAN_NORMAL_POSE)
        {
          targetPose = ocs2_state_.segment<6>(6);
          targetPose(2) = com_height_ + terrain_height;      //height
          targetPose(4) = pitch_ref;        //pitch
          targetPose(5) = 0.0;              //roll
        }
        if (move_type_ == PlanMoveType::PLAN_TURNING_POSE)
        {
          Vector4d currentTagetDelta = torsoPoseTrajectory[i-1].head(4) - ocs2_state_.segment<4>(6);
          targetPose.head(4) = ocs2_state_.segment<4>(6);
          targetPose(2) = com_height_ + terrain_height;       //height
          // targetPose(3) *= M_PI / 180.0;// convert to radian
          targetPose(3) = atan2(currentTagetDelta(1), currentTagetDelta(0));
          targetPose(4) = pitch_ref;         //pitch
          targetPose(5) = 0.0;               //roll
        }
        limitTargetPose(targetPose);
        // target reaching duration
        const double time_cost = computeTorsoChangeTimeCost(currentPose, targetPose);
        std::cout << "time_cost: " << time_cost << "\n";
        timeTrajectory[i] = currentTime + time_cost;
        stateTrajectory[i] = targetPose;
        std::cout << "targetPose["<< timeTrajectory[i] << "]: " << targetPose.transpose() << "\n";
      }
      // desired input trajectory (just right dimensions, they are not used)
      const ocs2::vector_array_t inputTrajectory(num+1, ocs2::vector_t::Zero(2));

      return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    double computeTorsoChangeTimeCost(const ocs2::vector_t& currentPose, const ocs2::vector_t& targetPose)
    {
      if(currentPose.size() != 6 || targetPose.size() != 6)
      {
        ROS_ERROR("currentPose or targetPose is not of size 6.");
        return 0.0;
      }
      const ocs2::vector_t delta_pose = targetPose - currentPose;

      ocs2::vector_t delta_pose_local = ocs2::vector_t::Zero(6);
      double yaw_local_rad = currentPose(3);
      delta_pose_local(0) = delta_pose(0) * std::cos(yaw_local_rad) + delta_pose(1) * std::sin(yaw_local_rad);
      delta_pose_local(1) = -delta_pose(0) * std::sin(yaw_local_rad) + delta_pose(1) * std::cos(yaw_local_rad);
      delta_pose_local(2) = delta_pose(2);
      delta_pose_local(3) = delta_pose(3);
      double time = 0.0;
      // std::cout << "delta_pose_local: " << delta_pose_local.transpose() << "\n";
      // std::cout << "detla_pose: " << delta_pose.transpose() << "\n";
      // std::cout << "currentPose(3) : " << currentPose(3) << "\n";

      int max_index = -1;
      if (delta_pose_local[0] < 0) {
        time = std::abs(delta_pose_local[0]) / torso_displacement_vel_[6];
        max_index = 6;
      } // 对X负方向的移动时间计算单独处理

      for(int i = 0; i < 6; i++) {
          // 计算当前的时间
          double current_time = std::abs(delta_pose_local[i]) / torso_displacement_vel_[i];
          
          // 如果当前时间大于已记录的最大时间，则更新最大时间和对应的索引
          if(current_time > time) {
              time = current_time;
              max_index = i;
          }

          // 如果需要，可以在这里打印每个时间的值
          // std::cout << "time [" << i << "]: " << current_time << "\n";
      }

      // 检查是否找到了有效的最大时间
      if(max_index != -1) {
          std::cout << "最大时间: " << time 
                    << ", 对应的 torso_displacement_vel_[" << max_index << "]: " 
                    << torso_displacement_vel_[max_index] << "\n";
      } else {
          std::cout << "未找到有效的时间值。" << std::endl;
      }

      // for(int i = 0; i < 6; i++)
      // {
      //   time = std::max(time, std::abs(delta_pose_local(i)) / torso_displacement_vel_[i]);
      //   // std::cout << "time [" << i << "]: " << time << "\n";
      // }
      return time;
    }

    void limitTargetPose(Vector6d& targetPose)
    {
      const double deg2rad = M_PI / 180.0;
      // targetPose(2) = std::max(0.3, std::min(targetPose(2), 0.9)); // height limit
      targetPose(4) = std::max(-5 * deg2rad, std::min(targetPose(4), 40 * deg2rad)); // pitch limit
      targetPose(5) = 0.0;
    }

    void publishGaitTemplate(const std::string &gaitName)
    {
      // 发布对应的gait模板
      ocs2::humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
      modeSequenceTemplatePublisher_.publish(ocs2::humanoid::createModeSequenceTemplateMsg(modeSequenceTemplate));
      // ros::Duration(0.5).sleep();
      // current_gait_ = gaitName;
    }
    bool callSetTorsoModeSrv(int32_t mode)
    {
      ros::NodeHandle nh;
      kuavo_msgs::changeTorsoCtrlMode srv;
      srv.request.control_mode = mode;
      auto change_torso_mode_service_client_ = nh.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/humanoid_change_torso_ctrl_mode");

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
  private:
    void computeTargertPose(const Eigen::VectorXd& box_pose_world, const Eigen::Vector3d& target_box_offset, Eigen::Vector4d& target_pose)
    {
      // 计算目标位姿

      std::cout << "box_pose_world_ : " << box_pose_world.transpose() << std::endl;

      Eigen::VectorXd destination_pose_box(7);
      destination_pose_box << target_box_offset(0), target_box_offset(1), target_box_offset(2), 0.0, 0.0, 0.0, 1.0;
    
    //根据设定的相对位置和BOX生成目标位置
      Eigen::VectorXd destination_pose_ = getDestination(destination_pose_box, box_pose_world);

      // std::cout << "destination_pose_:  " << destination_pose_.transpose() << std::endl;

      // 创建目标位姿的四维数组（X, Y, Z, YAW）
      target_pose << destination_pose_(0), destination_pose_(1), com_height_, getYawFromQuaternion(destination_pose_.segment<4>(3)) * 180.0 / M_PI;

    }

   Eigen::VectorXd getDestination(const Eigen::VectorXd& destination_pose_box, const Eigen::VectorXd& box_pose)
    {
      return PoseTransformer::transformPoseToWorld(destination_pose_box, box_pose);
    }

    // 使用四元数计算 Yaw
    double getYawFromQuaternion(const Eigen::Vector4d& quat)
    {
      Eigen::Quaterniond q(quat(3), quat(0), quat(1), quat(2));
      return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }


    std::map<std::string, ocs2::humanoid::ModeSequenceTemplate> gait_map_;
    ros::Publisher torsoTargetTrajectoriesPublisher_;
    ros::Publisher modeSequenceTemplatePublisher_;
    double gait_duration_{1.0};
    double start_time_{0};
    double end_time_{0};
    bool traj_published_{false};
    int move_type_ = PlanMoveType::PLAN_ALL_DOF;
    Eigen::VectorXd ocs2_state_;
    double com_height_;
    std::vector<double> torso_displacement_vel_;
    double delta_roll = 0.0, delta_pitch = M_PI/2, delta_yaw = M_PI/2;
    Eigen::VectorXd box_pose_world_;
    Eigen::Vector3d target_box_offset_;
    Eigen::Vector4d target_pose_;
    double current_time_ = 0.0;
    size_t current_mode_ = ocs2::humanoid::ModeNumber::SS;
    ros::Subscriber gait_time_name_sub_;
    ros::Subscriber feet_sub_;
    ocs2::vector_t feet_pos_measured_ = ocs2::vector_t::Zero(24);
    string current_gait_= "stance";
    double freq_ = 10.0;
    double last_pub_time_ = 0.0;
    ocs2::TargetTrajectories lastGoalTargetTrajectories_;
    double stance_pos_threshold_ = 0.01;
    double stance_yaw_threshold_ = 5.0;
    double stance_feet_diff_threshold_ = 0.08;
    ocs2::humanoid::GaitManager *gaitManagerPtr_=nullptr;
    bool send_walk_gait_ = false;
  };
} // namespace GrabBox
