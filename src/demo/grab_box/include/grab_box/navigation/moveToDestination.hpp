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

namespace GrabBox
{
  enum MoveType
  {
    ALL_DOF = 0,
    HEAD_FORWARD = 1,
    NORMAL_POSE = 2,
    TURNING_POSE = 3,
  };

  std::string moveTypeToString(MoveType move_type)
  {
    switch(move_type)
    {
      case ALL_DOF:
        return "ALL_DOF";
      case HEAD_FORWARD:
        return "HEAD_FORWARD";
      case NORMAL_POSE:
        return "NORMAL_POSE";
      case TURNING_POSE:
        return "TURNING_POSE";
      default:
        return "UNKNOWN";
    }
  }

  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  using namespace std::chrono;
  // Example of Asynchronous node that uses StatefulActionNode as base class
  class MoveToDestination : public BT::StatefulActionNode
  {
  public:
    MoveToDestination(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    {
      ros::NodeHandle nh;
      torsoTargetTrajectoriesPublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>("/humanoid_mpc_target_pose", 1);
      modeSequenceTemplatePublisher_ = nh.advertise<ocs2_msgs::mode_schedule>("/humanoid_mpc_mode_schedule", 10, true);
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
      torso_displacement_vel_ = getParamsFromBlackboard<std::vector<double>>(config, "moveToDestination.torso_displacement_vel");
      gait_duration_ = getParamsFromBlackboard<double>(config, "moveToDestination.gait_duration");
    }

    static BT::PortsList providedPorts()
    {
      // amount of milliseconds that we want to sleep
      return{ BT::InputPort<torsoPoseTraj>("planed_trajectory"), BT::InputPort<int>("move_type") };
    }

    BT::NodeStatus onStart() override
    {
      if(!config().blackboard->get("current_time", start_time_))
      {
        ROS_ERROR("Failed to retrieve current_time from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      // std::cout << "start_time: " << start_time_ << "\n";
      end_time_ = start_time_ + 2 * gait_duration_; // make sure end_time_ is after (start_time_ + gait_duration_)
      if(callSetTorsoModeSrv(0))//6-dof
      {
        ROS_INFO("SetTorsoModeSrv call successful, control mode is 6-dof");
      }
      getInput<int>("move_type", move_type_);
      std::cout << "[MoveToDestination] move_type: " << moveTypeToString(static_cast<MoveType>(move_type_)) << "\n";
      if(move_type_ != MoveType::NORMAL_POSE)
        publishGaitTemplate("walk");
      traj_published_ = false;
      while(!ros::param::has("/com_height"))
      {
        ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      ROS_INFO_STREAM("com_height parameter is founded.");
      ros::param::get("/com_height", com_height_);
      std::cout << "[MoveToDestination] comHeight: " << com_height_ <<std::endl;
      // limitArmControlMode();

      return BT::NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    BT::NodeStatus onRunning() override
    {
      torsoPoseTraj traj;
      getInput<torsoPoseTraj>("planed_trajectory", traj);
      double current_time = 0.0;
      if(!config().blackboard->get("current_time", current_time))
      {
        ROS_ERROR("Failed to retrieve current_time from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      if(!config().blackboard->get("ocs2_state", ocs2_state_))
      {
        ROS_ERROR("Failed to retrieve ocs2_state from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      // std::cout << "current_time: " << current_time << "\n";
      const double plus_time = (move_type_ == MoveType::NORMAL_POSE ? 0 : gait_duration_);
      if(current_time < start_time_ + plus_time)
      {
        // ROS_INFO_STREAM("Waiting walk become stable, time left: " << start_time_ + plus_time - current_time);
        if(move_type_ != MoveType::NORMAL_POSE)
          return BT::NodeStatus::RUNNING;
      }
      if(!traj_published_)
      {
        traj_published_ = true;
        torsoPoseTraj planed_trajectory;
        getInput("planed_trajectory", planed_trajectory);

        Vector6d currentPose = Vector6d::Zero();
        torsoPose torso_pose;
        // double current_time = 0.0;
        if(!config().blackboard->get("torso_pose", torso_pose))
        {
          ROS_ERROR("Failed to retrieve torso_pose from blackboard");
          return BT::NodeStatus::FAILURE;
        }
        // currentPose.head(4) = ocs2_state_.segment<4>(6);
        currentPose.head(6) = ocs2_state_.segment<6>(6);
        // std::cout << "MoveToDestination ocs2_state_: " << ocs2_state_.transpose() << "\n";

        ocs2::TargetTrajectories goalTargetTrajectories = goalTorsoPoseToTargetTrajectories(currentPose, current_time, planed_trajectory);
        const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(goalTargetTrajectories);
        torsoTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
        end_time_ = goalTargetTrajectories.timeTrajectory.back();
        std::cout << "end_time: " << end_time_ << "\n";
        // if(move_type_ == MoveType::NORMAL_POSE)
        //   return BT::NodeStatus::SUCCESS;
      }
      if(current_time > end_time_ + plus_time)
      {
        if(move_type_ != MoveType::NORMAL_POSE)
          publishGaitTemplate("stance");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        // ROS_INFO_STREAM("MoveToDestination still running, time left: " << end_time_ + gait_duration_ - current_time);
        return BT::NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "MoveToDestination interrupted" << std::endl;
    }
  private:
    /**
     * Converts the torsoPoseTrajectory to (Torso)TargetTrajectories.
    */
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
      std::cout << "[MoveToDestination] terrain_height: " << terrain_height << "\n";
      for(int i = 1; i < num+1; i++)
      {
        targetPose.head(4) = torsoPoseTrajectory[i-1];// xyz,yaw
        targetPose(2) = com_height_ + terrain_height;       //height
        targetPose(3) *= M_PI / 180.0;// convert to radian
        targetPose(4) = pitch_ref;         //pitch
        targetPose(5) = 0.0;               //roll
        if(move_type_ == MoveType::HEAD_FORWARD)
        {
          targetPose.head(4) = torsoPoseTrajectory[i-1];// xyz,yaw
          targetPose(2) = com_height_ + terrain_height;       //height
          targetPose(3) = ocs2_state_(9);    //using current yaw 
          targetPose(4) = pitch_ref;         //pitch
          ROS_INFO_STREAM("targetPose["<< i << "]: " << targetPose.transpose() << "\n");
        }
        if(move_type_ == MoveType::NORMAL_POSE)
        {
          targetPose = ocs2_state_.segment<6>(6);
          targetPose(2) = com_height_ + terrain_height;      //height
          targetPose(4) = pitch_ref;        //pitch
          targetPose(5) = 0.0;              //roll
        }
        if (move_type_ == MoveType::TURNING_POSE)
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
        std::cout << "currentTime: " << currentTime << "\n";
        std::cout << "stateTrajectory[0]: " << stateTrajectory[0].transpose() << "\n";
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
    std::map<std::string, ocs2::humanoid::ModeSequenceTemplate> gait_map_;
    ros::Publisher torsoTargetTrajectoriesPublisher_;
    ros::Publisher modeSequenceTemplatePublisher_;
    double gait_duration_{1.0};
    double start_time_{0};
    double end_time_{0};
    bool traj_published_{false};
    int move_type_ = MoveType::ALL_DOF;
    Eigen::VectorXd ocs2_state_;
    double com_height_;
    std::vector<double> torso_displacement_vel_;
  };
} // namespace GrabBox
