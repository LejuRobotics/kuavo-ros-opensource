#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Core>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <grab_box/utils/customIoPort.h>
#include <kuavo_msgs/gaitTimeName.h>
#include <string>
#include <std_srvs/SetBool.h>



namespace GrabBox
{

  class CmdPoseWorldMoveToDestination : public BT::StatefulActionNode
  {
  public:
    CmdPoseWorldMoveToDestination(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) 
    {
      ros::NodeHandle nh;
      cmdPosePublisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_pose_world", 10);
      gait_time_name_sub_ = nh.subscribe("/humanoid_mpc_gait_time_name", 10, &CmdPoseWorldMoveToDestination::gaitTimeNameCallback, this);
      auto_gait_mode_client_ = nh.serviceClient<std_srvs::SetBool>("/humanoid_auto_gait");
      
      xy_threshold_ = getParamsFromBlackboard<double>(config, "cmdPoseWorldMoveToDestination.xy_threshold");
      yaw_threshold_ = M_PI / 180.0 * getParamsFromBlackboard<double>(config, "cmdPoseWorldMoveToDestination.yaw_threshold");
      pitch_ref_ = M_PI / 180.0 * getParamsFromBlackboard<double>(config, "normal_torso_pitch");
      std::cout << "[CmdPoseWorldMoveToDestination] xy_threshold: " << xy_threshold_ << std::endl;
      std::cout << "[CmdPoseWorldMoveToDestination] yaw_threshold: " << yaw_threshold_ << std::endl;
      std::cout << "[CmdPoseWorldMoveToDestination] pitch_ref: " << pitch_ref_ << std::endl;
    }

    static BT::PortsList providedPorts()
    {
      return{
        BT::InputPort<Eigen::Vector4d>("target_pose") //  (x, y, z, yaw)};
      };
    }

    BT::NodeStatus onStart() override
    {
      if (!setAutoGaitEnabled())
      {
        ROS_WARN("Failed to enable auto gait mode");
      }
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      if(!config().blackboard->get("ocs2_state", ocs2_state_))
      {
        ROS_ERROR("Failed to retrieve ocs2_state from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      getInput<Eigen::Vector4d>("target_pose", target_pose_);
      // std::cout << "target_pose: " << target_pose_.transpose() << std::endl;
      target_pose_(3) *= M_PI / 180.0; // 转化为弧度制

      Eigen::VectorXd torso_pose = ocs2_state_.segment<4>(6);
      // std::cout << "torso_pose: " << torso_pose.transpose() << std::endl;
      // std::cout << "torso_pose: " << torso_pose.transpose() << std::endl;
      if(checkTargetReached(target_pose_, torso_pose))
      {
        std::string gait_name = "stance";
        if(!config().blackboard->get("current_gait_name", gait_name))
        {
          ROS_ERROR("Failed to retrieve current_gait_name from blackboard");
          return BT::NodeStatus::FAILURE;
        }
        bool single_step_mode = false;
        if(!config().blackboard->get("single_step_mode", single_step_mode))
        {
          ROS_ERROR("Failed to retrieve single_step_mode from blackboard");
          return BT::NodeStatus::FAILURE;
        }
        if(gait_name == "stance" || (gait_name == "custom_gait" && !single_step_mode))
          return BT::NodeStatus::SUCCESS;
        else{
          ROS_INFO("[CmdPoseWorldMoveToDestination]: Target reached, waiting for switching to stance gait");
          return BT::NodeStatus::RUNNING;
        }
      }
      else
      {
        // std::cout << "[CmdPoseWorldMoveToDestination] torso error: " << (target_pose_ - torso_pose).transpose() << std::endl;
        cmdPosePublisher_.publish(getCmdPose(target_pose_));
        return BT::NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      ROS_INFO("CmdPoseWorldMoveToDestination: Halted");
    }

  private:

    bool setAutoGaitEnabled()
    {
      std_srvs::SetBool srv_msg;
      srv_msg.request.data = true;
      if (auto_gait_mode_client_.call(srv_msg))
      {
        if (srv_msg.response.success)
        {
          ROS_INFO("Auto gait mode enabled successfully: %s", srv_msg.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("Failed to enable auto gait mode: %s", srv_msg.response.message.c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service /auto_gait_mode");
        return false;
      }
    }

    geometry_msgs::Twist getCmdPose(const Eigen::Vector4d& target_pose)
    {
      geometry_msgs::Twist cmd_twist;
      cmd_twist.linear.x = target_pose(0);
      cmd_twist.linear.y = target_pose(1);
      cmd_twist.linear.z = 0.0;
      cmd_twist.angular.x = 0.0;
      cmd_twist.angular.y = pitch_ref_;
      cmd_twist.angular.z = target_pose(3);

      return cmd_twist;
    }

    void gaitTimeNameCallback(kuavo_msgs::gaitTimeName::ConstPtr msg)
    {
      current_gait_ = msg->gait_name;
      std::cout << "current gait name : " << current_gait_ << std::endl;
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

    bool checkTargetReached(const Eigen::VectorXd &target_pose, const Eigen::VectorXd &current_pose)
    {
      double xy_error = (target_pose.head(2) - current_pose.head(2)).norm();
      // std::cout << "target yaw: " << target_pose(3) << " current yaw: " << current_pose(3) << std::endl;
      double yaw_error = M_PI/180.0*std::abs(normalizedYaw(normalizedYaw(180/M_PI*target_pose(3)) - normalizedYaw(180/M_PI*current_pose(3))));
      // std::cout << "[CmdPoseWorldMoveToDestination] yaw_error : " << yaw_error << " rad." << std::endl;
      return xy_error < xy_threshold_ && yaw_error < yaw_threshold_;
    }

  private:
    ros::Publisher cmdPosePublisher_;
    ros::Subscriber gait_time_name_sub_;
    ros::ServiceClient auto_gait_mode_client_;

    std::string current_gait_= "stance";
    Eigen::Vector4d target_pose_;      // (x, y, z, yaw)
    Eigen::Vector4d current_pose_;     // 机器人当前位姿 (x, y, yaw)
    double xy_threshold_ = 0.05; // m
    double yaw_threshold_ = 0.1; // rad
    double pitch_ref_ = 0.0; // rad

    Eigen::VectorXd ocs2_state_;
  };
} // namespace GrabBox
