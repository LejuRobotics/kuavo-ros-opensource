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
#include "grab_box/utils/poseTransformer.h"



namespace GrabBox
{
//   enum MoveType
//   {
//     ALL_DOF = 0,
//     HEAD_FORWARD = 1,
//     NORMAL_POSE = 2,
//     TURNING_POSE = 3,
//   };

//   std::string moveTypeToString(MoveType move_type)
//   {
//     switch(move_type)
//     {
//       case ALL_DOF:
//         return "ALL_DOF";
//       case HEAD_FORWARD:
//         return "HEAD_FORWARD";
//       case NORMAL_POSE:
//         return "NORMAL_POSE";
//       default:
//         return "UNKNOWN";
//     }
//   }
//   typedef Eigen::Matrix<double, 6, 1> Vector6d;
//   typedef Eigen::Matrix<double, 4, 1> Vector4d;


  class ComputeAndCmdPoseMoveToDestination : public BT::StatefulActionNode
  {
  public:
    ComputeAndCmdPoseMoveToDestination(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) 
    {
      ros::NodeHandle nh;
      cmdPosePublisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_pose", 10);
      gait_time_name_sub_ = nh.subscribe("/humanoid_mpc_gait_time_name", 10, &ComputeAndCmdPoseMoveToDestination::gaitTimeNameCallback, this);
      auto_gait_mode_client_ = nh.serviceClient<std_srvs::SetBool>("/humanoid_auto_gait");
      
      // 
      // nh.param<double>("cmd_pose_move_to_destination/max_linear_speed", max_linear_speed_, 2.0);
      // nh.param<double>("cmd_pose_move_to_destination/max_angular_speed", max_angular_speed_, 2.0);
      msg_update_distance_tolerance = getParamsFromBlackboard<double>(config, "cmdPoseMoveToDestination.msg_update_distance_tolerance");
      msg_update_angle_tolerance = getParamsFromBlackboard<double>(config, "cmdPoseMoveToDestination.msg_update_angle_tolerance");

    }

    static BT::PortsList providedPorts()
    {
      return{
        BT::InputPort<Eigen::Vector3d>("box_pos"),
        BT::InputPort<Eigen::Vector4d>("box_quat"),
        BT::InputPort<Eigen::Vector3d>("target_box_offset"),
        BT::InputPort<int>("move_type")
      };
    }

    BT::NodeStatus onStart() override
    {
      if(callSetTorsoModeSrv(0))//6-dof
      {
        ROS_INFO("SetTorsoModeSrv call successful, control mode is 6-dof");
      }
      if (!setAutoGaitEnabled())
      {
          ROS_WARN("Failed to enable auto gait mode");
      }


      Eigen::Vector3d box_pos;
      Eigen::Vector4d box_quat;

      // 获取 box_pos 和 box_quat 
      if (!getInput("box_pos", box_pos) || !getInput("box_quat", box_quat) || !getInput("target_box_offset", target_box_offset_)) {
        ROS_ERROR("Failed to get input ports for box_pos or box_quat");
        return BT::NodeStatus::FAILURE;
      }
      box_pose_world_.resize(7);
      box_pose_world_ << box_pos(0), box_pos(1), box_pos(2), box_quat(0), box_quat(1), box_quat(2), box_quat(3);

      computeTargertPose(box_pose_world_, target_box_offset_, target_pose_);

      if(!getInput<int>("move_type", move_type_))
      {
        ROS_ERROR("ComputeAndCmdPoseMoveToDestination: Failed to get input [move_type]");
        return BT::NodeStatus::FAILURE;
      }

      ROS_INFO_STREAM("[ComputeAndCmdPoseMoveToDestination] move_type: " << moveTypeToString(static_cast<MoveType>(move_type_)));
     
      // 获取质心高度
      while(!ros::param::has("/com_height"))
      {
        ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      ROS_INFO_STREAM("com_height parameter is founded.");
      ros::param::get("/com_height", com_height_);
      std::cout << "[ComputeAndCmdPoseMoveToDestination] comHeight: " << com_height_ <<std::endl;

      // limitArmControlMode();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

      if(!config().blackboard->get("ocs2_state", ocs2_state_))
      {
        ROS_ERROR("Failed to retrieve ocs2_state from blackboard");
        return BT::NodeStatus::FAILURE;
      }
      current_pose_ = ocs2_state_.segment<4>(6);

      if(!config().blackboard->get("tag_pose_world", box_pose_world_))
      {
        ROS_ERROR("Failed to retrieve box_pose_world_ from blackboard");
        return BT::NodeStatus::FAILURE;
      }

      if (move_type_ == NORMAL_POSE) 
      {
        geometry_msgs::Twist target_twist;
        // 发布命令
        cmdPosePublisher_.publish(target_twist);
        ros::Duration(1.0).sleep();
        return BT::NodeStatus::SUCCESS;
      }

      if (!is_cmd_pub_) 
      {
        computeTargertPose(box_pose_world_, target_box_offset_, target_pose_);
        // 计算目标相对于机器人的局部坐标
        if(!transformWorldToRobot(target_pose_, target_pose_local_))
        {
            ROS_ERROR("ComputeAndCmdPoseMoveToDestination: Failed to transform target_pose to robot frame");
            return BT::NodeStatus::FAILURE;
        }
        // 

        target_twist_ = getCmdPose(target_pose_local_, move_type_);
        // 发布命令
        cmdPosePublisher_.publish(target_twist_);
        ROS_INFO("ComputeAndCmdPoseMoveToDestination: Published initial Twist command");
        if (current_gait_ == "walk") is_cmd_pub_ = true;
        // is_cmd_pub_ = true;
        return BT::NodeStatus::RUNNING;
      } 

      if (is_cmd_pub_ && current_gait_ == "walk")
      {
        Eigen::Vector4d target_pose_local_updtated;
        if(!transformWorldToRobot(target_pose_, target_pose_local_updtated))
        {
            ROS_ERROR("ComputeAndCmdPoseMoveToDestination: Failed to transform target_pose to robot frame");
            return BT::NodeStatus::FAILURE;
        }
        if(!checkDistanceAndUpdateDelta(target_pose_local_updtated))
        {
          target_twist_ = getCmdPose(target_pose_local_, move_type_);
          // 发布命令
          cmdPosePublisher_.publish(target_twist_);
          ROS_INFO("ComputeAndCmdPoseMoveToDestination: Updated AndPublished Twist command");
        }

      }

      if (is_cmd_pub_ && current_gait_ == "stance") return BT::NodeStatus::SUCCESS;
    
      return BT::NodeStatus::RUNNING;

    }

    void onHalted() override
    {
      ROS_INFO("ComputeAndCmdPoseMoveToDestination: Halted");
    }

  private:


    void computeTargertPose(const Eigen::VectorXd& box_pose_world, const Eigen::Vector3d& target_box_offset, Eigen::Vector4d& target_pose)
    {
      // 计算目标位姿

      std::cout << "box_pose_world_ : " << box_pose_world << std::endl;

      Eigen::VectorXd destination_pose_box(7);
      destination_pose_box << target_box_offset(0), target_box_offset(1), target_box_offset(2), 0.0, 0.0, 0.0, 1.0;

    
    //根据设定的相对位置和BOX生成目标位置
      Eigen::VectorXd destination_pose_ = getDestination(destination_pose_box, box_pose_world);

      std::cout << "destination_pose_:  " << destination_pose_ << std::endl;

      // 创建目标位姿的四维数组（X, Y, Z, YAW）
      target_pose << destination_pose_(0), destination_pose_(1), com_height_, getYawFromQuaternion(destination_pose_.segment<4>(3));

    }

    bool transformWorldToRobot(const Eigen::Vector4d& world_pose, Eigen::Vector4d& target_pose_local_)
    {
      // 计算目标相对于机器人当前位姿的相对坐标
      double dx = world_pose(0) - current_pose_(0);
      double dy = world_pose(1) - current_pose_(1);

      double dyaw = world_pose(3) - current_pose_(3);

      // 计算目标相对于机器人的局部坐标
      double cos_yaw = std::cos(current_pose_(3));
      double sin_yaw = std::sin(current_pose_(3));

      double local_x = dx * cos_yaw + dy * sin_yaw;
      double local_y = - dx * sin_yaw + dy * cos_yaw;

    //   // 设置Twist消息
    //   cmd_twist.linear.x = local_x;
    //   cmd_twist.linear.y = local_y;
    //   cmd_twist.linear.z = 0.0;
    //   cmd_twist.angular.x = 0.0;
    //   cmd_twist.angular.y = 0.0;
    //   cmd_twist.angular.z = dyaw;
      target_pose_local_(0) = local_x;
      target_pose_local_(1) = local_y;
      target_pose_local_(2) = world_pose(2) - com_height_; 
      target_pose_local_(3) = dyaw;

      return true;
    }

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

    geometry_msgs::Twist getCmdPose(const Eigen::Vector4d& target_pose_local, const int move_type)
    {
        geometry_msgs::Twist cmd_twist;
        if (move_type == ALL_DOF)
        {
            cmd_twist.linear.x = target_pose_local(0);
            cmd_twist.linear.y = target_pose_local(1);
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = target_pose_local(3);
        }
        else if (move_type == HEAD_FORWARD)
        {
          cmd_twist.linear.x = target_pose_local(0);
          cmd_twist.linear.y = target_pose_local(1);
          cmd_twist.linear.z = 0.0;
          cmd_twist.angular.x = 0.0;
          cmd_twist.angular.y = 0.0;
          if (std::sqrt(std::pow(target_pose_local(0), 2) + std::pow(target_pose_local(1), 2)) < 0.1 )
            cmd_twist.angular.z = target_pose_local(3);
          else
            cmd_twist.angular.z = 0.0;
        }
        else if (move_type == NORMAL_POSE)
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = 0.0;
        }
        else if (move_type == TURNING_POSE)
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.x = 0.0;
            cmd_twist.angular.y = 0.0;
            cmd_twist.angular.z = atan2(target_pose_(1) - current_pose_(1), target_pose_(0) - current_pose_(0)) - current_pose_(3) ; 
        }

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

  private:



    bool checkDistanceAndUpdateDelta(const Eigen::Vector4d& target_pose_local)
    {
    //   std::cout << "msg_update_distance_tolerance: " << msg_update_distance_tolerance << std::endl;
    //   std::cout << "msg_update_angle_tolerance: " << msg_update_angle_tolerance << std::endl;
      double distance = std::sqrt(std::pow(target_pose_local(0) - target_pose_local_(0), 2) + std::pow(target_pose_local(1) - target_pose_local_(1), 2));
      double dyaw = (abs(target_pose_local(3) - target_pose_local_(3))) * 180.0 / M_PI;
      if (distance < msg_update_distance_tolerance && dyaw < msg_update_angle_tolerance)
      {
        return true;
      }
      target_pose_local_ = target_pose_local;
      return false;
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


    ros::Publisher cmdPosePublisher_;
    ros::Subscriber gait_time_name_sub_;
    ros::ServiceClient auto_gait_mode_client_;

    Eigen::Vector4d target_pose_;      // (x, y, z, yaw)
    Eigen::Vector4d target_pose_local_; //
    geometry_msgs::Twist target_twist_; // 转换后的Twist消息
    Eigen::Vector4d current_pose_;     // 机器人当前位姿 (x, y, yaw)
    int move_type_;
    double com_height_;

    Eigen::VectorXd ocs2_state_;
    Eigen::VectorXd box_pose_world_;
    Eigen::Vector3d target_box_offset_;


    double msg_update_distance_tolerance = 0.1;
    double msg_update_angle_tolerance = 10.0;
    string current_gait_= "stance";
    bool is_cmd_pub_ = false;
    double delta_roll = 0.0, delta_pitch = M_PI/2, delta_yaw = M_PI/2;

    ros::Time start_time_;
  };
} // namespace GrabBox
