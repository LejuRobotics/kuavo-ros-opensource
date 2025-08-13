#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <std_srvs/SetBool.h> 
#include <std_msgs/Float64.h>
#include "grab_box/common/ocs2_ros_interface.hpp"
#include "kuavo_msgs/endEffectorData.h"
#include <std_msgs/Float64MultiArray.h>
#include <kuavo_msgs/gaitTimeName.h>

namespace GrabBox
{
  class CheckStatusOK : public BT::ConditionNode
  {
  public:
    CheckStatusOK(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::ConditionNode(name, config)
    {
      ros::NodeHandle nh = ros::NodeHandle("~");
      eef_pose_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/mobile_manipulator/grab_box/eef_pose", 10);
      gait_time_name_sub_ = nh.subscribe("/humanoid_mpc_gait_time_name", 10, &CheckStatusOK::gaitTimeNameCallback, this);
      subObservation_ = nh.subscribe("/humanoid_wbc_observation", 10, &CheckStatusOK::observationCallback, this);
      continuousTrackClient_ = nh.serviceClient<std_srvs::SetBool>("/continuous_track");
      // eePoseSubscriber_ = nh.subscribe<kuavo_msgs::endEffectorData>("/humanoid_ee_State", 1, &CheckStatusOK::eePoseCallback, this);
      // eePoseSubscriber_ = nh.subscribe<std_msgs::Float64MultiArray>("/mobile_manipulator_eef_poses", 10, &CheckStatusOK::eePoseCallback, this);
      eePoseSubscriber_ = nh.subscribe<std_msgs::Float64MultiArray>("/humanoid_controller/wbc_arm_eef_pose", 10, &CheckStatusOK::eePoseCallback, this);

      terrainHeightSubscriber_ = nh.subscribe<std_msgs::Float64>("/humanoid/mpc/terrainHeight", 1, &CheckStatusOK::terrainHeightCallback, this);
      single_step_mode_sub_ = nh.subscribe("/humanoid/single_step_mode", 10, &CheckStatusOK::singleStepModeCallback, this);

      while(!nh.hasParam("/com_height"))
      {
        // ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
        ros::Duration(0.1).sleep();
      }
      ROS_INFO_STREAM("[CheckStatusOK] com_height parameter is founded.");
      nh.getParam("/com_height", com_height_);
      ROS_INFO_STREAM("[CheckStatusOK] com_height: " << com_height_);
      while(!ee_pose_updated_)
      {
        // ROS_WARN("[CheckStatusOK] Waiting for end-effector pose...");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
      ROS_INFO("[CheckStatusOK] Already received end-effector pose.");
      config.blackboard->set<std::string>("current_gait_name", "stance");
      config.blackboard->set<bool>("continuous_tracking_started", continuous_tracking_started_);
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("value_input")}; // 提供输入端口
    }

    BT::NodeStatus tick() override final
    {
      ros::spinOnce();
      config().blackboard->get<bool>("continuous_tracking_started", continuous_tracking_started_);
      if(!continuous_tracking_started_)
      {
        if(setContinuousTracking(true))
          continuous_tracking_started_ = true;
      }
      config().blackboard->set<bool>("continuous_tracking_started", continuous_tracking_started_);
      if(!ee_pose_updated_)
      {
        ROS_WARN("Failed to get end-effector pose.");
        return BT::NodeStatus::RUNNING;
      } 
      return BT::NodeStatus::SUCCESS;
    }
  private:
    bool setContinuousTracking(bool enable)
    {
      std_srvs::SetBool srv;
      srv.request.data = enable;
      if (!updated_)
      {
        return false;
      }
      else if (!continuousTrackClient_.call(srv) || !srv.response.success)
      {
        ROS_WARN("Failed to start continuous tracking.");
        return false;
      }
      return true;
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
      latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
      updated_ = true;
      config().blackboard->set<Eigen::Vector4d>("torso_pose", latestObservation_.state.segment<4>(6));
      config().blackboard->set<Eigen::VectorXd>("ocs2_state", latestObservation_.state);
      config().blackboard->set<double>("current_time", latestObservation_.time);
      config().blackboard->set<size_t>("current_mode", latestObservation_.mode);

    }

    // void eePoseCallback(const kuavo_msgs::endEffectorData::ConstPtr& msg)
    // {
    //   HandPose real_hand_pose_l, real_hand_pose_r;
    //   real_hand_pose_l.first << msg->position[0], msg->position[1], msg->position[2];
    //   real_hand_pose_l.second = Eigen::Quaterniond(msg->position[6], msg->position[3], msg->position[4], msg->position[5]);
    //   real_hand_pose_r.first << msg->position[7], msg->position[8], msg->position[9];
    //   real_hand_pose_r.second = Eigen::Quaterniond(msg->position[13], msg->position[10], msg->position[11], msg->position[12]);
    //   config().blackboard->set<TwoHandPose>("real_hand_poses", {real_hand_pose_l, real_hand_pose_r});
    //   std_msgs::Float64MultiArray pub_msg;
    //   pub_msg.data.resize(14);
    //   pub_msg.data = msg->position;
    //   eef_pose_pub_.publish(pub_msg);
    //   ee_pose_updated_ = true;
    // };

    void eePoseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      HandPose real_hand_pose_l, real_hand_pose_r;
      // double z_bias = com_height_ + terrain_height_;
      real_hand_pose_l.first << msg->data[0], msg->data[1], msg->data[2];
      real_hand_pose_l.second = Eigen::Quaterniond(msg->data[6], msg->data[3], msg->data[4], msg->data[5]);
      real_hand_pose_r.first << msg->data[7], msg->data[8], msg->data[9];
      real_hand_pose_r.second = Eigen::Quaterniond(msg->data[13], msg->data[10], msg->data[11], msg->data[12]);
      config().blackboard->set<TwoHandPose>("real_hand_poses", {real_hand_pose_l, real_hand_pose_r});
      ee_pose_updated_ = true;
      std_msgs::Float64MultiArray pub_msg;
      pub_msg.data.resize(14);
      pub_msg.data = msg->data;
      eef_pose_pub_.publish(pub_msg);
    };

    void terrainHeightCallback(const std_msgs::Float64::ConstPtr& msg)
    {
      terrain_height_ = msg->data;
      config().blackboard->set<double>("terrain_height", msg->data);
    }

    void gaitTimeNameCallback(kuavo_msgs::gaitTimeName::ConstPtr msg)
    {
      config().blackboard->set<double>("current_gait_start_time", msg->start_time);
      config().blackboard->set<std::string>("current_gait_name", msg->gait_name);
      std::cout << "[CheckStatusOK] current[" << msg->start_time << "] gait name : " << msg->gait_name << std::endl;
    }

    void singleStepModeCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      config().blackboard->set<bool>("single_step_mode", msg->data);
    }

  private:
    ros::Subscriber subObservation_;
    ros::Subscriber eePoseSubscriber_;
    ros::Subscriber terrainHeightSubscriber_;
    ros::Subscriber gait_time_name_sub_;
    ros::Subscriber single_step_mode_sub_;
    ros::Publisher eef_pose_pub_;
    ocs2::SystemObservation latestObservation_;
    ros::ServiceClient continuousTrackClient_; // Service client for continuous tracking
    bool updated_ = false;
    bool ee_pose_updated_ = false;
    bool continuous_tracking_started_ = false;
    double com_height_{0};
    double terrain_height_{0};
  };
} // namespace GrabBox
