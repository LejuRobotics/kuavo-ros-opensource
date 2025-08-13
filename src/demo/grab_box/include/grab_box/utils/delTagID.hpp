#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <kuavo_msgs/setTagId.h>
#include <kuavo_msgs/robotHeadMotionData.h>

namespace GrabBox
{
  class DelTagID : public BT::StatefulActionNode
  {
  public:
    DelTagID(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::StatefulActionNode(name, config)
    {
      ros::NodeHandle nh = ros::NodeHandle("~");
      continuousTrackClient_ = nh.serviceClient<std_srvs::SetBool>("/continuous_track");
      head_orientation_pub_ = nh.advertise<kuavo_msgs::robotHeadMotionData>("/robot_head_motion_data", 10);

      del_tag_id_client_ = nh.serviceClient<kuavo_msgs::setTagId>("/del_target_tag_id");
      timeout_ = getParamsFromBlackboard<double>(config, "box_tag_ok.timeout");
      ROS_INFO_STREAM("[DelTagID] timeout: " << timeout_);
      // start_time_ = ros::Time::now().toSec();
    }

    static BT::PortsList providedPorts()
    {
      return {
              BT::InputPort<int>("tag_id")
      };
    }

    BT::NodeStatus onStart() override
    {
      tag_id_ = -1;
      if (!getInput("tag_id", tag_id_))
      {
        ROS_ERROR("[DelTagID] Missing input port: tag_id");
        return BT::NodeStatus::FAILURE;
      }
      if (!setContinuousTracking(false)) {
        ROS_ERROR("[DelTagID] Failed to stop continuous tracking");
        return BT::NodeStatus::FAILURE;
      }
      else config().blackboard->set<bool>("continuous_tracking_started", false);
      start_time_ = ros::Time::now().toSec();
      return BT::NodeStatus::RUNNING;

    }

    BT::NodeStatus onRunning() override
    {
      publishHeadOrientationCommand(0.0, 0.0);
      current_time_ = ros::Time::now().toSec();
      // std::cout << "[DelTagID] start_time_ : " << start_time_ << std::endl;
      // std::cout << "[DelTagID] current_time_ : " << current_time_ << std::endl;
      // std::cout << "[DelTagID] current_time_ - start_time_ : " << current_time_ - start_time_ << std::endl;
      if (current_time_ - start_time_ < 3.0) 
      {
        std::cout << "[DelTagID] Waiting for head motion to stop : " << current_time_ - start_time_ << std::endl;
        return BT::NodeStatus::RUNNING;
      }
      if (!delTagId(tag_id_))
      {
        ROS_ERROR("[DelTagID] Failed to set target tag ID via service");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
    void onHalted() override
    {
      ROS_WARN("[DelTagID] interrupted");
      tag_id_ = -1;
    }

  private:

    bool delTagId(int tag_id)
    {
      kuavo_msgs::setTagId srv;
      srv.request.tag_id = tag_id;
      if (!del_tag_id_client_.call(srv))
      {
        ROS_ERROR("[DelTagID] Failed to call set_target_tag_id service");
        return false;
      }
      if (!srv.response.success)
      {
        ROS_ERROR_STREAM("[DelTagID] Service response: " << srv.response.message);
        return false;
      }
      ROS_INFO_STREAM("[DelTagID] Target tag ID : " << tag_id << "deleted , " << srv.response.message<< "   (tag_id: " << tag_id << ")");
      return true;
    }

    bool setContinuousTracking(bool enable)
    {
      std_srvs::SetBool srv;
      srv.request.data = enable;
      if (!continuousTrackClient_.call(srv) || !srv.response.success)
      {
        ROS_WARN("Failed to start continuous tracking.");
        return false;
      }
      return true;
    }

    void publishHeadOrientationCommand(double pitch, double yaw) {
        //
        pitch *= 180 / M_PI;
        yaw *= 180 / M_PI;

        // std::cout << "[Tag] Head pitch : " << pitch << std::endl;
        // std::cout << "[Tag] Head yaw : " << yaw << std::endl;

        yaw = std::max(-30.0, std::min(30.0, yaw));

        // Ensure pitch is within the range [-25, 25]
        pitch = std::max(-25.0, std::min(25.0, pitch));

        // 
        kuavo_msgs::robotHeadMotionData head_cmd;
        head_cmd.joint_data.resize(2);  
        head_cmd.joint_data[0] = yaw;   // 
        head_cmd.joint_data[1] = pitch; // 

        // Publish 
        head_orientation_pub_.publish(head_cmd);
    }

    ros::ServiceClient continuousTrackClient_; // Service client for continuous tracking
    ros::ServiceClient del_tag_id_client_;
    ros::Publisher head_orientation_pub_;
    Eigen::VectorXd tag_pose_world_;
    bool data_received_ = false;

    double timeout_ = 1.0;
    int tag_id_ = -1;
    double start_time_;
    double current_time_;
  };
} // namespace GrabBox
