#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <kuavo_msgs/setTagId.h>
#include "grab_box/utils/poseTransformer.h"


using namespace autoHeadChase;

namespace GrabBox
{
  class BoxTagOK : public BT::ConditionNode
  {
  public:
    BoxTagOK(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::ConditionNode(name, config)
    {
      ros::NodeHandle nh = ros::NodeHandle("~");
      tag_pose_world_.setZero(7);
      box_offset_grpc_.setZero(7);
      box_offset_grpc_(6) = 1.0;
      boxWorldPoseSuber = nh.subscribe("/tag_world_pose", 10, &BoxTagOK::tagWorldPoseCallBack, this);
      set_tag_id_client_ = nh.serviceClient<kuavo_msgs::setTagId>("/set_target_tag_id");
      timeout_ = getParamsFromBlackboard<double>(config, "box_tag_ok.timeout");
      ROS_INFO_STREAM("[BoxTagOK] timeout: " << timeout_);
      config.blackboard->set<int>("current_tag_id", -1);
    }

    static BT::PortsList providedPorts()
    {
      return {
              BT::InputPort<int>("tag_id"), 
              BT::InputPort<bool>("time_check"), 
              BT::InputPort<double>("isUingGrpc"), 
              BT::OutputPort<Eigen::Vector3d>("box_pos"), 
              BT::OutputPort<Eigen::Vector4d>("box_quat")
      };
    }

    BT::NodeStatus tick() override final
    {

      int tag_id = -1;
      if (!getInput("tag_id", tag_id))
      {
        ROS_ERROR("[BoxTagOK] Missing input port: tag_id");
        return BT::NodeStatus::FAILURE;
      }

      // if(!getInput("isUingGrpc", is_using_grpc_))
      // {
      //   ROS_ERROR("[BoxTagOK] Missing input port: isUingGrpc");
      // }

      if (is_using_grpc_)
      {
        delta_roll = -M_PI/2;
        delta_pitch = 0.0;
        delta_yaw = 0.0;
        box_offset_grpc_ << -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
      }

      int current_tag_id = -1;
      if (!config().blackboard->get<int>("current_tag_id", current_tag_id))
      {
        ROS_ERROR("[BoxTagOK] current_tag_id FAIL.");
        return BT::NodeStatus::FAILURE;
      }
      // 如果当前的 Tag ID 不匹配，则通过服务设置新的 Tag ID
      if (tag_id != current_tag_id)
      {
        if (!setTagId(tag_id))
        {
          ROS_ERROR("[BoxTagOK] Failed to set target tag ID via service");
          return BT::NodeStatus::FAILURE;
        }
        config().blackboard->set<int>("current_tag_id", tag_id);
        data_received_ = false; // 标记数据未接收，等待下一帧数据
      }

      if (!data_received_) {
        ROS_ERROR("[BoxTagOK] Data not yet received, stay in this tick until data is available.");
        // Data not yet received, stay in this tick until data is available
        return BT::NodeStatus::RUNNING;
      }
      Eigen::VectorXd box_pose_world;
      box_pose_world = tag_pose_world_;
      if (is_using_grpc_) checkAndCorrectBoxPose(box_pose_world);
      box_pose_world = getDestination(box_offset_grpc_, box_pose_world);
      Eigen::Vector3d box_pos(box_pose_world(0), box_pose_world(1), box_pose_world(2));
      Eigen::Vector4d box_quat(box_pose_world(3), box_pose_world(4), box_pose_world(5), box_pose_world(6));

      // Output box position and orientation to the behavior tree's blackboard
      setOutput("box_pos", box_pos);
      setOutput("box_quat", box_quat);

      // std::cout << "tag_pose_world_ : " << tag_pose_world_.transpose() << std::endl;

      // // Publish tag_pose_world_ to the blackboard 
      // data_received_ = false;

      return BT::NodeStatus::SUCCESS;
    }

    void tagWorldPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
      recieved_time_ = msg->header.stamp;
      Eigen::VectorXd tag_pose_world_tmp;
      tag_pose_world_tmp.setZero(7);

      tag_pose_world_tmp(0) = msg->pose.position.x;
      tag_pose_world_tmp(1) = msg->pose.position.y;
      tag_pose_world_tmp(2) = msg->pose.position.z;
      tag_pose_world_tmp(3) = msg->pose.orientation.x;
      tag_pose_world_tmp(4) = msg->pose.orientation.y;
      tag_pose_world_tmp(5) = msg->pose.orientation.z;
      tag_pose_world_tmp(6) = msg->pose.orientation.w;
      tag_pose_world_ = correctBoxPose(tag_pose_world_tmp, delta_roll, delta_pitch, delta_yaw);

      config().blackboard->set<Eigen::VectorXd>("tag_pose_world", tag_pose_world_);
      data_received_ = true;
    }

  private:

    bool setTagId(int tag_id)
    {
      kuavo_msgs::setTagId srv;
      srv.request.tag_id = tag_id;
      if (!set_tag_id_client_.call(srv))
      {
        ROS_ERROR("[BoxTagOK] Failed to call set_target_tag_id service");
        return false;
      }
      if (!srv.response.success)
      {
        ROS_ERROR_STREAM("[BoxTagOK] Service response: " << srv.response.message);
        return false;
      }
      ROS_INFO_STREAM("[BoxTagOK] Target tag ID set to " << tag_id << ": " << srv.response.message<< "   (tag_id: " << tag_id << ")");
      return true;
    }

    Eigen::VectorXd correctBoxPose(const Eigen::VectorXd& box_pose, double delta_roll, double delta_pitch, double delta_yaw)
    {
    //   autoHeadChase::PoseTransformer transformer;
      Eigen::Matrix4d box_matrix = PoseTransformer::poseToTransform(box_pose);
      Eigen::Matrix4d delta_matrix = PoseTransformer::eulerToTransformationMatrix(delta_roll, delta_pitch, delta_yaw);
      Eigen::Matrix4d box_matrix_correct = box_matrix * delta_matrix;
      return PoseTransformer::transformToPose(box_matrix_correct);
    }

   Eigen::VectorXd getDestination(const Eigen::VectorXd& destination_pose_box, const Eigen::VectorXd& box_pose)
    {
      return PoseTransformer::transformPoseToWorld(destination_pose_box, box_pose);
    }

    void checkAndCorrectBoxPose (Eigen::VectorXd& box_pose_world) 
    {
      Eigen::VectorXd ocs2_state;
      if(!config().blackboard->get("ocs2_state", ocs2_state))
      {
        ROS_ERROR("[ComputeTargetPose] Failed to retrieve ocs2_state from blackboard");
        return;
      }
      double robotYaw = ocs2_state(9); // robot yaw
      double boxYaw = getYawFromQuaternion(box_pose_world.segment<4>(3));
      Eigen::Matrix4d box_matrix = PoseTransformer::poseToTransform(box_pose_world);
      double delta_yaw = 0.0;
      std::cout << "[ComputeTargetPose] " << std::endl;
      std::cout << "robotYaw : " << robotYaw << std::endl;
      std::cout << "boxYaw : " << boxYaw << std::endl;

      double yawDelta = boxYaw - robotYaw;
      yawDelta = atan2(sin(yawDelta), cos(yawDelta));
      // std::cout <<
      if (yawDelta < - M_PI /2 ) {
        delta_yaw = M_PI;
        ROS_INFO("[ComputeTargetPose] BOX YAW Add Pi ");
      }
      if (yawDelta >  M_PI /2 ) {
        delta_yaw = -M_PI;
        ROS_INFO("[ComputeTargetPose] BOX YAW Minus Pi ");

      }
      Eigen::Matrix4d delta_matrix = PoseTransformer::eulerToTransformationMatrix(0.0, 0.0, delta_yaw);      
      Eigen::Matrix4d box_matrix_correct = box_matrix * delta_matrix;
      box_pose_world = PoseTransformer::transformToPose(box_matrix_correct);
    }

    // 使用四元数计算 Yaw
    double getYawFromQuaternion(const Eigen::Vector4d& quat)
    {
      Eigen::Quaterniond q(quat(3), quat(0), quat(1), quat(2));
      return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    ros::Subscriber boxWorldPoseSuber;
    ros::ServiceClient set_tag_id_client_;
    Eigen::VectorXd tag_pose_world_;
    bool data_received_ = false;
    double timeout_ = 1.0;
    ros::Time recieved_time_ = ros::Time::now();
    double delta_roll = 0.0, delta_pitch = M_PI/2, delta_yaw = M_PI/2;
    bool is_using_grpc_ = false;
    Eigen::VectorXd box_offset_grpc_;

  };
} // namespace GrabBox
