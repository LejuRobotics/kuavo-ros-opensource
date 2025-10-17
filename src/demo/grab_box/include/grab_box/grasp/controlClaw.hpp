#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Core>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <grab_box/utils/customIoPort.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/twoArmHandPoseCmdSrv.h"
#include "kuavo_msgs/ikSolveParam.h"
#include "kuavo_msgs/headBodyPose.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "grab_box/common/ocs2_ros_interface.hpp"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/lejuClawCommand.h"
#include "kuavo_msgs/controlLejuClaw.h"

namespace GrabBox
{

  enum class EndEffectorType { UNKNOWN, QIANGNAO, LEJUCLAW };

  using namespace std::chrono;
  // Example of Asynchronous node that uses StatefulActionNode as base class
  class ControlClaw : public BT::StatefulActionNode
  {
  public:
    ControlClaw(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), nh_()
    {
      // Determine end-effector type from ROS parameter server
      std::string end_effector_param;
      if (nh_.getParam("end_effector_type", end_effector_param)) {
        ROS_INFO("[%s] Found end_effector_type parameter: %s", name.c_str(), end_effector_param.c_str());
        if (end_effector_param == "qiangnao") {
          current_end_effector_type_ = EndEffectorType::QIANGNAO;
          hand_cmd_pub_ = nh_.advertise<kuavo_msgs::robotHandPosition>("hand_command_topic", 1, true);
          ROS_INFO("[%s] Configured for QIANGNAO end-effector.", name.c_str());
        } else if (end_effector_param == "lejuclaw") {
          current_end_effector_type_ = EndEffectorType::LEJUCLAW;
          leju_claw_service_client_ = nh_.serviceClient<kuavo_msgs::controlLejuClaw>("/control_robot_leju_claw");
          ROS_INFO("[%s] Configured for LEJUCLAW end-effector (service: /control_robot_leju_claw).", name.c_str());
        } else {
          current_end_effector_type_ = EndEffectorType::UNKNOWN;
          ROS_ERROR("[%s] Unknown end_effector_type: %s. Node will not operate correctly.", name.c_str(), end_effector_param.c_str());
        }
      } else {
        current_end_effector_type_ = EndEffectorType::UNKNOWN; // Or default to one, e.g., QIANGNAO
        ROS_ERROR("[%s] Failed to get 'end_effector_type' parameter. Node will not operate correctly.", name.c_str());
        // As a fallback, trying to initialize for QIANGNAO to avoid crashing if topic is there
        // hand_cmd_pub_ = nh_.advertise<kuavo_msgs::robotHandPosition>("hand_command_topic", 1, true);
        // ROS_WARN("[%s] Defaulting to QIANGNAO due to missing parameter, but this might be incorrect.", name.c_str());
      }
    }
    
    
    static BT::PortsList providedPorts()
    {
      return { 
        BT::InputPort<std::string>("action_mode"),     // "open" or "close"
        BT::InputPort<int>("hand_to_control"),           // 0: left, 1: right, 2: both
        BT::InputPort<double>("duration")           // Action duration in seconds
      };
    }

    BT::NodeStatus onStart() override
    {
      if (current_end_effector_type_ == EndEffectorType::UNKNOWN) {
        ROS_ERROR("[%s] End effector type is UNKNOWN. Cannot proceed.", name().c_str());
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput<std::string>("action_mode", action_mode_input_)) {
        ROS_ERROR("[%s] Missing required input [action_mode]", name().c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (action_mode_input_ != "open" && action_mode_input_ != "close") {
        ROS_ERROR("[%s] Invalid [action_mode]: %s. Must be 'open' or 'close'.", name().c_str(), action_mode_input_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // Read and validate hand_to_control (now an int)
      if (!getInput<int>("hand_to_control", hand_to_control_int_input_)) {
        ROS_ERROR("[%s] Missing required input [hand_to_control]", name().c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (hand_to_control_int_input_ < 0 || hand_to_control_int_input_ > 2) {
        ROS_ERROR("[%s] Invalid [hand_to_control]: %d. Must be 0 (left), 1 (right), or 2 (both).", 
                  name().c_str(), hand_to_control_int_input_);
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput<double>("duration", duration_input_)) {
        ROS_WARN("[%s] No duration provided, using default: 1.0s", name().c_str());
        duration_input_ = 1.0; // Default duration
      }
      if (duration_input_ < 0) {
        ROS_ERROR("[%s] Invalid [duration]: %f. Must be non-negative.", name().c_str(), duration_input_);
        return BT::NodeStatus::FAILURE;
      }

      kuavo_msgs::robotHandPosition hand_cmd_msg;
      kuavo_msgs::lejuClawCommand leju_cmd_msg; // Declare here for broader scope if needed, or move inside LEJUCLAW block

      if (current_end_effector_type_ == EndEffectorType::QIANGNAO) {
        uint8_t target_joint_value = (action_mode_input_ == "open") ? 0 : 100;
        std::vector<uint8_t> joint_positions(6, target_joint_value); // 6 joints, all set to target_joint_value

        bool control_left = (hand_to_control_int_input_ == 0 || hand_to_control_int_input_ == 2);
        bool control_right = (hand_to_control_int_input_ == 1 || hand_to_control_int_input_ == 2);

        if (control_left) {
          hand_cmd_msg.left_hand_position = joint_positions;
        }
        if (control_right) {
          hand_cmd_msg.right_hand_position = joint_positions;
        }
        
        std::string hand_side_str;
        if(control_left && control_right) hand_side_str = "both";
        else if(control_left) hand_side_str = "left";
        else if(control_right) hand_side_str = "right";
        else hand_side_str = "none"; // Should not happen due to validation

        if (hand_cmd_pub_) {
            hand_cmd_pub_.publish(hand_cmd_msg);
            ROS_INFO("[%s] QIANGNAO: Published hand command: %s %s hand(s). Waiting for %f seconds.", 
                      name().c_str(), action_mode_input_.c_str(), hand_side_str.c_str(), duration_input_);
        } else {
            ROS_ERROR("[%s] QIANGNAO publisher not initialized!", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

      } else if (current_end_effector_type_ == EndEffectorType::LEJUCLAW) {
        if (!leju_claw_service_client_) {
            ROS_ERROR("[%s] LEJUCLAW service client not initialized!", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!leju_claw_service_client_.exists()) {
            ROS_ERROR("[%s] LEJUCLAW service /control_robot_leju_claw not available.", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        kuavo_msgs::controlLejuClaw srv_call;

        double target_pos = (action_mode_input_ == "open") ? 100.0 : 0.0; // 0=closed, 100=open for Leju Claw
        const double default_velocity = 90.0;
        const double default_effort = 1.0;

        bool control_leju_left = (hand_to_control_int_input_ == 0 || hand_to_control_int_input_ == 2);
        bool control_leju_right = (hand_to_control_int_input_ == 1 || hand_to_control_int_input_ == 2);

        if (control_leju_left) {
          srv_call.request.data.name.push_back("left_claw");
          srv_call.request.data.position.push_back(target_pos);
          srv_call.request.data.velocity.push_back(default_velocity);
          srv_call.request.data.effort.push_back(default_effort);
        }
        if (control_leju_right) { // Note: If only one Leju claw, "right" might still map to it or be ignored by hardware
          srv_call.request.data.name.push_back("right_claw");
          srv_call.request.data.position.push_back(target_pos);
          srv_call.request.data.velocity.push_back(default_velocity);
          srv_call.request.data.effort.push_back(default_effort);
        }

        if (srv_call.request.data.name.empty()){
            // This case should not be hit if hand_to_control_int_input_ is validated to be 0, 1, or 2
            // unless the LejuClaw is singular and neither "left_claw" nor "right_claw" is appropriate.
            // For now, assuming the validation ensures at least one condition (left/right/both) is met.
            ROS_ERROR("[%s] LEJUCLAW: No target claws specified for hand_to_control code %d. This indicates an issue.", 
                        name().c_str(), hand_to_control_int_input_);
            return BT::NodeStatus::FAILURE;
        }
        
        if (leju_claw_service_client_.call(srv_call)) {
            if (srv_call.response.success) {
                std::string controlled_claws_str;
                for(size_t i = 0; i < srv_call.request.data.name.size(); ++i) {
                    controlled_claws_str += srv_call.request.data.name[i];
                    if (i < srv_call.request.data.name.size() - 1) controlled_claws_str += ", ";
                }
                ROS_INFO("[%s] LEJUCLAW: Service call successful for %s: %s. Message: '%s'. Waiting for %f seconds.", 
                          name().c_str(), action_mode_input_.c_str(), controlled_claws_str.c_str(), 
                          srv_call.response.message.c_str(), duration_input_);
            } else {
                ROS_ERROR("[%s] LEJUCLAW: Service call for %s failed. Message: '%s'", 
                          name().c_str(), action_mode_input_.c_str(), srv_call.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        } else {
            ROS_ERROR("[%s] LEJUCLAW: Failed to call service /control_robot_leju_claw for %s.", 
                      name().c_str(), action_mode_input_.c_str());
            return BT::NodeStatus::FAILURE;
        }
      }

      start_time_ = ros::Time::now().toSec();
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      std::string hand_side_str;
      if (hand_to_control_int_input_ == 0) hand_side_str = "left";
      else if (hand_to_control_int_input_ == 1) hand_side_str = "right";
      else if (hand_to_control_int_input_ == 2) hand_side_str = "both";
      else hand_side_str = "unknown_code_" + std::to_string(hand_to_control_int_input_); // Fallback for logging

      if (ros::Time::now().toSec() - start_time_ >= duration_input_) {
        ROS_INFO("[%s] Action %s %s hand(s) completed.", name().c_str(), action_mode_input_.c_str(), hand_side_str.c_str());
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
      ROS_WARN("[%s] ControlClaw action halted.", name().c_str());
    }


  private:
    ros::NodeHandle nh_;
    ros::Publisher hand_cmd_pub_;
    ros::ServiceClient leju_claw_service_client_;
    EndEffectorType current_end_effector_type_ = EndEffectorType::UNKNOWN;
    
    std::string action_mode_input_;
    int hand_to_control_int_input_;     // New member for integer input
    double duration_input_;
    double start_time_;
  };
} // namespace GrabBox
