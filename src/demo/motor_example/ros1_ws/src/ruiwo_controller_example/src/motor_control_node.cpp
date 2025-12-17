#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "ruiwo_controller_cxx/include/ruiwo_actuator.h"

class MotorevoControllerNode
{
public:
  MotorevoControllerNode() = default;

  ~MotorevoControllerNode()
  {
      close();
  }

  void close() {
      // Stop the state publishing thread
      running_ = false;
      if (state_thread_.joinable()) {
        state_thread_.join();
      }

      if(ruiwo_actuator_) {
        ruiwo_actuator_->close();
        delete ruiwo_actuator_;
        ruiwo_actuator_ = nullptr;
      }

      ROS_INFO("MotorevoControllerNode shutting down");
  }

  bool init() {
    bool cali = false;
    if(ros::param::has("cali_arm")) {
      ros::param::get("cali_arm", cali);
    }
    std::string arm_type = "left";
    if(ros::param::has("arm_type")) {
      ros::param::get("arm_type", arm_type);
    }

    // Set joint names based on arm_type
    if (arm_type == "right") {
      joint_names_ = {
        "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"   // Right arm joints
      };
    } else {
      joint_names_ = {
        "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint"   // Left arm joints
      };
    }
    
    // Initialize the RuiWoActuator
    ruiwo_actuator_ = new RuiWoActuator("", cali);
    if(ruiwo_actuator_->initialize() != 0) {
      std::cout << "RuiWoActuator initialization failed" << std::endl;
      return false;
    }

  
    // Initialize ROS node handle
    ros::NodeHandle nh;
    
    // Subscribe to the /ruiwo_motor/command topic
    motor_cmd_sub_ = nh.subscribe("/ruiwo_motor/command", 10,
                                    &MotorevoControllerNode::jointCmdCallback, this);
    motor_state_pub_ = nh.advertise<sensor_msgs::JointState>("/ruiwo_motor/state", 10);

    // Start the state publishing thread
    running_ = true;
    state_thread_ = std::thread(&MotorevoControllerNode::statePublishLoop, this);

    ROS_INFO("MotorevoControllerNode initialized, subscribing to /ruiwo_motor/command");
    return true;
  }

private:
  // 单位是弧度
  void jointCmdCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {    
    // Check if dimensions match
    if (msg->name.size() != joint_names_.size()) {
      ROS_WARN_STREAM("Dimension mismatch: received " << msg->name.size() 
                      << " joints, but joint_names_ has " << joint_names_.size() << " joints");
      return;
    }

    // Check if position array length matches joint names length
    if (msg->position.size() != msg->name.size()) {
      ROS_WARN_STREAM("Position array length (" << msg->position.size() 
                      << ") does not match joint names length (" << msg->name.size() << ")");
      return;
    }

    for (int i = 0; i < msg->position.size(); i++) {
      if (msg->position[i] > M_PI || msg->position[i] < -M_PI) {
        ROS_WARN_STREAM("Invalid joint position: " << msg->position[i] << " for joint " << msg->name[i]);
        return;
      }
    }

    /**************************************************/  
    // 调用 ruiwo_actuator 
    /**************************************************/
    std::vector<uint8_t> indexs;
    std::vector<double> positions, velocity, torque;

    for(int i = 0; i < msg->name.size(); i++) {
      indexs.push_back(static_cast<uint8_t>(i));
      positions.push_back(msg->position[i] * 180.0 / M_PI);  // 单位角度
      velocity.push_back(0.0); // 速度默认0.0
      torque.push_back(0.5);   // 默认0.5
    }   
    ruiwo_actuator_->set_positions(indexs, positions, torque, velocity);
  }

  void statePublishLoop()
  {
    ros::Rate rate(STATE_PUBLISH_RATE_HZ);
    
    while (running_ && ros::ok()) {
        sensor_msgs::JointState state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.name = joint_names_;
        /******************************************************************/
        state_msg.position = std::vector<double>(joint_names_.size(), 0.0);
        auto positions = ruiwo_actuator_->get_positions();
        if(positions.size() != joint_names_.size()) {
          std::cout << "positions.size() != " << positions.size() << std::endl;
          continue;
        }
        for (size_t i = 0; i < positions.size(); ++i) {
            state_msg.position[i] = positions[i]; // 单位弧度
        }
        /******************************************************************/
        motor_state_pub_.publish(state_msg);
        rate.sleep();
    }
  }

  // ROS subscriber and publisher
  ros::Subscriber motor_cmd_sub_;
  ros::Publisher motor_state_pub_;
  
  // Thread for state publishing
  std::thread state_thread_;
  std::atomic<bool> running_ {false};

  // Joint names constants
  std::vector<std::string> joint_names_;

  // Publishing rate
  static constexpr int STATE_PUBLISH_RATE_HZ = 200;

  RuiWoActuator* ruiwo_actuator_ = nullptr;
};

// Main function

MotorevoControllerNode* g_controller = nullptr;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "motor_control_node");

  // Create and initialize MotorevoControllerNode directly in main
  g_controller = new MotorevoControllerNode();
  if (!g_controller->init()) {
    ROS_ERROR("Failed to initialize MotorevoControllerNode");
    return 0;
  }

  // Spin to receive callbacks
  ros::spin();
  g_controller->close();
  std::cout << "MotorevoControllerNode shutting down" << std::endl;

  return 0;
}
