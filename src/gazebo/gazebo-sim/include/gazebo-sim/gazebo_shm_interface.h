#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "humanoid_controllers/shm_manager.h"
#include <ros/ros.h>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace gazebo
{

class GazeboShmInterface : public ModelPlugin
{
public:
    GazeboShmInterface();
    ~GazeboShmInterface();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    
    // 退出相关函数
    void cleanupAndExit();

private:
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    bool simStartCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    void OnUpdate(const common::UpdateInfo& _info);
    bool ParseImu(const sdf::ElementPtr& _sdf);
    bool ParseJoints(const sdf::ElementPtr& _sdf);
    bool ParseContacts(const sdf::ElementPtr& _sdf);
    
    // 初始化相关函数
    void waitForParams();
    void setInitialState();
    void setModelConfiguration(const std::vector<double>& positions);
    void setModelState(const std::vector<double>& pose);
    
    void updateSensorsData(const common::UpdateInfo& _info);
    // 轮子控制相关函数
    void updateWheelControl();
    double velocityPidControl(physics::JointPtr joint, double target_vel);

    // 里程计发布函数
    void publishOdometry();
    
    // 共享内存数据发布函数
    void publishSensorsData();
    void publishJointCmd(const gazebo_shm::JointCommand& cmd);
    
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection_;
    
    // IMU相关
    physics::LinkPtr imu_link_;
    std::string imu_frame_id_;
    
    // 关节相关
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    
    // 接触传感器相关
    std::vector<physics::LinkPtr> contact_links_;
    std::vector<std::string> contact_link_names_;
    physics::ContactManager* contact_manager_;
    
    // 共享内存管理
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    
    // 命令读取线程
    std::unique_ptr<std::thread> cmd_thread_;
    std::atomic<bool> running_;
    
    // 数据缓存
    gazebo_shm::SensorsData sensors_data_;
    gazebo_shm::JointCommand joint_cmd_;
    std::mutex cmd_mutex_;
    
    // 新增初始化相关变量
    std::vector<double> robot_init_state_param_;
    bool params_loaded_ = false;
    bool sim_start_ = false;
    
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Subscriber stop_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::ServiceServer sim_start_srv_;
    
    // 里程计发布相关
    ros::Publisher odom_pub_;
    ros::Time last_odom_time_;
    ignition::math::Pose3d last_pose_;
    ignition::math::Vector3d last_linear_vel_;
    ignition::math::Vector3d last_angular_vel_;
    
    // 传感器数据发布相关（从共享内存）
    ros::Publisher sensors_data_pub_;
    
    // 关节命令发布相关（从共享内存）
    ros::Publisher joint_cmd_pub_;
    
    // 轮子控制相关
    ignition::math::Vector3d cmd_vel_chassis_;  // [vx, vy, vyaw]
    std::mutex cmd_vel_mutex_;
    std::vector<physics::JointPtr> wheel_yaw_joints_;    // 4个yaw关节
    std::vector<physics::JointPtr> wheel_pitch_joints_;  // 4个pitch关节

    // 机器人版本
    int robotVersion_ = 60;
};

} 
