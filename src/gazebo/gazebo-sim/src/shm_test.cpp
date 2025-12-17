#include "humanoid_controllers/shm_manager.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "shm_test");
    ros::NodeHandle nh;
    
    // 创建发布器
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);
    
    // 创建共享内存管理器
    gazebo_shm::ShmManager shm_manager;
    
    // 初始化共享内存
    if (!shm_manager.initializeSensorsShm() || !shm_manager.initializeCommandShm()) {
        std::cerr << "Failed to initialize shared memory" << std::endl;
        return 1;
    }
    std::cout << "Shared memory initialized successfully" << std::endl;

    // 读取传感器数据的变量
    gazebo_shm::SensorsData sensors_data;
    
    // 关节命令变量
    gazebo_shm::JointCommand joint_cmd;
    
    // 设置关节数量
    // joint_cmd.num_joints = 28;  // 根据实际关节数量设置
    joint_cmd.num_joints = 20;
    
    // 设置最大力矩限制
    const double MAX_TORQUE = 100.0;  // 最大力矩限制
    for (int i = 0; i < joint_cmd.num_joints; ++i) {
        joint_cmd.tau_max[i] = MAX_TORQUE;
    }
    
    // 时间和频率参数
    const double freq = 0.5;  // 正弦波频率(Hz)
    const double amplitude = 50.0;  // 力矩幅值
    auto start_time = std::chrono::steady_clock::now();
    
    // 准备消息
    sensor_msgs::JointState joint_state_msg;
    sensor_msgs::Imu imu_msg;
    
    // 设置IMU消息的frame_id
    imu_msg.header.frame_id = "base_link";
    
    ros::Rate rate(500);  // 500Hz的循环频率
    
    while (ros::ok()) {
        // 读取当前传感器数据
        if (shm_manager.readSensorsData(sensors_data)) {
            // 使用仿真器时间创建ROS时间
            ros::Time sim_time;
            sim_time.fromSec(sensors_data.sensor_time);
            
            // 更新IMU消息
            imu_msg.header.stamp = sim_time;
            
            // 设置方向四元数
            imu_msg.orientation.x = sensors_data.imu_data.orientation[0];
            imu_msg.orientation.y = sensors_data.imu_data.orientation[1];
            imu_msg.orientation.z = sensors_data.imu_data.orientation[2];
            imu_msg.orientation.w = sensors_data.imu_data.orientation[3];
            
            // 设置角速度
            imu_msg.angular_velocity.x = sensors_data.imu_data.angular_velocity[0];
            imu_msg.angular_velocity.y = sensors_data.imu_data.angular_velocity[1];
            imu_msg.angular_velocity.z = sensors_data.imu_data.angular_velocity[2];
            
            // 设置线性加速度
            imu_msg.linear_acceleration.x = sensors_data.imu_data.linear_acceleration[0];
            imu_msg.linear_acceleration.y = sensors_data.imu_data.linear_acceleration[1];
            imu_msg.linear_acceleration.z = sensors_data.imu_data.linear_acceleration[2];
            
            // 发布IMU数据
            imu_pub.publish(imu_msg);
            
            // 更新关节状态消息
            joint_state_msg.header.stamp = sim_time;  // 使用相同的仿真时间
            joint_state_msg.position.resize(sensors_data.num_joints);
            joint_state_msg.velocity.resize(sensors_data.num_joints);
            joint_state_msg.effort.resize(sensors_data.num_joints);
            
            for (size_t i = 0; i < sensors_data.num_joints; ++i) {
                joint_state_msg.position[i] = sensors_data.joint_data[i].position;
                joint_state_msg.velocity[i] = sensors_data.joint_data[i].velocity;
                joint_state_msg.effort[i] = sensors_data.joint_data[i].effort;
            }
            
            // 发布关节状态
            joint_state_pub.publish(joint_state_msg);
        }
        
        // 计算当前时间
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();
        
        // 生成正弦力矩命令
        for (int i = 0; i < joint_cmd.num_joints; ++i) {
            // 为不同关节设置不同的相位，避免所有关节同时运动
            double phase = i * M_PI / joint_cmd.num_joints;
            joint_cmd.tau[i] = 0*amplitude * std::sin(2.0 * M_PI * freq * elapsed_seconds + phase);
            
            // 设置为力矩控制模式（1表示力矩控制）
            joint_cmd.control_modes[i] = 0;
        }
        
        // 写入关节命令
        shm_manager.writeJointCommand(joint_cmd);
        
        // 处理ROS回调
        ros::spinOnce();
        
        // 按照指定频率休眠
        rate.sleep();
    }
    
    return 0;
} 
