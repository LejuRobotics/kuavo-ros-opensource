#include "gazebo-sim/sensor_data_publisher.h"

SensorDataPublisher::SensorDataPublisher(ros::NodeHandle &nh) : nh_(nh)
{
    // 创建订阅者
    imu_subscriber_ = nh_.subscribe("/imu", 10, &SensorDataPublisher::imuCallback, this);
    joint_state_subscriber_ = nh_.subscribe("/joint_states", 10, &SensorDataPublisher::jointStateCallback, this);

    // 创建发布者
    sensor_data_publisher_ = nh_.advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 10);
}

void SensorDataPublisher::start()
{
    // 使用定时器确保以 500Hz 的频率发布数据
    sensor_time_ = ros::Time(0);
    timer_ = nh_.createTimer(ros::Duration(1.0 / 500.0), &SensorDataPublisher::publishSensorData, this);
    waitForParams();
}

// IMU 回调函数
void SensorDataPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    imu_msg_ = imu_msg; // 更新最新的 IMU 数据
    sensor_time_ += ros::Duration(1.0 / 500.0);
}
void SensorDataPublisher::waitForParams()
{
    std::cout << "[SensorDataPublisher] waiting For Params ..." << std::endl;
    int i = 0;
    while (ros::ok())
    {
        if (ros::param::has("/joint_state_controller/joint_names"))
        {
            std::cout << "[SensorDataPublisher] Gazebo initial state parameter found, proceeding with joint commands..." << std::endl;
            break;
        }
        else
        {
            ROS_WARN("Gazebo waiting for initial state parameter /joint_state_controller/joint_names ...");
            usleep(100000);
        }
        ros::spinOnce();
    }
    if (ros::param::has("/joint_state_controller/joint_names"))
    {
        ros::param::get("/joint_state_controller/joint_names", joint_names_);
        ROS_INFO("Successfully fetched joint names. %lu joints found.", joint_names_.size());
    }
    else
    {
        ROS_ERROR("Parameter '/joint_state_controller/joint_names' not found.");
    }
    active_joint_count_ = joint_names_.size();
}

sensor_msgs::JointState sortJointState(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names) {
    sensor_msgs::JointState sorted_joint_state;

    // 设置 Header
    sorted_joint_state.header = joint_state.header;

    // 预留空间
    sorted_joint_state.name.resize(joint_names.size());
    sorted_joint_state.position.resize(joint_names.size());
    sorted_joint_state.velocity.resize(joint_names.size());
    sorted_joint_state.effort.resize(joint_names.size());

    // 按照 joint_names 的顺序填充关节状态
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];

        // 查找当前关节名称在原始消息中的索引
        auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
        if (it != joint_state.name.end()) {
            size_t index = std::distance(joint_state.name.begin(), it);

            // 填充关节状态
            sorted_joint_state.name[i] = joint_name;
            sorted_joint_state.position[i] = joint_state.position[index];
            sorted_joint_state.velocity[i] = joint_state.velocity[index];
            sorted_joint_state.effort[i] = joint_state.effort[index];
        } else {
            // 如果关节名称未找到，填充默认值
            ROS_WARN("Joint '%s' not found in the incoming joint state message.", joint_name.c_str());
            sorted_joint_state.name[i] = joint_name;
            sorted_joint_state.position[i] = 0.0;
            sorted_joint_state.velocity[i] = 0.0;
            sorted_joint_state.effort[i] = 0.0;
        }
    }

    return sorted_joint_state;
}

// Joint State 回调函数
void SensorDataPublisher::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    joint_state_msg_ = sortJointState(*joint_state_msg, joint_names_);
}

// 定时器回调函数，发布数据
void SensorDataPublisher::publishSensorData(const ros::TimerEvent &event)
{
    if (imu_msg_ && joint_state_msg_.name.size())
    {
        kuavo_msgs::sensorsData sensor_data_msg;

        // 填充 Header 和时间戳
        sensor_data_msg.header.stamp = ros::Time::now();
        sensor_data_msg.sensor_time = sensor_time_;

        // 填充 IMU 数据
        sensor_data_msg.imu_data.quat = imu_msg_->orientation;
        sensor_data_msg.imu_data.gyro = imu_msg_->angular_velocity;
        sensor_data_msg.imu_data.acc = imu_msg_->linear_acceleration;

        // 填充关节数据
        sensor_data_msg.joint_data.joint_q = joint_state_msg_.position;
        sensor_data_msg.joint_data.joint_v = joint_state_msg_.velocity;
        sensor_data_msg.joint_data.joint_vd = joint_state_msg_.velocity; // TODO: 填充适当的加速度
        sensor_data_msg.joint_data.joint_torque = joint_state_msg_.effort;

        // 发布数据
        sensor_data_publisher_.publish(sensor_data_msg);
    }
}
