#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace gazebo_shm
{

struct ImuData {
    double orientation[4];  // quaternion
    double angular_velocity[3];
    double linear_acceleration[3];
};

struct JointData {
    double position;
    double velocity;
    double effort;
};

struct EndEffectorData {
    bool contact;
};

struct SensorsData {
    uint64_t timestamp_ms = 0;  // milliseconds since epoch
    double sensor_time = 0.0;
    bool is_updated = false; 
    ImuData imu_data;
    static const int MAX_JOINTS = 32;
    JointData joint_data[MAX_JOINTS];
    int num_joints;
    EndEffectorData end_effector_data;
};

struct JointCommand {
    uint64_t timestamp_ms;
    bool is_updated = false; 
    static const int MAX_JOINTS = 32;
    double joint_q[MAX_JOINTS];
    double joint_v[MAX_JOINTS];
    double tau[MAX_JOINTS];
    double tau_max[MAX_JOINTS];
    double tau_ratio[MAX_JOINTS];
    double joint_kp[MAX_JOINTS];
    double joint_kd[MAX_JOINTS];
    int32_t control_modes[MAX_JOINTS];
    int num_joints;
};

// 共享内存键值
const int SHM_KEY_SENSORS = 121211;
const int SHM_KEY_CMD = 232323;

} 
