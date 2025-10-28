#include "humanoid_controllers/shm_manager.h"
#include <cstring>
#include <sys/ipc.h>
#include <iostream>
#include <errno.h>
#include <thread>
#include <chrono>

namespace gazebo_shm
{

ShmManager::ShmManager()
    : sensors_shm_ptr_(nullptr)
    , cmd_shm_ptr_(nullptr)
    , sensors_shm_id_(-1)
    , cmd_shm_id_(-1)
    , sensors_sem_(nullptr)
    , cmd_sem_(nullptr)
{
    // 先清理已存在的信号量
    sem_unlink(SENSORS_SEM_NAME);
    sem_unlink(CMD_SEM_NAME);
    // 创建新的信号量
    sensors_sem_ = sem_open(SENSORS_SEM_NAME, O_CREAT, 0666, 1);
    if (sensors_sem_ == SEM_FAILED) {
        std::cerr << "Failed to create/open sensors semaphore: " << strerror(errno) << std::endl;
        exit(1);
    }
    
    cmd_sem_ = sem_open(CMD_SEM_NAME, O_CREAT, 0666, 1);
    if (cmd_sem_ == SEM_FAILED) {
        std::cerr << "Failed to create/open command semaphore: " << strerror(errno) << std::endl;
        exit(1);
    }
}

ShmManager::~ShmManager()
{
    cleanup();
    
    if (sensors_sem_ != nullptr && sensors_sem_ != SEM_FAILED) {
        sem_close(sensors_sem_);
        sem_unlink(SENSORS_SEM_NAME);
    }
    
    if (cmd_sem_ != nullptr && cmd_sem_ != SEM_FAILED) {
        sem_close(cmd_sem_);
        sem_unlink(CMD_SEM_NAME);
    }
}

bool ShmManager::initializeSensorsShm()
{
    void* ptr;
    if (!attachShm(SHM_KEY_SENSORS, &ptr, sizeof(SensorsData))) {
        return false;
    }
    sensors_shm_ptr_ = static_cast<SensorsData*>(ptr);  // 正确的类型转换

    // 初始化共享内存数据
    sem_wait(sensors_sem_);
    sensors_shm_ptr_->timestamp_ms = 0;
    sensors_shm_ptr_->sensor_time = 0.0;
    sensors_shm_ptr_->is_updated = false;  // 初始化更新标志
    sensors_shm_ptr_->num_joints = 0;
    
    // 初始化IMU数据
    std::memset(&sensors_shm_ptr_->imu_data, 0, sizeof(ImuData));
    sensors_shm_ptr_->imu_data.orientation[3] = 1.0;  // w = 1, x = y = z = 0
    
    // 初始化关节数据
    for (int i = 0; i < SensorsData::MAX_JOINTS; ++i) {
        sensors_shm_ptr_->joint_data[i].position = 0.0;
        sensors_shm_ptr_->joint_data[i].velocity = 0.0;
        sensors_shm_ptr_->joint_data[i].effort = 0.0;
    }
    sem_post(sensors_sem_);
    
    return true;
}

bool ShmManager::initializeCommandShm()
{
    void* ptr;
    if (!attachShm(SHM_KEY_CMD, &ptr, sizeof(JointCommand))) {
        return false;
    }
    cmd_shm_ptr_ = static_cast<JointCommand*>(ptr);

    // 初始化共享内存数据
    sem_wait(cmd_sem_);
    cmd_shm_ptr_->timestamp_ms = 0;  // 使用毫秒时间戳
    cmd_shm_ptr_->is_updated = false;  // 初始化更新标志
    cmd_shm_ptr_->num_joints = 0;
    
    // 初始化关节命令数据
    for (int i = 0; i < JointCommand::MAX_JOINTS; ++i) {
        cmd_shm_ptr_->joint_q[i] = 0.0;
        cmd_shm_ptr_->joint_v[i] = 0.0;
        cmd_shm_ptr_->tau[i] = 0.0;
        cmd_shm_ptr_->tau_max[i] = 0.0;
        cmd_shm_ptr_->tau_ratio[i] = 0.0;
        cmd_shm_ptr_->joint_kp[i] = 0.0;
        cmd_shm_ptr_->joint_kd[i] = 0.0;
        cmd_shm_ptr_->control_modes[i] = 0;
    }
    sem_post(cmd_sem_);
    
    return true;
}

void ShmManager::cleanup()
{
    if (sensors_shm_ptr_) {
        detachShm(sensors_shm_ptr_);
        sensors_shm_ptr_ = nullptr;
    }
    if (cmd_shm_ptr_) {
        detachShm(cmd_shm_ptr_);
        cmd_shm_ptr_ = nullptr;
    }
}

bool ShmManager::writeSensorsData(const SensorsData& data)
{
    if (!sensors_shm_ptr_ || sensors_sem_ == SEM_FAILED) {
        return false;
    }
    
    sem_wait(sensors_sem_);  // 获取信号量
    std::memcpy(sensors_shm_ptr_, &data, sizeof(SensorsData));
    sensors_shm_ptr_->is_updated = true;  // 设置更新标志
    sem_post(sensors_sem_);  // 释放信号量
    return true;
}

bool ShmManager::readSensorsData(SensorsData& data)
{
    if (!sensors_shm_ptr_ || sensors_sem_ == SEM_FAILED) {
        return false;
    }
    
    sem_wait(sensors_sem_);  // 获取信号量
    if (!sensors_shm_ptr_->is_updated) {  // 检查是否已更新
        sem_post(sensors_sem_);
        return false;
    }
    std::memcpy(&data, sensors_shm_ptr_, sizeof(SensorsData));
    sem_post(sensors_sem_);  // 释放信号量
    return true;
}

bool ShmManager::writeJointCommand(const JointCommand& cmd)
{
    if (!cmd_shm_ptr_ || cmd_sem_ == SEM_FAILED) {
        return false;
    }
    
    sem_wait(cmd_sem_);  // 获取信号量
    std::memcpy(cmd_shm_ptr_, &cmd, sizeof(JointCommand));
    cmd_shm_ptr_->is_updated = true;  // 设置更新标志
    sem_post(cmd_sem_);  // 释放信号量
    return true;
}

bool ShmManager::readJointCommand(JointCommand& cmd)
{
    if (!cmd_shm_ptr_ || cmd_sem_ == SEM_FAILED) {
        return false;
    }
    
    sem_wait(cmd_sem_);  // 获取信号量
    if (!cmd_shm_ptr_->is_updated) {  // 检查是否已更新
        sem_post(cmd_sem_);
        return false;
    }
    std::memcpy(&cmd, cmd_shm_ptr_, sizeof(JointCommand));
    sem_post(cmd_sem_);  // 释放信号量
    return true;
}

void ShmManager::setCmdTimeIncrement(double increment) 
{
    cmd_time_increment_ = increment;
}

bool ShmManager::readSensorsDataSync(SensorsData& data, double timeout_ms)
{
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(static_cast<int>(timeout_ms));
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        if (readSensorsData(data)) {
            // 检查时间戳是否递增
            if (data.sensor_time > last_sensor_timestamp_) {
                last_sensor_timestamp_ = data.sensor_time;
                return true;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cerr << "Timeout waiting for sensor data with increasing timestamp. Last timestamp: " 
              << last_sensor_timestamp_ << std::endl;
    return false;
}

bool ShmManager::readJointCommandSync(JointCommand& cmd, double timeout_ms)
{
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(static_cast<int>(timeout_ms));
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        if (readJointCommand(cmd)) {
            // 检查时间戳是否递增（现在使用毫秒单位）
            if (cmd.timestamp_ms > last_cmd_timestamp_) {
                last_cmd_timestamp_ = cmd.timestamp_ms;
                return true;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cerr << "Timeout waiting for joint command with increasing timestamp. Last timestamp: " 
              << last_cmd_timestamp_ << std::endl;
    return false;
}

bool ShmManager::writeJointCommandNext(JointCommand& cmd)
{
    // 自动递增时间戳（转换为毫秒）
    cmd.timestamp_ms = last_cmd_timestamp_ + static_cast<int64_t>(cmd_time_increment_ * 1e3);
    
    if (writeJointCommand(cmd)) {
        last_cmd_timestamp_ = cmd.timestamp_ms;
        return true;
    }
    
    std::cerr << "Failed to write joint command with timestamp: " << cmd.timestamp_ms << std::endl;
    return false;
}

void ShmManager::resetTimestamps()
{
    last_sensor_timestamp_ = 0.0;
    last_cmd_timestamp_ = 0.0;
    std::cout << "Reset timestamps to 0" << std::endl;
}

bool ShmManager::attachShm(int key, void** shm_ptr, int size)
{
    int shm_id = shmget(key, size, IPC_CREAT | 0666);
    if (shm_id == -1) {
        std::cerr << "Failed to create shared memory with key " << key << std::endl;
        return false;
    }

    *shm_ptr = shmat(shm_id, nullptr, 0);
    if (*shm_ptr == (void*)-1) {
        std::cerr << "Failed to attach shared memory with key " << key << std::endl;
        return false;
    }

    return true;
}

void ShmManager::detachShm(void* shm_ptr)
{
    if (shmdt(shm_ptr) == -1) {
        std::cerr << "Failed to detach shared memory" << std::endl;
    }
}
} 
