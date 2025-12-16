
#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"

namespace ocs2 {
namespace mobile_manipulator {

VelocityLimiter::VelocityLimiter(int vel_num)
: vel_num_(vel_num) 
{
    // 检查维度有效性
    if (vel_num_ <= 0) {
        throw std::invalid_argument("速度维度必须为正数");
    }
    
    // 初始化向量（默认加速度限制为0）
    prev_cmd_vel_ = Eigen::VectorXd::Zero(vel_num_);
    max_acceleration_ = Eigen::VectorXd::Zero(vel_num_);
    max_deceleration_ = Eigen::VectorXd::Zero(vel_num_);
    
    prev_cmd_vel_.setZero();
    acc_dt_ = 0.0;
}

// 加速度限制函数
Eigen::VectorXd VelocityLimiter::limitAcceleration(const Eigen::VectorXd& desired_vel) {
    // 检查输入维度
    if (desired_vel.size() != vel_num_) {
        throw std::invalid_argument("期望速度维度与vel_num不匹配");
    }
    
    // 计算时间间隔
    if (acc_dt_ <= 0) {
        return prev_cmd_vel_;
    }
    
    Eigen::VectorXd limited_vel(vel_num_);
    
    for (int i = 0; i < desired_vel.size(); ++i) {
        double desired_acc = (desired_vel[i] - prev_cmd_vel_[i]) / acc_dt_;
        double max_acc = max_acceleration_[i];
        double max_dec = max_deceleration_[i];
        
        // 限制加速度
        if (desired_acc > 0) {
            desired_acc = std::min(desired_acc, max_acceleration_[i]);
        } else {
            desired_acc = std::max(desired_acc, -max_deceleration_[i]);
        }
        
        // 计算限制后的速度
        limited_vel[i] = prev_cmd_vel_[i] + desired_acc * acc_dt_;
    }
    
    prev_cmd_vel_ = limited_vel;
    
    return limited_vel;
}

void VelocityLimiter::setAccelerationLimits(const Eigen::VectorXd& max_acceleration, const Eigen::VectorXd& max_deceleration) {
    // 检查维度一致性
    if (max_acceleration.size() != vel_num_ || max_deceleration.size() != vel_num_) {
        throw std::invalid_argument("加速度限制维度与vel_num不匹配");
    }
    
    // 检查数值有效性
    for (int i = 0; i < vel_num_; ++i) {
        if (max_acceleration[i] < 0 || max_deceleration[i] < 0) {
            throw std::invalid_argument("加速度限制值必须为非负数");
        }
    }
    max_acceleration_ = max_acceleration;
    max_deceleration_ = max_deceleration;
}

void VelocityLimiter::setAccelerationDt(const double dt) {
    if (dt < 0) {
        throw std::invalid_argument("时间间隔必须为非负数");
    }
    acc_dt_ = dt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2