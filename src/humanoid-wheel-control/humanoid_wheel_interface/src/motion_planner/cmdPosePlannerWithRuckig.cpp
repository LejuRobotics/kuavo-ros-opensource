
#include "humanoid_wheel_interface/motion_planner/cmdPosePlannerWithRuckig.h"

namespace ocs2 {
namespace mobile_manipulator {

cmdPosePlannerWithRuckig::cmdPosePlannerWithRuckig(int dofNum) 
{
    if(dofNum <= 0)
    {
        ROS_WARN_STREAM("Invalid DOF number: " << dofNum << ", using default 3 DOF");
        dofNum = 3;
    }
    
    dofNum_ = static_cast<size_t>(dofNum);

    // 初始化向量
    current_pose_ = Eigen::VectorXd::Zero(dofNum_);
    current_velocity_ = Eigen::VectorXd::Zero(dofNum_);
    current_acceleration_ = Eigen::VectorXd::Zero(dofNum_);
    target_pose_ = Eigen::VectorXd::Zero(dofNum_);

    // 初始化 ruckig 输入参数
    inputVec_.resize(dofNum_);
    ruckigPlannerVec_.resize(dofNum_);
    trajectoryVec_.resize(dofNum_);
    
    // 设置默认约束
    Eigen::VectorXd default_vel_limits = Eigen::VectorXd::Constant(dofNum_, 1.2);
    Eigen::VectorXd default_acc_limits = Eigen::VectorXd::Constant(dofNum_, 0.6);
    Eigen::VectorXd default_jerk_limits = Eigen::VectorXd::Constant(dofNum_, 0.3);
    
    setVelocityLimits(default_vel_limits, -default_vel_limits);
    setAccelerationLimits(default_acc_limits, -default_acc_limits);
    setJerkLimits(default_jerk_limits);
    
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].enabled = {true};  // 启用该自由度
        inputVec_[i].control_interface = ruckig::ControlInterface::Position;
    }
}

void cmdPosePlannerWithRuckig::setCurrentPose(const Eigen::VectorXd& pose) {
    // 自由度检查
    if (pose.size() != dofNum_) {
        ROS_ERROR_STREAM("Current pose dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << pose.size());
        return;
    }
    current_pose_ = pose;
}

void cmdPosePlannerWithRuckig::setTargetPose(const Eigen::VectorXd& pose) {
    // 自由度检查
    if (pose.size() != dofNum_) {
        ROS_ERROR_STREAM("Target pose dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << pose.size());
        return;
    }
    target_pose_ = pose;
}

void cmdPosePlannerWithRuckig::setCurrentVelocity(const Eigen::VectorXd& velocity) {
    // 自由度检查
    if (velocity.size() != dofNum_) {
        ROS_ERROR_STREAM("Current velocity dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << velocity.size());
        return;
    }
    current_velocity_ = velocity;
}

void cmdPosePlannerWithRuckig::setCurrentAcceleration(const Eigen::VectorXd& acceleration) {
    // 自由度检查
    if (acceleration.size() != dofNum_) {
        ROS_ERROR_STREAM("Current acceleration dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << acceleration.size());
        return;
    }
    current_acceleration_ = acceleration;
}

void cmdPosePlannerWithRuckig::setVelocityLimits(const Eigen::VectorXd& max_velocity,
                                                const Eigen::VectorXd& min_velocity) {
    // 自由度检查
    if (max_velocity.size() != dofNum_ || min_velocity.size() != dofNum_) {
        ROS_ERROR_STREAM("Velocity limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Max velocity: " << max_velocity.size() 
                         << ", Min velocity: " << min_velocity.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].max_velocity = {max_velocity[i]};
        inputVec_[i].min_velocity = {min_velocity[i]};
    }
}

void cmdPosePlannerWithRuckig::setAccelerationLimits(const Eigen::VectorXd& max_acceleration, 
                                                    const Eigen::VectorXd& max_deceleration) {
    // 自由度检查
    if (max_acceleration.size() != dofNum_ || max_deceleration.size() != dofNum_) {
        ROS_ERROR_STREAM("Acceleration limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Max acceleration: " << max_acceleration.size() 
                         << ", Max deceleration: " << max_deceleration.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].max_acceleration = {max_acceleration[i]};
        inputVec_[i].min_acceleration = {max_deceleration[i]};  // 注意：这里使用max_deceleration作为min_acceleration
    }
}

void cmdPosePlannerWithRuckig::setJerkLimits(const Eigen::VectorXd& max_jerk) {
    // 自由度检查
    if (max_jerk.size() != dofNum_) {
        ROS_ERROR_STREAM("Jerk limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << max_jerk.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].max_jerk = {max_jerk[i]};
    }
}

double cmdPosePlannerWithRuckig::calcTrajectory() {
    
    double maxDuration = 0.0;
    bool allSuccess = true;
    // 为每个自由度计算轨迹
    for (size_t i = 0; i < dofNum_; ++i)
    {
        // 设置当前状态和目标状态
        inputVec_[i].current_position = {current_pose_[i]};
        inputVec_[i].current_velocity = {current_velocity_[i]};
        inputVec_[i].current_acceleration = {current_acceleration_[i]};
        
        inputVec_[i].target_position = {target_pose_[i]};
        inputVec_[i].target_velocity = {0.0};  // 默认目标速度为0
        inputVec_[i].target_acceleration = {0.0};  // 默认目标加速度为0

        // 计算轨迹
        ruckig::Result result = ruckigPlannerVec_[i].calculate(inputVec_[i], trajectoryVec_[i]);

        if (result != ruckig::Result::Finished && result != ruckig::Result::Working) {
            ROS_ERROR_STREAM("Ruckig trajectory calculation failed for DOF " << i 
                            << " with error code: " << static_cast<int>(result));
            allSuccess = false;
            continue;
        }

        // 更新最大持续时间
        double duration = trajectoryVec_[i].get_duration();
        if (duration > maxDuration) {
            maxDuration = duration;
        }

        ROS_DEBUG_STREAM("DOF " << i << " trajectory duration: " << duration << "s");
    }

    if (!allSuccess) {
        ROS_ERROR("Some DOF trajectory calculations failed!");
        return -1.0;
    }

    // ROS_INFO_STREAM("Trajectory calculation successful. Max duration: " << maxDuration << "s");
    maxDuration_ = maxDuration;  // 存储最大持续时间

    return maxDuration;
}

void cmdPosePlannerWithRuckig::getTrajectoryAtTime(double time,
                                                   Eigen::VectorXd& position,
                                                   Eigen::VectorXd& velocity,
                                                   Eigen::VectorXd& acceleration) 
{
    // 检查是否已计算轨迹
    if (trajectoryVec_.empty() || trajectoryVec_[0].get_duration() < 0) {
        ROS_ERROR("Trajectory not calculated yet! Call calcTrajectory() first.");
        return;
    }

    // 检查输出向量维度
    if (position.size() != dofNum_) position.resize(dofNum_);
    if (velocity.size() != dofNum_) velocity.resize(dofNum_);
    if (acceleration.size() != dofNum_) acceleration.resize(dofNum_);

    // 获取每个自由度在指定时间点的状态
    for (size_t i = 0; i < dofNum_; ++i)
    {
        std::array<double, 1> pos, vel, acc;

        // 对于已经完成轨迹的自由度，使用最终状态
        double dofDuration = trajectoryVec_[i].get_duration();
        double queryTime = (time > dofDuration) ? dofDuration : time;

        trajectoryVec_[i].at_time(queryTime, pos, vel, acc);

        position[i] = pos[0];
        velocity[i] = vel[0];
        acceleration[i] = acc[0];
    }
}

}  // namespace mobile_manipulator
}  // namespace ocs2