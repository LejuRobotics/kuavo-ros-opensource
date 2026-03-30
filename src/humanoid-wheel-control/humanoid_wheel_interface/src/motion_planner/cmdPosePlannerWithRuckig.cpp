
#include "humanoid_wheel_interface/motion_planner/cmdPosePlannerWithRuckig.h"

namespace ocs2 {
namespace mobile_manipulator {

cmdPosePlannerWithRuckig::cmdPosePlannerWithRuckig(int dofNum, bool isSync) 
{
    isSync_ = isSync;

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
    inputPtr_ = std::make_unique<ruckig::InputParameter<0>>(dofNum_+1);     // 多出自由度用于做时间同步
    ruckigPlannerPtr_ = std::make_unique<ruckig::Ruckig<0>>(dofNum_+1);
    trajectoryPtr_ = std::make_unique<ruckig::Trajectory<0>>(dofNum_+1);
    
    // 用于非同步的预设时间功能，成员初始化
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
    
    for(size_t i = 0; i < dofNum_+1; ++i)
    {
        inputPtr_->enabled[i] = true;  // 启用该自由度
    }
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].enabled = {true, true};  // 启用该自由度
        inputVec_[i].control_interface = ruckig::ControlInterface::Position;
        inputVec_[i].synchronization = ruckig::Synchronization::Time;
    }
    inputPtr_->control_interface = ruckig::ControlInterface::Position;
    
    switch(isSync)
    {
        case true: inputPtr_->synchronization = ruckig::Synchronization::Time; break;
        case false: inputPtr_->synchronization = ruckig::Synchronization::None; break;
    }

    // 指定多余变量的预设参数
    inputPtr_->current_velocity[dofNum_] = 0.0;
    inputPtr_->target_velocity[dofNum_] = 0.0;
    inputPtr_->current_acceleration[dofNum_] = 0.0;
    inputPtr_->target_acceleration[dofNum_] = 0.0;
}

void cmdPosePlannerWithRuckig::setCurrentPose(const Eigen::VectorXd& pose) {
    // 自由度检查
    if (pose.size() != dofNum_) {
        ROS_ERROR_STREAM("Current pose dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << pose.size());
        return;
    }
    current_pose_ = pose;
    current_pose_ = current_pose_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdPosePlannerWithRuckig::setTargetPose(const Eigen::VectorXd& pose) {
    // 自由度检查
    if (pose.size() != dofNum_) {
        ROS_ERROR_STREAM("Target pose dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << pose.size());
        return;
    }
    target_pose_ = pose;
    target_pose_ = target_pose_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdPosePlannerWithRuckig::setCurrentVelocity(const Eigen::VectorXd& velocity) {
    // 自由度检查
    if (velocity.size() != dofNum_) {
        ROS_ERROR_STREAM("Current velocity dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << velocity.size());
        return;
    }
    current_velocity_ = velocity.cwiseMax(minVelocity_).cwiseMin(maxVelocity_);
    current_velocity_ = current_velocity_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdPosePlannerWithRuckig::setCurrentAcceleration(const Eigen::VectorXd& acceleration) {
    // 自由度检查
    if (acceleration.size() != dofNum_) {
        ROS_ERROR_STREAM("Current acceleration dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << acceleration.size());
        return;
    }
    current_acceleration_ = acceleration.cwiseMax(minAcceleration_).cwiseMin(maxAcceleration_);
    current_acceleration_ = current_acceleration_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
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

    minVelocity_ = min_velocity;
    maxVelocity_ = max_velocity;

    std::vector<double> max_vel_vec(dofNum_+1, 1.0);    // 1.0 是为了方便做时间同步
    std::vector<double> min_vel_vec(dofNum_+1, -1.0);

    for (size_t i = 0; i < dofNum_; ++i) {
        max_vel_vec[i] = max_velocity[i];
        min_vel_vec[i] = min_velocity[i];
    }
    
    inputPtr_->max_velocity = max_vel_vec;
    inputPtr_->min_velocity = min_vel_vec;

    // 非同步成员设置
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].max_velocity = {max_vel_vec[i], 1.0};
        inputVec_[i].min_velocity = {min_vel_vec[i], -1.0};
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
    
    minAcceleration_ = max_deceleration;
    maxAcceleration_ = max_acceleration;
    
    std::vector<double> max_acc_vec(dofNum_+1, std::numeric_limits<double>::infinity());
    std::vector<double> min_acc_vec(dofNum_+1, -std::numeric_limits<double>::infinity());

    for (size_t i = 0; i < dofNum_; ++i) {
        max_acc_vec[i] = max_acceleration[i];
        min_acc_vec[i] = max_deceleration[i]; // 注意：这里使用max_deceleration作为min_acceleration
    }

    inputPtr_->max_acceleration = max_acc_vec;
    inputPtr_->min_acceleration = min_acc_vec;

    // 非同步成员设置
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].max_acceleration = {max_acc_vec[i], std::numeric_limits<double>::infinity()};
        inputVec_[i].min_acceleration = {min_acc_vec[i], -std::numeric_limits<double>::infinity()};
    }
}

void cmdPosePlannerWithRuckig::setJerkLimits(const Eigen::VectorXd& max_jerk) {
    // 自由度检查
    if (max_jerk.size() != dofNum_) {
        ROS_ERROR_STREAM("Jerk limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << max_jerk.size());
        return;
    }

    std::vector<double> max_jerk_vec(dofNum_+1, std::numeric_limits<double>::infinity());

    for (size_t i = 0; i < dofNum_; ++i) {
        max_jerk_vec[i] = max_jerk[i];
    }

    inputPtr_->max_jerk = max_jerk_vec;

    // 非同步成员设置
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].max_jerk = {max_jerk_vec[i], std::numeric_limits<double>::infinity()};
    }
}

double cmdPosePlannerWithRuckig::calcTrajectory(double desiredTime) {
    
    isVecMode_ = false;

    if(desiredTime > 0.0 && !isSync_)
    {
        isVecMode_ = true;
        return calcTrajectoryNonSync(desiredTime);
    }

    double maxDuration = 0.0;
    bool allSuccess = true;
    // 为每个自由度计算轨迹
    for (size_t i = 0; i < dofNum_; ++i)
    {
        // 设置当前状态和目标状态
        inputPtr_->current_position[i] = current_pose_[i];
        inputPtr_->current_velocity[i] = current_velocity_[i];
        inputPtr_->current_acceleration[i] = current_acceleration_[i];
        
        inputPtr_->target_position[i] = target_pose_[i];
        inputPtr_->target_velocity[i] = 0.0;  // 默认目标速度为0
        inputPtr_->target_acceleration[i] = 0.0;  // 默认目标加速度为0
    }
    // 设置多余状态的目标，用于时间同步
    inputPtr_->current_position[dofNum_] = 0.0;
    inputPtr_->target_position[dofNum_] = desiredTime * 1.0;

    // 计算轨迹
    ruckig::Result result = ruckigPlannerPtr_->calculate(*inputPtr_, *trajectoryPtr_);

    if (result != ruckig::Result::Finished && result != ruckig::Result::Working) {
        ROS_ERROR_STREAM("Ruckig trajectory calculation failed, with error code: " 
                        << static_cast<int>(result));
        allSuccess = false;
        for (size_t i = 0; i < dofNum_; ++i)
        {
            // 设置当前状态和目标状态
            std::cout << "current_position[" << i << "]:  " << inputPtr_->current_position[i] << std::endl;
            std::cout << "current_velocity[" << i << "]:  " << inputPtr_->current_velocity[i] << std::endl;
            std::cout << "current_acceleration[" << i << "]:  " << inputPtr_->current_acceleration[i] << std::endl;
            std::cout << "target_position[" << i << "]:  " << inputPtr_->target_position[i] << std::endl;
            std::cout << "target_velocity[" << i << "]:  " << inputPtr_->target_velocity[i] << std::endl;
            std::cout << "target_acceleration[" << i << "]:  " << inputPtr_->target_acceleration[i] << std::endl;
        }
    }

    // 更新最大持续时间
    double duration = trajectoryPtr_->get_duration();

    ROS_DEBUG_STREAM("trajectory duration: " << duration << "s");

    if (!allSuccess) {
        ROS_ERROR("Some DOF trajectory calculations failed!");
        return -1.0;
    }

    return duration;
}

double cmdPosePlannerWithRuckig::calcTrajectoryNonSync(double desiredTime) 
{
    double maxDuration = 0.0;
    double slowestIndex = 0;
    bool allSuccess = true;
    // 为每个自由度计算轨迹
    for (size_t i = 0; i < dofNum_; ++i)
    {
        // 设置当前状态和目标状态
        inputVec_[i].current_position = {current_pose_[i], 0.0};
        inputVec_[i].current_velocity = {current_velocity_[i], 0.0};
        inputVec_[i].current_acceleration = {current_acceleration_[i], 0.0};
        
        inputVec_[i].target_position = {target_pose_[i], 0.0};  // 先计算出各轴最短时间
        inputVec_[i].target_velocity = {0.0, 0.0};  // 默认目标速度为0
        inputVec_[i].target_acceleration = {0.0, 0.0};  // 默认目标加速度为0
        
        // 计算轨迹
        ruckig::Result result = ruckigPlannerVec_[i].calculate(inputVec_[i], trajectoryVec_[i]);

        if (result != ruckig::Result::Finished && result != ruckig::Result::Working) {
            ROS_ERROR_STREAM("Ruckig trajectory calculation failed for DOF " << i 
                            << " with error code: " << static_cast<int>(result));
            allSuccess = false;
            for (size_t i = 0; i < dofNum_; ++i)
            {
                // 设置当前状态和目标状态
                std::cout << "current_position[" << i << "]:  " << inputPtr_->current_position[i] << std::endl;
                std::cout << "current_velocity[" << i << "]:  " << inputPtr_->current_velocity[i] << std::endl;
                std::cout << "current_acceleration[" << i << "]:  " << inputPtr_->current_acceleration[i] << std::endl;
                std::cout << "target_position[" << i << "]:  " << inputPtr_->target_position[i] << std::endl;
                std::cout << "target_velocity[" << i << "]:  " << inputPtr_->target_velocity[i] << std::endl;
                std::cout << "target_acceleration[" << i << "]:  " << inputPtr_->target_acceleration[i] << std::endl;
            }
            continue;
        }

        // 更新最大持续时间
        double duration = trajectoryVec_[i].get_duration();
        if (duration > maxDuration) {
            maxDuration = duration;
            slowestIndex = i;
        }
    }
    
    // 为每个自由度计算轨迹
    maxDuration = 0.0;
    for (size_t i = 0; i < dofNum_; ++i)
    {
        // 设置当前状态和目标状态
        inputVec_[i].current_position = {current_pose_[i], 0.0};
        inputVec_[i].current_velocity = {current_velocity_[i], 0.0};
        inputVec_[i].current_acceleration = {current_acceleration_[i], 0.0};
        
        if(i == slowestIndex)
        {
            inputVec_[i].target_position = {target_pose_[i], desiredTime * 1.0};  // 只对时间最长的轴做时间同步
        }
        else
        {
            inputVec_[i].target_position = {target_pose_[i], 0.0};
        }
        inputVec_[i].target_velocity = {0.0, 0.0};  // 默认目标速度为0
        inputVec_[i].target_acceleration = {0.0, 0.0};  // 默认目标加速度为0
        
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
        if(i == slowestIndex)
        {
            maxDuration = duration;
        }

        ROS_DEBUG_STREAM("DOF " << i << " trajectory duration: " << duration << "s");
    }

    if (!allSuccess) {
        ROS_ERROR("Some DOF trajectory calculations failed!");
        return -1.0;
    }

    return maxDuration;
}

void cmdPosePlannerWithRuckig::getTrajectoryAtTime(double time,
                                                   Eigen::VectorXd& position,
                                                   Eigen::VectorXd& velocity,
                                                   Eigen::VectorXd& acceleration) 
{
    if(isVecMode_ == true)  // 非同步预设时间, 对应轨迹获取
    {
        getTrajectoryAtTimeNonSync(time, position, velocity, acceleration);
        return;
    }

    // 检查是否已计算轨迹
    if (trajectoryPtr_->get_duration() < 0) {
        ROS_ERROR("Trajectory not calculated yet! Call calcTrajectory() first.");
        return;
    }

    // 检查输出向量维度
    if (position.size() != dofNum_) position.resize(dofNum_);
    if (velocity.size() != dofNum_) velocity.resize(dofNum_);
    if (acceleration.size() != dofNum_) acceleration.resize(dofNum_);

    // 获取每个自由度在指定时间点的状态
    std::vector<double> pos(dofNum_+1), vel(dofNum_+1), acc(dofNum_+1);

    double duration = trajectoryPtr_->get_duration();
    double queryTime = (time > duration) ? duration : time;

    trajectoryPtr_->at_time(queryTime, pos, vel, acc);

    for (size_t i = 0; i < dofNum_; ++i)
    {
        position[i] = pos[i];
        velocity[i] = vel[i];
        acceleration[i] = acc[i];
    }
}

void cmdPosePlannerWithRuckig::getTrajectoryAtTimeNonSync(double time,
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
        std::array<double, 2> pos, vel, acc;

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