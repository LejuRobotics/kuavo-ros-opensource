
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"

namespace ocs2 {
namespace mobile_manipulator {

KinemicLimitFilter::KinemicLimitFilter(int dofNum, double dt) 
{
    if(dofNum <= 0)
    {
        ROS_WARN_STREAM("Invalid DOF number: " << dofNum << ", using default 3 DOF");
        dofNum = 3;
    }
    
    dofNum_ = static_cast<size_t>(dofNum);

    // 初始化历史数据
    prevData_ = Eigen::VectorXd::Zero(dofNum_);
    prevDataFirstOrder_ = Eigen::VectorXd::Zero(dofNum_);
    prevDataSecondOrder_ = Eigen::VectorXd::Zero(dofNum_);

    // 初始化 ruckig 相关数据结构
    inputVec_.resize(dofNum_);
    outputVec_.resize(dofNum_);
    ruckigVec_.resize(dofNum_);
    
    // 设置默认约束, 默认无限制
    for(size_t i = 0; i < dofNum_; ++i)
    {
        // 配置控制周期
        ruckigVec_[i].delta_time = dt;

        inputVec_[i].synchronization = ruckig::Synchronization::None;
        inputVec_[i].enabled = {true};  // 启用该自由度
        inputVec_[i].control_interface = ruckig::ControlInterface::Position;

        // 设置无限大则求解失败, 设置一个理论不可达的数值
        inputVec_[i].max_velocity = {999.0};

        // 初始化状态为0
        inputVec_[i].current_position = {0.0};
        inputVec_[i].current_velocity = {0.0};
        inputVec_[i].current_acceleration = {0.0};
    }
}

Eigen::VectorXd KinemicLimitFilter::update(const Eigen::VectorXd& data) 
{
    // 检查输入数据维度
    if (data.size() != dofNum_) 
    {
        ROS_ERROR_STREAM("Input data dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << data.size());
        return prevData_;
    }

    Eigen::VectorXd filteredData(dofNum_);
    Eigen::VectorXd filteredFirstOrder(dofNum_);
    Eigen::VectorXd filteredSecondOrder(dofNum_);

    // 对每个自由度分别进行限制
    for (size_t i = 0; i < dofNum_; ++i) {
        // 设置当前状态为上一次的输出（位置、一阶导、二阶导）
        inputVec_[i].current_position = {prevData_(i)};
        inputVec_[i].current_velocity = {prevDataFirstOrder_(i)};
        inputVec_[i].current_acceleration = {prevDataSecondOrder_(i)};
        
        // 只设置目标位置，不设置目标速度和加速度
        inputVec_[i].target_position = {data(i)};
        
        // 使用ruckig进行轨迹规划
        auto result = ruckigVec_[i].update(inputVec_[i], outputVec_[i]);
        
        if (result == ruckig::Result::Finished || result == ruckig::Result::Working) {
            // 成功规划，获取下一时刻的位置、一阶导、二阶导
            filteredData(i) = outputVec_[i].new_position[0];
            filteredFirstOrder(i) = outputVec_[i].new_velocity[0];
            filteredSecondOrder(i) = outputVec_[i].new_acceleration[0];
        } else {
            // 规划失败，使用原始数据
            ROS_WARN_STREAM_THROTTLE(1.0, "Ruckig planning failed for DOF " << i 
                                    << ", using previous value, errCode: " << result);
            filteredData(i) = prevData_(i);
            filteredFirstOrder(i) = prevDataFirstOrder_(i);
            filteredSecondOrder(i) = prevDataSecondOrder_(i);
        }
    }
    
    // 更新历史数据
    prevData_ = filteredData;
    prevDataFirstOrder_ = filteredFirstOrder;
    prevDataSecondOrder_ = filteredSecondOrder;
    
    return filteredData;
}

const Eigen::VectorXd& KinemicLimitFilter::getFirstOrderDerivative() const {
    return prevDataFirstOrder_;
}

const Eigen::VectorXd& KinemicLimitFilter::getSecondOrderDerivative() const {
    return prevDataSecondOrder_;
}

void KinemicLimitFilter::setFirstOrderDerivativeLimit(const Eigen::VectorXd& limit) {
    if (limit.size() != dofNum_) {
        ROS_ERROR_STREAM("First order limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << limit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (limit(i) > 0) {
            inputVec_[i].max_velocity = {limit(i)};
        } else {
            ROS_WARN_STREAM("Invalid first order limit for DOF " << i << ": " << limit(i));
        }
    }
}

void KinemicLimitFilter::setSecondOrderDerivativeLimit(const Eigen::VectorXd& limit) {
    if (limit.size() != dofNum_) {
        ROS_ERROR_STREAM("Second order limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << limit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (limit(i) > 0) {
            inputVec_[i].max_acceleration = {limit(i)};
        } else {
            ROS_WARN_STREAM("Invalid second order limit for DOF " << i << ": " << limit(i));
        }
    }
}

void KinemicLimitFilter::setThirdOrderDerivativeLimit(const Eigen::VectorXd& limit) {
    if (limit.size() != dofNum_) {
        ROS_ERROR_STREAM("Third order limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << limit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (limit(i) > 0) {
            inputVec_[i].max_jerk = {limit(i)};
        } else {
            ROS_WARN_STREAM("Invalid third order limit for DOF " << i << ": " << limit(i));
        }
    }
}

void KinemicLimitFilter::reset(const Eigen::VectorXd& initialValue) {
    // 检查输入数据维度
    if (initialValue.size() != dofNum_) {
        ROS_ERROR_STREAM("Reset value dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << initialValue.size());
        return;
    }
    
    // 重置历史数据为指定值, 一阶和二阶导清零
    prevData_ = initialValue;
    prevDataFirstOrder_ = Eigen::VectorXd::Zero(dofNum_);
    prevDataSecondOrder_ = Eigen::VectorXd::Zero(dofNum_);
    
    // 重置每个自由度的ruckig状态
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].current_position = {initialValue(i)};
        inputVec_[i].current_velocity = {0.0};
        inputVec_[i].current_acceleration = {0.0};
    }
    
    // ROS_INFO_STREAM("KinemicLimitFilter reset to specified initial state");
}

}  // namespace mobile_manipulator
}  // namespace ocs2