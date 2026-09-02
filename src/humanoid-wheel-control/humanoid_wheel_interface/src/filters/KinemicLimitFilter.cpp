
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

    // 初始化值限制
    hasValueLimits_ = false;
    minValues_ = Eigen::VectorXd::Constant(dofNum_, -std::numeric_limits<double>::infinity());
    maxValues_ = Eigen::VectorXd::Constant(dofNum_, std::numeric_limits<double>::infinity());

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

    // 应用值限制
    Eigen::VectorXd clampedData = data;
    if (hasValueLimits_)
    {
        for (size_t i = 0; i < dofNum_; ++i) {
            clampedData(i) = std::max(minValues_(i), std::min(maxValues_(i), data(i)));
        }
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
        inputVec_[i].target_position = {clampedData(i)};
        
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

void KinemicLimitFilter::setValueLimit(const Eigen::VectorXd& minLimit, 
                                       const Eigen::VectorXd& maxLimit) {
    if (minLimit.size() != dofNum_ || maxLimit.size() != dofNum_) {
        ROS_ERROR_STREAM("Value limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got min: " << minLimit.size() 
                        << ", max: " << maxLimit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (minLimit(i) > maxLimit(i)) {
            ROS_ERROR_STREAM("Invalid value limits for DOF " << i 
                           << ": min (" << minLimit(i) << ") > max (" << maxLimit(i) << ")");
            return;
        }
        minValues_(i) = minLimit(i);
        maxValues_(i) = maxLimit(i);
    }
    hasValueLimits_ = true;
}

void KinemicLimitFilter::setFirstOrderDerivativeLimit(const Eigen::VectorXd& minLimit, 
                                                       const Eigen::VectorXd& maxLimit) 
{
    if (minLimit.size() != dofNum_ || maxLimit.size() != dofNum_) {
        ROS_ERROR_STREAM("First order limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got min: " << minLimit.size() 
                        << ", max: " << maxLimit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (maxLimit(i) > 0) {
            inputVec_[i].max_velocity = {maxLimit(i)};
        } else {
            ROS_WARN_STREAM("Invalid second order maxLimit for DOF " << i << ": " << maxLimit(i));
        }
    }

    for (size_t i = 0; i < dofNum_; ++i) {
        if (minLimit(i) < maxLimit(i)) {
            inputVec_[i].min_velocity = {minLimit(i)};
        } else {
            ROS_WARN_STREAM("Invalid second order minLimit for DOF " << i << ": " << minLimit(i));
        }
    }
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

void KinemicLimitFilter::setSecondOrderDerivativeLimit(const Eigen::VectorXd& minLimit, 
                                                       const Eigen::VectorXd& maxLimit) 
{
    if (minLimit.size() != dofNum_ || maxLimit.size() != dofNum_) {
        ROS_ERROR_STREAM("Second order limit dimension mismatch! Expected: " 
                        << dofNum_ << ", Got min: " << minLimit.size() 
                        << ", max: " << maxLimit.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        if (maxLimit(i) > 0) {
            inputVec_[i].max_acceleration = {maxLimit(i)};
        } else {
            ROS_WARN_STREAM("Invalid second order maxLimit for DOF " << i << ": " << maxLimit(i));
        }
    }

    for (size_t i = 0; i < dofNum_; ++i) {
        if (minLimit(i) < maxLimit(i)) {
            inputVec_[i].min_acceleration = {minLimit(i)};
        } else {
            ROS_WARN_STREAM("Invalid second order minLimit for DOF " << i << ": " << minLimit(i));
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
    if (initialValue.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_ERROR_STREAM("Reset value dimension mismatch! Expected: " 
                        << dofNum_ << ", Got: " << initialValue.size());
        return;
    }

    if (!resetRange(0, initialValue)) {
        ROS_ERROR_STREAM("Failed to reset KinemicLimitFilter");
    }

    // ROS_INFO_STREAM("KinemicLimitFilter reset to specified initial state");
}

bool KinemicLimitFilter::resetRange(std::size_t offset, const Eigen::VectorXd& values) {
    const Eigen::Index valueCountSigned = values.size();
    if (valueCountSigned <= 0) {
        ROS_ERROR_STREAM("Reset range must contain at least one DOF");
        return false;
    }
    if (!values.allFinite()) {
        ROS_ERROR_STREAM("Reset range contains non-finite values");
        return false;
    }

    const std::size_t valueCount = static_cast<std::size_t>(valueCountSigned);
    // Subtraction-based bounds checking avoids overflow in offset + valueCount.
    if (offset >= dofNum_ || valueCount > dofNum_ - offset) {
        ROS_ERROR_STREAM("Reset range out of bounds! DOFs: " << dofNum_
                         << ", offset: " << offset << ", count: " << valueCount);
        return false;
    }

    // Validate all inputs before touching state.  From here on the operation is
    // atomic from the caller's perspective (provided update/reset calls share
    // the external lock documented in the header).
    for (std::size_t localIndex = 0; localIndex < valueCount; ++localIndex) {
        const std::size_t dofIndex = offset + localIndex;
        const Eigen::Index valueIndex = static_cast<Eigen::Index>(localIndex);
        const double value = values(valueIndex);

        prevData_(static_cast<Eigen::Index>(dofIndex)) = value;
        prevDataFirstOrder_(static_cast<Eigen::Index>(dofIndex)) = 0.0;
        prevDataSecondOrder_(static_cast<Eigen::Index>(dofIndex)) = 0.0;

        auto& input = inputVec_[dofIndex];
        input.current_position = {value};
        input.current_velocity = {0.0};
        input.current_acceleration = {0.0};
        input.target_position = {value};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};

        // Ruckig caches the previous InputParameter internally.  Merely
        // changing inputVec_ is not sufficient when it happens to compare
        // equal to that cache, so force a fresh trajectory on the next update.
        ruckigVec_[dofIndex].reset();

        // Clear the public trajectory/output object as well.  This prevents a
        // stale time/section or derivative from being observed before the next
        // update and keeps the reset state self-consistent.
        outputVec_[dofIndex] = ruckig::OutputParameter<1>();
        outputVec_[dofIndex].new_position = {value};
        outputVec_[dofIndex].new_velocity = {0.0};
        outputVec_[dofIndex].new_acceleration = {0.0};
        outputVec_[dofIndex].new_jerk = {0.0};
    }

    return true;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
