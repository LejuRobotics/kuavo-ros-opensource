
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"

namespace ocs2 {
namespace mobile_manipulator {

// 自定义关节限制器
class jointCmdLimiter {
public:
    jointCmdLimiter(int dofNum, PinocchioInterface pinocchioInterface, 
                    std::string taskFile, const ManipulatorModelInfo& info, 
                    double dt);

    ~jointCmdLimiter() = default;

    // 更新数值
    void update(Eigen::VectorXd& qposCmd, Eigen::VectorXd& qvelCmd);

private:
    // 常规成员
    double dt_ = 0.001;

    // 插值类
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  jointPosFilter_ptr_;
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  jointVelFilter_ptr_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
