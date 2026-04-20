
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"

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
    Eigen::VectorXd lastQpos_, lastQvel_;
    double dt_ = 0.001;

    // 上下限制
    Eigen::VectorXd qposMax_, qposMin_;
    Eigen::VectorXd qvelMax_, qvelMin_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
