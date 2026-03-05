
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "ruckig/ruckig.hpp"

namespace ocs2 {
namespace mobile_manipulator {

// 支持自定义自由度的ruckig规划器封装类

class KinemicLimitFilter {
public:
    KinemicLimitFilter(int dofNum, double dt);

    ~KinemicLimitFilter() = default;

    // 更新数值
    Eigen::VectorXd update(const Eigen::VectorXd& data);

    // 获取一阶/二阶导数
    const Eigen::VectorXd& getFirstOrderDerivative() const;
    const Eigen::VectorXd& getSecondOrderDerivative() const;

    // 设置运动学约束
    void setFirstOrderDerivativeLimit(const Eigen::VectorXd& limit);
    void setSecondOrderDerivativeLimit(const Eigen::VectorXd& limit);
    void setThirdOrderDerivativeLimit(const Eigen::VectorXd& limit);

    // 重置滤波器
    void reset(const Eigen::VectorXd& initialValue);

private:
    // 常规成员 
    size_t dofNum_{3};                      // 自由度数量
    Eigen::VectorXd prevData_;              // 上一时刻数据
    Eigen::VectorXd prevDataFirstOrder_;    // 上一时刻数据的一阶导数
    Eigen::VectorXd prevDataSecondOrder_;   // 上一时刻数据的二阶导数

    // ruckig 相关成员
    std::vector<ruckig::InputParameter<1>> inputVec_;
    std::vector<ruckig::OutputParameter<1>> outputVec_;
    std::vector<ruckig::Ruckig<1>> ruckigVec_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
