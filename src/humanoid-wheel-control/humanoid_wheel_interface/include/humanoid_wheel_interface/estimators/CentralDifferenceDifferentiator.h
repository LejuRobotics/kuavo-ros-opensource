
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <deque>
#include <Eigen/Core>
#include "ruckig/ruckig.hpp"

namespace ocs2 {
namespace mobile_manipulator {

class CentralDifferenceDifferentiator {
private:
    std::deque<Eigen::VectorXd> history_;
    std::deque<double> time_history_;
    size_t max_history_;
    
public:
    CentralDifferenceDifferentiator(size_t max_history = 5) 
        : max_history_(max_history) {}
    
    void differentiate(const Eigen::VectorXd& x, double current_time,
                      Eigen::VectorXd& x_prime, Eigen::VectorXd& x_double_prime) {
        // 保存历史数据
        history_.push_back(x);
        time_history_.push_back(current_time);
        
        if (history_.size() > max_history_) {
            history_.pop_front();
            time_history_.pop_front();
        }
        
        if (history_.size() < 3) {
            // 数据不足，返回零
            x_prime = Eigen::VectorXd::Zero(x.size());
            x_double_prime = Eigen::VectorXd::Zero(x.size());
            return;
        }
        
        // 使用三点中心差分法
        size_t n = history_.size();
        double dt1 = time_history_[n-1] - time_history_[n-2];
        double dt2 = time_history_[n-2] - time_history_[n-3];
        
        if (dt1 <= 0 || dt2 <= 0) {
            x_prime = Eigen::VectorXd::Zero(x.size());
            x_double_prime = Eigen::VectorXd::Zero(x.size());
            return;
        }
        
        // 一阶导数（中心差分）：f'(t) ≈ (f(t+dt) - f(t-dt)) / (2dt)
        x_prime = (history_[n-1] - history_[n-3]) / (dt1 + dt2);
        
        // 二阶导数：f''(t) ≈ (f(t+dt) - 2f(t) + f(t-dt)) / dt²
        x_double_prime = (history_[n-1] - 2.0 * history_[n-2] + history_[n-3]) 
                        / (dt1 * dt2);
    }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
