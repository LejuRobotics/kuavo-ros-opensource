#pragma once
// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>

class ArmTorqueController {
public:
    ArmTorqueController(){}
    // 构造函数：加载 URDF 并初始化模型
    ArmTorqueController(const std::string& urdf_path, 
                        const Eigen::VectorXd& kp, 
                        const Eigen::VectorXd& kd);

    // 设置目标位置和速度
    void setMeasuredState(const Eigen::VectorXd& q_measured, 
                          const Eigen::VectorXd& v_measured);

    // 计算控制扭矩（完整动力学补偿 + PD 反馈）
    Eigen::VectorXd computeTorque(const Eigen::VectorXd& q_desired, 
                                  const Eigen::VectorXd& v_desired,
                                  const Eigen::VectorXd& a_desired);

private:
    pinocchio::Model model_;        // 机器人模型
    pinocchio::Data data_;          // 模型数据
    Eigen::VectorXd kp_;            // 比例增益
    Eigen::VectorXd kd_;            // 微分增益
    Eigen::VectorXd q_measured_;    // 测量关节位置
    Eigen::VectorXd v_measured_;    // 测量关节速度
    const int n_leg_joints_{12};    // 腿关节数量
    const int n_arm_joints_{14};    // 臂关节数量
};
