#include <pinocchio/fwd.hpp> // forward declarations must be included first.
// #include "pinocchio/algorithm/gravity.hpp"

#include "humanoid_controllers/armTorqueController.h"
#include <stdexcept>
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <functional>


ArmTorqueController::ArmTorqueController(const std::string& urdf_path,
                                         const Eigen::VectorXd& kp,
                                         const Eigen::VectorXd& kd)
{
    // 加载 URDF 模型
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    // 定义要固定的关节
    std::vector<std::string> fixed_joint_names = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint",
        "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
        "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
        "zhead_1_joint", "zhead_2_joint"
    };
    // TODO: 固定关节修改

    std::cout << "[ArmTorqueController] model.nq: " << model_.nq << std::endl;
    std::cout << "[ArmTorqueController] model.nv: " << model_.nv << std::endl;

    // 初始化期望状态
    q_measured_ = Eigen::VectorXd::Zero(model_.nq);
    v_measured_ = Eigen::VectorXd::Zero(model_.nv);

    const int n_fixed_joint = fixed_joint_names.size();
    int n_arm = model_.nq - n_fixed_joint;
    std::cout << "[ArmTorqueController] n_arm: " << n_arm << std::endl;
    // 校验增益矩阵维度 - 使用硬编码的手臂关节数而不是计算值
    if (kp.rows() != n_arm_joints_  || kd.rows() != n_arm_joints_ ) {
        std::cout << "[ArmTorqueController] 参数维度错误: kp.rows()=" << kp.rows() 
                  << ", kd.rows()=" << kd.rows() 
                  << ", 期望=" << n_arm_joints_ << std::endl;
        throw std::invalid_argument("KP/KD 矩阵维度与手臂关节数不匹配");
    }
    // kp kd
    kp_ = Eigen::VectorXd::Zero(model_.nq);
    kp_.segment(n_leg_joints_, n_arm_joints_) = kp;
    kd_ = Eigen::VectorXd::Zero(model_.nq);
    kd_.segment(n_leg_joints_, n_arm_joints_) = kd;
}

void ArmTorqueController::setMeasuredState(const Eigen::VectorXd& q_measured,
                                            const Eigen::VectorXd& v_measured) {
    assert(q_measured.rows() == n_arm_joints_ && v_measured.rows() == n_arm_joints_);
    q_measured_.segment(n_leg_joints_, n_arm_joints_) = q_measured;
    v_measured_.segment(n_leg_joints_, n_arm_joints_) = v_measured;
}

Eigen::VectorXd ArmTorqueController::computeTorque(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired,
    const Eigen::VectorXd& a_desired) {
    assert(q_desired.rows() == n_arm_joints_ && v_desired.rows() == n_arm_joints_ && a_desired.rows() == n_arm_joints_);
    Eigen::VectorXd q_desired_full = Eigen::VectorXd::Zero(model_.nq);
    q_desired_full.segment(n_leg_joints_, n_arm_joints_) = q_desired;
    Eigen::VectorXd v_desired_full = Eigen::VectorXd::Zero(model_.nv);
    v_desired_full.segment(n_leg_joints_, n_arm_joints_) = v_desired;
    Eigen::VectorXd a_desired_full = Eigen::VectorXd::Zero(model_.nv);
    a_desired_full.segment(n_leg_joints_, n_arm_joints_) = a_desired;
    // 计算完整动力学项
    pinocchio::computeAllTerms(model_, data_, q_measured_, v_measured_);
    const Eigen::MatrixXd& M = pinocchio::crba(model_, data_, q_measured_);  // 惯性矩阵
    const Eigen::VectorXd& Cv = data_.nle;                     // 科氏力 + 离心力
    const Eigen::VectorXd& G = data_.g;                        // 重力项

    // 前馈扭矩：M*a_desired + Cv + G
    // std::cout << "[ArmTorqueController] M*a_desired_full: " << M * a_desired_full.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] Cv: " << Cv.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // std::cout << "[ArmTorqueController] G: " << G.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // Eigen::VectorXd tau_ff = M * a_desired_full + Cv + G;

    // PD 反馈扭矩：KP*(q_desired - q_measured) + KD*(v_desired - v_measured)
    Eigen::VectorXd q_error = q_desired_full - q_measured_;
    Eigen::VectorXd v_error = v_desired_full - v_measured_;
    Eigen::VectorXd tau_fb = kp_.asDiagonal() * q_error + kd_.asDiagonal() * v_error;
    // std::cout << "[ArmTorqueController] kp_: " << kp_.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] kd_: " << kd_.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau_ff: " << tau_ff.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau_fb: " << tau_fb.segment(n_leg_joints_, n_arm_joints_).transpose() << std::endl;
    // Eigen::VectorXd tau = tau_ff + tau_fb;
    Eigen::VectorXd tau = G + tau_fb;
    // std::cout << "[ArmTorqueController] tau: " << tau.transpose() << std::endl;
    // std::cout << "[ArmTorqueController] tau.size(): " << tau.size() << std::endl;

    return tau.segment(n_leg_joints_, n_arm_joints_);
}